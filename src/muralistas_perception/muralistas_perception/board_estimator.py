"""Pose del mural con ArUco + profundidad.

Idea: cada marcador ArUco del margen tiene una posición conocida en el mural. Sus 4 esquinas
detectadas en la imagen se llevan a 3D con la profundidad. Con esas parejas (esquina en el mural,
esquina en 3D) se ajusta la transformación rígida mural -> odom. Basta un solo marcador
(4 esquinas); con más marcadores el ajuste mejora.

Como la cámara de la cabeza solo ve una franja del mural, las observaciones se acumulan en el
marco odom (fijo): conforme la cabeza o la base se mueven, se suman marcadores nuevos.

Corrección con vistas recientes: si la base patina o gira al apoyar el marcador, la odometría de
las ruedas no lo ve y el ajuste acumulado tarda decenas de cuadros en alcanzarlo. Por eso, sobre
el ajuste acumulado se aplica una corrección en el plano del piso (x, y, giro) calculada solo con
las esquinas vistas en los últimos `fresh_age` segundos, y solo si son de `fresh_min_markers`
marcadores o más. Sin vistas recientes suficientes se mantiene la última corrección.

Además se ajusta un plano a la nube de profundidad alrededor de los marcadores y se reporta
cuánto difiere su normal de la del ajuste por ArUco (sirve como chequeo de consistencia).
"""

import math

from collections import defaultdict, deque

import cv2
import numpy as np

from muralistas_common.geometry import fit_plane_ransac, fit_rigid, make_transform, transform_points

ARUCO_DICT = cv2.aruco.DICT_4X4_50


def marker_corner_coords(u, v, size):
    """Esquinas en el orden de OpenCV (sup-izq, sup-der, inf-der, inf-izq) vistas de frente."""
    h = size / 2.0
    return np.array([[u - h, v + h, 0.0], [u + h, v + h, 0.0], [u + h, v - h, 0.0], [u - h, v - h, 0.0]])


class ArucoDetector:
    def __init__(self):
        params = cv2.aruco.DetectorParameters()
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.detector = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(ARUCO_DICT), params)

    def detect(self, gray):
        corners, ids, _ = self.detector.detectMarkers(gray)
        if ids is None:
            return {}
        return {int(i): c.reshape(4, 2) for i, c in zip(ids.flatten(), corners)}


def sample_depth(depth_m, px, py, radius=2):
    """Mediana de la profundidad válida alrededor de un píxel (m), o None."""
    h, w = depth_m.shape
    x0, x1 = max(0, int(px) - radius), min(w, int(px) + radius + 1)
    y0, y1 = max(0, int(py) - radius), min(h, int(py) + radius + 1)
    patch = depth_m[y0:y1, x0:x1]
    valid = patch[np.isfinite(patch) & (patch > 0)]
    return float(np.median(valid)) if valid.size else None


def pixel_to_optical(px, py, z, K):
    fx, fy, cx, cy = K
    return np.array([(px - cx) * z / fx, (py - cy) * z / fy, z])


def planar_correction(src, dst):
    """Giro alrededor de z y traslación en x, y que llevan src a dst (solo se usa x, y). 4x4."""
    a, b = np.asarray(src, float)[:, :2], np.asarray(dst, float)[:, :2]
    ca, cb = a.mean(axis=0), b.mean(axis=0)
    a0, b0 = a - ca, b - cb
    yaw = math.atan2(float(np.sum(a0[:, 0] * b0[:, 1] - a0[:, 1] * b0[:, 0])), float(np.sum(a0 * b0)))
    c, s = math.cos(yaw), math.sin(yaw)
    R = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    t = np.zeros(3)
    t[:2] = cb - R[:2, :2] @ ca
    return make_transform(R, t)


class BoardEstimator:
    def __init__(self, layout, history=30, max_depth=3.0, fresh_age=2.0, fresh_min_markers=2):
        self.layout = layout
        self.markers = layout.marker_positions()
        self.obs = defaultdict(lambda: deque(maxlen=history))   # (id, esquina) -> puntos en odom
        self.fresh = defaultdict(lambda: deque(maxlen=10))      # (id, esquina) -> (tiempo, punto)
        self.max_depth = max_depth
        self.fresh_age = fresh_age                  # s; <= 0 apaga la corrección
        self.fresh_min_markers = fresh_min_markers
        self.T_odom_board = None
        self.T_full = None                          # ajuste con todo lo acumulado
        self.correction = np.eye(4)                 # odom -> odom corregido (x, y, giro)
        self.fresh_markers = 0                      # marcadores en la última corrección
        self.fresh_time = None                      # tiempo de la última corrección
        self.rms = None
        self.plane_angle_deg = None
        self._t = 0.0

    def add_frame(self, detections, depth_m, K, T_odom_cam, t=None):
        """Agrega las esquinas de un cuadro (t: tiempo de la imagen, s). Devuelve cuántas se usaron."""
        self._t = t if t is not None else self._t + 0.1
        used = 0
        for marker_id, corners in detections.items():
            if marker_id not in self.markers:
                continue
            for k, (px, py) in enumerate(corners):
                z = sample_depth(depth_m, px, py)
                if z is None or z > self.max_depth:
                    continue
                p_cam = pixel_to_optical(px, py, z, K)
                self.add_corner(marker_id, k, transform_points(T_odom_cam, p_cam)[0], self._t)
                used += 1
        return used

    def add_corner(self, marker_id, k, p_odom, t):
        # Lo acumulado se guarda sin la deriva actual (inversa de la corrección), para que el ajuste
        # completo no mezcle vistas de antes y después de que patinó la base
        self.obs[(marker_id, k)].append(transform_points(np.linalg.inv(self.correction), p_odom)[0])
        self.fresh[(marker_id, k)].append((t, np.asarray(p_odom, float)))
        self._t = max(self._t, t)

    def fit(self):
        """Ajusta mural -> odom con todo lo acumulado y le aplica la corrección por vistas recientes."""
        src, dst = [], []
        for (marker_id, k), pts in self.obs.items():
            if not pts:
                continue
            u, v = self.markers[marker_id]
            src.append(marker_corner_coords(u, v, self.layout.marker_size)[k])
            dst.append(np.median(np.array(pts), axis=0))
        if len(src) < 3:
            return None
        R, t, rms = fit_rigid(np.array(src), np.array(dst))
        self.T_full = make_transform(R, t)
        self.rms = rms
        self._fresh_correction()
        self.T_odom_board = self.correction @ self.T_full
        return self.T_odom_board

    def _fresh_correction(self):
        if self.fresh_age <= 0:
            return
        src, dst, ids = [], [], set()
        for (marker_id, k), obs in self.fresh.items():
            recent = [p for t, p in obs if self._t - t <= self.fresh_age]
            if not recent:
                continue
            u, v = self.markers[marker_id]
            corner = marker_corner_coords(u, v, self.layout.marker_size)[k]
            src.append(transform_points(self.T_full, corner)[0])
            dst.append(np.median(np.array(recent), axis=0))
            ids.add(marker_id)
        if len(ids) < self.fresh_min_markers:
            return                                  # se queda la última corrección
        self.correction = planar_correction(src, dst)
        self.fresh_markers = len(ids)
        self.fresh_time = self._t

    def correction_summary(self):
        """(mm de traslación, grados de giro) de la corrección actual."""
        C = self.correction
        return float(np.linalg.norm(C[:2, 3]) * 1000), float(np.degrees(math.atan2(C[1, 0], C[0, 0])))

    def check_plane(self, depth_m, K, T_odom_cam, stride=4, max_points=3000):
        """Ajusta un plano a la profundidad del pizarrón y lo compara con la normal del ajuste ArUco.

        Solo usa puntos dentro del rectángulo del pizarrón (la pared alrededor también es plana y
        con el pizarrón de 1.5 m eran decenas de miles de puntos: ~3 s de RANSAC por cuadro) y
        como mucho `max_points`, tomados al azar con semilla fija."""
        T_odom_board = self.T_odom_board
        if T_odom_board is None:
            return None
        from muralistas_common.geometry import deproject
        pts_cam, _ = deproject(depth_m, *K, stride=stride)
        if len(pts_cam) < 50:
            return None
        pts = transform_points(T_odom_cam, pts_cam)
        T_board_odom = np.linalg.inv(T_odom_board)
        local = transform_points(T_board_odom, pts)
        L = self.layout
        near = (np.abs(local[:, 2]) < 0.05) & (local[:, 0] >= 0) & (local[:, 0] <= L.width) \
            & (local[:, 1] >= 0) & (local[:, 1] <= L.height)
        idx = np.nonzero(near)[0]
        if len(idx) < 50:
            return None
        if len(idx) > max_points:
            idx = np.random.default_rng(0).choice(idx, max_points, replace=False)
        normal, _, _ = fit_plane_ransac(pts[idx], threshold=0.008)
        if normal is None:
            return None
        board_normal = T_odom_board[:3, 2]
        cos = abs(float(normal @ board_normal))
        self.plane_angle_deg = float(np.degrees(np.arccos(min(1.0, cos))))
        return self.plane_angle_deg

    def corners_used(self):
        return sum(1 for pts in self.obs.values() if pts)

    def markers_used(self):
        return len({marker_id for (marker_id, _), pts in self.obs.items() if pts})

    def reset(self):
        self.obs.clear()
        self.fresh.clear()
        self.T_odom_board = None
        self.T_full = None
        self.correction = np.eye(4)
        self.fresh_markers = 0
        self.fresh_time = None
        self.rms = None
