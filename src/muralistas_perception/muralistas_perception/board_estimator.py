"""Pose del mural con ArUco + profundidad.

Idea: cada marcador ArUco del margen tiene una posición conocida en el mural. Sus 4 esquinas
detectadas en la imagen se llevan a 3D con la profundidad. Con esas parejas (esquina en el mural,
esquina en 3D) se ajusta la transformación rígida mural -> odom. Basta un solo marcador
(4 esquinas); con más marcadores el ajuste mejora.

Como la cámara de la cabeza solo ve una franja del mural, las observaciones se acumulan en el
marco odom (fijo): conforme la cabeza o la base se mueven, se suman marcadores nuevos.

Además se ajusta un plano a la nube de profundidad alrededor de los marcadores y se reporta
cuánto difiere su normal de la del ajuste por ArUco (sirve como chequeo de consistencia).
"""

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


class BoardEstimator:
    def __init__(self, layout, history=30, max_depth=3.0):
        self.layout = layout
        self.markers = layout.marker_positions()
        self.obs = defaultdict(lambda: deque(maxlen=history))   # (id, esquina) -> puntos en odom
        self.max_depth = max_depth
        self.T_odom_board = None
        self.rms = None
        self.plane_angle_deg = None

    def add_frame(self, detections, depth_m, K, T_odom_cam):
        """Agrega las esquinas de un cuadro. Devuelve cuántas esquinas válidas se usaron."""
        used = 0
        for marker_id, corners in detections.items():
            if marker_id not in self.markers:
                continue
            for k, (px, py) in enumerate(corners):
                z = sample_depth(depth_m, px, py)
                if z is None or z > self.max_depth:
                    continue
                p_cam = pixel_to_optical(px, py, z, K)
                self.obs[(marker_id, k)].append(transform_points(T_odom_cam, p_cam)[0])
                used += 1
        return used

    def fit(self):
        """Ajusta mural -> odom con todas las esquinas acumuladas."""
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
        self.T_odom_board = make_transform(R, t)
        self.rms = rms
        return self.T_odom_board

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
        self.T_odom_board = None
        self.rms = None
