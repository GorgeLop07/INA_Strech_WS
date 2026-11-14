"""Segmenta el pincel en la nube 3D y estima dónde está su punta.

1. En la imagen a color se busca el verde del cuerpo del pincel (color reservado para eso).
2. Esos píxeles se llevan a 3D con la profundidad.
3. Con PCA se obtiene el eje del pincel; el extremo más lejano a la pinza es el final del cuerpo.
4. La punta está a una distancia conocida más allá del final del cuerpo (geometría del pincel).

Se usa para calibrar una sola vez dónde quedó la punta respecto a la pinza (el "offset" de la
herramienta). Durante el dibujo la punta se calcula con la cinemática + ese offset.
"""

import cv2
import numpy as np

from muralistas_common.geometry import principal_axis, transform_points


class BrushSegmenter:
    def __init__(self, hsv_low=(45, 120, 80), hsv_high=(80, 255, 255), tip_beyond_body=0.017,
                 body_radius=0.011, min_points=40, max_distance_from_gripper=0.30):
        self.hsv_low = np.array(hsv_low, dtype=np.uint8)
        self.hsv_high = np.array(hsv_high, dtype=np.uint8)
        self.tip_beyond_body = tip_beyond_body
        # La cámara solo ve la cara del cilindro que tiene enfrente: en promedio esos puntos quedan
        # (pi/4)·r más cerca de la cámara que el eje. Sin esta corrección el error era de ~10 mm.
        self.surface_to_axis = np.pi / 4.0 * body_radius
        self.min_points = min_points
        self.max_distance_from_gripper = max_distance_from_gripper

    def mask(self, rgb):
        hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
        m = cv2.inRange(hsv, self.hsv_low, self.hsv_high)
        return cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))

    def estimate(self, rgb, depth_m, K, T_base_cam, grasp_in_base):
        """Devuelve dict con puntos (base_link), eje, punta estimada; o None si no se ve el pincel."""
        m = self.mask(rgb)
        rows, cols = np.nonzero(m)
        if len(rows) < self.min_points:
            return None
        z = depth_m[rows, cols]
        ok = np.isfinite(z) & (z > 0)
        rows, cols, z = rows[ok], cols[ok], z[ok]
        if len(z) < self.min_points:
            return None
        fx, fy, cx, cy = K
        pts_cam = np.stack([(cols - cx) * z / fx, (rows - cy) * z / fy, z], axis=1)
        pts = transform_points(T_base_cam, pts_cam)
        near = np.linalg.norm(pts - grasp_in_base, axis=1) < self.max_distance_from_gripper
        pts = pts[near]
        if len(pts) < self.min_points:
            return None
        center, axis = principal_axis(pts)
        if axis @ (center - grasp_in_base) < 0:     # el eje apunta de la pinza hacia la punta
            axis = -axis
        view = center - T_base_cam[:3, 3]
        away = view - (view @ axis) * axis
        center = center + away / max(np.linalg.norm(away), 1e-9) * self.surface_to_axis
        proj = (pts - center) @ axis
        body_end = center + axis * np.percentile(proj, 99)
        tip = body_end + axis * self.tip_beyond_body
        return {'points': pts, 'axis': axis, 'center': center, 'body_end': body_end, 'tip': tip,
                'pixels': int(len(rows))}
