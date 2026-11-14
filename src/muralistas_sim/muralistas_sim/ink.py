"""La "tinta": registra dónde la punta del pincel toca el papel.

Cada paso de la física se revisan los contactos entre la punta y el papel. Los puntos
se guardan en coordenadas del mural (m) y se agrupan en trazos continuos. Con eso se
dibuja la tinta en las imágenes (geometrías extra en la escena) y en un lienzo 2D.
"""

import cv2
import mujoco
import numpy as np

from muralistas_common.colors import rgb


class InkTracker:
    def __init__(self, model, T_world_board, layout, min_step=0.002, gap_steps=25):
        self.model = model
        self.layout = layout
        self.T_board_world = np.linalg.inv(T_world_board)
        self.T_world_board = T_world_board
        self.tip_geom = model.geom('brush_tip').id
        self.surface_geom = model.geom('wall').id     # el papel es visual; se pinta donde la punta toca la pared
        self.min_step = min_step
        self.gap_steps = gap_steps       # pasos sin contacto antes de cortar el trazo
        self.strokes = []                # [{'color': str, 'points': [(u, v), ...]}]
        self._open = False
        self._missing = 0
        self.color = 'azul'
        self.version = 0                 # sube cada vez que cambia la tinta

    def set_color(self, color):
        rgb(color)  # valida
        self.color = color
        self._open = False

    def clear(self):
        self.strokes.clear()
        self._open = False
        self.version += 1

    def contact_point(self, data):
        """Punto de contacto punta-pared dentro del papel, en coordenadas del mural; o None."""
        for i in range(data.ncon):
            c = data.contact[i]
            pair = {c.geom1, c.geom2}
            if self.tip_geom in pair and self.surface_geom in pair:
                u, v, _ = (self.T_board_world @ np.append(c.pos, 1.0))[:3]
                if 0.0 <= u <= self.layout.width and 0.0 <= v <= self.layout.height:
                    return u, v
        return None

    def update(self, data):
        """Llamar después de cada mj_step. Devuelve True si se agregó tinta."""
        uv = self.contact_point(data)
        if uv is None:
            self._missing += 1
            if self._missing > self.gap_steps:
                self._open = False
            return False
        self._missing = 0
        u, v = uv
        if not self._open:
            self.strokes.append({'color': self.color, 'points': [(u, v)]})
            self._open = True
            self.version += 1
            return True
        pts = self.strokes[-1]['points']
        if np.hypot(u - pts[-1][0], v - pts[-1][1]) >= self.min_step:
            pts.append((u, v))
            self.version += 1
            return True
        return False

    # --- Dibujo ---------------------------------------------------------------
    def world_segments(self, offset=0.003):
        """Segmentos (p0, p1, rgb) en el mundo, un poco despegados del papel para que se vean.
        Se recalculan solo cuando cambia la tinta (se dibujan muchas veces por segundo)."""
        if getattr(self, '_seg_cache_version', None) == self.version:
            return self._seg_cache
        segs = []
        R, t = self.T_world_board[:3, :3], self.T_world_board[:3, 3]
        for s in self.strokes:
            pts = s['points']
            color = rgb(s['color'])
            if len(pts) == 1:
                pts = [pts[0], (pts[0][0] + 1e-4, pts[0][1])]
            uv = np.array(pts, dtype=float)
            world = np.column_stack([uv, np.full(len(uv), offset)]) @ R.T + t
            for a, b in zip(world[:-1], world[1:]):
                segs.append((a, b, color))
        self._seg_cache, self._seg_cache_version = segs, self.version
        return segs

    def add_to_scene(self, scene, width=0.006):
        """Agrega la tinta como cápsulas a un MjvScene (después de update_scene)."""
        for p0, p1, color in self.world_segments():
            if scene.ngeom >= scene.maxgeom:
                return
            g = scene.geoms[scene.ngeom]
            mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_CAPSULE, np.zeros(3), np.zeros(3),
                                np.eye(3).flatten(), np.array([*color, 1.0], dtype=np.float32))
            mujoco.mjv_connector(g, mujoco.mjtGeom.mjGEOM_CAPSULE, width / 2.0, p0, p1)
            g.rgba[:] = [*color, 1.0]
            scene.ngeom += 1

    def canvas(self, px_per_m=400, line_px=4):
        """Lienzo RGB del mural completo con la tinta (fila 0 = parte de arriba del papel)."""
        w = int(self.layout.width * px_per_m)
        h = int(self.layout.height * px_per_m)
        img = np.full((h, w, 3), 245, dtype=np.uint8)
        u0, v0, u1, v1 = self.layout.drawable_rect()
        cv2.rectangle(img, (int(u0 * px_per_m), int(h - v1 * px_per_m)),
                      (int(u1 * px_per_m), int(h - v0 * px_per_m)), (215, 215, 215), 1)
        for s in self.strokes:
            color = tuple(int(255 * c) for c in rgb(s['color']))
            pts = np.array([(u * px_per_m, h - v * px_per_m) for u, v in s['points']], dtype=np.int32)
            if len(pts) == 1:
                cv2.circle(img, tuple(pts[0]), line_px, color, -1)
            else:
                cv2.polylines(img, [pts], False, color, line_px, cv2.LINE_AA)
        return img
