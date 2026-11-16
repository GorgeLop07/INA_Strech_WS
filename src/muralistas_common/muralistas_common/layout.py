"""Geometría del pizarrón en su propio marco de referencia.

Marco "board" (el pizarrón visto de frente):
  - origen en la esquina inferior izquierda del pizarrón,
  - x hacia la derecha a lo largo de la pared (u),
  - y hacia arriba (v),
  - z saliendo de la pared, hacia el robot.

Los ArUco van en las 4 esquinas (IDs 0 a 3, en sentido antihorario desde abajo a la izquierda),
o en franjas abajo y arriba (el papel largo de la primera simulación). Se dibuja solo dentro de
la zona de trabajo, más chica que el pizarrón, para que el robot se mueva menos y las casillas
del barrido queden juntas. La cuadrícula de casillas cubre la zona de trabajo.
"""

from dataclasses import dataclass


@dataclass
class MuralLayout:
    width: float = 1.5            # ancho del pizarrón (m)
    height: float = 1.0           # alto del pizarrón (m)
    margin: float = 0.14          # franja del borde para los marcadores (m)
    marker_size: float = 0.10     # lado del cuadro negro del ArUco (m)
    markers: str = 'esquinas'     # esquinas (IDs 0..3) | franjas (abajo y arriba, cada marker_spacing)
    marker_spacing: float = 0.5   # solo franjas: distancia entre marcadores (m)
    top_id_offset: int = 10       # solo franjas: los marcadores de arriba empiezan en este ID
    work_width: float = 0.60      # zona de trabajo (m)
    work_height: float = 0.45
    work_center_u: float = -1.0   # centro de la zona en el pizarrón; -1 = centrada
    work_center_v: float = -1.0
    rows: int = 3                 # cuadrícula de casillas dentro de la zona de trabajo
    cols: int = 4
    shape_fill: float = 0.8       # qué fracción de la casilla ocupa una figura
    template_size: float = 0.30   # lado de las plantillas de rehabilitación (m)

    @classmethod
    def from_dict(cls, d):
        fields = cls.__dataclass_fields__
        return cls(**{k: v for k, v in d.items() if k in fields})

    # --- Marcadores -------------------------------------------------------
    def marker_positions(self):
        """Centro de cada marcador en coordenadas del pizarrón: {id: (u, v)}."""
        half = self.margin / 2.0
        if self.markers == 'esquinas':
            return {0: (half, half), 1: (self.width - half, half),
                    2: (self.width - half, self.height - half), 3: (half, self.height - half)}
        us = [half]
        u = self.marker_spacing
        while u < self.width - half - 1e-6:
            us.append(u)
            u += self.marker_spacing
        us.append(self.width - half)
        positions = {}
        for i, u in enumerate(us):
            positions[i] = (u, half)
            positions[self.top_id_offset + i] = (u, self.height - half)
        return positions

    # --- Zona de trabajo y cuadrícula ------------------------------------
    def work_rect(self):
        """(u_min, v_min, u_max, v_max) de la zona de trabajo, siempre dentro del margen."""
        cu = self.width / 2.0 if self.work_center_u < 0 else self.work_center_u
        cv = self.height / 2.0 if self.work_center_v < 0 else self.work_center_v
        w = min(self.work_width, self.width - 2 * self.margin)
        h = min(self.work_height, self.height - 2 * self.margin)
        u0 = min(max(cu - w / 2.0, self.margin), self.width - self.margin - w)
        v0 = min(max(cv - h / 2.0, self.margin), self.height - self.margin - h)
        return (u0, v0, u0 + w, v0 + h)

    def drawable_rect(self):
        """Donde se puede pintar: la zona de trabajo."""
        return self.work_rect()

    def work_to_board(self, x, y):
        """Del marco de la zona de trabajo (origen abajo a la izquierda) al del pizarrón."""
        u0, v0, _, _ = self.work_rect()
        return (u0 + x, v0 + y)

    def board_to_work(self, u, v):
        u0, v0, _, _ = self.work_rect()
        return (u - u0, v - v0)

    def cell_rect(self, row, col):
        """Rectángulo de una casilla. La fila 0 es la de arriba, como se ve en pantalla."""
        if not (0 <= row < self.rows and 0 <= col < self.cols):
            raise ValueError(f'Casilla fuera de la cuadrícula: fila {row}, columna {col}')
        u0, v0, u1, v1 = self.drawable_rect()
        cw = (u1 - u0) / self.cols
        ch = (v1 - v0) / self.rows
        left = u0 + col * cw
        top = v1 - row * ch
        return (left, top - ch, left + cw, top)

    def cell_center(self, row, col):
        a, b, c, d = self.cell_rect(row, col)
        return ((a + c) / 2.0, (b + d) / 2.0)

    def shape_box(self, row, col):
        """Cuadrado centrado en la casilla donde se dibuja una figura (u0, v0, lado)."""
        a, b, c, d = self.cell_rect(row, col)
        side = min(c - a, d - b) * self.shape_fill
        cu, cv = (a + c) / 2.0, (b + d) / 2.0
        return (cu - side / 2.0, cv - side / 2.0, side)

    def clamp_to_drawable(self, u, v):
        u0, v0, u1, v1 = self.drawable_rect()
        return (min(max(u, u0), u1), min(max(v, v0), v1))

    def distance_to_edge(self, u, v, du, dv):
        """Cuánto se puede avanzar desde (u, v) en la dirección unitaria (du, dv) sin salir del área."""
        u0, v0, u1, v1 = self.drawable_rect()
        limits = []
        if du > 1e-9:
            limits.append((u1 - u) / du)
        elif du < -1e-9:
            limits.append((u0 - u) / du)
        if dv > 1e-9:
            limits.append((v1 - v) / dv)
        elif dv < -1e-9:
            limits.append((v0 - v) / dv)
        return max(0.0, min(limits)) if limits else 0.0
