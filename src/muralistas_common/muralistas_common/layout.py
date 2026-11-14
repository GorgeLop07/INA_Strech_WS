"""Geometría del mural en su propio marco de referencia.

Marco "board" (el papel visto de frente):
  - origen en la esquina inferior izquierda del papel,
  - x hacia la derecha a lo largo de la pared (u),
  - y hacia arriba (v),
  - z saliendo de la pared, hacia el robot.

El papel tiene un margen donde van los marcadores ArUco (abajo y arriba).
El área de dibujo es lo que queda dentro del margen, dividida en una cuadrícula.
"""

from dataclasses import dataclass


@dataclass
class MuralLayout:
    width: float = 3.0            # largo del papel (m)
    height: float = 1.0           # alto del papel (m)
    margin: float = 0.12          # franja del borde para los marcadores (m)
    marker_size: float = 0.08     # lado del cuadro negro del ArUco (m)
    marker_spacing: float = 0.5   # distancia entre marcadores a lo largo del papel (m)
    top_id_offset: int = 10       # los marcadores de arriba empiezan en este ID
    rows: int = 3
    cols: int = 4
    shape_fill: float = 0.8       # qué fracción de la casilla ocupa una figura

    @classmethod
    def from_dict(cls, d):
        fields = cls.__dataclass_fields__
        return cls(**{k: v for k, v in d.items() if k in fields})

    # --- Marcadores -------------------------------------------------------
    def marker_positions(self):
        """Centro de cada marcador en coordenadas del mural: {id: (u, v)}."""
        half = self.margin / 2.0
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

    # --- Área de dibujo y cuadrícula -------------------------------------
    def drawable_rect(self):
        """(u_min, v_min, u_max, v_max) del área donde se puede pintar."""
        return (self.margin, self.margin, self.width - self.margin, self.height - self.margin)

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
