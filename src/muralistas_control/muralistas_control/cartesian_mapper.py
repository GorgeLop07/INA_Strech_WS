"""Del mural a las articulaciones del Stretch (sin ROS, fácil de probar).

El Stretch es casi cartesiano. Con la muñeca fija, mover la punta del pincel es:
  - base:     avanzar/retroceder   -> x de base_link
  - elevador: subir/bajar          -> z de base_link
  - brazo:    extender/retraer     -> -y de base_link (el brazo sale hacia la derecha del robot)

Así que un desplazamiento Δ de la punta (en base_link) se reparte directo:
  base += Δ·x,   elevador += Δ·z,   brazo += Δ·(−y)
Si la pared no está perfectamente paralela a la base, el brazo compensa solo, porque Δ se
calcula con la pose real del mural.
"""

import math
from dataclasses import dataclass

import numpy as np


@dataclass
class Limits:
    lift_min: float = 0.05
    lift_max: float = 1.08
    ext_min: float = 0.0
    ext_max: float = 0.50
    base_max: float = 1.6       # cuánto puede desplazarse la base en un solo trazo (m)


class CartesianMapper:
    def __init__(self, T_base_board, tip_base, lift0, ext0, limits=Limits()):
        """T_base_board: pose del mural en base_link al inicio del trazo.
        tip_base: punta actual del pincel en base_link. lift0/ext0: articulaciones actuales."""
        self.T = T_base_board
        self.tip0 = np.asarray(tip_base, dtype=float)
        self.lift0 = lift0
        self.ext0 = ext0
        self.limits = limits

    def board_to_base(self, u, v, w=0.0):
        """Punto del mural (w = distancia fuera de la pared, + hacia el robot) en base_link."""
        return self.T[:3, :3] @ np.array([u, v, w]) + self.T[:3, 3]

    def base_to_board(self, p):
        return self.T[:3, :3].T @ (np.asarray(p) - self.T[:3, 3])

    def joints_for(self, p_base):
        """(traslación de base relativa, elevador, extensión) para llevar la punta a p_base."""
        d = np.asarray(p_base) - self.tip0
        return (float(d[0]), self.lift0 + float(d[2]), self.ext0 - float(d[1]))

    def check(self, joints):
        s, lift, ext = joints
        L = self.limits
        problems = []
        if not L.lift_min <= lift <= L.lift_max:
            problems.append(f'elevador {lift:.3f} fuera de [{L.lift_min}, {L.lift_max}]')
        if not L.ext_min <= ext <= L.ext_max:
            problems.append(f'brazo {ext:.3f} fuera de [{L.ext_min}, {L.ext_max}]')
        if abs(s) > L.base_max:
            problems.append(f'base {s:.3f} más allá de ±{L.base_max}')
        return problems

    def trajectory(self, board_points, w, speed, start_time=0.0, min_dt=0.05, start_base=None, accel=None):
        """Puntos del mural a la misma distancia w de la pared -> [(t, (s, lift, ext))].

        La velocidad es de la punta. Si start_base se da, el primer tramo arranca desde ahí
        (la punta actual); si no, el primer punto se alcanza en min_dt. Con accel (m/s²) la
        velocidad sube y baja en rampa (trapecio) al inicio y al final, en vez de arrancar y
        frenar de golpe: el tirón de la base al arrancar es lo que hace vibrar el trazo.
        """
        ps = [self.board_to_base(u, v, w) for u, v in board_points]
        prev0 = np.asarray(start_base) if start_base is not None else None
        seg = [float(np.linalg.norm(p - q)) for p, q in zip(ps, [prev0] + ps[:-1])] if prev0 is not None \
            else [0.0] + [float(np.linalg.norm(p - q)) for p, q in zip(ps[1:], ps[:-1])]
        total = sum(seg)
        out = []
        t = start_time
        done = 0.0
        for i, p in enumerate(ps):
            if i == 0 and prev0 is None:
                t += min_dt
            else:
                v = max(speed, 1e-3)
                if accel:
                    mid = done + seg[i] / 2.0
                    v_floor = 0.2 * v                     # para no quedarse en 0 en los extremos
                    v = min(v, math.sqrt(v_floor ** 2 + 2 * accel * mid),
                            math.sqrt(v_floor ** 2 + 2 * accel * max(total - mid, 0.0)))
                t += max(min_dt, seg[i] / v)
            done += seg[i]
            out.append((t, self.joints_for(p)))
        return out


def densify(points, step=0.01):
    """Agrega puntos intermedios cada `step` m para que el seguimiento sea suave."""
    if len(points) < 2:
        return list(points)
    out = [points[0]]
    for a, b in zip(points[:-1], points[1:]):
        a, b = np.asarray(a, float), np.asarray(b, float)
        n = max(1, int(np.ceil(np.linalg.norm(b - a) / step)))
        out += [tuple(a + (b - a) * k / n) for k in range(1, n + 1)]
    return out


def path_length(points):
    pts = np.asarray(points, dtype=float)
    return float(np.sum(np.linalg.norm(np.diff(pts, axis=0), axis=1))) if len(pts) > 1 else 0.0
