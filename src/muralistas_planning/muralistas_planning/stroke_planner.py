"""Convierte lo que eligió el niño en trazos sobre el pizarrón (sin ROS, fácil de probar).

Todo está en el marco del pizarrón (metros): u a lo largo de la pared, v hacia arriba.
"""

import math

from muralistas_common.colors import MARKER_COLOR, PALETTE
from muralistas_common.shapes import SHAPES, shape_in_box
from muralistas_common.templates import resample


class PlanError(ValueError):
    pass


def plan_shape(layout, shape, row, col):
    """Figura en una casilla -> lista de trazos [(u, v), ...]."""
    if shape not in SHAPES:
        raise PlanError(f'Figura desconocida: {shape}')
    try:
        u0, v0, side = layout.shape_box(row, col)
    except ValueError as exc:
        raise PlanError(str(exc)) from exc
    return shape_in_box(shape, u0, v0, side)


def plan_free_segment(layout, tip_uv, direction_deg, max_length=None):
    """Trazo libre: desplazamiento (du, dv) desde la punta actual hasta el borde del área (o max_length)."""
    a = math.radians(direction_deg)
    du, dv = math.cos(a), math.sin(a)
    length = layout.distance_to_edge(tip_uv[0], tip_uv[1], du, dv)
    if max_length is not None and max_length > 0:
        length = min(length, max_length)
    if length < 0.005:
        raise PlanError('Ya está en el borde del área de dibujo en esa dirección')
    return (du * length, dv * length)


def douglas_peucker(points, tol):
    """Simplifica una polilínea quitando puntos a menos de `tol` de la recta entre sus vecinos."""
    if len(points) < 3:
        return list(points)
    a, b = points[0], points[-1]
    dx, dy = b[0] - a[0], b[1] - a[1]
    L = math.hypot(dx, dy)
    best_i, best_d = 0, -1.0
    for i in range(1, len(points) - 1):
        p = points[i]
        d = math.dist(p, a) if L == 0 else abs(dy * (p[0] - a[0]) - dx * (p[1] - a[1])) / L
        if d > best_d:
            best_i, best_d = i, d
    if best_d <= tol:
        return [a, b]
    left = douglas_peucker(points[:best_i + 1], tol)
    return left[:-1] + douglas_peucker(points[best_i:], tol)


def chaikin(points, iterations=2):
    """Redondea esquinas (corte de Chaikin) conservando el primer y el último punto."""
    pts = list(points)
    for _ in range(iterations):
        if len(pts) < 3:
            return pts
        out = [pts[0]]
        for a, b in zip(pts[:-1], pts[1:]):
            out.append((0.75 * a[0] + 0.25 * b[0], 0.75 * a[1] + 0.25 * b[1]))
            out.append((0.25 * a[0] + 0.75 * b[0], 0.25 * a[1] + 0.75 * b[1]))
        out.append(pts[-1])
        pts = out
    return pts


def process_ghost(layout, points, step=0.01, simplify_tol=0.002, smooth_iterations=2):
    """Fantasma crudo (puntos del cursor) -> trazo que el robot puede seguir.

    1. quita puntos repetidos, 2. simplifica (Douglas-Peucker), 3. redondea (Chaikin),
    4. remuestrea a un punto por `step`, 5. recorta a la zona de trabajo.
    """
    pts = []
    for p in points:
        if not pts or math.dist(p, pts[-1]) > 1e-6:
            pts.append((float(p[0]), float(p[1])))
    if not pts:
        raise PlanError('Fantasma vacío')
    if len(pts) > 1:
        pts = douglas_peucker(pts, simplify_tol)
        pts = chaikin(pts, smooth_iterations)
        pts = resample(pts, step)
    return [layout.clamp_to_drawable(u, v) for u, v in pts]


def validate_color(color):
    """El marcador es uno solo: vacío se toma como el azul."""
    color = color or MARKER_COLOR
    if color not in PALETTE:
        raise PlanError(f'Color desconocido: {color}')
    return color
