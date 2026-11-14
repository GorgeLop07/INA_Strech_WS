"""Convierte lo que eligió el niño en trazos sobre el mural (sin ROS, fácil de probar).

Todo está en el marco del mural (metros): u a lo largo de la pared, v hacia arriba.
"""

import math

from muralistas_common.colors import PALETTE
from muralistas_common.shapes import SHAPES, shape_in_box


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
    if max_length is not None:
        length = min(length, max_length)
    if length < 0.005:
        raise PlanError('Ya está en el borde del área de dibujo en esa dirección')
    return (du * length, dv * length)


def validate_color(color):
    if color not in PALETTE:
        raise PlanError(f'Color desconocido: {color}')
