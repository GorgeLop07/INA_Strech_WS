"""Poses del barrido de cabeza para ver todos los ArUco del pizarrón.

Se calculan con la geometría del pizarrón (config/mural.yaml) y la posición del robot: frente al
pizarrón, a wall_distance de la pared y centrado a lo ancho. Para cada marcador se calcula a
dónde tendría que mirar la cabeza (pan, tilt) y se juntan en el menor número de poses: primero en
columnas (pan), luego, dentro de cada columna, en filas (tilt). Cada pose deja a sus marcadores a
margin grados o más del borde de la imagen, contando el tamaño del marcador.

Convenciones del Stretch: pan = -pi/2 mira de frente a la pared que está a la derecha del robot,
hacia 0 mira a la izquierda (u chica del pizarrón); tilt = 0 mira al horizonte, negativo hacia abajo.
"""

import math


def _groups(intervals, span):
    """Junta intervalos (centro, medio ancho), ordenados por centro, en grupos que caben en span."""
    groups = []
    for c, a in sorted(intervals):
        if groups and max(c + a, groups[-1][1]) - min(c - a, groups[-1][0]) <= span:
            g = groups[-1]
            g[0], g[1] = min(c - a, g[0]), max(c + a, g[1])
            g[2].append(c)
        else:
            groups.append([c - a, c + a, [c]])
    return [(lo, hi, members) for lo, hi, members in groups]


def head_scan_for(layout, wall_distance=0.75, camera_height=1.30, half_fov=(21.0, 34.0), margin=8.0,
                  robot_u=None):
    """Lista de (pan, tilt) en radianes, de izquierda a derecha y de abajo hacia arriba."""
    robot_u = layout.width / 2.0 if robot_u is None else robot_u
    targets = []                 # (pan, tilt, medio ángulo que ocupa el marcador)
    for u, v in layout.marker_positions().values():
        left = robot_u - u
        r = math.hypot(left, wall_distance)
        dist = math.hypot(r, camera_height - layout.bottom_height - v)
        targets.append((-math.pi / 2 + math.atan2(left, wall_distance),
                        -math.atan2(camera_height - layout.bottom_height - v, r),
                        math.atan2(layout.marker_size / 2, dist)))
    span_pan = math.radians(2 * (half_fov[0] - margin))
    span_tilt = math.radians(2 * (half_fov[1] - margin))
    poses = []
    for lo, hi, pans in reversed(_groups([(t[0], t[2]) for t in targets], span_pan)):
        column = [t for t in targets if min(pans) <= t[0] <= max(pans)]
        for tlo, thi, _ in _groups([(t[1], t[2]) for t in column], span_tilt):
            poses.append((round((lo + hi) / 2, 3), round((tlo + thi) / 2, 3)))
    return poses
