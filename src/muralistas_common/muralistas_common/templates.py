"""Plantillas del modo rehabilitación y utilidades de polilíneas (sin ROS).

Cada plantilla es un solo trazo continuo, normalizado al cuadrado [0, 1] x [0, 1] con y hacia
arriba, así el marcador no se levanta en toda la plantilla. Con switch (modo S) el niño elige
direcciones de una flecha que gira por 8 direcciones, así que esas plantillas solo tienen tramos
a múltiplos de 45°: el círculo se aproxima con un octágono y el techo de la casa va a 45°.
"""

import math

TEMPLATE_LABELS = {
    'linea': 'Línea',
    'linea_vertical': 'Línea de pie',
    'cuadrado': 'Cuadrado',
    'circulo': 'Círculo',
    'octagono': 'Círculo',
    'casa': 'Casa',
}

# Etapa -> plantillas que se alternan intento a intento
STAGES = {
    0: ['linea', 'linea_vertical'],
    1: ['cuadrado'],
    2: ['circulo'],
    3: ['casa'],
}
LAST_STAGE = max(STAGES)


def _octagon():
    pts = [(0.0, 0.0)]
    for k in range(8):
        a = math.radians(45 * k)
        x, y = pts[-1]
        pts.append((x + math.cos(a), y + math.sin(a)))
    xs, ys = [p[0] for p in pts], [p[1] for p in pts]
    x0, y0, span = min(xs), min(ys), max(max(xs) - min(xs), max(ys) - min(ys))
    return [((x - x0) / span, (y - y0) / span) for x, y in pts]


def _circle(n=64):
    # Empieza abajo y va en sentido antihorario, como el octágono
    return [(0.5 + 0.5 * math.cos(-math.pi / 2 + 2 * math.pi * i / n),
             0.5 + 0.5 * math.sin(-math.pi / 2 + 2 * math.pi * i / n)) for i in range(n + 1)]


TEMPLATES = {
    'linea': [(0.0, 0.5), (1.0, 0.5)],
    'linea_vertical': [(0.5, 0.0), (0.5, 1.0)],
    'cuadrado': [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0), (0.0, 0.0)],
    'circulo': _circle(),
    'octagono': _octagon(),
    'casa': [(0.0, 0.0), (1.0, 0.0), (1.0, 0.5), (0.5, 1.0), (0.0, 0.5), (0.0, 0.0)],
}


def stage_template(stage, attempt, switch=False):
    """Nombre de la plantilla de una etapa en un intento. Con switch, el círculo es octágono."""
    names = STAGES[min(max(stage, 0), LAST_STAGE)]
    name = names[attempt % len(names)]
    if switch and name == 'circulo':
        return 'octagono'
    return name


def template_in_zone(name, layout, size=None):
    """Plantilla centrada en la zona de trabajo, en coordenadas del pizarrón (m)."""
    if name not in TEMPLATES:
        raise KeyError(f'Plantilla desconocida: {name}')
    size = size or layout.template_size
    u0, v0, u1, v1 = layout.work_rect()
    side = min(size, u1 - u0, v1 - v0)
    cu, cv = (u0 + u1) / 2.0, (v0 + v1) / 2.0
    return [(cu - side / 2.0 + x * side, cv - side / 2.0 + y * side) for x, y in TEMPLATES[name]]


def segments(points, min_length=1e-6):
    """Tramos rectos (inicio, fin) de una polilínea, sin tramos de largo cero."""
    return [(a, b) for a, b in zip(points[:-1], points[1:]) if math.dist(a, b) > min_length]


def path_length(points):
    return sum(math.dist(a, b) for a, b in zip(points[:-1], points[1:]))


def resample(points, step):
    """Puntos a distancia `step` a lo largo de la polilínea (incluye el primero y el último)."""
    if len(points) < 2:
        return list(points)
    out = [points[0]]
    carry = 0.0
    for a, b in zip(points[:-1], points[1:]):
        seg = math.dist(a, b)
        if seg <= 0.0:
            continue
        d = step - carry
        while d <= seg:
            t = d / seg
            out.append((a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t))
            d += step
        carry = seg - (d - step)
    if math.dist(out[-1], points[-1]) > 1e-9:
        out.append(points[-1])
    else:
        out[-1] = points[-1]          # el redondeo deja el último a 1e-16: que sea exacto
    return out


def nearest_on_polyline(p, points):
    """(punto más cercano de la polilínea a p, distancia)."""
    best, best_d = points[0], math.dist(p, points[0])
    for a, b in zip(points[:-1], points[1:]):
        ax, ay = a
        dx, dy = b[0] - ax, b[1] - ay
        L2 = dx * dx + dy * dy
        t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((p[0] - ax) * dx + (p[1] - ay) * dy) / L2))
        q = (ax + t * dx, ay + t * dy)
        d = math.dist(p, q)
        if d < best_d:
            best, best_d = q, d
    return best, best_d


def direction_name(a, b):
    """La dirección de las 8 del barrido más cercana al tramo a -> b."""
    from .shapes import FREE_DIRECTIONS
    ang = math.degrees(math.atan2(b[1] - a[1], b[0] - a[0])) % 360.0
    return min(FREE_DIRECTIONS, key=lambda k: abs((FREE_DIRECTIONS[k] - ang + 180.0) % 360.0 - 180.0))
