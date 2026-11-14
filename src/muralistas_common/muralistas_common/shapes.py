"""Biblioteca de figuras (los "trazos preestablecidos").

Cada figura es una lista de trazos. Cada trazo es una lista de puntos (x, y)
normalizados en el cuadrado [0, 1] x [0, 1], con y hacia arriba.
Un trazo de un solo punto es un "toque" (el pincel se apoya y se levanta).

La planeación las escala a la casilla elegida y la pantalla las usa como iconos,
así la figura que ve el niño es exactamente la que pinta el robot.
"""

import math

# Nombre visible para el niño / la terapeuta, en el orden del barrido.
SHAPE_LABELS = {
    'linea': 'Línea',
    'vertical': 'Línea de pie',
    'ola': 'Ola',
    'circulo': 'Círculo',
    'zigzag': 'Zigzag',
    'espiral': 'Espiral',
    'puntos': 'Puntos',
    'sol': 'Sol',
}


def _circle(cx, cy, r, n=48, start=0.0, turns=1.0):
    return [(cx + r * math.cos(start + 2 * math.pi * turns * i / n),
             cy + r * math.sin(start + 2 * math.pi * turns * i / n)) for i in range(n + 1)]


def _build():
    shapes = {}
    shapes['linea'] = [[(0.05, 0.5), (0.95, 0.5)]]
    shapes['vertical'] = [[(0.5, 0.05), (0.5, 0.95)]]
    shapes['ola'] = [[(0.05 + 0.9 * i / 60, 0.5 + 0.3 * math.sin(2 * math.pi * 2 * i / 60))
                      for i in range(61)]]
    shapes['circulo'] = [_circle(0.5, 0.5, 0.42)]
    shapes['zigzag'] = [[(0.05 + 0.15 * i, 0.2 if i % 2 == 0 else 0.8) for i in range(7)]]
    shapes['espiral'] = [[(0.5 + (0.03 + 0.42 * i / 90) * math.cos(2 * math.pi * 2.5 * i / 90),
                           0.5 + (0.03 + 0.42 * i / 90) * math.sin(2 * math.pi * 2.5 * i / 90))
                          for i in range(91)]]
    shapes['puntos'] = [[(x, y)] for y in (0.8, 0.5, 0.2) for x in (0.2, 0.5, 0.8)]
    sol = [_circle(0.5, 0.5, 0.2, n=36)]
    for k in range(8):
        a = 2 * math.pi * k / 8
        sol.append([(0.5 + 0.28 * math.cos(a), 0.5 + 0.28 * math.sin(a)),
                    (0.5 + 0.46 * math.cos(a), 0.5 + 0.46 * math.sin(a))])
    shapes['sol'] = sol
    return shapes


SHAPES = _build()


def shape_names():
    return list(SHAPE_LABELS.keys())


def shape_in_box(name, u0, v0, side):
    """Escala una figura al cuadrado (u0, v0, lado) en coordenadas del mural (m)."""
    if name not in SHAPES:
        raise KeyError(f'Figura desconocida: {name}')
    return [[(u0 + x * side, v0 + y * side) for (x, y) in stroke] for stroke in SHAPES[name]]


def path_length(stroke):
    return sum(math.dist(a, b) for a, b in zip(stroke[:-1], stroke[1:]))


# Direcciones del trazo libre (barrido giratorio), en grados: 0 = derecha, 90 = arriba.
FREE_DIRECTIONS = {
    'derecha': 0.0,
    'arriba-derecha': 45.0,
    'arriba': 90.0,
    'arriba-izquierda': 135.0,
    'izquierda': 180.0,
    'abajo-izquierda': 225.0,
    'abajo': 270.0,
    'abajo-derecha': 315.0,
}
