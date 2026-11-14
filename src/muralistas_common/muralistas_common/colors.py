"""Paleta de pintura. El verde se reserva para el cuerpo del pincel (la percepción lo segmenta por ese color)."""

PALETTE = {
    'rojo': (0.85, 0.12, 0.12),
    'amarillo': (0.98, 0.80, 0.05),
    'azul': (0.10, 0.30, 0.85),
    'morado': (0.55, 0.20, 0.70),
    'negro': (0.08, 0.08, 0.08),
}

BRUSH_BODY_RGB = (0.20, 0.85, 0.20)


def rgb(name):
    if name not in PALETTE:
        raise KeyError(f'Color desconocido: {name}. Opciones: {list(PALETTE)}')
    return PALETTE[name]


def hex_color(name):
    r, g, b = rgb(name)
    return '#{:02x}{:02x}{:02x}'.format(int(r * 255), int(g * 255), int(b * 255))
