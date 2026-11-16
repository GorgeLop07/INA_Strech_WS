"""Genera los ArUco del pizarrón para imprimir a tamaño real (DICT_4X4_50, IDs 0 a 5).

  ros2 run muralistas_perception generar_arucos                 # 15 cm, en ./arucos/
  ros2 run muralistas_perception generar_arucos --lado 0.155 --dpi 300 --salida ~/arucos
  ros2 run muralistas_perception generar_arucos --distribucion esquinas     # solo los 4 de las esquinas

Cada PNG lleva el ID y una regla de 5 cm: imprímelo al 100% (sin "ajustar a la página") y mide
con una regla que el cuadro negro mida exactamente `lado`. Si no, la pose del pizarrón sale escalada.
Posición (config/mural.yaml, markers: seis): centro a margin/2 del borde de abajo o de arriba;
ID 0 abajo-izquierda, 1 abajo-centro, 2 abajo-derecha, 3 arriba-derecha, 4 arriba-centro,
5 arriba-izquierda. Los del centro van a la mitad del ancho.
"""

import argparse
from pathlib import Path

import cv2
import numpy as np

from muralistas_common.layout import MuralLayout


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--lado', type=float, default=MuralLayout().marker_size, help='lado del cuadro negro (m)')
    ap.add_argument('--dpi', type=int, default=300)
    ap.add_argument('--distribucion', default='seis', choices=['seis', 'esquinas'])
    ap.add_argument('--ids', type=int, nargs='+', help='solo estos IDs (por omisión, todos los de la distribución)')
    ap.add_argument('--salida', default='arucos')
    args = ap.parse_args()
    names = MuralLayout(markers=args.distribucion).marker_names()
    ids = args.ids or sorted(names)
    out = Path(args.salida).expanduser()
    out.mkdir(parents=True, exist_ok=True)
    px_per_m = args.dpi / 0.0254
    side = int(round(args.lado * px_per_m))
    border = int(round(0.015 * px_per_m))             # 1.5 cm de blanco alrededor (zona de silencio)
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    for marker_id in ids:
        marker = cv2.aruco.generateImageMarker(dictionary, marker_id, side)
        h = side + 2 * border + int(0.04 * px_per_m)
        page = np.full((h, side + 2 * border), 255, np.uint8)
        page[border:border + side, border:border + side] = marker
        y = border + side + int(0.012 * px_per_m)
        ruler = int(0.05 * px_per_m)
        cv2.line(page, (border, y), (border + ruler, y), 0, max(2, args.dpi // 150))
        cv2.putText(page, f'ID {marker_id} ({names.get(marker_id, "")})  5 cm', (border + ruler + 20, y + 10),
                    cv2.FONT_HERSHEY_SIMPLEX, args.dpi / 300, 0, max(1, args.dpi // 150))
        path = out / f'aruco_{marker_id}.png'
        cv2.imwrite(str(path), page)
        print(f'{path}  (cuadro negro {100 * args.lado:.1f} cm = {side} px a {args.dpi} dpi)')


if __name__ == '__main__':
    main()
