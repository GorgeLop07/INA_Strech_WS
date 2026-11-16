"""Genera los ArUco del pizarrón para imprimir a tamaño real (DICT_4X4_50, IDs 0 a 3).

  ros2 run muralistas_perception generar_arucos                 # 10 cm, en ./arucos/
  ros2 run muralistas_perception generar_arucos --lado 0.10 --dpi 300 --salida ~/arucos

Cada PNG lleva el ID y una regla de 5 cm: imprímelo al 100% (sin "ajustar a la página") y mide
con una regla que el cuadro negro mida exactamente `lado`. Si no, la pose del pizarrón sale escalada.
Posición (config/mural.yaml, markers: esquinas): centro a margin/2 de cada borde;
ID 0 abajo-izquierda, 1 abajo-derecha, 2 arriba-derecha, 3 arriba-izquierda.
"""

import argparse
from pathlib import Path

import cv2
import numpy as np


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--lado', type=float, default=0.10, help='lado del cuadro negro (m)')
    ap.add_argument('--dpi', type=int, default=300)
    ap.add_argument('--ids', type=int, nargs='+', default=[0, 1, 2, 3])
    ap.add_argument('--salida', default='arucos')
    args = ap.parse_args()
    out = Path(args.salida).expanduser()
    out.mkdir(parents=True, exist_ok=True)
    px_per_m = args.dpi / 0.0254
    side = int(round(args.lado * px_per_m))
    border = int(round(0.015 * px_per_m))             # 1.5 cm de blanco alrededor (zona de silencio)
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    names = {0: 'abajo-izquierda', 1: 'abajo-derecha', 2: 'arriba-derecha', 3: 'arriba-izquierda'}
    for marker_id in args.ids:
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
