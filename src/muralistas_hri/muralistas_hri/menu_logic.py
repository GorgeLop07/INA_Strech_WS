"""Lógica del barrido (sin ROS, para poder probarla sola).

El niño no hace gestos: su único "sí" es el switch. La pantalla ilumina las opciones una por una
(barrido) y el niño presiona cuando se ilumina la que quiere.

Modo "figuras" (trazos preestablecidos), por peldaño de la escalera de control:
  1. Causa-efecto: una presión pinta (color y figura al azar, casilla siguiente).
  2. Elegir:       el niño elige el color; figura al azar.
  3. Componer:     color -> figura -> fila -> columna.
  4. Planear:      igual que 3, pero la pantalla pide copiar un diseño ("haz un sol amarillo arriba").

Modo "libre" (líneas continuas):
  color -> fila -> columna (ahí se apoya el pincel) -> dirección (flecha que gira) ->
  el robot pinta en esa dirección hasta que el niño vuelve a presionar (o suelta, en modo
  "mantener") -> otra dirección ... -> "Levantar" termina el trazo.

Cada elección queda registrada con su fuente: "nino", "adulto" o "auto" (lo que eligió el sistema).
"""

import random
import time
from dataclasses import dataclass, field

from muralistas_common.colors import PALETTE
from muralistas_common.shapes import FREE_DIRECTIONS, SHAPE_LABELS

DIRECTION_ARROWS = {
    'derecha': '→', 'arriba-derecha': '↗', 'arriba': '↑', 'arriba-izquierda': '↖',
    'izquierda': '←', 'abajo-izquierda': '↙', 'abajo': '↓', 'abajo-derecha': '↘',
}
ROW_LABELS_3 = ['Arriba', 'En medio', 'Abajo']


@dataclass
class MenuConfig:
    mode: str = 'figuras'             # figuras | libre
    level: int = 3                    # 1 a 4
    colors: list = field(default_factory=lambda: ['rojo', 'amarillo', 'azul'])
    shapes: list = field(default_factory=lambda: ['linea', 'ola', 'circulo', 'zigzag', 'puntos', 'sol'])
    rows: int = 3
    cols: int = 4
    free_activation: str = 'toggle'   # toggle (presiona para empezar y para parar) | hold (mientras lo sostiene)
    template: list = field(default_factory=list)   # peldaño 4: ["sol:amarillo:0:0", ...]
    seed: int = 0


@dataclass
class Action:
    """Lo que el menú le pide al resto del sistema."""
    kind: str                 # shape | move_to_cell | free_start | free_stop | pen_up
    color: str = ''
    shape: str = ''
    row: int = 0
    col: int = 0
    direction_deg: float = 0.0


@dataclass
class DecisionRecord:
    stage: str
    choice: str
    source: str
    response_time: float
    scan_cycles: int


class ScanningMenu:
    def __init__(self, cfg: MenuConfig, clock=time.monotonic):
        self.cfg = cfg
        self.clock = clock
        self.rng = random.Random(cfg.seed)
        self.selection = {}
        self.cell_cursor = 0
        self.template_index = 0
        self.stage = ''
        self.options, self.labels = [], []
        self.highlighted = -1
        self.cycles = 0
        self.paused = False
        self.waiting = False          # esperando que el robot termine
        self._saw_busy = False
        self._wait_since = 0.0
        self._after_wait = None
        self.question_start = clock()
        self._enter(self._sequence()[0])

    # --- Secuencias ----------------------------------------------------------
    def _sequence(self):
        if self.cfg.mode == 'libre':
            return ['color', 'fila', 'columna']
        return {1: ['pinta'], 2: ['color']}.get(self.cfg.level, ['color', 'figura', 'fila', 'columna'])

    def _enter(self, stage):
        self.stage = stage
        self.options, self.labels = self._options_for(stage)
        self.highlighted = 0 if self.options else -1
        self.cycles = 0
        self.paused = False
        self.question_start = self.clock()

    def _options_for(self, stage):
        c = self.cfg
        if stage == 'pinta':
            return ['pinta'], ['¡Pinta!']
        if stage == 'color':
            return list(c.colors), [k.capitalize() for k in c.colors]
        if stage == 'figura':
            return list(c.shapes), [SHAPE_LABELS[s] for s in c.shapes]
        if stage == 'fila':
            labels = ROW_LABELS_3 if c.rows == 3 else [f'Fila {i + 1}' for i in range(c.rows)]
            return [str(i) for i in range(c.rows)], labels
        if stage == 'columna':
            return [str(i) for i in range(c.cols)], [f'{i + 1}' for i in range(c.cols)]
        if stage == 'direccion':
            keys = list(FREE_DIRECTIONS) + ['levantar']
            return keys, [f'{DIRECTION_ARROWS[k]} {k}' for k in FREE_DIRECTIONS] + ['Levantar']
        if stage == 'dibujando':
            return ['alto'], ['¡Alto!']
        return [], []

    def reset_stage(self):
        """Reinicia el barrido de la etapa actual (por ejemplo, cuando el robot queda listo)."""
        if not self.waiting:
            self._enter(self.stage)

    @property
    def prompt(self):
        if self.paused:
            return 'Presiona para seguir'
        return {
            'pinta': '¡Presiona para pintar!', 'color': 'Elige un color', 'figura': 'Elige una figura',
            'fila': '¿Dónde? Elige la fila', 'columna': 'Elige la casilla', 'direccion': '¿Hacia dónde?',
            'dibujando': 'Pintando... presiona para parar' if self.cfg.free_activation == 'toggle'
            else 'Pintando... suelta para parar',
            'pintando': 'El robot está pintando', 'moviendo': 'El robot está pintando',
        }.get(self.stage, '')

    def target_hint(self):
        if self.cfg.level != 4 or not self.cfg.template or self.cfg.mode != 'figuras':
            return ''
        shape, color, row, col = self._template_item()
        where = ROW_LABELS_3[row].lower() if self.cfg.rows == 3 else f'fila {row + 1}'
        return f'Diseño: {SHAPE_LABELS[shape].lower()} {color}, {where}, casilla {col + 1}'

    def _template_item(self):
        item = self.cfg.template[self.template_index % len(self.cfg.template)]
        shape, color, row, col = item.split(':')
        return shape, color, int(row), int(col)

    def summary(self):
        parts = []
        if 'color' in self.selection:
            parts.append(self.selection['color'])
        if 'shape' in self.selection:
            parts.append(SHAPE_LABELS.get(self.selection['shape'], self.selection['shape']).lower())
        if 'row' in self.selection:
            parts.append(f'fila {self.selection["row"] + 1}')
        if 'col' in self.selection:
            parts.append(f'casilla {self.selection["col"] + 1}')
        return ' · '.join(parts)

    # --- Barrido -----------------------------------------------------------
    @property
    def scanning(self):
        return not self.waiting and not self.paused and self.stage not in ('dibujando',) and len(self.options) > 1

    def advance(self):
        """Siguiente opción (lo llama el temporizador, o el segundo switch en barrido por pasos)."""
        if not self.options or self.waiting or self.paused:
            return
        self.highlighted = (self.highlighted + 1) % len(self.options)
        if self.highlighted == 0:
            self.cycles += 1

    def pause(self):
        self.paused = True
        self.highlighted = -1

    # --- Switch --------------------------------------------------------------
    def press(self, source='nino'):
        """Presión aceptada del switch principal. Devuelve (acciones, decisiones)."""
        if self.waiting:
            return [], []
        if self.paused:
            self.paused = False
            self.highlighted = 0
            self.cycles = 0
            self.question_start = self.clock()
            return [], []
        if self.stage == 'dibujando':
            if self.cfg.free_activation == 'toggle':
                return self._stop_free(source)
            return [], []
        if self.highlighted < 0:
            return [], []
        return self._select(self.options[self.highlighted], source)

    def release(self, source='nino'):
        if self.stage == 'dibujando' and self.cfg.free_activation == 'hold':
            return self._stop_free(source)
        return [], []

    def _record(self, stage, choice, source):
        return DecisionRecord(stage, choice, source, self.clock() - self.question_start, self.cycles)

    def _select(self, key, source):
        decisions = [self._record(self.stage, key, source)]
        actions = []
        st = self.stage
        if st == 'pinta':
            self.selection['color'] = self.rng.choice(self.cfg.colors)
            self.selection['shape'] = self.rng.choice(self.cfg.shapes)
            decisions += [self._record('color', self.selection['color'], 'auto'),
                          self._record('figura', self.selection['shape'], 'auto')]
            decisions += self._auto_cell()
        elif st == 'color':
            self.selection['color'] = key
            if self.cfg.mode == 'figuras' and self.cfg.level == 2:
                self.selection['shape'] = self.rng.choice(self.cfg.shapes)
                decisions.append(self._record('figura', self.selection['shape'], 'auto'))
                decisions += self._auto_cell()
        elif st == 'figura':
            self.selection['shape'] = key
        elif st == 'fila':
            self.selection['row'] = int(key)
        elif st == 'columna':
            self.selection['col'] = int(key)
        elif st == 'direccion':
            if key == 'levantar':
                actions.append(Action('pen_up', color=self.selection.get('color', '')))
                self._wait_then(restart=True)
                return actions, decisions
            actions.append(Action('free_start', color=self.selection['color'], direction_deg=FREE_DIRECTIONS[key]))
            self._enter('dibujando')
            return actions, decisions

        seq = self._sequence()
        if st in seq and seq.index(st) + 1 < len(seq):
            self._enter(seq[seq.index(st) + 1])
            return actions, decisions

        # Secuencia completa
        s = self.selection
        if self.cfg.mode == 'libre':
            actions.append(Action('move_to_cell', color=s['color'], row=s['row'], col=s['col']))
            self._wait_then(restart=False)
        else:
            actions.append(Action('shape', color=s['color'], shape=s['shape'], row=s['row'], col=s['col']))
            if self.cfg.level == 4 and self.cfg.template:
                self.template_index += 1
            self._wait_then(restart=True)
        return actions, decisions

    def _auto_cell(self):
        n = self.cfg.rows * self.cfg.cols
        cell = self.cell_cursor % n
        self.cell_cursor += 1
        self.selection['row'], self.selection['col'] = divmod(cell, self.cfg.cols)
        return [self._record('casilla', f'{self.selection["row"]},{self.selection["col"]}', 'auto')]

    def _stop_free(self, source):
        decisions = [self._record('dibujando', 'alto', source)]
        self._enter('direccion')
        return [Action('free_stop')], decisions

    # --- Espera al robot -------------------------------------------------------
    def _wait_then(self, restart):
        self.waiting = True
        self._saw_busy = False
        self._wait_since = self.clock()
        self._after_wait = 'restart' if restart else 'direccion'
        self.stage = 'pintando'
        self.options, self.labels = [], []
        self.highlighted = -1

    def robot_busy(self, busy):
        """Llamar cuando cambia /planning/busy."""
        if self.stage == 'dibujando' and not busy:
            self._enter('direccion')      # el trazo libre llegó al borde
            return
        if not self.waiting:
            return
        if busy:
            self._saw_busy = True
            return
        if self._saw_busy or self.clock() - self._wait_since > 3.0:
            self._finish_wait()

    def tick(self):
        """Por si /planning/busy nunca se puso en verdadero (pedido rechazado)."""
        if self.waiting and not self._saw_busy and self.clock() - self._wait_since > 3.0:
            self._finish_wait()

    def _finish_wait(self):
        self.waiting = False
        if self._after_wait == 'direccion':
            self._enter('direccion')
        else:
            self.selection = {}
            self._enter(self._sequence()[0])


def default_colors():
    return [c for c in PALETTE if c != 'negro']
