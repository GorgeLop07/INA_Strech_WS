"""Lógica del barrido (sin ROS, para poder probarla sola).

El niño no hace gestos: su único "sí" es el switch. La pantalla ilumina las opciones una por una
(barrido) y el niño presiona cuando se ilumina la que quiere.

El marcador es uno solo (azul): no hay menú de colores.

Modo "figuras" (trazos preestablecidos), por peldaño de la escalera de control:
  1. Causa-efecto: una presión pinta (figura al azar, casilla siguiente).
  2. Elegir:       el niño elige la figura; la casilla es la siguiente.
  3. Componer:     figura -> fila -> columna.
  4. Planear:      igual que 3, pero la pantalla pide copiar un diseño ("haz un sol arriba").

Modo "libre" (líneas continuas):
  fila -> columna (ahí se apoya el pincel) -> dirección (flecha que gira) ->
  el robot pinta en esa dirección hasta que el niño vuelve a presionar (o suelta, en modo
  "mantener") -> otra dirección ... -> "Levantar" termina el trazo.

Modo "rehabilitacion" (detener en el momento justo):
  la plantilla de la etapa se divide en tramos. En cada esquina la flecha gira y el niño elige
  la dirección (una equivocada se registra y se vuelve a preguntar); luego el robot avanza sobre
  la plantilla y el niño presiona (o suelta) justo cuando el trazo llega al final del tramo. El
  tramo se alarga `overshoot` para que pueda pasarse. Se mide el error de paro con signo
  (+ = se pasó) y el intento se califica con estrellas.

Cada elección queda registrada con su fuente: "nino", "adulto" o "auto" (lo que eligió el sistema).
"""

import math
import random
import time
from dataclasses import dataclass, field

from muralistas_common.colors import MARKER_COLOR
from muralistas_common.metrics import stop_stars
from muralistas_common.shapes import FREE_DIRECTIONS, SHAPE_LABELS
from muralistas_common.templates import direction_name, segments

DIRECTION_ARROWS = {
    'derecha': '→', 'arriba-derecha': '↗', 'arriba': '↑', 'arriba-izquierda': '↖',
    'izquierda': '←', 'abajo-izquierda': '↙', 'abajo': '↓', 'abajo-derecha': '↘',
}
ROW_LABELS_3 = ['Arriba', 'En medio', 'Abajo']


@dataclass
class MenuConfig:
    mode: str = 'figuras'             # figuras | libre | rehabilitacion
    level: int = 3                    # 1 a 4 (peldaño del modo figuras)
    marker_color: str = MARKER_COLOR
    shapes: list = field(default_factory=lambda: ['linea', 'ola', 'circulo', 'zigzag', 'puntos', 'sol'])
    rows: int = 3
    cols: int = 4
    free_activation: str = 'toggle'   # toggle (presiona para empezar y para parar) | hold (mientras lo sostiene)
    template: list = field(default_factory=list)   # peldaño 4: ["sol:0:0", ...] (figura:fila:columna)
    seed: int = 0
    # Rehabilitación con switch
    rehab_points: list = field(default_factory=list)   # plantilla en el marco del pizarrón (m)
    rehab_name: str = ''
    rehab_size: float = 0.30
    switch_speed: float = 0.03        # m/s del robot sobre la plantilla
    overshoot: float = 0.05           # m que se alarga cada tramo


@dataclass
class Action:
    """Lo que el menú le pide al resto del sistema."""
    kind: str                 # shape | move_to_cell | move_to_point | free_start | free_stop | pen_up
    color: str = ''
    shape: str = ''
    row: int = 0
    col: int = 0
    direction_deg: float = 0.0
    length: float = 0.0       # free_start: largo del tramo (0 = el del planificador)
    speed: float = 0.0
    point: tuple = (0.0, 0.0)  # move_to_point: apoyar el pincel aquí (marco del pizarrón)


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
        self.segments = segments(cfg.rehab_points) if cfg.mode == 'rehabilitacion' else []
        self.seg_index = 0
        self.seg_start = None         # dónde empezó el tramo actual (punta real)
        self.attempt = None           # datos del intento en curso
        self.attempts = []            # intentos terminados (para el registro)
        self.tip = None
        self._saw_busy = False
        self._wait_since = 0.0
        self._after_wait = None
        self.question_start = clock()
        self._enter(self._sequence()[0])

    # --- Secuencias ----------------------------------------------------------
    def _sequence(self):
        if self.cfg.mode == 'libre':
            return ['fila', 'columna']
        if self.cfg.mode == 'rehabilitacion':
            return ['inicio']
        return {1: ['pinta'], 2: ['figura']}.get(self.cfg.level, ['figura', 'fila', 'columna'])

    def _enter(self, stage):
        self.stage = stage
        self.options, self.labels = self._options_for(stage)
        self.highlighted = 0 if self.options else -1
        self.cycles = 0
        self.paused = False
        self.question_start = self.clock()
        self._lit_since = self.question_start     # desde cuándo está iluminada la opción actual

    def _options_for(self, stage):
        c = self.cfg
        if stage == 'pinta':
            return ['pinta'], ['¡Pinta!']
        if stage == 'inicio':
            return ['empezar'], ['¡Empezar!']
        if stage == 'figura':
            return list(c.shapes), [SHAPE_LABELS[s] for s in c.shapes]
        if stage == 'fila':
            labels = ROW_LABELS_3 if c.rows == 3 else [f'Fila {i + 1}' for i in range(c.rows)]
            return [str(i) for i in range(c.rows)], labels
        if stage == 'columna':
            return [str(i) for i in range(c.cols)], [f'{i + 1}' for i in range(c.cols)]
        if stage == 'direccion' and c.mode == 'rehabilitacion':
            keys = list(FREE_DIRECTIONS)
            return keys, [f'{DIRECTION_ARROWS[k]} {k}' for k in keys]
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
            'pinta': '¡Presiona para pintar!', 'inicio': '¡Presiona para empezar!', 'figura': 'Elige una figura',
            'fila': '¿Dónde? Elige la fila', 'columna': 'Elige la casilla', 'direccion': '¿Hacia dónde?',
            'dibujando': self._drawing_prompt(),
            'pintando': 'El robot está pintando', 'moviendo': 'El robot está pintando',
        }.get(self.stage, '')

    def _drawing_prompt(self):
        stop = 'presiona para parar' if self.cfg.free_activation == 'toggle' else 'suelta para parar'
        if self.cfg.mode == 'rehabilitacion':
            return f'¡Para en la esquina! ({stop})'
        return f'Pintando... {stop}'

    def target_hint(self):
        if self.cfg.mode == 'rehabilitacion' and self.segments:
            return f'Tramo {min(self.seg_index + 1, len(self.segments))} de {len(self.segments)}'
        if self.cfg.level != 4 or not self.cfg.template or self.cfg.mode != 'figuras':
            return ''
        shape, row, col = self._template_item()
        where = ROW_LABELS_3[row].lower() if self.cfg.rows == 3 else f'fila {row + 1}'
        return f'Diseño: {SHAPE_LABELS[shape].lower()}, {where}, casilla {col + 1}'

    def _template_item(self):
        """'figura:fila:columna' (también acepta el formato viejo con color, que se ignora)."""
        parts = self.cfg.template[self.template_index % len(self.cfg.template)].split(':')
        shape, row, col = parts[0], parts[-2], parts[-1]
        return shape, int(row), int(col)

    def summary(self):
        parts = []
        if self.cfg.mode == 'rehabilitacion' and self.attempts:
            last = self.attempts[-1]
            return f'Último intento: {"★" * last["stars"] or "sin estrellas"} · error {100 * last["mean_abs_error"]:.1f} cm'
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
        self._lit_since = self.clock()
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
        if st == 'inicio':
            return self._start_attempt(decisions)
        if st == 'direccion' and self.cfg.mode == 'rehabilitacion':
            return self._rehab_direction(key, decisions)
        if st == 'pinta':
            self.selection['shape'] = self.rng.choice(self.cfg.shapes)
            decisions.append(self._record('figura', self.selection['shape'], 'auto'))
            decisions += self._auto_cell()
        elif st == 'figura':
            self.selection['shape'] = key
            if self.cfg.level == 2:
                decisions += self._auto_cell()
        elif st == 'fila':
            self.selection['row'] = int(key)
        elif st == 'columna':
            self.selection['col'] = int(key)
        elif st == 'direccion':
            if key == 'levantar':
                actions.append(Action('pen_up', color=self.cfg.marker_color))
                self._wait_then(restart=True)
                return actions, decisions
            actions.append(Action('free_start', color=self.cfg.marker_color, direction_deg=FREE_DIRECTIONS[key]))
            self._enter('dibujando')
            return actions, decisions

        seq = self._sequence()
        if st in seq and seq.index(st) + 1 < len(seq):
            self._enter(seq[seq.index(st) + 1])
            return actions, decisions

        # Secuencia completa
        s = self.selection
        color = self.cfg.marker_color
        if self.cfg.mode == 'libre':
            actions.append(Action('move_to_cell', color=color, row=s['row'], col=s['col']))
            self._wait_then(restart=False)
        else:
            actions.append(Action('shape', color=color, shape=s['shape'], row=s['row'], col=s['col']))
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
        if self.cfg.mode == 'rehabilitacion':
            self._wait_then(after='medir')
            self._saw_busy = True         # el robot ya iba avanzando: el siguiente "libre" es el paro
            return [Action('free_stop')], decisions
        self._enter('direccion')
        return [Action('free_stop')], decisions

    # --- Rehabilitación con switch ------------------------------------------------
    def _start_attempt(self, decisions):
        if not self.segments:
            return [], decisions
        self.seg_index = 0
        self.attempt = {'template': self.cfg.rehab_name, 'start': self.clock(), 'stop_errors': [],
                        'wrong_directions': 0, 'reaction_times': [], 'choice_times': [], 'reached_end': 0}
        self._wait_then(after='direccion')
        start = self.segments[0][0]
        return [Action('move_to_point', color=self.cfg.marker_color, point=tuple(start))], decisions

    def _rehab_direction(self, key, decisions):
        a, b = self.segments[self.seg_index]
        correct = direction_name(a, b)
        if key != correct:
            self.attempt['wrong_directions'] += 1
            decisions[0].choice = f'{key} (equivocada; era {correct})'
            self._enter('direccion')
            return [], decisions
        # Reacción: desde que se iluminó la dirección hasta la presión. Elección: desde la pregunta
        # (incluye esperar a que el barrido llegue a la opción).
        self.attempt['reaction_times'].append(self.clock() - self._lit_since)
        self.attempt['choice_times'].append(self.clock() - self.question_start)
        self.seg_start = self.tip if self.tip is not None else a
        seg_len = math.dist(a, b)
        self._enter('dibujando')
        self._seg_t0 = self.clock()
        return [Action('free_start', color=self.cfg.marker_color, direction_deg=FREE_DIRECTIONS[key],
                       length=seg_len + self.cfg.overshoot, speed=self.cfg.switch_speed)], decisions

    def _measure_stop(self):
        """Error de paro con signo: + = se pasó del final del tramo, - = paró antes."""
        a, b = self.segments[self.seg_index]
        seg_len = math.dist(a, b)
        start = self.seg_start if self.seg_start is not None else a
        tip = self.tip if self.tip is not None else start
        du, dv = (b[0] - a[0]) / seg_len, (b[1] - a[1]) / seg_len
        advanced = (tip[0] - start[0]) * du + (tip[1] - start[1]) * dv
        err = advanced - seg_len
        self.attempt['stop_errors'].append(err)
        if err >= self.cfg.overshoot - 0.005:
            self.attempt['reached_end'] += 1      # no paró: el robot llegó al final del tramo alargado
        self.seg_index += 1
        if self.seg_index < len(self.segments):
            self._enter('direccion')
            return []
        return self._finish_attempt()

    def _finish_attempt(self):
        att = self.attempt
        errs = att['stop_errors']
        mean_abs = sum(abs(e) for e in errs) / len(errs) if errs else float('nan')
        att.update({'duration': self.clock() - att['start'], 'mean_abs_error': mean_abs,
                    'stars': stop_stars(mean_abs, self.cfg.rehab_size) if errs else 0,
                    'completed': len(errs) - att['reached_end'], 'segments': len(self.segments)})
        self.attempts.append(att)
        self.attempt = None
        self._wait_then(restart=True)
        return [Action('pen_up', color=self.cfg.marker_color)]

    def set_template(self, name, points, size=None):
        """Cambia la plantilla (la siguiente etapa o el siguiente intento)."""
        self.cfg.rehab_name, self.cfg.rehab_points = name, list(points)
        if size:
            self.cfg.rehab_size = size
        self.segments = segments(self.cfg.rehab_points)

    # --- Espera al robot -------------------------------------------------------
    def _wait_then(self, restart=False, after=None):
        self.waiting = True
        self._saw_busy = False
        self._wait_since = self.clock()
        self._after_wait = after or ('restart' if restart else 'direccion')
        self.stage = 'pintando'
        self.options, self.labels = [], []
        self.highlighted = -1

    def robot_busy(self, busy, tip=None):
        """Llamar cuando cambia /planning/busy. `tip` = punta actual (u, v), para medir el paro.

        Devuelve acciones (al terminar un intento de rehabilitación, levantar el pincel)."""
        if tip is not None:
            self.tip = tuple(tip)
        if self.stage == 'dibujando' and not busy:
            if self.cfg.mode == 'rehabilitacion':
                return self._measure_stop()   # llegó al final del tramo alargado sin que el niño parara
            self._enter('direccion')      # el trazo libre llegó al borde
            return []
        if not self.waiting:
            return []
        if busy:
            self._saw_busy = True
            return []
        if self._saw_busy or self.clock() - self._wait_since > 3.0:
            return self._finish_wait()
        return []

    def tick(self):
        """Por si /planning/busy nunca se puso en verdadero (pedido rechazado)."""
        if self.waiting and not self._saw_busy and self.clock() - self._wait_since > 3.0:
            return self._finish_wait()
        return []

    def _finish_wait(self):
        self.waiting = False
        if self._after_wait == 'medir':
            return self._measure_stop()
        if self._after_wait == 'direccion':
            self._enter('direccion')
        else:
            self.selection = {}
            self._enter(self._sequence()[0])
        return []
