"""Interacción con joystick: el fantasma (sin ROS, para poder probarla).

El robot nunca sigue la mano en directo: el niño traza un fantasma en la pantalla y el robot
pinta solo lo confirmado. Modos de interacción:
  - J1: una mano que no puede soltar el joystick. Confirma dejándolo quieto `dwell` segundos
        (se llena un anillo); el fantasma debe medir al menos `min_length`, y el robot espera
        `start_delay` más: cualquier movimiento en ese tiempo cancela. Borrar: regresar el
        cursor al inicio del fantasma y quedarse quieto ahí (solo en modo libre).
  - J2: una mano, joystick y luego switch. Confirma soltando el switch; presión larga = borrar.
  - J3: dos manos (joystick en una, switch en la otra). Igual que J2.

Modo libre, ciclo: menú levantado / pegado (izquierda / derecha del joystick) -> trazar el
fantasma desde la punta -> confirmar -> el robot se desplaza (levantado) o pinta (pegado) ->
otra vez el menú. Modo rehabilitación: el robot lleva el marcador al inicio de la plantilla, el
niño traza encima (con imán opcional) y al confirmar se califica y el robot pinta el fantasma.

Todo en el marco del pizarrón (m). Las acciones que devuelve:
  ('place_cursor', (u, v))                       mover el cursor (p. ej. a la punta)
  ('ghost', points, pen_down, lift_at_end)       mandar el fantasma al robot
  ('decision', stage, choice, response_time, source)   para el registro de decisiones
  ('attempt', dict)                              intento calificado (rehabilitación)
"""

import math
from dataclasses import dataclass, field

from muralistas_common.metrics import ldlj, msj, sparc, trace_metrics, trace_stars
from muralistas_common.templates import nearest_on_polyline, path_length


@dataclass
class GhostConfig:
    interaction: str = 'J2'        # J1 | J2 | J3
    mode: str = 'libre'            # libre | rehabilitacion
    dwell: float = 1.5             # J1: t_quieto (s)
    still_radius: float = 0.003    # m: el cursor está quieto si no sale de este radio
    min_length: float = 0.03       # m: un fantasma más corto no se confirma
    start_delay: float = 1.0       # J1: espera antes de que el robot arranque
    erase_radius: float = 0.02     # J1: quieto a esta distancia del inicio = borrar
    long_press: float = 1.0        # J2/J3: presión de al menos esto = borrar
    sample_step: float = 0.002     # m entre puntos del fantasma
    menu_threshold: float = 0.5    # deflexión para elegir en el menú levantado / pegado
    center_threshold: float = 0.15
    magnet: float = 0.0            # rehabilitación: 0 a 1
    template: list = field(default_factory=list)
    template_name: str = ''
    template_size: float = 0.30
    tolerance: float = 0.015       # m: cobertura de la plantilla
    rate: float = 50.0             # Hz de update(): para la suavidad


class GhostInteraction:
    def __init__(self, cfg: GhostConfig, t0=0.0):
        self.cfg = cfg
        self.stage = 'preparando'
        self.pen_state = ''            # levantado | pegado
        self.highlight = ''
        self.ghost, self.raw_path, self.deflections = [], [], []
        self.cursor = (0.0, 0.0)
        self.tip = None
        self.dwell_progress = 0.0
        self.last_attempt = None
        self._t = t0
        self._question_t = t0
        self._still_anchor, self._still_since = None, t0
        self._center_since = None
        self._armed_t, self._armed_pos = None, None
        self._saw_busy = False
        self._wait_since = t0
        self._after_wait = ''
        self._attempt_t0 = t0

    # --- Arranque ---------------------------------------------------------------
    def start(self, tip, t):
        """El robot está listo y la punta está en `tip`. Devuelve acciones."""
        self.tip = tuple(tip)
        self._t = t
        if self.cfg.mode == 'rehabilitacion':
            return self._go_to_template_start(t)
        return self._enter_menu(t)

    def set_template(self, name, points, size=None):
        self.cfg.template_name, self.cfg.template = name, list(points)
        if size:
            self.cfg.template_size = size

    # --- Entradas ---------------------------------------------------------------
    def update(self, t, cursor, deflection=(0.0, 0.0), raw=None):
        """Llamar a `rate` Hz con el cursor (marco del pizarrón), la deflexión filtrada (menú) y la
        cruda (métricas de suavidad; si no se da, se usa la filtrada)."""
        self._t = t
        self.cursor = tuple(cursor)
        if self.stage == 'menu':
            return self._update_menu(t, deflection)
        if self.stage == 'trazando':
            self.deflections.append(tuple(raw if raw is not None else deflection))
            return self._update_tracing(t)
        if self.stage == 'armando':
            if math.dist(self.cursor, self._armed_pos) > 0.01:
                self.stage = 'trazando'          # se movió: no era el final
                self._reset_still(t)
                return []
            if t - self._armed_t >= self.cfg.start_delay:
                return self._send(t)
        if self.stage == 'esperando' and not self._saw_busy and t - self._wait_since > 3.0:
            return self._finish_wait(t)          # el pedido nunca puso al robot ocupado
        return []

    def switch(self, pressed, hold_duration, t):
        """Switch ya filtrado (ZR o el switch del niño). J2/J3 actúan al soltar."""
        self._t = t
        if pressed or self.cfg.interaction == 'J1':
            return []
        if self.stage == 'menu':
            return self._choose(self.highlight, t) if self.highlight else []
        if self.stage == 'trazando':
            if hold_duration >= self.cfg.long_press:
                return self._erase(t)
            if path_length(self.ghost) >= self.cfg.min_length:
                return self._send(t)
        return []

    def confirm(self, t):
        """Botón A (el adulto o el experimentador): confirma en cualquier modo."""
        if self.stage == 'menu' and self.highlight:
            return self._choose(self.highlight, t, source='adulto')
        if self.stage in ('trazando', 'armando') and path_length(self.ghost) >= self.cfg.min_length:
            return self._send(t, source='adulto')
        return []

    def cancel(self, t):
        """Botón B: borra el fantasma."""
        if self.stage in ('trazando', 'armando'):
            return self._erase(t)
        return []

    def robot_busy(self, busy, tip, t):
        if tip is not None:
            self.tip = tuple(tip)
        if self.stage != 'esperando':
            return []
        if busy:
            self._saw_busy = True
            return []
        if self._saw_busy or t - self._wait_since > 3.0:
            return self._finish_wait(t)
        return []

    # --- Menú levantado / pegado ---------------------------------------------------
    def _enter_menu(self, t):
        self.stage = 'menu'
        self.highlight = ''
        self._center_since = None
        self._question_t = t
        self.dwell_progress = 0.0
        return [('place_cursor', self.tip)] if self.tip else []

    def _update_menu(self, t, deflection):
        x = deflection[0]
        if x <= -self.cfg.menu_threshold:
            self.highlight = 'levantado'
        elif x >= self.cfg.menu_threshold:
            self.highlight = 'pegado'
        if self.cfg.interaction != 'J1' or not self.highlight:
            return []
        # J1: elige lo iluminado dejando el joystick en el centro, quieto, `dwell` segundos
        if math.hypot(*deflection) < self.cfg.center_threshold:
            if self._center_since is None:
                self._center_since = t
            self.dwell_progress = min(1.0, (t - self._center_since) / self.cfg.dwell)
            if t - self._center_since >= self.cfg.dwell:
                return self._choose(self.highlight, t)
        else:
            self._center_since = None
            self.dwell_progress = 0.0
        return []

    def _choose(self, choice, t, source='nino'):
        self.pen_state = choice
        actions = [('decision', 'estado', choice, t - self._question_t, source)]
        start = self.tip or self.cursor
        self._begin_tracing(start, t)
        return actions + [('place_cursor', start)]

    # --- Trazo -------------------------------------------------------------------
    def _begin_tracing(self, start, t):
        self.stage = 'trazando'
        self.ghost = [tuple(start)]
        self.raw_path = [tuple(start)]
        self.deflections = []
        self.cursor = tuple(start)
        self._question_t = t
        self._attempt_t0 = t
        self._reset_still(t)
        self.dwell_progress = 0.0

    def _reset_still(self, t):
        self._still_anchor, self._still_since = self.cursor, t

    def _update_tracing(self, t):
        p = self.cursor
        if math.dist(p, self.raw_path[-1]) >= self.cfg.sample_step:
            self.raw_path.append(p)
            shown = p
            if self.cfg.mode == 'rehabilitacion' and self.cfg.magnet > 0 and self.cfg.template:
                q, _ = nearest_on_polyline(p, self.cfg.template)
                k = self.cfg.magnet
                shown = (p[0] + k * (q[0] - p[0]), p[1] + k * (q[1] - p[1]))
            self.ghost.append(shown)
        if self.cfg.interaction != 'J1':
            return []
        if math.dist(p, self._still_anchor) > self.cfg.still_radius:
            self._reset_still(t)
        length = path_length(self.ghost)
        if length < self.cfg.min_length:
            self.dwell_progress = 0.0
            return []
        still = t - self._still_since
        self.dwell_progress = min(1.0, still / self.cfg.dwell)
        if still < self.cfg.dwell:
            return []
        if self.cfg.mode == 'libre' and math.dist(p, self.ghost[0]) < self.cfg.erase_radius \
                and max(math.dist(q, self.ghost[0]) for q in self.ghost) > 2 * self.cfg.erase_radius:
            return self._erase(t)
        self.stage = 'armando'
        self._armed_t, self._armed_pos = t, p
        return []

    def _erase(self, t):
        start = self.ghost[0] if self.ghost else self.cursor
        self._begin_tracing(start, t)
        return [('decision', 'fantasma', 'borrar', 0.0, 'nino'), ('place_cursor', start)]

    def _send(self, t, source='nino'):
        pts = list(self.ghost)
        actions = [('decision', 'fantasma', 'confirmar', t - self._question_t, source)]
        if self.cfg.mode == 'rehabilitacion':
            actions.append(('attempt', self._score(t)))
            actions.append(('ghost', pts, True, True))
        else:
            pegado = self.pen_state == 'pegado'
            actions.append(('ghost', pts, pegado, False))
        self.stage = 'esperando'
        self._saw_busy = False
        self._wait_since = t
        self._after_wait = 'rehab' if self.cfg.mode == 'rehabilitacion' else 'menu'
        self.dwell_progress = 0.0
        return actions

    def _score(self, t):
        err, cov = trace_metrics(self.raw_path, self.cfg.template, tol=self.cfg.tolerance)
        stars = trace_stars(err, cov, self.cfg.template_size)
        fs = self.cfg.rate
        d = self.deflections
        smooth = {'ldlj': ldlj(d, fs), 'sparc': sparc(d, fs), 'msj': msj(d, fs)} if len(d) > 10 else \
            {'ldlj': float('nan'), 'sparc': float('nan'), 'msj': float('nan')}
        self.last_attempt = {'template': self.cfg.template_name, 'stars': stars, 'mean_error': err,
                             'coverage': cov, 'duration': t - self._attempt_t0, **smooth,
                             'raw_path': list(self.raw_path), 'ghost': list(self.ghost),
                             'magnet': self.cfg.magnet}
        return self.last_attempt

    # --- Espera al robot --------------------------------------------------------
    def _go_to_template_start(self, t):
        start = tuple(self.cfg.template[0])
        self.stage = 'esperando'
        self._saw_busy = False
        self._wait_since = t
        self._after_wait = 'trazar_plantilla'
        return [('ghost', [start], False, False)]

    def _finish_wait(self, t):
        after = self._after_wait
        if after == 'menu':
            return self._enter_menu(t)
        if after == 'rehab':
            return self._go_to_template_start(t)
        if after == 'trazar_plantilla':
            start = tuple(self.cfg.template[0])
            self.pen_state = 'pegado'
            self._begin_tracing(start, t)
            return [('place_cursor', start)]
        return []

    # --- Para la pantalla ------------------------------------------------------------
    @property
    def prompt(self):
        confirm = {'J1': 'quédate quieto para pintar', 'J2': 'suelta el joystick y presiona el botón',
                   'J3': 'presiona el botón'}[self.cfg.interaction]
        if self.stage == 'menu':
            how = 'quédate quieto en el centro' if self.cfg.interaction == 'J1' else 'presiona el botón'
            return f'¿Levantado o pegado? Mueve el joystick y {how}'
        if self.stage == 'trazando':
            if self.cfg.mode == 'rehabilitacion':
                return f'Sigue la figura; al terminar, {confirm}'
            what = 'pintar' if self.pen_state == 'pegado' else 'mover el marcador'
            return f'Dibuja hacia dónde quieres {what}; al terminar, {confirm}'
        if self.stage == 'armando':
            return '¡Listo! El robot arranca... (muévete para cancelar)'
        if self.stage == 'esperando':
            return 'El robot está pintando'
        return 'Preparando el robot...'
