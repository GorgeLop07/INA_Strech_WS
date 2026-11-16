"""Nodo del fantasma: la interacción con joystick (modos J1, J2, J3) conectada con ROS.

Entradas:  /hri/cursor (marco 'zona_trabajo'), /hri/joystick (filtrada), /hri/joystick_crudo,
           /hri/switch (ZR o el switch del niño, ya filtrado), /hri/confirmar (A), /hri/cancelar (B),
           /planning/busy, /control/tip_board, /control/status
Salidas:   /hri/stroke_request (GHOST), /hri/cursor_colocar, /hri/ghost_state (pantalla),
           /hri/decision, /hri/attempt (rehabilitación), /hri/asistencia (solo cuando la
           progresión cambia el nivel; al arrancar manda el perfil, que ya trae lo calibrado)
Registro:  datos/trazos_<usuario>_<fecha>.jsonl con cada fantasma confirmado (crudo y mostrado),
           y al cerrar datos/progreso_<usuario>_<fecha>.yaml con la etapa y la asistencia sugeridas.
"""

import json
import math
import os
import signal
from datetime import datetime
from pathlib import Path

import rclpy
import yaml
from geometry_msgs.msg import Point, PointStamped, Vector3Stamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, Empty, String

from muralistas_common.assistance import preset
from muralistas_common.colors import MARKER_COLOR
from muralistas_common.layout import MuralLayout
from muralistas_common.metrics import Progression
from muralistas_common.templates import TEMPLATE_LABELS, stage_template, template_in_zone
from muralistas_interfaces.msg import Attempt, Decision, GhostState, StrokeRequest, SwitchEvent

from .ghost_logic import GhostConfig, GhostInteraction


def _pts(points):
    return [Point(x=float(u), y=float(v), z=0.0) for u, v in points]


class GhostNode(Node):
    def __init__(self):
        super().__init__('fantasma')
        p = self.declare_parameter
        self.layout = MuralLayout.from_dict({k: p(f'mural.{k}', v).value for k, v in MuralLayout().__dict__.items()})
        self.user_id = p('user_id', 'nino_01').value
        level = p('asistencia', 'media').value
        self.progression = Progression(stage=p('rehab.etapa', 0).value, level=level)
        self.dwell_override = p('t_quieto', 0.0).value       # > 0: fijo, no sigue a la asistencia
        self.rate = p('frecuencia', 50.0).value
        pr = preset(level)
        cfg = GhostConfig(
            interaction=p('interaccion', 'J2').value,
            mode=p('modo', 'libre').value,
            dwell=self.dwell_override or pr['dwell'],
            min_length=p('largo_minimo', 0.03).value,
            start_delay=p('retraso_arranque', 1.0).value,
            long_press=p('presion_larga', 1.0).value,
            magnet=pr['magnet'],
            template_size=self.layout.template_size,
            rate=self.rate,
        )
        if cfg.interaction not in ('J1', 'J2', 'J3'):
            raise ValueError(f'interaccion debe ser J1, J2 o J3 (el modo S usa el menú de barrido): {cfg.interaction}')
        if cfg.mode not in ('libre', 'rehabilitacion'):
            raise ValueError(f'modo debe ser libre o rehabilitacion: {cfg.mode}')
        self.attempt_no = 0
        if cfg.mode == 'rehabilitacion':
            self._load_template(cfg)
        self.logic = GhostInteraction(cfg)

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_request = self.create_publisher(StrokeRequest, '/hri/stroke_request', 10)
        self.pub_place = self.create_publisher(PointStamped, '/hri/cursor_colocar', 10)
        self.pub_state = self.create_publisher(GhostState, '/hri/ghost_state', latched)
        self.pub_decision = self.create_publisher(Decision, '/hri/decision', 50)
        self.pub_attempt = self.create_publisher(Attempt, '/hri/attempt', 10)
        self.pub_assist = self.create_publisher(String, '/hri/asistencia', latched)

        self.cursor = None
        self.deflection, self.raw = (0.0, 0.0), (0.0, 0.0)
        self.tip = None
        self.status = ''
        self.robot_ready = not p('esperar_robot', True).value
        self.started = False
        self.create_subscription(PointStamped, '/hri/cursor', self._on_cursor, 10)
        self.create_subscription(Vector3Stamped, '/hri/joystick',
                                 lambda m: setattr(self, 'deflection', (m.vector.x, m.vector.y)), 10)
        self.create_subscription(Vector3Stamped, '/hri/joystick_crudo',
                                 lambda m: setattr(self, 'raw', (m.vector.x, m.vector.y)), 10)
        self.create_subscription(SwitchEvent, '/hri/switch', self._on_switch, 10)
        self.create_subscription(Empty, '/hri/confirmar', lambda _: self._apply(self.logic.confirm(self._now())), 10)
        self.create_subscription(Empty, '/hri/cancelar', lambda _: self._apply(self.logic.cancel(self._now())), 10)
        self.create_subscription(Bool, '/planning/busy', self._on_busy, latched)
        # La punta, recortada a la zona de trabajo: ahí empieza el fantasma (al arrancar puede estar fuera)
        self.create_subscription(PointStamped, '/control/tip_board',
                                 lambda m: setattr(self, 'tip', self.layout.clamp_to_drawable(m.point.x, m.point.y)), 10)
        self.create_subscription(String, '/control/status', self._on_status, latched)

        default_dir = Path(os.environ.get('INA_STRECH_WS', Path.cwd())) / 'datos'
        self.out_dir = Path(p('output_dir', str(default_dir)).value)
        self.out_dir.mkdir(parents=True, exist_ok=True)
        stamp = f'{datetime.now():%Y%m%d_%H%M%S}'
        self.trazos = open(self.out_dir / f'trazos_{self.user_id}_{stamp}.jsonl', 'a')
        self.progress_path = self.out_dir / f'progreso_{self.user_id}_{stamp}.yaml'

        self.create_timer(1.0 / self.rate, self._tick)
        self.get_logger().info(f'Fantasma: {cfg.interaction}, modo {cfg.mode}, asistencia {level} '
                               f'(t_quieto {cfg.dwell:.1f} s, imán {cfg.magnet:.2f})')

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # --- Rehabilitación ------------------------------------------------------------
    def _load_template(self, cfg):
        name = stage_template(self.progression.stage, self.attempt_no)
        cfg.template_name, cfg.template = name, template_in_zone(name, self.layout)

    def _on_attempt(self, att):
        stage_used, level_used = self.progression.stage, self.progression.level
        change = self.progression.update(att['stars'], att['coverage'])
        msg = Attempt()
        msg.stamp = self.get_clock().now().to_msg()
        msg.user_id, msg.interaction = self.user_id, self.logic.cfg.interaction
        msg.stage, msg.template_name, msg.assistance = stage_used, att['template'], level_used
        msg.stars = att['stars']
        msg.mean_error = float(att['mean_error'])
        msg.coverage = float(att['coverage'])
        msg.duration = float(att['duration'])
        msg.ldlj, msg.sparc, msg.msj = float(att['ldlj']), float(att['sparc']), float(att['msj'])
        msg.detail = json.dumps({'iman': att['magnet']})
        msg.progression = change or ''
        self.pub_attempt.publish(msg)
        self._log_trace('intento', att, extra={'etapa': stage_used, 'asistencia': level_used})
        self.get_logger().info(f'Intento {att["template"]}: {att["stars"]} estrellas, error {100 * att["mean_error"]:.1f} cm, '
                               f'cobertura {100 * att["coverage"]:.0f}%, LDLJ {att["ldlj"]:.2f}'
                               + (f' -> {change}' if change else ''))
        self.attempt_no += 1
        cfg = self.logic.cfg
        if change:
            pr = preset(self.progression.level)
            cfg.magnet = pr['magnet']
            cfg.dwell = self.dwell_override or pr['dwell']
            self.pub_assist.publish(String(data=self.progression.level))
        self._load_template(cfg)

    # --- Entradas ---------------------------------------------------------------
    def _on_cursor(self, msg):
        self.cursor = self.layout.work_to_board(msg.point.x, msg.point.y)

    def _on_switch(self, msg):
        if msg.switch_id == 0 and self._ready:
            self._apply(self.logic.switch(msg.pressed, msg.hold_duration, self._now()))

    def _on_busy(self, msg):
        self._apply(self.logic.robot_busy(msg.data, self.tip, self._now()))

    def _on_status(self, msg):
        self.status = msg.data
        if msg.data == 'listo':
            self.robot_ready = True

    @property
    def _ready(self):
        """El robot terminó su arranque (dijo "listo" al menos una vez) y no hay paro."""
        return self.robot_ready and self.status != 'paro'

    def _tick(self):
        now = self._now()
        if not self.started:
            if self._ready and self.tip is not None:
                self.started = True
                self._apply(self.logic.start(self.tip, now))
            self._publish_state()
            return
        if self.cursor is not None and self.status != 'paro':
            self._apply(self.logic.update(now, self.cursor, self.deflection, self.raw))
        self._publish_state()

    # --- Salidas ---------------------------------------------------------------
    def _apply(self, actions):
        for a in actions:
            kind = a[0]
            if kind == 'place_cursor':
                x, y = self.layout.board_to_work(*a[1])
                msg = PointStamped()
                msg.header.stamp, msg.header.frame_id = self.get_clock().now().to_msg(), 'zona_trabajo'
                msg.point.x, msg.point.y = float(x), float(y)
                self.pub_place.publish(msg)
                self.cursor = tuple(a[1])
            elif kind == 'ghost':
                _, points, pen_down, lift = a
                req = StrokeRequest()
                req.action = StrokeRequest.GHOST
                req.color = MARKER_COLOR
                req.points = _pts(points)
                req.pen_down, req.lift_at_end = bool(pen_down), bool(lift)
                req.source = 'nino'
                self.pub_request.publish(req)
                if pen_down and self.logic.cfg.mode == 'libre':
                    self._log_trace('fantasma', {'ghost': points, 'raw_path': self.logic.raw_path})
            elif kind == 'decision':
                _, stage, choice, rt, source = a
                msg = Decision()
                msg.stamp = self.get_clock().now().to_msg()
                msg.user_id, msg.mode, msg.stage, msg.choice, msg.source = \
                    self.user_id, self.logic.cfg.mode, stage, choice, source
                msg.response_time = float(rt)
                self.pub_decision.publish(msg)
            elif kind == 'attempt':
                self._on_attempt(a[1])

    def _log_trace(self, kind, data, extra=None):
        row = {'tiempo': self._now(), 'tipo': kind, 'interaccion': self.logic.cfg.interaction,
               'modo': self.logic.cfg.mode, 'plantilla': self.logic.cfg.template_name,
               'fantasma': [[round(u, 4), round(v, 4)] for u, v in data.get('ghost', [])],
               'crudo': [[round(u, 4), round(v, 4)] for u, v in data.get('raw_path', [])]}
        for k in ('stars', 'mean_error', 'coverage', 'duration', 'ldlj', 'sparc', 'msj', 'magnet'):
            if k in data:
                v = data[k]
                row[k] = None if isinstance(v, float) and math.isnan(v) else v
        row.update(extra or {})
        self.trazos.write(json.dumps(row) + '\n')
        self.trazos.flush()

    def _publish_state(self):
        lg = self.logic
        msg = GhostState()
        msg.interaction, msg.mode = lg.cfg.interaction, lg.cfg.mode
        msg.stage = lg.stage if self.status != 'paro' else 'paro'
        msg.prompt = lg.prompt if self.status != 'paro' else 'Paro: espera a tu maestra'
        if not self.started:
            msg.stage, msg.prompt = 'preparando', 'Preparando el robot...'
        msg.menu_highlight = lg.highlight if lg.stage == 'menu' else ''
        msg.pen_down = lg.pen_state == 'pegado'
        if lg.cursor:
            msg.cursor = Point(x=float(lg.cursor[0]), y=float(lg.cursor[1]))
        msg.ghost = _pts(lg.ghost)
        msg.template_points = _pts(lg.cfg.template) if lg.cfg.mode == 'rehabilitacion' else []
        msg.dwell_progress = float(lg.dwell_progress)
        att = lg.last_attempt
        msg.stars = att['stars'] if att else -1
        if att:
            msg.score_text = f'error {100 * att["mean_error"]:.1f} cm · cobertura {100 * att["coverage"]:.0f}%'
        if lg.cfg.mode == 'rehabilitacion':
            label = TEMPLATE_LABELS.get(lg.cfg.template_name, lg.cfg.template_name)
            msg.level_text = f'Etapa {self.progression.stage} · {label} · asistencia {self.progression.level}'
        else:
            msg.level_text = f'Modo libre · {lg.cfg.interaction}'
        self.pub_state.publish(msg)

    def destroy_node(self):
        self.trazos.close()
        with open(self.progress_path, 'w') as f:
            yaml.safe_dump({'usuario': self.user_id, 'interaccion': self.logic.cfg.interaction,
                            'modo': self.logic.cfg.mode, 'intentos': self.attempt_no,
                            'siguiente_sesion': {'etapa': self.progression.stage,
                                                 'asistencia': self.progression.level}},
                           f, allow_unicode=True, sort_keys=False)
        self.get_logger().info(f'Progreso guardado en {self.progress_path}')
        super().destroy_node()


def main():
    rclpy.init()
    node = GhostNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)   # un segundo Ctrl+C no interrumpe el cierre
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
