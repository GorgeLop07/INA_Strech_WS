"""Nodo del menú de barrido: conecta la lógica del barrido con ROS.

Entradas:  /hri/switch (presiones ya filtradas), /hri/adult_select (el adulto elige por el niño),
           /planning/busy
Salidas:   /hri/scan_state (para la pantalla), /hri/stroke_request (a la planeación),
           /hri/decision (registro de decisiones)

Parámetros principales (perfil de cada niño): mode (figuras | libre | rehabilitacion), level,
scan_interval, first_item_delay, max_cycles, input (auto = un switch con barrido automático |
pasos = dos switches), free_activation (toggle | hold), shapes, template, voice.
Rehabilitación: rehab.stage (etapa inicial), rehab.assistance, rehab.switch_speed,
rehab.overshoot. Cada intento se califica y se publica en /hri/attempt; la regla de progresión
(5 de 8) cambia la asistencia y la etapa.
"""

import json
import math
import signal
import shutil
import subprocess

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import Bool, Empty, String

from muralistas_common.assistance import preset
from muralistas_common.layout import MuralLayout
from muralistas_common.metrics import Progression
from muralistas_common.templates import TEMPLATE_LABELS, stage_template, template_in_zone
from muralistas_interfaces.msg import Attempt, Decision, ScanState, StrokeRequest, SwitchEvent

from .menu_logic import MenuConfig, ScanningMenu

ACTION_CODES = {
    'shape': StrokeRequest.SHAPE, 'move_to_cell': StrokeRequest.MOVE_TO_CELL,
    'free_start': StrokeRequest.FREE_START, 'free_stop': StrokeRequest.FREE_STOP, 'pen_up': StrokeRequest.PEN_UP,
    'move_to_point': StrokeRequest.GHOST,
}


class ScanningMenuNode(Node):
    def __init__(self):
        super().__init__('scanning_menu')
        p = self.declare_parameter
        layout = MuralLayout.from_dict({k: p(f'mural.{k}', v).value for k, v in MuralLayout().__dict__.items()})
        self.layout = layout
        self.user_id = p('user_id', 'nino_01').value
        self.scan_interval = p('scan_interval', 1.5).value
        self.first_item_delay = p('first_item_delay', 1.5).value   # factor: la primera opción dura más
        self.max_cycles = p('max_cycles', 4).value
        self.input_mode = p('input', 'auto').value
        self.voice = p('voice', False).value and shutil.which('spd-say') is not None
        template = [t for t in p('template', ['']).value if t]   # [''] = sin diseño a copiar
        self.progression = Progression(stage=p('rehab.stage', 0).value,
                                       level=p('rehab.assistance', 'media').value)
        self.fixed_speed = p('rehab.switch_speed', 0.0).value     # > 0: no cambia con la asistencia
        cfg = MenuConfig(
            mode=p('mode', 'figuras').value,
            level=p('level', 3).value,
            shapes=list(p('shapes', ['linea', 'ola', 'circulo', 'zigzag', 'puntos', 'sol']).value),
            rows=layout.rows, cols=layout.cols,
            free_activation=p('free_activation', 'toggle').value,
            template=template,
            overshoot=p('rehab.overshoot', 0.05).value,
            rehab_size=layout.template_size,
        )
        self.attempt_no = 0
        if cfg.mode == 'rehabilitacion':
            self._load_template(cfg)
        self.menu = ScanningMenu(cfg, clock=self._now)
        self.pub_state = self.create_publisher(ScanState, '/hri/scan_state',
                                               QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.pub_request = self.create_publisher(StrokeRequest, '/hri/stroke_request', 10)
        self.pub_decision = self.create_publisher(Decision, '/hri/decision', 50)
        self.pub_attempt = self.create_publisher(Attempt, '/hri/attempt', 10)
        self.tip = None
        self.create_subscription(PointStamped, '/control/tip_board',
                                 lambda m: setattr(self, 'tip', (m.point.x, m.point.y)), 10)
        self.create_subscription(SwitchEvent, '/hri/switch', self._on_switch, 10)
        self.create_subscription(Empty, '/hri/adult_select', self._on_adult, 10)
        self.create_subscription(Bool, '/planning/busy', self._on_busy,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        # No barrer hasta que el robot termine su arranque, ni durante un paro: si no, el barrido
        # da sus vueltas solo y se pausa antes de que el niño pueda hacer algo.
        self.wait_for_robot = p('wait_for_robot', True).value
        self.robot_status = '' if self.wait_for_robot else 'listo'
        self.robot_ready = not self.wait_for_robot
        self.create_subscription(String, '/control/status', self._on_status,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self._next_step = self._now() + self.scan_interval * self.first_item_delay
        self._last_spoken = None
        self.create_timer(0.05, self._tick)
        self._publish()
        self.get_logger().info(f'Menú: modo {cfg.mode}, peldaño {cfg.level}, barrido cada {self.scan_interval} s, '
                               f'entrada "{self.input_mode}"')

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # --- Entradas ---------------------------------------------------------------
    @property
    def _blocked(self):
        return not self.robot_ready or self.robot_status == 'paro'

    def _on_status(self, msg):
        self.robot_status = msg.data
        if not self.robot_ready and msg.data == 'listo':
            self.robot_ready = True
            self.menu.reset_stage()
            self._restart_timer()
            self.get_logger().info('Robot listo: empieza el barrido')
        self._publish()

    def _on_switch(self, msg):
        if self._blocked:
            return
        if self.input_mode == 'pasos' and msg.switch_id == 1:
            if msg.pressed:
                self.menu.advance()
                self._publish()
            return
        if msg.switch_id != 0:
            return
        actions, decisions = self.menu.press('nino') if msg.pressed else self.menu.release('nino')
        self._apply(actions, decisions)

    def _on_adult(self, _):
        if self._blocked:
            return
        actions, decisions = self.menu.press('adulto')
        self._apply(actions, decisions)

    def _on_busy(self, msg):
        n = len(self.menu.attempts)
        actions = self.menu.robot_busy(msg.data, tip=self.tip)
        self._apply(actions or [], [])
        if len(self.menu.attempts) > n:
            self._on_attempt(self.menu.attempts[-1])

    # --- Rehabilitación ------------------------------------------------------------
    def _load_template(self, cfg):
        name = stage_template(self.progression.stage, self.attempt_no, switch=True)
        cfg.rehab_name = name
        cfg.rehab_points = template_in_zone(name, self.layout)
        cfg.switch_speed = self.fixed_speed or preset(self.progression.level)['switch_speed']

    def _on_attempt(self, att):
        stage_used, level_used = self.progression.stage, self.progression.level
        change = self.progression.update(att['stars'], att['completed'] / max(1, att['segments']))
        msg = Attempt()
        msg.stamp = self.get_clock().now().to_msg()
        msg.user_id, msg.interaction = self.user_id, 'S'
        msg.stage, msg.template_name, msg.assistance = stage_used, att['template'], level_used
        msg.stars = att['stars']
        msg.mean_error = float(att['mean_abs_error'])
        msg.coverage = float(att['completed'] / max(1, att['segments']))
        msg.duration = float(att['duration'])
        msg.ldlj = msg.sparc = msg.msj = math.nan
        msg.detail = json.dumps({'errores_paro_m': [round(e, 4) for e in att['stop_errors']],
                                 'direcciones_equivocadas': att['wrong_directions'],
                                 'tiempos_reaccion_s': [round(t, 2) for t in att['reaction_times']],
                                 'tiempos_eleccion_s': [round(t, 2) for t in att['choice_times']],
                                 'no_paro': att['reached_end'], 'velocidad_m_s': self.menu.cfg.switch_speed})
        msg.progression = change or ''
        self.pub_attempt.publish(msg)
        self.get_logger().info(f'Intento {att["template"]}: {att["stars"]} estrellas, error medio de paro '
                               f'{100 * att["mean_abs_error"]:.1f} cm' + (f' -> {change}' if change else ''))
        self.attempt_no += 1
        self._load_template(self.menu.cfg)
        self.menu.set_template(self.menu.cfg.rehab_name, self.menu.cfg.rehab_points)

    # --- Barrido automático -----------------------------------------------------
    def _tick(self):
        n = len(self.menu.attempts)
        actions = self.menu.tick()
        if actions:
            self._apply(actions, [])
        if len(self.menu.attempts) > n:
            self._on_attempt(self.menu.attempts[-1])
        if self._blocked:
            self._restart_timer()
            self._publish_if_changed()
            return
        if self.input_mode != 'auto' or not self.menu.scanning:
            self._publish_if_changed()
            return
        now = self._now()
        if now >= self._next_step:
            self.menu.advance()
            if self.menu.cycles >= self.max_cycles:
                self.menu.pause()
            factor = self.first_item_delay if self.menu.highlighted == 0 else 1.0
            self._next_step = now + self.scan_interval * factor
            self._publish()
        else:
            self._publish_if_changed()

    def _restart_timer(self):
        self._next_step = self._now() + self.scan_interval * self.first_item_delay

    # --- Salidas ---------------------------------------------------------------
    def _apply(self, actions, decisions):
        for d in decisions:
            msg = Decision()
            msg.stamp = self.get_clock().now().to_msg()
            msg.user_id = self.user_id
            msg.level = self.menu.cfg.level
            msg.mode = self.menu.cfg.mode
            msg.stage, msg.choice, msg.source = d.stage, d.choice, d.source
            msg.response_time = float(d.response_time)
            msg.scan_cycles = d.scan_cycles
            self.pub_decision.publish(msg)
        for a in actions:
            req = StrokeRequest()
            req.action = ACTION_CODES[a.kind]
            req.color, req.shape, req.row, req.col = a.color, a.shape, a.row, a.col
            req.direction_deg = float(a.direction_deg)
            req.length, req.speed = float(a.length), float(a.speed)
            if a.kind == 'move_to_point':       # apoyar el pincel en el inicio de la plantilla
                req.points = [Point(x=float(a.point[0]), y=float(a.point[1]))]
                req.pen_down, req.lift_at_end = True, False
            req.source = decisions[0].source if decisions else 'nino'
            self.pub_request.publish(req)
        self._restart_timer()
        self._publish()

    def _snapshot(self):
        m = self.menu
        return (m.stage, tuple(m.options), m.highlighted, m.waiting, m.paused, m.summary(), m.target_hint(),
                self.robot_ready, self.robot_status == 'paro')

    def _publish_if_changed(self):
        if self._snapshot() != getattr(self, '_last_snapshot', None):
            self._publish()

    def _publish(self):
        m = self.menu
        self._last_snapshot = self._snapshot()
        msg = ScanState()
        msg.mode, msg.level, msg.stage, msg.prompt = m.cfg.mode, m.cfg.level, m.stage, m.prompt
        msg.options, msg.labels = list(m.options), list(m.labels)
        msg.highlighted = m.highlighted
        msg.busy = m.waiting or m.stage == 'dibujando'
        msg.selection_summary = m.summary()
        msg.target_hint = m.target_hint()
        if m.cfg.mode == 'rehabilitacion':
            label = TEMPLATE_LABELS.get(m.cfg.rehab_name, m.cfg.rehab_name)
            msg.target_hint = (f'Etapa {self.progression.stage} · {label} · asistencia {self.progression.level}'
                               f' · {m.target_hint()}')
        if self._blocked:
            msg.stage = 'paro' if self.robot_ready else 'preparando'
            msg.prompt = 'Paro: espera a tu maestra' if self.robot_ready else 'Preparando el robot...'
            msg.options, msg.labels, msg.highlighted, msg.busy = [], [], -1, True
        self.pub_state.publish(msg)
        if self.voice:
            spoken = m.labels[m.highlighted] if 0 <= m.highlighted < len(m.labels) else m.prompt
            if spoken != self._last_spoken:
                self._last_spoken = spoken
                subprocess.Popen(['spd-say', '-l', 'es', '-C'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                subprocess.Popen(['spd-say', '-l', 'es', spoken.replace('→', '').strip()],
                                 stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def main():
    rclpy.init()
    node = ScanningMenuNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)   # un segundo Ctrl+C no interrumpe el cierre
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
