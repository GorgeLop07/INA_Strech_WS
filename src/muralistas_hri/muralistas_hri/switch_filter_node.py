"""Filtro del switch: convierte presiones crudas en presiones "aceptadas".

En PC son comunes los movimientos involuntarios (espasmos, temblor, distonía). Por eso los
comunicadores con switch tienen dos ajustes que aquí se replican, configurables por niño:
  - tiempo de aceptación: la presión debe durar al menos esto para contar (filtra roces);
  - retardo posterior: tras una presión aceptada se ignoran las siguientes durante este tiempo
    (filtra rebotes y presiones dobles).

Entradas: /hri/switch_raw (de la pantalla o de un puente de hardware) y /joy (interfaces USB de
switch que se presentan como joystick). Salida: /hri/switch.
"""

import signal
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Joy

from muralistas_interfaces.msg import SwitchEvent


class SwitchFilterNode(Node):
    def __init__(self):
        super().__init__('switch_filter')
        self.accept_time = self.declare_parameter('acceptance_time', 0.10).value
        self.refractory = self.declare_parameter('post_acceptance_delay', 0.30).value
        self.joy_buttons = list(self.declare_parameter('joy_buttons', [0, 1]).value)
        self.pub = self.create_publisher(SwitchEvent, '/hri/switch', 10)
        self.create_subscription(SwitchEvent, '/hri/switch_raw', self._on_raw, 10)
        self.create_subscription(Joy, '/joy', self._on_joy, 10)
        self.state = {}            # switch_id -> dict(press_t, accepted)
        self.last_accept = {}
        self.joy_prev = {}
        self.rejected = 0
        self.create_timer(0.01, self._tick)
        self.get_logger().info(f'Switch: aceptación {self.accept_time:.2f} s, retardo {self.refractory:.2f} s')

    def _on_raw(self, msg):
        self._event(msg.switch_id, msg.pressed)

    def _on_joy(self, msg):
        for switch_id, button in enumerate(self.joy_buttons):
            if button < len(msg.buttons):
                pressed = bool(msg.buttons[button])
                if self.joy_prev.get(switch_id) != pressed:
                    self.joy_prev[switch_id] = pressed
                    self._event(switch_id, pressed)

    def _event(self, switch_id, pressed):
        now = time.monotonic()
        st = self.state.get(switch_id)
        if pressed:
            if st is not None:
                return
            if now - self.last_accept.get(switch_id, -1e9) < self.refractory:
                return
            self.state[switch_id] = {'t': now, 'accepted': False}
        else:
            if st is None:
                return
            del self.state[switch_id]
            if st['accepted']:
                self._emit(switch_id, False, now - st['t'])
            else:
                self.rejected += 1
                self.get_logger().debug(f'Presión ignorada (duró {now - st["t"]:.2f} s)')

    def _tick(self):
        now = time.monotonic()
        for switch_id, st in self.state.items():
            if not st['accepted'] and now - st['t'] >= self.accept_time:
                st['accepted'] = True
                self.last_accept[switch_id] = now
                self._emit(switch_id, True, 0.0)

    def _emit(self, switch_id, pressed, duration):
        msg = SwitchEvent()
        msg.stamp = self.get_clock().now().to_msg()
        msg.switch_id = switch_id
        msg.pressed = pressed
        msg.hold_duration = float(duration)
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = SwitchFilterNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)   # un segundo Ctrl+C no interrumpe el cierre
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
