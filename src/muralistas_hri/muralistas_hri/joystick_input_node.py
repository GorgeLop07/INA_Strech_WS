"""Entrada del joystick: /joy -> cursor del fantasma, botones y registro crudo.

Lee el mando con el mapeo de config/mandos/<mando>.yaml y publica:
  - /hri/cursor (PointStamped, marco 'zona_trabajo'): el cursor en metros, origen en la
    esquina inferior izquierda de la zona de trabajo;
  - /hri/joystick (Vector3Stamped): la deflexión ya filtrada (menú levantado / pegado, pantalla);
  - /hri/joystick_crudo (Vector3Stamped): la deflexión cruda, para las métricas de suavidad;
  - /hri/confirmar y /hri/cancelar (Empty): al presionar A o B;
  - /muralistas/stop (Bool true): al presionar el botón de paro de respaldo.
El botón que hace de switch (ZR) no pasa por aquí: lo lee switch_filter de /joy, con su
tiempo de aceptación y retardo (parámetro joy_buttons).

/hri/cursor_colocar (PointStamped, marco 'zona_trabajo') mueve el cursor a donde está la punta.
/hri/asistencia (String: alta | media | baja) cambia en vivo la velocidad máxima y el filtro.

Registro: un CSV por sesión en <workspace>/datos/ con la señal cruda y la filtrada a la
frecuencia del lazo (100 Hz). La suavidad (LDLJ, MSJ) se calcula sobre la columna cruda.
"""

import csv
import os
import signal
import time
from datetime import datetime
from pathlib import Path

import rclpy
from geometry_msgs.msg import PointStamped, Vector3Stamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, Empty

from std_msgs.msg import String

from muralistas_common.assistance import preset

from .joystick_filter import JoystickCursor, JoystickParams, OneEuroFilter

COLUMNS = ['tiempo', 'x_crudo', 'y_crudo', 'x_filtrado', 'y_filtrado', 'u', 'v',
           'confirmar', 'cancelar', 'switch']


class JoystickInputNode(Node):
    def __init__(self):
        super().__init__('entrada_joystick')
        p = self.declare_parameter
        self.ax_x = p('mando.ejes.x', 0).value
        self.inv_x = p('mando.ejes.x_invertido', True).value
        self.ax_y = p('mando.ejes.y', 1).value
        self.inv_y = p('mando.ejes.y_invertido', False).value
        self.btn = {
            'confirmar': p('mando.botones.confirmar', 1).value,
            'cancelar': p('mando.botones.cancelar', 0).value,
            'switch': p('mando.botones.switch', 8).value,
            'paro': p('mando.botones.paro', 10).value,
        }
        params = JoystickParams(
            deadzone=p('zona_muerta', 0.10).value,
            gains=list(p('ganancias', [1.0] * 8).value),
            min_cutoff=p('one_euro.min_cutoff', 0.8).value,
            beta=p('one_euro.beta', 0.5).value,
            max_speed=p('velocidad_max', 0.18).value,
            max_accel=p('aceleracion_max', 0.6).value,
            area=(p('mural.work_width', 0.60).value, p('mural.work_height', 0.45).value),   # de mural.yaml
        )
        if len(params.gains) != 8:
            raise ValueError('ganancias necesita 8 valores (derecha, arriba-derecha, arriba, ...)')
        self.cursor = JoystickCursor(params)
        rate = p('frecuencia', 100.0).value

        self.pub_cursor = self.create_publisher(PointStamped, '/hri/cursor', 10)
        self.pub_joy = self.create_publisher(Vector3Stamped, '/hri/joystick', 10)
        self.pub_raw = self.create_publisher(Vector3Stamped, '/hri/joystick_crudo', 10)
        self.create_subscription(String, '/hri/asistencia', self._on_assistance, 10)
        self.pub_confirm = self.create_publisher(Empty, '/hri/confirmar', 10)
        self.pub_cancel = self.create_publisher(Empty, '/hri/cancelar', 10)
        self.pub_stop = self.create_publisher(Bool, '/muralistas/stop', 10)
        self.create_subscription(Joy, '/joy', self._on_joy, 50)
        self.create_subscription(PointStamped, '/hri/cursor_colocar', self._on_place, 10)

        self.raw = (0.0, 0.0)
        self.buttons = {k: False for k in self.btn}
        self.last_t = None
        self.joy_seen = False

        self.writer = self.file = None
        if p('registrar', True).value:
            default_dir = Path(os.environ.get('INA_STRECH_WS', Path.cwd())) / 'datos'
            out_dir = Path(p('output_dir', str(default_dir)).value)
            out_dir.mkdir(parents=True, exist_ok=True)
            user = p('user_id', 'nino_01').value
            self.path = out_dir / f'joystick_{user}_{datetime.now():%Y%m%d_%H%M%S}.csv'
            self.file = open(self.path, 'w', newline='')
            self.writer = csv.writer(self.file)
            self.writer.writerow(COLUMNS)
            self.get_logger().info(f'Registrando el joystick en {self.path}')

        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            f'Joystick: zona muerta {params.deadzone:.2f}, One Euro {params.min_cutoff:.2f} Hz / beta {params.beta:.2f}, '
            f'velocidad máx {params.max_speed:.2f} m/s, zona {params.area[0]:.2f} x {params.area[1]:.2f} m')

    def _axis(self, msg, index, invert):
        if index >= len(msg.axes):
            return 0.0
        return -msg.axes[index] if invert else msg.axes[index]

    def _on_joy(self, msg):
        if not self.joy_seen:
            self.joy_seen = True
            self.get_logger().info(f'Llegó /joy: {len(msg.axes)} ejes, {len(msg.buttons)} botones')
        self.raw = (self._axis(msg, self.ax_x, self.inv_x), self._axis(msg, self.ax_y, self.inv_y))
        for name, index in self.btn.items():
            pressed = index < len(msg.buttons) and bool(msg.buttons[index])
            if pressed and not self.buttons[name]:
                self._on_button(name)
            self.buttons[name] = pressed

    def _on_button(self, name):
        if name == 'confirmar':
            self.pub_confirm.publish(Empty())
        elif name == 'cancelar':
            self.pub_cancel.publish(Empty())
        elif name == 'paro':
            self.pub_stop.publish(Bool(data=True))
            self.get_logger().warn('PARO desde el botón del mando')

    def _on_place(self, msg):
        self.cursor.place(msg.point.x, msg.point.y)

    def _on_assistance(self, msg):
        try:
            pr = preset(msg.data)
        except KeyError as exc:
            self.get_logger().warn(str(exc))
            return
        p = self.cursor.p
        p.max_speed, p.min_cutoff = pr['max_speed'], pr['min_cutoff']
        self.cursor.fx = OneEuroFilter(p.min_cutoff, p.beta)
        self.cursor.fy = OneEuroFilter(p.min_cutoff, p.beta)
        self.get_logger().info(f'Asistencia {msg.data}: velocidad máx {p.max_speed:.2f} m/s, '
                               f'One Euro {p.min_cutoff:.2f} Hz')

    def _tick(self):
        now = time.monotonic()
        dt = 0.0 if self.last_t is None else now - self.last_t
        self.last_t = now
        u, v = self.cursor.update(self.raw[0], self.raw[1], dt)
        fx, fy = self.cursor.filtered

        stamp = self.get_clock().now().to_msg()
        cur = PointStamped()
        cur.header.stamp, cur.header.frame_id = stamp, 'zona_trabajo'
        cur.point.x, cur.point.y = float(u), float(v)
        self.pub_cursor.publish(cur)
        joy = Vector3Stamped()
        joy.header.stamp = stamp
        joy.vector.x, joy.vector.y = float(fx), float(fy)
        self.pub_joy.publish(joy)
        raw = Vector3Stamped()
        raw.header.stamp = stamp
        raw.vector.x, raw.vector.y = float(self.raw[0]), float(self.raw[1])
        self.pub_raw.publish(raw)

        if self.writer is not None and self.joy_seen:
            t = stamp.sec + stamp.nanosec * 1e-9
            self.writer.writerow([f'{t:.3f}', f'{self.raw[0]:.4f}', f'{self.raw[1]:.4f}', f'{fx:.4f}', f'{fy:.4f}',
                                  f'{u:.4f}', f'{v:.4f}', int(self.buttons['confirmar']),
                                  int(self.buttons['cancelar']), int(self.buttons['switch'])])
            self.rows = getattr(self, 'rows', 0) + 1
            if self.rows % 100 == 0:      # una vez por segundo, por si el proceso muere
                self.file.flush()

    def destroy_node(self):
        if self.file is not None:
            self.file.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = JoystickInputNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)   # un segundo Ctrl+C no interrumpe el cierre
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
