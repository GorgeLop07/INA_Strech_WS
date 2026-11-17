"""Teleop de la base del Stretch con un control: para acomodar el robot frente al pizarrón.

Joystick izquierdo (arriba/abajo) = avanzar/retroceder; joystick derecho (izq/der) = girar.
Pone el driver en modo navigation (/stretch/cmd_vel) y al salir lo regresa a trajectory.
Si deja de llegar /joy, manda velocidad cero.

Uso (en el robot, con el driver corriendo):
  ros2 run joy joy_node
  ros2 run muralistas_control teleop_base
  ros2 run muralistas_control teleop_base --ros-args -p eje_giro:=3 -p vel_max:=0.1
"""
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Joy
from std_srvs.srv import Trigger


class TeleopBase(Node):
    def __init__(self):
        super().__init__('teleop_base')
        p = self.declare_parameter
        self.eje_avance = p('eje_avance', 1).value     # joystick izquierdo, vertical (arriba = +)
        self.eje_giro = p('eje_giro', 2).value         # joystick derecho, horizontal (izquierda = +)
        self.vel_max = p('vel_max', 0.15).value        # m/s
        self.giro_max = p('giro_max', 0.5).value       # rad/s
        self.zona_muerta = p('zona_muerta', 0.1).value
        self.boton_habilitar = p('boton_habilitar', -1).value   # -1 = sin botón de hombre muerto
        self.pub = self.create_publisher(Twist, '/stretch/cmd_vel', 1)
        self.create_subscription(Joy, '/joy', self._on_joy, 10)
        self.cmd = Twist()
        self.ultimo_joy = 0.0
        self.avisado = False
        self.create_timer(0.05, self._publicar)
        self._cambiar_modo('navigation')
        self.get_logger().info(f'Listo: eje {self.eje_avance} avanza, eje {self.eje_giro} gira. Mueve los joysticks.')

    def _cambiar_modo(self, modo):
        cli = self.create_client(Trigger, f'/switch_to_{modo}_mode')
        if not cli.wait_for_service(timeout_sec=3.0):
            self.get_logger().warn(f'No encontré /switch_to_{modo}_mode: ¿está corriendo el driver?')
            return
        fut = cli.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=3.0)
        self.get_logger().info(f'Driver en modo {modo}')

    def _eje(self, msg, i):
        v = msg.axes[i] if 0 <= i < len(msg.axes) else 0.0
        return 0.0 if abs(v) < self.zona_muerta else v

    def _on_joy(self, msg):
        self.ultimo_joy = time.monotonic()
        if not self.avisado:
            self.get_logger().info(f'/joy: {len(msg.axes)} ejes, {len(msg.buttons)} botones')
            self.avisado = True
        habilitado = self.boton_habilitar < 0 or (
            self.boton_habilitar < len(msg.buttons) and msg.buttons[self.boton_habilitar])
        cmd = Twist()
        if habilitado:
            cmd.linear.x = self.vel_max * self._eje(msg, self.eje_avance)
            cmd.angular.z = self.giro_max * self._eje(msg, self.eje_giro)
        self.cmd = cmd

    def _publicar(self):
        if time.monotonic() - self.ultimo_joy > 0.5:   # se perdió el control: quieto
            self.cmd = Twist()
        self.pub.publish(self.cmd)

    def detener(self):
        for _ in range(3):
            self.pub.publish(Twist())
        self._cambiar_modo('trajectory')


def main():
    # Sin el manejador de rclpy: Ctrl+C llega como KeyboardInterrupt con el contexto vivo,
    # para frenar y regresar el driver a trajectory antes de cerrar.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = TeleopBase()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.detener()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
