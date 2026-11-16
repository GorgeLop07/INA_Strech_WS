"""Registro de la sesión en CSV (un archivo nuevo por sesión en <workspace>/datos/):

  - decisiones_<usuario>_<fecha>.csv: cada decisión y su fuente. Base de la métrica
    % decisiones propias = decisiones del niño / (niño + adulto + automáticas).
  - intentos_<usuario>_<fecha>.csv: cada intento de rehabilitación calificado (/hri/attempt).
  - punta_<usuario>_<fecha>.csv: la punta del marcador en el pizarrón y si está apoyada (10 Hz).
"""

import signal
import csv
import os
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from geometry_msgs.msg import PointStamped
from std_msgs.msg import Bool

from muralistas_interfaces.msg import Attempt, Decision

ATTEMPT_COLUMNS = ['tiempo', 'usuario', 'interaccion', 'etapa', 'plantilla', 'asistencia', 'estrellas',
                   'error_medio_m', 'cobertura', 'duracion_s', 'ldlj', 'sparc', 'msj', 'progresion', 'detalle']
TIP_COLUMNS = ['tiempo', 'u', 'v', 'apoyado']
COLUMNS = ['tiempo', 'usuario', 'peldano', 'modo', 'etapa', 'eleccion', 'fuente', 'tiempo_respuesta_s', 'vueltas_barrido']


class DecisionLoggerNode(Node):
    def __init__(self):
        super().__init__('decision_logger')
        default_dir = Path(os.environ.get('INA_STRECH_WS', Path.cwd())) / 'datos'
        out_dir = Path(self.declare_parameter('output_dir', str(default_dir)).value)
        out_dir.mkdir(parents=True, exist_ok=True)
        self.user = self.declare_parameter('user_id', 'nino_01').value
        stamp = f'{datetime.now():%Y%m%d_%H%M%S}'
        self.path = out_dir / f'decisiones_{self.user}_{stamp}.csv'
        self.file = open(self.path, 'w', newline='')
        self.writer = csv.writer(self.file)
        self.writer.writerow(COLUMNS)
        self.counts = {'nino': 0, 'adulto': 0, 'auto': 0}
        self.create_subscription(Decision, '/hri/decision', self._on_decision, 50)
        self.attempt_file, self.attempt_writer = self._open(out_dir / f'intentos_{self.user}_{stamp}.csv', ATTEMPT_COLUMNS)
        self.tip_file, self.tip_writer = self._open(out_dir / f'punta_{self.user}_{stamp}.csv', TIP_COLUMNS)
        self.pen_down = False
        self.create_subscription(Attempt, '/hri/attempt', self._on_attempt, 10)
        self.create_subscription(PointStamped, '/control/tip_board', self._on_tip, 10)
        self.create_subscription(Bool, '/control/pen_down', lambda m: setattr(self, 'pen_down', m.data), 10)
        self.get_logger().info(f'Registrando decisiones en {self.path}')

    def _on_decision(self, d):
        t = d.stamp.sec + d.stamp.nanosec * 1e-9
        self.writer.writerow([f'{t:.3f}', d.user_id, d.level, d.mode, d.stage, d.choice, d.source,
                              f'{d.response_time:.2f}', d.scan_cycles])
        self.file.flush()
        self.counts[d.source] = self.counts.get(d.source, 0) + 1
        total = sum(self.counts.values())
        own = 100.0 * self.counts.get('nino', 0) / total
        self.get_logger().info(f'{d.stage}: {d.choice} ({d.source}, {d.response_time:.1f} s) | '
                               f'decisiones propias {own:.0f}% ({self.counts.get("nino", 0)}/{total})')

    @staticmethod
    def _open(path, columns):
        f = open(path, 'w', newline='')
        w = csv.writer(f)
        w.writerow(columns)
        return f, w

    def _on_attempt(self, a):
        t = a.stamp.sec + a.stamp.nanosec * 1e-9
        self.attempt_writer.writerow([f'{t:.3f}', a.user_id, a.interaction, a.stage, a.template_name, a.assistance,
                                      a.stars, f'{a.mean_error:.4f}', f'{a.coverage:.3f}', f'{a.duration:.1f}',
                                      f'{a.ldlj:.3f}', f'{a.sparc:.3f}', f'{a.msj:.4g}', a.progression, a.detail])
        self.attempt_file.flush()

    def _on_tip(self, m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        self.tip_writer.writerow([f'{t:.3f}', f'{m.point.x:.4f}', f'{m.point.y:.4f}', int(self.pen_down)])

    def destroy_node(self):
        self.attempt_file.close()
        self.tip_file.close()
        self.file.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = DecisionLoggerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)   # un segundo Ctrl+C no interrumpe el cierre
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
