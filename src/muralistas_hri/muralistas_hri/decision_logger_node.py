"""Registro de decisiones en CSV: la base de la métrica "% de decisiones propias".

% decisiones propias = decisiones del niño / (niño + adulto + automáticas).
Cada sesión crea un archivo nuevo en <workspace>/datos/.
"""

import signal
import csv
import os
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from muralistas_interfaces.msg import Decision

COLUMNS = ['tiempo', 'usuario', 'peldano', 'modo', 'etapa', 'eleccion', 'fuente', 'tiempo_respuesta_s', 'vueltas_barrido']


class DecisionLoggerNode(Node):
    def __init__(self):
        super().__init__('decision_logger')
        default_dir = Path(os.environ.get('INA_STRECH_WS', Path.cwd())) / 'datos'
        out_dir = Path(self.declare_parameter('output_dir', str(default_dir)).value)
        out_dir.mkdir(parents=True, exist_ok=True)
        self.user = self.declare_parameter('user_id', 'nino_01').value
        self.path = out_dir / f'decisiones_{self.user}_{datetime.now():%Y%m%d_%H%M%S}.csv'
        self.file = open(self.path, 'w', newline='')
        self.writer = csv.writer(self.file)
        self.writer.writerow(COLUMNS)
        self.counts = {'nino': 0, 'adulto': 0, 'auto': 0}
        self.create_subscription(Decision, '/hri/decision', self._on_decision, 50)
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

    def destroy_node(self):
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
