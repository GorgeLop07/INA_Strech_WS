"""Abre o cierra la pinza del Stretch real (driver en modo trajectory).

El driver pide al menos 2 puntos y que el primero, en t=0, sea la posición actual; por eso no
basta un `ros2 action send_goal` de un solo punto.

Uso (en el robot, con el driver corriendo):
  ros2 run muralistas_control pinza 0.4      # abre
  ros2 run muralistas_control pinza 0.0      # casi cerrada
  ros2 run muralistas_control pinza -0.15    # aprieta (no bajar de -0.3)
"""
import sys
import time

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

JOINT = 'joint_gripper_finger_left'
LIMITES = (-0.3, 0.6)


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 1
    objetivo = float(sys.argv[1])
    if not LIMITES[0] <= objetivo <= LIMITES[1]:
        print(f'{objetivo} está fuera de {LIMITES}: no lo mando')
        return 1
    rclpy.init()
    node = Node('pinza')
    actual = {}
    node.create_subscription(JointState, '/joint_states',
                             lambda m: actual.update(zip(m.name, m.position)), 10)
    deadline = time.monotonic() + 5.0
    while JOINT not in actual and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    if JOINT not in actual:
        print(f'No llega {JOINT} en /joint_states: ¿está corriendo el driver?')
        return 1
    cli = ActionClient(node, FollowJointTrajectory, '/stretch_controller/follow_joint_trajectory')
    if not cli.wait_for_server(timeout_sec=5.0):
        print('No responde /stretch_controller/follow_joint_trajectory')
        return 1
    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = [JOINT]
    goal.trajectory.points = [
        JointTrajectoryPoint(positions=[actual[JOINT]], time_from_start=Duration(sec=0)),
        JointTrajectoryPoint(positions=[objetivo], time_from_start=Duration(sec=2)),
    ]
    print(f'Pinza: {actual[JOINT]:.3f} -> {objetivo:.3f}')
    fut = cli.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)
    handle = fut.result()
    if not handle or not handle.accepted:
        print('El driver rechazó la meta')
        return 1
    res = handle.get_result_async()
    rclpy.spin_until_future_complete(node, res, timeout_sec=10.0)
    r = res.result()
    print('Listo' if r and r.result.error_code == 0 else f'Error: {r.result.error_string if r else "sin respuesta"}')
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
