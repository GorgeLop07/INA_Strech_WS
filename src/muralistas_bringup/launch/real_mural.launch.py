"""El experimento con el Stretch real.

Antes, en el robot (ver PRUEBAS.md):
  ros2 launch stretch_core stretch_driver.launch.py mode:=trajectory broadcast_odom_tf:=True
  ros2 launch stretch_core d435i_high_resolution.launch.py

Luego, con el mismo ROS_DOMAIN_ID en el robot y en la laptop:
  # en el robot: percepción, planeación y control
  ros2 launch muralistas_bringup real_mural.launch.py parte:=robot
  # en la laptop: mando, interacción, pantalla del niño y registro
  ros2 launch muralistas_bringup real_mural.launch.py parte:=laptop interaccion:=J2 modo:=libre

parte:=todo levanta todo en la misma máquina. Además de los de la interacción (perfil, interaccion,
modo, nivel, asistencia, etapa, mando, ui), acepta board_frame, auto_startup y rviz.
"""

import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from muralistas_comun import CFG, MURAL, SHARE, hri_nodes, interaction_arguments, overrides  # noqa: E402


def _nodes(context):
    arg = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731
    is_true = lambda name: arg(name).lower() in ('true', '1', 'si', 'sí')  # noqa: E731
    part = arg('parte')
    if part not in ('robot', 'laptop', 'todo'):
        raise RuntimeError(f'parte debe ser robot, laptop o todo, no "{part}"')
    nodes = []
    if part in ('robot', 'todo'):
        nodes += [
            Node(package='muralistas_perception', executable='perception', name='perception', output='screen',
                 parameters=[MURAL, os.path.join(CFG, 'perception.yaml'), os.path.join(CFG, 'perception_real.yaml')]),
            Node(package='muralistas_planning', executable='planner', name='planner', output='screen',
                 parameters=[MURAL, overrides('planner', {'board_frame': arg('board_frame')})]),
            Node(package='muralistas_control', executable='control', name='control', output='screen',
                 parameters=[os.path.join(CFG, 'control.yaml'), os.path.join(CFG, 'control_real.yaml'),
                             overrides('control', {'board_frame': arg('board_frame'),
                                                   'auto_startup': is_true('auto_startup')})]),
        ]
    if part in ('laptop', 'todo'):
        hri, info = hri_nodes(context, sim_time=False)
        nodes = [info] + nodes + hri
        if is_true('rviz'):
            nodes.append(Node(package='rviz2', executable='rviz2', name='rviz2', output='log',
                              arguments=['-d', os.path.join(SHARE, 'rviz', 'muralistas.rviz')]))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('parte', default_value='todo', description='robot | laptop | todo'),
        DeclareLaunchArgument('board_frame', default_value='board'),
        DeclareLaunchArgument('auto_startup', default_value='true',
                              description='Pose inicial, barrido de cabeza y calibración del marcador al arrancar'),
        DeclareLaunchArgument('rviz', default_value='false'),
        *interaction_arguments(default_mando='pro_controller'),
        OpaqueFunction(function=_nodes),
    ])
