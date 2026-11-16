"""Levanta todo el experimento en simulación.

  ros2 launch muralistas_bringup sim_mural.launch.py                               # S, figuras
  ros2 launch muralistas_bringup sim_mural.launch.py modo:=libre
  ros2 launch muralistas_bringup sim_mural.launch.py modo:=rehabilitacion          # S: parar en la esquina
  ros2 launch muralistas_bringup sim_mural.launch.py interaccion:=J2 modo:=libre   # fantasma con teclado
  ros2 launch muralistas_bringup sim_mural.launch.py interaccion:=J1 modo:=rehabilitacion asistencia:=alta
  ros2 launch muralistas_bringup sim_mural.launch.py interaccion:=J3 mando:=pro_controller
  ros2 launch muralistas_bringup sim_mural.launch.py perfil:=/ruta/a/perfil_nino.yaml

Argumentos: viewer, rviz, ui, perfil, interaccion, modo, nivel, asistencia, etapa, mando,
board_frame, auto_startup. En simulación el mando por defecto es el teclado.
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
    sim_time = {'use_sim_time': True}
    nodes = [
        Node(package='muralistas_sim', executable='mural_sim', name='mural_sim', output='screen',
             parameters=[MURAL, os.path.join(CFG, 'sim.yaml'),
                         overrides('mural_sim', {'viewer': is_true('viewer')})]),
        Node(package='muralistas_perception', executable='perception', name='perception', output='screen',
             parameters=[MURAL, os.path.join(CFG, 'perception.yaml'), sim_time]),
        Node(package='muralistas_planning', executable='planner', name='planner', output='screen',
             parameters=[MURAL, sim_time, overrides('planner', {'board_frame': arg('board_frame')})]),
        Node(package='muralistas_control', executable='control', name='control', output='screen',
             parameters=[MURAL, os.path.join(CFG, 'control.yaml'), sim_time,
                         overrides('control', {'board_frame': arg('board_frame'),
                                               'auto_startup': is_true('auto_startup')})]),
    ]
    # La interacción usa el reloj de pared: el barrido y el fantasma van a ritmo humano.
    hri, info = hri_nodes(context, sim_time=False)
    nodes += hri
    if is_true('rviz'):
        nodes.append(Node(package='rviz2', executable='rviz2', name='rviz2', output='log',
                          arguments=['-d', os.path.join(SHARE, 'rviz', 'muralistas.rviz')],
                          parameters=[sim_time]))
    return [info] + nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('viewer', default_value='true', description='Ventana de MuJoCo'),
        DeclareLaunchArgument('rviz', default_value='true', description='RViz con la nube 3D y la tinta'),
        DeclareLaunchArgument('board_frame', default_value='board',
                              description='board (percepción) o board_gt (verdad de la simulación)'),
        DeclareLaunchArgument('auto_startup', default_value='true',
                              description='Pose inicial, barrido de cabeza y calibración del pincel al arrancar'),
        *interaction_arguments(default_mando='teclado'),
        OpaqueFunction(function=_nodes),
    ])
