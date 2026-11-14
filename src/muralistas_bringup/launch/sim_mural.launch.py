"""Levanta todo el experimento en simulación.

  ros2 launch muralistas_bringup sim_mural.launch.py
  ros2 launch muralistas_bringup sim_mural.launch.py modo:=libre
  ros2 launch muralistas_bringup sim_mural.launch.py nivel:=1 viewer:=false
  ros2 launch muralistas_bringup sim_mural.launch.py perfil:=/ruta/a/otro_nino.yaml

Argumentos: viewer, rviz, ui, perfil, modo, nivel, board_frame, auto_startup.
"""

import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _overrides(node_name, params):
    """YAML con el nombre del nodo para los argumentos del launch.

    Un diccionario en `parameters=[...]` se pasa como comodín (/**), y en ROS 2 lo que un YAML
    pone bajo el nombre del nodo le gana al comodín sin importar el orden. Así, `modo:=libre`
    no sobrescribía el perfil. Escribiéndolo bajo el nombre del nodo y al final, sí gana.
    """
    f = tempfile.NamedTemporaryFile('w', suffix=f'_{node_name}.yaml', delete=False)
    yaml.safe_dump({node_name: {'ros__parameters': params}}, f)
    f.close()
    return f.name


def _nodes(context):
    share = get_package_share_directory('muralistas_bringup')
    cfg = os.path.join(share, 'config')
    mural = os.path.join(cfg, 'mural.yaml')
    arg = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731
    perfil = arg('perfil') or os.path.join(cfg, 'perfiles', 'nino_ejemplo.yaml')
    is_true = lambda name: arg(name).lower() in ('true', '1', 'si', 'sí')  # noqa: E731
    sim_time = {'use_sim_time': True}

    menu_overrides = {}
    if arg('modo'):
        menu_overrides['mode'] = arg('modo')
    if arg('nivel'):
        menu_overrides['level'] = int(arg('nivel'))

    nodes = [
        Node(package='muralistas_sim', executable='mural_sim', name='mural_sim', output='screen',
             parameters=[mural, os.path.join(cfg, 'sim.yaml'),
                         _overrides('mural_sim', {'viewer': is_true('viewer')})]),
        Node(package='muralistas_perception', executable='perception', name='perception', output='screen',
             parameters=[mural, os.path.join(cfg, 'perception.yaml'), sim_time]),
        Node(package='muralistas_planning', executable='planner', name='planner', output='screen',
             parameters=[mural, sim_time, _overrides('planner', {'board_frame': arg('board_frame')})]),
        Node(package='muralistas_control', executable='control', name='control', output='screen',
             parameters=[os.path.join(cfg, 'control.yaml'), sim_time,
                         _overrides('control', {'board_frame': arg('board_frame'),
                                                'auto_startup': is_true('auto_startup')})]),
        # La interacción usa el reloj de pared: el barrido es a ritmo humano, no de la simulación.
        Node(package='muralistas_hri', executable='switch_filter', name='switch_filter', output='screen',
             parameters=[perfil]),
        Node(package='muralistas_hri', executable='scanning_menu', name='scanning_menu', output='screen',
             parameters=[mural, perfil] + ([_overrides('scanning_menu', menu_overrides)] if menu_overrides else [])),
        Node(package='muralistas_hri', executable='decision_logger', name='decision_logger', output='screen',
             parameters=[perfil]),
    ]
    if is_true('ui'):
        nodes.append(Node(package='muralistas_hri', executable='scanning_ui', name='scanning_ui',
                          output='screen', parameters=[mural]))
    if is_true('rviz'):
        nodes.append(Node(package='rviz2', executable='rviz2', name='rviz2', output='log',
                          arguments=['-d', os.path.join(share, 'rviz', 'muralistas.rviz')],
                          parameters=[sim_time]))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('viewer', default_value='true', description='Ventana de MuJoCo'),
        DeclareLaunchArgument('rviz', default_value='true', description='RViz con la nube 3D y la tinta'),
        DeclareLaunchArgument('ui', default_value='true', description='Pantalla de pictogramas del niño'),
        DeclareLaunchArgument('perfil', default_value='', description='YAML con el perfil del niño'),
        DeclareLaunchArgument('modo', default_value='', description='figuras | libre (sobrescribe el perfil)'),
        DeclareLaunchArgument('nivel', default_value='', description='1 a 4 (sobrescribe el perfil)'),
        DeclareLaunchArgument('board_frame', default_value='board',
                              description='board (percepción) o board_gt (verdad de la simulación)'),
        DeclareLaunchArgument('auto_startup', default_value='true',
                              description='Pose inicial, barrido de cabeza y calibración del pincel al arrancar'),
        OpaqueFunction(function=_nodes),
    ])
