"""Calibración del niño (C1 a C7): no necesita robot ni simulación.

  ros2 launch muralistas_bringup calibracion.launch.py user_id:=nino_03 macs:=3
  ros2 launch muralistas_bringup calibracion.launch.py mando:=teclado      # probar sin el control

Al terminar escribe datos/perfil_<usuario>_<fecha>.yaml, que se pasa a los launch con perfil:=.
Argumentos: user_id, macs (0 = no se sabe), mando (pro_controller | teclado).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _nodes(context):
    share = get_package_share_directory('muralistas_bringup')
    arg = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731
    keyboard = arg('mando') == 'teclado'
    mando = os.path.join(share, 'config', 'mandos', 'pro_controller.yaml' if keyboard else f'{arg("mando")}.yaml')
    nodes = [] if keyboard else [Node(package='joy', executable='joy_node', name='joy_node', output='screen',
                                      parameters=[mando])]
    nodes.append(Node(package='muralistas_hri', executable='calibracion', name='calibracion', output='screen',
                      parameters=[mando, {'user_id': arg('user_id'), 'macs': int(arg('macs')), 'teclado': keyboard}]))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('user_id', default_value='nino_01'),
        DeclareLaunchArgument('macs', default_value='0', description='Nivel MACS de la terapeuta (1 a 5; 0 = no se sabe)'),
        DeclareLaunchArgument('mando', default_value='pro_controller'),
        OpaqueFunction(function=_nodes),
    ])
