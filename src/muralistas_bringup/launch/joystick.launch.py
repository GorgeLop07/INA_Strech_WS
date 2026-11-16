"""Solo la entrada del mando, para probarla sin robot ni simulación.

  ros2 launch muralistas_bringup joystick.launch.py
  ros2 launch muralistas_bringup joystick.launch.py mando:=pro_controller registrar:=false

Levanta joy_node, entrada_joystick (cursor, botones, registro) y switch_filter (ZR como switch).
Argumentos: mando (archivo en config/mandos/), registrar, user_id.
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
    mando = os.path.join(share, 'config', 'mandos', f'{arg("mando")}.yaml')
    mural = os.path.join(share, 'config', 'mural.yaml')
    registrar = arg('registrar').lower() in ('true', '1', 'si', 'sí')
    return [
        Node(package='joy', executable='joy_node', name='joy_node', output='screen', parameters=[mando]),
        Node(package='muralistas_hri', executable='entrada_joystick', name='entrada_joystick', output='screen',
             parameters=[mural, mando, {'registrar': registrar, 'user_id': arg('user_id')}]),
        Node(package='muralistas_hri', executable='switch_filter', name='switch_filter', output='screen',
             parameters=[mando]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('mando', default_value='pro_controller'),
        DeclareLaunchArgument('registrar', default_value='true'),
        DeclareLaunchArgument('user_id', default_value='prueba'),
        OpaqueFunction(function=_nodes),
    ])
