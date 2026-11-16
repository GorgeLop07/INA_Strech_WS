"""Piezas comunes de los launch: el perfil del niño y los nodos de interacción.

Lo usan sim_mural.launch.py y real_mural.launch.py, así la interacción es la misma en
simulación y en el robot real.
"""

import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

SHARE = get_package_share_directory('muralistas_bringup')
CFG = os.path.join(SHARE, 'config')
MURAL = os.path.join(CFG, 'mural.yaml')

# (valor de la asistencia) -> parámetros, igual que muralistas_common.assistance
PRESETS = {
    'alta': {'max_speed': 0.10, 'min_cutoff': 0.3},
    'media': {'max_speed': 0.18, 'min_cutoff': 0.8},
    'baja': {'max_speed': 0.25, 'min_cutoff': 1.5},
}


def overrides(node_name, params):
    """YAML con el nombre del nodo para los argumentos del launch.

    Un diccionario en `parameters=[...]` se pasa como comodín (/**), y en ROS 2 lo que un YAML
    pone bajo el nombre del nodo le gana al comodín sin importar el orden. Así, `modo:=libre`
    no sobrescribía el perfil. Escribiéndolo bajo el nombre del nodo y al final, sí gana.
    """
    nested = {}
    for key, value in params.items():          # 'rehab.assistance' -> {'rehab': {'assistance': ...}}
        *parents, leaf = key.split('.')
        d = nested
        for part in parents:
            d = d.setdefault(part, {})
        d[leaf] = value
    f = tempfile.NamedTemporaryFile('w', suffix=f'_{node_name}.yaml', delete=False)
    yaml.safe_dump({node_name: {'ros__parameters': nested}}, f)
    f.close()
    return f.name


def interaction_arguments(default_mando):
    return [
        DeclareLaunchArgument('perfil', default_value='', description='YAML con el perfil del niño'),
        DeclareLaunchArgument('interaccion', default_value='', description='S | J1 | J2 | J3 (sobrescribe el perfil)'),
        DeclareLaunchArgument('modo', default_value='', description='figuras | libre | rehabilitacion'),
        DeclareLaunchArgument('nivel', default_value='', description='peldaño 1 a 4 del modo figuras (S)'),
        DeclareLaunchArgument('asistencia', default_value='', description='alta | media | baja'),
        DeclareLaunchArgument('etapa', default_value='', description='rehabilitación: 0 línea, 1 cuadrado, 2 círculo, 3 casa'),
        DeclareLaunchArgument('mando', default_value=default_mando,
                              description='pro_controller (control real) | teclado (flechas, espacio, Enter, Retroceso)'),
        DeclareLaunchArgument('ui', default_value='true', description='Pantalla del niño'),
    ]


def hri_nodes(context, sim_time=False):
    """Nodos de interacción según el perfil y los argumentos. Devuelve (nodos, resumen)."""
    arg = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731
    perfil = arg('perfil') or os.path.join(CFG, 'perfiles', 'nino_ejemplo.yaml')
    with open(perfil) as f:
        prof = (yaml.safe_load(f) or {}).get('perfil', {}).get('ros__parameters', {})
    interaction = arg('interaccion') or prof.get('interaccion', 'S')
    mode = arg('modo') or prof.get('modo', 'figuras')
    level = arg('asistencia') or prof.get('asistencia', 'media')
    if interaction not in ('S', 'J1', 'J2', 'J3'):
        raise RuntimeError(f'interaccion debe ser S, J1, J2 o J3, no "{interaction}"')
    if level not in PRESETS:
        raise RuntimeError(f'asistencia debe ser alta, media o baja, no "{level}"')
    if interaction != 'S' and mode == 'figuras':
        mode = 'libre'          # las figuras preestablecidas son del barrido; con joystick, el fantasma
    mando = arg('mando')
    keyboard = mando == 'teclado'
    mando_yaml = os.path.join(CFG, 'mandos', 'pro_controller.yaml' if keyboard else f'{mando}.yaml')
    show_ui = arg('ui').lower() in ('true', '1', 'si', 'sí')
    extra = [{'use_sim_time': True}] if sim_time else []

    nodes = []
    if not keyboard:
        nodes.append(Node(package='joy', executable='joy_node', name='joy_node', output='screen',
                          parameters=[mando_yaml]))
    # El mapeo del mando dice qué botón es el switch (ZR); con teclado, el espacio publica ese mismo botón
    switch_params = [perfil, mando_yaml]
    nodes.append(Node(package='muralistas_hri', executable='switch_filter', name='switch_filter', output='screen',
                      parameters=switch_params))
    nodes.append(Node(package='muralistas_hri', executable='decision_logger', name='decision_logger',
                      output='screen', parameters=[perfil]))

    if interaction == 'S':
        menu = {'mode': mode, 'rehab.assistance': level}
        if arg('nivel'):
            menu['level'] = int(arg('nivel'))
        if arg('etapa'):
            menu['rehab.stage'] = int(arg('etapa'))
        nodes.append(Node(package='muralistas_hri', executable='scanning_menu', name='scanning_menu', output='screen',
                          parameters=[MURAL, perfil, overrides('scanning_menu', menu)]))
        if show_ui:
            nodes.append(Node(package='muralistas_hri', executable='scanning_ui', name='scanning_ui',
                              output='screen', parameters=[MURAL]))
    else:
        joy_over = {}
        if arg('asistencia'):          # el nivel pedido en el launch gana a lo calibrado
            joy_over = {'velocidad_max': PRESETS[level]['max_speed'],
                        'one_euro.min_cutoff': PRESETS[level]['min_cutoff']}
        nodes.append(Node(package='muralistas_hri', executable='entrada_joystick', name='entrada_joystick',
                          output='screen',
                          parameters=[MURAL, mando_yaml, perfil] + ([overrides('entrada_joystick', joy_over)]
                                                                    if joy_over else [])))
        ghost = {'interaccion': interaction, 'modo': mode, 'asistencia': level}
        if arg('etapa'):
            ghost['rehab.etapa'] = int(arg('etapa'))
        nodes.append(Node(package='muralistas_hri', executable='fantasma', name='fantasma', output='screen',
                          parameters=[MURAL, perfil, overrides('fantasma', ghost)] + extra))
        if show_ui:
            nodes.append(Node(package='muralistas_hri', executable='pantalla_fantasma', name='pantalla_fantasma',
                              output='screen', parameters=[MURAL, mando_yaml, {'teclado': keyboard}]))
    summary = (f'Interacción {interaction}, modo {mode}, asistencia {level}, mando {mando}, '
               f'perfil {os.path.basename(perfil)}')
    return nodes, LogInfo(msg=summary)
