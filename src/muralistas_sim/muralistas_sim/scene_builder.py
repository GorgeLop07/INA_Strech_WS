"""Construye la escena del experimento a partir del modelo oficial del Stretch 3.

Qué agrega al modelo de Hello Robot:
  - un pincel con resorte sostenido por la pinza (articulación deslizante con rigidez),
  - una pared con papel kraft blanco,
  - marcadores ArUco en el margen del papel (para que la percepción ubique el mural).

El modelo original no se modifica: se lee, se le inserta el pincel y se escribe una
copia en build/muralistas_sim_scene/.
"""

from dataclasses import dataclass, field

import cv2
import mujoco
import numpy as np

from muralistas_common.colors import BRUSH_BODY_RGB, rgb
from muralistas_common.geometry import make_transform, matrix_to_quat
from muralistas_common.layout import MuralLayout

from .paths import scene_cache_dir, stretch_models_dir

# Orientación del marco del mural en el mundo: x del mural = -x del mundo,
# y del mural = +z (arriba), z del mural = +y (sale de la pared hacia el robot).
R_WORLD_BOARD = np.array([[-1.0, 0.0, 0.0],
                          [0.0, 0.0, 1.0],
                          [0.0, 1.0, 0.0]])

ARUCO_DICT = cv2.aruco.DICT_4X4_50
PAPER_HALF_THICKNESS = 0.001


@dataclass
class SceneConfig:
    layout: MuralLayout = field(default_factory=MuralLayout)
    wall_y: float = -0.75             # superficie del papel (y del mundo); el robot está en y = 0
    board_origin_x: float = -1.0      # x del mundo donde empieza el papel (u = 0); < 0 = centrado en el robot
    paper_bottom_z: float = -1.0      # altura del borde inferior del papel; < 0 = mural.bottom_height
    robot_x: float = 0.0
    robot_y: float = 0.0
    robot_yaw: float = 0.0
    # Error de agarre del pincel en la pinza (m, marco de la pinza). Sirve para probar la calibración.
    grip_error: tuple = (0.0, 0.0, 0.0)
    brush_spring_stiffness: float = 300.0   # N/m
    initial_color: str = 'azul'
    camera_fovy: float = 42.0               # cámara de la cabeza (D435if), como en stretch_mujoco
    stretch_models: str = ''
    cache_dir: str = ''

    def __post_init__(self):
        if self.board_origin_x < 0:
            self.board_origin_x = self.robot_x + self.layout.width / 2.0
        if self.paper_bottom_z < 0:
            self.paper_bottom_z = self.layout.bottom_height

    def board_transform(self):
        """T_world_board (4x4)."""
        origin = np.array([self.board_origin_x, self.wall_y, self.paper_bottom_z])
        return make_transform(R_WORLD_BOARD, origin)


def _mj_quat(R):
    x, y, z, w = matrix_to_quat(R)
    return f'{w:.6f} {x:.6f} {y:.6f} {z:.6f}'


def _fmt(v):
    return ' '.join(f'{float(a):.5f}' for a in v)


def _brush_xml(cfg):
    ex, ey, ez = cfg.grip_error
    tip = rgb(cfg.initial_color)
    body = BRUSH_BODY_RGB
    # El centro de agarre está a 0.23 m sobre +Z del cuerpo de la pinza; el pincel apunta en +Z.
    return f"""
                          <body name="brush_mount" pos="{ex:.4f} {ey:.4f} {0.23 + ez:.4f}">
                            <geom name="brush_sleeve" type="cylinder" fromto="0 0 -0.045 0 0 0.012" size="0.016"
                                  rgba="0.35 0.35 0.38 1" contype="0" conaffinity="0" group="1" mass="0.02"/>
                            <body name="brush" gravcomp="1">
                              <joint name="joint_brush_spring" type="slide" axis="0 0 1" range="-0.03 0.003"
                                     stiffness="{cfg.brush_spring_stiffness}" springref="0" damping="6" armature="0.001"/>
                              <geom name="brush_barrel" type="cylinder" fromto="0 0 -0.06 0 0 0.072" size="0.011"
                                    rgba="{_fmt(body)} 1" contype="0" conaffinity="0" group="1" mass="0.02"/>
                              <geom name="brush_tip" type="sphere" pos="0 0 0.082" size="0.007"
                                    rgba="{_fmt(tip)} 1" contype="2" conaffinity="0" condim="3"
                                    friction="0.15 0.005 0.0001" solref="0.004 1" group="1" mass="0.002"/>
                              <site name="brush_tip_site" pos="0 0 0.089" size="0.003"/>
                            </body>
                          </body>"""


def _write_marker_textures(cfg, cache):
    dictionary = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
    files = {}
    for marker_id in cfg.layout.marker_positions():
        # Sin voltear: con la orientación del plano (x del mural, y hacia arriba) el marcador
        # queda derecho visto de frente. Voltearlo lo deja en espejo y ya no se detecta.
        img = cv2.aruco.generateImageMarker(dictionary, marker_id, 240)
        path = cache / f'aruco_{marker_id}.png'
        cv2.imwrite(str(path), cv2.cvtColor(img, cv2.COLOR_GRAY2BGR))
        files[marker_id] = path
    return files


def build_scene(cfg: SceneConfig):
    """Escribe el MJCF y devuelve (MjModel, ruta de la escena, metadatos)."""
    models = stretch_models_dir(cfg.stretch_models)
    cache = scene_cache_dir(cfg.cache_dir)

    robot_xml = (models / 'stretch.xml').read_text()
    robot_xml = robot_xml.replace('assetdir="assets"', f'assetdir="{models / "assets"}"')
    anchor = '<body name="link_grasp_center" pos="0 0 0.23" euler="-1.5707963267949 -1.5707963267948 0"> </body>'
    if anchor not in robot_xml:
        raise RuntimeError('No encontré link_grasp_center en stretch.xml: ¿cambió el modelo del repo?')
    robot_xml = robot_xml.replace(anchor, anchor + _brush_xml(cfg))
    robot_path = cache / 'stretch_muralista.xml'
    robot_path.write_text(robot_xml)

    layout = cfg.layout
    T = cfg.board_transform()
    R, origin = T[:3, :3], T[:3, 3]

    def board_to_world(u, v, w=0.0):
        return origin + R @ np.array([u, v, w])

    textures = _write_marker_textures(cfg, cache)
    asset_lines, marker_lines = [], []
    half = layout.marker_size / 2.0
    for marker_id, (u, v) in layout.marker_positions().items():
        asset_lines.append(f'<texture name="tex_aruco_{marker_id}" type="2d" file="{textures[marker_id]}"/>')
        asset_lines.append(f'<material name="mat_aruco_{marker_id}" texture="tex_aruco_{marker_id}" '
                           f'texrepeat="1 1" texuniform="false" specular="0" shininess="0"/>')
        pos = board_to_world(u, v, 2 * PAPER_HALF_THICKNESS + 0.0005)
        marker_lines.append(f'<geom name="aruco_{marker_id}" type="plane" size="{half} {half} 0.01" '
                            f'pos="{_fmt(pos)}" quat="{_mj_quat(R)}" material="mat_aruco_{marker_id}" '
                            f'contype="0" conaffinity="0"/>')

    # La superficie de contacto es la pared (z = 0 del mural). El papel es solo una capa visual
    # delante de ella: si el papel también chocara, la punta quedaba atrapada entre las dos cajas
    # (~60 N en vez de los ~2 N del resorte). La tinta se registra donde la punta toca la pared
    # dentro del rectángulo del papel.
    paper_center = board_to_world(layout.width / 2, layout.height / 2, PAPER_HALF_THICKNESS)
    wall_center = np.array([cfg.board_origin_x - layout.width / 2, cfg.wall_y - 0.05, 1.2])

    scene_xml = f"""<mujoco model="muralistas">
  <include file="{robot_path}"/>
  <statistic center="0 -0.4 0.8" extent="1.6" meansize="0.05"/>
  <visual>
    <headlight diffuse="0.5 0.5 0.5" ambient="0.35 0.35 0.35" specular="0 0 0"/>
    <global azimuth="-60" elevation="-15" offwidth="1280" offheight="720"/>
    <quality shadowsize="4096"/>
  </visual>
  <asset>
    <texture name="tex_floor" type="2d" builtin="checker" rgb1="0.82 0.80 0.76" rgb2="0.74 0.72 0.68" width="512" height="512"/>
    <material name="mat_floor" texture="tex_floor" texrepeat="6 6" reflectance="0.05"/>
    <material name="mat_wall" rgba="0.62 0.70 0.78 1" specular="0"/>
    <material name="mat_paper" rgba="0.97 0.96 0.93 1" specular="0"/>
    {chr(10).join('    ' + line for line in asset_lines).strip()}
  </asset>
  <worldbody>
    <light name="sun" pos="0.5 1.5 3.0" dir="-0.2 -0.5 -1" directional="true" diffuse="0.55 0.55 0.55" castshadow="true"/>
    <light name="fill" pos="0 -0.2 2.5" dir="0 -0.3 -1" diffuse="0.25 0.25 0.25" castshadow="false"/>
    <geom name="floor" type="plane" size="0 0 0.05" material="mat_floor"/>
    <geom name="wall" type="box" size="{layout.width / 2 + 1.5} 0.05 1.2" pos="{_fmt(wall_center)}"
          material="mat_wall" contype="1" conaffinity="3" friction="0.15 0.005 0.0001"/>
    <geom name="paper" type="box" size="{layout.width / 2} {PAPER_HALF_THICKNESS} {layout.height / 2}"
          pos="{_fmt(paper_center)}" material="mat_paper" contype="0" conaffinity="0"/>
    {chr(10).join('    ' + line for line in marker_lines).strip()}
    <site name="board_origin" pos="{_fmt(origin)}" quat="{_mj_quat(R)}" size="0.01" rgba="1 0 1 1"/>
  </worldbody>
</mujoco>
"""
    scene_path = cache / 'escena_mural.xml'
    scene_path.write_text(scene_xml)

    model = mujoco.MjModel.from_xml_path(str(scene_path))
    head_cam = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, 'd435i_camera_rgb')
    model.cam_fovy[head_cam] = cfg.camera_fovy

    meta = {'T_world_board': T, 'scene_path': scene_path}
    return model, scene_path, meta


def initial_pose(model, data, cfg: SceneConfig, joints):
    """Coloca la base y las articulaciones (dict nombre -> valor) y fija los controles en esa pose."""
    base = free_joint_qposadr(model)
    data.qpos[base:base + 3] = [cfg.robot_x, cfg.robot_y, 0.0]
    half = cfg.robot_yaw / 2.0
    data.qpos[base + 3:base + 7] = [np.cos(half), 0.0, 0.0, np.sin(half)]
    for name, value in joints.items():
        if name == 'wrist_extension':
            for i in range(4):
                data.qpos[model.joint(f'joint_arm_l{i}').qposadr[0]] = value / 4.0
        else:
            data.qpos[model.joint(name).qposadr[0]] = value
    mujoco.mj_forward(model, data)


def free_joint_qposadr(model):
    """La base del Stretch es una articulación libre sin nombre: se busca por tipo."""
    free = np.where(model.jnt_type == mujoco.mjtJoint.mjJNT_FREE)[0]
    if len(free) == 0:
        raise RuntimeError('El modelo no tiene base libre')
    return int(model.jnt_qposadr[free[0]])
