"""Nodo de simulación: el Stretch 3 en MuJoCo frente al mural.

Expone las mismas interfaces que usaría el robot real, para que el resto del sistema
no sepa si habla con la simulación o con el driver:
  - acción  /stretch_controller/follow_joint_trajectory  (control_msgs/FollowJointTrajectory)
  - tópico  /stretch/cmd_vel                             (geometry_msgs/Twist)
  - tópicos /joint_states, /odom, /tf, /clock
  - cámara  /camera/color/image_raw, /camera/color/camera_info,
            /camera/aligned_depth_to_color/image_raw (16UC1 en mm, como el RealSense)

Y algunas cosas propias de la simulación:
  - /mural/canvas (imagen del mural con la tinta), /mural/ink_markers (tinta en RViz)
  - servicios /sim/set_brush_color y /sim/clear_mural
  - marcos de "verdad" para medir errores: board_gt y brush_tip_gt
"""

import os

if 'MUJOCO_GL' not in os.environ:
    os.environ['MUJOCO_GL'] = 'egl'   # render sin ventana; el visor de MuJoCo usa su propia ventana

import array  # noqa: E402
import signal  # noqa: E402
import threading  # noqa: E402
import time  # noqa: E402

import mujoco  # noqa: E402
import mujoco.viewer  # noqa: E402
import numpy as np  # noqa: E402
import rclpy  # noqa: E402
from builtin_interfaces.msg import Time  # noqa: E402
from control_msgs.action import FollowJointTrajectory  # noqa: E402
from geometry_msgs.msg import Point, TransformStamped, Twist  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from rclpy.action import ActionServer, CancelResponse, GoalResponse  # noqa: E402
from rclpy.callback_groups import ReentrantCallbackGroup  # noqa: E402
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile  # noqa: E402
from rosgraph_msgs.msg import Clock  # noqa: E402
from sensor_msgs.msg import CameraInfo, Image, JointState  # noqa: E402
from std_srvs.srv import Trigger  # noqa: E402
from tf2_ros import TransformBroadcaster  # noqa: E402
from visualization_msgs.msg import Marker, MarkerArray  # noqa: E402

from muralistas_common.colors import rgb  # noqa: E402
from muralistas_common.geometry import (intrinsics_from_fovy, invert_transform,  # noqa: E402
                                        make_transform, matrix_to_quat)
from muralistas_common.layout import MuralLayout  # noqa: E402
from muralistas_interfaces.srv import SetBrushColor  # noqa: E402

from .ink import InkTracker  # noqa: E402
from .joint_controller import POSITION_ACTUATORS, StretchJointController  # noqa: E402
from .scene_builder import SceneConfig, build_scene, initial_pose  # noqa: E402

HEAD_CAMERA = 'd435i_camera_rgb'
MJ_TO_OPTICAL = np.diag([1.0, -1.0, -1.0])   # cámara MuJoCo (mira a -z, y arriba) -> óptica (z al frente, y abajo)


def _stamp(sim_time):
    sec = int(sim_time)
    return Time(sec=sec, nanosec=int((sim_time - sec) * 1e9))


class MuralSimNode(Node):
    def __init__(self):
        super().__init__('mural_sim', parameter_overrides=[Parameter('use_sim_time', value=False)])
        p = self.declare_parameter
        layout_params = {k: p(f'mural.{k}', v).value for k, v in MuralLayout().__dict__.items()}
        self.layout = MuralLayout.from_dict(layout_params)
        self.cfg = SceneConfig(
            layout=self.layout,
            wall_y=p('scene.wall_y', -0.75).value,
            board_origin_x=p('scene.board_origin_x', -1.0).value,
            paper_bottom_z=p('scene.paper_bottom_z', -1.0).value,
            robot_x=p('scene.robot_x', 0.0).value,
            robot_y=p('scene.robot_y', 0.0).value,
            robot_yaw=p('scene.robot_yaw', 0.0).value,
            grip_error=tuple(p('scene.grip_error', [0.0, 0.0, 0.0]).value),
            brush_spring_stiffness=p('scene.brush_spring_stiffness', 300.0).value,
            initial_color=p('scene.initial_color', 'azul').value,
            camera_fovy=p('camera.fovy', 42.0).value,
        )
        self.cam_w = p('camera.width', 640).value
        self.cam_h = p('camera.height', 360).value
        self.cam_rate = p('camera.rate', 10.0).value
        self.state_rate = p('state_rate', 50.0).value
        self.show_viewer = p('viewer', True).value
        self.real_time = p('real_time', True).value
        self.initial_joints = {
            'joint_lift': p('initial.lift', 0.6).value,
            'wrist_extension': p('initial.extension', 0.05).value,
            'joint_wrist_yaw': p('initial.wrist_yaw', 0.0).value,
            'joint_wrist_pitch': p('initial.wrist_pitch', 0.0).value,
            'joint_wrist_roll': p('initial.wrist_roll', 0.0).value,
            'joint_head_pan': p('initial.head_pan', -1.5708).value,
            'joint_head_tilt': p('initial.head_tilt', -0.6).value,
        }

        self.model, scene_path, meta = build_scene(self.cfg)
        # Con 4 ms y 50 iteraciones la simulación va ~1.7x tiempo real y el trazo sale igual
        # que con la configuración original del repo (2 ms, 100 iteraciones).
        self.model.opt.timestep = p('physics.timestep', 0.004).value
        self.model.opt.iterations = p('physics.iterations', 50).value
        self.model.opt.ls_iterations = p('physics.ls_iterations', 20).value
        # El lidar del modelo son 360 rayos contra mallas de cientos de miles de vértices: cuesta
        # ~2.4 ms por paso (28 veces más que todo lo demás). Se apaga mientras no se use.
        if not p('physics.lidar', False).value:
            self.model.opt.disableflags |= int(mujoco.mjtDisableBit.mjDSBL_SENSOR)
        self.data = mujoco.MjData(self.model)
        self.T_world_board = meta['T_world_board']
        initial_pose(self.model, self.data, self.cfg, self.initial_joints)
        for name, value in self.initial_joints.items():
            self.data.ctrl[self.model.actuator(POSITION_ACTUATORS[name]).id] = value
        self.get_logger().info(f'Escena: {scene_path}')

        self.lock = threading.Lock()
        self.controller = StretchJointController(self.model, self.data)
        self.ink = InkTracker(self.model, self.T_world_board, self.layout)
        self.ink.set_color(self.cfg.initial_color)
        self.cam_id = self.model.camera(HEAD_CAMERA).id
        self.fx, self.fy, self.cx, self.cy = intrinsics_from_fovy(self.cfg.camera_fovy, self.cam_w, self.cam_h)

        # Publicadores
        self.pub_clock = self.create_publisher(Clock, '/clock', 10)
        self.pub_joints = self.create_publisher(JointState, '/joint_states', 10)
        self.pub_odom = self.create_publisher(Odometry, '/odom', 10)
        self.pub_rgb = self.create_publisher(Image, '/camera/color/image_raw', 2)
        self.pub_depth = self.create_publisher(Image, '/camera/aligned_depth_to_color/image_raw', 2)
        self.pub_info = self.create_publisher(CameraInfo, '/camera/color/camera_info', 2)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_canvas = self.create_publisher(Image, '/mural/canvas', latched)
        self.pub_ink = self.create_publisher(MarkerArray, '/mural/ink_markers', latched)
        self.tf = TransformBroadcaster(self)

        group = ReentrantCallbackGroup()
        self.create_subscription(Twist, '/stretch/cmd_vel', self._on_cmd_vel, 10, callback_group=group)
        self.create_service(SetBrushColor, '/sim/set_brush_color', self._on_set_color, callback_group=group)
        self.create_service(Trigger, '/sim/clear_mural', self._on_clear, callback_group=group)
        self.traj_server = ActionServer(
            self, FollowJointTrajectory, '/stretch_controller/follow_joint_trajectory',
            execute_callback=self._execute_trajectory, goal_callback=self._on_goal,
            cancel_callback=lambda _: CancelResponse.ACCEPT, callback_group=group)

        self._goal_lock = threading.Lock()
        self._active_goal = 0
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._physics_loop, daemon=True)
        self._thread.start()

    # --- Hilo de la física (también renderiza: el contexto de OpenGL vive aquí) ---
    def _physics_loop(self):
        m, d = self.model, self.data
        renderer = mujoco.Renderer(m, self.cam_h, self.cam_w)
        scene_option = mujoco.MjvOption()
        scene_option.flags[mujoco.mjtVisFlag.mjVIS_RANGEFINDER] = False
        viewer = None
        if self.show_viewer:
            try:
                viewer = mujoco.viewer.launch_passive(m, d, show_left_ui=False, show_right_ui=False)
                with viewer.lock():   # vista inicial: detrás y a la derecha del robot, mirando al mural
                    viewer.cam.lookat[:] = [self.cfg.robot_x, (self.cfg.robot_y + self.cfg.wall_y) / 2, 0.7]
                    viewer.cam.distance = 2.7
                    viewer.cam.azimuth = -127.0
                    viewer.cam.elevation = -20.0
            except Exception as exc:  # sin pantalla, por ejemplo
                self.get_logger().warn(f'No pude abrir el visor de MuJoCo: {exc}')

        control_dt = 0.01
        substeps = max(1, int(round(control_dt / m.opt.timestep)))
        cost = {'física': 0.0, 'estado': 0.0, 'cámara': 0.0, 'tinta': 0.0, 'visor': 0.0}
        try:
            self._loop(m, d, renderer, scene_option, viewer, substeps, cost)
        except Exception:
            if rclpy.ok():
                raise
            # Ctrl+C: ROS ya se apagó y una publicación a medias falla; no es un error.
        renderer.close()
        if viewer is not None and viewer.is_running():
            viewer.close()

    def _loop(self, m, d, renderer, scene_option, viewer, substeps, cost):
        next_state = next_cam = next_canvas = 0.0
        next_viewer = 0.0
        last_ink_version = -1
        wall_start, sim_start = time.monotonic(), d.time
        rtf_wall, rtf_sim = wall_start, d.time
        while not self._stop.is_set() and rclpy.ok():
            now = time.monotonic()
            if now - rtf_wall > 10.0:
                rtf = (d.time - rtf_sim) / (now - rtf_wall)
                detail = ', '.join(f'{k} {100 * v / (now - rtf_wall):.0f}%' for k, v in cost.items())
                self.get_logger().info(f'Simulación a {rtf:.2f}x tiempo real ({detail} del tiempo)')
                rtf_wall, rtf_sim = now, d.time
                cost = dict.fromkeys(cost, 0.0)
            t0 = time.monotonic()
            with self.lock:
                for _ in range(substeps):
                    self.controller.update()
                    mujoco.mj_step(m, d)
                    self.ink.update(d)
                sim_time = d.time
            self.pub_clock.publish(Clock(clock=_stamp(sim_time)))
            t1 = time.monotonic()
            cost['física'] += t1 - t0

            if sim_time >= next_state:
                next_state = sim_time + 1.0 / self.state_rate
                with self.lock:
                    self._publish_state(sim_time)
                t2 = time.monotonic()
                cost['estado'] += t2 - t1
                t1 = t2
            if sim_time >= next_cam:
                next_cam = sim_time + 1.0 / self.cam_rate
                self._publish_camera(renderer, scene_option, sim_time)
                t2 = time.monotonic()
                cost['cámara'] += t2 - t1
                t1 = t2
            if sim_time >= next_canvas and self.ink.version != last_ink_version:
                next_canvas = sim_time + 0.5
                last_ink_version = self.ink.version
                self._publish_ink(sim_time)
                t2 = time.monotonic()
                cost['tinta'] += t2 - t1
                t1 = t2
            if viewer is not None and time.monotonic() >= next_viewer:
                next_viewer = time.monotonic() + 1.0 / 30.0
                if not viewer.is_running():
                    viewer = None
                else:
                    with self.lock, viewer.lock():
                        viewer.user_scn.ngeom = 0
                        self.ink.add_to_scene(viewer.user_scn)
                        viewer.sync()
                cost['visor'] += time.monotonic() - t1

            if self.real_time:
                ahead = (sim_time - sim_start) - (time.monotonic() - wall_start)
                if ahead > 0:
                    time.sleep(ahead)

    def _publish_state(self, sim_time):
        stamp = _stamp(sim_time)
        c = self.controller
        names = ['joint_lift', 'wrist_extension', 'joint_wrist_yaw', 'joint_wrist_pitch', 'joint_wrist_roll',
                 'joint_head_pan', 'joint_head_tilt', 'joint_gripper_finger_left']
        js = JointState()
        js.header.stamp = stamp
        js.name = names + [f'joint_arm_l{i}' for i in range(4)] + ['joint_brush_spring']
        js.position = [c.joint_position(n) for n in names] + \
            [float(self.data.qpos[self.model.joint(f'joint_arm_l{i}').qposadr[0]]) for i in range(4)] + \
            [float(self.data.qpos[self.model.joint('joint_brush_spring').qposadr[0]])]
        self.pub_joints.publish(js)

        T_world_base = self._body_transform('base_link')
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z = T_world_base[:3, 3]
        q = matrix_to_quat(T_world_base[:3, :3])
        odom.pose.pose.orientation.x, odom.pose.pose.orientation.y = q[0], q[1]
        odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = q[2], q[3]
        self.pub_odom.publish(odom)

        T_base_world = invert_transform(T_world_base)
        tip = make_transform(np.eye(3), self.data.site('brush_tip_site').xpos)
        cam_R = self.data.cam_xmat[self.cam_id].reshape(3, 3) @ MJ_TO_OPTICAL
        T_world_cam = make_transform(cam_R, self.data.cam_xpos[self.cam_id])
        transforms = [
            ('odom', 'base_link', T_world_base),
            ('base_link', 'link_grasp_center', T_base_world @ self._body_transform('link_grasp_center')),
            ('base_link', 'camera_color_optical_frame', T_base_world @ T_world_cam),
            ('base_link', 'brush_tip_gt', T_base_world @ tip),
            ('odom', 'board_gt', self.T_world_board),
        ]
        self.tf.sendTransform([self._tf_msg(stamp, a, b, T) for a, b, T in transforms])

    def _publish_camera(self, renderer, scene_option, sim_time):
        with self.lock:
            renderer.update_scene(self.data, HEAD_CAMERA, scene_option)
            self.ink.add_to_scene(renderer.scene)
            color = renderer.render().copy()
            renderer.enable_depth_rendering()
            renderer.update_scene(self.data, HEAD_CAMERA, scene_option)
            depth = renderer.render().copy()
            renderer.disable_depth_rendering()
        stamp = _stamp(sim_time)
        depth_mm = np.where((depth > 0.1) & (depth < 6.0), depth * 1000.0, 0).astype(np.uint16)

        img = Image()
        img.header.stamp = stamp
        img.header.frame_id = 'camera_color_optical_frame'
        img.height, img.width = color.shape[:2]
        img.encoding = 'rgb8'
        img.step = img.width * 3
        img.data = array.array("B", color.tobytes())   # con bytes, ROS valida byte por byte (~50 ms)
        self.pub_rgb.publish(img)

        dimg = Image()
        dimg.header = img.header
        dimg.height, dimg.width = depth_mm.shape
        dimg.encoding = '16UC1'
        dimg.step = dimg.width * 2
        dimg.data = array.array("B", depth_mm.tobytes())
        self.pub_depth.publish(dimg)

        info = CameraInfo()
        info.header = img.header
        info.height, info.width = img.height, img.width
        info.distortion_model = 'plumb_bob'
        info.d = [0.0] * 5
        info.k = [self.fx, 0.0, self.cx, 0.0, self.fy, self.cy, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [self.fx, 0.0, self.cx, 0.0, 0.0, self.fy, self.cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        self.pub_info.publish(info)

    def _publish_ink(self, sim_time):
        stamp = _stamp(sim_time)
        with self.lock:
            canvas = self.ink.canvas()
            strokes = [(s['color'], list(s['points'])) for s in self.ink.strokes]
        img = Image()
        img.header.stamp = stamp
        img.header.frame_id = 'board_gt'
        img.height, img.width = canvas.shape[:2]
        img.encoding = 'rgb8'
        img.step = img.width * 3
        img.data = array.array("B", canvas.tobytes())
        self.pub_canvas.publish(img)

        markers = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        for i, (color, pts) in enumerate(strokes):
            mk = Marker()
            mk.header.stamp = stamp
            mk.header.frame_id = 'board_gt'
            mk.ns, mk.id = 'tinta', i
            mk.type = Marker.LINE_STRIP if len(pts) > 1 else Marker.SPHERE_LIST
            mk.scale.x = mk.scale.y = mk.scale.z = 0.006
            mk.pose.orientation.w = 1.0
            r, g, b = rgb(color)
            mk.color.r, mk.color.g, mk.color.b, mk.color.a = r, g, b, 1.0
            mk.points = [Point(x=float(u), y=float(v), z=0.002) for u, v in pts]
            markers.markers.append(mk)
        self.pub_ink.publish(markers)

    # --- Callbacks ---------------------------------------------------------
    def _on_cmd_vel(self, msg):
        with self.lock:
            self.controller.set_cmd_vel(msg.linear.x, msg.angular.z)

    def _on_set_color(self, request, response):
        try:
            color = rgb(request.color)
        except KeyError as exc:
            response.success, response.message = False, str(exc)
            return response
        with self.lock:
            self.model.geom_rgba[self.model.geom('brush_tip').id] = [*color, 1.0]
            self.ink.set_color(request.color)
        response.success, response.message = True, f'Pincel {request.color}'
        return response

    def _on_clear(self, request, response):
        with self.lock:
            self.ink.clear()
        response.success, response.message = True, 'Mural limpio'
        return response

    def _on_goal(self, goal_request):
        names = goal_request.trajectory.joint_names
        unknown = [n for n in names if n not in POSITION_ACTUATORS and n != 'joint_mobile_base_translation']
        if unknown:
            self.get_logger().warn(f'Trayectoria rechazada, articulaciones desconocidas: {unknown}')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _execute_trajectory(self, goal_handle):
        traj = goal_handle.request.trajectory
        points = [(pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9, list(pt.positions))
                  for pt in traj.points]
        result = FollowJointTrajectory.Result()
        with self._goal_lock:
            self._active_goal += 1
            my_goal = self._active_goal
        try:
            with self.lock:
                self.controller.set_trajectory(list(traj.joint_names), points)
        except ValueError as exc:
            goal_handle.abort()
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = str(exc)
            return result

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                with self.lock:
                    if self._active_goal == my_goal:
                        self.controller.cancel()
                goal_handle.canceled()
                result.error_string = 'Cancelada'
                return result
            with self.lock:
                status = self.controller.status
                preempted = self._active_goal != my_goal
            if preempted:
                goal_handle.abort()
                result.error_string = 'Reemplazada por otra trayectoria'
                return result
            if status == 'done':
                break
            time.sleep(0.01)
        goal_handle.succeed()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        return result

    # --- Utilidades --------------------------------------------------------
    def _body_transform(self, body):
        b = self.data.body(body)
        return make_transform(b.xmat.reshape(3, 3), b.xpos)

    @staticmethod
    def _tf_msg(stamp, parent, child, T):
        msg = TransformStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = parent
        msg.child_frame_id = child
        msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z = T[:3, 3]
        q = matrix_to_quat(T[:3, :3])
        msg.transform.rotation.x, msg.transform.rotation.y = q[0], q[1]
        msg.transform.rotation.z, msg.transform.rotation.w = q[2], q[3]
        return msg

    def shutdown(self):
        self._stop.set()
        self._thread.join(timeout=2.0)


def main():
    rclpy.init()
    node = MuralSimNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)   # un segundo Ctrl+C no interrumpe el cierre
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
