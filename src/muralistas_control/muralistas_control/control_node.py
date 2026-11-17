"""Nodo de control: pinta los trazos que manda la planeación.

Acción /control/draw_stroke (DrawStroke). Cada trazo se ejecuta por fases:
  1. acercar:  llevar la punta a unos cm del papel, frente al primer punto (pincel levantado)
  2. apoyar:   empujar hasta comprimir un poco el resorte del pincel
  3. pintar:   recorrer los puntos a la profundidad de apoyo
  4. levantar: alejar la punta del papel (si el trazo lo pide)
Cada fase es una trayectoria FollowJointTrajectory con base + elevador + brazo, igual que
con el driver real del Stretch. La geometría se recalcula al inicio de cada fase con la TF
actual, así los errores de seguimiento no se acumulan.

También:
  - /muralistas/stop (Bool): paro. Cancela el movimiento y, si el pincel está apoyado, lo levanta.
  - /control/startup (Trigger): pose inicial, barrido de cabeza para ubicar el mural y
    calibración del pincel con la percepción.
  - /control/tip_board: punta del pincel en el marco del mural (10 Hz).
  - pen_idle_lift: si el pincel se queda apoyado sin moverse ese tiempo, se levanta solo
    (la punta del plumón se aplasta). El siguiente trazo lo vuelve a apoyar.
"""

import math
import signal
import threading
import time

import numpy as np
import rclpy
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import PointStamped, Transform, Vector3Stamped
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from sensor_msgs.msg import JointState, PointCloud2
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectoryPoint, MultiDOFJointTrajectoryPoint

from muralistas_common.head_scan import head_scan_for
from muralistas_common.layout import MuralLayout
from muralistas_interfaces.action import DrawStroke
from muralistas_interfaces.msg import Board
from muralistas_interfaces.srv import CalibrateTool, SetBrushColor

from .cartesian_mapper import CartesianMapper, Limits, densify, path_length


def _duration(t):
    sec = int(t)
    return Duration(seconds=sec, nanoseconds=int((t - sec) * 1e9)).to_msg()


def _base_point(t, x):
    """Punto de la base para multi_dof_trajectory: avanzar x metros (sin girar)."""
    tf = Transform()
    tf.translation.x = float(x)
    tf.rotation.w = 1.0
    return MultiDOFJointTrajectoryPoint(transforms=[tf], time_from_start=_duration(t))


class ControlNode(Node):
    def __init__(self):
        super().__init__('control')
        p = self.declare_parameter
        self.robot_action = p('robot_action', '/stretch_controller/follow_joint_trajectory').value
        # 'sim': el simulador acepta la base como articulación y metas de un solo punto.
        # 'stretch': driver real en modo trajectory (ver _run_trajectory).
        self.robot_driver = p('robot_driver', 'sim').value
        self.base_joint = p('joints.base', 'joint_mobile_base_translation').value
        self.lift_joint = p('joints.lift', 'joint_lift').value
        self.ext_joint = p('joints.extension', 'wrist_extension').value
        self.board_frame = p('board_frame', 'board').value
        self.base_frame = p('base_frame', 'base_link').value
        self.grasp_frame = p('grasp_frame', 'link_grasp_center').value
        self.tool_offset = np.array(p('tool_offset', [0.0, -0.089, 0.0]).value, dtype=float)
        self.use_calibrated_tool = p('use_perception_tool_offset', True).value
        self.hover = p('hover_distance', 0.03).value
        self.press = p('press_depth', 0.008).value
        self.draw_speed = p('draw_speed', 0.05).value
        self.travel_speed = p('travel_speed', 0.10).value
        self.approach_speed = p('approach_speed', 0.03).value
        # Tope de la velocidad al pintar (gana a la que pida el planeador) y rampa de arranque/frenado
        self.max_draw_speed = p('max_draw_speed', 0.0).value     # 0 = sin tope
        self.draw_accel = p('draw_accel', 0.0).value             # m/s²; 0 = sin rampa
        self.retract_on_stop = p('retract_on_stop', True).value
        self.pen_idle_lift = p('pen_idle_lift', 10.0).value      # s; 0 = nunca
        self.continue_radius = p('continue_radius', 0.015).value  # m: un trazo que empieza aquí sigue sin levantar
        self.color_service = p('color_service', '/sim/set_brush_color').value
        self.limits = Limits(lift_min=p('limits.lift_min', 0.05).value, lift_max=p('limits.lift_max', 1.08).value,
                             ext_min=p('limits.ext_min', 0.0).value, ext_max=p('limits.ext_max', 0.50).value,
                             base_max=p('limits.base_max', 1.6).value)
        self.ready_pose = dict(zip(
            ['joint_lift', 'wrist_extension', 'joint_wrist_yaw', 'joint_wrist_pitch', 'joint_wrist_roll'],
            p('ready_pose', [0.6, 0.05, 0.0, 0.0, 0.0]).value))
        # La muñeca solo se manda en la pose inicial; con el peso del marcador y el apoyo el pitch se va
        # cayendo. Con hold_wrist cada trazo la vuelve a mandar a la ready_pose.
        self.hold_wrist = p('hold_wrist', True).value
        self.wrist_hold = {n: v for n, v in self.ready_pose.items() if n.startswith('joint_wrist_')}
        self.wrist_tolerance = p('wrist_tolerance', 0.03).value     # rad: antes de apoyar, si se desvió, se regresa
        # Antes de apoyar el marcador: la cabeza mira la columna de ArUco más cercana y se espera una
        # corrección del pizarrón con board_refresh_min_markers o más vistos en ese momento (si la base
        # patinó en el trazo anterior, el siguiente empieza bien). Ver board_estimator.py.
        self.board_refresh = p('board_refresh', True).value
        self.board_refresh_min = p('board_refresh_min_markers', 2).value
        self.board_refresh_timeout = p('board_refresh_timeout', 3.0).value
        self.camera_height = p('scan.camera_height', 1.30).value
        self.layout = MuralLayout.from_dict({k: p(f'mural.{k}', v).value for k, v in MuralLayout().__dict__.items()})
        if p('head_scan_auto', True).value:
            # Calculado con el pizarrón de config/mural.yaml: ver head_scan.py
            self.head_scan = head_scan_for(
                self.layout, wall_distance=p('scan.wall_distance', 0.75).value,
                camera_height=self.camera_height,
                half_fov=tuple(p('scan.half_fov', [21.0, 34.0]).value), margin=p('scan.margin_deg', 8.0).value)
        else:
            scan = p('head_scan', [-1.0, -0.45, -1.57, -0.45, -2.15, -0.45]).value
            self.head_scan = list(zip(scan[0::2], scan[1::2]))
        # Elevador abajo y brazo recogido durante el barrido: a la altura de trabajo tapan los ArUco de abajo
        self.scan_pose = tuple(p('scan_pose', [0.2, 0.0]).value)
        self.head_work = tuple(p('head_work_pose', [-1.5708, -0.6]).value)
        self.auto_startup = p('auto_startup', True).value
        self.head_scan_dwell = p('head_scan_dwell', 1.0).value          # s quieta en cada pose del barrido
        # 0 = todos los del pizarrón (config/mural.yaml)
        self.board_markers_expected = (p('board_markers_expected', 0).value
                                       or len(self.layout.marker_positions()))
        self.calibrate_on_startup = p('calibrate_on_startup', True).value

        self.cb = ReentrantCallbackGroup()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.robot = ActionClient(self, FollowJointTrajectory, self.robot_action, callback_group=self.cb)
        self.color_client = self.create_client(SetBrushColor, self.color_service, callback_group=self.cb) \
            if self.color_service else None
        self.calib_client = self.create_client(CalibrateTool, '/perception/calibrate_tool', callback_group=self.cb)

        self.joints = {}
        self.create_subscription(JointState, '/joint_states', self._on_joints, 20, callback_group=self.cb)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Vector3Stamped, '/perception/tool_offset', self._on_tool_offset, latched,
                                 callback_group=self.cb)
        self.create_subscription(Bool, '/muralistas/stop', self._on_stop, 10, callback_group=self.cb)
        # Arranque: saber si la percepción ya procesa imágenes y cuántos marcadores lleva vistos
        self._perception_frames = 0
        self._board_markers = 0
        self._board_msg = None
        self.create_subscription(PointCloud2, '/perception/points', self._on_cloud, 2, callback_group=self.cb)
        self.create_subscription(Board, '/perception/board', self._on_board, latched, callback_group=self.cb)
        self.pub_tip = self.create_publisher(PointStamped, '/control/tip_board', 10)
        self.pub_pen = self.create_publisher(Bool, '/control/pen_down', latched)
        self.pub_status = self.create_publisher(String, '/control/status', latched)

        self.server = ActionServer(self, DrawStroke, '/control/draw_stroke', execute_callback=self._execute,
                                   goal_callback=self._on_goal, cancel_callback=lambda _: CancelResponse.ACCEPT,
                                   callback_group=self.cb)
        self.create_service(Trigger, '/control/startup', self._on_startup, callback_group=self.cb)
        self.create_timer(0.1, self._publish_tip, callback_group=self.cb)
        self._last_motion = time.monotonic()
        if self.pen_idle_lift > 0:
            self.create_timer(0.5, self._check_pen_idle, callback_group=self.cb)

        self.color = None
        self.pen_down = False
        self.stopped = False
        self.busy = threading.Lock()          # un trazo a la vez
        self._robot_goal = None
        self._set_pen(False)
        if self.auto_startup:
            self._status('esperando')
            threading.Thread(target=self._startup_sequence, daemon=True).start()
        else:
            self._status('listo')   # se saltó el arranque: el menú puede empezar a barrer

    # --- Estado ---------------------------------------------------------------
    def _on_joints(self, msg):
        for n, q in zip(msg.name, msg.position):
            self.joints[n] = q
        if self.ext_joint not in msg.name and all(f'joint_arm_l{i}' in self.joints for i in range(4)):
            self.joints[self.ext_joint] = sum(self.joints[f'joint_arm_l{i}'] for i in range(4))

    def _on_cloud(self, _):
        self._perception_frames += 1

    def _on_board(self, msg):
        self._board_markers = msg.markers_used
        self._board_msg = msg

    def _on_tool_offset(self, msg):
        if self.use_calibrated_tool:
            self.tool_offset = np.array([msg.vector.x, msg.vector.y, msg.vector.z])
            self.get_logger().info(f'Usando la punta calibrada por percepción: {np.round(self.tool_offset, 4)}')

    def _status(self, text):
        self.pub_status.publish(String(data=text))

    def _idle_status(self):
        """Al terminar un trazo: 'paro' si el paro sigue activo (para que el menú no barra), si no 'listo'."""
        self._status('paro' if self.stopped else 'listo')

    def _set_pen(self, down):
        self.pen_down = down
        self.pub_pen.publish(Bool(data=down))

    def _lookup(self, target, source):
        from muralistas_common.geometry import make_transform, quat_to_matrix
        t = self.tf_buffer.lookup_transform(target, source, Time(), Duration(seconds=0.5))
        tr, q = t.transform.translation, t.transform.rotation
        return make_transform(quat_to_matrix((q.x, q.y, q.z, q.w)), [tr.x, tr.y, tr.z])

    def _tip_base(self):
        T = self._lookup(self.base_frame, self.grasp_frame)
        return T[:3, :3] @ self.tool_offset + T[:3, 3]

    def _geometry(self):
        """Mapper con la TF y las articulaciones de este instante."""
        T_base_board = self._lookup(self.base_frame, self.board_frame)
        tip = self._tip_base()
        lift, ext = self.joints.get(self.lift_joint), self.joints.get(self.ext_joint)
        if lift is None or ext is None:
            raise RuntimeError('Todavía no llegan /joint_states')
        return CartesianMapper(T_base_board, tip, lift, ext, self.limits), tip

    def _publish_tip(self):
        try:
            T_board_base = self._lookup(self.board_frame, self.base_frame)
            tip = T_board_base[:3, :3] @ self._tip_base() + T_board_base[:3, 3]
        except Exception:
            return
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.board_frame
        msg.point.x, msg.point.y, msg.point.z = map(float, tip)
        self.pub_tip.publish(msg)

    # --- Trayectorias del robot --------------------------------------------
    def _run_trajectory(self, names, points, goal_handle=None):
        """Manda una trayectoria y espera. Devuelve 'ok', 'canceled', 'stopped' o 'error'."""
        if not self.robot.wait_for_server(timeout_sec=3.0):
            self.get_logger().error(f'No responde {self.robot_action}')
            return 'error'
        if self.robot_driver == 'stretch':
            goal = self._stretch_goal(names, points)
        else:
            goal = FollowJointTrajectory.Goal()
            goal.trajectory.joint_names = list(names)
            goal.trajectory.points = [JointTrajectoryPoint(positions=[float(x) for x in q], time_from_start=_duration(t))
                                      for t, q in points]
        future = self.robot.send_goal_async(goal)
        while not future.done():
            time.sleep(0.01)
        handle = future.result()
        if not handle.accepted:
            return 'error'
        self._robot_goal = handle
        result = handle.get_result_async()
        outcome = 'ok'
        while not result.done():
            if self.stopped:
                handle.cancel_goal_async()
                outcome = 'stopped'
                break
            if goal_handle is not None and goal_handle.is_cancel_requested:
                handle.cancel_goal_async()
                outcome = 'canceled'
                break
            time.sleep(0.01)
        if outcome != 'ok':
            deadline = time.monotonic() + 2.0
            while not result.done() and time.monotonic() < deadline:
                time.sleep(0.01)
        self._robot_goal = None
        return outcome

    def _stretch_goal(self, names, points):
        """Meta para el driver real (stretch_core, modo trajectory). Lo que pide distinto al simulador:
          - cada trayectoria necesita al menos 2 puntos y el primero, en t=0, debe ser la posición
            actual (fail_if_motor_initial_point_is_not_trajectory_first_point);
          - la base no es una articulación: va en multi_dof_trajectory como 'position' (x, y, θ),
            relativa a donde está la base al arrancar la trayectoria.
        """
        goal = FollowJointTrajectory.Goal()
        names = list(names)
        base_i = names.index(self.base_joint) if self.base_joint in names else None
        joint_names = [n for i, n in enumerate(names) if i != base_i]
        if joint_names:
            missing = [n for n in joint_names if n not in self.joints]
            if missing:
                raise RuntimeError(f'No llegan /joint_states de {missing}')
            goal.trajectory.joint_names = joint_names
            goal.trajectory.points = [JointTrajectoryPoint(positions=[float(self.joints[n]) for n in joint_names],
                                                           time_from_start=_duration(0.0))]
            goal.trajectory.points += [
                JointTrajectoryPoint(positions=[float(x) for i, x in enumerate(q) if i != base_i],
                                     time_from_start=_duration(t))
                for t, q in points]
        if base_i is not None:
            goal.multi_dof_trajectory.joint_names = ['position']
            goal.multi_dof_trajectory.points = [_base_point(0.0, 0.0)]
            goal.multi_dof_trajectory.points += [_base_point(t, q[base_i]) for t, q in points]
        return goal

    def _cartesian_phase(self, board_points, w, speed, goal_handle=None, accel=None):
        mapper, tip = self._geometry()
        traj = mapper.trajectory(board_points, w, speed, start_base=tip, accel=accel or None)
        for _, joints in traj:
            problems = mapper.check(joints)
            if problems:
                raise RuntimeError('Fuera de alcance: ' + '; '.join(problems))
        names = [self.base_joint, self.lift_joint, self.ext_joint]
        if self.hold_wrist:
            names += list(self.wrist_hold)
            traj = [(t, tuple(joints) + tuple(self.wrist_hold.values())) for t, joints in traj]
        return self._run_trajectory(names, traj, goal_handle)

    # --- Acción DrawStroke ----------------------------------------------------
    def _on_goal(self, _):
        if self.stopped:
            self.get_logger().warn('Trazo rechazado: el paro está activo')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _execute(self, goal_handle):
        with self.busy:
            try:
                return self._draw(goal_handle)
            finally:
                self._last_motion = time.monotonic()

    def _check_pen_idle(self):
        """Levanta el pincel si lleva pen_idle_lift segundos apoyado sin moverse."""
        if not self.pen_down or self.stopped or time.monotonic() - self._last_motion < self.pen_idle_lift:
            return
        if not self.busy.acquire(blocking=False):
            return
        try:
            self.get_logger().info(f'Pincel apoyado {self.pen_idle_lift:.0f} s sin moverse: lo levanto')
            self._status('levantar')
            self._cartesian_phase([tuple(self._current_uv())], self.hover, self.approach_speed)
            self._set_pen(False)
            self._idle_status()
        except Exception as exc:
            self.get_logger().warn(f'No pude levantar el pincel: {exc}')
        finally:
            self._last_motion = time.monotonic()
            self.busy.release()

    def _draw(self, goal_handle):
        g = goal_handle.request
        result = DrawStroke.Result()
        feedback = DrawStroke.Feedback()
        try:
            mapper, tip = self._geometry()
        except Exception as exc:
            goal_handle.abort()
            result.message = f'Aún no puedo pintar: {exc}'
            self.get_logger().warn(result.message)
            return result
        tip_uv = mapper.base_to_board(tip)[:2]
        pts = [(pt.x, pt.y) for pt in g.points]
        if g.relative_to_current:
            pts = [(tip_uv[0] + du, tip_uv[1] + dv) for du, dv in pts]
        speed = g.speed if g.speed > 0 else self.draw_speed
        if self.max_draw_speed > 0:
            speed = min(speed, self.max_draw_speed)

        if g.color and g.color != self.color:
            self._change_color(g.color)

        phases = []
        if g.travel_only:
            # Desplazarse con el pincel levantado hasta el último punto, sin pintar
            if self.pen_down:
                phases.append(('levantar', [tuple(tip_uv)], self.hover, self.approach_speed, False))
            if pts:
                phases.append(('desplazar', [pts[-1]], self.hover, self.travel_speed, None))
            pts, draw_pts, pen = [], [], False
        elif pts and self.pen_down and not g.relative_to_current \
                and math.dist(pts[0], tuple(tip_uv)) > self.continue_radius:
            phases.append(('levantar', [tuple(tip_uv)], self.hover, self.approach_speed, False))
            pen = False
        else:
            pen = self.pen_down
        if g.travel_only:
            pass
        elif pts and g.relative_to_current and not pen:
            # Trazo libre con el pincel levantado (p. ej. tras levantarse solo): apoyar donde está
            phases.append(('apoyar', [tuple(tip_uv)], -self.press, self.approach_speed, True))
            draw_pts = densify([tuple(tip_uv)] + pts, 0.01)[1:]
        elif pts and not pen:
            phases.append(('acercar', [pts[0]], self.hover, self.travel_speed, None))
            phases.append(('apoyar', [pts[0]], -self.press, self.approach_speed, True))
            draw_pts = densify(pts, 0.01)[1:]
        elif pts:
            draw_pts = densify([tuple(tip_uv)] + pts, 0.01)[1:]
        else:
            draw_pts = []
        if draw_pts:
            phases.append(('pintar', draw_pts, -self.press, speed, None))
        if g.lift_at_end and not g.travel_only:
            last = pts[-1] if pts else tuple(tip_uv)
            phases.append(('levantar', [last], self.hover, self.approach_speed, False))

        drawn = 0.0
        for i, (name, board_pts, w, v, pen_after) in enumerate(phases):
            if name == 'apoyar':
                self._before_contact()
            self._status(name)
            feedback.progress = i / max(1, len(phases))
            feedback.pen_down = self.pen_down
            goal_handle.publish_feedback(feedback)
            try:
                outcome = self._cartesian_phase(board_pts, w, v, goal_handle,
                                                accel=self.draw_accel if name == 'pintar' else None)
            except Exception as exc:
                self._status('error')
                goal_handle.abort()
                result.message = str(exc)
                self.get_logger().warn(f'Trazo abortado en "{name}": {exc}')
                return result
            if outcome == 'ok':
                if pen_after is not None:
                    self._set_pen(pen_after)
                if name == 'pintar':
                    drawn = path_length(board_pts)
                    dev = self._wrist_deviation()
                    if dev and max(abs(d) for d in dev.values()) > self.wrist_tolerance:
                        self.get_logger().warn('La muñeca se desvió al pintar (¿se atoró el marcador?): ' + ', '.join(
                            f'{n.replace("joint_wrist_", "")} {math.degrees(d):+.1f}°' for n, d in dev.items()))
                continue
            # Cancelado (el niño detuvo el trazo libre) o paro
            if outcome == 'canceled' and g.lift_at_end and self.pen_down:
                self._cartesian_phase([tuple(self._current_uv())], self.hover, self.approach_speed)
                self._set_pen(False)
            self._idle_status()
            result.message = 'Detenido' if outcome == 'canceled' else 'Paro'
            result.drawn_length = float(drawn)
            if outcome == 'canceled':
                goal_handle.canceled()
            else:
                goal_handle.abort()
            return result

        self._idle_status()
        feedback.progress = 1.0
        goal_handle.publish_feedback(feedback)
        goal_handle.succeed()
        result.success = True
        result.drawn_length = float(drawn)
        result.message = 'Trazo terminado'
        return result

    # --- Antes de apoyar: muñeca derecha y pizarrón recién visto ----------------
    def _wrist_deviation(self):
        """{articulación: desviación (rad)} respecto a la pose de trabajo, o {} sin hold_wrist."""
        if not self.hold_wrist:
            return {}
        return {n: self.joints[n] - v for n, v in self.wrist_hold.items() if n in self.joints}

    def _before_contact(self):
        dev = self._wrist_deviation()
        if dev and max(abs(d) for d in dev.values()) > self.wrist_tolerance:
            self.get_logger().info('Enderezo la muñeca antes de apoyar: ' + ', '.join(
                f'{n.replace("joint_wrist_", "")} {math.degrees(d):+.1f}°' for n, d in dev.items()))
            self._move(list(self.wrist_hold), tuple(self.wrist_hold.values()), 0.8)
        if self.board_refresh:
            try:
                self._refresh_board()
            except Exception as exc:
                self.get_logger().warn(f'No pude volver a ubicar el pizarrón: {exc}')

    def _head_target(self):
        """(pan, tilt) para ver la columna de ArUco (abajo y arriba) más cercana a la cabeza."""
        try:
            head = self._lookup(self.base_frame, 'link_head_tilt')[:3, 3]
        except Exception:                           # sin TF de la cabeza: sobre la base, a camera_height
            head = np.array([0.0, 0.0, self.camera_height])
        T_board_base = self._lookup(self.board_frame, self.base_frame)
        u_head = (T_board_base[:3, :3] @ head + T_board_base[:3, 3])[0]
        columns = {}
        for u, v in self.layout.marker_positions().values():
            columns.setdefault(round(u, 4), []).append(v)
        u_col = min(columns, key=lambda u: abs(u - u_head))
        v_mid = (min(columns[u_col]) + max(columns[u_col])) / 2.0
        target = self._lookup(self.base_frame, self.board_frame) @ np.array([u_col, v_mid, 0.0, 1.0])
        d = target[:3] - head
        pan = math.atan2(d[1], d[0])
        tilt = math.atan2(d[2], math.hypot(d[0], d[1]))
        return pan, tilt, u_col

    def _refresh_board(self):
        """Mira la columna de ArUco más cercana y espera una corrección con suficientes marcadores."""
        self._status('ubicar')
        pan, tilt, u_col = self._head_target()
        q_pan, q_tilt = self.joints.get('joint_head_pan', pan), self.joints.get('joint_head_tilt', tilt)
        if abs(pan - q_pan) > 0.03 or abs(tilt - q_tilt) > 0.03:
            self._move(['joint_head_pan', 'joint_head_tilt'], (pan, tilt), 1.0)
        asked = self.get_clock().now()
        deadline = time.monotonic() + self.board_refresh_timeout
        while time.monotonic() < deadline and not self.stopped:
            m = self._board_msg
            if m is not None and m.fresh_markers >= self.board_refresh_min \
                    and Time.from_msg(m.fresh_stamp) >= asked:
                if m.correction_mm > 5.0 or abs(m.correction_deg) > 0.5:
                    self.get_logger().info(f'Pizarrón corregido con {m.fresh_markers} ArUco recientes: '
                                           f'{m.correction_mm:.0f} mm, {m.correction_deg:+.1f}° de deriva')
                return True
            time.sleep(0.05)
        m = self._board_msg
        self.get_logger().warn(
            f'No vi {self.board_refresh_min} ArUco en {self.board_refresh_timeout:.0f} s (columna u={u_col:.2f} m, '
            f'pan {pan:.2f}, tilt {tilt:.2f}; última corrección con {m.fresh_markers if m else 0}): '
            'pinto con la última ubicación del pizarrón')
        return False

    def _current_uv(self):
        mapper, tip = self._geometry()
        return mapper.base_to_board(tip)[:2]

    def _change_color(self, color):
        if self.color_client is None:
            self.color = color
            return
        if not self.color_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(f'No hay servicio de color ({self.color_service}); sigo con el mismo pincel')
            return
        future = self.color_client.call_async(SetBrushColor.Request(color=color))
        while not future.done():
            time.sleep(0.01)
        if future.result().success:
            self.color = color

    # --- Paro -------------------------------------------------------------------
    def _on_stop(self, msg):
        if msg.data and not self.stopped:
            self.stopped = True
            self._status('paro')
            self.get_logger().warn('PARO activado')
            if self.retract_on_stop:
                threading.Thread(target=self._retract_after_stop, daemon=True).start()
        elif not msg.data and self.stopped:
            self.stopped = False
            self._status('listo')
            self.get_logger().info('Paro liberado')

    def _retract_after_stop(self):
        time.sleep(0.3)   # deja que se cancele la trayectoria en curso
        if not self.pen_down:
            return
        try:
            mapper, tip = self._geometry()
            p = tip + mapper.T[:3, 2] * (self.hover + self.press)   # alejarse en la normal del mural
            s, lift, ext = mapper.joints_for(p)
            was_stopped, self.stopped = self.stopped, False
            self._move([self.lift_joint, self.ext_joint], (lift, ext), 1.0)
            self.stopped = was_stopped
            self._set_pen(False)
        except Exception as exc:
            self.get_logger().error(f'No pude levantar el pincel tras el paro: {exc}')

    # Velocidad media para los movimientos punto a punto. El driver rechaza la trayectoria entera si
    # una articulación pasa su límite (elevador: 0.15 m/s de media por tramo; 17-nov: 0.4 m en 2 s se rechazó).
    MOVE_SPEED = {'joint_lift': 0.10, 'wrist_extension': 0.10, 'joint_head_pan': 0.8, 'joint_head_tilt': 0.8,
                  'joint_wrist_yaw': 0.6, 'joint_wrist_pitch': 0.6, 'joint_wrist_roll': 0.6}

    def _move(self, names, target, minimum):
        """Va a target en al menos `minimum` s, más si la distancia lo pide para no pasar los límites."""
        t = minimum
        for n, q in zip(names, target):
            if n in self.joints and n in self.MOVE_SPEED:
                t = max(t, abs(float(q) - self.joints[n]) / self.MOVE_SPEED[n])
        return self._run_trajectory(names, [(t, tuple(target))])

    # --- Arranque -----------------------------------------------------------
    def _on_startup(self, request, response):
        ok, msg = self._startup_sequence()
        response.success, response.message = ok, msg
        return response

    def _startup_sequence(self):
        deadline = time.monotonic() + 30.0
        while not self.joints and time.monotonic() < deadline:
            time.sleep(0.2)
        if not self.joints:
            return False, 'No llegan /joint_states'
        self._status('pose inicial')
        names = list(self.ready_pose) + ['joint_head_pan', 'joint_head_tilt']
        self._move(names, list(self.ready_pose.values()) + list(self.head_work), 3.0)
        # Si la percepción todavía no procesa imágenes, el barrido se desperdicia: esperar su primera nube
        deadline = time.monotonic() + 30.0
        while self._perception_frames == 0 and time.monotonic() < deadline:
            time.sleep(0.2)
        if self._perception_frames == 0:
            self.get_logger().warn('La percepción no ha procesado imágenes: barro la cabeza de todos modos')
        self._status('buscando el mural')
        expected = self.board_markers_expected
        poses = ', '.join(f'({a:.2f}, {b:.2f})' for a, b in self.head_scan)
        self.get_logger().info(f'Barrido de cabeza (pan, tilt): {poses}')
        self._move([self.lift_joint, self.ext_joint], self.scan_pose, 2.0)
        for attempt in range(2):
            for pan, tilt in self.head_scan:
                self._move(['joint_head_pan', 'joint_head_tilt'], (pan, tilt), 1.2)
                time.sleep(self.head_scan_dwell)
            if self._board_markers >= expected:
                break
            if attempt == 0:
                self.get_logger().info(f'El barrido vio {self._board_markers} de {expected} marcadores: lo repito')
        self._move([self.lift_joint, self.ext_joint, 'joint_head_pan', 'joint_head_tilt'],
                   (self.ready_pose['joint_lift'], self.ready_pose['wrist_extension']) + tuple(self.head_work), 2.0)
        try:
            self._lookup(self.base_frame, self.board_frame)
            found = True
        except Exception:
            found = False
        msg = (f'Mural ubicado ({self._board_markers} de {expected} marcadores)' if found
               else 'No encontré el mural (revisa que se vean los ArUco)')
        if self.calibrate_on_startup and self.calib_client.wait_for_service(timeout_sec=3.0):
            self._status('calibrando pincel')
            future = self.calib_client.call_async(CalibrateTool.Request(samples=15))
            while not future.done():
                time.sleep(0.05)
            msg += ' | ' + future.result().message
        self._status('listo' if found else 'sin mural')
        self.get_logger().info(f'Arranque: {msg}')
        return found, msg


def main():
    rclpy.init()
    node = ControlNode()
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)   # un segundo Ctrl+C no interrumpe el cierre
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
