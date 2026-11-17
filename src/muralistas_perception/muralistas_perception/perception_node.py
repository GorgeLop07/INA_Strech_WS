"""Nodo de percepción con la cámara de la cabeza (RGB-D).

Publica:
  /perception/points        nube 3D a color de lo que ve la cabeza (la "proyección 3D")
  /perception/board         pose del mural (muralistas_interfaces/Board) y TF odom -> board
  /perception/brush_points  puntos 3D del pincel segmentado
  /perception/markers       contorno y cuadrícula del mural, eje y punta del pincel (RViz)
  /perception/debug_image   imagen con los ArUco y el pincel marcados
  /perception/tool_offset   punta del pincel en el marco link_grasp_center (tras calibrar, latcheado)
Servicios:
  /perception/calibrate_tool   (muralistas_interfaces/CalibrateTool)
  /perception/reset_board      (std_srvs/Trigger)
"""

import signal
import threading
import time

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Point, Vector3Stamped
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformBroadcaster, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from muralistas_common.geometry import invert_transform, matrix_to_quat, transform_points
from muralistas_common.layout import MuralLayout
from muralistas_interfaces.msg import Board
from muralistas_interfaces.srv import CalibrateTool

from .board_estimator import ArucoDetector, BoardEstimator
from .brush_segmenter import BrushSegmenter
from .ros_utils import image_to_numpy, make_cloud, matrix_to_transform, numpy_to_image, transform_to_matrix


class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception')
        p = self.declare_parameter
        self.layout = MuralLayout.from_dict(
            {k: p(f'mural.{k}', v).value for k, v in MuralLayout().__dict__.items()})
        self.odom_frame = p('odom_frame', 'odom').value
        self.base_frame = p('base_frame', 'base_link').value
        self.grasp_frame = p('grasp_frame', 'link_grasp_center').value
        self.cloud_stride = p('cloud_stride', 3).value
        self.cloud_rate = p('cloud_rate', 2.0).value
        self.max_rate = p('max_rate', 5.0).value
        self.rotate_debug = p('rotate_debug_image', True).value   # la cámara de la cabeza va girada 90°
        # Color del cuerpo del pincel en HSV de OpenCV (H 0-179). Simulación: verde. Robot real: el
        # azul del plumón (config/perception_real.yaml).
        self.brush = BrushSegmenter(hsv_low=tuple(p('brush.hsv_low', [45, 120, 80]).value),
                                    hsv_high=tuple(p('brush.hsv_high', [80, 255, 255]).value),
                                    tip_beyond_body=p('brush.tip_beyond_body', 0.017).value,
                                    body_radius=p('brush.body_radius', 0.011).value,
                                    max_distance_from_gripper=p('brush.max_distance_from_gripper', 0.30).value)
        # Corrección por vistas recientes (ver board_estimator.py): si la base patina, el pizarrón
        # se recoloca con lo visto en los últimos fresh_age s, con fresh_min_markers ArUco o más
        self.board = BoardEstimator(self.layout, history=p('board.history', 30).value,
                                    fresh_age=p('board.fresh_age', 2.0).value,
                                    fresh_min_markers=p('board.fresh_min_markers', 2).value,
                                    max_correction_mm=p('board.max_correction_mm', 80.0).value,
                                    max_correction_deg=p('board.max_correction_deg', 4.0).value)
        # Cámara en movimiento (cabeza girando, base andando): la pose de la cámara y la imagen no
        # coinciden bien y las esquinas salen corridas centímetros. Esos cuadros no se usan para el pizarrón.
        self.still_window = p('board.still_window', 0.15).value       # s antes de la imagen
        self.still_max_deg = p('board.still_max_deg', 0.6).value
        self.still_max_m = p('board.still_max_m', 0.005).value
        self._last_rejected = None
        self.aruco = ArucoDetector()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_pub = TransformBroadcaster(self)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_cloud = self.create_publisher(PointCloud2, '/perception/points', 2)
        self.pub_brush = self.create_publisher(PointCloud2, '/perception/brush_points', 2)
        self.pub_board = self.create_publisher(Board, '/perception/board', latched)
        self.pub_markers = self.create_publisher(MarkerArray, '/perception/markers', 2)
        self.pub_debug = self.create_publisher(Image, '/perception/debug_image', 2)
        self.pub_offset = self.create_publisher(Vector3Stamped, '/perception/tool_offset', latched)

        image_group = MutuallyExclusiveCallbackGroup()
        color_topic = p('color_topic', '/camera/color/image_raw').value
        depth_topic = p('depth_topic', '/camera/aligned_depth_to_color/image_raw').value
        info_topic = p('info_topic', '/camera/color/camera_info').value
        subs = [Subscriber(self, Image, color_topic, qos_profile=qos_profile_sensor_data, callback_group=image_group),
                Subscriber(self, Image, depth_topic, qos_profile=qos_profile_sensor_data, callback_group=image_group),
                Subscriber(self, CameraInfo, info_topic, qos_profile=qos_profile_sensor_data, callback_group=image_group)]
        self.sync = ApproximateTimeSynchronizer(subs, queue_size=5, slop=0.05)
        self.sync.registerCallback(self._on_frame)

        services = ReentrantCallbackGroup()
        self.create_service(CalibrateTool, '/perception/calibrate_tool', self._on_calibrate, callback_group=services)
        self.create_service(Trigger, '/perception/reset_board', self._on_reset, callback_group=services)

        self._last_proc = 0.0
        self._last_cloud = 0.0
        self._calib_lock = threading.Lock()
        self._calib_samples = None
        self._T_odom_board = None
        self._board_lock = threading.Lock()
        self._last_plane = 0.0
        self.plane_period = p('board.plane_check_period', 2.0).value   # s entre verificaciones del plano
        self.create_timer(0.1, self._republish_board_tf)
        self.get_logger().info('Percepción lista: esperando imágenes de la cabeza')

    # --- Cuadros -----------------------------------------------------------
    def _on_frame(self, color_msg, depth_msg, info_msg):
        now = time.monotonic()
        if now - self._last_proc < 1.0 / self.max_rate:
            return
        self._last_proc = now
        stamp = color_msg.header.stamp
        cam_frame = color_msg.header.frame_id
        try:
            T_odom_cam = transform_to_matrix(self.tf_buffer.lookup_transform(
                self.odom_frame, cam_frame, Time.from_msg(stamp), Duration(seconds=0.2)))
            T_base_cam = transform_to_matrix(self.tf_buffer.lookup_transform(
                self.base_frame, cam_frame, Time.from_msg(stamp), Duration(seconds=0.2)))
            T_base_grasp = transform_to_matrix(self.tf_buffer.lookup_transform(
                self.base_frame, self.grasp_frame, Time.from_msg(stamp), Duration(seconds=0.2)))
        except Exception as exc:
            self.get_logger().warn(f'Sin TF para la cámara todavía: {exc}', throttle_duration_sec=5.0)
            return

        rgb = image_to_numpy(color_msg)
        depth = image_to_numpy(depth_msg)
        K = (info_msg.k[0], info_msg.k[4], info_msg.k[2], info_msg.k[5])
        gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)

        # 1) Mural: ArUco + profundidad, acumulado en odom (solo con la cámara quieta)
        detections = self.aruco.detect(gray)
        if detections and not self._camera_still(T_odom_cam, stamp, cam_frame):
            detections = {}
        if detections:
            with self._board_lock:      # /perception/reset_board corre en otro hilo
                self.board.add_frame(detections, depth, K, T_odom_cam, t=Time.from_msg(stamp).nanoseconds * 1e-9)
                T = self.board.fit()
                if T is not None:
                    self._T_odom_board = T
                    if now - self._last_plane > self.plane_period:   # verificación, no hace falta cada cuadro
                        self._last_plane = now
                        self.board.check_plane(depth, K, T_odom_cam)
                    self._publish_board(stamp)
                    if self.board.rejected is not None and self.board.rejected != self._last_rejected:
                        self._last_rejected = self.board.rejected
                        mm, deg, n = self.board.rejected
                        self.get_logger().warn(f'Corrección del pizarrón descartada ({mm:.0f} mm, {deg:+.1f}° con '
                                               f'{n} ArUco): demasiado grande para ser un desliz de la base')

        # 2) Nube 3D a color (lo que ve la cabeza)
        if now - self._last_cloud > 1.0 / self.cloud_rate:
            self._last_cloud = now
            s = self.cloud_stride
            d = depth[::s, ::s]
            rows, cols = np.nonzero(d > 0)
            z = d[rows, cols]
            fx, fy, cx, cy = K
            pts = np.stack([(cols * s - cx) * z / fx, (rows * s - cy) * z / fy, z], axis=1)
            self.pub_cloud.publish(make_cloud(pts, stamp, cam_frame, rgb[::s, ::s][rows, cols]))

        # 3) Pincel: segmentación en 3D
        brush = self.brush.estimate(rgb, depth, K, T_base_cam, T_base_grasp[:3, 3])
        markers = MarkerArray()
        if brush is not None:
            self.pub_brush.publish(make_cloud(brush['points'], stamp, self.base_frame))
            markers.markers += self._brush_markers(brush, stamp)
            with self._calib_lock:
                if self._calib_samples is not None:
                    tip_in_grasp = transform_points(invert_transform(T_base_grasp), brush['tip'])[0]
                    self._calib_samples.append(tip_in_grasp)
        if self._T_odom_board is not None:
            markers.markers += self._board_markers(stamp)
        if markers.markers:
            self.pub_markers.publish(markers)

        # 4) Imagen de depuración
        vis = rgb.copy()
        mask = self.brush.mask(rgb)
        vis[mask > 0] = (255, 0, 255)
        for marker_id, corners in detections.items():
            cv2.polylines(vis, [corners.astype(np.int32)], True, (255, 40, 40), 2)
            c = corners.mean(axis=0).astype(int)
            cv2.putText(vis, str(marker_id), tuple(c), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 40, 40), 1)
        if self.rotate_debug:
            vis = np.ascontiguousarray(np.rot90(vis, -1))
        self.pub_debug.publish(numpy_to_image(vis, stamp, cam_frame))

    def _camera_still(self, T_odom_cam, stamp, cam_frame):
        """True si la cámara casi no se movió en los still_window s antes de la imagen."""
        try:
            before = Time.from_msg(stamp) - Duration(seconds=self.still_window)
            T0 = transform_to_matrix(self.tf_buffer.lookup_transform(self.odom_frame, cam_frame, before))
        except Exception:
            return True                     # sin historia de TF todavía: no se puede saber
        dR = T0[:3, :3].T @ T_odom_cam[:3, :3]
        angle = np.degrees(np.arccos(np.clip((np.trace(dR) - 1) / 2, -1.0, 1.0)))
        return angle <= self.still_max_deg and np.linalg.norm(T_odom_cam[:3, 3] - T0[:3, 3]) <= self.still_max_m

    # --- Publicación ---------------------------------------------------------
    def _publish_board(self, stamp):
        T = self._T_odom_board
        msg = Board()
        msg.header.stamp = stamp
        msg.header.frame_id = self.odom_frame
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = map(float, T[:3, 3])
        x, y, z, w = matrix_to_quat(T[:3, :3])
        msg.pose.orientation.x, msg.pose.orientation.y = float(x), float(y)
        msg.pose.orientation.z, msg.pose.orientation.w = float(z), float(w)
        msg.width, msg.height = float(self.layout.width), float(self.layout.height)
        msg.markers_used = self.board.markers_used()
        msg.corners_used = self.board.corners_used()
        msg.rms_error = float(self.board.rms or 0.0)
        msg.plane_normal_angle_deg = float(self.board.plane_angle_deg or -1.0)
        msg.fresh_markers = self.board.fresh_markers
        if self.board.fresh_time is not None:
            msg.fresh_stamp = Time(nanoseconds=int(self.board.fresh_time * 1e9)).to_msg()
        msg.correction_mm, msg.correction_deg = self.board.correction_summary()
        self.pub_board.publish(msg)

    def _republish_board_tf(self):
        if self._T_odom_board is not None:
            self.tf_pub.sendTransform(matrix_to_transform(
                self._T_odom_board, self.get_clock().now().to_msg(), self.odom_frame, 'board'))

    def _board_markers(self, stamp):
        L = self.layout
        outline = Marker(type=Marker.LINE_STRIP, ns='mural', id=0)
        outline.header.frame_id, outline.header.stamp = 'board', stamp
        outline.scale.x = 0.006
        outline.color.r, outline.color.g, outline.color.b, outline.color.a = 0.1, 0.5, 1.0, 1.0
        outline.pose.orientation.w = 1.0
        outline.points = [Point(x=float(u), y=float(v), z=0.003) for u, v in
                          ((0, 0), (L.width, 0), (L.width, L.height), (0, L.height), (0, 0))]
        grid = Marker(type=Marker.LINE_LIST, ns='mural', id=1)
        grid.header.frame_id, grid.header.stamp = 'board', stamp
        grid.scale.x = 0.003
        grid.color.r, grid.color.g, grid.color.b, grid.color.a = 0.6, 0.6, 0.6, 0.8
        grid.pose.orientation.w = 1.0
        u0, v0, u1, v1 = L.drawable_rect()
        for c in range(L.cols + 1):
            u = u0 + (u1 - u0) * c / L.cols
            grid.points += [Point(x=float(u), y=float(v0), z=0.003), Point(x=float(u), y=float(v1), z=0.003)]
        for r in range(L.rows + 1):
            v = v0 + (v1 - v0) * r / L.rows
            grid.points += [Point(x=float(u0), y=float(v), z=0.003), Point(x=float(u1), y=float(v), z=0.003)]
        return [outline, grid]

    def _brush_markers(self, brush, stamp):
        axis = Marker(type=Marker.ARROW, ns='pincel', id=0)
        axis.header.frame_id, axis.header.stamp = self.base_frame, stamp
        axis.scale.x, axis.scale.y, axis.scale.z = 0.008, 0.016, 0.02
        axis.color.r, axis.color.g, axis.color.b, axis.color.a = 1.0, 0.0, 1.0, 1.0
        axis.pose.orientation.w = 1.0
        start = brush['center'] - brush['axis'] * 0.06
        axis.points = [Point(x=float(a), y=float(b), z=float(c)) for a, b, c in (start, brush['tip'])]
        tip = Marker(type=Marker.SPHERE, ns='pincel', id=1)
        tip.header.frame_id, tip.header.stamp = self.base_frame, stamp
        tip.scale.x = tip.scale.y = tip.scale.z = 0.015
        tip.color.r, tip.color.g, tip.color.b, tip.color.a = 1.0, 0.9, 0.0, 1.0
        tip.pose.position.x, tip.pose.position.y, tip.pose.position.z = map(float, brush['tip'])
        tip.pose.orientation.w = 1.0
        return [axis, tip]

    # --- Servicios ---------------------------------------------------------
    def _on_calibrate(self, request, response):
        n = max(3, request.samples or 15)
        with self._calib_lock:
            self._calib_samples = []
        deadline = time.monotonic() + 20.0
        while time.monotonic() < deadline:
            with self._calib_lock:
                if len(self._calib_samples) >= n:
                    break
            time.sleep(0.05)
        with self._calib_lock:
            samples = np.array(self._calib_samples)
            self._calib_samples = None
        if len(samples) < 3:
            response.success = False
            response.message = 'No vi el pincel: revisa que la cabeza mire hacia la pinza'
            return response
        offset = np.median(samples, axis=0)
        spread = float(np.median(np.linalg.norm(samples - offset, axis=1)))
        msg = Vector3Stamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.grasp_frame
        msg.vector.x, msg.vector.y, msg.vector.z = map(float, offset)
        self.pub_offset.publish(msg)
        response.success = True
        response.offset = msg.vector
        response.spread = spread
        response.points_used = int(len(samples))
        response.message = (f'Punta del pincel en {self.grasp_frame}: '
                            f'({offset[0]:.4f}, {offset[1]:.4f}, {offset[2]:.4f}) m, dispersión {1000 * spread:.1f} mm')
        self.get_logger().info(response.message)
        return response

    def _on_reset(self, request, response):
        with self._board_lock:
            self.board.reset()
            self._T_odom_board = None
        response.success, response.message = True, 'Pose del mural reiniciada'
        return response


def main():
    rclpy.init()
    node = PerceptionNode()
    executor = MultiThreadedExecutor(num_threads=3)
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
