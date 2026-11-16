"""Nodo de planeación: de "lo que eligió el niño" a trazos que el control puede pintar.

Entradas:  /hri/stroke_request (StrokeRequest), /control/tip_board (punta actual en el mural)
Salidas:   acción /control/draw_stroke (DrawStroke), /planning/busy, /planning/preview (RViz)

Una figura puede tener varios trazos (el sol, los puntos): se mandan uno tras otro,
levantando el pincel entre ellos. El trazo libre se manda como un tramo largo hasta el
borde y se cancela cuando el niño vuelve a presionar (o suelta, en modo "mantener").
El fantasma confirmado (joystick) se simplifica, se redondea y se remuestrea antes de
mandarlo: si estaba "levantado", el robot solo se desplaza; si "pegado", pinta.
"""

import signal
import threading
from collections import deque

import rclpy
from geometry_msgs.msg import Point, PointStamped
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker, MarkerArray

from muralistas_common.colors import rgb
from muralistas_common.layout import MuralLayout
from muralistas_interfaces.action import DrawStroke
from muralistas_interfaces.msg import StrokeRequest

from .stroke_planner import PlanError, plan_free_segment, plan_shape, process_ghost, validate_color


class PlannerNode(Node):
    def __init__(self):
        super().__init__('planner')
        p = self.declare_parameter
        self.layout = MuralLayout.from_dict(
            {k: p(f'mural.{k}', v).value for k, v in MuralLayout().__dict__.items()})
        self.draw_speed = p('draw_speed', 0.05).value
        self.free_speed = p('free_speed', 0.04).value
        # Un tramo libre no va hasta el borde del mural (puede estar a 3 m): se corta aquí y el niño
        # elige otra dirección. Así la base nunca se pide más allá de su alcance por trazo.
        self.free_max_length = p('free_max_length', 0.6).value
        self.board_frame = p('board_frame', 'board').value

        group = ReentrantCallbackGroup()
        self.client = ActionClient(self, DrawStroke, '/control/draw_stroke', callback_group=group)
        self.create_subscription(StrokeRequest, '/hri/stroke_request', self._on_request, 10, callback_group=group)
        self.create_subscription(PointStamped, '/control/tip_board', self._on_tip, 10, callback_group=group)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_busy = self.create_publisher(Bool, '/planning/busy', latched)
        self.pub_preview = self.create_publisher(MarkerArray, '/planning/preview', 2)

        self.lock = threading.Lock()
        self.queue = deque()           # goals pendientes
        self.active = None             # goal handle en curso
        self.tip_uv = None
        self._busy = None
        self._set_busy(False)
        self.get_logger().info('Planeación lista')

    # --- Entradas -----------------------------------------------------------
    def _on_tip(self, msg):
        self.tip_uv = (msg.point.x, msg.point.y)

    def _on_request(self, req):
        try:
            color = validate_color(req.color)
            if req.action == StrokeRequest.SHAPE:
                strokes = plan_shape(self.layout, req.shape, req.row, req.col)
                self._preview(strokes, color)
                goals = [self._goal(color, s, lift=True, speed=self.draw_speed) for s in strokes]
                self._enqueue(goals, replace=False)
                self.get_logger().info(f'Figura {req.shape} en ({req.row}, {req.col}): {len(goals)} trazos')
            elif req.action == StrokeRequest.MOVE_TO_CELL:
                u, v = self.layout.cell_center(req.row, req.col)
                self._enqueue([self._goal(color, [(u, v)], lift=False, speed=self.draw_speed)], replace=True)
            elif req.action == StrokeRequest.FREE_START:
                if self.tip_uv is None:
                    raise PlanError('Todavía no sé dónde está la punta del pincel')
                length = req.length if req.length > 0 else self.free_max_length
                du, dv = plan_free_segment(self.layout, self.tip_uv, req.direction_deg, length)
                goal = self._goal(color, [(du, dv)], lift=False, speed=req.speed or self.free_speed)
                goal.relative_to_current = True
                self._enqueue([goal], replace=True)
            elif req.action == StrokeRequest.FREE_STOP:
                self._cancel_all()
            elif req.action == StrokeRequest.PEN_UP:
                self._enqueue([self._goal(color, [], lift=True, speed=self.draw_speed)], replace=True)
            elif req.action == StrokeRequest.GHOST:
                pts = process_ghost(self.layout, [(p.x, p.y) for p in req.points])
                goal = self._goal(color, pts, lift=req.lift_at_end, speed=req.speed or self.draw_speed)
                goal.travel_only = not req.pen_down
                if req.pen_down:
                    self._preview([pts], color)
                self._enqueue([goal], replace=False)
                kind = 'pintar' if req.pen_down else 'desplazarse'
                self.get_logger().info(f'Fantasma ({kind}): {len(req.points)} puntos -> {len(pts)}')
            else:
                raise PlanError(f'Acción desconocida: {req.action}')
        except (PlanError, ValueError, KeyError) as exc:
            self.get_logger().warn(f'Pedido rechazado: {exc}')

    # --- Cola de trazos -----------------------------------------------------
    def _goal(self, color, points, lift, speed):
        g = DrawStroke.Goal()
        g.color = color
        g.points = [Point(x=float(u), y=float(v), z=0.0) for u, v in points]
        g.lift_at_end = lift
        g.speed = float(speed)
        return g

    def _enqueue(self, goals, replace):
        if replace:
            self._cancel_all()
        with self.lock:
            self.queue.extend(goals)
            start = self.active is None
        self._set_busy(True)
        if start:
            self._send_next()

    def _cancel_all(self):
        with self.lock:
            self.queue.clear()
            active = self.active
        if active is not None:
            active.cancel_goal_async()

    def _send_next(self):
        with self.lock:
            if not self.queue:
                self.active = None
                done = True
            else:
                goal = self.queue.popleft()
                done = False
        if done:
            self._set_busy(False)
            return
        if not self.client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('El control no responde (/control/draw_stroke)')
            with self.lock:
                self.queue.clear()
            self._set_busy(False)
            return
        future = self.client.send_goal_async(goal)
        future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn('El control rechazó el trazo')
            with self.lock:
                self.queue.clear()
                self.active = None
            self._set_busy(False)
            return
        with self.lock:
            self.active = handle
        handle.get_result_async().add_done_callback(self._on_result)

    def _on_result(self, future):
        result = future.result().result
        if not result.success and result.message:
            self.get_logger().info(f'Trazo terminado: {result.message}')
        with self.lock:
            self.active = None
        self._send_next()

    def _set_busy(self, busy):
        if busy != self._busy:
            self._busy = busy
            self.pub_busy.publish(Bool(data=busy))

    def _preview(self, strokes, color):
        arr = MarkerArray()
        arr.markers.append(Marker(action=Marker.DELETEALL))
        r, g, b = rgb(color)
        for i, s in enumerate(strokes):
            m = Marker(type=Marker.LINE_STRIP if len(s) > 1 else Marker.SPHERE_LIST, ns='plan', id=i)
            m.header.frame_id = self.board_frame
            m.header.stamp = self.get_clock().now().to_msg()
            m.scale.x = m.scale.y = m.scale.z = 0.01
            m.color.r, m.color.g, m.color.b, m.color.a = r, g, b, 0.35
            m.pose.orientation.w = 1.0
            m.points = [Point(x=float(u), y=float(v), z=0.01) for u, v in s]
            arr.markers.append(m)
        self.pub_preview.publish(arr)


def main():
    rclpy.init()
    node = PlannerNode()
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
