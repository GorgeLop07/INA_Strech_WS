"""Prueba de articulaciones del Stretch real, una por una y con confirmación.

Primer paso de simulación -> robot físico: confirma que llegan /joint_states con
los nombres que usa el control y que cada articulación responde por
/stretch_controller/follow_joint_trajectory (driver en modo position).

Cada movimiento es pequeño, relativo a la posición actual, lento, y vuelve a
donde estaba. Antes de cada uno el script dice qué se va a mover y espera Enter.

Uso (en el robot, con el driver corriendo):
  python3 prueba_articulaciones.py --solo-leer          # no mueve nada
  python3 prueba_articulaciones.py                      # brazo, muñeca y cabeza
  python3 prueba_articulaciones.py --solo joint_lift    # una sola
  python3 prueba_articulaciones.py --pinza --base       # agrega pinza y base
  python3 prueba_articulaciones.py --trayectoria        # driver en modo trajectory
"""
import argparse
import math
import sys
import threading
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Transform
from trajectory_msgs.msg import JointTrajectoryPoint, MultiDOFJointTrajectoryPoint
from tf2_ros import Buffer, TransformListener

# (articulación, cambio, unidad, tolerancia, mínimo, máximo)
PRUEBAS = [
    ('joint_lift', 0.05, 'm', 0.01, 0.15, 1.05),
    ('wrist_extension', 0.05, 'm', 0.01, 0.0, 0.45),
    ('joint_wrist_yaw', 0.30, 'rad', 0.06, -1.3, 4.0),
    ('joint_wrist_pitch', -0.20, 'rad', 0.06, -1.4, 0.3),
    ('joint_wrist_roll', 0.30, 'rad', 0.06, -2.8, 2.8),
    ('joint_head_pan', 0.30, 'rad', 0.06, -3.9, 1.4),
    ('joint_head_tilt', -0.20, 'rad', 0.06, -1.5, 0.3),
]
PINZA = ('joint_gripper_finger_left', 0.15, 'rad', 0.10, -0.3, 0.5)
BASE_DELTA = 0.05  # m, translate_mobile_base (relativo, solo en modo position)
TRAYECTORIA_DELTA = 0.03  # m, prueba en modo trajectory
MARCOS_TF = ['link_grasp_center', 'link_gripper_fingertip_left', 'camera_color_optical_frame',
             'camera_depth_optical_frame']

# Lo que usa muralistas_control: si falta alguno, hay que ajustar control.yaml
NOMBRES_CONTROL = ['joint_lift', 'wrist_extension', 'joint_wrist_yaw', 'joint_wrist_pitch',
                   'joint_wrist_roll', 'joint_head_pan', 'joint_head_tilt']


class Prueba(Node):
    def __init__(self, action_name):
        super().__init__('prueba_articulaciones')
        self.joints = {}
        self.odom = None
        self.runstop = None
        self.modo = None
        self.lock = threading.Lock()
        self.create_subscription(JointState, '/joint_states', self._on_joints, 20)
        self.create_subscription(Odometry, '/odom', self._on_odom, 20)
        self.create_subscription(Bool, '/is_runstopped', lambda m: setattr(self, 'runstop', m.data), 5)
        self.create_subscription(String, '/mode', lambda m: setattr(self, 'modo', m.data), 5)
        self.client = ActionClient(self, FollowJointTrajectory, action_name)
        self.goal_handle = None
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)

    def _on_joints(self, msg):
        with self.lock:
            for n, q in zip(msg.name, msg.position):
                self.joints[n] = q
            # Igual que control_node: si no viene wrist_extension, es la suma de los 4 tramos
            if 'wrist_extension' not in msg.name and all(f'joint_arm_l{i}' in self.joints for i in range(4)):
                self.joints['wrist_extension'] = sum(self.joints[f'joint_arm_l{i}'] for i in range(4))

    def _on_odom(self, msg):
        p = msg.pose.pose.position
        self.odom = (p.x, p.y)

    def pos(self, name):
        with self.lock:
            return self.joints.get(name)

    def mover(self, names, positions, duracion):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(names)
        pt = JointTrajectoryPoint()
        pt.positions = [float(x) for x in positions]
        sec = int(duracion)
        pt.time_from_start = Duration(sec=sec, nanosec=int((duracion - sec) * 1e9))
        goal.trajectory.points = [pt]
        return self.enviar(goal, duracion)

    def enviar(self, goal, duracion):
        fut = self.client.send_goal_async(goal)
        if not _esperar(fut, 5.0):
            return False, 'el driver no respondió al goal'
        self.goal_handle = fut.result()
        if not self.goal_handle.accepted:
            return False, 'goal RECHAZADO por el driver (¿nombre de articulación o modo?)'
        res = self.goal_handle.get_result_async()
        if not _esperar(res, duracion + 10.0):
            self.cancelar()
            return False, 'no terminó a tiempo (cancelado)'
        self.goal_handle = None
        r = res.result().result
        if r.error_code != 0:
            return False, f'error_code={r.error_code} {r.error_string}'
        return True, 'ok'

    def cancelar(self):
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
            self.goal_handle = None


def _dur(t):
    sec = int(t)
    return Duration(sec=sec, nanosec=int((t - sec) * 1e9))


def _meta_trayectoria(q0, q1, base, duracion):
    """Igual que control_node en modo 'stretch': 2 puntos (el actual en t=0) y la base en multi_dof."""
    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = ['joint_lift', 'wrist_extension']
    goal.trajectory.points = [JointTrajectoryPoint(positions=list(q0), time_from_start=_dur(0.0)),
                              JointTrajectoryPoint(positions=list(q1), time_from_start=_dur(duracion))]
    goal.multi_dof_trajectory.joint_names = ['position']
    for t, x in [(0.0, 0.0), (duracion, base)]:
        tf = Transform()
        tf.translation.x = float(x)
        tf.rotation.w = 1.0
        goal.multi_dof_trajectory.points.append(MultiDOFJointTrajectoryPoint(transforms=[tf], time_from_start=_dur(t)))
    return goal


def probar_trayectoria(node, duracion):
    """Elevador, brazo y base a la vez, como un trazo del mural (driver en modo trajectory)."""
    d = TRAYECTORIA_DELTA
    lift0, ext0 = node.pos('joint_lift'), node.pos('wrist_extension')
    if lift0 is None or ext0 is None or node.odom is None:
        print('   faltan /joint_states del elevador o brazo, o /odom; se salta')
        return 'trayectoria', 'sin datos'
    print(f'\n-- trayectoria: elevador {lift0:.3f}->{lift0 + d:.3f} m, brazo {ext0:.3f}->{ext0 + d:.3f} m '
          f'y base +{d * 100:.0f} cm, todo junto en {duracion:.0f} s; luego regresa igual')
    print('   Revisa que haya espacio libre adelante y atrás del robot.')
    if not preguntar('   ¿Mover?'):
        return 'trayectoria', 'saltada'
    x0, y0 = node.odom
    ok, msg = node.enviar(_meta_trayectoria((lift0, ext0), (lift0 + d, ext0 + d), d, duracion), duracion)
    time.sleep(0.5)
    e_lift, e_ext = node.pos('joint_lift') - (lift0 + d), node.pos('wrist_extension') - (ext0 + d)
    avance = math.hypot(node.odom[0] - x0, node.odom[1] - y0)
    print(f'   ida    : {msg}; error elevador {e_lift:+.4f} m, brazo {e_ext:+.4f} m; base avanzó {avance * 100:.1f} cm')
    if not ok:
        return 'trayectoria', f'FALLA ida: {msg}'
    lift1, ext1 = node.pos('joint_lift'), node.pos('wrist_extension')
    ok2, msg2 = node.enviar(_meta_trayectoria((lift1, ext1), (lift0, ext0), -avance, duracion), duracion)
    time.sleep(0.5)
    regreso = math.hypot(node.odom[0] - x0, node.odom[1] - y0)
    print(f'   regreso: {msg2}; elevador {node.pos("joint_lift"):.3f} m, brazo {node.pos("wrist_extension"):.3f} m; '
          f'base a {regreso * 100:.1f} cm del inicio')
    paso = ok2 and abs(e_lift) < 0.01 and abs(e_ext) < 0.01 and abs(avance - d) < 0.01
    return 'trayectoria', ('OK' if paso else 'FALLA') + f' (elevador {e_lift:+.4f}, brazo {e_ext:+.4f} m, base {avance * 100:.1f} cm)'


def _esperar(fut, timeout):
    t0 = time.time()
    while not fut.done():
        if time.time() - t0 > timeout:
            return False
        time.sleep(0.02)
    return True


def preguntar(texto):
    """Enter = sí, s = saltar, q = salir."""
    r = input(f'{texto}  [Enter = mover, s = saltar, q = salir] ').strip().lower()
    if r == 'q':
        raise KeyboardInterrupt
    return r != 's'


def leer_estado(node):
    print('\n=== Estado del robot ===')
    print(f'  modo del driver : {node.modo or "(no llega /mode)"}')
    print(f'  paro (runstop)  : {node.runstop if node.runstop is not None else "(no llega /is_runstopped)"}')
    print(f'  odometría       : {"ok" if node.odom else "(no llega /odom)"}')
    with node.lock:
        nombres = sorted(node.joints)
    print(f'\n=== /joint_states ({len(nombres)} articulaciones) ===')
    for n in nombres:
        print(f'  {n:28s} {node.pos(n): .4f}')
    faltan = [n for n in NOMBRES_CONTROL if node.pos(n) is None]
    print('\n=== Nombres que usa muralistas_control ===')
    if faltan:
        print(f'  FALTAN: {faltan}  -> ajustar control.yaml antes de pintar')
    else:
        print('  todos presentes')
    # Marcos que usa el control (grasp_frame) y la percepción (cámara de la cabeza)
    print('\n=== Marcos TF (desde base_link) ===')
    for frame in MARCOS_TF:
        try:
            t = node.tf.lookup_transform('base_link', frame, rclpy.time.Time()).transform.translation
            print(f'  {frame:34s} x={t.x: .3f}  y={t.y: .3f}  z={t.z: .3f}')
        except Exception:
            print(f'  {frame:34s} NO EXISTE')
    return faltan


def probar_articulacion(node, prueba, duracion):
    name, delta, unidad, tol, lo, hi = prueba
    q0 = node.pos(name)
    if q0 is None:
        print(f'\n-- {name}: no viene en /joint_states, se salta')
        return name, 'no existe'
    # Si el cambio se sale de los límites, va para el otro lado
    if not lo <= q0 + delta <= hi:
        delta = -delta
    if not lo <= q0 + delta <= hi:
        print(f'\n-- {name}: está en {q0:.3f} {unidad}, fuera del rango seguro de prueba; se salta')
        return name, 'fuera de rango'
    destino = q0 + delta
    print(f'\n-- {name}: está en {q0:.3f} {unidad}; va a {destino:.3f} ({delta:+.2f}) en {duracion:.0f} s y regresa')
    if not preguntar('   ¿Mover?'):
        return name, 'saltada'
    ok, msg = node.mover([name], [destino], duracion)
    time.sleep(0.5)
    q1 = node.pos(name)
    err = abs(q1 - destino)
    print(f'   ida    : {msg}; llegó a {q1:.4f} (error {err:.4f} {unidad})')
    if not ok:
        return name, f'FALLA ida: {msg}'
    ok2, msg2 = node.mover([name], [q0], duracion)
    time.sleep(0.5)
    q2 = node.pos(name)
    err2 = abs(q2 - q0)
    print(f'   regreso: {msg2}; quedó en {q2:.4f} (error {err2:.4f} {unidad})')
    if not ok2:
        return name, f'FALLA regreso: {msg2}'
    paso = err <= tol and err2 <= tol
    return name, ('OK' if paso else 'FALLA') + f' (error ida {err:.4f}, regreso {err2:.4f} {unidad})'


def probar_base(node, duracion):
    print(f'\n-- base: avanza {BASE_DELTA * 100:.0f} cm y regresa (translate_mobile_base, modo position)')
    print('   Revisa que haya espacio libre adelante y atrás del robot.')
    if node.odom is None:
        print('   no llega /odom; se salta')
        return 'base', 'sin /odom'
    if not preguntar('   ¿Mover la base?'):
        return 'base', 'saltada'
    x0, y0 = node.odom
    ok, msg = node.mover(['translate_mobile_base'], [BASE_DELTA], duracion)
    time.sleep(0.5)
    d = math.hypot(node.odom[0] - x0, node.odom[1] - y0)
    print(f'   ida    : {msg}; avanzó {d * 100:.1f} cm según /odom')
    if not ok:
        return 'base', f'FALLA ida: {msg}'
    ok2, msg2 = node.mover(['translate_mobile_base'], [-BASE_DELTA], duracion)
    time.sleep(0.5)
    d2 = math.hypot(node.odom[0] - x0, node.odom[1] - y0)
    print(f'   regreso: {msg2}; quedó a {d2 * 100:.1f} cm del inicio')
    paso = ok2 and abs(d - BASE_DELTA) < 0.01 and d2 < 0.01
    return 'base', ('OK' if paso else 'FALLA') + f' (avanzó {d * 100:.1f} cm, quedó a {d2 * 100:.1f} cm)'


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--solo-leer', action='store_true', help='solo imprime el estado; no mueve nada')
    ap.add_argument('--solo', nargs='+', metavar='JOINT', help='probar solo estas articulaciones')
    ap.add_argument('--pinza', action='store_true', help='incluir la pinza (abre un poco: ¡sin marcador!)')
    ap.add_argument('--base', action='store_true', help='incluir la base (avanza 5 cm y regresa)')
    ap.add_argument('--trayectoria', action='store_true',
                    help='driver en modo trajectory: elevador, brazo y base juntos, como control_node')
    ap.add_argument('--duracion', type=float, default=3.0, help='segundos por movimiento (default 3)')
    ap.add_argument('--action', default='/stretch_controller/follow_joint_trajectory')
    args = ap.parse_args()

    rclpy.init()
    node = Prueba(args.action)
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()

    resultados = []
    try:
        print('Esperando /joint_states ...')
        t0 = time.time()
        while not node.joints and time.time() - t0 < 10.0:
            time.sleep(0.1)
        if not node.joints:
            print('No llegan /joint_states en 10 s. ¿Está corriendo el driver (stretch_driver.launch.py)?')
            return 1
        time.sleep(1.0)  # que lleguen /mode, /odom y /is_runstopped
        leer_estado(node)
        if args.solo_leer:
            return 0

        if node.runstop:
            print('\nEl robot está en PARO (runstop). Suéltalo antes de probar.')
            return 1
        print(f'\nEsperando la acción {args.action} ...')
        if not node.client.wait_for_server(timeout_sec=10.0):
            print('La acción no está disponible. ¿El driver está en modo position?')
            return 1

        if args.trayectoria:
            print('\nTen a la mano el botón de paro del robot. Ctrl+C cancela el movimiento.')
            resultados.append(probar_trayectoria(node, max(args.duracion, 4.0)))
            return 0

        pruebas = list(PRUEBAS) + ([PINZA] if args.pinza else [])
        if args.solo:
            pruebas = [p for p in pruebas + [PINZA] if p[0] in args.solo]
        print('\nTen a la mano el botón de paro del robot. Ctrl+C cancela el movimiento.')
        for p in pruebas:
            resultados.append(probar_articulacion(node, p, args.duracion))
        if args.base or (args.solo and 'base' in args.solo):
            resultados.append(probar_base(node, args.duracion))
    except KeyboardInterrupt:
        node.cancelar()
        print('\nCancelado.')
    finally:
        if resultados:
            print('\n=== Resumen ===')
            for name, r in resultados:
                print(f'  {name:28s} {r}')
        node.cancelar()
        time.sleep(0.2)
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
