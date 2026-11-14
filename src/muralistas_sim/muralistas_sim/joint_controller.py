"""Sigue trayectorias de articulaciones como el driver real del Stretch (FollowJointTrajectory).

Nombres de articulaciones aceptados (los mismos del driver de stretch_ros2):
  joint_lift, wrist_extension, joint_wrist_yaw, joint_wrist_pitch, joint_wrist_roll,
  joint_head_pan, joint_head_tilt, joint_gripper_finger_left,
  joint_mobile_base_translation  (desplazamiento de la base, relativo al inicio de la trayectoria).

Todo corre dentro del hilo de la física: set_trajectory() y cancel() solo guardan el pedido
y update() aplica los controles antes de cada paso.
"""

import math

import mujoco
import numpy as np

from .scene_builder import free_joint_qposadr

POSITION_ACTUATORS = {
    'joint_lift': 'lift',
    'wrist_extension': 'arm',
    'joint_arm': 'arm',
    'joint_wrist_yaw': 'wrist_yaw',
    'joint_wrist_pitch': 'wrist_pitch',
    'joint_wrist_roll': 'wrist_roll',
    'joint_head_pan': 'head_pan',
    'joint_head_tilt': 'head_tilt',
    'joint_gripper_finger_left': 'gripper',
}
BASE_JOINT = 'joint_mobile_base_translation'

# Los actuadores del MJCF para elevador y brazo son solo proporcionales: dejan ~5 mm de error
# estable y ~1.7 cm de retraso a 5 cm/s. Los motores del Stretch real tienen su propio lazo
# cerrado, así que aquí se agrega prealimentación de velocidad + integral para imitarlos.
SERVO_JOINTS = ('joint_lift', 'wrist_extension')

WHEEL_RADIUS = 0.1016 / 2.0      # mismos valores que stretch_mujoco/config.py
WHEEL_SEPARATION = 0.3153


def _wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class StretchJointController:
    def __init__(self, model, data, base_kp=4.0, yaw_kp=4.0, max_base_speed=0.3):
        self.m = model
        self.d = data
        self.act = {name: model.actuator(name).id for name in set(POSITION_ACTUATORS.values())}
        self.wheels = (model.actuator('left_wheel_vel').id, model.actuator('right_wheel_vel').id)
        self.wheel_gear = float(model.actuator_gear[self.wheels[0], 0])
        self.free = free_joint_qposadr(model)
        self.base_kp = base_kp
        self.yaw_kp = yaw_kp
        self.max_base_speed = max_base_speed
        self._traj = None
        self._cmd_vel = (0.0, 0.0)
        self._cmd_vel_until = -1.0
        self.status = 'idle'            # idle | running | done | canceled
        self.hold_yaw = self.base_yaw()
        self.servos = {name: self._servo_params(name) for name in SERVO_JOINTS}

    def _servo_params(self, name):
        """Ganancias del lazo que imita a los motores del Stretch real (ver SERVO_JOINTS)."""
        aid = self.act[POSITION_ACTUATORS[name]]
        kp = float(self.m.actuator_gainprm[aid, 0])
        kv = -float(self.m.actuator_biasprm[aid, 2])
        if name == 'joint_lift':
            damping = float(self.m.dof_damping[self.m.joint('joint_lift').dofadr[0]])
        else:   # el brazo son 4 eslabones en serie movidos por un tendón: amortiguamiento equivalente
            d = [float(self.m.dof_damping[self.m.joint(f'joint_arm_l{i}').dofadr[0]]) for i in range(4)]
            damping = sum(d) / len(d) ** 2
        return {'kff': (kv + damping) / kp, 'ki': 4.0, 'limit': 0.03, 'i': 0.0, 'target': None, 'vel': 0.0}

    # --- Lecturas -----------------------------------------------------------
    def joint_position(self, name):
        if name == 'wrist_extension' or name == 'joint_arm':
            return sum(self.d.qpos[self.m.joint(f'joint_arm_l{i}').qposadr[0]] for i in range(4))
        if name == 'joint_gripper_finger_left':
            return float(self.d.qpos[self.m.joint('joint_gripper_slide').qposadr[0]])
        return float(self.d.qpos[self.m.joint(name).qposadr[0]])

    def base_xy(self):
        return np.array(self.d.qpos[self.free:self.free + 2])

    def base_yaw(self):
        w, x, y, z = self.d.qpos[self.free + 3:self.free + 7]
        return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))

    # --- Pedidos (desde otros hilos, protegidos por el candado del nodo) ----
    def set_trajectory(self, names, points):
        """points: lista de (t, [posiciones]) con t en segundos desde el inicio."""
        for n in names:
            if n not in POSITION_ACTUATORS and n != BASE_JOINT:
                raise ValueError(f'Articulación no soportada: {n}')
        times = [p[0] for p in points]
        if any(b <= a for a, b in zip(times[:-1], times[1:])):
            raise ValueError('Los tiempos de la trayectoria deben ser crecientes')
        start = []
        for n in names:
            start.append(0.0 if n == BASE_JOINT else self.joint_position(n))
        pts = list(points)
        if pts[0][0] > 1e-6:
            pts.insert(0, (0.0, start))
        self._traj = {
            'names': list(names),
            't': np.array([p[0] for p in pts]),
            'q': np.array([p[1] for p in pts], dtype=float),
            't0': self.d.time,
            'base_xy0': self.base_xy(),
            'yaw0': self.base_yaw(),
        }
        self.hold_yaw = self._traj['yaw0']
        self.status = 'running'

    def cancel(self):
        """Se queda donde va: fija como objetivo la posición actual."""
        if self._traj is None:
            return
        for n in self._traj['names']:
            if n in self.servos:
                self.servos[n]['target'] = self.joint_position(n)
                self.servos[n]['vel'] = 0.0
            elif n in POSITION_ACTUATORS:
                self.d.ctrl[self.act[POSITION_ACTUATORS[n]]] = self.joint_position(n)
        self._traj = None
        self._set_wheels(0.0, 0.0)
        self.status = 'canceled'

    def set_cmd_vel(self, v, w, duration=0.5):
        if self._traj is not None:
            return
        self._cmd_vel = (v, w)
        self._cmd_vel_until = self.d.time + duration
        self.hold_yaw = None

    def progress(self):
        if self._traj is None:
            return 1.0
        t = self.d.time - self._traj['t0']
        return float(min(1.0, t / max(self._traj['t'][-1], 1e-6)))

    # --- Paso de control (hilo de la física) -------------------------------
    def _update_servos(self):
        dt = self.m.opt.timestep
        for name, s in self.servos.items():
            aid = self.act[POSITION_ACTUATORS[name]]
            if s['target'] is None:            # primera vez: tomar el control inicial como objetivo
                s['target'] = float(self.d.ctrl[aid])
            q = self.joint_position(name)
            s['i'] = float(np.clip(s['i'] + s['ki'] * (s['target'] - q) * dt, -s['limit'], s['limit']))
            lo, hi = self.m.actuator_ctrlrange[aid]
            self.d.ctrl[aid] = float(np.clip(s['target'] + s['kff'] * s['vel'] + s['i'], lo, hi))

    def update(self):
        tr = self._traj
        if tr is None:
            self._update_servos()
            if self.d.time < self._cmd_vel_until:
                self._set_wheels(*self._cmd_vel)
            elif self.hold_yaw is None:
                self.hold_yaw = self.base_yaw()
                self._set_wheels(0.0, 0.0)
            else:
                self._set_wheels(0.0, 0.0)
            return

        t = self.d.time - tr['t0']
        q = np.array([np.interp(t, tr['t'], tr['q'][:, j]) for j in range(len(tr['names']))])
        for j, (name, value) in enumerate(zip(tr['names'], q)):
            if name in self.servos:
                self.servos[name]['target'] = float(value)
                ahead = np.interp(t + 0.02, tr['t'], tr['q'][:, j])
                self.servos[name]['vel'] = float((ahead - value) / 0.02) if t + 0.02 <= tr['t'][-1] else 0.0
            elif name in POSITION_ACTUATORS:
                self.d.ctrl[self.act[POSITION_ACTUATORS[name]]] = value
        self._update_servos()

        if BASE_JOINT in tr['names']:
            j = tr['names'].index(BASE_JOINT)
            dt = 0.02
            s_des = q[j]
            s_next = np.interp(t + dt, tr['t'], tr['q'][:, j])
            heading = np.array([math.cos(tr['yaw0']), math.sin(tr['yaw0'])])
            s_meas = float((self.base_xy() - tr['base_xy0']) @ heading)
            v = (s_next - s_des) / dt + self.base_kp * (s_des - s_meas)
            v = float(np.clip(v, -self.max_base_speed, self.max_base_speed))
            w = self.yaw_kp * _wrap(tr['yaw0'] - self.base_yaw())
            self._set_wheels(v, w)
        else:
            self._set_wheels(0.0, 0.0)

        if t > tr['t'][-1] + 0.4:
            self._traj = None
            self._set_wheels(0.0, 0.0)
            self.status = 'done'

    def _set_wheels(self, v, w):
        if self.hold_yaw is not None and self._traj is None and abs(v) < 1e-9 and abs(w) < 1e-9:
            w = self.yaw_kp * _wrap(self.hold_yaw - self.base_yaw())
        wl = (v - w * WHEEL_SEPARATION / 2.0) / WHEEL_RADIUS
        wr = (v + w * WHEEL_SEPARATION / 2.0) / WHEEL_RADIUS
        self.d.ctrl[self.wheels[0]] = wl * self.wheel_gear
        self.d.ctrl[self.wheels[1]] = wr * self.wheel_gear
