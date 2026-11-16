"""Del joystick crudo al cursor del fantasma (sin ROS, para poder probarlo).

Cadena, en este orden:
  1. zona muerta radial: lo que cae dentro es cero; el resto se reescala para que el
     borde de la zona sea 0 y el tope del joystick sea 1 (sin salto al salir de la zona);
  2. ganancia por dirección: 8 ganancias (derecha, arriba-derecha, arriba, ... en sentido
     antihorario) interpoladas por ángulo; compensa el lado al que al niño le cuesta llegar;
  3. filtro One Euro (Casiez et al. 2012): suaviza mucho cuando el movimiento es lento, que
     es donde está el temblor, y casi no retrasa cuando es rápido e intencional;
  4. control de velocidad: la deflexión es la velocidad del cursor, con límite de velocidad
     y de aceleración, así un golpe al joystick nunca mueve el cursor de golpe;
  5. el cursor se integra y se queda dentro de la zona de trabajo.
"""

import math
from dataclasses import dataclass, field


def apply_deadzone(x, y, radius):
    """Zona muerta radial con reescalado. Devuelve (x, y) con magnitud en [0, 1]."""
    r = math.hypot(x, y)
    if r <= radius or r == 0.0:
        return 0.0, 0.0
    scaled = min(1.0, (r - radius) / (1.0 - radius))
    return x / r * scaled, y / r * scaled


def directional_gain(x, y, gains):
    """Multiplica (x, y) por la ganancia de su dirección, interpolando entre las 8 vecinas.

    gains[0] = derecha, gains[2] = arriba, gains[4] = izquierda, gains[6] = abajo.
    """
    if x == 0.0 and y == 0.0:
        return 0.0, 0.0
    sector = (math.atan2(y, x) % (2 * math.pi)) / (math.pi / 4)
    i = int(sector) % 8
    frac = sector - int(sector)
    g = gains[i] * (1.0 - frac) + gains[(i + 1) % 8] * frac
    return x * g, y * g


class OneEuroFilter:
    """Filtro One Euro de una señal escalar."""

    def __init__(self, min_cutoff=1.0, beta=0.0, d_cutoff=1.0):
        self.min_cutoff = min_cutoff
        self.beta = beta
        self.d_cutoff = d_cutoff
        self.x_prev = None
        self.dx_prev = 0.0

    @staticmethod
    def _alpha(cutoff, dt):
        tau = 1.0 / (2 * math.pi * cutoff)
        return 1.0 / (1.0 + tau / dt)

    def reset(self):
        self.x_prev = None
        self.dx_prev = 0.0

    def __call__(self, x, dt):
        if self.x_prev is None or dt <= 0.0:
            self.x_prev = x
            return x
        dx = (x - self.x_prev) / dt
        a_d = self._alpha(self.d_cutoff, dt)
        dx_hat = a_d * dx + (1 - a_d) * self.dx_prev
        cutoff = self.min_cutoff + self.beta * abs(dx_hat)
        a = self._alpha(cutoff, dt)
        x_hat = a * x + (1 - a) * self.x_prev
        self.x_prev, self.dx_prev = x_hat, dx_hat
        return x_hat


@dataclass
class JoystickParams:
    deadzone: float = 0.10
    gains: list = field(default_factory=lambda: [1.0] * 8)
    min_cutoff: float = 0.8          # Hz (One Euro)
    beta: float = 0.5
    max_speed: float = 0.18          # m/s del cursor en el pizarrón
    max_accel: float = 0.6           # m/s²
    area: tuple = (0.60, 0.45)       # ancho y alto de la zona de trabajo (m)


class JoystickCursor:
    """Convierte muestras del joystick en la posición del cursor dentro de la zona de trabajo.

    El origen es la esquina inferior izquierda de la zona; u a la derecha, v hacia arriba.
    """

    def __init__(self, params=None, start=None):
        self.p = params = params or JoystickParams()
        self.fx = OneEuroFilter(params.min_cutoff, params.beta)
        self.fy = OneEuroFilter(params.min_cutoff, params.beta)
        w, h = params.area
        self.u, self.v = start if start is not None else (w / 2, h / 2)
        self.vu = self.vv = 0.0
        self.filtered = (0.0, 0.0)

    def place(self, u, v):
        """Pone el cursor en (u, v), por ejemplo donde está la punta del marcador."""
        w, h = self.p.area
        self.u, self.v = min(max(u, 0.0), w), min(max(v, 0.0), h)
        self.vu = self.vv = 0.0

    def update(self, x, y, dt):
        """x, y: deflexión cruda en [-1, 1] (x a la derecha, y arriba). Devuelve (u, v)."""
        if dt <= 0.0:                 # primer ciclo: todavía no hay intervalo de tiempo
            return self.u, self.v
        x, y = apply_deadzone(x, y, self.p.deadzone)
        x, y = directional_gain(x, y, self.p.gains)
        r = math.hypot(x, y)
        if r > 1.0:
            x, y = x / r, y / r
        x, y = self.fx(x, dt), self.fy(y, dt)
        self.filtered = (x, y)

        # Velocidad deseada, limitada en magnitud y en cambio por paso (aceleración)
        tu, tv = x * self.p.max_speed, y * self.p.max_speed
        du, dv = tu - self.vu, tv - self.vv
        step = math.hypot(du, dv)
        max_step = self.p.max_accel * dt
        if step > max_step > 0.0:
            du, dv = du / step * max_step, dv / step * max_step
        self.vu += du
        self.vv += dv

        w, h = self.p.area
        self.u = min(max(self.u + self.vu * dt, 0.0), w)
        self.v = min(max(self.v + self.vv * dt, 0.0), h)
        # En el borde no se acumula velocidad contra la pared
        if self.u in (0.0, w):
            self.vu = 0.0
        if self.v in (0.0, h):
            self.vv = 0.0
        return self.u, self.v
