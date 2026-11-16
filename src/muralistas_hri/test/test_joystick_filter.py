import math
import random

from muralistas_hri.joystick_filter import (JoystickCursor, JoystickParams, OneEuroFilter, apply_deadzone,
                                            directional_gain)

DT = 0.01


def test_zona_muerta_sin_salto():
    assert apply_deadzone(0.05, 0.05, 0.10) == (0.0, 0.0)
    x, _ = apply_deadzone(0.1001, 0.0, 0.10)
    assert 0.0 < x < 0.001                     # justo al salir de la zona, casi cero
    x, _ = apply_deadzone(1.0, 0.0, 0.10)
    assert math.isclose(x, 1.0)                # el tope sigue siendo 1


def test_ganancia_por_direccion_interpola():
    gains = [2.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
    assert directional_gain(0.5, 0.0, gains) == (1.0, 0.0)      # derecha: ganancia 2
    x, y = directional_gain(0.0, 0.5, gains)                     # arriba: ganancia 1
    assert math.isclose(y, 0.5) and abs(x) < 1e-12
    x, _ = directional_gain(math.cos(math.pi / 8) * 0.5, math.sin(math.pi / 8) * 0.5, gains)
    assert math.isclose(x, math.cos(math.pi / 8) * 0.5 * 1.5)   # a medio sector: promedio


def test_one_euro_quita_temblor_lento_y_sigue_un_escalon():
    random.seed(0)
    f = OneEuroFilter(min_cutoff=0.5, beta=0.5)
    out = [f(0.3 + random.uniform(-0.05, 0.05), DT) for _ in range(300)]
    spread = max(out[100:]) - min(out[100:])
    assert spread < 0.03                       # el ruido de ±0.05 (rango 0.10) baja más de 70%
    f2 = OneEuroFilter(min_cutoff=0.5, beta=0.5)
    f2(0.0, DT)
    vals = [f2(1.0, DT) for _ in range(30)]    # escalón: 0.3 s después ya casi llegó
    assert vals[-1] > 0.9


def test_golpe_al_joystick_no_salta_el_cursor():
    p = JoystickParams(max_speed=0.18, max_accel=0.6, min_cutoff=10.0, beta=0.0)
    c = JoystickCursor(p, start=(0.30, 0.20))
    u0 = c.u
    c.update(1.0, 0.0, DT)                     # golpe a tope durante un solo ciclo
    c.update(0.0, 0.0, DT)
    assert abs(c.u - u0) < 0.001               # el cursor se movió menos de 1 mm


def test_velocidad_maxima_y_borde():
    p = JoystickParams(max_speed=0.18, max_accel=10.0, min_cutoff=10.0, beta=0.0)
    c = JoystickCursor(p, start=(0.0, 0.20))
    for _ in range(100):                       # 1 s a tope a la derecha
        c.update(1.0, 0.0, DT)
    assert 0.15 < c.u <= 0.18 + 1e-9           # no pasa de 0.18 m/s
    for _ in range(1000):
        c.update(1.0, 0.0, DT)
    assert c.u == p.area[0]                    # se queda en el borde de la zona
    for _ in range(5):                         # 50 ms: lo que tarda el filtro en cruzar el cero
        c.update(-1.0, 0.0, DT)
    assert c.u < p.area[0]                     # regresa sin velocidad acumulada contra el borde


def test_reposo_dentro_de_zona_muerta_no_mueve():
    random.seed(1)
    c = JoystickCursor(JoystickParams(deadzone=0.10), start=(0.30, 0.20))
    for _ in range(500):                       # deriva tipo Joy-Con, dentro de la zona muerta
        c.update(0.06 + random.uniform(-0.02, 0.02), random.uniform(-0.02, 0.02), DT)
    assert (c.u, c.v) == (0.30, 0.20)
