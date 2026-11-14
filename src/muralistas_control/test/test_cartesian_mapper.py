import numpy as np

from muralistas_control.cartesian_mapper import CartesianMapper, densify, path_length

# Mural como en la simulación: x del mural = -x de la base, y = z, z (sale de la pared) = +y.
R = np.array([[-1.0, 0, 0], [0, 0, 1.0], [0, 1.0, 0]])


def make_mapper(tip=(0.0, -0.55, 0.7), lift=0.6, ext=0.05):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [1.5, -0.75, 0.18]
    return CartesianMapper(T, np.array(tip), lift, ext)


def test_reparte_el_desplazamiento_en_base_elevador_y_brazo():
    m = make_mapper()
    target = m.tip0 + np.array([0.10, -0.20, 0.05])
    s, lift, ext = m.joints_for(target)
    assert np.isclose(s, 0.10) and np.isclose(lift, 0.65) and np.isclose(ext, 0.25)


def test_ida_y_vuelta_mural_base():
    m = make_mapper()
    p = m.board_to_base(1.2, 0.5, 0.01)
    assert np.allclose(m.base_to_board(p), [1.2, 0.5, 0.01])


def test_apoyar_es_extender_el_brazo():
    m = make_mapper()
    _, _, ext_hover = m.joints_for(m.board_to_base(1.5, 0.5, 0.03))
    _, _, ext_press = m.joints_for(m.board_to_base(1.5, 0.5, -0.008))
    assert np.isclose(ext_press - ext_hover, 0.038)


def test_limites():
    m = make_mapper()
    assert m.check((0.0, 1.5, 0.2))      # elevador fuera de rango
    assert not m.check((0.0, 0.6, 0.2))


def test_densify_y_longitud():
    pts = densify([(0, 0), (0.1, 0)], step=0.01)
    assert len(pts) == 11
    assert np.isclose(path_length(pts), 0.1)


def test_tiempos_con_velocidad():
    m = make_mapper()
    traj = m.trajectory([(1.5, 0.5), (1.4, 0.5)], w=0.0, speed=0.05)
    assert np.isclose(traj[1][0] - traj[0][0], 0.1 / 0.05)
