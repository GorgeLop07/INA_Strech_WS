import math

import numpy as np

from muralistas_common.geometry import make_transform, transform_points
from muralistas_common.layout import MuralLayout
from muralistas_perception.board_estimator import BoardEstimator, marker_corner_coords, planar_correction

# Pizarrón del cristal del Lab 345 (config/mural.yaml), visto desde odom como en la simulación
L = MuralLayout(width=2.095, height=1.01, bottom_height=0.1925, marker_size=0.155)
T_TRUE = make_transform(np.array([[-1.0, 0, 0], [0, 0, 1.0], [0, 1.0, 0]]), [1.5, -0.75, 0.19])


def drift(dx, dy, yaw_deg):
    a = math.radians(yaw_deg)
    R = np.array([[math.cos(a), -math.sin(a), 0], [math.sin(a), math.cos(a), 0], [0, 0, 1.0]])
    return make_transform(R, [dx, dy, 0.0])


def see(est, T_odom_board, ids, t0, frames=5, noise=0.0, rng=None):
    for f in range(frames):
        for i in ids:
            u, v = L.marker_positions()[i]
            for k, c in enumerate(marker_corner_coords(u, v, L.marker_size)):
                p = transform_points(T_odom_board, c)[0]
                if noise:
                    p = p + rng.normal(0, noise, 3)
                est.add_corner(i, k, p, t0 + 0.1 * f)
        est.fit()


def zone_error(T_est, T_ref):
    u0, v0, u1, v1 = L.work_rect()
    pts = np.array([[u, v, 0.0] for u in (u0, u1) for v in (v0, v1)])
    return float(np.max(np.linalg.norm(transform_points(T_est, pts) - transform_points(T_ref, pts), axis=1)))


def test_planar_correction_recupera_el_desliz():
    D = drift(0.02, -0.01, 1.5)
    src = np.random.default_rng(0).normal(size=(8, 3))
    C = planar_correction(src, transform_points(D, src))
    assert np.allclose(C, D, atol=1e-9)


def test_corrige_el_desliz_con_dos_marcadores_recientes():
    est = BoardEstimator(L, fresh_age=2.0, fresh_min_markers=2)
    see(est, T_TRUE, range(6), t0=0.0)                     # barrido de arranque: los 6
    assert zone_error(est.T_odom_board, T_TRUE) < 1e-6
    D = drift(0.03, 0.01, 1.0)                              # la base patinó: 3 cm y 1° que odom no vio
    see(est, D @ T_TRUE, [1, 4], t0=10.0)                   # luego solo ve la columna del centro
    assert est.fresh_markers == 2
    assert zone_error(est.T_odom_board, D @ T_TRUE) < 1e-3  # la zona de trabajo queda bien (< 1 mm)
    mm, deg = est.correction_summary()                     # lo acumulado ya absorbió parte del desliz
    assert 0 < mm <= 40 and 0 < deg <= 1.0 + 1e-6


def test_con_un_solo_marcador_reciente_no_corrige():
    est = BoardEstimator(L, fresh_age=2.0, fresh_min_markers=2)
    see(est, T_TRUE, range(6), t0=0.0)
    before = est.correction.copy()
    see(est, drift(0.03, 0.0, 0.0) @ T_TRUE, [1], t0=10.0)
    assert np.allclose(est.correction, before)


def test_muchas_vistas_despues_del_desliz_siguen_bien():
    est = BoardEstimator(L, fresh_age=2.0, fresh_min_markers=2)
    see(est, T_TRUE, range(6), t0=0.0)
    D = drift(0.03, 0.01, 1.0)
    see(est, D @ T_TRUE, [1, 4], t0=10.0, frames=40)        # lo acumulado de 1 y 4 ya es todo nuevo
    assert zone_error(est.T_odom_board, D @ T_TRUE) < 2e-3
    see(est, D @ T_TRUE, [2, 3], t0=20.0)                   # cambia a la otra columna: sigue bien
    assert zone_error(est.T_odom_board, D @ T_TRUE) < 2e-3


def test_descarta_una_correccion_enorme():
    est = BoardEstimator(L, fresh_age=2.0, fresh_min_markers=2, max_correction_mm=80.0)
    see(est, T_TRUE, range(6), t0=0.0)
    see(est, drift(0.236, 0.0, 0.9) @ T_TRUE, [1, 4], t0=10.0, frames=2)   # lo del 17-nov: 236 mm
    assert np.allclose(est.correction, np.eye(4))
    assert est.rejected is not None and est.rejected[0] > 200


def test_con_ruido_queda_en_milimetros():
    rng = np.random.default_rng(1)
    est = BoardEstimator(L, fresh_age=2.0, fresh_min_markers=2)
    see(est, T_TRUE, range(6), t0=0.0, frames=20, noise=0.003, rng=rng)
    D = drift(0.02, 0.0, 0.5)
    see(est, D @ T_TRUE, [1, 4], t0=10.0, frames=10, noise=0.003, rng=rng)
    assert zone_error(est.T_odom_board, D @ T_TRUE) < 0.006
