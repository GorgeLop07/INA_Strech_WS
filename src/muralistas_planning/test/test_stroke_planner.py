import math

import pytest

from muralistas_common.layout import MuralLayout
from muralistas_common.shapes import SHAPES
from muralistas_planning.stroke_planner import PlanError, plan_free_segment, plan_shape


def test_todas_las_figuras_caben_en_su_casilla():
    L = MuralLayout()
    for name in SHAPES:
        for row in range(L.rows):
            for col in range(L.cols):
                a, b, c, d = L.cell_rect(row, col)
                for stroke in plan_shape(L, name, row, col):
                    for u, v in stroke:
                        assert a - 1e-9 <= u <= c + 1e-9 and b - 1e-9 <= v <= d + 1e-9


def test_fila_cero_es_la_de_arriba():
    L = MuralLayout()
    top = L.cell_center(0, 0)[1]
    bottom = L.cell_center(L.rows - 1, 0)[1]
    assert top > bottom


def test_trazo_libre_llega_al_borde():
    L = MuralLayout()
    u0, v0, u1, v1 = L.drawable_rect()
    du, dv = plan_free_segment(L, (1.0, 0.5), 0.0)
    assert math.isclose(1.0 + du, u1) and math.isclose(dv, 0.0, abs_tol=1e-12)
    du, dv = plan_free_segment(L, (1.0, 0.5), 90.0)
    assert math.isclose(0.5 + dv, v1)


def test_trazo_libre_en_el_borde_se_rechaza():
    L = MuralLayout()
    u0, v0, u1, v1 = L.drawable_rect()
    with pytest.raises(PlanError):
        plan_free_segment(L, (u1, 0.5), 0.0)


def test_figura_desconocida():
    with pytest.raises(PlanError):
        plan_shape(MuralLayout(), 'dragon', 0, 0)


def test_fantasma_se_simplifica_redondea_y_queda_en_la_zona():
    from muralistas_planning.stroke_planner import process_ghost
    L = MuralLayout()
    u0, v0, u1, v1 = L.drawable_rect()
    raw = [(u0 + 0.1 + 0.001 * i, v0 + 0.1 + 0.0005 * (i % 3)) for i in range(200)] + [(u1 + 0.2, v0 + 0.1)]
    pts = process_ghost(L, raw)
    assert all(u0 - 1e-9 <= u <= u1 + 1e-9 and v0 - 1e-9 <= v <= v1 + 1e-9 for u, v in pts)
    assert all(math.dist(a, b) <= 0.0101 for a, b in zip(pts[:-1], pts[1:]))
    assert math.dist(pts[0], raw[0]) < 1e-9
    assert process_ghost(L, [(0.75, 0.5)]) == [(0.75, 0.5)]          # un solo punto: apoyar ahí


def test_tramo_libre_con_largo():
    L = MuralLayout()
    du, dv = plan_free_segment(L, (0.75, 0.5), 0.0, max_length=0.1)
    assert math.isclose(du, 0.1)
