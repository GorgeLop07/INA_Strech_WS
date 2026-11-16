import math

import numpy as np

from muralistas_common.layout import MuralLayout
from muralistas_common.metrics import Progression, ldlj, msj, sparc, stop_stars, trace_metrics, trace_stars
from muralistas_common.templates import (STAGES, TEMPLATES, direction_name, nearest_on_polyline, resample,
                                         segments, stage_template, template_in_zone)

FS = 100.0


def _min_jerk(T=1.0, A=0.5):
    t = np.arange(0, T + 1e-9, 1 / FS) / T
    s = A * (10 * t ** 3 - 15 * t ** 4 + 6 * t ** 5)
    return np.c_[s, np.zeros_like(s)]


def test_ldlj_de_minimo_jerk_y_no_depende_de_tamano_ni_duracion():
    a = ldlj(_min_jerk(1.0, 0.5), FS)
    assert -6.0 < a < -4.8                                    # teórico: -ln(204.8) = -5.32
    assert abs(ldlj(_min_jerk(2.0, 0.2), FS) - a) < 0.3
    assert msj(_min_jerk(2.0, 0.2), FS) < msj(_min_jerk(1.0, 0.5), FS)   # el MSJ sí cambia


def test_movimiento_con_submovimientos_es_menos_suave():
    smooth = _min_jerk(1.0, 0.5)
    jerky = smooth.copy()
    jerky[:, 0] += 0.02 * np.sin(2 * np.pi * 4 * np.arange(len(jerky)) / FS)   # 4 Hz encima
    assert ldlj(jerky, FS) < ldlj(smooth, FS)
    assert sparc(jerky, FS) < sparc(smooth, FS)


def test_layout_esquinas_y_zona_de_trabajo():
    L = MuralLayout()
    assert set(L.marker_positions()) == {0, 1, 2, 3}
    u0, v0, u1, v1 = L.work_rect()
    assert math.isclose(u1 - u0, 0.60) and math.isclose(v1 - v0, 0.45)
    assert math.isclose((u0 + u1) / 2, 0.75) and math.isclose((v0 + v1) / 2, 0.5)
    a, b, c, d = L.cell_rect(0, 0)
    assert math.isclose(c - a, 0.15) and math.isclose(d - b, 0.15)       # casillas de 15 cm
    assert L.work_to_board(*L.board_to_work(0.9, 0.4)) == (0.9, 0.4)
    assert len(MuralLayout(markers='franjas', width=3.0).marker_positions()) == 14   # la simulación vieja


def test_plantillas_de_switch_solo_usan_las_8_direcciones():
    for name in ('linea', 'linea_vertical', 'cuadrado', 'octagono', 'casa'):
        for a, b in segments(TEMPLATES[name]):
            ang = math.degrees(math.atan2(b[1] - a[1], b[0] - a[0])) % 45.0
            assert min(ang, 45.0 - ang) < 1e-6, name
    assert stage_template(2, 0, switch=True) == 'octagono' and stage_template(2, 0) == 'circulo'
    assert [stage_template(0, i) for i in range(2)] == STAGES[0]
    assert direction_name((0, 0), (1, 1)) == 'arriba-derecha'


def test_plantilla_centrada_en_la_zona():
    L = MuralLayout()
    pts = template_in_zone('cuadrado', L)
    us, vs = [p[0] for p in pts], [p[1] for p in pts]
    assert math.isclose(max(us) - min(us), 0.30) and math.isclose((max(us) + min(us)) / 2, 0.75)


def test_resample_y_punto_mas_cercano():
    pts = resample([(0, 0), (1, 0)], 0.1)
    assert len(pts) == 11 and pts[-1] == (1, 0)
    q, d = nearest_on_polyline((0.5, 0.2), [(0, 0), (1, 0)])
    assert q == (0.5, 0.0) and math.isclose(d, 0.2)


def test_calificacion_del_trazo_y_del_paro():
    tmpl = [(0.0, 0.0), (0.3, 0.0)]
    err, cov = trace_metrics([(x / 100, 0.01) for x in range(31)], tmpl)
    assert abs(err - 0.01) < 1e-3 and cov == 1.0
    assert trace_stars(err, cov) in (2, 3)
    err, cov = trace_metrics([(x / 100, 0.0) for x in range(15)], tmpl)        # solo la mitad
    assert cov < 0.6 and trace_stars(err, cov) == 0
    assert stop_stars(0.005) == 3 and stop_stars(0.025) == 1 and stop_stars(0.05) == 0


def test_progresion_5_de_8():
    p = Progression(stage=0, level='alta')
    changes = [p.update(2) for _ in range(8)]
    assert changes[-1] == 'baja_asistencia' and p.level == 'media'
    for s in [2, 2, 0, 2, 0, 2, 0, 2]:
        change = p.update(s)
    assert change == 'baja_asistencia' and p.level == 'baja'
    for _ in range(8):
        change = p.update(3)
    assert change == 'sube_etapa' and p.stage == 1 and p.level == 'alta'
    p.level = 'media'
    for _ in range(3):
        change = p.update(0, coverage=0.3)
    assert change == 'sube_asistencia' and p.level == 'alta'
