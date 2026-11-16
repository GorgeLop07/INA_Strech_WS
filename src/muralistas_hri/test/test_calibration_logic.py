import math

import numpy as np

from muralistas_hri import calibration_logic as cal

FS = 100.0


def test_switch_c1():
    trials = [{'rt': 0.6 + 0.05 * i, 'hold': 0.3, 'bounce': None} for i in range(9)] + [{'rt': None}]
    r = cal.switch_results(trials, false_presses=1)
    assert r['sin_respuesta'] == 1 and r['confiable']
    assert 1.0 <= r['periodo_barrido'] <= 5.0 and math.isclose(r['tiempo_aceptacion'], 0.15)
    # Dos presiones a 3.9 s no son rebote (es otra luz); a 0.12 s sí. El retardo no pasa de 1 s
    r = cal.switch_results([{'rt': 0.5, 'hold': 0.2, 'bounce': 3.9}] * 5, 0)
    assert r['retardo'] == 0.3
    r = cal.switch_results([{'rt': 0.5, 'hold': 0.2, 'bounce': 0.12}] * 5, 0)
    assert r['retardo'] == 0.3                                   # 2 x 0.12 = 0.24, sube al mínimo de 0.3
    assert cal.switch_results([{'rt': 0.5, 'hold': 0.2, 'bounce': 0.45}], 0)['retardo'] == 0.9
    r = cal.switch_results([{'rt': None}] * 5 + [{'rt': 0.5, 'hold': 0.2}] * 5, 0)
    assert not r['confiable']                                  # 50% sin respuesta


def test_reposo_c2_zona_muerta_cubre_temblor_y_deriva():
    rng = np.random.default_rng(0)
    xy = np.array([0.05, 0.0]) + rng.normal(0, 0.02, (1000, 2))
    r = cal.rest_results(xy)
    assert 0.03 < r['temblor'] < 0.07 and r['zona_muerta'] > r['temblor']


def _reach_trial(direction, amplitude, fs=FS, stuck=False):
    t = np.arange(0, 6.0, 1 / fs)
    a = math.radians(direction)
    s = np.clip(t / 1.0, 0, 1) if stuck else np.where(t < 4.0, np.clip(t / 1.0, 0, 1), np.clip(1 - (t - 4.0) / 0.5, 0, 1))
    s = s * s * (3 - 2 * s)                                    # perfil suave
    return direction, np.c_[amplitude * s * math.cos(a), amplitude * s * math.sin(a)]


def test_alcance_c3_ganancias_y_asignacion():
    trials = [_reach_trial(d, 0.9 if d != 180 else 0.5) for d in cal.DIRECTIONS for _ in range(2)]
    r = cal.reach_results(trials, FS, 0.1)
    assert r['direcciones_cortas'] == 0 and r['atorado'] == 0
    assert r['ganancias'][4] > r['ganancias'][0]               # a la izquierda le cuesta más: más ganancia
    d = cal.assign(None, {'temblor': 0.02}, r, {'error_relativo': 0.05}, two_hands_ok=True, macs=2)
    assert d['interaccion'] == 'J3' and d['asistencia'] == 'baja' and d['coincide_con_macs']
    d = cal.assign(None, {'temblor': 0.2}, r, {'error_relativo': 0.3}, two_hands_ok=False)
    assert d['interaccion'] == 'J2' and d['asistencia'] == 'alta'


def test_sin_alcance_es_modo_s_y_atorado_es_j1():
    weak = [_reach_trial(d, 0.2) for d in cal.DIRECTIONS]
    assert cal.assign(None, None, cal.reach_results(weak, FS, 0.1), None, False)['interaccion'] == 'S'
    stuck = [_reach_trial(d, 0.9, stuck=True) for d in cal.DIRECTIONS]
    r = cal.reach_results(stuck, FS, 0.1)
    assert r['atorado'] == 8
    assert cal.assign(None, None, r, None, True)['interaccion'] == 'J1'


def test_perfil_sugerido_trae_lo_calibrado():
    sw = cal.switch_results([{'rt': 0.8, 'hold': 0.3}] * 10, 0)
    rest = cal.rest_results(np.zeros((100, 2)) + 0.01)
    trials = [_reach_trial(d, 0.9) for d in cal.DIRECTIONS]
    reach = cal.reach_results(trials, FS, 0.1)
    dec = cal.assign(sw, rest, reach, {'error_relativo': 0.15}, two_hands_ok=False)
    prof = cal.suggested_profile('nino_07', sw, rest, reach, {'error_relativo': 0.15}, dec)
    assert prof['perfil']['ros__parameters']['interaccion'] == 'J2'
    assert prof['entrada_joystick']['ros__parameters']['zona_muerta'] == rest['zona_muerta']
    assert prof['scanning_menu']['ros__parameters']['scan_interval'] == sw['periodo_barrido']
    assert len(prof['entrada_joystick']['ros__parameters']['ganancias']) == 8
