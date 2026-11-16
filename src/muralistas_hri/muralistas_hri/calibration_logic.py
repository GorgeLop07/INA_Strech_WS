"""Cálculos de la calibración C1 a C7 (sin ROS, para poder probarlos).

La pantalla de calibración (calibration_node) junta los datos; aquí se convierten en el
modo de interacción, el nivel de asistencia y los parámetros del perfil. Los umbrales son los
de la planeación, iniciales: se ajustan en el piloto con la terapeuta.
"""

import math

import numpy as np

from muralistas_common.assistance import preset
from muralistas_common.metrics import ldlj, segment_by_radius, smoothness, sparc

DIRECTIONS = [0, 45, 90, 135, 180, 225, 270, 315]
BOUNCE_MAX = 0.5      # s: una segunda presión antes de esto cuenta como rebote del switch


def _clamp(x, lo, hi):
    return min(max(x, lo), hi)


def switch_results(trials, false_presses):
    """C1. trials: [{'rt': s o None, 'hold': s o None, 'bounce': s más corto entre presiones o None}]."""
    rts = [t['rt'] for t in trials if t.get('rt') is not None]
    holds = [t['hold'] for t in trials if t.get('hold') is not None]
    # Rebote = otra presión muy poco después de la primera (< 0.5 s); más tarde ya es otra intención
    bounces = [t['bounce'] for t in trials if t.get('bounce') is not None and t['bounce'] < BOUNCE_MAX]
    failed = sum(1 for t in trials if t.get('rt') is None)
    out = {'ensayos': len(trials), 'sin_respuesta': failed, 'presiones_de_mas': false_presses,
           'rt_mediana_s': float(np.median(rts)) if rts else None,
           'rt_p90_s': float(np.percentile(rts, 90)) if rts else None}
    out['periodo_barrido'] = round(_clamp(1.5 * out['rt_p90_s'], 1.0, 5.0), 2) if rts else 3.0
    out['tiempo_aceptacion'] = round(_clamp(0.5 * float(np.median(holds)), 0.05, 0.5), 3) if holds else 0.10
    out['retardo'] = round(_clamp(2 * max(bounces), 0.3, 1.0) if bounces else 0.3, 3)
    out['confiable'] = len(trials) > 0 and failed / len(trials) <= 0.3
    return out


def rest_results(xy):
    """C2. xy: deflexión cruda (N, 2) con la mano en el joystick sin moverlo."""
    xy = np.asarray(xy, dtype=float)
    if len(xy) == 0:
        return {'deriva': 0.0, 'temblor': 0.0, 'zona_muerta': 0.10}
    mean = xy.mean(axis=0)
    tremor = float(np.percentile(np.linalg.norm(xy - mean, axis=1), 95))
    return {'deriva': float(np.linalg.norm(mean)), 'temblor': tremor,
            'zona_muerta': round(_clamp(1.5 * tremor + float(np.linalg.norm(mean)), 0.05, 0.35), 3)}


def reach_results(trials, fs, deadzone):
    """C3. trials: [(dirección en grados, xy (N, 2) desde que aparece el objetivo hasta el final)].

    Alcance por dirección (percentil 95 de la proyección), ganancias, suavidad por movimiento y si
    el niño se queda "atorado" (más de 2 s fuera de la zona muerta al terminar el ensayo)."""
    reach = {d: [] for d in DIRECTIONS}
    stuck, moves = 0, []
    for direction, xy in trials:
        xy = np.asarray(xy, dtype=float)
        if len(xy) == 0:
            continue
        a = math.radians(direction)
        proj = xy @ np.array([math.cos(a), math.sin(a)])
        reach[direction].append(float(np.percentile(proj, 95)))
        tail = np.linalg.norm(xy[-int(2 * fs):], axis=1) if len(xy) >= 2 * fs else np.array([])
        if len(tail) and np.all(tail > deadzone):
            stuck += 1
        for i0, i1 in segment_by_radius(xy, deadzone):
            moves.append(smoothness(xy[i0:i1], fs))
    alcance = {d: (max(0.0, float(np.mean(v))) if v else 0.0) for d, v in reach.items()}
    ganancias = [round(_clamp(1.0 / a, 1.0, 3.0), 2) if a > 0.05 else 3.0 for a in (alcance[d] for d in DIRECTIONS)]
    med = (lambda k: float(np.nanmedian([m[k] for m in moves])) if moves else float('nan'))
    return {'alcance': {str(d): round(v, 3) for d, v in alcance.items()},
            'direcciones_cortas': sum(1 for v in alcance.values() if v < 0.4),
            'ganancias': ganancias, 'atorado': stuck, 'ensayos': len(trials),
            'movimientos': len(moves), 'ldlj': med('ldlj'), 'sparc': med('sparc'), 'msj': med('msj')}


def circle_results(xy, fs, radius=0.7):
    """C4. Error RMS al círculo dividido entre el radio, y suavidad."""
    xy = np.asarray(xy, dtype=float)
    if len(xy) < 10:
        return {'error_relativo': float('nan'), 'ldlj': float('nan'), 'sparc': float('nan')}
    r = np.linalg.norm(xy, axis=1)
    err = float(np.sqrt(np.mean((r - radius) ** 2)) / radius)
    return {'error_relativo': err, 'ldlj': ldlj(xy, fs), 'sparc': sparc(xy, fs)}


def assign(switch, rest, reach, circle, two_hands_ok, macs=0):
    """C6. Modo de interacción y nivel de asistencia inicial."""
    if reach is None or reach['ensayos'] == 0 or reach['direcciones_cortas'] >= 4:
        mode = 'S'
        why = 'el joystick no se puede usar (alcance menor a 0.4 en 4 o más direcciones)'
    elif reach['atorado'] >= max(2, reach['ensayos'] // 4):
        mode, why = 'J1', 'no logra soltar el joystick ni regresarlo al centro'
    elif two_hands_ok:
        mode, why = 'J3', 'pasó la prueba de dos manos'
    else:
        mode, why = 'J2', 'usa el joystick pero no pasó la prueba de dos manos'
    tremor = rest['temblor'] if rest else 0.0
    cerr = circle['error_relativo'] if circle and not math.isnan(circle['error_relativo']) else 0.5
    if tremor >= 0.15 or cerr > 0.25:
        level = 'alta'
    elif tremor < 0.05 and cerr < 0.10:
        level = 'baja'
    else:
        level = 'media'
    macs_mode = {5: 'S', 4: 'J1 o S', 3: 'J2', 2: 'J3', 1: 'J3'}.get(int(macs or 0), '')
    return {'interaccion': mode, 'razon': why, 'asistencia': level, 'sugerencia_macs': macs_mode,
            'coincide_con_macs': (not macs_mode) or mode in macs_mode}


def cursor_filter(rest, reach):
    """C7. min_cutoff del One Euro (temblor) y beta (suavidad)."""
    tremor = rest['temblor'] if rest else 0.0
    min_cutoff = 1.5 - (1.5 - 0.3) * _clamp(tremor / 0.2, 0.0, 1.0)
    smooth = reach['ldlj'] if reach and not math.isnan(reach.get('ldlj', float('nan'))) else -8.0
    beta = 0.2 + 0.8 * _clamp((smooth + 10.0) / 4.5, 0.0, 1.0)    # LDLJ -10 -> 0.2, -5.5 -> 1.0
    return round(min_cutoff, 2), round(beta, 2)


def suggested_profile(user_id, switch, rest, reach, circle, decision):
    """Perfil sugerido (formato de parámetros de ROS 2), para pasarlo al launch con perfil:=."""
    level = decision['asistencia']
    pr = preset(level)
    min_cutoff, beta = cursor_filter(rest, reach)
    prof = {
        'perfil': {'ros__parameters': {'interaccion': decision['interaccion'], 'modo': 'libre',
                                       'asistencia': level}},
        'switch_filter': {'ros__parameters': {
            'acceptance_time': switch['tiempo_aceptacion'] if switch else 0.10,
            'post_acceptance_delay': switch['retardo'] if switch else 0.30}},
        'scanning_menu': {'ros__parameters': {
            'user_id': user_id, 'mode': 'libre', 'level': 3,
            'scan_interval': switch['periodo_barrido'] if switch else 1.5,
            'rehab': {'assistance': level}}},
        'decision_logger': {'ros__parameters': {'user_id': user_id}},
    }
    if decision['interaccion'] != 'S':
        prof['entrada_joystick'] = {'ros__parameters': {
            'user_id': user_id, 'zona_muerta': rest['zona_muerta'] if rest else 0.10,
            'ganancias': reach['ganancias'] if reach else [1.0] * 8,
            'one_euro': {'min_cutoff': min_cutoff, 'beta': beta},
            'velocidad_max': pr['max_speed']}}
        prof['fantasma'] = {'ros__parameters': {
            'user_id': user_id, 'interaccion': decision['interaccion'], 'modo': 'libre',
            'asistencia': level}}
    return prof
