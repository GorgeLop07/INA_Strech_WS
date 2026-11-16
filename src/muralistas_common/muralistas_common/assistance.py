"""Niveles de asistencia (salen de la calibración y bajan conforme el niño mejora).

Valores propuestos en la planeación, a validar en el piloto:
  - min_cutoff: filtro One Euro del cursor (Hz); más bajo = suaviza más el temblor.
  - max_speed: velocidad máxima del cursor del fantasma en el pizarrón (m/s).
  - magnet: imán hacia la plantilla en rehabilitación (0 = sin imán).
  - dwell: t_quieto de J1 (s).
  - switch_speed: modo S en rehabilitación, velocidad del robot sobre la plantilla (m/s).
"""

PRESETS = {
    'alta': {'min_cutoff': 0.3, 'max_speed': 0.10, 'magnet': 0.5, 'dwell': 2.0, 'switch_speed': 0.02},
    'media': {'min_cutoff': 0.8, 'max_speed': 0.18, 'magnet': 0.25, 'dwell': 1.5, 'switch_speed': 0.03},
    'baja': {'min_cutoff': 1.5, 'max_speed': 0.25, 'magnet': 0.0, 'dwell': 1.0, 'switch_speed': 0.05},
}

INTERACTIONS = ('S', 'J1', 'J2', 'J3')


def preset(level):
    if level not in PRESETS:
        raise KeyError(f'Nivel de asistencia desconocido: {level}. Opciones: {list(PRESETS)}')
    return dict(PRESETS[level])
