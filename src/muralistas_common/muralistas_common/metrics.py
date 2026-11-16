"""Métricas de motricidad y calificación de intentos (sin ROS, para poder probarlas).

Suavidad, sobre la señal cruda del joystick (la intención del niño, no el robot):
  - MSJ: promedio del jerk al cuadrado (como en el repo de JIGSAWS, para comparar con ese trabajo).
    Depende de la duración y del tamaño del movimiento.
  - LDLJ: jerk adimensional en logaritmo, versión de velocidad (Balasubramanian et al. 2015). No
    depende de la duración ni del tamaño. Más cerca de cero = más suave; un movimiento de mínimo
    jerk da unos -5.3.
  - SPARC: longitud de arco del espectro de la velocidad. Menos sensible al ruido.
Antes de derivar se filtra con un Butterworth de 2.º orden ida y vuelta (sin desfase).

Calificación: error medio y cobertura contra la plantilla (joystick), error de paro (switch),
estrellas y la regla de progresión (5 de los últimos 8 intentos).
"""

import math
from dataclasses import dataclass, field

import numpy as np

from .templates import nearest_on_polyline, resample

LEVELS = ['alta', 'media', 'baja']   # asistencia, de más a menos


# --- Señal ---------------------------------------------------------------------
def lowpass(x, fs, fc=8.0, order=2):
    """Butterworth ida y vuelta por columna. Si la señal es muy corta, la regresa igual."""
    from scipy.signal import butter, filtfilt
    x = np.asarray(x, dtype=float)
    if fc >= fs / 2.0:
        return x
    b, a = butter(order, fc / (fs / 2.0))
    if x.shape[0] <= 3 * max(len(a), len(b)):
        return x
    return filtfilt(b, a, x, axis=0)


def _derivatives(pos, fs, fc):
    pos = np.asarray(pos, dtype=float)
    if pos.ndim == 1:
        pos = pos[:, None]
    p = lowpass(pos, fs, fc)
    dt = 1.0 / fs
    v = np.gradient(p, dt, axis=0)
    a = np.gradient(v, dt, axis=0)
    j = np.gradient(a, dt, axis=0)
    return v, a, j


def msj(pos, fs, fc=8.0):
    """Mean squared jerk (unidades de la señal² / s⁶)."""
    if len(pos) < 4:
        return float('nan')
    _, _, j = _derivatives(pos, fs, fc)
    return float(np.mean(np.sum(j * j, axis=1)))


def ldlj(pos, fs, fc=8.0):
    """Log dimensionless jerk (versión de velocidad)."""
    if len(pos) < 4:
        return float('nan')
    v, _, j = _derivatives(pos, fs, fc)
    speed = np.linalg.norm(v, axis=1)
    v_peak = speed.max()
    if v_peak <= 0:
        return float('nan')
    T = (len(pos) - 1) / fs
    integral = np.sum(np.sum(j * j, axis=1)) / fs
    return float(-math.log(T ** 3 / v_peak ** 2 * integral))


def sparc(pos, fs, fc=10.0, amp_th=0.05, padlevel=4, filter_fc=8.0):
    """Spectral arc length (Balasubramanian et al. 2015). Más cerca de cero = más suave."""
    if len(pos) < 4:
        return float('nan')
    v, _, _ = _derivatives(pos, fs, filter_fc)
    speed = np.linalg.norm(v, axis=1)
    nfft = int(2 ** (math.ceil(math.log2(len(speed))) + padlevel))
    f = np.arange(0, fs, fs / nfft)
    Mf = np.abs(np.fft.fft(speed, nfft))
    if Mf.max() <= 0:
        return float('nan')
    Mf = Mf / Mf.max()
    sel = f <= fc
    f, Mf = f[sel], Mf[sel]
    above = np.nonzero(Mf >= amp_th)[0]
    f, Mf = f[above[0]:above[-1] + 1], Mf[above[0]:above[-1] + 1]
    df = np.diff(f) / (f[-1] - f[0]) if f[-1] > f[0] else np.diff(f)
    return float(-np.sum(np.sqrt(df ** 2 + np.diff(Mf) ** 2)))


def segment_by_radius(xy, radius, min_samples=10):
    """Movimientos de un joystick: desde que sale de la zona `radius` hasta que regresa.

    Devuelve [(i0, i1), ...] con i1 exclusivo.
    """
    r = np.linalg.norm(np.asarray(xy, dtype=float), axis=1)
    out, start = [], None
    for i, ri in enumerate(r):
        if ri > radius and start is None:
            start = i
        elif ri <= radius and start is not None:
            if i - start >= min_samples:
                out.append((start, i))
            start = None
    if start is not None and len(r) - start >= min_samples:
        out.append((start, len(r)))
    return out


def smoothness(xy, fs):
    """Las tres métricas de un movimiento."""
    return {'ldlj': ldlj(xy, fs), 'sparc': sparc(xy, fs), 'msj': msj(xy, fs)}


# --- Calificación ----------------------------------------------------------------
def trace_metrics(path, template, tol=0.015, step=0.005):
    """Error medio (m) del trazo a la plantilla y cobertura (fracción de la plantilla a menos de tol)."""
    if len(path) < 2:
        return float('nan'), 0.0
    path_rs = resample(list(path), step)
    err = float(np.mean([nearest_on_polyline(p, template)[1] for p in path_rs]))
    tmpl_rs = resample(list(template), step)
    covered = sum(1 for q in tmpl_rs if nearest_on_polyline(q, path_rs)[1] <= tol)
    return err, covered / len(tmpl_rs)


def trace_stars(mean_error, coverage, size=0.30):
    """★ cobertura ≥ 80%; ★★ además error ≤ 2 cm; ★★★ error ≤ 1 cm (escalado al tamaño, base 30 cm)."""
    s = size / 0.30
    if coverage < 0.8 or math.isnan(mean_error):
        return 0
    if mean_error <= 0.01 * s:
        return 3
    if mean_error <= 0.02 * s:
        return 2
    return 1


def stop_stars(mean_abs_error, size=0.30):
    """Modo S: ★ error de paro ≤ 3 cm, ★★ ≤ 2 cm, ★★★ ≤ 1 cm (escalado al tamaño)."""
    s = size / 0.30
    for stars, limit in ((3, 0.01), (2, 0.02), (1, 0.03)):
        if mean_abs_error <= limit * s:
            return stars
    return 0


@dataclass
class Progression:
    """Regla de progresión: 8 intentos por decisión, se avanza con 5 de 8 en ★★ o más.

    Avanzar = bajar la asistencia (alta -> media -> baja); ya en baja, pasar a la siguiente etapa
    con asistencia alta. Si 3 intentos seguidos tienen cobertura menor a 50%, se sube la asistencia.
    """
    stage: int = 0
    level: str = 'media'
    last_stage: int = 3
    window: int = 8
    needed: int = 5
    history: list = field(default_factory=list)
    misses: int = 0

    def update(self, stars, coverage=1.0):
        """Registra un intento. Devuelve el cambio ('baja_asistencia', 'sube_etapa', 'sube_asistencia',
        'completo') o None."""
        self.history.append(stars)
        self.misses = self.misses + 1 if coverage < 0.5 else 0
        if self.misses >= 3:
            self.misses = 0
            self.history.clear()
            i = LEVELS.index(self.level)
            if i > 0:
                self.level = LEVELS[i - 1]
                return 'sube_asistencia'
            return None
        recent = self.history[-self.window:]
        if len(recent) < self.window or sum(1 for s in recent if s >= 2) < self.needed:
            return None
        self.history.clear()
        i = LEVELS.index(self.level)
        if i < len(LEVELS) - 1:
            self.level = LEVELS[i + 1]
            return 'baja_asistencia'
        if self.stage < self.last_stage:
            self.stage += 1
            self.level = LEVELS[0]
            return 'sube_etapa'
        return 'completo'
