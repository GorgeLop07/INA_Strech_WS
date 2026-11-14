"""Transformaciones y ajustes geométricos con numpy (sin dependencias de ROS)."""

import numpy as np


# --- Cuaterniones (convención x, y, z, w como en ROS) ---------------------
def quat_to_matrix(q):
    x, y, z, w = q
    n = x * x + y * y + z * z + w * w
    if n < 1e-12:
        return np.eye(3)
    s = 2.0 / n
    return np.array([
        [1 - s * (y * y + z * z), s * (x * y - z * w), s * (x * z + y * w)],
        [s * (x * y + z * w), 1 - s * (x * x + z * z), s * (y * z - x * w)],
        [s * (x * z - y * w), s * (y * z + x * w), 1 - s * (x * x + y * y)],
    ])


def matrix_to_quat(m):
    m = np.asarray(m, dtype=float)
    t = np.trace(m)
    if t > 0:
        s = np.sqrt(t + 1.0) * 2
        w = 0.25 * s
        x = (m[2, 1] - m[1, 2]) / s
        y = (m[0, 2] - m[2, 0]) / s
        z = (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
        w = (m[2, 1] - m[1, 2]) / s
        x = 0.25 * s
        y = (m[0, 1] + m[1, 0]) / s
        z = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
        w = (m[0, 2] - m[2, 0]) / s
        x = (m[0, 1] + m[1, 0]) / s
        y = 0.25 * s
        z = (m[1, 2] + m[2, 1]) / s
    else:
        s = np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
        w = (m[1, 0] - m[0, 1]) / s
        x = (m[0, 2] + m[2, 0]) / s
        y = (m[1, 2] + m[2, 1]) / s
        z = 0.25 * s
    q = np.array([x, y, z, w])
    return q / np.linalg.norm(q)


def make_transform(R, t):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


def invert_transform(T):
    R, t = T[:3, :3], T[:3, 3]
    return make_transform(R.T, -R.T @ t)


def transform_points(T, pts):
    pts = np.asarray(pts, dtype=float).reshape(-1, 3)
    return pts @ T[:3, :3].T + T[:3, 3]


def yaw_from_matrix(R):
    return float(np.arctan2(R[1, 0], R[0, 0]))


# --- Cámara -----------------------------------------------------------------
def intrinsics_from_fovy(fovy_deg, width, height):
    """fx, fy, cx, cy para una cámara de píxeles cuadrados con campo de visión vertical fovy."""
    fy = (height / 2.0) / np.tan(np.radians(fovy_deg) / 2.0)
    return fy, fy, (width - 1) / 2.0, (height - 1) / 2.0


def deproject(depth, fx, fy, cx, cy, stride=1):
    """Imagen de profundidad (m) -> puntos 3D en el marco óptico (z al frente, x a la derecha, y abajo).

    Devuelve (puntos Nx3, índices (filas, columnas)) solo de los píxeles con profundidad válida.
    """
    d = depth[::stride, ::stride]
    rows, cols = np.nonzero(np.isfinite(d) & (d > 0))
    z = d[rows, cols]
    rows_full = rows * stride
    cols_full = cols * stride
    x = (cols_full - cx) * z / fx
    y = (rows_full - cy) * z / fy
    return np.stack([x, y, z], axis=1), (rows_full, cols_full)


# --- Ajustes ------------------------------------------------------------------
def fit_rigid(src, dst):
    """Rotación R y traslación t que llevan los puntos src a dst (Kabsch). Devuelve (R, t, rms)."""
    src = np.asarray(src, dtype=float)
    dst = np.asarray(dst, dtype=float)
    cs, cd = src.mean(axis=0), dst.mean(axis=0)
    H = (src - cs).T @ (dst - cd)
    U, _, Vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    D = np.diag([1.0, 1.0, d])
    R = Vt.T @ D @ U.T
    t = cd - R @ cs
    rms = float(np.sqrt(np.mean(np.sum((src @ R.T + t - dst) ** 2, axis=1))))
    return R, t, rms


def fit_plane_ransac(pts, threshold=0.01, iterations=200, rng=None):
    """Plano por RANSAC + mínimos cuadrados. Devuelve (normal unitaria, punto en el plano, máscara de inliers)."""
    pts = np.asarray(pts, dtype=float)
    if len(pts) < 3:
        return None, None, np.zeros(len(pts), dtype=bool)
    rng = rng or np.random.default_rng(0)
    best = None
    best_count = 0
    for _ in range(iterations):
        a, b, c = pts[rng.choice(len(pts), 3, replace=False)]
        n = np.cross(b - a, c - a)
        norm = np.linalg.norm(n)
        if norm < 1e-9:
            continue
        n /= norm
        inliers = np.abs((pts - a) @ n) < threshold
        count = int(inliers.sum())
        if count > best_count:
            best_count, best = count, inliers
    if best is None:
        return None, None, np.zeros(len(pts), dtype=bool)
    inl = pts[best]
    centroid = inl.mean(axis=0)
    _, _, Vt = np.linalg.svd(inl - centroid)
    normal = Vt[2]
    return normal / np.linalg.norm(normal), centroid, best


def principal_axis(pts):
    """Centroide y eje principal (PCA) de una nube de puntos."""
    pts = np.asarray(pts, dtype=float)
    c = pts.mean(axis=0)
    _, _, Vt = np.linalg.svd(pts - c)
    return c, Vt[0]
