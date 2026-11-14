"""Conversión de mensajes de ROS a numpy, sin cv_bridge (evita choques de numpy con ROS Humble)."""

import array

import numpy as np
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Image, PointCloud2, PointField

from muralistas_common.geometry import make_transform, matrix_to_quat, quat_to_matrix


def image_to_numpy(msg: Image):
    if msg.encoding in ('rgb8', 'bgr8'):
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step // 3, 3)[:, :msg.width]
        return img[:, :, ::-1].copy() if msg.encoding == 'bgr8' else img.copy()
    if msg.encoding == '16UC1':
        d = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.step // 2)[:, :msg.width]
        return d.astype(np.float32) / 1000.0
    if msg.encoding == '32FC1':
        return np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.step // 4)[:, :msg.width].copy()
    raise ValueError(f'Codificación no soportada: {msg.encoding}')


def numpy_to_image(img, stamp, frame_id):
    msg = Image()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height, msg.width = img.shape[:2]
    msg.encoding = 'rgb8'
    msg.step = msg.width * 3
    msg.data = array.array("B", np.ascontiguousarray(img, dtype=np.uint8).tobytes())   # bytes: ~50 ms por imagen
    return msg


def transform_to_matrix(t: TransformStamped):
    tr, q = t.transform.translation, t.transform.rotation
    return make_transform(quat_to_matrix((q.x, q.y, q.z, q.w)), [tr.x, tr.y, tr.z])


def matrix_to_transform(T, stamp, parent, child):
    msg = TransformStamped()
    msg.header.stamp = stamp
    msg.header.frame_id = parent
    msg.child_frame_id = child
    msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z = map(float, T[:3, 3])
    x, y, z, w = matrix_to_quat(T[:3, :3])
    msg.transform.rotation.x, msg.transform.rotation.y = float(x), float(y)
    msg.transform.rotation.z, msg.transform.rotation.w = float(z), float(w)
    return msg


def make_cloud(points, stamp, frame_id, colors=None):
    """PointCloud2 con x, y, z (y rgb si se dan colores uint8 Nx3)."""
    points = np.asarray(points, dtype=np.float32).reshape(-1, 3)
    fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1) for i, n in enumerate('xyz')]
    if colors is not None:
        fields.append(PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1))
        c = np.asarray(colors, dtype=np.uint32).reshape(-1, 3)
        packed = ((c[:, 0] << 16) | (c[:, 1] << 8) | c[:, 2]).astype(np.uint32).view(np.float32)
        data = np.column_stack([points, packed]).astype(np.float32)
    else:
        data = points
    msg = PointCloud2()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height, msg.width = 1, len(points)
    msg.fields = fields
    msg.is_bigendian = False
    msg.point_step = 4 * data.shape[1]
    msg.row_step = msg.point_step * msg.width
    msg.is_dense = True
    msg.data = array.array("B", data.tobytes())
    return msg
