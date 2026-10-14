"""Small numpy helpers to go between 4x4 transforms and geometry_msgs."""
import numpy as np
from geometry_msgs.msg import Pose


def matrix_to_quaternion(R):
    """3x3 rotation matrix -> (x, y, z, w)."""
    t = np.trace(R)
    if t > 0.0:
        s = 2.0 * np.sqrt(t + 1.0)
        return ((R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s,
                (R[1, 0] - R[0, 1]) / s, 0.25 * s)
    i = int(np.argmax(np.diag(R)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = 2.0 * np.sqrt(1.0 + R[i, i] - R[j, j] - R[k, k])
    q = [0.0, 0.0, 0.0, 0.0]
    q[i] = 0.25 * s
    q[j] = (R[j, i] + R[i, j]) / s
    q[k] = (R[k, i] + R[i, k]) / s
    q[3] = (R[k, j] - R[j, k]) / s
    return tuple(q)


def quaternion_to_matrix(x, y, z, w):
    """(x, y, z, w) -> 3x3 rotation matrix."""
    n = np.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def matrix_to_pose(T):
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = (float(v) for v in T[:3, 3])
    q = matrix_to_quaternion(T[:3, :3])
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = (
        float(v) for v in q)
    return pose


def pose_to_matrix(pose):
    T = np.eye(4)
    o = pose.orientation
    T[:3, :3] = quaternion_to_matrix(o.x, o.y, o.z, o.w)
    T[:3, 3] = (pose.position.x, pose.position.y, pose.position.z)
    return T


def transform_to_matrix(transform):
    """geometry_msgs/Transform -> 4x4."""
    T = np.eye(4)
    r = transform.rotation
    T[:3, :3] = quaternion_to_matrix(r.x, r.y, r.z, r.w)
    T[:3, 3] = (transform.translation.x, transform.translation.y, transform.translation.z)
    return T
