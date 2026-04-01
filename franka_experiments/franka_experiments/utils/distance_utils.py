"""Utility functions for real-time robot-obstacle distance estimation.

Ported from franka_simulation/src/utils.py with hardcoded paths removed.
All file-loading helpers now accept an explicit path argument.
"""
import yaml
import numpy as np


def load_extrinsics(path: str):
    """Load camera extrinsic calibration from a YAML file.

    Returns
    -------
    R : np.ndarray, shape (3, 3)
        Rotation matrix (camera → base).
    t : np.ndarray, shape (3,)
        Translation vector (camera → base).
    """
    with open(path, 'r') as f:
        data = yaml.safe_load(f)

    tx = data['translation']['x']
    ty = data['translation']['y']
    tz = data['translation']['z']

    qx = data['rotation']['x']
    qy = data['rotation']['y']
    qz = data['rotation']['z']
    qw = data['rotation']['w']

    R = np.array([
        [1 - 2*(qy*qy + qz*qz), 2*(qx*qy - qz*qw), 2*(qx*qz + qy*qw)],
        [2*(qx*qy + qz*qw), 1 - 2*(qx*qx + qz*qz), 2*(qy*qz - qx*qw)],
        [2*(qx*qz - qy*qw), 2*(qy*qz + qx*qw), 1 - 2*(qx*qx + qy*qy)],
    ], dtype=float)

    return R, np.array([tx, ty, tz], dtype=float)


def load_robot_config(path: str) -> dict:
    """Load robot configuration from a YAML file."""
    with open(path, 'r') as f:
        return yaml.safe_load(f)


def point_to_segment_distance_with_projection(p, a, b):
    """Return (distance, projection) from point *p* to segment [a, b]."""
    ab = b - a
    denom = np.dot(ab, ab)
    if denom < 1e-12:
        return np.linalg.norm(p - a), a
    t = np.clip(np.dot(p - a, ab) / denom, 0.0, 1.0)
    proj = a + t * ab
    return np.linalg.norm(p - proj), proj


def get_robot_segments_from_transforms(transforms: dict, segment_links: list):
    """Build ordered line segments from consecutive link positions.

    Parameters
    ----------
    transforms:
        Dict mapping link_name -> (R, t).
    segment_links:
        Ordered list of link names as defined in the robot config.
        The first entry (base link) is excluded from the segment chain.
    """
    link_names = [name for name in segment_links if name != segment_links[0]]

    points = []
    for name in link_names:
        if name not in transforms:
            return None
        _, t = transforms[name]
        points.append(t)

    return [(points[i], points[i + 1]) for i in range(len(points) - 1)]


def get_rotation_from_quaternion(q) -> np.ndarray:
    """Convert a quaternion message object to a 3x3 rotation matrix."""
    qx, qy, qz, qw = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2*(qy*qy + qz*qz), 2*(qx*qy - qz*qw), 2*(qx*qz + qy*qw)],
        [2*(qx*qy + qz*qw), 1 - 2*(qx*qx + qz*qz), 2*(qy*qz - qx*qw)],
        [2*(qx*qz - qy*qw), 2*(qy*qz + qx*qw), 1 - 2*(qx*qx + qy*qy)],
    ], dtype=float)
