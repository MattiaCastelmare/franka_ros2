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


def define_robot_segments(transforms: dict, robot_cfg: dict, distance_cfg: dict):
    """Build the per-segment capsule list for distance computation.

    Unlike :func:`get_robot_segments_from_transforms` (which returns bare
    point-pairs for visualization), this function includes capsule radii and
    handles the end-effector tip offset so that the last segment ends exactly
    at the physical fingertip rather than at fr3_link8 origin.

    Parameters
    ----------
    transforms:
        Dict mapping link_name -> (R, t).
    robot_cfg:
        ``robot`` section of fr3_complete.yaml.
    distance_cfg:
        ``distance`` section of fr3_complete.yaml (needs ``ee_tip_axis``
        and ``ee_tip_offset``).

    Returns
    -------
    list of dicts, or None if no segments could be built.
    """
    robot_segments = []
    for seg in robot_cfg['segments']:
        seg_idx = int(seg['seg_idx'])
        start_link = seg['start_link']
        end_link = seg['end_link']
        radius = float(seg.get('radius', 0.0))

        if start_link not in transforms or end_link not in transforms:
            continue

        _, p0 = transforms[start_link]
        _, p1 = transforms[end_link]

        # Special case: extend last segment to physical EE tip
        if end_link == 'fr3_link8':
            ee_tip_axis = distance_cfg['ee_tip_axis']
            ee_tip_offset = distance_cfg['ee_tip_offset']
            R_ee, p_ee = transforms['fr3_link8']
            direction = R_ee[:, ee_tip_axis]
            p1 = p_ee + ee_tip_offset * direction

        robot_segments.append({
            'seg_idx': seg_idx,
            'start_link': start_link,
            'end_link': end_link,
            'p0': p0,
            'p1': p1,
            'radius': radius,
        })

    return robot_segments if robot_segments else None


def compute_closest_distance_from_segments(
    last_depth,
    K_inv,
    transform_camera_to_base_fn,
    robot_segments,
    x,
    y,
    step,
    search_exclusion_mask,
    distance_cfg,
) -> tuple:
    """Find the closest obstacle point to any robot capsule segment.

    Iterates over every valid depth pixel in the ROI and computes
    point-to-segment distance against each capsule, accounting for the
    capsule radius.

    Returns
    -------
    best_result : dict | None
        Keys: seg_idx, start_link, end_link, point (projection on segment),
        distance, direction, closest_obstacle_point, closest_pixel, radius.
    valid_point_count : int
    """
    if last_depth is None or robot_segments is None:
        return None, 0

    min_depth = float(distance_cfg['min_depth_m'])
    max_depth = float(distance_cfg['max_depth_m'])

    valid_point_count = 0
    best_result = None
    best_dist = np.inf

    for v in range(y[0], y[1], step):
        for u in range(x[0], x[1], step):
            if search_exclusion_mask is not None and search_exclusion_mask[v, u]:
                continue

            Z = float(last_depth[v, u]) / 1000.0
            if Z < min_depth or Z > max_depth:
                continue
            valid_point_count += 1

            uv1 = np.array([u, v, 1.0], dtype=float)
            p_cam = Z * (K_inv @ uv1)
            p_obs = transform_camera_to_base_fn(p_cam)
            if p_obs is None:
                continue

            for seg in robot_segments:
                raw_dist, proj = point_to_segment_distance_with_projection(
                    p_obs, seg['p0'], seg['p1'])
                dist = max(raw_dist - seg['radius'], 0.0)

                if dist < best_dist:
                    best_dist = dist
                    vec = proj - p_obs
                    norm = np.linalg.norm(vec)
                    direction = vec / norm if norm > 1e-9 else np.zeros(3)
                    best_result = {
                        'seg_idx': seg['seg_idx'],
                        'start_link': seg['start_link'],
                        'end_link': seg['end_link'],
                        'point': proj,
                        'distance': dist,
                        'direction': direction,
                        'closest_obstacle_point': p_obs,
                        'closest_pixel': (u, v),
                        'radius': seg['radius'],
                    }

    return best_result, valid_point_count


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


def find_pt_confidence(best_result: float, n_pts: int) -> float:
    """Compute confidence score based on valid point count and distance value."""
    lm_conf = float(np.clip(n_pts / 500.0, 0.2, 1.0))
    dist_conf = (
        1.0 if best_result < 2.0
        else float(np.clip(1.0 - (best_result - 2.0) / 3.0, 0.3, 1.0))
    )
    return float(np.clip(lm_conf * dist_conf, 0.0, 1.0))


def compute_direction_vector(p_obs: np.ndarray, cp_positions: np.ndarray, i: int) -> np.ndarray:
    """Compute normalised direction vector from obstacle point to robot control point i."""
    vec = cp_positions[i] - p_obs
    norm = np.linalg.norm(vec)
    if norm > 1e-9:
        return vec / norm
    return np.zeros(3)
