import yaml
import numpy as np


def load_extrinsics():
    filename = '/ros2_ws/src/franka_experiments/config/camera_extrinsics.yaml'
    with open(filename, 'r') as f:
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
        [2*(qx*qz - qy*qw), 2*(qy*qz + qx*qw), 1 - 2*(qx*qx + qy*qy)]
    ], dtype=float)

    t = np.array([tx, ty, tz], dtype=float)

    return R, t

def load_robot_config():
    filename = '/ros2_ws/src/franka_simulation/config/fr3_complete.yaml'
    with open(filename, 'r') as f:
        data = yaml.safe_load(f)
    return data

def point_to_segment_distance_with_projection(p, a, b):
        ab = b - a # segment vector
        denom = np.dot(ab, ab) # norm of segment

        if denom < 1e-12: # if segment is a point, then distance is just distance to that point
            return np.linalg.norm(p - a), a

        t = np.dot(p - a, ab) / denom # point p projected onto line defined by a and b, expressed as t in [0, 1]
        t = np.clip(t, 0.0, 1.0) # clamp to segment

        proj = a + t * ab # projection of p onto segment
        d = np.linalg.norm(p - proj) # distance from p to its projection on the segment
        return d, proj

def define_robot_segments(transforms, robot_cfg, distance_cfg):
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

        # Caso speciale EE tip
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
            'radius': radius
        })

    if len(robot_segments) == 0:
        return None

    return robot_segments

def compute_closest_distance_from_segments(last_depth,K_inv,transform_camera_to_base_fn,robot_segments,x,y,step,search_exclusion_mask,distance_cfg):
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
                p0 = seg['p0']
                p1 = seg['p1']
                radius = seg['radius']

                raw_dist, proj = point_to_segment_distance_with_projection(p_obs, p0, p1)
                dist = max(raw_dist - radius, 0.0)

                if dist < best_dist:
                    best_dist = dist

                    vec = proj - p_obs
                    norm = np.linalg.norm(vec)
                    if norm > 1e-9:
                        direction = vec / norm
                    else:
                        direction = np.zeros(3)

                    best_result = {
                        'seg_idx': seg['seg_idx'],
                        'start_link': seg['start_link'],
                        'end_link': seg['end_link'],
                        'point': proj,
                        'distance': dist,
                        'direction': direction,
                        'closest_obstacle_point': p_obs,
                        'closest_pixel': (u, v),
                        'radius': radius
                    }

    return best_result, valid_point_count
    
def get_robot_segments_from_transforms(transforms):
    link_names = load_robot_config()['robot']['segment_links']
    link_names = [name for name in link_names if name != 'fr3_link0']  # Exclude the base link

    points = []
    for name in link_names:
        if name not in transforms:
            return None
        _, t = transforms[name]
        points.append(t)

    segments = []
    for i in range(len(points) - 1):
        segments.append((points[i], points[i + 1]))

    return segments

def get_rotation_from_quaternion(q):
    qx, qy, qz, qw = q.x, q.y, q.z, q.w

    R = np.array([
        [1 - 2*(qy*qy + qz*qz), 2*(qx*qy - qz*qw), 2*(qx*qz + qy*qw)],
        [2*(qx*qy + qz*qw), 1 - 2*(qx*qx + qz*qz), 2*(qy*qz - qx*qw)],
        [2*(qx*qz - qy*qw), 2*(qy*qz + qx*qw), 1 - 2*(qx*qx + qy*qy)]
    ], dtype=float)

    return R

def find_pt_confidence(best_result, n_pts):
    # Confidence is based on number of valid points and distance value
    lm_conf = float(np.clip(n_pts / 500.0, 0.2, 1.0))
    dist_conf = (
        1.0 if best_result < 2.0
        else float(np.clip(1.0 - (best_result - 2.0) / 3.0, 0.3, 1.0))
    )
    confidence = float(np.clip(lm_conf * dist_conf, 0.0, 1.0))

    return confidence

def compute_direction_vector(p_obs, cp_positions, i):
    # Direction from human (obs) to robot (cp)
    vec = cp_positions[i] - p_obs
    norm = np.linalg.norm(vec)

    # Avoid division by zero
    if norm > 1e-9:
        direction = vec / norm
    else:
        direction = np.zeros(3)
    
    return direction
