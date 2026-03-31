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

def load_link_mesh_files():
    filename = '/ros2_ws/src/franka_simulation/config/fr3_collision.yaml'
    with open(filename, 'r') as f:
        data = yaml.safe_load(f)

    return data['link_mesh_files']

def load_link_names():
    filename = '/ros2_ws/src/franka_simulation/config/fr3_links.yaml'
    with open(filename, 'r') as f:
        data = yaml.safe_load(f)

    return data['link_names']

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
    
def get_robot_segments_from_transforms(transforms):
    link_names = load_link_names()
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