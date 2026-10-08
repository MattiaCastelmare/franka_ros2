#!/usr/bin/env python3
"""Grasp pose on the object held by the active hand -> /handover/grasp_pose.

Backends (both non-commercial, research use):
  - gsnet: GSNet (Wang et al., ICCV 2021) via graspnet/graspness_unofficial, RealSense
    checkpoint; installed in /ros2_ws/graspness (MinkowskiEngine + CUDA ops);
  - anygrasp: AnyGrasp SDK (licensed binary gsnet.so + license/ + checkpoint in sdk_dir).
Without its backend the node logs once and stays idle.

  - scene cloud: main-camera depth around the object (hand included);
  - object: Hands23 outline from /handover/hand_object (only grasps on it);
  - approach: within approach_thresh_deg of the robot TCP -> object direction;
  - collisions: gripper volume against the cloud (so against the hand), plus a
    clearance from the tracked hand landmarks;
  - hysteresis: the previous grasp is kept while it stays close to the best one.

Output: Franka TCP axes (z approach, y finger closing) at the fingertip centre, base frame.
"""

import os
import site
import sys
import threading
import time
import types
from argparse import Namespace

import cv2
import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from franka_msgs.msg import HandObjectState, HandoverDistance, HandTrackingFiltered
from geometry_msgs.msg import PoseStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, Image

TRACKED = (HandTrackingFiltered.TRACKING, HandTrackingFiltered.PREDICT_ONLY)


def collisions(points, t, R, widths, depths, heights, finger_w=0.01, finger_l=0.06,
               approach_dist=0.05, voxel=0.01, thresh=0.01):
    """GraspNet model-free collision check (graspness_unofficial), numpy only.

    Grasp frame: x approach, y closing, z height. True = gripper volume hits the cloud."""
    pts = np.unique(np.round(points / voxel), axis=0) * voxel  # voxel downsample
    q = np.matmul(pts[None] - t[:, None], R)                    # (M, N, 3) in grasp frames
    w, d, h = widths[:, None] / 2, depths[:, None], heights[:, None] / 2
    x, y, z = q[..., 0], q[..., 1], q[..., 2]
    in_h = (z > -h) & (z < h)
    span = (y > -(w + finger_w)) & (y < w + finger_w)
    fingers = (x > d - finger_l) & (x < d) & (((y > -(w + finger_w)) & (y < -w)) | ((y < w + finger_w) & (y > w)))
    bottom = (x <= d - finger_l) & (x > d - finger_l - finger_w) & span
    shift = (x <= d - finger_l - finger_w) & (x > d - finger_l - finger_w - approach_dist) & span
    hits = (in_h & (fingers | bottom | shift)).sum(1)
    volume = (2 * finger_l * finger_w + (2 * w[:, 0] + 2 * finger_w) * (finger_w + approach_dist)) * 2 * h[:, 0]
    return hits / (volume / voxel ** 3 + 1e-6) > thresh


class Gsnet:
    """GSNet inference -> grasps (graspnet format) on the whole cloud."""

    def __init__(self, root, venv, checkpoint, num_point=15000, voxel=0.005):
        site.addsitedir(f'{venv}/lib/python3.10/site-packages')  # MinkowskiEngine, pointnet2
        sys.path[:0] = [root, f'{root}/utils', f'{root}/dataset', f'{root}/pointnet2']
        # knn (THC API, no longer builds) only makes training labels and open3d only draws:
        # empty stand-ins, inference never calls them
        for name in ('knn', 'knn.knn_modules', 'open3d'):
            sys.modules.setdefault(name, types.ModuleType(name))
        sys.modules['knn.knn_modules'].knn = None
        import torch
        from models.graspnet import GraspNet, pred_decode
        from graspnet_dataset import minkowski_collate_fn
        self.torch, self.decode, self.collate = torch, pred_decode, minkowski_collate_fn
        self.net = GraspNet(seed_feat_dim=512, is_training=False).cuda().eval()
        self.net.load_state_dict(torch.load(checkpoint, map_location='cuda')['model_state_dict'])
        self.num_point, self.voxel = num_point, voxel

    def __call__(self, points):
        n = len(points)
        idx = (np.random.choice(n, self.num_point, replace=False) if n >= self.num_point else
               np.r_[np.arange(n), np.random.choice(n, self.num_point - n, replace=True)])
        cloud = points[idx].astype(np.float32)
        batch = self.collate([{'point_clouds': cloud, 'coors': cloud / self.voxel,
                               'feats': np.ones_like(cloud)}])
        for key, value in batch.items():
            if 'list' in key:
                batch[key] = [[v.cuda() for v in level] for level in value]
            else:
                batch[key] = value.cuda()
        with self.torch.no_grad():
            p = self.decode(self.net(batch))[0].float().cpu().numpy()
        # score, width, height, depth, rotation (9), translation (3), object id
        return dict(score=p[:, 0], width=p[:, 1], height=p[:, 2], depth=p[:, 3],
                    R=p[:, 4:13].reshape(-1, 3, 3), t=p[:, 13:16])


class AnyGrasp:
    """AnyGrasp SDK: region / approach steering and collision check inside the SDK."""

    def __init__(self, sdk_dir, checkpoint, max_width, height):
        sys.path.insert(0, sdk_dir)
        os.chdir(sdk_dir)  # the SDK looks for license/ here
        from gsnet import create_detector
        self.detector = create_detector(Namespace(checkpoint_path=checkpoint,
                                                  max_gripper_width=min(0.1, max_width), gripper_height=height))
        if self.detector is None:
            raise RuntimeError('license check failed')

    def __call__(self, points, region, approach, thresh):
        g = self.detector.get_grasp(points, {'region_steering': region, 'collision_detection': True,
                                             'dense_grasp': False, 'approach_steering': approach,
                                             'approach_thresh': thresh})
        if g is None or len(g) == 0:
            return None
        g = g.nms()
        return dict(score=np.asarray(g.scores, float), width=g.widths, height=g.heights, depth=g.depths,
                    R=g.rotation_matrices, t=g.translations)


class GraspPoseNode(Node):

    def __init__(self):
        super().__init__('grasp_pose')
        par = lambda n, v: self.declare_parameter(n, v).value
        self.backend_name = par('backend', 'gsnet')
        self.max_width = float(par('max_gripper_width_m', 0.08))   # Franka hand
        self.crop_m = float(par('crop_m', 0.25))                   # cloud around the object
        self.approach_thresh = np.radians(float(par('approach_thresh_deg', 60.0)))
        self.clearance = float(par('hand_clearance_m', 0.05))
        self.keep_ratio = float(par('keep_ratio', 0.8))            # hysteresis on the choice
        self.period = 1.0 / float(par('max_rate_hz', 10.0))
        g = '/ros2_ws/graspness'
        backend_args = {
            'gsnet': (par('gsnet_root', f'{g}/graspness_unofficial'), par('gsnet_venv', f'{g}/venv'),
                      par('gsnet_checkpoint', f'{g}/ckpt/minkuresunet_realsense.tar')),
            'anygrasp': (par('anygrasp_sdk_dir', '/ros2_ws/anygrasp/grasp_detection'),
                         par('anygrasp_checkpoint', 'log/checkpoint_detection.tar'), self.max_width, 0.03)}

        ext = yaml.safe_load(open(os.path.join(
            get_package_share_directory('franka_experiments'), 'config', 'camera_extrinsics.yaml')))
        t, q = ext['translation'], ext['rotation']
        self.t = np.array([t['x'], t['y'], t['z']])
        self.R = Rotation.from_quat([q['x'], q['y'], q['z'], q['w']]).as_matrix()  # camera -> base
        self.base_frame = ext['parent_frame']

        self.bridge, self.K = CvBridge(), None
        self.depth = self.obj = self.hand = self.ee = None
        self.last = None  # (tip, approach) of the published grasp, base frame
        self.lock = threading.Lock()
        self.pub = self.create_publisher(PoseStamped, '/handover/grasp_pose', 10)
        cam = '/camera/camera/aligned_depth_to_color/'
        self.create_subscription(CameraInfo, cam + 'camera_info', self.on_info, qos_profile_sensor_data)
        self.create_subscription(Image, cam + 'image_raw', lambda m: self._set('depth', m),
                                 qos_profile_sensor_data)
        self.create_subscription(HandObjectState, '/handover/hand_object', lambda m: self._set('obj', m), 10)
        self.create_subscription(HandTrackingFiltered, '/handover/hand_tracking_filtered',
                                 lambda m: self._set('hand', m), 10)
        self.create_subscription(HandoverDistance, '/handover/distance', lambda m: self._set('ee', m), 10)

        self.stats = dict.fromkeys(('frames', 'no object', 'no grasp', 'filtered', 'near hand', 'published'), 0)
        self.create_timer(5.0, self.log_stats)
        self.stop = threading.Event()
        try:
            self.backend = {'gsnet': Gsnet, 'anygrasp': AnyGrasp}[self.backend_name](
                *backend_args[self.backend_name])
            self.get_logger().info(f'grasp backend: {self.backend_name}')
            threading.Thread(target=self.worker, daemon=True).start()
        except Exception as error:
            self.backend = None
            self.get_logger().error(f'grasp backend {self.backend_name} not available ({error}): idle')

    def log_stats(self):
        if self.backend is not None and self.stats['frames']:
            self.get_logger().info('last 5 s: ' + ', '.join(f'{k} {v}' for k, v in self.stats.items()))
        self.stats = dict.fromkeys(self.stats, 0)

    def _set(self, key, msg):
        with self.lock:
            setattr(self, key, msg)

    def on_info(self, msg):
        if msg.k[0] > 0.0:
            self.K = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])

    # ---------------------------------------------------------------- worker
    def worker(self):
        while rclpy.ok() and not self.stop.is_set():
            t0 = time.monotonic()
            with self.lock:
                depth_msg, obj, hand, ee = self.depth, self.obj, self.hand, self.ee
                self.depth = None
            if depth_msg is not None and self.K is not None:
                self.stats['frames'] += 1
            if depth_msg is not None and self.K is not None and not (obj is not None and obj.object_present):
                self.stats['no object'] += 1
            if depth_msg is not None and self.K is not None and obj is not None and obj.object_present:
                try:
                    self.step(depth_msg, obj, hand, ee)
                except Exception as error:
                    self.get_logger().warn(f'grasp step failed: {error}', throttle_duration_sec=5.0)
            time.sleep(max(0.005, self.period - (time.monotonic() - t0)))

    def step(self, depth_msg, obj, hand, ee):
        centre = np.array([obj.object_centroid_3d.x, obj.object_centroid_3d.y, obj.object_centroid_3d.z])
        contour = np.array(obj.contour_px, np.int32).reshape(-1, 2)
        stamp = lambda m: m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec
        if not np.isfinite(centre).all() or len(contour) < 3 or abs(stamp(obj) - stamp(depth_msg)) > 0.3:
            return
        depth = self.bridge.imgmsg_to_cv2(depth_msg).astype(np.float32)
        if depth_msg.encoding == '16UC1':
            depth *= 1e-3
        fx, fy, cx, cy = self.K
        c_cam = self.R.T @ (centre - self.t)
        r = int(self.crop_m * fx / max(c_cam[2], 0.1))
        u_c, v_c = int(fx * c_cam[0] / c_cam[2] + cx), int(fy * c_cam[1] / c_cam[2] + cy)
        h, w = depth.shape
        u0, u1, v0, v1 = max(0, u_c - r), min(w, u_c + r), max(0, v_c - r), min(h, v_c + r)
        if u1 - u0 < 8 or v1 - v0 < 8:
            return
        mask = np.zeros((h, w), np.uint8)
        cv2.fillPoly(mask, [contour], 1)
        v, u = np.mgrid[v0:v1, u0:u1]
        z = depth[v0:v1, u0:u1]
        keep = (z > 0.1) & (np.abs(z - c_cam[2]) < self.crop_m)
        u, v, z = u[keep], v[keep], z[keep]
        points = np.column_stack([(u - cx) * z / fx, (v - cy) * z / fy, z]).astype(np.float32)
        region = mask[v, u].astype(bool)
        if region.sum() < 30:
            return
        # approach from the robot TCP towards the object, in the camera frame
        source = (np.array([ee.ee_control_point.x, ee.ee_control_point.y, ee.ee_control_point.z])
                  if ee is not None and ee.valid else np.array([0.0, 0.0, 0.5]))
        approach = self.R.T @ (centre - source)
        approach /= max(np.linalg.norm(approach), 1e-9)

        if self.backend_name == 'anygrasp':
            g = self.backend(points, region, approach, self.approach_thresh)
            if g is None:
                self.stats['no grasp'] += 1
                return
        else:
            g = self.backend(points)
            # on the object, from the robot side, within the gripper opening, then collision-free
            near = cKDTree(points[region]).query(g['t'], distance_upper_bound=0.02)[0] < 0.02
            ok = (near & (g['R'][:, :, 0] @ approach > np.cos(self.approach_thresh))
                  & (g['width'] <= self.max_width))
            g = {k: val[ok] for k, val in g.items()}
            if len(g['score']):
                free = ~collisions(points, g['t'], g['R'], g['width'], g['depth'], g['height'])
                g = {k: val[free] for k, val in g.items()}
        if not len(g['score']):
            self.stats['filtered'] += 1
            return
        self.publish(depth_msg, g, hand)

    def publish(self, depth_msg, g, hand):
        rot = g['R']  # graspnet: x approach, y closing
        tips = (g['t'] + g['depth'][:, None] * rot[:, :, 0]) @ self.R.T + self.t
        axes = np.einsum('ij,njk->nik', self.R, rot)
        scores = g['score']
        ok = np.ones(len(scores), bool)
        if hand is not None:  # fingertips away from the tracked hand landmarks
            lm = np.array([[p.x, p.y, p.z] for p, s in zip(hand.positions, hand.landmark_state) if s in TRACKED])
            if len(lm):
                for side in (-1.0, 1.0):
                    fingers = tips + side * 0.5 * g['width'][:, None] * axes[:, :, 1]
                    ok &= np.linalg.norm(fingers[:, None] - lm[None], axis=2).min(1) >= self.clearance
        if not ok.any():
            self.stats['near hand'] += 1
            return
        idx = np.where(ok)[0]
        best = idx[np.argmax(scores[idx])]
        if self.last is not None:
            near = [i for i in idx if np.linalg.norm(tips[i] - self.last[0]) < 0.03
                    and axes[i, :, 0] @ self.last[1] > np.cos(np.radians(30.0))
                    and scores[i] >= self.keep_ratio * scores[best]]
            if near:
                best = max(near, key=lambda i: scores[i])
        self.last = (tips[best], axes[best, :, 0])
        x_ap, y_cl = axes[best, :, 0], axes[best, :, 1]
        R_tcp = np.column_stack([np.cross(y_cl, x_ap), y_cl, x_ap])  # Franka TCP: z approach
        out = PoseStamped()
        out.header.stamp, out.header.frame_id = depth_msg.header.stamp, self.base_frame
        out.pose.position.x, out.pose.position.y, out.pose.position.z = map(float, tips[best])
        qx, qy, qz, qw = Rotation.from_matrix(R_tcp).as_quat()
        out.pose.orientation.x, out.pose.orientation.y = float(qx), float(qy)
        out.pose.orientation.z, out.pose.orientation.w = float(qz), float(qw)
        self.pub.publish(out)
        self.stats['published'] += 1


def main(args=None):
    rclpy.init(args=args)
    node = GraspPoseNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.stop.set()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
