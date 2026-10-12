#!/usr/bin/env python3
"""Handover grasp: the grasp pose on the object in the hand and the robot cycle that takes it.

  python3 handover_grasp.py pose ...      grasp pose node -> /handover/grasp_pose
  python3 handover_grasp.py executor ...  grasp cycle on the handover commander (robot_handover grasp_executor:=true)

GRASP POSE
  cloud      aligned depth around the object of HandObjectState (+-crop_m), at most max_rate_hz
  backend    GSNet (default) or AnyGrasp (backend:=anygrasp)
  threads    GPU thread (cloud + backend) and CPU thread (filters, collisions, choice) on consecutive
             frames: GSNet 49 ms, collisions on all its 1024 grasps 109 ms on the laptop (RTX 3050)
  filter     on the object, approach from the robot TCP within approach_thresh_deg, opening
             <= max_gripper_width_m, fingertips >= hand_clearance_m from the hand, then collision-free
             (GSNet), checked in score order 64 at a time
  palm       approach within palm_approach_deg of -palm normal (HandState, palm UP from above, SIDE from the
             palm side, DOWN from below): never through the back of the hand. 180 = off
  choice     best score; the previous grasp is kept while close and >= keep_ratio of the best
  rate       between two backend grasps the last one is republished at every HandState (camera rate),
             moved with the palm translation, up to propagate_s old: GSNet under the full pipeline
             load is ~160 ms (~6 Hz) on the laptop GPU. source_stamp = the backend frame
  output     PoseStamped in the base frame, Franka TCP convention (z approach, y closing)

GRASP EXECUTOR
  HOME -> PREGRASP (pregrasp_m back along the approach) -> APPROACH (approach_step_m) -> CLOSE
  -> HOLD (hold_s) -> OPEN -> RETREAT -> RETURN -> HOME
  A cycle starts with an empty hand first (rearm_s), an object in the hand, a fresh reachable grasp
  and a trusted hand. Hand / grasp lost (lost_s, approach_lost_s) or phase_timeout_s -> RETURN.
"""

import os
import site
import sys
import threading
import time
import types
from argparse import Namespace

from collections import deque

import cv2
import numpy as np
import pinocchio as pin
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from franka_msgs.msg import GraspPose, HandObjectState, HandoverDistance, HandState, HandTrackingFiltered
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation, Slerp
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

from franka_experiments.utils.kinematics import compute_arm_jacobian, so3_log
from franka_experiments.utils.node_runtime import run_node_main
from handover_qddot_commander import HandoverQddotCommander

# ============================================================ grasp pose
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
    fingers = ((x > d - finger_l) & (x < d)
               & (((y > -(w + finger_w)) & (y < -w)) | ((y < w + finger_w) & (y > w))))
    bottom = (x <= d - finger_l) & (x > d - finger_l - finger_w) & span
    shift = (x <= d - finger_l - finger_w) & (x > d - finger_l - finger_w - approach_dist) & span
    hits = (in_h & (fingers | bottom | shift)).sum(1)

    # collision when the cloud fills more than thresh of the gripper volume (in voxels)
    volume = ((2 * finger_l * finger_w + (2 * w[:, 0] + 2 * finger_w) * (finger_w + approach_dist))
              * 2 * h[:, 0])
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
        idx = (np.random.choice(n, self.num_point, replace=False) if n >= self.num_point
               else np.r_[np.arange(n), np.random.choice(n, self.num_point - n, replace=True)])
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
                                                  max_gripper_width=min(0.1, max_width),
                                                  gripper_height=height))
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
        self.clearance = float(par('hand_clearance_m', 0.05))         # finger pads - hand landmarks
        self.finger_clearance = float(par('finger_clearance_m', 0.03))  # finger pads - hand surface (depth)
        self.hand_radius = float(par('hand_radius_m', 0.12))          # hand surface: cloud around the palm
        self.validity_s = float(par('grasp_validity_s', 0.3))         # GraspPose.valid_until - stamp
        self.keep_ratio = float(par('keep_ratio', 0.8))            # hysteresis on the choice
        self.palm_approach = np.radians(float(par('palm_approach_deg', 70.0)))
        self.palm_max_age_s = float(par('palm_max_age_s', 0.3))
        self.propagate_s = float(par('propagate_s', 0.5))
        self.palms = deque(maxlen=60)  # (stamp, palm position) of HandState, for the moved grasp
        self.source = None             # (GraspPose, palm position at its stamp) of the last backend grasp
        self.period = 1.0 / float(par('max_rate_hz', 30.0))

        g = '/ros2_ws/graspness'
        backend_args = {
            'gsnet': (par('gsnet_root', f'{g}/graspness_unofficial'), par('gsnet_venv', f'{g}/venv'),
                      par('gsnet_checkpoint', f'{g}/ckpt/minkuresunet_realsense.tar')),
            'anygrasp': (par('anygrasp_sdk_dir', '/ros2_ws/anygrasp/grasp_detection'),
                         par('anygrasp_checkpoint', 'log/checkpoint_detection.tar'), self.max_width, 0.03)}

        with open(os.path.join(get_package_share_directory('franka_experiments'), 'config',
                               'camera_extrinsics.yaml'), 'r', encoding='utf-8') as file:
            ext = yaml.safe_load(file)
        t, q = ext['translation'], ext['rotation']
        self.t = np.array([t['x'], t['y'], t['z']])
        self.R = Rotation.from_quat([q['x'], q['y'], q['z'], q['w']]).as_matrix()  # camera -> base
        self.base_frame = ext['parent_frame']

        self.bridge, self.K = CvBridge(), None
        self.depth = self.obj = self.hand = self.ee = self.palm = None
        self.last = None  # (tip, approach) of the published grasp, base frame
        self.recent = deque(maxlen=10)  # (tip, approach) of the last frames that gave a grasp
        self.lock = threading.Lock()

        self.pub = self.create_publisher(GraspPose, '/handover/grasp_pose', 10)
        cam = '/camera/camera/aligned_depth_to_color/'
        self.create_subscription(CameraInfo, cam + 'camera_info', self.on_info, qos_profile_sensor_data)
        self.create_subscription(Image, cam + 'image_raw', lambda m: self._set('depth', m),
                                 qos_profile_sensor_data)
        self.create_subscription(HandObjectState, '/handover/hand_object', lambda m: self._set('obj', m), 10)
        self.create_subscription(HandTrackingFiltered, '/handover/hand_tracking_filtered',
                                 lambda m: self._set('hand', m), 10)
        self.create_subscription(HandoverDistance, '/handover/distance', lambda m: self._set('ee', m), 10)
        self.create_subscription(HandState, '/handover/hand_state', self.on_palm, 10)

        self.stats = dict.fromkeys(
            ('frames', 'no object', 'no grasp', 'filtered', 'near hand', 'palm side', 'collision', 'published',
             'moved'), 0)
        self.create_timer(5.0, self.log_stats)
        self.stop = threading.Event()
        try:
            self.backend = {'gsnet': Gsnet, 'anygrasp': AnyGrasp}[self.backend_name](
                *backend_args[self.backend_name])
            self.get_logger().info(f'grasp backend: {self.backend_name}')
            self.job, self.job_ready = None, threading.Event()  # GPU thread -> CPU thread, latest frame only
            threading.Thread(target=self.worker, daemon=True).start()
            threading.Thread(target=self.selector, daemon=True).start()
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

    def on_palm(self, msg):
        t = msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec
        n = np.array([msg.palm_normal.x, msg.palm_normal.y, msg.palm_normal.z])
        if msg.geometry_ok and np.linalg.norm(n) > 0.5:
            self._set('palm', (t, n / np.linalg.norm(n)))
        if not msg.position_valid:
            return
        palm = np.array([msg.palm_position.x, msg.palm_position.y, msg.palm_position.z])
        self.palms.append((t, palm))
        source = self.source
        if source is None or source[1] is None:
            return
        src, palm0 = source
        if not 0.0 < t - (src.header.stamp.sec + 1e-9 * src.header.stamp.nanosec) <= self.propagate_s:
            return
        out = GraspPose(source_stamp=src.source_stamp, width=src.width, score=src.score, confidence=src.confidence)
        out.header.frame_id, out.header.stamp = src.header.frame_id, msg.header.stamp
        out.pose.orientation = src.pose.orientation
        p = np.array([src.pose.position.x, src.pose.position.y, src.pose.position.z]) + palm - palm0
        out.pose.position.x, out.pose.position.y, out.pose.position.z = map(float, p)
        out.valid_until.sec, out.valid_until.nanosec = divmod(int((t + self.validity_s) * 1e9), 1_000_000_000)
        self.pub.publish(out)
        self.stats['moved'] += 1

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

    def selector(self):
        while rclpy.ok() and not self.stop.is_set():
            if not self.job_ready.wait(0.5):
                continue
            self.job_ready.clear()
            job, self.job = self.job, None
            if job is not None:
                try:
                    self.publish(*job)
                except Exception as error:
                    self.get_logger().warn(f'grasp choice failed: {error}', throttle_duration_sec=5.0)

    def step(self, depth_msg, obj, hand, ee):
        centre = np.array([obj.object_centroid_3d.x, obj.object_centroid_3d.y, obj.object_centroid_3d.z])
        contour = np.array(obj.contour_px, np.int32).reshape(-1, 2)
        stamp = lambda m: m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec
        if not np.isfinite(centre).all() or len(contour) < 3 or abs(stamp(obj) - stamp(depth_msg)) > 0.3:
            return

        depth = self.bridge.imgmsg_to_cv2(depth_msg).astype(np.float32)
        if depth_msg.encoding == '16UC1':
            depth *= 1e-3

        # crop around the object (+-crop_m) and back-project it; region = pixels inside the contour
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
            # on the object, from the robot side, within the gripper opening (collisions: CPU thread)
            near = cKDTree(points[region]).query(g['t'], distance_upper_bound=0.02)[0] < 0.02
            ok = (near & (g['R'][:, :, 0] @ approach > np.cos(self.approach_thresh))
                  & (g['width'] <= self.max_width))
            g = {k: val[ok] for k, val in g.items()}

        if not len(g['score']):
            self.stats['filtered'] += 1
            return
        # hand surface: cloud around the palm outside the object mask (fingers in any pose)
        hand_surface = np.zeros((0, 3))
        if hand is not None:
            mcp = [[p.x, p.y, p.z] for i, (p, s) in enumerate(zip(hand.positions, hand.landmark_state))
                   if i and s in TRACKED]
            if mcp:
                palm = self.R.T @ (np.mean(mcp, 0) - self.t)
                near = ~region & (np.linalg.norm(points - palm, axis=1) < self.hand_radius)
                hand_surface = points[near][::max(1, int(near.sum()) // 400)] @ self.R.T + self.t
        self.job = (depth_msg, g, hand, hand_surface, points)
        self.job_ready.set()

    def _collision_free(self, points, g, ok, tips, axes):
        """Free grasps among ok, checked in score order 64 at a time down to the first free one, plus the
        candidates that may keep the previous grasp: the same choice as checking them all."""
        if self.backend_name == 'anygrasp':  # checked inside the SDK
            return ok
        scores, free, checked = g['score'], np.zeros(len(ok), bool), np.zeros(len(ok), bool)

        def check(c):
            free[c] = ~collisions(points, g['t'][c], g['R'][c], g['width'][c], g['depth'][c], g['height'][c])
            checked[c] = True

        order = np.where(ok)[0][np.argsort(-scores[ok])]
        for i in range(0, len(order), 64):
            check(order[i:i + 64])
            if free.any():
                break
        if free.any() and self.last is not None:
            keep = (ok & ~checked & (scores >= self.keep_ratio * scores[free].max())
                    & (np.linalg.norm(tips - self.last[0], axis=1) < 0.03)
                    & (axes[:, :, 0] @ self.last[1] > np.cos(np.radians(30.0))))
            if keep.any():
                check(np.where(keep)[0])
        return free

    def publish(self, depth_msg, g, hand, hand_surface, points):
        rot = g['R']  # graspnet: x approach, y closing
        tips = (g['t'] + g['depth'][:, None] * rot[:, :, 0]) @ self.R.T + self.t  # camera -> base
        axes = np.einsum('ij,njk->nik', self.R, rot)
        scores = g['score']

        ok = np.ones(len(scores), bool)
        lm = np.array([[p.x, p.y, p.z] for p, s in zip(hand.positions, hand.landmark_state) if s in TRACKED]
                      if hand is not None else np.zeros((0, 3))).reshape(-1, 3)
        for side in (-1.0, 1.0):  # finger pads away from the landmarks and from the whole hand surface
            fingers = tips + side * 0.5 * g['width'][:, None] * axes[:, :, 1]
            for pts, clearance in ((lm, self.clearance), (hand_surface, self.finger_clearance)):
                if len(pts):
                    ok &= np.linalg.norm(fingers[:, None] - pts[None], axis=2).min(1) >= clearance
        if not ok.any():
            self.stats['near hand'] += 1
            return
        palm = self.palm
        t = depth_msg.header.stamp.sec + 1e-9 * depth_msg.header.stamp.nanosec
        if palm is not None and abs(t - palm[0]) <= self.palm_max_age_s and self.palm_approach < np.pi:
            ok &= axes[:, :, 0] @ -palm[1] > np.cos(self.palm_approach)
            if not ok.any():
                self.stats['palm side'] += 1
                return
        ok = self._collision_free(points, g, ok, tips, axes)
        if not ok.any():
            self.stats['collision'] += 1
            return

        # best score, but keep the previous grasp while it is still close and good enough
        idx = np.where(ok)[0]
        best = idx[np.argmax(scores[idx])]
        if self.last is not None:
            near = [i for i in idx if np.linalg.norm(tips[i] - self.last[0]) < 0.03
                    and axes[i, :, 0] @ self.last[1] > np.cos(np.radians(30.0))
                    and scores[i] >= self.keep_ratio * scores[best]]
            if near:
                best = max(near, key=lambda i: scores[i])
        self.last = (tips[best], axes[best, :, 0])
        same = [np.linalg.norm(tip - tips[best]) < 0.03 and a @ axes[best, :, 0] > np.cos(np.radians(30.0))
                for tip, a in self.recent]
        self.recent.append(self.last)

        x_ap, y_cl = axes[best, :, 0], axes[best, :, 1]
        R_tcp = np.column_stack([np.cross(y_cl, x_ap), y_cl, x_ap])  # Franka TCP: z approach

        out = GraspPose(width=float(g['width'][best]), score=float(scores[best]))
        out.header.stamp, out.header.frame_id = depth_msg.header.stamp, self.base_frame
        out.source_stamp = depth_msg.header.stamp
        out.confidence = float(np.clip(scores[best], 0.0, 1.0) * (1 + sum(same)) / (1 + len(same)))
        out.valid_until.sec, out.valid_until.nanosec = divmod(int((t + self.validity_s) * 1e9), 1_000_000_000)
        out.pose.position.x, out.pose.position.y, out.pose.position.z = map(float, tips[best])
        qx, qy, qz, qw = Rotation.from_matrix(R_tcp).as_quat()
        out.pose.orientation.x, out.pose.orientation.y = float(qx), float(qy)
        out.pose.orientation.z, out.pose.orientation.w = float(qz), float(qw)
        self.pub.publish(out)
        self.stats['published'] += 1
        near = [(abs(tp - t), p) for tp, p in list(self.palms) if abs(tp - t) <= 0.1]
        self.source = (out, min(near, key=lambda x: x[0])[1] if near else None)


# ============================================================ grasp executor
class GraspExecutor(HandoverQddotCommander):

    def __init__(self):
        super().__init__()
        par = lambda n, v: self.declare_parameter(n, v).value
        self.pregrasp_m = float(par('pregrasp_m', 0.10))
        self.approach_step_m = float(par('approach_step_m', 0.02))  # slow: ~0.1 m/s
        self.reach_tol_m = float(par('reach_tol_m', 0.015))
        self.rot_tol = np.radians(float(par('rot_tol_deg', 10.0)))
        self.rot_rate = np.radians(float(par('rot_rate_deg_s', 45.0)))
        self.hold_s = float(par('hold_s', 2.0))
        self.gripper_s = float(par('gripper_s', 1.5))               # time given to each gripper motion
        self.rearm_s = float(par('rearm_s', 1.0))                   # empty hand before the next cycle
        self.grasp_timeout_s = float(par('grasp_timeout_s', 0.3))  # fresh grasp to start
        self.lost_s = float(par('lost_s', 1.0))                     # gap tolerated before PREGRASP aborts
        self.approach_lost_s = float(par('approach_lost_s', 0.4))  # same, slow final approach
        self.retry_s = float(par('retry_s', 1.0))                   # wait at home after an abort
        self.max_reach_m = float(par('max_reach_m', 0.80))
        self.min_z_m = float(par('min_z_m', 0.10))
        self.phase_timeout_s = float(par('phase_timeout_s', 8.0))
        self.min_manipulability = float(par('min_manipulability', 0.01))   # sqrt(det J J^T) at the grasp
        self.joint_margin = float(par('ik_joint_margin_rad', 0.05))
        # observer RETREATING during PREGRASP / APPROACH: hold the arm; back home after retreat_abort_s
        par('pause_on_retreat', True)
        par('retreat_abort_s', 0.5)
        self._retreat_t = None
        self._ik_data, self._ik_cache = self.pin_model.createData(), {}
        self._ik_q_ids = [self.pin_model.joints[j].idx_q for j in self._pin_joint_ids]

        self._grasp = None          # (position, R_tcp, stamp_ns)
        self._frozen = None         # grasp used by APPROACH / CLOSE / HOLD
        self._object = False
        self._armed, self._empty_since = True, None   # a new cycle needs an empty hand first
        self._last_t = None
        self._good_t = None         # last time grasp and hand were both fine
        self._retry_at = 0.0
        self._phase, self._phase_t = 'HOME', self._now()
        self._R_home = None

        self.create_subscription(GraspPose, '/handover/grasp_pose', self._grasp_cb, 10)
        self.phase_pub = self.create_publisher(String, '/handover/grasp_phase', 10)  # 'PHASE|abort reason'
        self.create_subscription(HandObjectState, '/handover/hand_object', self._object_cb, 10)
        self.get_logger().info('Grasp executor ready: HOME')

    # ------------------------------------------------------------ inputs
    def _grasp_cb(self, msg):
        p, q = msg.pose.position, msg.pose.orientation
        R = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        self._grasp = (np.array([p.x, p.y, p.z]), R,
                       msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec)
        self._grasp_valid_until = msg.valid_until.sec * 1_000_000_000 + msg.valid_until.nanosec
        # IK once per backend grasp, not at every republished (moved) one
        self._grasp_source = msg.source_stamp.sec * 1_000_000_000 + msg.source_stamp.nanosec or self._grasp[2]

    def _object_cb(self, msg):
        self._object = bool(msg.valid and msg.object_present)
        if self._object or not msg.valid:
            self._empty_since = None
        elif self._empty_since is None:
            self._empty_since = self._now()
        elif self._now() - self._empty_since >= self.rearm_s:
            self._armed = True

    def _fresh_grasp(self, max_age=None):
        if self._grasp is None:
            return None

        age = (self.get_clock().now().nanoseconds - self._grasp[2]) * 1e-9
        p, R, stamp = self._grasp
        reachable = np.linalg.norm(p[:2]) <= self.max_reach_m and p[2] >= self.min_z_m
        limit = self.grasp_timeout_s if max_age is None else max_age
        if not (0.0 <= age <= limit and reachable):
            return None
        if max_age is None:  # a new grasp: still valid and reachable by the arm
            if self.get_clock().now().nanoseconds > self._grasp_valid_until:
                return None
            key = self._grasp_source
            if key not in self._ik_cache:
                R = self._closest_flip(R)
                self._ik_cache = {key: self.ik_check(p - self.pregrasp_m * R[:, 2], R) or self.ik_check(p, R)}
            if self._ik_cache[key]:
                self.get_logger().info(f'grasp rejected: {self._ik_cache[key]}', throttle_duration_sec=2.0)
                return None
        return self._grasp

    def ik_check(self, p, R):
        """Damped least squares IK from the current joints to the TCP pose (p, R): None if it converges
        within 5 mm / 3 deg, ik_joint_margin_rad inside the joint limits, with sqrt(det J J^T) >=
        min_manipulability; otherwise the reason."""
        m, d, qi = self.pin_model, self._ik_data, self._ik_q_ids
        lo, hi = m.lowerPositionLimit[qi], m.upperPositionLimit[qi]
        q = self._q_full.copy()
        for _ in range(100):
            J = compute_arm_jacobian(m, d, q, self.ee_frame_id, self._pin_joint_ids)
            pin.updateFramePlacement(m, d, self.ee_frame_id)
            T = d.oMf[self.ee_frame_id]
            err = np.r_[p - T.translation, so3_log(R @ T.rotation.T)]
            if np.linalg.norm(err[:3]) < 0.005 and np.linalg.norm(err[3:]) < np.radians(3.0):
                break
            q[qi] = np.clip(q[qi] + J.T @ np.linalg.solve(J @ J.T + 1e-4 * np.eye(6), err), lo, hi)
        else:
            return 'IK does not converge (out of reach)'
        if np.any(q[qi] - lo < self.joint_margin) or np.any(hi - q[qi] < self.joint_margin):
            return 'at a joint limit'
        manipulability = float(np.sqrt(max(np.linalg.det(J @ J.T), 0.0)))
        return None if manipulability >= self.min_manipulability else f'manipulability {manipulability:.4f}'

    def _now(self):
        """ROS time [s]: the bag clock in replays, like the stamps of the data."""
        return self.get_clock().now().nanoseconds * 1e-9

    def _set_phase(self, phase, reason=''):
        self.get_logger().info(f'{self._phase} -> {phase}')
        if reason:
            self.get_logger().warn(reason)
        self._phase, self._phase_t = phase, self._now()
        self.phase_pub.publish(String(data=f'{phase}|{reason}'))

    # ------------------------------------------------------------ orientation
    def _turn_towards(self, R_goal, dt):
        """Rotate the held orientation towards R_goal at rot_rate (base controller hold)."""
        R_now = Rotation.from_matrix(self._R_des)
        angle = (R_now.inv() * Rotation.from_matrix(R_goal)).magnitude()
        if angle > 1e-4:
            f = min(1.0, self.rot_rate * dt / angle)
            R_new = Slerp([0.0, 1.0], Rotation.concatenate([R_now, Rotation.from_matrix(R_goal)]))(f)
            self._R_des[:] = R_new.as_matrix()
            self._R_des_nom[:] = self._R_des
        return angle

    def _closest_flip(self, R):
        """A parallel gripper grasp is the same turned by 180 deg about z: take the nearer one."""
        flipped = R @ np.diag([-1.0, -1.0, 1.0])
        now = Rotation.from_matrix(self._R_des)
        d = lambda A: (now.inv() * Rotation.from_matrix(A)).magnitude()
        return R if d(R) <= d(flipped) else flipped

    # ------------------------------------------------------------ target
    def desired_position(self):
        if self._hold_position is None:
            return self._p_ee.copy()
        if self._R_home is None and self._orient_ok:
            self._R_home = self._R_des.copy()

        now = self._now()
        dt = min(0.05, max(0.0, now - self._last_t)) if self._last_t is not None else 0.0
        self._last_t = now
        elapsed = now - self._phase_t
        home = self._hold_position
        grasp = self._fresh_grasp()
        if grasp is not None and self.hand_trusted():
            self._good_t = now

        if self._phase == 'HOME':
            if self._R_home is not None:
                self._turn_towards(self._R_home, dt)
            ready = {'armed (empty hand first)': self._armed, 'object in hand': self._object,
                     'fresh reachable grasp': grasp is not None, 'hand trusted': self.hand_trusted()}
            if not all(ready.values()):
                missing = ', '.join(k for k, ok in ready.items() if not ok)
                self.get_logger().info(f'HOME, waiting for: {missing}', throttle_duration_sec=2.0)
            if all(ready.values()) and now >= self._retry_at:
                self._armed, self._good_t = False, now
                self.gripper.command(False)
                self._set_phase('PREGRASP')
            return self._step_to(home, self.max_step())

        if self._phase in ('PREGRASP', 'APPROACH'):
            # short gaps (grasp at ~10 Hz, tracking dropouts) ride on the last grasp
            tolerance = self.lost_s if self._phase == 'PREGRASP' else self.approach_lost_s
            if self._good_t is None or now - self._good_t > tolerance:
                self._armed, self._retry_at = True, now + self.retry_s  # nothing was grasped
                self._set_phase('RETURN', 'hand / grasp lost: back home, gripper stays open, retry')
                return self._p_ee.copy()
            grasp = grasp or self._fresh_grasp(max_age=tolerance)
            if grasp is None and self._phase == 'PREGRASP':
                return self._p_ee.copy()  # hold while waiting for the next grasp
            if self.get_parameter('pause_on_retreat').value and self.retreating():  # the hand is taken back
                self._retreat_t = self._retreat_t or now
                if now - self._retreat_t > self.get_parameter('retreat_abort_s').value:
                    self._armed, self._retry_at, self._retreat_t = True, now + self.retry_s, None
                    self._set_phase('RETURN', 'hand withdrawn: back home, retry')
                self._phase_t += dt  # the pause does not count towards phase_timeout_s
                return self._p_ee.copy()
            self._retreat_t = None

        if self._phase == 'PREGRASP':
            p, R, _ = grasp
            R = self._closest_flip(R)
            angle = self._turn_towards(R, dt)
            target = p - self.pregrasp_m * R[:, 2]
            # final approach only on a fresh grasp
            if (np.linalg.norm(target - self._p_ee) < self.reach_tol_m * 2
                    and angle < self.rot_tol
                    and now - self._good_t <= self.grasp_timeout_s):
                self._frozen = (p.copy(), R.copy())
                self._set_phase('APPROACH')
            elif elapsed > self.phase_timeout_s:
                self._set_phase('RETURN', f'PREGRASP not reached in {self.phase_timeout_s:.0f} s')
            return self._step_to(target, self.max_step())

        if self._phase == 'APPROACH':
            p, R = self._frozen
            self._turn_towards(R, dt)
            if np.linalg.norm(p - self._p_ee) < self.reach_tol_m:
                self.gripper.command(True)
                self._set_phase('CLOSE')
            elif elapsed > self.phase_timeout_s:
                self._set_phase('RETURN', f'grasp not reached in {self.phase_timeout_s:.0f} s')
            return self._step_to(p, self.approach_step_m)

        if self._phase in ('CLOSE', 'HOLD', 'OPEN'):
            if self._phase == 'CLOSE' and elapsed >= self.gripper_s:
                self._set_phase('HOLD')
            elif self._phase == 'HOLD' and elapsed >= self.hold_s:
                self.gripper.command(False)
                self._set_phase('OPEN')
            elif self._phase == 'OPEN' and elapsed >= self.gripper_s:
                self._set_phase('RETREAT')
            return self._step_to(self._frozen[0], self.approach_step_m)

        if self._phase == 'RETREAT':  # straight back along the approach, away from the hand
            p, R = self._frozen
            back = p - self.pregrasp_m * R[:, 2]
            if np.linalg.norm(back - self._p_ee) < self.reach_tol_m * 2 or elapsed > self.phase_timeout_s:
                self._set_phase('RETURN')
            return self._step_to(back, self.approach_step_m)

        # RETURN: home position and orientation
        if self._R_home is not None:
            angle = self._turn_towards(self._R_home, dt)
        else:
            angle = 0.0
        if np.linalg.norm(home - self._p_ee) < self.reach_tol_m * 2 and angle < self.rot_tol:
            self._frozen = None
            self._set_phase('HOME')
        return self._step_to(home, self.max_step())


def main(args=None):
    role = sys.argv[1] if len(sys.argv) > 1 and not sys.argv[1].startswith('-') else 'executor'
    if role == 'executor':
        run_node_main(GraspExecutor, args=args)
        return

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
