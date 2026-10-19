#!/usr/bin/env python3
"""Handover grasp: the grasp pose on the object in the hand and the robot cycle that takes it.

  python3 handover_grasp.py pose ...      grasp pose node -> /handover/grasp_pose
  python3 handover_grasp.py executor ...  grasp cycle on the handover commander (robot_handover grasp_executor:=true)

GRASP POSE
  camera     the gripper D405 only (TF base -> fr3_link8 + d405_extrinsics.yaml): the main camera finds the hand
             and the object, the executor's PREGRASP brings the D405 to 0.25-0.35 m of the hand (GraspNet scale,
             sharper depth, larger object, no main camera calibration error). Idle without gripper_camera
  region     only a confirmed object (HandObjectState.object_present) detected by Hands23 on the D405
             (gripper_camera), with its outline (EfficientTAM, grasp.py), once per detection: on the newest D405
             frame with the detection outline moved there by the optical flow (the detection frame of the
             detection_stamp is then ~0.27 s old; kept if the flow fails), palm normal of that frame. 17/10 night,
             D405 bag: frame age when HGGD starts 274 -> 60 ms, grasps 344 (350), off object 9 (12). 16/10,
             user's 50 contours: grasp centre on the object 100 % (outline moved with the palm onto the newest frame:
             81 %); 14/10 lab bag, ball: grasp centre on the ball 94 % with the D405 mask, 36 % with a sphere
  backend    HGGD (Chen et al., RA-L 2023, MIT) with TensorRT FP16 nets, hggd_num_point scene points within crop_m
             of the object: 72 / 88 ms (p50 / p90) under the full pipeline on the laptop (RTX 3050), 20 ms with a
             free GPU; the tracker stays at 30 Hz
  threads    GPU thread (cloud + HGGD) and CPU thread (filters, collisions, choice) on consecutive frames
  yield      the tracker first: HGGD waits tracker_yield_s after the hand tracker skipped a camera frame (a main
             camera frame, its camera_info stamp, without its HandTrackingRaw), so a smaller GPU (laptop RTX 3050, D405 active) runs HGGD
             less often instead of slowing the tracker; a GPU with headroom (lab RTX 5090) never waits
  filter     on the object, approach from the robot TCP within approach_thresh_deg, opening
             <= max_gripper_width_m; hand skeleton of that frame (21 points; tracker HandTrackingRaw.hand_points
             of the tracker would be ~5 cm off on the D405: MediaPipe on the D405 frame itself): whole gripper >= BONE_R + HAND_GAP from the
             bones and the object between the fingers. Without a skeleton: finger pads >= hand_clearance_m from
             the 4 landmarks and >= finger_clearance_m from the hand surface. Then collision-free, checked in score
             order 64 at a time. 16-17/10: grasps 53-59 / 10-22 per bag replay (1005 / rosbag2) instead of 10-11 /
             7-11; user's 50 contours: grasp centre on the object 100 %
  palm       approach within palm_approach_deg of -palm normal (HandState, palm UP from above, SIDE from the
             palm side, DOWN from below): never through the back of the hand. 180 = off
  choice     best score; the previous grasp is kept while close and >= keep_ratio of the best
  rate       between two HGGD grasps the last one is republished at every HandState (camera rate), up to propagate_s
             old, moved with the object as the D405 sees it (track: optical flow of the outline + D405 depth and pose;
             17/10 D405 bag: 1 / 8 mm median / p90 against the next detection, the main camera palm 11 / 58 mm), the
             palm translation when the flow fails. source_stamp = the HGGD frame. The new grasp itself goes out moved
             to the newest D405 frame already: published pose 47 ms old instead of 456 (17/10 night, D405 bag)
  output     PoseStamped in the base frame, Franka TCP convention (z approach, y closing)

GRASP EXECUTOR
  HOME -> PREGRASP -> APPROACH (follows the fresh grasps that stay within 3 cm / 30 deg, frozen in the last freeze_m;
  slows down to approach_step_m in the last slow_m) -> CLOSE -> HOLD (hold_s) -> OPEN -> RETREAT -> RETURN -> HOME
  A cycle starts with an empty hand first (rearm_s), an object in the hand (main camera), the observer
  APPROACHING, a trusted hand and a reachable pregrasp estimate.
  PREGRASP is one motion, no stop to look: the target is the pregrasp estimated from the hand alone (view_m from
  the palm along the palm normal, on the robot side, within view_tilt_deg of the vertical, the approach axis into
  the palm: the direction the grasps must come from, palm_approach_deg) until the first D405 grasp, then blended in
  blend_s into that grasp's pregrasp (pregrasp_m back along its approach). On the way the approach axis, and the D405
  that looks along it, points at the object (HandObjectState centroid, else the palm) and turns into the pregrasp
  orientation in the last look_m: Hands23 switches to the D405 and confirms the object while the arm moves, HGGD
  (D405 only) gives the grasp. Hand estimate not reachable: halfway to the vertical, then from above, else the arm
  holds; a palm jump > JUMP_M between two tracker frames holds it 0.5 s.
  Hand lost (lost_s), grasp lost in APPROACH (approach_lost_s) or phase_timeout_s -> RETURN.
"""

import os
import site
import sys
import threading
import time

from collections import deque

import cv2
import mediapipe as mp
import numpy as np
import pinocchio as pin
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from franka_msgs.msg import (GraspPose, HandObjectState, HandoverDistance, HandoverObserver, HandState,
                             HandTrackingFiltered, HandTrackingRaw)
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation, Slerp
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from franka_experiments.utils.kinematics import compute_arm_jacobian, so3_log
from franka_experiments.utils.node_runtime import run_node_main
from handover_qddot_commander import HandoverQddotCommander

# ============================================================ grasp pose
TRACKED = (HandTrackingFiltered.TRACKING, HandTrackingFiltered.PREDICT_ONLY)
# hand skeleton: bones between the 21 keypoints (MediaPipe order), finger radius, free gap to the gripper
BONES = np.array([(0, 1), (1, 2), (2, 3), (3, 4), (0, 5), (5, 6), (6, 7), (7, 8), (0, 9), (9, 10), (10, 11),
                  (11, 12), (0, 13), (13, 14), (14, 15), (15, 16), (0, 17), (17, 18), (18, 19), (19, 20), (5, 9),
                  (9, 13), (13, 17)])
BONE_R, HAND_GAP = 0.01, 0.015
# object between the fingers: points in the finger slab (FINGER_L x opening x 2 SLAB_Z), at least MIN_IN within
# the opening, at most MAX_OUT of them beyond the pads, centre within CENTRE of the half opening
FINGER_L, SLAB_Z, MIN_IN, MAX_OUT, CENTRE = 0.06, 0.01, 15, 0.10, 0.35


def stamp(msg):
    return msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec


def outline_centre(mask, depth, R, t_cam, K, shift=(0.0, 0.0), scale=1.0):
    """Base-frame centre of the outline mask moved by shift (px) in a D405 frame: mean pixel, median depth (depth
    image x scale = m)."""
    fx, fy, cx, cy = K
    v, u = np.nonzero(mask)
    u, v = np.round(u + shift[0]).astype(int), np.round(v + shift[1]).astype(int)
    ok = (u >= 0) & (u < depth.shape[1]) & (v >= 0) & (v < depth.shape[0])
    z = depth[v[ok], u[ok]].astype(np.float32) * scale
    z = z[(z > 0.07) & (z < 1.0)]
    if len(z) < 30:
        return None
    z = float(np.median(z))
    return R @ np.array([(u[ok].mean() - cx) * z / fx, (v[ok].mean() - cy) * z / fy, z]) + t_cam


def flow_shift(grey0, grey1, corners):
    """Median Lucas-Kanade shift (px) of corners from grey0 to grey1, forward-backward checked; None if < 5 agree."""
    p1, ok, _ = cv2.calcOpticalFlowPyrLK(grey0, grey1, corners, None, winSize=(21, 21), maxLevel=3)
    back, ok2, _ = cv2.calcOpticalFlowPyrLK(grey1, grey0, p1, None, winSize=(21, 21), maxLevel=3)
    good = (ok.ravel() == 1) & (ok2.ravel() == 1) & (np.linalg.norm(back - corners, axis=2).ravel() < 1.0)
    return np.median((p1 - corners)[good].reshape(-1, 2), axis=0) if good.sum() >= 5 else None


def outline_corners(grey, mask):
    """Corners to track inside an outline (dilated 4 px), None if fewer than 5."""
    c = cv2.goodFeaturesToTrack(grey, 40, 0.01, 4, mask=cv2.dilate(mask.astype(np.uint8) * 255, np.ones((9, 9), np.uint8)))
    return None if c is None or len(c) < 5 else c


def skeleton_of(kp):
    """Bones (A, B) between the 21 hand points (base frame, NaN rows unknown): None with < 8 points."""
    bones = BONES[np.isfinite(kp[BONES]).all((1, 2))]
    return (kp[bones[:, 0]], kp[bones[:, 1]]) if np.isfinite(kp[:, 0]).sum() >= 8 and len(bones) else None


def seg_dist(P, A, B):
    """Distance of points P (N, 3) from the segments A -> B (M, 3): (N, M)."""
    AB = B - A
    t = np.clip(np.einsum('nmk,mk->nm', P[:, None] - A[None], AB) / np.maximum((AB * AB).sum(1), 1e-12), 0, 1)
    return np.linalg.norm(P[:, None] - (A[None] + t[..., None] * AB[None]), axis=2)


def skeleton_clear(tips, axes, width, A, B):
    """Whole gripper (both fingers pad -> FINGER_L back, palm bar) >= BONE_R + HAND_GAP from the bones A -> B."""
    x, y, s = axes[:, :, 0], axes[:, :, 1], np.linspace(0, 1, 7)
    pts = [tips + side * 0.5 * width[:, None] * y - f * FINGER_L * x for side in (-1.0, 1.0) for f in s]
    pts += [tips - FINGER_L * x + f * 0.5 * width[:, None] * y for f in np.linspace(-1, 1, 7)]
    G = np.stack(pts, 1)
    return seg_dist(G.reshape(-1, 3), A, B).min(1).reshape(len(tips), -1).min(1) >= BONE_R + HAND_GAP


def between_fingers(obj, centre, axes, depth, width):
    """Object points obj (base) held between the fingers of each candidate (centre, axes x approach / y closing)."""
    ok = np.zeros(len(centre), bool)
    for i in range(len(centre)):
        q = (obj - centre[i]) @ axes[i]
        y = q[(q[:, 0] > depth[i] - FINGER_L) & (q[:, 0] < depth[i]) & (np.abs(q[:, 2]) < SLAB_Z), 1]
        inside = np.abs(y) <= 0.5 * width[i]
        ok[i] = (inside.sum() >= MIN_IN and 1.0 - inside.mean() <= MAX_OUT
                 and abs(y.max() + y.min()) / max(width[i], 2e-3) <= CENTRE)
    return ok


def voxels(points, voxel=0.01):
    """Centres of the occupied voxels (one int64 key per voxel: same set as np.unique(axis=0), ~10x faster)."""
    k = np.round(points / voxel).astype(np.int64) + (1 << 20)
    u = np.unique((k[:, 0] << 42) | (k[:, 1] << 21) | k[:, 2])
    return (np.column_stack([u >> 42, (u >> 21) & 0x1FFFFF, u & 0x1FFFFF]) - (1 << 20)) * voxel


def collisions(points, t, R, widths, depths, heights, finger_w=0.01, finger_l=0.06,
               approach_dist=0.05, voxel=0.01, thresh=0.01):
    """GraspNet model-free collision check (graspness_unofficial), numpy only.

    Grasp frame: x approach, y closing, z height. True = gripper volume hits the cloud."""
    pts = voxels(points, voxel)  # voxel downsample
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


class TrtNet:
    """TensorRT FP16 engine of an HGGD net, built once from its ONNX: torch CUDA tensors in and out."""

    def __init__(self, trt_dir, name, profile=None):
        import tensorrt as trt
        log, plan = trt.Logger(trt.Logger.WARNING), f'{trt_dir}/{name}.plan'
        if not os.path.exists(plan):  # engines are per GPU
            builder = trt.Builder(log)
            net = builder.create_network(0)
            parser = trt.OnnxParser(net, log)
            with open(f'{trt_dir}/{name}.onnx', 'rb') as file:
                if not parser.parse(file.read()):
                    raise RuntimeError(parser.get_error(0))
            config = builder.create_builder_config()
            config.set_flag(trt.BuilderFlag.FP16)
            if profile:
                shapes = builder.create_optimization_profile()
                for name_in, (low, opt, high) in profile.items():
                    shapes.set_shape(name_in, low, opt, high)
                config.add_optimization_profile(shapes)
            with open(plan, 'wb') as file:
                file.write(builder.build_serialized_network(net, config))
        with open(plan, 'rb') as file:
            engine = trt.Runtime(log).deserialize_cuda_engine(file.read())
        self.ctx = engine.create_execution_context()
        names = [engine.get_tensor_name(i) for i in range(engine.num_io_tensors)]
        is_input = lambda n: engine.get_tensor_mode(n) == trt.TensorIOMode.INPUT
        self.inputs, self.outputs = [n for n in names if is_input(n)], [n for n in names if not is_input(n)]

    def __call__(self, *args):
        import torch
        self.args = [a.contiguous().float() for a in args]  # alive until the stream has run
        for name, a in zip(self.inputs, self.args):
            self.ctx.set_input_shape(name, tuple(a.shape))
            self.ctx.set_tensor_address(name, a.data_ptr())
        outs = [torch.empty(tuple(self.ctx.get_tensor_shape(n)), device='cuda') for n in self.outputs]
        for name, out in zip(self.outputs, outs):
            self.ctx.set_tensor_address(name, out.data_ptr())
        self.ctx.execute_async_v3(torch.cuda.current_stream().cuda_stream)
        return outs


class Hggd:
    """HGGD inference -> grasps (graspnet format: x approach, y closing) in the camera frame.

    HGGD learns the 2D grasp widths in pixels on GraspNet (RealSense fx 927): a window around the object is zoomed
    to 1280x720 (K scaled, at most max_zoom) so that the object has the pixel size it would have at ref_m. ref_m 0.3:
    the widths match the objects (ball ~7 cm -> 8-9 cm, screwdriver handle 6 cm); at 0.6 they came out 1.5-2x wider
    (14/10, bags at 1.4 m). Heatmap only on the object pixels (region steering), then the 6D stage on the scene
    points within crop_m of the object."""

    def __init__(self, root, num_point=8000, crop_m=0.25, ref_m=0.3, max_zoom=10.0):
        site.addsitedir(f'{root}/venv/lib/python3.10/site-packages')  # numba
        sys.path[:0] = [f'{root}/stubs', f'{root}/HGGD']  # stand-ins of pytorch3d & co.: stubs/README.txt
        import torch
        import torch.nn.functional as F
        import dataset.config as config
        import dataset.evaluation as evaluation
        import dataset.pc_dataset_tools as pc_tools

        # intrinsics of the zoomed frame, in every HGGD module that imported get_camera_intrinsic (2D -> 6D too)
        self.K, fixed = np.eye(3), config.get_camera_intrinsic
        for module in list(sys.modules.values()):
            if getattr(module, 'get_camera_intrinsic', None) is fixed:
                module.get_camera_intrinsic = lambda camera=None: self.K.copy()
        self.torch, self.F, self.ev, self.pc = torch, F, evaluation, pc_tools
        self.num_point, self.crop_m, self.ref_m, self.max_zoom = num_point, crop_m, ref_m, max_zoom

        ck = torch.load(f'{root}/ckpt/HGGD_realsense_checkpoint', map_location='cuda', weights_only=False)
        self.anchors = {'gamma': ck['gamma'], 'beta': ck['beta']}  # the nets run as TensorRT engines
        anchornet = TrtNet(f'{root}/trt', 'anchornet')
        self.anchornet = lambda x: (lambda o: (tuple(o[:5]), o[5]))(anchornet(x))  # 2D maps, features
        groups = {'pc': ((1, 512, 35), (48, 512, 35), (128, 512, 35)), 'info': ((1, 3), (48, 3), (128, 3))}
        self.localnet = TrtNet(f'{root}/trt', 'localnet', groups)  # 1-128 local point groups

    def __call__(self, rgb, depth, K, region):
        """rgb HxWx3 uint8, depth [m] HxW, K (fx, fy, cx, cy), region HxW bool: the object pixels (not empty)."""
        torch, F = self.torch, self.F
        fx, fy, cx, cy = K
        h, w = depth.shape
        vv, uu = np.nonzero(region)
        z_obj = float(np.median(depth[region]))
        zoom = float(np.clip(927.0 / fx * z_obj / self.ref_m, 1280 / w, self.max_zoom))
        ww, hh = round(1280 / zoom), round(720 / zoom)
        u0 = int(np.clip(int(uu.mean()) - ww // 2, 0, max(0, w - ww)))
        v0 = int(np.clip(int(vv.mean()) - hh // 2, 0, max(0, h - hh)))
        win, s = np.s_[v0:v0 + hh, u0:u0 + ww], 1280 / ww
        self.K[:] = [[s * fx, 0, s * (cx - u0)], [0, s * fy, s * (cy - v0)], [0, 0, 1]]
        d = torch.from_numpy(np.clip(depth[win] * 1e3, 0, 3000)).float().cuda()        # mm
        c = torch.from_numpy(rgb[win] / 255.0).float().cuda().permute(2, 1, 0)[None]   # 1x3xWxH
        depth_v = F.interpolate(d.T[None, None], (1280, 720), mode='nearest')[0]       # 1x1280x720
        rgb_v = F.interpolate(c, (1280, 720), mode='bilinear', align_corners=False)

        with torch.no_grad():
            # scene points within crop_m of the object, in pixels and in depth
            vv, uu = np.nonzero(region[win])
            r = self.crop_m * fx / max(z_obj, 0.1)
            crop = torch.zeros((1280, 720), dtype=torch.bool, device='cuda')
            crop[int(max(0, s * (uu.min() - r))):int(s * (uu.max() + r)) + 1,
                 int(max(0, s * (vv.min() - r))):int(s * (vv.max() + r)) + 1] = True
            crop &= (depth_v[0] / 1e3 - z_obj).abs() < self.crop_m
            points = self._scene_points(rgb_v, depth_v, crop)
            if points.shape[1] < 32:  # no depth around the object
                return None

            x_d = F.interpolate(depth_v[None], (640, 360))[0] / 1e3
            x = torch.cat([torch.clip(x_d - x_d.mean(), -1, 1)[None], F.interpolate(rgb_v, (640, 360))], 1)
            pred_2d, feats = self.anchornet(x)
            loc_map, cls_mask, th, dz, dw = self.ev.anchor_output_process(*pred_2d, sigma=10)
            steer = F.interpolate(torch.from_numpy(region[win].T.astype(np.float32))[None, None], loc_map.shape)
            rect = self.ev.detect_2d_grasp(loc_map * steer[0, 0].numpy(), cls_mask, th, dz, dw, ratio=8,
                                           anchor_k=6, anchor_w=50.0, anchor_z=20.0, mask_thre=0.01, center_num=48,
                                           grid_size=8, grasp_nms=8, reduce='max')
            if rect.size == 0:
                return None
            points = self.pc.feature_fusion(points[..., :3], feats, self._xyz_maps(depth_v))
            rects = [rect]
            pc_group, valid = self.pc.data_process(points, depth_v, rects, 48, 512, (640, 360), min_points=32,
                                                   is_training=False)
            if not len(pc_group):
                return None
            info = torch.from_numpy(np.vstack([rects[0].thetas, rects[0].widths, rects[0].depths]).T).float().cuda()
            _, pred, offset = self.localnet(pc_group, info)
            _, rect_6d = self.ev.detect_6d_grasp_multi(rects[0], pred, offset, valid, (640, 360), self.anchors, k=10)
            gg = rect_6d.to_6d_grasp_group(depth=0.02)
        if not len(gg):
            return None
        return dict(score=gg.scores, width=gg.widths, height=gg.heights, depth=gg.depths, R=gg.rotations,
                    t=gg.translations)

    def _scene_points(self, rgb, depth, crop):
        torch = self.torch
        u, v = torch.meshgrid(torch.arange(1280, device='cuda'), torch.arange(720, device='cuda'), indexing='ij')
        z = depth[0] / 1e3
        mask = (z > 0) & crop
        pts = torch.stack([(u - self.K[0, 2]) / self.K[0, 0] * z, (v - self.K[1, 2]) / self.K[1, 1] * z, z], -1)[mask]
        cols = rgb[0][:, mask].T
        if len(pts) >= self.num_point:
            idx = torch.randperm(len(pts), device='cuda')[:self.num_point]
            pts, cols = pts[idx], cols[idx]
        return torch.cat([pts, cols], 1)[None]

    def _xyz_maps(self, depth):
        torch, f = self.torch, 1280 / 80
        d = self.F.interpolate(depth[:, None], size=(80, 45), mode='nearest').squeeze(1) / 1e3
        u, v = torch.meshgrid(torch.arange(80, device='cuda'), torch.arange(45, device='cuda'), indexing='ij')
        return torch.stack([(u - self.K[0, 2] / f) / (self.K[0, 0] / f) * d,
                            (v - self.K[1, 2] / f) / (self.K[1, 1] / f) * d, d], -1).permute(0, 3, 1, 2)


class GraspPoseNode(Node):

    def __init__(self):
        super().__init__('grasp_pose')
        par = lambda n, v: self.declare_parameter(n, v).value
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
        self.propagate_s = float(par('propagate_s', 1.0))  # republished (moved) up to this after the HGGD frame
        self.gripper = bool(par('gripper_camera', True))           # D405 on the flange
        self.palms = deque(maxlen=60)  # (stamp, palm position) of HandState, for the moved grasp
        self.source = None             # (GraspPose, palm position at its stamp) of the last HGGD grasp
        self.view = None               # D405 view of the last HGGD grasp: grey, corners, outline, centre, stamp
        self.shift = None              # (stamp, object shift since that grasp as the D405 sees it)
        self.track_ms = deque(maxlen=100)
        self.period = 1.0 / float(par('max_rate_hz', 30.0))
        self.yield_s = float(par('tracker_yield_s', 2.0))  # HGGD pause after a frame the tracker skipped
        self.cam_stamps, self.tracked = deque(maxlen=90), deque(maxlen=90)  # main camera infos / tracker frames
        self.skipped_t = -np.inf  # newest main camera frame the tracker skipped

        poses = {}
        for name in ('camera_extrinsics', 'd405_extrinsics'):
            with open(os.path.join(get_package_share_directory('franka_experiments'), 'config', name + '.yaml'),
                      'r', encoding='utf-8') as file:
                e = yaml.safe_load(file)
            poses[name] = (e['parent_frame'], Rotation.from_quat([e['rotation'][k] for k in 'xyzw']).as_matrix(),
                           np.array([e['translation'][k] for k in 'xyz']))
        self.base_frame = poses['camera_extrinsics'][0]
        self.link, *self.d405_on_link = poses['d405_extrinsics']          # D405 -> fr3_link8

        self.bridge, self.K = CvBridge(), None
        self.frames = deque(maxlen=20)  # D405 (colour, depth), last 0.6 s
        self.detection = None    # stamp of the last Hands23 detection run
        self.obj = self.ee = None
        self.hands, self.normals = deque(maxlen=30), deque(maxlen=30)  # last 1 s: landmarks, (stamp, palm normal)
        self.last = None  # (tip, approach) of the published grasp, base frame
        self.recent = deque(maxlen=10)  # (tip, approach) of the last frames that gave a grasp
        self.lock = threading.Lock()

        self.pub = self.create_publisher(GraspPose, '/handover/grasp_pose', 10)
        self.create_subscription(CameraInfo, '/camera/camera/aligned_depth_to_color/camera_info',
                                 lambda m: self.cam_stamps.append(stamp(m)), qos_profile_sensor_data)
        if self.gripper:  # the hand skeleton on the D405 frame itself (MediaPipe)
            self.sync = ApproximateTimeSynchronizer(
                [Subscriber(self, Image, '/d405/d405' + topic, qos_profile=qos_profile_sensor_data)
                 for topic in ('/color/image_raw', '/aligned_depth_to_color/image_raw')], queue_size=2, slop=0.02)
            self.sync.registerCallback(self.on_frame)
            self.create_subscription(CameraInfo, '/d405/d405/aligned_depth_to_color/camera_info',
                                     self.on_info, qos_profile_sensor_data)
            self.hands_d405 = mp.solutions.hands.Hands(static_image_mode=True, max_num_hands=2, model_complexity=1)
            self.tf = Buffer()
            self.tf_listener = TransformListener(self.tf, self)
        self.create_subscription(HandObjectState, '/handover/hand_object', lambda m: self._set('obj', m), 10)
        self.create_subscription(HandTrackingFiltered, '/handover/hand_tracking_filtered',
                                 self.hands.append, 10)
        self.create_subscription(HandoverDistance, '/handover/distance', lambda m: self._set('ee', m), 10)
        self.create_subscription(HandState, '/handover/hand_state', self.on_palm, 10)
        self.create_subscription(HandTrackingRaw, '/handover/hand_tracking_raw',
                                 lambda m: self.tracked.append(stamp(m)), 30)

        self.stats = dict.fromkeys(
            ('cycles', 'no object', 'no d405 object', 'd405', 'no grasp', 'filtered', 'near hand', 'palm side',
             'collision', 'off object', 'published', 'newest frame', 'moved', 'moved d405', 'yield', 'skeleton'), 0)
        self.ages = []  # (frame age when HGGD starts, when the grasp is published, of the published pose) [s]
        self.create_timer(5.0, self.log_stats)
        self.stop = threading.Event()
        try:
            self.backend = Hggd(par('hggd_root', '/ros2_ws/hggd'), int(par('hggd_num_point', 8000)), self.crop_m)
            self.get_logger().info('grasp backend: HGGD (TensorRT), D405' if self.gripper else
                                   'grasp backend: HGGD (TensorRT), no D405: idle')
            self.job, self.job_ready = None, threading.Event()  # GPU thread -> CPU thread, latest frame only
            threading.Thread(target=self.worker, daemon=True).start()
            threading.Thread(target=self.selector, daemon=True).start()
        except Exception as error:
            self.backend = None
            self.get_logger().error(f'HGGD not available ({error}): idle')

    def log_stats(self):
        if self.backend is not None and self.stats['cycles']:
            a = 1e3 * np.median(self.ages, axis=0) if self.ages else None
            age = (f', frame age at HGGD / at publish {a[0]:.0f} / {a[1]:.0f} ms, published pose {a[2]:.0f} ms'
                   if self.ages else '')
            age += f', D405 tracking {np.median(self.track_ms):.1f} ms' if self.track_ms else ''
            self.get_logger().info('last 5 s: ' + ', '.join(f'{k} {v}' for k, v in self.stats.items()) + age)
        self.stats, self.ages = dict.fromkeys(self.stats, 0), []

    def _set(self, key, msg):
        with self.lock:
            setattr(self, key, msg)

    def on_frame(self, rgb, depth):
        with self.lock:
            self.frames.append((rgb, depth))
        try:
            self.track(rgb, depth)
        except Exception as error:
            self.get_logger().warn(f'grasp tracking failed: {error}', throttle_duration_sec=5.0)

    def track(self, rgb_msg, depth_msg):
        """Shift of the object since the last HGGD frame as the D405 sees it: median Lucas-Kanade flow of corners
        inside the outline, depth of the moved outline in the new frame, D405 pose of each frame (the arm may move).
        17/10, D405 bag, 2722 pairs of detections 0.15-1.0 s apart, error on the object centre of the later one:
        moved with the main camera palm 11 / 58 mm (median / p90), not moved 6 / 31 mm, D405 flow 1 / 8 mm."""
        view, t = self.view, stamp(rgb_msg)
        if (view is None or not 0.0 < t - view['t'] <= self.propagate_s or self.K is None
                or (self.shift is not None and t - self.shift[0] < 0.06)):  # every other frame: 15 Hz is enough
            return
        t0 = time.perf_counter()
        pose = self.d405_pose(rgb_msg.header.stamp)
        if pose is None:
            return
        grey = cv2.cvtColor(self.bridge.imgmsg_to_cv2(rgb_msg, 'rgb8'), cv2.COLOR_RGB2GRAY)
        moved = flow_shift(view['grey'], grey, view['corners'])
        if moved is None:
            return
        depth = self.bridge.imgmsg_to_cv2(depth_msg)  # outline_centre reads only the outline pixels
        c = outline_centre(view['mask'], depth, *pose, self.K, moved, 1e-3 if depth_msg.encoding == '16UC1' else 1.0)
        if c is not None:
            self.shift = (t, c - view['centre'])
        self.track_ms.append(1e3 * (time.perf_counter() - t0))

    def tracker_behind(self):
        """True within yield_s of a main camera frame older than the tracker's newest output but missing from it."""
        with self.lock:
            cams, done = list(self.cam_stamps), np.array(self.tracked)
        if not len(done) or not cams:
            return False
        for t in cams:
            if t < done[-1] and np.abs(done - t).min() > 1e-3:
                self.skipped_t = max(self.skipped_t, t)
        return cams[-1] - self.skipped_t < self.yield_s

    def on_palm(self, msg):
        t = msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec
        n = np.array([msg.palm_normal.x, msg.palm_normal.y, msg.palm_normal.z])
        if msg.geometry_ok and np.linalg.norm(n) > 0.5:
            self.normals.append((t, n / np.linalg.norm(n)))
        if not msg.position_valid:
            return
        palm = np.array([msg.palm_position.x, msg.palm_position.y, msg.palm_position.z])
        self.palms.append((t, palm))
        source = self.source
        if source is None or (source[1] is None and self.shift is None):
            return
        src, palm0 = source
        if not 0.0 < t - (src.header.stamp.sec + 1e-9 * src.header.stamp.nanosec) <= self.propagate_s:
            return
        out = GraspPose(source_stamp=src.source_stamp, width=src.width, score=src.score, confidence=src.confidence,
                        gripper_camera=True)
        out.header.frame_id, out.header.stamp = src.header.frame_id, msg.header.stamp
        out.pose.orientation = src.pose.orientation
        shift = self.shift  # the object as the D405 sees it (track), else the main camera palm
        d405 = shift is not None and abs(shift[0] - t) <= 0.1
        if not d405 and palm0 is None:
            return
        move = shift[1] if d405 else palm - palm0
        self.stats['moved d405'] += d405
        p = np.array([src.pose.position.x, src.pose.position.y, src.pose.position.z]) + move
        out.pose.position.x, out.pose.position.y, out.pose.position.z = map(float, p)
        out.valid_until.sec, out.valid_until.nanosec = divmod(int((t + self.validity_s) * 1e9), 1_000_000_000)
        self.pub.publish(out)
        self.stats['moved'] += 1

    def on_info(self, msg):
        if msg.k[0] > 0.0:
            self.K = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])

    def d405_pose(self, stamp):
        """D405 -> base at stamp (TF base -> fr3_link8, d405_extrinsics.yaml); None without TF."""
        try:
            tf = self.tf.lookup_transform(self.base_frame, self.link, Time.from_msg(stamp)).transform
        except TransformException:
            return None
        q, p = tf.rotation, tf.translation
        R_link = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        R, t = self.d405_on_link
        return R_link @ R, R_link @ t + np.array([p.x, p.y, p.z])

    # ---------------------------------------------------------------- worker
    def worker(self):
        while rclpy.ok() and not self.stop.is_set():
            t0 = time.monotonic()
            with self.lock:
                obj, ee = self.obj, self.ee

            self.stats['cycles'] += 1
            present = obj is not None and obj.object_present
            own = present and obj.gripper_camera and len(obj.detection_contour_px) >= 6
            self.stats['no object'] += not present
            self.stats['no d405 object'] += present and not own
            behind = own and self.tracker_behind()  # the tracker first (yield)
            self.stats['yield'] += behind
            if own and not behind and self.K is not None:
                try:
                    job = self.frame(obj)
                    if job is not None:
                        self.step(*job, obj, ee)
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

    def frame(self, obj):
        """(colour, depth, outline) for HGGD, for a Hands23 detection not run yet: the newest D405 frame with the
        detection outline moved there by the optical flow (the detection frame is ~0.27 s old by then: Hands23 +
        outline), else the detection frame with its outline."""
        det = obj.detection_stamp.sec + 1e-9 * obj.detection_stamp.nanosec
        if det == self.detection:
            return None
        with self.lock:
            pair = next((f for f in reversed(self.frames) if abs(stamp(f[0]) - det) < 1e-3), None)
            newest = self.frames[-1] if self.frames else None
        if pair is None:
            return None
        self.detection = det
        contour = np.array(obj.detection_contour_px, np.int32).reshape(-1, 2)
        if newest is None or stamp(newest[0]) - det < 0.02:
            return (*pair, contour)
        grey0, grey1 = (cv2.cvtColor(self.bridge.imgmsg_to_cv2(f[0], 'rgb8'), cv2.COLOR_RGB2GRAY) for f in (pair, newest))
        mask = np.zeros(grey0.shape, np.uint8)
        cv2.fillPoly(mask, [contour], 1)
        corners = outline_corners(grey0, mask)
        moved = None if corners is None else flow_shift(grey0, grey1, corners)
        if moved is None:
            return (*pair, contour)
        self.stats['newest frame'] += 1
        return (*newest, np.round(contour + moved).astype(np.int32))

    def step(self, rgb_msg, depth_msg, contour, obj, ee):
        """HGGD on a D405 frame with its object outline; hand landmarks of that frame. Nothing without TF."""
        t = stamp(depth_msg)
        hand = min(list(self.hands), key=lambda m: abs(stamp(m) - t), default=None)
        if abs(stamp(obj) - t) > 0.5:
            return
        pose = self.d405_pose(depth_msg.header.stamp)
        if pose is None:
            return
        R, t_cam = pose  # D405 -> base

        depth = self.bridge.imgmsg_to_cv2(depth_msg).astype(np.float32)
        if depth_msg.encoding == '16UC1':
            depth *= 1e-3

        # crop around the object (+-crop_m) and back-project it; centre of the outline in its frame (the published
        # centroid follows the newest palm)
        fx, fy, cx, cy = self.K
        h, w = depth.shape
        mask = np.zeros((h, w), np.uint8)
        cv2.fillPoly(mask, [contour], 1)
        vv, uu = np.nonzero(mask & (depth > 0.07))  # 0.07 m: D405 minimum range
        if len(uu) < 30:
            return
        z_c = float(np.median(depth[vv, uu]))
        c_cam = np.array([(uu.mean() - cx) * z_c / fx, (vv.mean() - cy) * z_c / fy, z_c])
        centre = R @ c_cam + t_cam
        r = int(self.crop_m * fx / c_cam[2])
        u_c, v_c = int(fx * c_cam[0] / c_cam[2] + cx), int(fy * c_cam[1] / c_cam[2] + cy)
        u0, u1, v0, v1 = max(0, u_c - r), min(w, u_c + r), max(0, v_c - r), min(h, v_c + r)
        if u1 - u0 < 8 or v1 - v0 < 8:
            return

        v, u = np.mgrid[v0:v1, u0:u1]
        z = depth[v0:v1, u0:u1]
        keep = (z > 0.07) & (np.abs(z - c_cam[2]) < self.crop_m)
        u, v, z = u[keep], v[keep], z[keep]
        points = np.column_stack([(u - cx) * z / fx, (v - cy) * z / fy, z]).astype(np.float32)
        region = mask[v, u].astype(bool)
        if region.sum() < 30:
            return
        image_region = np.zeros((h, w), bool)
        image_region[v[region], u[region]] = True
        self.stats['d405'] += 1

        rgb = self.bridge.imgmsg_to_cv2(rgb_msg, 'rgb8')
        t_start = self.get_clock().now().nanoseconds * 1e-9 - t
        g = self.backend(rgb, depth, self.K, image_region)
        if g is None:
            self.stats['no grasp'] += 1
            return
        # on the object, from the robot side, within the gripper opening (collisions: CPU thread)
        source = (np.array([ee.ee_control_point.x, ee.ee_control_point.y, ee.ee_control_point.z])
                  if ee is not None and ee.valid else np.array([0.0, 0.0, 0.5]))
        approach = R.T @ (centre - source)
        approach /= max(np.linalg.norm(approach), 1e-9)
        near = cKDTree(points[region]).query(g['t'], distance_upper_bound=0.02)[0] < 0.02
        ok = (near & (g['R'][:, :, 0] @ approach > np.cos(self.approach_thresh))
              & (g['width'] <= self.max_width))
        g = {k: val[ok] for k, val in g.items()}
        if not len(g['score']):
            self.stats['filtered'] += 1
            return

        # hand skeleton of that frame (21 points, >= 8 with depth) on the D405 frame itself (the main camera
        # calibration differs by ~5 cm on the same hand, 17/10), else the hand surface: cloud around the palm
        # outside the object region (fingers in any pose)
        skeleton = self.d405_skeleton(rgb, depth, self.K, R, t_cam, image_region)
        self.stats['skeleton'] += skeleton is not None
        hand_surface = np.zeros((0, 3))
        if hand is not None and skeleton is None:
            mcp = [[p.x, p.y, p.z] for i, (p, s) in enumerate(zip(hand.positions, hand.landmark_state))
                   if i and s in TRACKED]
            if mcp:
                palm = R.T @ (np.mean(mcp, 0) - t_cam)
                near = ~region & (np.linalg.norm(points - palm, axis=1) < self.hand_radius)
                hand_surface = points[near][::max(1, int(near.sum()) // 400)] @ R.T + t_cam
        self.job = (depth_msg, pose, g, hand, hand_surface, points, region, skeleton, t_start,
                    (cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY), image_region, depth))
        self.job_ready.set()

    def d405_skeleton(self, rgb, depth, K, R, t_cam, region):
        """Skeleton of the hand nearest the object on a D405 frame: MediaPipe, 5x5 median depth, a depth > 10 cm
        from the hand median takes the median (as the tracker), D405 pose of that frame."""
        found = self.hands_d405.process(rgb).multi_hand_landmarks
        if not found:
            return None
        h, w = depth.shape
        fx, fy, cx, cy = K
        obj = np.argwhere(region)[::max(1, int(region.sum()) // 200), ::-1]  # object pixels (u, v)
        uv = min((np.array([[p.x * w, p.y * h] for p in lm.landmark]) for lm in found),
                 key=lambda a: np.linalg.norm(a[:, None] - obj[None], axis=2).min())
        z = np.full(21, np.nan)
        for i, (u, v) in enumerate(np.round(uv).astype(int)):
            if 0 <= u < w and 0 <= v < h:
                patch = depth[max(0, v - 2):v + 3, max(0, u - 2):u + 3]
                patch = patch[patch > 0.07]
                z[i] = np.median(patch) if patch.size else np.nan
        if not np.isfinite(z).any():
            return None
        med = np.nanmedian(z)
        z = np.where(np.abs(z - med) > 0.10, med, z)
        cam = np.column_stack([(uv[:, 0] - cx) * z / fx, (uv[:, 1] - cy) * z / fy, z])
        return skeleton_of(cam @ R.T + t_cam)

    def _collision_free(self, points, g, ok, tips, axes):
        """Free grasps among ok, checked in score order 64 at a time down to the first free one, plus the
        candidates that may keep the previous grasp: the same choice as checking them all."""
        scores, free, checked = g['score'], np.zeros(len(ok), bool), np.zeros(len(ok), bool)
        points = voxels(points)  # the 1 cm voxels of collisions(), once per choice

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

    def publish(self, depth_msg, pose, g, hand, hand_surface, points, region, skeleton, t_start, frame):
        R, t_cam = pose
        rot = g['R']  # graspnet: x approach, y closing
        tips = (g['t'] + g['depth'][:, None] * rot[:, :, 0]) @ R.T + t_cam  # camera -> base
        axes = np.einsum('ij,njk->nik', R, rot)
        scores = g['score']

        ok = np.ones(len(scores), bool)
        if skeleton is not None:  # whole gripper off the hand skeleton, the object between the fingers
            ok = skeleton_clear(tips, axes, g['width'], *skeleton)
            if not ok.any():
                self.stats['near hand'] += 1
                return
            obj = points[region] @ R.T + t_cam
            obj = obj[seg_dist(obj, *skeleton).min(1) >= BONE_R + 0.005]  # region pixels on the fingers out
            obj = obj[::max(1, len(obj) // 1500)]
            idx = np.where(ok)[0]
            ok[idx] = len(obj) > 0 and between_fingers(obj, g['t'][idx] @ R.T + t_cam, axes[idx], g['depth'][idx],
                                                       g['width'][idx])
            if not ok.any():
                self.stats['off object'] += 1
                return
        else:  # finger pads away from the landmarks and from the whole hand surface
            lm = np.array([[p.x, p.y, p.z] for p, s in zip(hand.positions, hand.landmark_state) if s in TRACKED]
                          if hand is not None else np.zeros((0, 3))).reshape(-1, 3)
            for side in (-1.0, 1.0):
                fingers = tips + side * 0.5 * g['width'][:, None] * axes[:, :, 1]
                for pts, clearance in ((lm, self.clearance), (hand_surface, self.finger_clearance)):
                    if len(pts):
                        ok &= np.linalg.norm(fingers[:, None] - pts[None], axis=2).min(1) >= clearance
            if not ok.any():
                self.stats['near hand'] += 1
                return
        t = stamp(depth_msg)
        palm = min(list(self.normals), key=lambda n: abs(n[0] - t), default=None)  # palm normal of that frame
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

        out = GraspPose(width=float(g['width'][best]), score=float(scores[best]), gripper_camera=True)
        out.header.stamp, out.header.frame_id = depth_msg.header.stamp, self.base_frame
        out.source_stamp = depth_msg.header.stamp
        out.confidence = float(np.clip(scores[best], 0.0, 1.0) * (1 + sum(same)) / (1 + len(same)))
        out.valid_until.sec, out.valid_until.nanosec = divmod(int((t + self.validity_s) * 1e9), 1_000_000_000)
        out.pose.position.x, out.pose.position.y, out.pose.position.z = map(float, tips[best])
        qx, qy, qz, qw = Rotation.from_matrix(R_tcp).as_quat()
        out.pose.orientation.x, out.pose.orientation.y = float(qx), float(qy)
        out.pose.orientation.z, out.pose.orientation.w = float(qz), float(qw)

        # D405 view of this frame for track(); the grasp goes out moved to the newest D405 frame already (HGGD and
        # the checks took ~0.2 s), as the republished ones
        near = [(abs(tp - t), p) for tp, p in list(self.palms) if abs(tp - t) <= 0.1]
        grey, outline, depth = frame
        corners = outline_corners(grey, outline)
        centre = outline_centre(outline, depth, *pose, self.K)
        self.shift = None
        self.view = (None if corners is None or centre is None else
                     dict(grey=grey, corners=corners, mask=outline, centre=centre, t=t))
        self.source = (out, min(near, key=lambda x: x[0])[1] if near else None)
        with self.lock:
            newest = self.frames[-1] if self.frames else None
        if newest is not None and stamp(newest[0]) > t:
            self.track(*newest)
        shift = self.shift
        sent = out
        if shift is not None:
            sent = GraspPose(width=out.width, score=out.score, confidence=out.confidence, gripper_camera=True,
                             source_stamp=out.source_stamp)
            sent.header.frame_id, sent.pose.orientation = out.header.frame_id, out.pose.orientation
            sent.header.stamp.sec, sent.header.stamp.nanosec = divmod(int(shift[0] * 1e9), 1_000_000_000)
            sent.valid_until.sec, sent.valid_until.nanosec = divmod(int((shift[0] + self.validity_s) * 1e9),
                                                                    1_000_000_000)
            sent.pose.position.x, sent.pose.position.y, sent.pose.position.z = map(float, tips[best] + shift[1])
        self.pub.publish(sent)
        now = self.get_clock().now().nanoseconds * 1e-9
        self.ages.append((t_start, now - t, now - stamp(sent)))
        self.stats['published'] += 1


# ============================================================ grasp executor
class GraspExecutor(HandoverQddotCommander):
    JUMP_M = 0.15  # palm jump between two tracker frames: another hand / a wrong detection (PREGRASP holds)
    VIEW_NEAR_M = 0.22  # hand pregrasp distance when view_m is out of reach (sim 17/10: hand at 0.70 m, 0.35 m high)
    APPROACH_HOLD_S = 3.0  # an approach seen this recently still starts a cycle (the hand stops to wait)
    SIDE_TILTS_DEG = (45.0, 60.0)  # hand pregrasp from the robot side when the near-vertical ones are out of reach
    STILL_MS = 0.15  # ... and the hand is held still

    def __init__(self):
        super().__init__()
        par = lambda n, v: self.declare_parameter(n, v).value
        self.pregrasp_m = float(par('pregrasp_m', 0.25))            # standoff along the grasp approach
        self.view_m = float(par('view_m', 0.30))                    # hand pregrasp: TCP - palm (D405 at 0.25-0.35 m)
        self.normal_s = float(par('view_normal_s', 0.3))            # hand pregrasp: palm normal low-pass
        self.view_tilt = np.radians(float(par('view_tilt_deg', 30.0)))  # approach axis from the vertical
        self.view_rate = np.radians(float(par('view_rate_deg_s', 30.0)))  # turn rate of the hand pregrasp direction
        self.blend_s = float(par('blend_s', 0.6))                   # hand estimate -> grasp pregrasp, no step
        self.look_m = float(par('look_m', 0.25))                    # looking at the object farther than this
        self.approach_step_m = float(par('approach_step_m', 0.02))  # slow: ~0.1 m/s
        self.slow_m = float(par('slow_m', 0.10))                    # approach_step_m within this of the grasp
        self.freeze_m = float(par('freeze_m', 0.05))                # grasp no longer followed
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
        self._normal, self._normal_t = None, None  # palm normal (low-passed), HandState stamp
        self._view_n, self._view_check, self._view_d = None, -np.inf, 0.30  # hand pregrasp direction (reachable, None: none), last IK
        self._view_dir, self._view_t = None, None        # hand pregrasp direction followed, rate limited
        self._view_palm, self._view_hold = None, -np.inf  # (stamp ns, palm) of the last frame; hold until (jump)
        self._grasp_since = None  # first grasp of this PREGRASP (blend start)
        self._approach_t = -np.inf  # last observer APPROACHING
        self._hand_speed = np.inf  # palm speed (HandState), inf without a valid velocity
        self._centroid, self._centroid_t = None, -np.inf  # object centroid (HandObjectState), its time

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

    def _hand_cb(self, msg):
        super()._hand_cb(msg)
        v = msg.palm_velocity
        self._hand_speed = float(np.linalg.norm([v.x, v.y, v.z])) if msg.velocity_valid else np.inf
        n = np.array([msg.palm_normal.x, msg.palm_normal.y, msg.palm_normal.z])
        t = msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec
        if not (msg.geometry_ok and np.linalg.norm(n) > 0.5):
            return
        n /= np.linalg.norm(n)
        if self._normal is None or not 0.0 < t - self._normal_t <= 0.5:
            self._normal = n
        else:  # one-pole low-pass: the palm normal of one frame is noisy (~10 deg)
            a = 1.0 - np.exp(-(t - self._normal_t) / self.normal_s)
            self._normal = (1.0 - a) * self._normal + a * n
            self._normal /= max(np.linalg.norm(self._normal), 1e-9)
        self._normal_t = t

    def _object_cb(self, msg):
        self._object = bool(msg.valid and msg.object_present)
        c = np.array([msg.object_centroid_3d.x, msg.object_centroid_3d.y, msg.object_centroid_3d.z])
        if self._object and np.isfinite(c).all() and np.linalg.norm(c) > 1e-6:
            self._centroid, self._centroid_t = c, self._now()
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

    def approaching(self):
        return self.fresh(self.observer) and int(self.observer.state) == int(HandoverObserver.APPROACHING)

    def _view_pose(self, now):
        """Pregrasp from the hand alone (TCP position, orientation), None while not reachable: view_m (VIEW_NEAR_M if
        out of reach) from the palm along the palm
        normal n (low-passed), z of the TCP = -n, x = the present x on the plane (least turn about the approach).
        n kept on the robot side of the hand (the path does not cross the hand) and within view_tilt_deg of the
        vertical: sim 17/10, a side palm asked 60-90 deg turns, joint 4 at its limit and the controller's HARD
        reset: 30 deg, and the followed direction turns at most view_rate_deg_s (a palm normal jumping 20-40 deg moved
        the target 10-20 cm at once). IK every 0.1 s on n, then halfway to the vertical, then the vertical; the target
        checked against the reach at every cycle. A palm jumping > JUMP_M between two tracker frames (another hand, a
        wrong detection; a hand at 1 m/s moves ~3 cm per frame) holds the arm for 0.5 s: sim 17/10 rosbag2, chasing a palm that jumped 0.5-0.9 m stretched the elbow to its
        limit (q4 -0.12) and the controller lost the reference."""
        palm, up = self._hand_position.copy(), np.array([0.0, 0.0, 1.0])
        if self._view_palm is None or self._view_palm[0] != self._hand_stamp_ns:  # a new tracker frame
            if (self._view_palm is not None and (self._hand_stamp_ns - self._view_palm[0]) * 1e-9 <= 0.1
                    and np.linalg.norm(palm - self._view_palm[1]) > self.JUMP_M):
                self._view_hold = now + 0.5
            self._view_palm = (self._hand_stamp_ns, palm)
        if now < self._view_hold:
            return None
        n = up if self._normal is None or now - self._normal_t > 0.5 else self._normal.copy()
        side = self._p_ee - palm
        side[2] = 0.0
        side /= max(np.linalg.norm(side), 1e-9)
        if n @ side < 0.0:  # palm facing away from the robot: seen from its edge, not from behind the hand
            n -= (n @ side) * side
        h = n - (n @ up) * up  # horizontal part
        if np.linalg.norm(h) < 1e-3:
            h = side
        if np.arccos(np.clip(n @ up / max(np.linalg.norm(n), 1e-9), -1.0, 1.0)) > self.view_tilt:
            n = np.cos(self.view_tilt) * up + np.sin(self.view_tilt) * h / np.linalg.norm(h)
        n /= np.linalg.norm(n)

        def rotation(n):
            z = -n
            x = self._R_des[:, 0] - (self._R_des[:, 0] @ z) * z
            if np.linalg.norm(x) < 1e-3:
                x = self._R_des[:, 1] - (self._R_des[:, 1] @ z) * z
            x /= np.linalg.norm(x)
            return np.column_stack([x, np.cross(z, x), z])

        if now - self._view_check >= 0.1:
            self._view_check, self._view_n = now, None
            # from the robot side (tilted SIDE_TILTS_DEG from the vertical) only when the near-vertical ones are out of
            # reach: a hand 0.7-0.8 m away is reachable, 30 cm above it is not (sim 17/10 night)
            # only for a hand held still (< STILL_MS): coming towards the robot it would run into that pregrasp
            # (sim 17/10 night: started at once, 0/8 closed with the hand brought onto the gripper)
            base = np.r_[-palm[:2], 0.0] / max(np.linalg.norm(palm[:2]), 1e-9)  # palm -> robot base, horizontal
            tilted = ([np.cos(a) * up + np.sin(a) * base for a in np.radians(self.SIDE_TILTS_DEG)]
                      if self._hand_speed < self.STILL_MS else [])
            for m in (self.view_m, self.VIEW_NEAR_M):
                for c in (n, (n + up) / np.linalg.norm(n + up), up, *tilted):
                    p = palm + m * c
                    if (p[2] >= self.min_z_m and np.linalg.norm(p[:2]) <= self.max_reach_m
                            and not self.ik_check(p, rotation(c))):
                        self._view_n, self._view_d = c, m
                        break
                if self._view_n is not None:
                    break
        if self._view_n is None:
            return None
        p = palm + self._view_d * self._view_n
        if p[2] < self.min_z_m or np.linalg.norm(p[:2]) > self.max_reach_m:  # the palm moved since the IK check
            return None
        if self._view_dir is None or now - self._view_t > 0.5:
            self._view_dir = self._view_n
        else:  # rate-limited turn towards the reachable direction
            angle = np.arccos(np.clip(self._view_dir @ self._view_n, -1.0, 1.0))
            f = 1.0 if angle < 1e-6 else min(1.0, self.view_rate * (now - self._view_t) / angle)
            d = (1.0 - f) * self._view_dir + f * self._view_n
            self._view_dir = d / np.linalg.norm(d)
        self._view_t = now
        return palm + self._view_d * self._view_dir, rotation(self._view_dir)

    @staticmethod
    def _slerp(R0, R1, w):
        return Slerp([0.0, 1.0], Rotation.from_matrix([R0, R1]))(w).as_matrix()

    def _looking(self, R_end, target):
        """Orientation on the way to the pregrasp: farther than look_m from it the approach axis (and the D405, which
        looks along it) points at the object (its centroid, else the palm), within view_tilt_deg of the vertical;
        nearer it turns into R_end (at the pregrasp the object is on the approach axis anyway). 17/10 night, sim: 55-80
        deg or turning to the object already in HOME did not show it sooner (the hand comes at the D405's height) and
        lost cycles."""
        look = self._centroid if self._centroid is not None and self._now() - self._centroid_t < 0.5 else self._hand_position
        z = look - self._p_ee
        if np.linalg.norm(z) < 1e-3:
            return R_end
        z /= np.linalg.norm(z)
        down = np.array([0.0, 0.0, -1.0])
        if np.arccos(np.clip(z @ down, -1.0, 1.0)) > self.view_tilt:
            h = z - (z @ down) * down
            z = np.cos(self.view_tilt) * down + np.sin(self.view_tilt) * h / max(np.linalg.norm(h), 1e-9)
        x = R_end[:, 0] - (R_end[:, 0] @ z) * z
        if np.linalg.norm(x) < 1e-3:
            x = R_end[:, 1] - (R_end[:, 1] @ z) * z
        x /= np.linalg.norm(x)
        R_look = np.column_stack([x, np.cross(z, x), z])
        w = float(np.clip((np.linalg.norm(target - self._p_ee) - 0.05) / max(self.look_m - 0.05, 1e-3), 0.0, 1.0))
        return self._slerp(R_end, R_look, w)

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

    def _approach_step(self, grasp):
        """Target step around the grasp: approach_step_m within slow_m of it, growing linearly farther out."""
        return min(self.max_step(), self.approach_step_m * max(1.0, np.linalg.norm(grasp - self._p_ee) / self.slow_m))

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
        elif (self._phase == 'APPROACH' and self.hand_trusted()
              and self._fresh_grasp(max_age=self.grasp_timeout_s) is not None):
            # the frozen grasp passed the IK when APPROACH started; a new one 1-3 cm away failing it by a little
            # (manipulability 0.0085 < 0.01, sim 17/10 night: 4 cycles of 8 back home) is still the object there
            self._good_t = now

        if self._phase == 'HOME':
            if self._R_home is not None:
                self._turn_towards(self._R_home, dt)
            if self.approaching():
                self._approach_t = now
            ready = {'armed (empty hand first)': self._armed, 'object in hand': self._object,
                     'approach': now - self._approach_t <= self.APPROACH_HOLD_S, 'hand trusted': self.hand_trusted()}
            ready['view pose reachable'] = all(ready.values()) and self._view_pose(now) is not None
            if not all(ready.values()):
                missing = ', '.join(k for k, ok in ready.items() if not ok)
                self.get_logger().info(f'HOME, waiting for: {missing}', throttle_duration_sec=2.0)
            if all(ready.values()) and now >= self._retry_at:
                self._armed, self._good_t, self._grasp_since = False, now, None
                self.gripper.command(False)
                self._set_phase('PREGRASP')
            return self._step_to(home, self.max_step())

        if self._phase == 'PREGRASP':  # one motion to the pregrasp, the D405 on the object all the way
            if self.hand_trusted() and self._object:
                self._good_t = now
            if now - self._good_t > self.lost_s:
                self._armed, self._retry_at = True, now + self.retry_s  # nothing was grasped
                self._set_phase('RETURN', 'hand / object lost: back home, gripper stays open, retry')
                return self._p_ee.copy()
            if self.get_parameter('pause_on_retreat').value and self.retreating():  # the hand is taken back
                self._retreat_t = self._retreat_t or now
                if now - self._retreat_t > self.get_parameter('retreat_abort_s').value:
                    self._armed, self._retry_at, self._retreat_t = True, now + self.retry_s, None
                    self._set_phase('RETURN', 'hand withdrawn: back home, retry')
                self._phase_t += dt  # the pause does not count towards phase_timeout_s
                return self._p_ee.copy()
            self._retreat_t = None
            fresh = grasp is not None
            grasp = grasp or self._fresh_grasp(max_age=self.lost_s)  # short gaps ride on the last grasp
            view = self._view_pose(now) if self.hand_trusted() else None
            if grasp is None and view is None:
                return self._p_ee.copy()  # no grasp yet and the hand out of reach: hold
            if grasp is not None:
                self._grasp_since = self._grasp_since or now
                p, R, _ = grasp
                R = self._closest_flip(R)
                target_g = p - self.pregrasp_m * R[:, 2]
            w = 0.0 if grasp is None else (1.0 if view is None else min(1.0, (now - self._grasp_since) / self.blend_s))
            if w < 1.0:  # hand-only estimate of the pregrasp, blended into the grasp's once it comes
                target = view[0] if w == 0.0 else (1.0 - w) * view[0] + w * target_g
                R_end = view[1] if w == 0.0 else self._slerp(view[1], R, w)
            else:
                target, R_end = target_g, R
            angle = self._turn_towards(self._looking(R_end, target), dt)
            if (fresh and w >= 1.0 and np.linalg.norm(target - self._p_ee) < self.reach_tol_m * 2
                    and angle < self.rot_tol):  # final approach only on a fresh grasp
                self._frozen = (p.copy(), R.copy())
                self._set_phase('APPROACH')
            elif elapsed > self.phase_timeout_s:
                self._armed, self._retry_at = True, now + self.retry_s
                self._set_phase('RETURN', f'no D405 grasp / pregrasp not reached in {self.phase_timeout_s:.0f} s')
            return self._step_to(target, self.max_step())

        if self._phase == 'APPROACH':
            # short gaps (grasp at ~10 Hz, tracking dropouts) ride on the last grasp
            tolerance = self.approach_lost_s
            if self._good_t is None or now - self._good_t > tolerance:
                self._armed, self._retry_at = True, now + self.retry_s  # nothing was grasped
                self._set_phase('RETURN', 'hand / grasp lost: back home, gripper stays open, retry')
                return self._p_ee.copy()
            grasp = grasp or self._fresh_grasp(max_age=tolerance)
            if self.get_parameter('pause_on_retreat').value and self.retreating():  # the hand is taken back
                self._retreat_t = self._retreat_t or now
                if now - self._retreat_t > self.get_parameter('retreat_abort_s').value:
                    self._armed, self._retry_at, self._retreat_t = True, now + self.retry_s, None
                    self._set_phase('RETURN', 'hand withdrawn: back home, retry')
                self._phase_t += dt  # the pause does not count towards phase_timeout_s
                return self._p_ee.copy()
            self._retreat_t = None

        if self._phase == 'APPROACH':
            p, R = self._frozen
            if grasp is not None and np.linalg.norm(p - self._p_ee) > self.freeze_m:
                q, Q = grasp[0], self._closest_flip(grasp[1])
                if np.linalg.norm(q - p) < 0.03 and Q[:, 2] @ R[:, 2] > np.cos(np.radians(30.0)):
                    self._frozen = p, R = q.copy(), Q.copy()  # the same grasp, moved with the hand
            self._turn_towards(R, dt)
            if np.linalg.norm(p - self._p_ee) < self.reach_tol_m:
                self.gripper.command(True)
                self._set_phase('CLOSE')
            elif elapsed > self.phase_timeout_s:
                self._set_phase('RETURN', f'grasp not reached in {self.phase_timeout_s:.0f} s')
            return self._step_to(p, self._approach_step(p))

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
            return self._step_to(back, self._approach_step(p))

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
