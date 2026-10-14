#!/usr/bin/env python3
"""Handover grasp: the grasp pose on the object in the hand and the robot cycle that takes it.

  python3 handover_grasp.py pose ...      grasp pose node -> /handover/grasp_pose
  python3 handover_grasp.py executor ...  grasp cycle on the handover commander (robot_handover grasp_executor:=true)

GRASP POSE
  cameras    main camera (camera_extrinsics.yaml) and gripper D405 (TF base -> fr3_link8 + d405_extrinsics.yaml,
             gripper_camera:=true). HGGD runs on one frame per cycle, the D405 one first when the object is in its
             view: GraspNet scale at 0.1-0.5 m, sharper depth, no main camera calibration error. Main camera grasps
             wait while a D405 grasp is younger than propagate_s: no switching between the two calibrations
  region     the Hands23 object mask of the camera Hands23 runs on (HandObjectState.gripper_camera: the D405
             while the palm is in its image, else the main camera), hand pixels kept (the Hands23 hand mask often
             holds the object; the finger filters keep the hand safe). D405 without its mask: the depth points
             within half the object extent + 4 cm of the centroid, only with all of it in the image. 14/10 lab
             bag, ball: grasp centre on the ball 94 % with the D405 mask, 36 % with the sphere
  backend    HGGD (Chen et al., RA-L 2023, MIT) with TensorRT FP16 nets, hggd_num_point scene points within crop_m
             of the object: 72 / 88 ms (p50 / p90) under the full pipeline on the laptop (RTX 3050), 20 ms with a
             free GPU; the tracker stays at 30 Hz
  threads    GPU thread (cloud + HGGD) and CPU thread (filters, collisions, choice) on consecutive frames
  filter     on the object, approach from the robot TCP within approach_thresh_deg, opening
             <= max_gripper_width_m, fingertips >= hand_clearance_m from the hand, then collision-free,
             checked in score order 64 at a time
  palm       approach within palm_approach_deg of -palm normal (HandState, palm UP from above, SIDE from the
             palm side, DOWN from below): never through the back of the hand. 180 = off
  choice     best score; the previous grasp is kept while close and >= keep_ratio of the best
  rate       between two HGGD grasps the last one is republished at every HandState (camera rate), moved with
             the palm translation, up to propagate_s old. source_stamp = the HGGD frame
  output     PoseStamped in the base frame, Franka TCP convention (z approach, y closing)

GRASP EXECUTOR
  HOME -> PREGRASP (pregrasp_m back along the approach: a standoff where the D405 sees the object)
  -> APPROACH (follows the fresh grasps that stay within 3 cm / 30 deg, frozen in the last freeze_m; slows down
  to approach_step_m in the last slow_m) -> CLOSE -> HOLD (hold_s) -> OPEN -> RETREAT -> RETURN -> HOME
  A cycle starts with an empty hand first (rearm_s), an object in the hand, a fresh reachable grasp
  and a trusted hand. Hand / grasp lost (lost_s, approach_lost_s) or phase_timeout_s -> RETURN.
"""

import os
import site
import sys
import threading
import time

from collections import deque

import cv2
import numpy as np
import pinocchio as pin
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from franka_msgs.msg import GraspPose, HandObjectState, HandoverDistance, HandState, HandTrackingFiltered
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
        self.propagate_s = float(par('propagate_s', 0.5))
        self.gripper = bool(par('gripper_camera', True))           # D405 on the flange
        self.palms = deque(maxlen=60)  # (stamp, palm position) of HandState, for the moved grasp
        self.source = None             # (GraspPose, palm position at its stamp) of the last HGGD grasp
        self.gripper_t = -np.inf       # stamp of the last D405 grasp
        self.period = 1.0 / float(par('max_rate_hz', 30.0))

        poses = {}
        for name in ('camera_extrinsics', 'd405_extrinsics'):
            with open(os.path.join(get_package_share_directory('franka_experiments'), 'config', name + '.yaml'),
                      'r', encoding='utf-8') as file:
                e = yaml.safe_load(file)
            poses[name] = (e['parent_frame'], Rotation.from_quat([e['rotation'][k] for k in 'xyzw']).as_matrix(),
                           np.array([e['translation'][k] for k in 'xyz']))
        self.base_frame, *self.camera_pose = poses['camera_extrinsics']  # camera -> base
        self.link, *self.d405_on_link = poses['d405_extrinsics']          # D405 -> fr3_link8

        self.bridge, self.K, self.frames = CvBridge(), {}, {}  # per camera: intrinsics, newest colour + depth
        self.obj = self.hand = self.ee = self.palm = None
        self.last = None  # (tip, approach) of the published grasp, base frame
        self.recent = deque(maxlen=10)  # (tip, approach) of the last frames that gave a grasp
        self.lock = threading.Lock()

        self.pub = self.create_publisher(GraspPose, '/handover/grasp_pose', 10)
        self.syncs = []
        for cam, ns in (('camera', '/camera/camera'), ('d405', '/d405/d405'))[:1 + self.gripper]:
            sync = ApproximateTimeSynchronizer(
                [Subscriber(self, Image, ns + topic, qos_profile=qos_profile_sensor_data)
                 for topic in ('/color/image_raw', '/aligned_depth_to_color/image_raw')], queue_size=2, slop=0.02)
            sync.registerCallback(lambda rgb, depth, cam=cam: self.on_frame(cam, rgb, depth))
            self.syncs.append(sync)
            self.create_subscription(CameraInfo, ns + '/aligned_depth_to_color/camera_info',
                                     lambda m, cam=cam: self.on_info(cam, m), qos_profile_sensor_data)
        if self.gripper:
            self.tf = Buffer()
            self.tf_listener = TransformListener(self.tf, self)
        self.create_subscription(HandObjectState, '/handover/hand_object', lambda m: self._set('obj', m), 10)
        self.create_subscription(HandTrackingFiltered, '/handover/hand_tracking_filtered',
                                 lambda m: self._set('hand', m), 10)
        self.create_subscription(HandoverDistance, '/handover/distance', lambda m: self._set('ee', m), 10)
        self.create_subscription(HandState, '/handover/hand_state', self.on_palm, 10)

        self.stats = dict.fromkeys(
            ('frames', 'no object', 'd405', 'no grasp', 'filtered', 'near hand', 'palm side', 'collision',
             'published', 'published d405', 'moved'), 0)
        self.create_timer(5.0, self.log_stats)
        self.stop = threading.Event()
        try:
            self.backend = Hggd(par('hggd_root', '/ros2_ws/hggd'), int(par('hggd_num_point', 8000)), self.crop_m)
            self.get_logger().info('grasp backend: HGGD (TensorRT)' + (', D405 first' if self.gripper else ''))
            self.job, self.job_ready = None, threading.Event()  # GPU thread -> CPU thread, latest frame only
            threading.Thread(target=self.worker, daemon=True).start()
            threading.Thread(target=self.selector, daemon=True).start()
        except Exception as error:
            self.backend = None
            self.get_logger().error(f'HGGD not available ({error}): idle')

    def log_stats(self):
        if self.backend is not None and self.stats['frames']:
            self.get_logger().info('last 5 s: ' + ', '.join(f'{k} {v}' for k, v in self.stats.items()))
        self.stats = dict.fromkeys(self.stats, 0)

    def _set(self, key, msg):
        with self.lock:
            setattr(self, key, msg)

    def on_frame(self, cam, rgb, depth):
        with self.lock:
            self.frames[cam] = (rgb, depth)

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
        out = GraspPose(source_stamp=src.source_stamp, width=src.width, score=src.score, confidence=src.confidence,
                        gripper_camera=src.gripper_camera)
        out.header.frame_id, out.header.stamp = src.header.frame_id, msg.header.stamp
        out.pose.orientation = src.pose.orientation
        p = np.array([src.pose.position.x, src.pose.position.y, src.pose.position.z]) + palm - palm0
        out.pose.position.x, out.pose.position.y, out.pose.position.z = map(float, p)
        out.valid_until.sec, out.valid_until.nanosec = divmod(int((t + self.validity_s) * 1e9), 1_000_000_000)
        self.pub.publish(out)
        self.stats['moved'] += 1

    def on_info(self, cam, msg):
        if msg.k[0] > 0.0:
            self.K[cam] = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])

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
                frames, obj, hand, ee = self.frames, self.obj, self.hand, self.ee
                self.frames = {}

            present = obj is not None and obj.object_present
            if 'camera' in frames and 'camera' in self.K:
                self.stats['frames'] += 1
                self.stats['no object'] += not present
            for cam in ('d405', 'camera') if present else ():  # the D405 first; one HGGD call per cycle
                try:
                    if cam in frames and cam in self.K and self.step(cam, *frames[cam], obj, hand, ee):
                        break
                except Exception as error:
                    self.get_logger().warn(f'grasp step failed ({cam}): {error}', throttle_duration_sec=5.0)

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

    def step(self, cam, rgb_msg, depth_msg, obj, hand, ee):
        """HGGD on a frame of cam; False when nothing ran (object out of view, no TF, D405 grasp still moved)."""
        stamp = lambda m: m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec
        t = stamp(depth_msg)
        centre = np.array([obj.object_centroid_3d.x, obj.object_centroid_3d.y, obj.object_centroid_3d.z])
        contour = np.array(obj.contour_px, np.int32).reshape(-1, 2)
        own = len(contour) >= 3 and obj.gripper_camera == (cam == 'd405')  # Hands23 outline in this image
        if not np.isfinite(centre).all() or abs(stamp(obj) - t) > 0.3:
            return False
        if cam == 'camera' and (not own or t - self.gripper_t <= self.propagate_s):
            return False
        pose = self.camera_pose if cam == 'camera' else self.d405_pose(depth_msg.header.stamp)
        if pose is None:
            return False
        R, t_cam = pose  # camera -> base

        depth = self.bridge.imgmsg_to_cv2(depth_msg).astype(np.float32)
        if depth_msg.encoding == '16UC1':
            depth *= 1e-3

        # crop around the object (+-crop_m) and back-project it
        fx, fy, cx, cy = self.K[cam]
        c_cam = R.T @ (centre - t_cam)
        if c_cam[2] < 0.07:  # behind the camera or under the D405 minimum range
            return False
        r = int(self.crop_m * fx / c_cam[2])
        u_c, v_c = int(fx * c_cam[0] / c_cam[2] + cx), int(fy * c_cam[1] / c_cam[2] + cy)
        h, w = depth.shape
        radius = 0.5 * (obj.dimensions.x if np.isfinite(obj.dimensions.x) else 0.08) + 0.04
        r_obj = radius * fx / c_cam[2]
        if cam == 'd405' and not own and not (r_obj <= u_c < w - r_obj and r_obj <= v_c < h - r_obj):
            return False
        u0, u1, v0, v1 = max(0, u_c - r), min(w, u_c + r), max(0, v_c - r), min(h, v_c + r)
        if u1 - u0 < 8 or v1 - v0 < 8:
            return False

        v, u = np.mgrid[v0:v1, u0:u1]
        z = depth[v0:v1, u0:u1]
        keep = (z > 0.07) & (np.abs(z - c_cam[2]) < self.crop_m)
        u, v, z = u[keep], v[keep], z[keep]
        points = np.column_stack([(u - cx) * z / fx, (v - cy) * z / fy, z]).astype(np.float32)
        if own:
            mask = np.zeros((h, w), np.uint8)
            cv2.fillPoly(mask, [contour], 1)
            region = mask[v, u].astype(bool)
        else:
            region = np.linalg.norm(points - c_cam, axis=1) < radius
        if region.sum() < 30:
            return False
        image_region = np.zeros((h, w), bool)
        image_region[v[region], u[region]] = True
        self.stats['d405'] += cam == 'd405'

        g = self.backend(self.bridge.imgmsg_to_cv2(rgb_msg, 'rgb8'), depth, self.K[cam], image_region)
        if g is None:
            self.stats['no grasp'] += 1
            return True
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
            return True

        # hand surface: cloud around the palm outside the object region (fingers in any pose)
        hand_surface = np.zeros((0, 3))
        if hand is not None:
            mcp = [[p.x, p.y, p.z] for i, (p, s) in enumerate(zip(hand.positions, hand.landmark_state))
                   if i and s in TRACKED]
            if mcp:
                palm = R.T @ (np.mean(mcp, 0) - t_cam)
                near = ~region & (np.linalg.norm(points - palm, axis=1) < self.hand_radius)
                hand_surface = points[near][::max(1, int(near.sum()) // 400)] @ R.T + t_cam
        self.job = (cam, depth_msg, pose, g, hand, hand_surface, points)
        self.job_ready.set()
        return True

    def _collision_free(self, points, g, ok, tips, axes):
        """Free grasps among ok, checked in score order 64 at a time down to the first free one, plus the
        candidates that may keep the previous grasp: the same choice as checking them all."""
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

    def publish(self, cam, depth_msg, pose, g, hand, hand_surface, points):
        R, t_cam = pose
        rot = g['R']  # graspnet: x approach, y closing
        tips = (g['t'] + g['depth'][:, None] * rot[:, :, 0]) @ R.T + t_cam  # camera -> base
        axes = np.einsum('ij,njk->nik', R, rot)
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

        out = GraspPose(width=float(g['width'][best]), score=float(scores[best]), gripper_camera=cam == 'd405')
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
        if cam == 'd405':
            self.gripper_t = t
            self.stats['published d405'] += 1
        near = [(abs(tp - t), p) for tp, p in list(self.palms) if abs(tp - t) <= 0.1]
        self.source = (out, min(near, key=lambda x: x[0])[1] if near else None)


# ============================================================ grasp executor
class GraspExecutor(HandoverQddotCommander):

    def __init__(self):
        super().__init__()
        par = lambda n, v: self.declare_parameter(n, v).value
        self.pregrasp_m = float(par('pregrasp_m', 0.25))            # standoff: the object in the D405 view
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
