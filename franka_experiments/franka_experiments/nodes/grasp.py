#!/usr/bin/env python3
"""Object held by the ACTIVE hand: presence, mask, box and size, in real time.

Hands23 (Cheng et al., NeurIPS 2023) runs on a metric crop centred on the
active palm, in a worker thread. It classifies the hand contact state and,
for "object contact", returns the held object's mask. A gate rejects what a
held object cannot be (much larger than the hand, or away from the palm
depth: tables, robot, background) or only touched by a non-prehensile hand
(palm resting on a marker). Two concordant detections confirm or
release the object. Between detections outline and centroid follow the palm,
so /handover/hand_object is published at HandState rate.
"""

import ctypes
import os
import sys
import threading
import time
from collections import deque

import cv2
import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from franka_msgs.msg import HandObjectState, HandState
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, Image

VENV = '/ros2_ws/offline_object_tests/venv_hands23'
H23 = '/ros2_ws/offline_object_tests/hands23_detector'

CROP_M = 0.30          # half side of the palm-centred crop [m]
DEPTH_BAND_M = 0.15    # object pixels must be this close to the palm depth
MIN_IN_BAND = 0.2      # ... at least this fraction (thin objects: rest is background)
MAX_OBJ_HAND = 2.5     # object box / hand box size (rejects table, robot)
JUMP_M = 0.20          # palm jump meaning "another hand": forget the object
MAX_CENTROID_M = 0.20  # visible object surface farther than this from the palm: background
MAX_LATERAL_M = 0.12   # centroid this far to the side of the hand (half hand + object radius): not in it
OBJECT_CONTACT = 3     # Hands23 contact state "object contact"
NON_PREHENSILE = (0, 1)     # Hands23 grasp: NP-Palm, NP-Fin
TOUCHED_ONLY = (0, 3, 5)    # Hands23 touch: tool / container / neither "touched"
THIN_RATIO = 2.5       # mask principal axes ratio above which an object is thin and elongated


def elongated(mask):
    """Thin elongated mask (pen, screwdriver): main axis > THIN_RATIO x the other."""
    v, u = np.nonzero(mask)
    if len(u) < 5:
        return False
    p = np.column_stack([u, v]).astype(float)
    s = np.linalg.svd(p - p.mean(0), full_matrices=False)[1]
    return s[0] > THIN_RATIO * s[1]


def stamp(msg):
    return msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec


class Hands23:
    """Hands23 on a palm-centred crop -> (observed, object mask | None)."""

    def __init__(self, root=H23, input_size=384, device='cuda', fp16=False, thin_objects=True):
        if device == 'cuda':
            # On this laptop the first cuInit after GPU idle often returns
            # NOT_INITIALIZED: wait until the driver answers before torch does.
            cuda, t0 = ctypes.CDLL('libcuda.so.1'), time.time()
            while cuda.cuInit(0) != 0 and time.time() - t0 < 10.0:
                time.sleep(0.2)
        sys.path.insert(0, root)
        from detectron2.config import get_cfg
        from detectron2.engine import DefaultPredictor
        from hodetector.modeling import roi_heads  # noqa: F401 (registers heads)
        cfg = get_cfg()
        cfg.merge_from_file(f'{root}/faster_rcnn_X_101_32x8d_FPN_3x_Hands23.yaml')
        cfg.MODEL.WEIGHTS = f'{root}/model_weights/model_hands23.pth'
        cfg.MODEL.DEVICE = device
        cfg.INPUT.MIN_SIZE_TEST = cfg.INPUT.MAX_SIZE_TEST = input_size
        cfg.MODEL.ROI_HEADS.SCORE_THRESH_TEST = 0.3
        cfg.HAND, cfg.FIRSTOBJ, cfg.SECONDOBJ = 0.5, 0.3, 0.3
        cfg.HAND_RELA, cfg.OBJ_RELA = 0.3, 0.7
        self.predictor = DefaultPredictor(cfg)
        import torch
        # half precision on the GPU (tensor cores): same detections, faster
        self.torch, self.fp16 = torch, bool(fp16) and device == 'cuda'
        self.thin_objects = bool(thin_objects)

    def __call__(self, bgr, depth, palm_px, palm_z, fx, keep=False):
        observed, obj, gripping = self._detect(bgr, depth, palm_px, palm_z, fx, CROP_M, False)
        if self.thin_objects and keep and observed and obj is None and gripping:
            # the hand grips something but no object passed: a thin one (pen) is a few pixels
            # wide at 384 px and its mask bleeds onto the background. Second pass on a crop of
            # half the side (2x pixels on hand and object), only in these frames. It only keeps
            # an object already confirmed (keep), never confirms one: cables, table edge and
            # robot links next to the hand would otherwise pass as short false objects.
            observed2, obj2, _ = self._detect(bgr, depth, palm_px, palm_z, fx, CROP_M / 2, True)
            if obj2 is not None:
                return observed2, obj2
        return observed, obj

    def _detect(self, bgr, depth, palm_px, palm_z, fx, crop_m, clean):
        h, w = depth.shape
        r = int(crop_m * fx / palm_z)
        x0, y0 = max(0, int(palm_px[0]) - r), max(0, int(palm_px[1]) - r)
        x1, y1 = min(w, int(palm_px[0]) + r), min(h, int(palm_px[1]) + r)
        if x1 - x0 < 32 or y1 - y0 < 32:
            return False, None, False
        with self.torch.autocast('cuda', dtype=self.torch.float16, enabled=self.fp16):
            inst = self.predictor(bgr[y0:y1, x0:x1])['instances'].to('cpu')
        box = inst.pred_boxes.tensor.numpy() + [x0, y0, x0, y0]
        dz, cls, score = inst.pred_dz.numpy(), inst.pred_classes.numpy(), inst.scores.numpy()
        hands = [i for i in np.flatnonzero(cls == 0)
                 if box[i, 0] <= palm_px[0] <= box[i, 2] and box[i, 1] <= palm_px[1] <= box[i, 3]]
        if not hands:
            return False, None, False  # active hand not seen: no evidence either way
        i = max(hands, key=lambda j: score[j])
        o = int(dz[i, 4])
        gripping = int(dz[i, 8]) == OBJECT_CONTACT and int(dz[i, 6]) not in NON_PREHENSILE
        if int(dz[i, 8]) != OBJECT_CONTACT or o < 0:
            return True, None, gripping
        # a flat hand resting on something (marker, table item) is not holding it
        if int(dz[i, 6]) in NON_PREHENSILE and int(dz[o, 7]) in TOUCHED_ONLY:
            return True, None, gripping
        size = lambda b: max(b[2] - b[0], b[3] - b[1])

        def full(m):
            out = np.zeros((h, w), bool)
            out[y0:y1, x0:x1] = m
            return out
        obj = full(inst.pred_masks[o].numpy())
        hand = full(inst.pred_masks[i].numpy())
        if clean:
            # Keep the parts of the object mask attached to the hand and drop the pieces lying
            # behind it: a thin object's mask bleeds onto what it points at (robot base, cables).
            near_hand = cv2.dilate(hand.astype(np.uint8), np.ones((15, 15), np.uint8)) > 0
            n, labels = cv2.connectedComponents(obj.astype(np.uint8))
            kept = np.zeros_like(obj)
            for c in range(1, n):
                part = labels == c
                z = depth[part]
                z = z[z > 0.1]
                behind = z.size and np.mean(z > palm_z + DEPTH_BAND_M) > 0.8
                if (part & near_hand).any() and not behind:
                    kept |= part
            obj = kept
            if obj.sum() < 20:
                return True, None, gripping
            v, u = np.nonzero(obj)
            box[o] = [u.min(), v.min(), u.max(), v.max()]
        if size(box[o]) > MAX_OBJ_HAND * size(box[i]):
            return True, None, gripping
        if obj.sum() < 20:
            return True, None, gripping
        z = depth[obj]
        z = z[z > 0.1]
        if z.size and np.mean(np.abs(z - palm_z) < DEPTH_BAND_M) < MIN_IN_BAND:
            return True, None, gripping
        return True, {'mask': obj, 'hand': hand, 'score': float(score[o])}, gripping


def measure(obj, depth, K, palm_z, thin_objects=True):
    """Outline, box, centroid (camera) and principal extents of the object.

    3D only from object pixels near the palm depth: the mask of a thin object
    (pen) or one bleeding onto the background has mostly background depth.
    """
    m = obj['mask'].astype(np.uint8)
    cs, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contour = cv2.approxPolyDP(max(cs, key=cv2.contourArea), 1.5, True).reshape(-1, 2)
    x, y, bw, bh = cv2.boundingRect(m)
    out = {'contour': contour.astype(float), 'bbox': np.array([x, y, x + bw, y + bh], float),
           'centroid': None, 'dims': np.full(3, np.nan), 'score': obj['score']}
    # depth of the object only: drop the fingers wrapped around it
    hand = cv2.dilate(obj['hand'].astype(np.uint8), np.ones((5, 5), np.uint8)).astype(bool)
    thin = thin_objects and elongated(obj['mask'])
    mask = obj['mask']
    if thin:
        # only the mask pieces attached to the hand: a separate piece is bleeding onto the
        # background (robot base) and would add its distance to the length
        near_hand = cv2.dilate(obj['hand'].astype(np.uint8), np.ones((15, 15), np.uint8)) > 0
        n, labels = cv2.connectedComponents(mask.astype(np.uint8))
        attached = np.isin(labels, [c for c in range(1, n) if (near_hand & (labels == c)).any()])
        if attached.sum() >= 30:
            mask = attached
    for m in (mask & ~hand, mask):  # whole mask if fingers cover it
        v, u = np.nonzero(m)
        z = depth[v, u]
        keep = (z > 0.1) & (np.abs(z - palm_z) < DEPTH_BAND_M)
        if thin and keep.sum() < 30:
            continue
        if thin:
            # a thin object (shaft, tip, pen) has the depth of what is behind it: put those
            # pixels at the depth of the object near the fingers, so its length is complete
            z = np.where(keep, z, np.median(z[keep]))
            keep = np.ones_like(keep)
            break
        if keep.sum() >= 30:
            break
    else:
        if not thin:
            return out
        v, u = np.nonzero(mask & ~hand)
        if len(u) < 30:
            return out
        z = np.full(len(u), palm_z)
        keep = np.ones(len(u), bool)
    u, v, z = u[keep], v[keep], z[keep]
    keep = np.abs(z - np.median(z)) < 0.10  # background bleeding at the outline
    fx, fy, cx, cy = K
    p = np.column_stack([(u - cx) * z / fx, (v - cy) * z / fy, z])[keep]
    c = p.mean(0)
    _, _, axes = np.linalg.svd(p - c, full_matrices=False)
    q = (p - c) @ axes.T
    out['centroid'] = c
    out['dims'] = np.sort(np.percentile(q, 98, 0) - np.percentile(q, 2, 0))[::-1]
    return out


class Grasp(Node):

    def __init__(self):
        super().__init__('grasp')
        par = lambda n, v: self.declare_parameter(n, v).value
        self.confirm = int(par('confirm_detections', 2))
        self.release = int(par('release_detections', 2))
        self.release_s = float(par('release_s', 0.3))
        self.hold_s = float(par('hold_s', 0.6))
        self.max_dt = float(par('max_state_image_dt_s', 0.05))
        self.thin_objects = bool(par('thin_objects', True))
        self.detector = Hands23(par('hands23_root', H23), int(par('input_size', 384)),
                                par('device', 'cuda'), bool(par('fp16', True)), self.thin_objects)

        ext = yaml.safe_load(open(os.path.join(
            get_package_share_directory('franka_experiments'), 'config', 'camera_extrinsics.yaml')))
        t, q = ext['translation'], ext['rotation']
        self.t = np.array([t['x'], t['y'], t['z']])
        self.R = Rotation.from_quat([q['x'], q['y'], q['z'], q['w']]).as_matrix()  # camera -> base

        self.bridge = CvBridge()
        self.K = None
        self.rgb, self.depth, self.states = deque(maxlen=4), deque(maxlen=8), deque(maxlen=60)
        self.lock = threading.Lock()
        self.reset()
        self.last_palm = None
        self.pub = self.create_publisher(HandObjectState, '/handover/hand_object', 10)
        cam = '/camera/camera/'
        self.create_subscription(CameraInfo, cam + 'aligned_depth_to_color/camera_info',
                                 self.on_info, qos_profile_sensor_data)
        self.create_subscription(Image, cam + 'color/image_raw', self.rgb.append,
                                 qos_profile_sensor_data)
        self.create_subscription(Image, cam + 'aligned_depth_to_color/image_raw',
                                 self.depth.append, qos_profile_sensor_data)
        self.create_subscription(HandState, '/handover/hand_state', self.on_state, 10)
        self.ms = deque(maxlen=50)
        self.create_timer(5.0, self.log_rate,
                          clock=Clock(clock_type=ClockType.STEADY_TIME))  # sim time jumps at bag start
        self.stopping = threading.Event()
        self.thread = threading.Thread(target=self.worker, daemon=True)
        self.thread.start()

    def log_rate(self):
        # only while detections are running (silent once the bag/camera stops)
        if self.ms:
            self.get_logger().info(f'Hands23 {np.median(self.ms):.0f} ms/detection, present={self.present}')
            self.ms.clear()

    def reset(self):
        self.palm_ref, self.pos, self.neg, self.present = None, 0, 0, False
        self.vel_ref, self.t_ref = np.zeros(3), None
        self.obj, self.seen, self.observed = None, None, None

    def on_info(self, msg):
        self.K = np.array([msg.k[0], msg.k[4], msg.k[2], msg.k[5]])

    def to_camera(self, p_base):
        return self.R.T @ (p_base - self.t)

    def to_pixel(self, p_cam):
        fx, fy, cx, cy = self.K
        return np.array([fx * p_cam[0] / p_cam[2] + cx, fy * p_cam[1] / p_cam[2] + cy])

    @staticmethod
    def palm(state):
        p = state.palm_position
        return np.array([p.x, p.y, p.z])

    @staticmethod
    def velocity(state):
        v = state.palm_velocity
        v = np.array([v.x, v.y, v.z])
        return v if state.velocity_valid and np.isfinite(v).all() else np.zeros(3)

    def jump(self, palm, t):
        """Palm distance from where the reference hand should be now (constant velocity,
        at most 0.3 s ahead): a fast hand moves >JUMP_M between two detections, another
        hand does not follow the previous one's motion."""
        dt = 0.0 if self.t_ref is None else min(max(t - self.t_ref, 0.0), 0.3)
        return np.linalg.norm(palm - (self.palm_ref + self.vel_ref * dt))

    @classmethod
    def lateral(cls, state, c_base):
        """Distance of the object centroid from the hand's own plane (wrist -> fingers,
        palm normal): an object held in the hand lies across it, not beside it (table
        under a side palm, neighbouring objects). 0 if the hand axes are unknown."""
        n, u = state.palm_normal, state.palm_longitudinal
        side = np.cross([n.x, n.y, n.z], [u.x, u.y, u.z])
        norm = np.linalg.norm(side)
        return 0.0 if not norm > 0.5 else abs(float((c_base - cls.palm(state)) @ side) / norm)

    # ------------------------------------------------------------ detection
    def next_job(self, done):
        if self.K is None or not self.rgb or self.rgb[-1] is done:
            return None
        rgb = self.rgb[-1]
        t = stamp(rgb)
        depth = next((d for d in reversed(self.depth) if abs(stamp(d) - t) < 1e-3), None)
        state = min(self.states, key=lambda s: abs(stamp(s) - t), default=None)
        if depth is None or state is None or abs(stamp(state) - t) > self.max_dt:
            return None
        return rgb, depth, state

    def worker(self):
        done = None
        while rclpy.ok() and not self.stopping.is_set():
            job = self.next_job(done)
            if job is None:
                time.sleep(0.005)
                continue
            rgb, depth_msg, state = job
            done = rgb
            if not state.position_valid:
                continue
            bgr = self.bridge.imgmsg_to_cv2(rgb, 'bgr8')
            depth = self.bridge.imgmsg_to_cv2(depth_msg).astype(np.float32)
            if depth_msg.encoding == '16UC1':
                depth *= 1e-3
            palm_cam = self.to_camera(self.palm(state))
            palm_px = self.to_pixel(palm_cam)
            t0 = time.perf_counter()
            observed, obj = self.detector(bgr, depth, palm_px, palm_cam[2], self.K[0], keep=self.present)
            self.ms.append(1e3 * (time.perf_counter() - t0))
            if obj is not None:
                obj = measure(obj, depth, self.K, palm_cam[2], self.thin_objects)
                c = obj['centroid']
                c_base = None if c is None else self.R @ c + self.t
                if c is not None and (np.linalg.norm(c - palm_cam) > MAX_CENTROID_M
                                      or self.lateral(state, c_base) > MAX_LATERAL_M):
                    obj = None
                else:
                    obj.update(palm_px=palm_px, palm=self.palm(state))
                    if c is not None:
                        obj['centroid'] = c_base
            self.update(stamp(rgb), self.palm(state), self.velocity(state), observed, obj)

    def update(self, t, palm, velocity, observed, obj):
        # Same hand = continuous palm, not the LEFT/RIGHT label (it flips).
        with self.lock:
            if self.palm_ref is not None and self.jump(palm, t) > JUMP_M:
                self.reset()
            self.palm_ref, self.vel_ref, self.t_ref = palm, velocity, t
            if not observed:
                return
            self.observed = t
            if obj is not None:
                self.pos, self.neg, self.obj, self.seen = self.pos + 1, 0, obj, t
            else:
                self.pos, self.neg = 0, self.neg + 1
            if self.pos >= self.confirm:
                self.present = True
            if self.neg >= self.release and (self.seen is None or t - self.seen >= self.release_s):
                self.present = False

    # ------------------------------------------------------------ output
    def on_state(self, state):
        self.states.append(state)
        t = stamp(state)
        out = HandObjectState(header=state.header, physical_hand=state.physical_hand)
        out.object_centroid_3d.x = out.object_centroid_3d.y = out.object_centroid_3d.z = float('nan')
        out.object_age = float('inf')
        if state.position_valid:
            self.last_palm = self.palm(state)
        palm = self.last_palm  # short tracking gaps: keep the last palm
        with self.lock:
            out.valid = bool(palm is not None and self.palm_ref is not None
                             and self.jump(palm, t) <= JUMP_M
                             and self.observed is not None and 0 <= t - self.observed <= self.hold_s)
            out.object_present = bool(out.valid and self.present and self.obj is not None)
            if self.seen is not None:
                out.object_age = float(max(0.0, t - self.seen))
            if out.object_present and self.K is not None:
                o = self.obj
                dpx = self.to_pixel(self.to_camera(palm)) - o['palm_px']
                out.object_confidence = o['score']
                out.bbox_px = (o['bbox'] + np.tile(dpx, 2)).tolist()
                out.contour_px = np.round(o['contour'] + dpx).astype(int).ravel().tolist()
                out.dimensions.x, out.dimensions.y, out.dimensions.z = map(float, o['dims'])
                if o['centroid'] is not None:
                    c = o['centroid'] + palm - o['palm']
                    out.object_centroid_3d.x, out.object_centroid_3d.y, out.object_centroid_3d.z = map(float, c)
        self.pub.publish(out)


def main(args=None):
    # torch/detectron2 live in the Hands23 venv: re-exec this node there.
    if os.path.realpath(sys.prefix) != os.path.realpath(VENV):
        py = os.path.join(VENV, 'bin', 'python')
        os.execv(py, [py, '-m', 'franka_experiments.nodes.grasp', *sys.argv[1:]])
    rclpy.init(args=args)
    node = Grasp()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        # stop CUDA work before interpreter teardown (else segfault + core dump)
        node.stopping.set()
        node.thread.join(timeout=2.0)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
