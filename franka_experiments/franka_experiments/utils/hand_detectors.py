#!/usr/bin/env python3
"""Hand detectors: the two alternative main-camera detectors (hand_detector parameter) and the
gripper camera (D405, eye-in-hand) as a second view of the hand.

image -> body pose + left / right hand landmarks of one person.

RtmwHandDetector       RTMW whole-body (rtmlib, TensorRT FP16 on the GPU), the default.
MediapipeHandDetector  MediaPipe Holistic on the CPU, fallback when RTMW cannot start.
process() runs in the camera callback, complete() in the tracker worker (MediaPipe re-detection).
Both return pose_landmarks / left_hand_landmarks / right_hand_landmarks in the MediaPipe
format (normalised x, y; pose ids 11/12 shoulders, 13/14 elbows, 15/16 wrists, 23/24 hips;
hand ids 0 wrist, 5/9/13/17 MCP), so the tracker does not depend on the detector.

GripperCamera: Passive unless its images and the robot TF arrive: MediaPipe Hands runs in a
worker thread and the hands are mapped to the base frame with
TF(base -> fr3_link8) and the fixed transform of config/d405_extrinsics.yaml.
It runs only while the last ACTIVE palm (<= 1 s old) is in its view: no CPU
taken from the main tracker when the hand is elsewhere.

Offline (handover_20260929_161100, hand near the gripper): it re-finds 84% of
the hands the main camera loses inside its view, and its palm normal is right
on 20/22 labelled hands (main camera 16/22).
"""

import ctypes
import os
import threading
import time
from types import SimpleNamespace

import cv2
import mediapipe as mp
import numpy as np
import yaml
from cv_bridge import CvBridge
from mediapipe.framework.formats import landmark_pb2
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, Image


class OneEuro:
    """One-Euro filter (Casiez et al. 2012) on hand keypoints, speed in hand sizes/s.
    Tuned offline (6 bags): MCP jitter 0.035 -> 0.020 hand sizes, lag ~0.02 hand sizes."""

    def __init__(self, min_cutoff=3.0, beta=5.0, d_cutoff=1.0):
        self.min_cutoff, self.beta, self.d_cutoff = min_cutoff, beta, d_cutoff
        self.x = self.dx = self.t = None

    def __call__(self, t, x, scale):
        if self.x is None or not 0.0 < t - self.t <= 0.2:
            self.x, self.dx, self.t = x, np.zeros_like(x), t
            return x

        dt = t - self.t
        alpha = lambda fc: 1.0 / (1.0 + 1.0 / (2.0 * np.pi * fc * dt))
        self.dx = self.dx + alpha(self.d_cutoff) * ((x - self.x) / dt / scale - self.dx)
        self.x = self.x + alpha(self.min_cutoff + self.beta * np.abs(self.dx)) * (x - self.x)
        self.t = t
        return self.x


def patch_depth(depth, scale, uv, radius=3):
    """Median depth [m] of a (2 radius + 1)^2 patch, None with <= 5 valid pixels."""
    h, w = depth.shape[:2]
    u, v = int(np.clip(uv[0], 0, w - 1)), int(np.clip(uv[1], 0, h - 1))
    patch = depth[max(0, v - radius):v + radius + 1, max(0, u - radius):u + radius + 1]
    patch = patch[patch > 0]
    return float(np.median(patch)) * scale if patch.size > 5 else None


def iou(a, b):
    x0, y0, x1, y1 = max(a[0], b[0]), max(a[1], b[1]), min(a[2], b[2]), min(a[3], b[3])
    inter = max(0.0, x1 - x0) * max(0.0, y1 - y0)
    union = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
    return inter / union if union > 0 else 0.0


class RtmwHandDetector:
    """RTMW (Jiang et al. 2024) on the person boxes; COCO-WholeBody hands 91-111 (left) and
    112-132 (right) already follow the MediaPipe hand order. RTMW gives no hand z.

    Hands kept: score hysteresis (enter 0.8, keep 0.5), one box per hand (IoU), metric
    size 5-30 cm at the wrist depth; offline against Hands23: recall 0.905 (Holistic 0.800)."""

    POSE_FROM_BODY = {0: 0, 5: 11, 6: 12, 7: 13, 8: 14, 9: 15, 10: 16, 11: 23, 12: 24}
    # key, first keypoint, body wrist keypoint
    HANDS = (('left_hand_landmarks', 91, 9), ('right_hand_landmarks', 112, 10))
    SWITCH_M = 0.3     # another person takes over when this much nearer the robot
    HUMAN_MIN = 0.7    # mean face + best hand keypoint score; the robot arm seen as a person stays below
    SWITCH_FRAMES = 5  # frames in a row a nearer person must be seen before taking over

    def __init__(self, mode='lightweight', det_frequency=10, enter=0.8, keep=0.5, tensorrt=True, to_base=None):
        cuda, t0 = ctypes.CDLL('libcuda.so.1'), time.time()
        while cuda.cuInit(0) != 0 and time.time() - t0 < 10.0:  # flaky first cuInit
            time.sleep(0.2)
        import torch  # noqa: F401  loads the CUDA / cuDNN libraries onnxruntime needs
        from rtmlib import Wholebody
        from rtmlib.tools.solution.pose_tracker import pose_to_bbox

        model = Wholebody(mode=mode, backend='onnxruntime', device='cuda')
        self.engine = self._tensorrt(model) if tensorrt else 'CUDA FP32'
        self.detector, self.pose, self.pose_to_bbox = model.det_model, model.pose_model, pose_to_bbox

        # as rtmlib's PoseTracker (person boxes from the last keypoints, refreshed by the person
        # detector every det_frequency frames), but the detector runs in its own thread
        self.det_frequency, self.n, self.boxes = det_frequency, 0, []
        self.det_in, self.det_out = None, None
        self.det_wake, self.det_stop = threading.Event(), threading.Event()
        self.det_thread = threading.Thread(target=self._detect_loop, daemon=True)
        self.det_thread.start()

        self.enter, self.keep = enter, keep
        self.kept, self.centre = set(), None
        self.pending, self.pending_n = None, 0  # nearer person waiting to take over
        self.to_base = to_base  # (u, v, z) -> robot base frame point; None = largest person
        self.filters = {key: OneEuro() for key, *_ in self.HANDS}

    @staticmethod
    def _tensorrt(model):
        """Same ONNX models compiled by TensorRT in FP16 (RTX 3050: pose 16.5 -> 2.0 ms, YOLOX-tiny
        8.1 -> 1.6 ms, outputs within ~1 %); engines cached on disk, built at the first start (~2 min).
        YOLOX-m (balanced detector) does not build with TensorRT (TopK) and stays on CUDA."""
        try:
            import tensorrt  # noqa: F401  loads libnvinfer for onnxruntime's TensorRT EP
            import onnxruntime as ort
        except ImportError:
            return 'CUDA FP32 (TensorRT not installed)'

        cache = os.path.expanduser('~/.cache/rtmlib/trt')
        os.makedirs(cache, exist_ok=True)
        providers = [('TensorrtExecutionProvider', {
            'trt_fp16_enable': True, 'trt_engine_cache_enable': True,
            'trt_engine_cache_path': cache, 'trt_timing_cache_enable': True}), 'CUDAExecutionProvider']

        done = []
        for name, tool in (('pose', model.pose_model), ('detector', model.det_model)):
            if 'yolox_m' in tool.onnx_model:
                continue
            try:
                session = ort.InferenceSession(tool.onnx_model, providers=providers)
                if session.get_providers()[0] == 'TensorrtExecutionProvider':
                    tool.session = session
                    done.append(name)
            except Exception:  # keep the CUDA session
                pass

        return f'TensorRT FP16 ({", ".join(done)})' if done else 'CUDA FP32'

    @staticmethod
    def complete(result, bgr, depth, depth_scale, t):
        return result

    def stop(self):
        self.det_stop.set()
        self.det_wake.set()
        self.det_thread.join(timeout=1.0)

    def _detect_loop(self):
        while not self.det_stop.is_set():
            if not self.det_wake.wait(0.5):
                continue
            self.det_wake.clear()
            image, self.det_in = self.det_in, None
            if image is not None:
                self.det_out = list(self.detector(image))

    def _people(self, bgr):
        self.n += 1
        if self.n % self.det_frequency == 0 or not self.boxes:
            self.det_in = bgr
            self.det_wake.set()

        if self.det_out is not None:  # newest detection replaces the keypoint boxes
            self.boxes, self.det_out = self.det_out, None
        if not self.boxes:
            return [], []

        keypoints, scores = self.pose(bgr, bboxes=self.boxes)
        self.boxes = [self.pose_to_bbox(k) for k in keypoints]
        return keypoints, scores

    # ------------------------------------------------------------ one person -> pose + hands
    def process(self, bgr, depth, depth_scale, fx, t):
        out = SimpleNamespace(pose_landmarks=None, left_hand_landmarks=None, right_hand_landmarks=None)
        keypoints, scores = self._people(bgr)
        h, w = bgr.shape[:2]
        person = self._person(keypoints, scores, w, depth, depth_scale)
        if person is None:
            self.kept = set()
            for f in self.filters.values():
                f.x = None
            return out

        # body: COCO keypoints -> MediaPipe pose ids
        k, s = keypoints[person], np.clip(scores[person], 0.0, 1.0)
        pose = landmark_pb2.NormalizedLandmarkList()
        for _ in range(33):
            pose.landmark.add(visibility=0.0)
        for body, mp_id in self.POSE_FROM_BODY.items():
            q = pose.landmark[mp_id]
            q.x, q.y, q.visibility = k[body, 0] / w, k[body, 1] / h, s[body]
        out.pose_landmarks = pose

        # hands: score hysteresis, metric size at the wrist depth, one box per hand
        hands = {}
        for key, first, wrist in self.HANDS:
            p, c = k[first:first + 21], s[first:first + 21]
            score, box = float(c.mean()), np.r_[p.min(0), p.max(0)]
            if score < (self.keep if key in self.kept else self.enter):
                continue
            z = patch_depth(depth, depth_scale, k[wrist])
            if z is not None and not 0.05 <= max(box[2] - box[0], box[3] - box[1]) * z / fx <= 0.30:
                continue
            hands[key] = (score, p, c, box)

        if len(hands) == 2 and iou(*(v[3] for v in hands.values())) > 0.3:  # both boxes on one hand
            del hands[min(hands, key=lambda key: hands[key][0])]
        self.kept = set(hands)
        for key, f in self.filters.items():
            if key not in hands:
                f.x = None  # hand lost: restart the filter

        for key, (_, p, c, box) in hands.items():
            p = self.filters[key](t, p, max(box[2] - box[0], box[3] - box[1], 10.0))
            lms = landmark_pb2.NormalizedLandmarkList()
            for (u, v), ci in zip(p, c):
                lms.landmark.add(x=u / w, y=v / h, z=0.0, visibility=ci)
            setattr(out, key, lms)
        return out

    def _person(self, keypoints, scores, w, depth=None, depth_scale=1.0):
        """The person interacting with the robot: the one tracked so far; another one takes over
        when its torso is nearer the robot base by SWITCH_M for SWITCH_FRAMES frames (a person at a
        desk behind does not keep the hands). Only people with face and hands count (HUMAN_MIN):
        RTMW also sees the robot arm as a "person" whose torso sits on the robot base.
        Without depth / robot geometry: the largest person."""
        people = []
        for i, (k, s) in enumerate(zip(keypoints, scores)):
            pts = k[:17][s[:17] > 0.3]
            if len(pts) >= 4:
                human = 0.5 * (s[23:91].mean() + max(s[91:112].mean(), s[112:133].mean()))
                people.append((i, pts.mean(0), np.ptp(pts[:, 0]) * np.ptp(pts[:, 1]),
                               self._robot_distance(k, s, depth, depth_scale), human))
        if not people:
            self.centre = None
            return None

        # people: (index, centre px, area px, robot distance, human score)
        near = ([] if self.centre is None
                else [p for p in people if np.linalg.norm(p[1] - self.centre) < 0.15 * w])
        current = min(near, key=lambda p: np.linalg.norm(p[1] - self.centre)) if near else None
        located = [p for p in people if p[3] is not None and p[4] >= self.HUMAN_MIN]
        nearest = min(located, key=lambda p: p[3]) if located else None

        if nearest is not None and current is None:
            current = nearest
        elif (nearest is not None and nearest[0] != current[0]
              and (current[3] is None
                   or current[4] < self.HUMAN_MIN
                   or nearest[3] < current[3] - self.SWITCH_M)):
            same = self.pending is not None and np.linalg.norm(nearest[1] - self.pending) < 0.15 * w
            self.pending, self.pending_n = nearest[1], (self.pending_n + 1 if same else 1)
            if self.pending_n >= self.SWITCH_FRAMES:
                current, self.pending, self.pending_n = nearest, None, 0
        else:
            self.pending, self.pending_n = None, 0

        i, self.centre = (current or max(people, key=lambda p: p[2]))[:2]
        return i

    def _robot_distance(self, k, s, depth, depth_scale):
        """Horizontal distance of the torso (nearest shoulder or hip) from the robot base."""
        if self.to_base is None or depth is None:
            return None

        d = []
        for j in (5, 6, 11, 12):
            z = patch_depth(depth, depth_scale, k[j]) if s[j] > 0.3 else None
            if z is not None and 0.1 < z < 6.0:
                d.append(float(np.linalg.norm(self.to_base(k[j, 0], k[j, 1], z)[:2])))
        return min(d) if d else None


class MediapipeHandDetector:
    """MediaPipe Holistic (CPU), made robust (evaluation in GRASP.md):
      - a hand seen in the last REDETECT_S but missing now (palm covered by the held object,
        fast motion) is re-detected by MediaPipe Hands on a crop around its last box, if near
        it and at the same depth (15 cm);
      - a hand is kept only if the pose wrist of its side is near it."""

    REDETECT_S = 0.5      # re-detect a hand lost for at most this long
    REDETECT_SCALE = 2.2  # crop side / last hand box side

    def __init__(self, static_image_mode=False, model_complexity=1,
                 min_detection_confidence=0.4, min_tracking_confidence=0.5):
        self.holistic = mp.solutions.holistic.Holistic(
            static_image_mode=static_image_mode, model_complexity=model_complexity,
            smooth_landmarks=True, enable_segmentation=False, refine_face_landmarks=False,
            min_detection_confidence=min_detection_confidence, min_tracking_confidence=min_tracking_confidence)
        self.redetector = mp.solutions.hands.Hands(
            static_image_mode=True, max_num_hands=1, model_complexity=1, min_detection_confidence=0.3)
        self.engine = 'MediaPipe Holistic (CPU)'
        self._last_hands = {}

    def stop(self):
        self.holistic.close()
        self.redetector.close()

    def process(self, bgr, depth, depth_scale, fx, t):
        return self.holistic.process(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB))

    def complete(self, result, bgr, depth, depth_scale, t):
        h, w = bgr.shape[:2]
        out = SimpleNamespace(pose_landmarks=result.pose_landmarks,
                              left_hand_landmarks=result.left_hand_landmarks,
                              right_hand_landmarks=result.right_hand_landmarks)

        for key, wrist_id in (('left_hand_landmarks', 15), ('right_hand_landmarks', 16)):
            lms, last = getattr(out, key), self._last_hands.get(key)
            if lms is None and last is not None and t - last[0] <= self.REDETECT_S:
                lms = self._redetect(bgr, depth, depth_scale, *last[1:])

            # keep the hand only if the pose wrist of its side is near it
            p = None if lms is None else np.array([[q.x * w, q.y * h] for q in lms.landmark])
            if p is not None and out.pose_landmarks is not None:
                q = out.pose_landmarks.landmark[wrist_id]
                if np.hypot(q.x * w - p[0, 0], q.y * h - p[0, 1]) >= 1.5 * max(np.ptp(p, 0).max(), 20):
                    p = None
            elif out.pose_landmarks is None:
                p = None

            setattr(out, key, None if p is None else lms)
            if p is not None:
                self._last_hands[key] = (t, p, self._palm_depth(p, depth, depth_scale))
        return out

    @staticmethod
    def _palm_depth(p, depth, scale):
        h, w = depth.shape[:2]
        z = [float(depth[int(np.clip(v, 0, h - 1)), int(np.clip(u, 0, w - 1))]) * scale
             for u, v in p[[0, 5, 9, 17]]]
        z = [x for x in z if x > 0.1]
        return float(np.median(z)) if z else np.nan

    def _redetect(self, bgr, depth, scale, last_px, last_z):
        h, w = bgr.shape[:2]
        c = (last_px.min(0) + last_px.max(0)) / 2
        r = max(np.ptp(last_px, 0).max() * self.REDETECT_SCALE, 48) / 2
        x0, y0 = int(max(0, c[0] - r)), int(max(0, c[1] - r))
        x1, y1 = int(min(w, c[0] + r)), int(min(h, c[1] + r))
        if x1 - x0 < 8 or y1 - y0 < 8:  # last box left the image
            return None

        crop = cv2.cvtColor(bgr[y0:y1, x0:x1], cv2.COLOR_BGR2RGB)
        k = 192 / max(1, min(crop.shape[:2]))
        found = self.redetector.process(cv2.resize(crop, None, fx=k, fy=k) if k > 1 else crop)
        if not found.multi_hand_landmarks:
            return None

        # crop coordinates -> full image; accepted only near the last box and at its depth
        lms = landmark_pb2.NormalizedLandmarkList()
        for q in found.multi_hand_landmarks[0].landmark:
            lms.landmark.add(x=(x0 + q.x * (x1 - x0)) / w, y=(y0 + q.y * (y1 - y0)) / h, z=q.z)
        p = np.array([[q.x * w, q.y * h] for q in lms.landmark])
        if (np.linalg.norm(p.mean(0) - last_px.mean(0)) >= np.ptp(last_px, 0).max()
                or not abs(self._palm_depth(p, depth, scale) - last_z) < 0.15):
            return None
        return lms


# ============================================================ gripper camera (D405)
def signed_palm_normal(points, right):
    """Normal of the wrist + MCP plane (21 hand points), out of the palm."""
    P = np.asarray(points, dtype=float)
    Q = P[[0, 5, 9, 13, 17]] - P[[0, 5, 9, 13, 17]].mean(0)
    n = np.linalg.eigh(Q.T @ Q)[1][:, 0]
    a = np.cross(P[5] - P[0], P[17] - P[0]) * (1.0 if right else -1.0)
    return n if n @ a >= 0.0 else -n


class GripperCamera:

    PALM_IDS = (0, 5, 9, 17)  # same landmarks as HandTrackingRaw

    def __init__(self, node, extrinsics_path, tf_lookup, namespace='/d405/d405', debug=False, target=None):
        with open(extrinsics_path, 'r', encoding='utf-8') as file:
            e = yaml.safe_load(file)
        q, t = e['rotation'], e['translation']
        self.link = e['parent_frame']
        self.R_link_cam = Rotation.from_quat([q['x'], q['y'], q['z'], q['w']]).as_matrix()
        self.t_link_cam = np.array([t['x'], t['y'], t['z']], dtype=float)

        self.tf_lookup = tf_lookup
        self.target = target  # () -> (t, palm in base) of the last ACTIVE hand, or None
        self.logger, self.state = node.get_logger(), None
        self.bridge = CvBridge()
        self.K, self.frame, self.result = None, None, None
        self.hands = mp.solutions.hands.Hands(
            static_image_mode=False, max_num_hands=2, model_complexity=1,
            min_detection_confidence=0.5, min_tracking_confidence=0.5)
        self.ms = []

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.sync = ApproximateTimeSynchronizer(
            [Subscriber(node, Image, f'{namespace}/color/image_raw', qos_profile=qos),
             Subscriber(node, Image, f'{namespace}/aligned_depth_to_color/image_raw', qos_profile=qos)],
            queue_size=2, slop=0.05)
        self.sync.registerCallback(self._images)
        node.create_subscription(CameraInfo, f'{namespace}/aligned_depth_to_color/camera_info',
                                 self._info, qos_profile_sensor_data)

        # landmarks drawn on the D405 image; hand_visualizer adds the HandState overlay
        self.debug = node.create_publisher(Image, '/handover/gripper_debug_image_raw', 2) if debug else None

        self.stopping, self.wake = threading.Event(), threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def _info(self, msg):
        if msg.k[0] > 0.0:
            self.K = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])

    def _images(self, rgb_msg, depth_msg):
        self.frame = (rgb_msg, depth_msg)  # newest wins
        self.wake.set()

    def stop(self):
        self.stopping.set()
        self.wake.set()
        self.thread.join(timeout=2.0)
        self.hands.close()

    def _run(self):
        while not self.stopping.is_set():
            self.wake.wait(0.5)
            self.wake.clear()
            frame, self.frame = self.frame, None
            if frame is None or self.K is None:
                continue

            rgb_msg, depth_msg = frame
            pose = self.tf_lookup(self.link, rgb_msg.header.stamp)
            self._log_state(pose is not None)
            if pose is None:            # no robot TF: the camera cannot be placed
                self.result = None
                continue

            # camera pose in the base frame
            R_base_link, t_base_link = pose
            R = R_base_link @ self.R_link_cam
            t = R_base_link @ self.t_link_cam + t_base_link

            rgb = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='rgb8')
            h, w = rgb.shape[:2]
            t_img = rgb_msg.header.stamp.sec + 1e-9 * rgb_msg.header.stamp.nanosec
            hands, found = [], []

            if self._in_view(t_img, R, t, w, h):
                t0 = time.perf_counter()
                depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
                found = self._detect(rgb)
                for V in found:
                    pts = [self._point(depth, *V[i, :2]) for i in self.PALM_IDS]
                    hands.append({'pts': [None if p is None else R @ p + t for p in pts], 'V': V, 'R': R})
                self._log_time(1e3 * (time.perf_counter() - t0))
            self.result = (t_img, hands)

            if self.debug is not None:
                image = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
                for V in found:
                    for a, b in mp.solutions.hands.HAND_CONNECTIONS:
                        cv2.line(image, tuple(map(int, V[a, :2])), tuple(map(int, V[b, :2])),
                                 (255, 255, 255), 2)
                    for u, v in V[:, :2].astype(int):
                        cv2.circle(image, (u, v), 3, (0, 0, 255), -1)
                msg = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
                msg.header = rgb_msg.header
                self.debug.publish(msg)

    def _detect(self, rgb):
        """MediaPipe hands as 21 x 3 arrays (u, v, z in px)."""
        h, w = rgb.shape[:2]
        return [np.array([[p.x * w, p.y * h, p.z * w] for p in lms.landmark])
                for lms in self.hands.process(rgb).multi_hand_landmarks or []]

    def _log_time(self, ms):
        self.ms.append(ms)
        if len(self.ms) >= 150:
            self.logger.info(f'Gripper camera: {np.median(self.ms):.0f}/{np.percentile(self.ms, 95):.0f} ms '
                             f'per frame with the hand in view (median/p95)')
            self.ms = []

    def _in_view(self, t_img, R, t, w, h):
        """Last ACTIVE palm (<= 1 s) projects inside the image (25 % margin), 5-150 cm away."""
        last = None if self.target is None else self.target()
        if last is None or abs(t_img - last[0]) > 1.0:
            return False

        q = R.T @ (np.asarray(last[1]) - t)
        if not 0.05 < q[2] < 1.5:
            return False

        fx, fy, cx, cy = self.K
        u, v = fx * q[0] / q[2] + cx, fy * q[1] / q[2] + cy
        return -0.25 * w < u < 1.25 * w and -0.25 * h < v < 1.25 * h

    def _log_state(self, placed):
        # no TF for ~1 s of images (not just the start-up race): say it once
        self.missing = 0 if placed else getattr(self, 'missing', 0) + 1
        state = True if placed else (False if self.missing >= 30 else self.state)

        if state != self.state:
            self.state = state
            if state:
                self.logger.info(f'Gripper camera active (TF base -> {self.link})')
            else:
                self.logger.warn(f'Gripper camera images but no TF base -> {self.link}: not used')

    def _point(self, depth, u, v):
        h, w = depth.shape[:2]
        u, v = int(np.clip(round(u), 0, w - 1)), int(np.clip(round(v), 0, h - 1))
        patch = depth[max(0, v - 2):v + 3, max(0, u - 2):u + 3]
        patch = patch[patch > 0]
        if patch.size == 0:
            return None

        z = float(np.median(patch)) * (1e-3 if depth.dtype == np.uint16 else 1.0)
        if not 0.07 <= z <= 1.5:
            return None

        fx, fy, cx, cy = self.K
        return np.array([(u - cx) * z / fx, (v - cy) * z / fy, z])

    def hand_near(self, point, t, radius, max_age=0.1):
        """Hand whose palm (base frame) is within radius of point, from a frame <= max_age old."""
        result = self.result
        if result is None or abs(t - result[0]) > max_age:
            return None

        best = None
        for hand in result[1]:
            valid = [p for p in hand['pts'] if p is not None]
            if len(valid) < 3:
                continue
            d = float(np.linalg.norm(np.mean(valid, 0) - point))
            if d < radius and (best is None or d < best[0]):
                best = (d, hand)
        return None if best is None else best[1]

    @staticmethod
    def palm_normal(hand, right):
        """Out-of-palm normal in the base frame (MediaPipe 3D landmarks, side from the main camera)."""
        n = hand['R'] @ signed_palm_normal(hand['V'], right)
        return n / np.linalg.norm(n)
