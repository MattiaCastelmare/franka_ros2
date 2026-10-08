#!/usr/bin/env python3
"""RTMW whole-body front-end with the MediaPipe Holistic interface.

RTMW (Jiang et al. 2024) through rtmlib (Apache-2.0), ONNX on the GPU. The
tracker uses it in place of Holistic: everything after the front-end is the
same (re-detection excluded, RTMW already detects every hand of the body).

Keypoint names (COCO-WholeBody 133):
  hands 91-111 (left) / 112-132 (right) = MediaPipe hand order:
    0 wrist (hand_root), 1-4 thumb, 5 index MCP (forefinger1), 9 middle MCP,
    13 ring MCP, 17 pinky MCP  ->  LANDMARK_IDS (0, 5, 9, 17) unchanged;
  body 5/6 shoulders, 7/8 elbows, 9/10 wrists, 11/12 hips -> MediaPipe Pose
  11/12, 13/14, 15/16, 23/24 (the indices the tracker reads).
Left / right are the subject's, as in Holistic.

RTMW gives no hand z: the tracker uses palm_normal_method depth. Hand
keypoints are smoothed in time (OneEuro) as Holistic does with its landmarks.

Ghost filters, offline on 6 bags against Hands23 (recall 0.905 vs 0.800 of
Holistic; the remaining extra detections are mostly real occluded hands):
  - hand score hysteresis: enter >= 0.8, keep >= 0.5;
  - left and right boxes on the same hand: keep the higher score;
  - metric hand size (box side at the wrist depth) within 5-30 cm.
"""

import ctypes
import threading
import time
from types import SimpleNamespace

import numpy as np
from mediapipe.framework.formats import landmark_pb2


class OneEuro:
    """One-Euro filter (Casiez et al. 2012) on hand keypoints, speed in hand sizes/s.

    Holistic smooths its landmarks internally; RTMW keypoints are per frame.
    Tuned offline (6 bags): MCP jitter 0.035 -> 0.020 hand sizes (Holistic
    0.023), lag ~0.02 hand sizes."""

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


class RtmwHolistic:

    POSE_FROM_BODY = {0: 0, 5: 11, 6: 12, 7: 13, 8: 14, 9: 15, 10: 16, 11: 23, 12: 24}
    HANDS = (('left_hand_landmarks', 91, 9, 7), ('right_hand_landmarks', 112, 10, 8))  # key, first kp, body wrist, elbow
    # Body cues per mode, tuned on the Hands23 reference of all bags (palm_eval_tmp/body_grid.py):
    #   wrist_min: ghost hands come with an unsure body wrist (score 0.6 vs 0.86 on real hands)
    #   ratio_max: forearm length / hand size above this = hand far too small for its arm
    #   reacquire: a new hand sitting on a confident body wrist starts at this score (not enter)
    BODY_CUES = {'lightweight': {'wrist_min': 0.55, 'ratio_max': 3.0, 'reacquire': 0.65},
                 'balanced': {'wrist_min': 0.6, 'ratio_max': None, 'reacquire': None}}
    SWITCH_M = 0.3  # another person takes over when this much nearer the robot
    HUMAN_MIN = 0.7  # mean face + best hand keypoint score; the robot arm seen as a person stays below
    SWITCH_FRAMES = 5  # frames in a row a nearer person must be seen before taking over

    def __init__(self, mode='lightweight', det_frequency=10, enter=0.8, keep=0.5, tensorrt=True, body_cues=True,
                 to_base=None):
        cuda, t0 = ctypes.CDLL('libcuda.so.1'), time.time()
        while cuda.cuInit(0) != 0 and time.time() - t0 < 10.0:  # flaky first cuInit
            time.sleep(0.2)
        import torch  # noqa: F401  loads the CUDA / cuDNN libraries onnxruntime needs
        from rtmlib import Wholebody
        from rtmlib.tools.solution.pose_tracker import pose_to_bbox
        # As rtmlib's PoseTracker (person boxes from the last keypoints, refreshed by the
        # person detector every det_frequency frames), but the detector runs in its own
        # thread: no 10 ms spike every det_frequency frames on the frame path.
        model = Wholebody(mode=mode, backend='onnxruntime', device='cuda')
        self.engine = 'CUDA FP32'
        if tensorrt:
            self.engine = self._tensorrt(model)
        self.detector, self.pose, self.pose_to_bbox = model.det_model, model.pose_model, pose_to_bbox
        self.det_frequency, self.n, self.boxes = det_frequency, 0, []
        self.det_in, self.det_out = None, None
        self.det_wake, self.det_stop = threading.Event(), threading.Event()
        self.det_thread = threading.Thread(target=self._detect_loop, daemon=True)
        self.det_thread.start()
        self.enter, self.keep = enter, keep
        self.body = self.BODY_CUES.get(mode, {}) if body_cues else {}
        self.kept, self.centre = set(), None
        self.pending, self.pending_n = None, 0  # nearer person waiting to take over
        self.to_base = to_base  # (u, v, z) -> robot base frame point; None = largest person
        self.filters = {key: OneEuro() for key, *_ in self.HANDS}

    @staticmethod
    def _tensorrt(model):
        """Same ONNX models compiled by TensorRT in FP16 (RTX 3050: pose 16.5 -> 2.0 ms,
        YOLOX-tiny 8.1 -> 1.6 ms, outputs within ~1 %). Engines cached on disk: the first
        start on a new GPU builds them (~2 min). YOLOX-m (balanced detector) does not build
        with TensorRT (TopK) and stays on CUDA, in its own thread anyway."""
        import os
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
        k, s = keypoints[person], np.clip(scores[person], 0.0, 1.0)
        pose = landmark_pb2.NormalizedLandmarkList()
        for _ in range(33):
            pose.landmark.add(visibility=0.0)
        for body, mp_id in self.POSE_FROM_BODY.items():
            q = pose.landmark[mp_id]
            q.x, q.y, q.visibility = k[body, 0] / w, k[body, 1] / h, s[body]
        out.pose_landmarks = pose
        hands = {}
        body = self.body
        for key, first, wrist, elbow in self.HANDS:
            p, c = k[first:first + 21], s[first:first + 21]
            score = float(c.mean())
            box = np.r_[p.min(0), p.max(0)]
            size = max(box[2] - box[0], box[3] - box[1], 10.0)
            threshold = self.keep if key in self.kept else self.enter
            if (body.get('reacquire') and key not in self.kept and s[wrist] >= 0.5
                    and np.linalg.norm(p[0] - k[wrist]) <= 0.5 * size):
                threshold = body['reacquire']
            if score < threshold:
                continue
            if body.get('wrist_min') and s[wrist] < body['wrist_min']:
                continue
            if (body.get('ratio_max') and s[elbow] >= 0.3
                    and np.linalg.norm(k[wrist] - k[elbow]) / size > body['ratio_max']):
                continue
            z = self._depth(depth, depth_scale, k[wrist])
            if z is not None and not 0.05 <= max(box[2] - box[0], box[3] - box[1]) * z / fx <= 0.30:
                continue
            hands[key] = (score, p, c, box)
        if len(hands) == 2 and self._iou(*(v[3] for v in hands.values())) > 0.3:
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
        """One person, as Holistic: the one tracked so far, else the largest.

        With the robot geometry (to_base) the person interacting with the robot: the one
        whose torso is nearest the robot base. Someone else takes over only when nearer
        by SWITCH_M for SWITCH_FRAMES frames, so a person sitting at a desk behind does not
        keep the hands. Only people with face and hands (HUMAN_MIN) count, not the robot arm."""
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
        near = [] if self.centre is None else [p for p in people if np.linalg.norm(p[1] - self.centre) < 0.15 * w]
        current = min(near, key=lambda p: np.linalg.norm(p[1] - self.centre)) if near else None
        # only people take part: RTMW also sees the robot arm as a "person" whose torso sits
        # on the robot base, so it would always be the nearest (face and hands score low)
        located = [p for p in people if p[3] is not None and p[4] >= self.HUMAN_MIN]
        nearest = min(located, key=lambda p: p[3]) if located else None
        if nearest is not None and current is None:
            current = nearest
        elif nearest is not None and nearest[0] != current[0] and (
                current[3] is None or current[4] < self.HUMAN_MIN or nearest[3] < current[3] - self.SWITCH_M):
            # takes over only if seen nearer for SWITCH_FRAMES frames in a row
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
            z = self._depth(depth, depth_scale, k[j]) if s[j] > 0.3 else None
            if z is not None and 0.1 < z < 6.0:
                d.append(float(np.linalg.norm(self.to_base(k[j, 0], k[j, 1], z)[:2])))
        return min(d) if d else None

    @staticmethod
    def _depth(depth, scale, uv):
        h, w = depth.shape[:2]
        u, v = int(np.clip(uv[0], 0, w - 1)), int(np.clip(uv[1], 0, h - 1))
        patch = depth[max(0, v - 3):v + 4, max(0, u - 3):u + 4]
        patch = patch[patch > 0]
        return float(np.median(patch)) * scale if patch.size > 5 else None

    @staticmethod
    def _iou(a, b):
        x0, y0, x1, y1 = max(a[0], b[0]), max(a[1], b[1]), min(a[2], b[2]), min(a[3], b[3])
        inter = max(0.0, x1 - x0) * max(0.0, y1 - y0)
        union = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
        return inter / union if union > 0 else 0.0
