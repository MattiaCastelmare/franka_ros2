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
    HANDS = (('left_hand_landmarks', 91, 9), ('right_hand_landmarks', 112, 10))  # key, first kp, body wrist

    def __init__(self, mode='lightweight', det_frequency=10, enter=0.8, keep=0.5):
        cuda, t0 = ctypes.CDLL('libcuda.so.1'), time.time()
        while cuda.cuInit(0) != 0 and time.time() - t0 < 10.0:  # flaky first cuInit
            time.sleep(0.2)
        import torch  # noqa: F401  loads the CUDA / cuDNN libraries onnxruntime needs
        from rtmlib import PoseTracker, Wholebody
        # person detector every det_frequency frames, boxes from the last keypoints in between
        self.model = PoseTracker(Wholebody, det_frequency=det_frequency, mode=mode,
                                 tracking=False, backend='onnxruntime', device='cuda')
        self.enter, self.keep = enter, keep
        self.kept, self.centre = set(), None
        self.filters = {key: OneEuro() for key, _, _ in self.HANDS}

    def process(self, bgr, depth, depth_scale, fx, t):
        out = SimpleNamespace(pose_landmarks=None, left_hand_landmarks=None, right_hand_landmarks=None)
        keypoints, scores = self.model(bgr)
        h, w = bgr.shape[:2]
        person = self._person(keypoints, scores, w)
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
        for key, first, wrist in self.HANDS:
            p, c = k[first:first + 21], s[first:first + 21]
            score = float(c.mean())
            if score < (self.keep if key in self.kept else self.enter):
                continue
            box = np.r_[p.min(0), p.max(0)]
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

    def _person(self, keypoints, scores, w):
        """One person, as Holistic: the one tracked so far, else the largest."""
        people = []
        for i, (k, s) in enumerate(zip(keypoints, scores)):
            pts = k[:17][s[:17] > 0.3]
            if len(pts) >= 4:
                people.append((i, pts.mean(0), np.ptp(pts[:, 0]) * np.ptp(pts[:, 1])))
        if not people:
            self.centre = None
            return None
        near = [] if self.centre is None else [p for p in people if np.linalg.norm(p[1] - self.centre) < 0.15 * w]
        i, self.centre, _ = (min(near, key=lambda p: np.linalg.norm(p[1] - self.centre)) if near
                             else max(people, key=lambda p: p[2]))
        return i

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
