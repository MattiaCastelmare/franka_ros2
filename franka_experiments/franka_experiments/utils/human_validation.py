"""Plausibility checks on the 3D arm keypoints before they reach the Kalman filter.

Rejects MediaPipe detections that are not a human arm. MediaPipe reports a full, confident
body (visibility and segmentation ~1.0) on robots and other static objects, so its own
scores cannot tell them apart; the checks here use the scene instead:
- identity of the arm, on its shoulder (else elbow): a person is IN FRONT of the static
  scene (DepthBackground) and never inside the FR3's body (robot_distance);
- reach: an elbow/wrist/index sampled on the static scene or on the robot, out of reach
  of its own shoulder, is the object seen where the real arm is hidden (a person walking
  behind a robot);
- occlusion: a keypoint nearer than its arm neighbour by more than the segment between
  them can span, lying on the robot or the static scene, reads the depth of what is in
  front of the arm (occluded), and is placed again with a depth borrowed from the arm;
- workspace box, anthropometric segment lengths, lengths learned on the tracked person,
  and a torso check required only to start tracking.
Keypoint order: shoulder, elbow, wrist, index (as human_tracker.KEYPOINT_NAMES).
"""

from collections import Counter
from itertools import combinations

import numpy as np

from franka_experiments.utils.capsule_geometry import point_to_segment_distance

SEGMENTS = (("upper_arm", 0, 1), ("forearm", 1, 2), ("hand", 2, 3))
SHOULDER, ELBOW, WRIST, INDEX = 0, 1, 2, 3


def in_box(points, lo, hi):
    """(N,) True where a point is finite and inside the axis-aligned box [lo, hi]."""
    points = np.asarray(points, dtype=float)
    return np.all(np.isfinite(points), axis=1) & np.all((points >= lo) & (points <= hi), axis=1)


def segment_lengths(points, direct):
    """Lengths of SEGMENTS; NaN unless both ends are finite and have their own depth."""
    points = np.asarray(points, dtype=float)
    lengths = np.full(len(SEGMENTS), np.nan)
    for k, (_, a, b) in enumerate(SEGMENTS):
        if direct[a] and direct[b] and np.all(np.isfinite(points[[a, b]])):
            lengths[k] = float(np.linalg.norm(points[b] - points[a]))
    return lengths


def outside_bands(lengths, bands):
    """True if a measured length falls outside its [min, max] band."""
    return any(np.isfinite(l) and not (lo <= l <= hi) for l, (lo, hi) in zip(lengths, bands))


def deviates(lengths, reference, tol):
    """True if a measured length differs from its learned reference by more than tol (relative)."""
    return any(np.isfinite(l) and np.isfinite(r) and abs(l - r) > tol * r
               for l, r in zip(lengths, reference))


def torso_plausible(shoulders, hips, width_band, length_band):
    """Both shoulders at a human width; if both hips are known, below the shoulders at a human length."""
    shoulders = np.asarray(shoulders, dtype=float)
    if not np.all(np.isfinite(shoulders)):
        return False
    width = np.linalg.norm(shoulders[0] - shoulders[1])
    if not (width_band[0] <= width <= width_band[1]):
        return False
    hips = None if hips is None else np.asarray(hips, dtype=float)
    if hips is None or not np.all(np.isfinite(hips)):
        return True
    mid_sh, mid_hip = shoulders.mean(axis=0), hips.mean(axis=0)
    length = np.linalg.norm(mid_sh - mid_hip)
    return mid_sh[2] > mid_hip[2] and length_band[0] <= length <= length_band[1]


def robot_distance(point, nodes):
    """Distance from a point to the robot's kinematic polyline (link origins, base to tip)."""
    return min(point_to_segment_distance(point, a, b)[0] for a, b in zip(nodes[:-1], nodes[1:]))


def anchor_keypoint(positions, direct):
    """Index of the keypoint that carries the arm's identity: shoulder, else elbow, else None.

    Only a keypoint with its own depth counts: a borrowed depth says nothing about what
    surface sits at that pixel.
    """
    for i in (SHOULDER, ELBOW):
        if direct[i] and np.all(np.isfinite(positions[i])):
            return i
    return None


def to_meters(depth):
    """Aligned depth image (uint16 mm or float m) as float32 meters."""
    depth = np.asarray(depth)
    if depth.dtype == np.uint16:
        return depth.astype(np.float32) * 0.001
    return depth.astype(np.float32)


class DepthBackground:
    """Depth of the static scene per image cell, learned online from the aligned depth.

    A person is an object in FRONT of the static scene; a robot that is switched off, or
    the FR3 at rest, IS the static scene. Each cell keeps the farthest surface seen there:
    it moves back quickly when whatever covered it leaves (reveal), and creeps forward
    only slowly (absorb_tau_s, an object placed and left there) and never under the
    tracked person (protect). The range is the sensor's (max_depth_m), wider than the
    tracker's: a wall 4 m away must be learned, or the first person standing in front of
    it would be taken for it. A cell first seen after the warm-up starts infinitely far,
    so whatever appears there is foreground until absorbed.
    A person standing in the image since startup is background until they move: tracking
    can start only once the scene behind them has been seen.
    """

    def __init__(self, cfg: dict, min_depth_m: float):
        self.cell = int(cfg["cell_px"])
        self.margin = float(cfg["foreground_margin_m"])
        self.reveal_tol = float(cfg["reveal_tolerance_m"])
        self.reveal_rate = float(cfg["reveal_rate"])
        self.absorb_tau = float(cfg["absorb_tau_s"])
        self.warmup = int(cfg["warmup_frames"])
        self.min_depth, self.max_depth = float(min_depth_m), float(cfg["max_depth_m"])
        self.reset()

    def reset(self):
        self.bg = None
        self.frames = 0
        self._last_t = None
        self._unseen_s = None

    def cells(self, depth):
        """(h, w) robust depth of each cell [m], NaN where too few valid pixels."""
        c = self.cell
        h, w = depth.shape[0] // c, depth.shape[1] // c
        # Every other pixel of each cell is enough for a median, at a quarter of the cost
        s = to_meters(np.asarray(depth)[:h * c, :w * c].reshape(h, c, w, c)[:, ::2, :, ::2])
        s = s.transpose(0, 2, 1, 3).reshape(h, w, -1)
        s = np.where((s >= self.min_depth) & (s <= self.max_depth), s, np.inf)
        s.sort(axis=2)
        n = np.isfinite(s).sum(axis=2)
        med = np.take_along_axis(s, np.maximum((n - 1) // 2, 0)[..., None], axis=2)[..., 0]
        med[n < s.shape[2] // 4] = np.nan
        return med

    def update(self, depth, t, protect=()):
        """Fold one depth frame in. protect: (u, v, radius_px) circles kept from absorption."""
        d = self.cells(depth)
        dt = 0.0 if self._last_t is None else float(np.clip(t - self._last_t, 0.0, 0.2))
        self._last_t = t
        self.frames += 1
        valid = np.isfinite(d)
        if self.bg is None:
            self.bg = d.copy()
            self._unseen_s = np.zeros(d.shape)
        if self.frames <= self.warmup:
            # The startup scene is the background; afterwards an unseen cell is "far away"
            new = valid & np.isnan(self.bg)
            self.bg[new] = d[new]
            if self.frames == self.warmup:
                self.bg[np.isnan(self.bg)] = np.inf
            return

        keep = np.zeros(d.shape, dtype=bool)
        if protect:
            rows, cols = np.ogrid[:d.shape[0], :d.shape[1]]
            for u, v, r in protect:
                r_cells = r / self.cell + 1.0
                keep |= (cols - u / self.cell) ** 2 + (rows - v / self.cell) ** 2 <= r_cells ** 2

        known = valid & np.isfinite(self.bg)
        diff = np.where(known, d - self.bg, 0.0)
        far = diff > self.reveal_tol                       # something in front has left
        self.bg[far] += self.reveal_rate * diff[far]
        same = known & (np.abs(diff) <= self.reveal_tol) & ~keep   # noise on the same surface
        self.bg[same] += 0.2 * diff[same]
        if self.absorb_tau <= 0.0:
            return
        near = (diff < -self.reveal_tol) & ~keep           # a new object, absorbed slowly
        self.bg[near] += min(1.0, dt / self.absorb_tau) * diff[near]
        # An unseen cell joins the scene once something has stayed there absorb_tau_s
        unseen = valid & np.isinf(self.bg) & ~keep
        self._unseen_s = np.where(unseen, self._unseen_s + dt, 0.0)
        settled = self._unseen_s >= self.absorb_tau
        self.bg[settled] = d[settled]

    def foreground(self, u, v, depth_m):
        """True if a surface at depth_m seen at pixel (u, v) is in front of the static scene.

        None until the model has warmed up. The nearest background in the 3x3 cells around
        the pixel is used: at the border of a static object it is the object itself, so a
        keypoint on its edge is not taken for a person.
        """
        if self.bg is None or self.frames < self.warmup:
            return None
        r, c = int(v) // self.cell, int(u) // self.cell
        patch = self.bg[max(0, r - 1):r + 2, max(0, c - 1):c + 2]
        if patch.size == 0 or np.all(np.isnan(patch)):
            return None
        return bool(depth_m < np.nanmin(patch) - self.margin)


class HumanValidator:
    """Per-frame validation of the tracked arms, with rejection counts by reason."""

    def __init__(self, cfg: dict, visibility_threshold: float):
        self.enabled = bool(cfg["enabled"])
        self.check_workspace = self.enabled and bool(cfg["check_workspace"])
        self.check_segments = self.enabled and bool(cfg["check_segments"])
        self.check_torso = self.enabled and bool(cfg["check_torso"])
        self.check_background = self.enabled and bool(cfg["check_background"])
        self.check_robot = self.enabled and bool(cfg["check_robot"])
        self.engage_visibility_threshold = (
            float(cfg["engage_visibility_threshold"]) if self.enabled else visibility_threshold)
        self.box_lo = np.asarray(cfg["workspace_min"], dtype=float)
        self.box_hi = np.asarray(cfg["workspace_max"], dtype=float)
        self.robot_clearance = float(cfg["robot_clearance_m"])
        self.max_reach = {ELBOW: float(cfg["max_reach_m"]["elbow"]),
                          WRIST: float(cfg["max_reach_m"]["wrist"]),
                          INDEX: float(cfg["max_reach_m"]["index"])}
        self.shoulder_memory_s = float(cfg["shoulder_memory_s"])
        self.check_occlusion = self.enabled and bool(cfg["check_occlusion"])
        self.occlusion_margin = float(cfg["occlusion_margin_m"])
        self.occlusion_clearance = float(cfg["occlusion_robot_clearance_m"])
        self.bands = [tuple(cfg["segment_bands"][name]) for name, _, _ in SEGMENTS]
        self.learn_frames = int(cfg["learn_frames"])
        self.learned_tol = float(cfg["learned_tolerance"])
        self.width_band = tuple(cfg["shoulder_width_band"])
        self.length_band = tuple(cfg["torso_length_band"])
        self.rejects = Counter()
        self._reported = Counter()
        # Per side, outcome of the last identity check: None (no shoulder/elbow with its
        # own depth, undecided), "person", or the rejection reason
        self.identity = {}
        self.reset()

    def reset(self):
        """Forget the learned person (call on disengage)."""
        self._samples = {}
        self._reference = {}
        self._last_shoulder = {}

    def recent_shoulder(self, side, t):
        """Last accepted shoulder of this side, if at most shoulder_memory_s old at time t."""
        last = self._last_shoulder.get(side)
        if last is None or t is None or not 0.0 <= t - last[1] <= self.shoulder_memory_s:
            return None
        return last[0]

    def torso_ok(self, shoulders, hips) -> bool:
        if not self.check_torso:
            return True
        ok = torso_plausible(shoulders, hips, self.width_band, self.length_band)
        if not ok:
            self.rejects["torso"] += 1
        return ok

    def identity_reject(self, positions, direct, foreground=None, robot_nodes=None):
        """Reason the arm is not a person ('background', 'on_robot'), or None.

        Decided on the shoulder (else elbow) alone: a real hand may rest on a table or
        touch the robot, a real shoulder is never part of the static scene or inside the
        FR3. foreground: per-keypoint True/False/None from DepthBackground.foreground.
        robot_nodes: (K, 3) link origins in the base frame, or None if unknown.
        """
        i = anchor_keypoint(positions, direct)
        if i is None:
            return None
        if self.check_background and foreground is not None and foreground[i] is False:
            return "background"
        if (self.check_robot and robot_nodes is not None and len(robot_nodes) > 1
                and robot_distance(positions[i], robot_nodes) < self.robot_clearance):
            return "on_robot"
        return None

    def on_scene_or_robot(self, point, direct, foreground, robot_nodes, clearance=None):
        """True if a keypoint lies on the static scene (own depth only) or on the robot.

        clearance: distance from the link polyline that counts as on the robot
        (robot_clearance_m if None).
        """
        clearance = self.robot_clearance if clearance is None else clearance
        on_scene = self.check_background and direct and foreground is False
        on_robot = (self.check_robot and robot_nodes is not None and len(robot_nodes) > 1
                    and robot_distance(point, robot_nodes) < clearance)
        return on_scene or on_robot

    def distal_rejects(self, positions, direct, foreground=None, robot_nodes=None, shoulder_ref=None,
                       points=(WRIST, INDEX)):
        """Indices among points to drop: on the static scene or on the robot, out of reach.

        A real hand or elbow may rest on a table or touch the robot, always within reach of
        its own shoulder. When MediaPipe puts the wrist on a robot the person walks behind,
        the depth there is the robot's: far from the shoulder. shoulder_ref is the shoulder to
        measure the reach from (this frame's, else the last accepted one); without it such a
        point cannot be vouched for and is dropped.
        """
        drop = []
        for i in points:
            p = positions[i]
            if not np.all(np.isfinite(p)) or not self.on_scene_or_robot(
                    p, direct[i], None if foreground is None else foreground[i], robot_nodes):
                continue
            if shoulder_ref is None or np.linalg.norm(p - shoulder_ref) > self.max_reach[i]:
                drop.append(i)
        return drop

    def occluded(self, positions, depths, direct, foreground=None, robot_nodes=None):
        """Indices of keypoints whose own depth is that of something in front of the arm.

        Two keypoints of the arm cannot differ in depth by more than the segments between
        them span (their anthropometric maxima, plus occlusion_margin_m of noise). Every pair
        with its own depth is compared, so that a neighbour with a borrowed depth in between
        does not hide the jump. When the bound is broken one reading is wrong; an occluder is
        always in front, so the nearer one is the suspect, and it is taken as occluded when
        it lies on the robot or the static scene. The farther one being wrong (depth through
        a gap) is left alone: pushing the nearer one back would move the arm away from the
        robot.
        """
        if not self.check_occlusion:
            return []
        measured = [i for i in range(len(depths)) if direct[i] and depths[i] > 0.0]
        occluded = []
        for a, b in combinations(measured, 2):
            span = sum(self.bands[k][1] for k in range(a, b)) + self.occlusion_margin
            if abs(depths[a] - depths[b]) <= span:
                continue
            near = a if depths[a] < depths[b] else b
            # The polyline runs through the link origins, not the surface (the elbow casing
            # sticks out ~0.2 m): a wider clearance, the impossible jump being the evidence
            if near not in occluded and self.on_scene_or_robot(
                    positions[near], True, None if foreground is None else foreground[near],
                    robot_nodes, self.occlusion_clearance):
                occluded.append(near)
        if occluded:
            self.rejects["occluded_depth"] += len(occluded)
        return occluded

    @staticmethod
    def engageable(positions, direct):
        """Tracking may start on an arm only with a measured shoulder and elbow (own depth)."""
        return all(direct[i] and np.all(np.isfinite(positions[i])) for i in (SHOULDER, ELBOW))

    def filter_arm(self, side, positions, direct, foreground=None, robot_nodes=None, learn=True,
                   t=None):
        """Copy of positions with rejected keypoints set to NaN (all of them for a bad arm).

        t: time of the frame [s]. The accepted shoulder is remembered with it, so that the
        reach of a hand seen without its shoulder is measured from the last one (up to
        shoulder_memory_s before).
        """
        positions = np.array(positions, dtype=float)
        if self.check_workspace:
            measured = np.all(np.isfinite(positions), axis=1)
            outside = measured & ~in_box(positions, self.box_lo, self.box_hi)
            if np.any(outside):
                self.rejects["workspace"] += int(np.count_nonzero(outside))
                positions[outside] = np.nan

        shoulder_measured = bool(direct[SHOULDER]) and np.all(np.isfinite(positions[SHOULDER]))
        shoulder_ref = (positions[SHOULDER].copy() if shoulder_measured
                        else self.recent_shoulder(side, t))
        if shoulder_measured or shoulder_ref is None:
            reason = self.identity_reject(positions, direct, foreground, robot_nodes)
            decided = anchor_keypoint(positions, direct) is not None
        else:
            # Shoulder hidden, but a person's shoulder was accepted a moment ago: the arm is
            # still that person, and its elbow is checked by reach like the hand, so an elbow
            # resting on the robot or a table no longer rejects the whole arm
            reason, decided = None, True
        self.identity[side] = reason or ("person" if decided else None)
        if reason is not None:
            self.rejects[reason] += 1
            return np.full_like(positions, np.nan)

        if shoulder_measured and t is not None:
            self._last_shoulder[side] = (shoulder_ref, t)
        # Without any shoulder the elbow carried the identity: only the hand is left to check
        points = (WRIST, INDEX) if shoulder_ref is None else (ELBOW, WRIST, INDEX)
        drop = self.distal_rejects(positions, direct, foreground, robot_nodes, shoulder_ref, points)
        if drop:
            self.rejects["out_of_reach"] += len(drop)
            positions[drop] = np.nan
        if not self.check_segments:
            return positions

        lengths = segment_lengths(positions, direct)
        if outside_bands(lengths, self.bands):
            self.rejects["segment_band"] += 1
            return np.full_like(positions, np.nan)
        reference = self._reference.get(side, np.full(len(SEGMENTS), np.nan))
        if deviates(lengths, reference, self.learned_tol):
            self.rejects["learned_length"] += 1
            return np.full_like(positions, np.nan)

        if learn:
            self._learn(side, lengths)
        return positions

    def _learn(self, side, lengths):
        samples = self._samples.setdefault(side, [[] for _ in SEGMENTS])
        reference = self._reference.setdefault(side, np.full(len(SEGMENTS), np.nan))
        for k, length in enumerate(lengths):
            if np.isfinite(length) and not np.isfinite(reference[k]):
                samples[k].append(length)
                if len(samples[k]) >= self.learn_frames:
                    reference[k] = float(np.median(samples[k]))

    def not_a_person(self, sides):
        """Reason the pose MediaPipe tracks this frame is not a person, or None.

        Set when no arm is a person and at least one was rejected on its identity: the
        whole skeleton sits on the static scene or on the robot.
        """
        outcomes = [self.identity.get(side) for side in sides]
        if "person" in outcomes:
            return None
        return next((o for o in outcomes if o is not None), None)

    def summary(self) -> str:
        return ", ".join(f"{k}={v}" for k, v in sorted(self.rejects.items())) or "none"

    def new_rejects(self) -> str:
        """Rejections since the previous call ('' if none): the log shows what is happening now."""
        delta = self.rejects - self._reported
        self._reported = Counter(self.rejects)
        return ", ".join(f"{k}={v}" for k, v in sorted(delta.items()))