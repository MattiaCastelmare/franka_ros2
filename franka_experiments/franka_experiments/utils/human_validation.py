"""Plausibility checks on the 3D arm keypoints before they reach the Kalman filter.

Rejects MediaPipe detections that are not a human arm (objects, posters, screens):
workspace box, anthropometric segment lengths, lengths learned on the tracked person,
and a torso check required only to start tracking. Keypoint order: shoulder, elbow,
wrist, index (as human_tracker.KEYPOINT_NAMES).
"""

from collections import Counter

import numpy as np

SEGMENTS = (("upper_arm", 0, 1), ("forearm", 1, 2), ("hand", 2, 3))


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


class HumanValidator:
    """Per-frame validation of the tracked arms, with rejection counts by reason."""

    def __init__(self, cfg: dict, visibility_threshold: float):
        self.enabled = bool(cfg["enabled"])
        self.check_workspace = self.enabled and bool(cfg["check_workspace"])
        self.check_segments = self.enabled and bool(cfg["check_segments"])
        self.check_torso = self.enabled and bool(cfg["check_torso"])
        self.engage_visibility_threshold = (
            float(cfg["engage_visibility_threshold"]) if self.enabled else visibility_threshold)
        self.box_lo = np.asarray(cfg["workspace_min"], dtype=float)
        self.box_hi = np.asarray(cfg["workspace_max"], dtype=float)
        self.bands = [tuple(cfg["segment_bands"][name]) for name, _, _ in SEGMENTS]
        self.learn_frames = int(cfg["learn_frames"])
        self.learned_tol = float(cfg["learned_tolerance"])
        self.width_band = tuple(cfg["shoulder_width_band"])
        self.length_band = tuple(cfg["torso_length_band"])
        self.rejects = Counter()
        self.reset()

    def reset(self):
        """Forget the learned person (call on disengage)."""
        self._samples = {}
        self._reference = {}

    def torso_ok(self, shoulders, hips) -> bool:
        if not self.check_torso:
            return True
        ok = torso_plausible(shoulders, hips, self.width_band, self.length_band)
        if not ok:
            self.rejects["torso"] += 1
        return ok

    def filter_arm(self, side, positions, direct):
        """Copy of positions with rejected keypoints set to NaN (all of them for a bad segment)."""
        positions = np.array(positions, dtype=float)
        if self.check_workspace:
            measured = np.all(np.isfinite(positions), axis=1)
            outside = measured & ~in_box(positions, self.box_lo, self.box_hi)
            if np.any(outside):
                self.rejects["workspace"] += int(np.count_nonzero(outside))
                positions[outside] = np.nan
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

    def summary(self) -> str:
        return ", ".join(f"{k}={v}" for k, v in sorted(self.rejects.items())) or "none"
