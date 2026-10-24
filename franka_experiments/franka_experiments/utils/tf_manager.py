"""TF lookup manager: 3-level fallback + critical-link validation.

Level 1 — exact depth timestamp (temporally consistent)
Level 2 — latest available TF (tolerates TF lag at cost of small temporal error)
Level 3 — stale per-link cache (keeps robot frozen rather than dropping the frame)
Hard failure — no data at all for this link; link is absent from the result dict.

Critical links (configurable) are checked after aggregation: if any are missing
the entire frame is rejected, regardless of how many other links are available.

``cache_max_age_s`` (optional) sets a maximum age for Level-3 cache entries.
When set, cache entries older than the threshold are discarded instead of being
returned as fallback.  When ``None`` (default) the original unlimited behaviour
is preserved for full backward compatibility.
"""
from __future__ import annotations

import time
from typing import Dict, List, Optional, Set, Tuple

import numpy as np
from rclpy.time import Time as RclpyTime
from tf2_ros import Buffer

from franka_experiments.utils.distance_utils import get_rotation_from_quaternion

# Cache entry: (R, t, wall-clock timestamp of last successful lookup)
_CacheEntry = Tuple[np.ndarray, np.ndarray, float]


class TFManager:
    """Wraps a tf2 Buffer with fallback and per-frame validation semantics."""

    def __init__(
        self,
        tf_buffer: Buffer,
        base_frame: str,
        critical_links: List[str],
        throttle_s: float = 2.0,
        cache_max_age_s: Optional[float] = None,
        logger=None,
    ):
        """
        Parameters
        ----------
        tf_buffer:
            The shared tf2 Buffer instance (from ``TransformListener``).
        base_frame:
            Root frame for all lookups (e.g. ``'fr3_link0'``).
        critical_links:
            Links whose absence causes the entire frame to be rejected.
        throttle_s:
            Minimum interval [s] between consecutive warning log lines.
        cache_max_age_s:
            Maximum age [s] for a Level-3 cache entry.  ``None`` keeps the
            original unlimited-age behaviour (backward compatible default).
        logger:
            ROS 2 logger (``node.get_logger()``).  ``None`` suppresses all logs.
        """
        self._buf      = tf_buffer
        self._base     = base_frame
        self._critical: Set[str] = set(critical_links)
        self._throttle = throttle_s
        self._max_age  = cache_max_age_s
        self._log      = logger

        self._cache: Dict[str, _CacheEntry] = {}
        self._last_warn_t = 0.0

        # ── Diagnostics for the LAST lookup_all call ────────────────────────
        # How old the poses the mask was drawn from are, relative to the DEPTH
        # FRAME they are drawn onto. Level 1 asks for the transform at the
        # frame's stamp and fails whenever /tf has not reached it yet; Level 2
        # then takes the latest available one SILENTLY, so a mask built on a
        # pose one /tf period old looks identical in the logs to a mask built
        # on the right one. At 15 Hz /tf that period is 67 ms, and an end
        # effector at 0.5 m/s moves 33 mm in it — a leading edge of the arm
        # left uncovered, which reads downstream as an obstacle at a gap of ~0.
        #: [s] worst per-link pose age of the last lookup, frame stamp minus
        #: transform stamp. Negative means the transform is AHEAD of the frame.
        self.last_pose_age_s: float = 0.0
        #: How many links resolved at each level: exact stamp / latest / cache.
        self.last_levels = [0, 0, 0]

    # ── Public API ───────────────────────────────────────────────────────────

    def lookup_all(
        self,
        link_names: List[str],
        stamp,
    ) -> Optional[Dict[str, Tuple[np.ndarray, np.ndarray]]]:
        """Look up all links; return None if the frame should be skipped.

        Rejection criteria (in order):
          - Any critical link is missing (no data at any fallback level)
          - Fewer than half of all requested links are available
        """
        transforms: Dict[str, Tuple[np.ndarray, np.ndarray]] = {}
        failed: List[str] = []
        self.last_pose_age_s = 0.0
        self.last_levels = [0, 0, 0]

        for name in link_names:
            R, t = self._lookup_one(name, stamp)
            if R is not None:
                transforms[name] = (R, t)
            else:
                failed.append(name)

        if failed:
            self._warn(
                f'TF unavailable for: {failed} — '
                f'{len(transforms)}/{len(link_names)} links available')

        missing_critical = [lk for lk in self._critical if lk not in transforms]
        if missing_critical:
            self._warn(f'Critical links missing: {missing_critical} — skipping frame')
            return None

        if len(transforms) < len(link_names) / 2:
            self._warn(
                f'Too many TF failures ({len(failed)}/{len(link_names)}) — skipping frame')
            return None

        return transforms

    def lookup_best_effort(
        self,
        link_names: List[str],
        stamp,
    ) -> Dict[str, Tuple[np.ndarray, np.ndarray]]:
        """Look up all links, returning whatever is available — never rejects.

        Unlike :meth:`lookup_all`, critical-link checks and the 50 % threshold
        are bypassed.  A partial skeleton is always returned; missing links are
        simply absent from the dict.

        Intended for best-effort applications (e.g. visualization) where a
        partial robot skeleton is acceptable and blocking is undesirable.

        Returns
        -------
        dict
            Mapping ``link_name → (R, t)`` for every successfully resolved
            link.  May be empty if no TF data is available at all.
        """
        transforms: Dict[str, Tuple[np.ndarray, np.ndarray]] = {}
        for name in link_names:
            R, t = self._lookup_one(name, stamp)
            if R is not None:
                transforms[name] = (R, t)
        return transforms

    # ── Internal helpers ─────────────────────────────────────────────────────

    def _lookup_one(
        self, link_name: str, stamp
    ) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        tf_time = RclpyTime(seconds=stamp.sec, nanoseconds=stamp.nanosec)
        now     = time.monotonic()

        # Level 1 — exact timestamp
        try:
            tf = self._buf.lookup_transform(self._base, link_name, tf_time)
            R, t = _extract_Rt(tf)
            self._cache[link_name] = (R, t, now)
            self._note_pose_age(stamp, tf.header.stamp, 0)
            return R, t
        except Exception:
            pass

        # Level 2 — latest available (silent: temporal mismatch is normal at high rates)
        try:
            tf = self._buf.lookup_transform(self._base, link_name, RclpyTime())
            R, t = _extract_Rt(tf)
            self._cache[link_name] = (R, t, now)
            self._note_pose_age(stamp, tf.header.stamp, 1)
            return R, t
        except Exception:
            pass

        # Level 3 — stale cache (subject to optional age limit)
        if link_name in self._cache:
            R, t, ts = self._cache[link_name]
            age = now - ts
            if self._max_age is None or age <= self._max_age:
                self._warn(f'TF using stale cache for {link_name} (age={age:.2f}s)')
                self.last_levels[2] += 1
                self.last_pose_age_s = max(self.last_pose_age_s, age)
                return R, t
            # Cache entry is too old — treat as hard failure
            self._warn(
                f'TF cache expired for {link_name} '
                f'(age={age:.2f}s > max={self._max_age:.2f}s)')

        return None, None

    def _note_pose_age(self, frame_stamp, tf_stamp, level: int) -> None:
        """Record how far behind the frame the pose actually is.

        Kept as a MAXIMUM over the links of one lookup: the mask is only as
        current as its worst link, and one lagging link is one uncovered limb.
        """
        self.last_levels[level] += 1
        age = ((frame_stamp.sec - tf_stamp.sec)
               + (frame_stamp.nanosec - tf_stamp.nanosec) * 1e-9)
        if age > self.last_pose_age_s:
            self.last_pose_age_s = age

    def _warn(self, msg: str):
        now = time.monotonic()
        if now - self._last_warn_t >= self._throttle:
            self._last_warn_t = now
            if self._log:
                self._log.warn(msg)


def _extract_Rt(tf) -> Tuple[np.ndarray, np.ndarray]:
    t = tf.transform.translation
    q = tf.transform.rotation
    R = get_rotation_from_quaternion(q)
    t_vec = np.array([t.x, t.y, t.z], dtype=np.float64)
    return R, t_vec
