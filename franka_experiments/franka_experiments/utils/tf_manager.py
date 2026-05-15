"""TF lookup manager: 3-level fallback + critical-link validation.

Level 1 — exact depth timestamp (temporally consistent)
Level 2 — latest available TF (tolerates TF lag at cost of small temporal error)
Level 3 — stale per-link cache (keeps robot frozen rather than dropping the frame)
Hard failure — no data at all for this link; link is absent from the result dict.

Critical links (configurable) are checked after aggregation: if any are missing
the entire frame is rejected, regardless of how many other links are available.
"""
from __future__ import annotations

import time
from typing import Dict, List, Optional, Set, Tuple

import numpy as np
from rclpy.time import Time as RclpyTime
from tf2_ros import Buffer

from franka_experiments.utils.distance_utils import get_rotation_from_quaternion


class TFManager:
    """Wraps a tf2 Buffer with fallback and per-frame validation semantics."""

    def __init__(
        self,
        tf_buffer: Buffer,
        base_frame: str,
        critical_links: List[str],
        throttle_s: float = 2.0,
        logger=None,
    ):
        self._buf      = tf_buffer
        self._base     = base_frame
        self._critical: Set[str] = set(critical_links)
        self._throttle = throttle_s
        self._log      = logger

        self._cache: Dict[str, Tuple[np.ndarray, np.ndarray]] = {}
        self._last_warn_t = 0.0

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
        transforms: Dict[str, Tuple] = {}
        failed: List[str] = []

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

    # ── Internal helpers ─────────────────────────────────────────────────────

    def _lookup_one(
        self, link_name: str, stamp
    ) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        tf_time = RclpyTime(seconds=stamp.sec, nanoseconds=stamp.nanosec)

        # Level 1 — exact timestamp
        try:
            tf = self._buf.lookup_transform(self._base, link_name, tf_time)
            R, t = _extract_Rt(tf)
            self._cache[link_name] = (R, t)
            return R, t
        except Exception:
            pass

        # Level 2 — latest available (silent: temporal mismatch is normal at high rates)
        try:
            tf = self._buf.lookup_transform(self._base, link_name, RclpyTime())
            R, t = _extract_Rt(tf)
            self._cache[link_name] = (R, t)
            return R, t
        except Exception:
            pass

        # Level 3 — stale cache
        if link_name in self._cache:
            self._warn(f'TF using stale cache for {link_name}')
            return self._cache[link_name]

        return None, None

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
