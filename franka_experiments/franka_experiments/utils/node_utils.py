"""Shared utilities for RealTimeDistance ROS 2 nodes.

Centralises boilerplate used by real_time_distance.py:

  PerfTimer          — lightweight named-stage wall-clock profiler
  get_safety_zone()  — distance → zone label classification
  build_cp_messages()— populate MultiDistance + MultiLinkDistance from CP results
  no_obs_warn()      — throttled 'no obstacle' debug log
"""
from __future__ import annotations

import math
import time

import numpy as np

from franka_msgs.msg import (
    HumanRobotDistance, LinkDistance, MultiDistance, MultiLinkDistance)
from geometry_msgs.msg import Point, Vector3

from franka_experiments.utils.distance_engine import ControlPointResult
from franka_experiments.utils.distance_utils import find_pt_confidence


# ── Performance profiler ──────────────────────────────────────────────────────

class PerfTimer:
    """Accumulates wall-clock timings for a set of named stages.

    Usage::

        perf = PerfTimer()
        with perf('tf'):
            ...
        print(perf.summary())   # 'tf=1.2ms  mask=3.4ms'
    """

    def __init__(self):
        self._ms: dict[str, float] = {}

    def __call__(self, key: str) -> '_TimerCtx':
        return _TimerCtx(self._ms, key)

    def summary(self) -> str:
        return '  '.join(f'{k}={v:.1f}ms' for k, v in self._ms.items())


class _TimerCtx:
    """Context manager returned by PerfTimer.__call__."""

    def __init__(self, store: dict, key: str):
        self._s = store
        self._k = key

    def __enter__(self):
        self._t0 = time.perf_counter()
        return self

    def __exit__(self, *_):
        self._s[self._k] = (time.perf_counter() - self._t0) * 1000.0


# ── Zone classification ───────────────────────────────────────────────────────

def get_safety_zone(distance: float, zones: dict) -> str:
    """Return the safety-zone label for *distance* given a zones config dict.

    Parameters
    ----------
    distance:
        Distance to the nearest obstacle in metres.
    zones:
        Dict with optional keys ``'critical'``, ``'danger'``, ``'warning'``
        (all thresholds in metres).  An empty or None dict returns ``'unknown'``.

    Returns
    -------
    str
        One of ``'critical'``, ``'danger'``, ``'warning'``, ``'safe'``,
        or ``'unknown'``.
    """
    if not zones:
        return 'unknown'
    if distance <= zones.get('critical', 0.1):
        return 'critical'
    if distance <= zones.get('danger', 0.2):
        return 'danger'
    if distance <= zones.get('warning', 0.3):
        return 'warning'
    return 'safe'


# ── CP message builders ───────────────────────────────────────────────────────

def build_cp_messages(
    cp_results: list,
    n_pts: int,
    stamp,
    frame_id: str,
    segment_links: list,
    thresholds: dict,
    fallback: float,
    zones: dict,
) -> tuple:
    """Build a (MultiDistance, MultiLinkDistance) pair from CP distance results.

    Parameters
    ----------
    cp_results:
        List of :class:`ControlPointResult` produced by ``DistanceEngine.compute``.
    n_pts:
        Number of valid depth points used during the computation (for confidence).
    stamp:
        ROS timestamp to attach to all message headers.
    frame_id:
        Robot base frame id (e.g. ``'fr3_link0'``).
    segment_links:
        Ordered list of link names for the ``MultiLinkDistance`` message;
        read from ``robot_cfg['segment_links']``.
    thresholds:
        Dict with keys ``'min_thresh'`` and ``'max_thresh'`` (metres).
    fallback:
        Distance value used for invalid / out-of-range entries.
    zones:
        Safety-zone thresholds dict (forwarded to :func:`get_safety_zone`).

    Returns
    -------
    (MultiDistance, MultiLinkDistance)
        Both messages are fully populated and ready to publish.
    """
    min_thresh = thresholds['min_thresh']
    max_thresh = thresholds['max_thresh']

    # Single pass: best result per segment index and per end-link
    seg_best:  dict[int, ControlPointResult] = {}
    link_best: dict[str, ControlPointResult] = {}
    for r in cp_results:
        s = r.seg_idx
        if s not in seg_best or r.distance < seg_best[s].distance:
            seg_best[s] = r
        lk = r.end_link
        if lk not in link_best or r.distance < link_best[lk].distance:
            link_best[lk] = r

    # ── MultiDistance (one HumanRobotDistance per segment) ────────────────
    entries = []
    for r in seg_best.values():
        msg = HumanRobotDistance()
        msg.header.stamp    = stamp
        msg.header.frame_id = frame_id
        msg.robot_link_name = r.end_link
        d  = r.distance
        di = r.direction
        if math.isfinite(d) and min_thresh <= d <= max_thresh and di is not None:
            pt = r.point
            msg.valid    = True
            msg.distance = d
            msg.closest_point_robot = Point(
                x=float(pt[0]), y=float(pt[1]), z=float(pt[2]))
            msg.direction = Vector3(
                x=float(di[0]), y=float(di[1]), z=float(di[2]))
            msg.zone       = get_safety_zone(d, zones)
            msg.confidence = float(find_pt_confidence(d, n_pts))
        else:
            msg.valid    = False
            msg.distance = fallback
        entries.append(msg)

    multi_msg = MultiDistance()
    multi_msg.header.stamp    = stamp
    multi_msg.header.frame_id = frame_id
    multi_msg.distances       = entries

    # ── MultiLinkDistance (one LinkDistance per segment_link) ─────────────
    link_entries = []
    for lk in segment_links:
        if lk not in link_best:
            continue
        r   = link_best[lk]
        d   = r.distance
        di  = r.direction
        pt  = r.point
        obs = r.closest_obstacle_point
        ld  = LinkDistance()
        ld.robot_link_name = lk
        if pt is not None:
            ld.closest_point_robot = Point(
                x=float(pt[0]), y=float(pt[1]), z=float(pt[2]))
        if obs is not None:
            ld.closest_point_human = Point(
                x=float(obs[0]), y=float(obs[1]), z=float(obs[2]))
        if di is not None:
            ld.direction = Vector3(
                x=float(di[0]), y=float(di[1]), z=float(di[2]))
        ld.distance   = d
        ld.valid      = math.isfinite(d) and d > 0.0 and di is not None
        ld.confidence = 1.0
        ld.zone       = get_safety_zone(d, zones)
        link_entries.append(ld)

    mld_msg = MultiLinkDistance()
    mld_msg.header.stamp    = stamp
    mld_msg.header.frame_id = frame_id
    mld_msg.links           = link_entries

    return multi_msg, mld_msg


# ── Throttled warning ─────────────────────────────────────────────────────────

def no_obs_warn(
    logger,
    last_warn_t: float,
    throttle_s: float,
    fallback: float,
    mode: str,
) -> float:
    """Emit a throttled debug log when no valid obstacle point is found.

    Parameters
    ----------
    logger:
        ROS 2 logger obtained from ``node.get_logger()``.
    last_warn_t:
        Timestamp (``time.monotonic()``) of the previous emission.
    throttle_s:
        Minimum interval in seconds between successive log lines.
    fallback:
        Fallback distance, included in the message for diagnostic clarity.
    mode:
        Pipeline label shown in the log (e.g. ``'CP'``).

    Returns
    -------
    float
        Updated ``last_warn_t`` (advanced to *now* if the throttle elapsed,
        unchanged otherwise).
    """
    now = time.monotonic()
    if now - last_warn_t >= throttle_s:
        logger.debug(f'No near obstacle ({mode} mode). Fallback={fallback} m')
        return now
    return last_warn_t
