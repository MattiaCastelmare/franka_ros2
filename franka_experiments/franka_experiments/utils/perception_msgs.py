"""Construction and interpretation of the distance messages.

OWNS
----
Everything that turns ``DistanceEngine`` control-point results into the
ROS messages the CBF chain consumes, and the scalar classifications that go
inside them:

* :func:`build_cp_messages`  — (MultiDistance, MultiLinkDistance) pair
* :func:`annotate_track_fields` — fills the obstacle-track fields in place
* :func:`get_safety_zone`    — distance → zone label
* :func:`find_pt_confidence` — distance + pixel count → confidence scalar
* :func:`no_obs_warn`        — throttled "no obstacle" debug log

DOES NOT OWN
------------
* Computing the distances themselves — that is ``utils.distance_engine``.
* Geometry / TF / control-point definition — that is ``utils.distance_utils``.
* Publishing; callers own their publishers.

Message shape
-------------
``MultiDistance`` carries one entry per SEGMENT (argmin over that segment's
control points) and is the legacy topic, consumed by the visualiser and the
logger.  ``MultiLinkDistance`` — the topic the CBF chain reads — carries one
entry per CONTROL POINT, so ``robot_link_name`` repeats across the CPs of a
link and the filter builds one HOCBF row for each.  It used to be pooled to
argmin per link too, which hid 6 of the 11 CPs from the QP entirely.

``LinkDistance.distance`` is the surface gap: capsule radius and mask-dilation
margin already subtracted by ``DistanceEngine``, clamped at 0.  A gap of
exactly 0.0 means "at the capsule surface" and is published ``valid`` — it is a
measurement, not a dropout.

Hot-path note: :func:`build_cp_messages` runs once per camera frame (~30 Hz),
not on the 100 Hz control loop.  Per-CP publishing raised the LinkDistance
allocation count from ~5 to ~11 per frame, which is immaterial at 30 Hz but is
the reason the QP's ``n_c`` now ranges over 11 values instead of 5 (see
``cbf_obstacle_horizon`` in fr3_control.yaml for why that matters to OSQP).
"""
from __future__ import annotations

import math
import time

from typing import Any

import numpy as np

from franka_msgs.msg import (
    HumanRobotDistance, LinkDistance, MultiDistance, MultiLinkDistance)
from geometry_msgs.msg import Point, Vector3

from franka_experiments.utils.distance_engine import ControlPointResult


def find_pt_confidence(best_result: float, n_pts: int) -> float:
    """Compute confidence score based on valid point count and distance value."""
    lm_conf = float(np.clip(n_pts / 500.0, 0.2, 1.0))
    dist_conf = (
        1.0 if best_result < 2.0
        else float(np.clip(1.0 - (best_result - 2.0) / 3.0, 0.3, 1.0))
    )
    return float(np.clip(lm_conf * dist_conf, 0.0, 1.0))

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

def build_cp_messages(
    cp_results: list,
    n_pts: int,
    stamp: Any,
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

    # Single pass: best result per segment index (MultiDistance), and every
    # control point grouped by end-link (MultiLinkDistance).
    #
    # MultiLinkDistance used to keep only argmin over each link's control
    # points, collapsing 11 CPs to 5 entries — the other 6 never reached the
    # CBF at all, so a link could be pulled clear at its closest CP while a
    # second CP on the SAME link kept closing on the obstacle.  The QP has
    # always assembled one HOCBF row per entry it receives (it was never
    # min-only), so publishing per-CP is what actually turns on simultaneous
    # multi-CP activation; cbf_safety_filter needs no change to consume it.
    seg_best: dict[int, ControlPointResult] = {}
    by_link:  dict[str, list] = {}
    for r in cp_results:
        s = r.seg_idx
        if s not in seg_best or r.distance < seg_best[s].distance:
            seg_best[s] = r
        by_link.setdefault(r.end_link, []).append(r)

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

    # ── MultiLinkDistance (one LinkDistance per CONTROL POINT) ────────────
    # Ordered by segment_links, then by (seg_idx, cp_idx) within a link, so the
    # row order the CBF sees is stable frame to frame.  That matters: OSQP
    # reuses its factorization while n_c holds and only pushes new Ax values,
    # so a permuted row order would silently degrade every warm start.
    #
    # robot_link_name repeats across the CPs of one link — that is intended and
    # safe.  cbf_safety_filter uses it only to resolve a Pinocchio frame id, and
    # then calls point_jacobian(fid, ob.pr), which builds the Jacobian of the
    # arbitrary world point ob.pr rigidly attached to that frame.  Each CP
    # therefore gets its own correct row from its own closest_point_robot.
    link_entries = []
    for lk in segment_links:
        for r in sorted(by_link.get(lk, ()), key=lambda x: (x.seg_idx, x.cp_idx)):
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
            # d >= 0.0, NOT d > 0.0.  DistanceEngine clamps the surface gap with
            # np.maximum(..., 0.0), so a control point that has reached the
            # capsule surface reports EXACTLY 0.0.  The old `d > 0.0` therefore
            # marked the single most dangerous sample invalid, and
            # cbf_safety_filter._on_distances (`for ld in msg.links if ld.valid`)
            # dropped that CP's HOCBF row from the QP at the exact moment it was
            # needed.  The clamp is reachable well before physical contact: the
            # EE dilation margin subtracted upstream is 24 px, i.e. ~0.11 m at
            # Z = 2 m.  Zero is a valid, maximally-urgent measurement — only a
            # non-finite distance or a missing direction is not.
            ld.valid      = math.isfinite(d) and d >= 0.0 and di is not None
            ld.confidence = 1.0
            ld.zone       = get_safety_zone(d, zones)
            link_entries.append(ld)

    mld_msg = MultiLinkDistance()
    mld_msg.header.stamp    = stamp
    mld_msg.header.frame_id = frame_id
    mld_msg.links           = link_entries

    return multi_msg, mld_msg

def annotate_track_fields(msg, pipeline, skip_keys=None) -> int:
    """Fill the track fields of every entry of ``msg`` IN PLACE.

    Kept as a free function, and taking only the message and the pipeline, so
    the annotation rule is unit-testable without a node, a camera or a clock.

    Args:
        msg: the ``MultiLinkDistance`` about to be published.
        pipeline: an ``ObstacleTrackPipeline`` that has seen this frame.
        skip_keys: control-point labels (``'fr3_link5#0'``, same ``link#k``
            convention the CBF uses) whose track fields must be left at the
            all-zero "no track" defaults. This is where
            :class:`~franka_experiments.utils.self_detection.SelfDetectionMonitor`
            takes effect: a control point whose "obstacle" is moving rigidly
            with the arm gets no VELOCITY, while its DISTANCE goes out
            untouched. Suppressing an estimate degrades to today's behaviour;
            suppressing a distance would delete a barrier.

    Returns:
        How many entries were matched to a confirmed track.
    """
    skip = set(skip_keys or ())
    link_seen: dict = {}
    n = 0
    for ld in msg.links:
        # link#k in arrival order — the same label cbf_safety_filter builds, so
        # a key means the same control point on both sides of the wire. NOTE
        # the counter must advance for EVERY entry, skipped or not, or the
        # labels would shift and a skip would land on the wrong control point.
        k = link_seen.get(ld.robot_link_name, 0)
        link_seen[ld.robot_link_name] = k + 1
        if f'{ld.robot_link_name}#{k}' in skip:
            continue
        if not ld.valid:
            # An invalid entry has no meaningful closest_point_human — annotating
            # it would attach a velocity to a point that was never measured.
            continue
        p_base = np.array([ld.closest_point_human.x,
                           ld.closest_point_human.y,
                           ld.closest_point_human.z])
        tid, seen, v, P = pipeline.velocity_for_point(p_base)
        ld.track_id = int(tid)
        ld.frames_seen = int(seen)
        ld.obstacle_velocity.x = float(v[0])
        ld.obstacle_velocity.y = float(v[1])
        ld.obstacle_velocity.z = float(v[2])
        ld.velocity_covariance = np.asarray(P, dtype=np.float64).ravel()
        if tid:
            n += 1
    return n


def no_obs_warn(
    logger: Any,
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
