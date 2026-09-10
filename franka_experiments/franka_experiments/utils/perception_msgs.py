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
    return_cluster_ids: bool = False,
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
    return_cluster_ids:
        Also return the cluster id of every ``MultiLinkDistance`` entry
        (index-aligned with ``links``, -1 = unknown), for
        :func:`annotate_track_fields`.

    Returns
    -------
    (MultiDistance, MultiLinkDistance)
        Both messages are fully populated and ready to publish.
        ``(MultiDistance, MultiLinkDistance, cluster_ids)`` with
        ``return_cluster_ids``.

    With ``perception.multi_obstacle_k > 1`` a control point carries up to k
    obstacle entries (``ControlPointResult.extras``) and gets one LinkDistance
    PER ENTRY: its own nearest point first, exactly as with k = 1, then the
    nearest point of each other cluster, right behind it with the same
    ``closest_point_robot``. :func:`labelled_links` reads the rank back from
    that adjacency.
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
    link_cluster_ids = []
    for lk in segment_links:
        for r, d, di, obs, cid in _cp_rows(by_link.get(lk, ())):
            pt  = r.point
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
            link_cluster_ids.append(cid)

    mld_msg = MultiLinkDistance()
    mld_msg.header.stamp    = stamp
    mld_msg.header.frame_id = frame_id
    mld_msg.links           = link_entries

    if return_cluster_ids:
        return multi_msg, mld_msg, link_cluster_ids
    return multi_msg, mld_msg


def _cp_rows(results):
    """``(r, distance, direction, obstacle_point, cluster_id)`` per LinkDistance.

    Control points in ``(seg_idx, cp_idx)`` order — the contractual row order —
    each one's own result first (rank 0, what was always published), then its
    ``extras`` (rank >= 1, only with multi_obstacle_k > 1).
    """
    for r in sorted(results, key=lambda x: (x.seg_idx, x.cp_idx)):
        yield (r, r.distance, r.direction, r.closest_obstacle_point,
               getattr(r, 'cluster_id', -1))
        for h in getattr(r, 'extras', ()):
            yield r, h.distance, h.direction, h.point, h.cluster_id


def labelled_links(msg):
    """Yield ``(label, ld)`` for every entry of a MultiLinkDistance.

    ``label`` is ``'<robot_link_name>#<k>'`` with k counting the CONTROL POINTS
    of THIS link, and the counter advances for every control point — invalid
    ones included. That last clause is the whole reason this function exists.

    Multi-obstacle rows (multi_obstacle_k > 1): an entry with the same link and
    ``closest_point_robot`` as the one before it but a DIFFERENT
    ``closest_point_human`` is another obstacle of the same control point, and
    is labelled ``'<link>#<k>.<rank>'`` (rank >= 1) without advancing k. Rank 0
    keeps the bare ``'<link>#<k>'``, so every k = 1 label is unchanged and
    rank-0 labels never shift when a control point gains or loses extras.
    ``build_cp_messages`` emits the ranks of one control point adjacently.

    The convention is a contract between three places: this module writes the
    track fields of the entry a ``skip_keys`` label names, ``cbf_safety_filter``
    keys every per-control-point filter on the same label, and CBFDIAG prints
    it. It used to be open-coded on each side, and the two implementations
    disagreed: the consumer counted only the entries it had KEPT (valid, inside
    the obstacle horizon, enough Jacobian leverage, finite), so dropping the
    nearest control point of a link shifted every later one down by one — a
    self-detection skip landed on the wrong control point, and the barrier's
    recovery EMA, the residual closing-speed estimator, its rotation guard, the
    v_obs median and the uncertainty EMA were all handed another control
    point's state mid-run.

    ``msg.links`` is ordered by ``segment_links`` and then by
    ``(seg_idx, cp_idx)`` within a link, and ``build_cp_messages`` documents
    that order as contractual (OSQP warm starts depend on it), so the position
    of an entry is a stable identity for the control point that produced it.

    Yields every entry, valid or not; callers decide what to do with
    ``ld.valid``. Skipping them here would put the counter back inside a filter,
    which is the bug.
    """
    seen: dict = {}
    prev = None          # (robot key, human point) of the previous entry
    cp, rank = '', 0
    for ld in msg.links:
        pr, ph = ld.closest_point_robot, ld.closest_point_human
        robot = (ld.robot_link_name, pr.x, pr.y, pr.z)
        human = (ph.x, ph.y, ph.z)
        if prev is not None and robot == prev[0] and human != prev[1]:
            rank += 1
            prev = (robot, human)
            yield f'{cp}.{rank}', ld
            continue
        k = seen.get(ld.robot_link_name, 0)
        seen[ld.robot_link_name] = k + 1
        cp, rank, prev = f'{ld.robot_link_name}#{k}', 0, (robot, human)
        yield cp, ld


def annotate_track_fields(msg, pipeline, skip_keys=None, cluster_ids=None) -> int:
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
        cluster_ids: optional list index-aligned with ``msg.links``
            (``build_cp_messages(..., return_cluster_ids=True)``). An entry
            >= 0 gets the track of ITS OWN cluster
            (``pipeline.track_info_for_cluster``) rather than of whichever
            cluster sphere covers its point — with several rows per control
            point the nearest sphere is often another obstacle's. ``None``
            (always with multi_obstacle_k = 1) keeps the point lookup.

    Returns:
        How many entries were matched to a confirmed track.
    """
    skip = set(skip_keys or ())
    n = 0
    for i, (lbl, ld) in enumerate(labelled_links(msg)):
        # link#k positionally — the same label cbf_safety_filter builds, from
        # the same function, so a key means the same control point on both
        # sides of the wire.
        if lbl in skip:
            continue
        if not ld.valid:
            # An invalid entry has no meaningful closest_point_human — annotating
            # it would attach a velocity to a point that was never measured.
            continue
        p_base = np.array([ld.closest_point_human.x,
                           ld.closest_point_human.y,
                           ld.closest_point_human.z])
        cid = cluster_ids[i] if cluster_ids is not None else -1
        if cid >= 0 and hasattr(pipeline, 'track_info_for_cluster'):
            info = pipeline.track_info_for_cluster(cid)
        elif hasattr(pipeline, 'track_info_for_point'):
            info = pipeline.track_info_for_point(p_base)
        else:
            # A pipeline (or a test double) that predates the latency fields:
            # velocity only, the rest stays at the "no estimate" defaults.
            from franka_experiments.utils.obstacle_track_pipeline import TrackInfo
            tid_, seen_, v_, P_ = pipeline.velocity_for_point(p_base)
            info = TrackInfo(tid_, seen_, np.asarray(v_), np.asarray(P_), np.zeros(3),
                             np.zeros((3, 3)), np.zeros((3, 3)))
        tid, seen, v, P = info.track_id, info.frames_seen, info.velocity, info.velocity_cov
        ld.track_id = int(tid)
        ld.frames_seen = int(seen)
        ld.obstacle_velocity.x = float(v[0])
        ld.obstacle_velocity.y = float(v[1])
        ld.obstacle_velocity.z = float(v[2])
        # Latency-compensation fields. Guarded on the attribute so a message
        # package built before they existed still publishes the four fields
        # above — the consumer treats the missing ones as "no estimate".
        if hasattr(ld, 'obstacle_acceleration'):
            a = info.acceleration
            ld.obstacle_acceleration.x = float(a[0])
            ld.obstacle_acceleration.y = float(a[1])
            ld.obstacle_acceleration.z = float(a[2])
            ld.position_covariance = [float(x) for x in np.asarray(info.position_cov).ravel()]
            ld.position_velocity_covariance = [float(x) for x in np.asarray(info.pos_vel_cov).ravel()]
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
