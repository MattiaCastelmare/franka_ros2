#!/usr/bin/env python3
"""Replay a rosbag and compare the two obstacle-velocity estimators head to head.

WHY THIS SCRIPT EXISTS
----------------------
The cluster/track/Kalman pipeline is a replacement for
``ConstraintBuilder._obstacle_speed``, and a replacement inside a safety filter
has to be justified with numbers before it is wired in — not after. This script
produces those numbers on REAL data:

  (a) today's estimate, the scalar residual  v_res = aᵀq̇ − ḋ , EMA'd at
      obstacle_velocity_alpha and clamped at obstacle_velocity_max;
  (b) the new estimate, n̂ᵀ v_track from the tracked 3D velocity.

Both are computed for the SAME control point, on the SAME depth frame, from the
SAME obstacle pixels — the script runs one ``DistanceEngine`` and feeds both
estimators from it, so nothing in the comparison depends on two pipelines
agreeing about geometry.

(a) IS NOT REIMPLEMENTED HERE. The script constructs a bare
``ConstraintBuilder`` and calls its real ``_obstacle_speed`` method, so what is
plotted is the shipped estimator and cannot drift from it.

WHAT IT REPORTS
---------------
* RMS difference between the two estimates, overall and on approach only. On
  approach is the number that matters: both are clamped at ≥ 0 downstream, so
  disagreement in the receding half never reaches the QP.
* LAG, from the peak of the cross-correlation of the two series. A positive lag
  means the TRACKER LEADS — which is the whole point of the exercise, since the
  residual pays a 75 ms EMA for its smoothness.
* NOISE FLOOR on static-obstacle segments. "Static" is decided WITHOUT either
  estimator, so the test is not circular: a window counts as static when the
  measured surface gap varies by less than a few millimetres AND the arm is
  nearly still, in which case ḋ ≈ 0 and aᵀq̇ ≈ 0, so the true obstacle speed is
  ≈ 0 and whatever an estimator reports there is its own noise.

USAGE
-----
    python3 scripts/compare_vobs.py --bag rosbag/arm_complex \\
        --robot-config config/fr3_complete.yaml \\
        --extrinsics config/camera_extrinsics.yaml \\
        --out /tmp/vobs

Needs a sourced ROS 2 environment (message types, tf2, Pinocchio, xacro).
"""
from __future__ import annotations

import argparse
import glob
import os
import sqlite3
import sys
from collections import defaultdict

import numpy as np


# ── Logger shim ──────────────────────────────────────────────────────────────
# The perception classes take a ROS logger and call throttled methods on it.
# Offline there is no node, and a print-everything logger would bury the report
# under RTDDIAG lines, so this one is quiet by default.

class _Log:
    def __init__(self, verbose=False):
        self.verbose = verbose

    def _emit(self, level, msg):
        if self.verbose:
            print(f'[{level}] {msg}', file=sys.stderr)

    def info(self, m, **k):    self._emit('info', m)
    def warn(self, m, **k):    self._emit('warn', m)
    def warning(self, m, **k): self._emit('warn', m)
    def error(self, m, **k):   self._emit('error', m)
    def debug(self, m, **k):   self._emit('debug', m)


# ── Bag reading ──────────────────────────────────────────────────────────────

def read_bag(bag_dir, topics):
    """Yield ``(topic, stamp_ns, deserialised_msg)`` in log order.

    sqlite3 straight off the .db3 rather than ``rosbag2_py``: the storage plugin
    is not always importable outside a launched node, and the schema here is two
    tables. Only the requested topics are deserialised — the colour image in
    these bags is ~90 % of the bytes and is never used.
    """
    from rosidl_runtime_py.utilities import get_message
    from rclpy.serialization import deserialize_message

    dbs = sorted(glob.glob(os.path.join(bag_dir, '*.db3')))
    if not dbs:
        raise SystemExit(f'no .db3 found under {bag_dir}')
    for db in dbs:
        con = sqlite3.connect(f'file:{db}?mode=ro', uri=True)
        try:
            meta = {name: (tid, get_message(ttype)) for tid, name, ttype in
                    con.execute('SELECT id, name, type FROM topics')
                    if name in topics}
            if not meta:
                continue
            ids = {tid: (name, cls) for name, (tid, cls) in meta.items()}
            q = ('SELECT topic_id, timestamp, data FROM messages '
                 f'WHERE topic_id IN ({",".join("?" * len(ids))}) ORDER BY timestamp')
            for tid, ts, blob in con.execute(q, tuple(ids)):
                name, cls = ids[tid]
                yield name, ts, deserialize_message(bytes(blob), cls)
        finally:
            con.close()


def build_tf_buffer(bag_dir, tf_topic='/tf', tf_static_topic='/tf_static'):
    """A tf2 Buffer preloaded with the WHOLE bag's transforms.

    Loading everything up front, rather than streaming, is what lets the depth
    loop below use TFManager's Level-1 (exact-timestamp) path on every frame:
    online the buffer only ever holds the recent past, but offline there is no
    reason to reproduce that handicap — and a Level-2 "latest available"
    fallback would put the robot in a slightly wrong pose, which lands directly
    in aᵀq̇ and hence in estimator (a).
    """
    from rclpy.duration import Duration
    from tf2_ros import Buffer

    # A default Buffer keeps only the last 10 s. These bags run for ~2 minutes,
    # so with the default every transform older than the last 10 s is evicted
    # before the depth loop starts and EVERY early lookup fails — silently, via
    # TFManager's Level-2 fallback, which would quietly pose the robot from the
    # END of the bag. Size the cache to the whole recording instead.
    buf = Buffer(cache_time=Duration(seconds=24 * 3600))
    n = 0
    for topic, _, msg in read_bag(bag_dir, {tf_topic, tf_static_topic}):
        static = topic == tf_static_topic
        for tr in msg.transforms:
            if static:
                buf.set_transform_static(tr, 'bag')
            else:
                buf.set_transform(tr, 'bag')
            n += 1
    return buf, n


# ── Joint states ─────────────────────────────────────────────────────────────

class JointTrace:
    """Time-indexed (q, q̇) with nearest-sample lookup, in Pinocchio's order."""

    def __init__(self, model, joint_names):
        self.idx_q = [model.joints[model.getJointId(j)].idx_q for j in joint_names]
        self.idx_v = [model.joints[model.getJointId(j)].idx_v for j in joint_names]
        self.names = list(joint_names)
        self.nq, self.nv = model.nq, model.nv
        self.t = []
        self.q = []
        self.v = []

    def add(self, stamp_s, msg):
        pos = dict(zip(msg.name, msg.position))
        vel = dict(zip(msg.name, msg.velocity)) if len(msg.velocity) else {}
        if any(n not in pos for n in self.names):
            return
        q = np.zeros(self.nq)
        v = np.zeros(self.nv)
        for k, n in enumerate(self.names):
            q[self.idx_q[k]] = pos[n]
            v[self.idx_v[k]] = vel.get(n, 0.0)
        self.t.append(stamp_s)
        self.q.append(q)
        self.v.append(v)

    def finish(self):
        self.t = np.asarray(self.t)
        self.q = np.asarray(self.q)
        self.v = np.asarray(self.v)

    def at(self, stamp_s, max_age=0.1):
        """Nearest sample, or None when the nearest is further than ``max_age``.

        Nearest rather than interpolated: q̇ is published, not differenced, so
        interpolating it would smooth a signal the residual estimator consumes
        raw — and the comparison must not flatter the new estimator by handing
        the old one a filtered input.
        """
        if self.t.size == 0:
            return None
        i = int(np.argmin(np.abs(self.t - stamp_s)))
        if abs(self.t[i] - stamp_s) > max_age:
            return None
        return self.q[i], self.v[i]


# The injected sphere lives in utils/obstacle_sim so the live pipeline can
# render the SAME obstacle — see that module for why it exists at all.
from franka_experiments.utils.obstacle_sim import InjectedSphere


# ── The two estimators ───────────────────────────────────────────────────────

class _PShim:
    """The two parameters ``_obstacle_speed`` reads, and nothing else."""

    def __init__(self, alpha, vmax):
        self.obstacle_velocity_alpha = float(alpha)
        self.obstacle_velocity_max = float(vmax)


def make_residual_estimator(alpha, vmax):
    """A callable with today's estimator's REAL code and its own state.

    ``ConstraintBuilder`` is instantiated through ``__new__`` and given only the
    three attributes ``_obstacle_speed`` touches. Calling the shipped bound
    method is the point: a reimplementation here could silently drift from the
    filter and the comparison would then be against a straw man.
    """
    from franka_experiments.utils.cbf_state_rows import ConstraintBuilder

    cb = ConstraintBuilder.__new__(ConstraintBuilder)
    cb._P = _PShim(alpha, vmax)
    cb._obs_vel = {}
    cb._obs_frames = {}
    return cb


# ── Metrics ──────────────────────────────────────────────────────────────────

def cross_correlation_lag(a, b, dt, max_lag_s=0.5):
    """Lag [s] of ``b`` relative to ``a``, from the cross-correlation peak.

    POSITIVE means ``b`` LEADS ``a`` — i.e. b's features appear earlier in time.
    Both series are mean-removed first; without that, two signals sharing a
    large DC offset correlate best at zero lag whatever their timing.
    """
    a = np.asarray(a, dtype=np.float64)
    b = np.asarray(b, dtype=np.float64)
    if a.size < 8 or np.std(a) < 1e-9 or np.std(b) < 1e-9:
        return float('nan'), float('nan')
    a = a - a.mean()
    b = b - b.mean()
    n = int(round(max_lag_s / dt))
    denom = np.sqrt((a @ a) * (b @ b))

    def corr(L):
        if L >= 0:
            x, y = a[L:], (b[:b.size - L] if L else b)
        else:
            x, y = a[:a.size + L], b[-L:]
        m = min(x.size, y.size)
        return float(x[:m] @ y[:m]) / denom if m >= 8 else -np.inf

    rs = {L: corr(L) for L in range(-n, n + 1)}
    best = max(rs, key=rs.get)
    # Parabolic interpolation through the peak and its two neighbours. Without
    # it the answer is quantised to one perception frame (33 ms at 30 Hz),
    # which is the same order as the lag being measured — so the raw peak
    # cannot distinguish "no lag" from "half a frame of lag".
    lo, hi = rs.get(best - 1, -np.inf), rs.get(best + 1, -np.inf)
    off = 0.0
    if np.isfinite(lo) and np.isfinite(hi):
        den = lo - 2.0 * rs[best] + hi
        if abs(den) > 1e-12:
            off = float(np.clip(0.5 * (lo - hi) / den, -0.5, 0.5))
    return (best + off) * dt, rs[best]


def static_windows(d, qdot_norm, win=9, d_tol=0.004, qdot_tol=0.05):
    """Boolean mask of samples inside a STATIC-obstacle window.

    Decided from the raw measurements only — the surface gap and the joint
    speed — never from either estimator, so the noise-floor comparison is not
    circular. Inside such a window ḋ ≈ 0 and aᵀq̇ ≈ 0, hence the true obstacle
    speed is ≈ 0 and anything an estimator reports is its own noise.
    """
    d = np.asarray(d, dtype=np.float64)
    q = np.asarray(qdot_norm, dtype=np.float64)
    mask = np.zeros(d.size, dtype=bool)
    h = win // 2
    for i in range(h, d.size - h):
        seg = d[i - h:i + h + 1]
        if np.all(np.isfinite(seg)) and (seg.max() - seg.min()) < d_tol \
                and np.nanmax(q[i - h:i + h + 1]) < qdot_tol:
            mask[i] = True
    return mask


def rms(x):
    x = np.asarray(x, dtype=np.float64)
    x = x[np.isfinite(x)]
    return float(np.sqrt((x * x).mean())) if x.size else float('nan')


# ── Replay ───────────────────────────────────────────────────────────────────

def replay(args):
    import pinocchio as pin

    from franka_experiments.utils.distance_engine import DistanceEngine
    from franka_experiments.utils.distance_utils import (
        compute_roi, define_control_points, load_extrinsics, load_robot_config)
    from franka_experiments.utils.kinematics import CBFKinematics, build_urdf_no_hand
    from franka_experiments.utils.mask_builder import MaskBuilder
    from franka_experiments.utils.obstacle_track_pipeline import ObstacleTrackPipeline
    from franka_experiments.utils.self_detection import SelfDetectionMonitor
    from franka_experiments.utils.tf_manager import TFManager

    log = _Log(args.verbose)
    cfg = load_robot_config(args.robot_config)
    robot_cfg, mask_cfg, mesh_cfg = cfg['robot'], cfg['mask'], cfg['meshes']
    distance_cfg = dict(cfg['distance'])
    distance_cfg['export_obstacle_cloud'] = True

    R_base, t_base = load_extrinsics(args.extrinsics)
    R32, t32 = R_base.astype(np.float32), t_base.astype(np.float32)

    print(f'reading TF from {args.bag} ...', flush=True)
    tf_buf, n_tf = build_tf_buffer(args.bag)
    print(f'  {n_tf} transforms loaded')
    ee_link = robot_cfg.get('ee_link', 'fr3_link8')
    tf_mgr = TFManager(tf_buffer=tf_buf, base_frame=robot_cfg['base_frame'],
                       critical_links=robot_cfg.get('critical_links', [ee_link]),
                       cache_max_age_s=None, logger=log)

    model = pin.buildModelFromUrdf(build_urdf_no_hand())
    kin = CBFKinematics(model)
    joints = JointTrace(model, [f'fr3_joint{i}' for i in range(1, 8)])

    import trimesh
    from ament_index_python.packages import get_package_share_directory
    mesh_dir = get_package_share_directory(mesh_cfg.get('package', 'franka_description'))
    samples = {n: trimesh.load(os.path.join(mesh_dir, rel), force='mesh')
               .sample(int(mesh_cfg.get('sample_points_per_link', 300)))
               for n, rel in mesh_cfg['files'].items()}
    mask_builder = MaskBuilder(link_mesh_samples=samples, R_base=R_base,
                               t_base=t_base, ee_link=ee_link,
                               mask_cfg=mask_cfg, logger=log)
    engine = DistanceEngine(distance_cfg=distance_cfg, logger=log)
    pipeline = ObstacleTrackPipeline(
        voxel_m=args.voxel_m, min_cluster_points=args.min_cluster_points,
        max_cluster_radius=args.max_cluster_radius,
        depth_jump=args.depth_jump,
        contains_tol=args.contains_tol, q_jerk=args.q_jerk,
        sigma_meas=args.sigma_meas)
    residual = make_residual_estimator(args.alpha, args.vmax)
    # Runs unconditionally: on a bag whose extrinsics no longer match the
    # recording, a large share of what the pipeline calls 'obstacle' is the
    # arm itself, and every number below would be about the robot rather
    # than about an obstacle. Better to measure that than to discover it
    # afterwards.
    self_detect = SelfDetectionMonitor()

    from cv_bridge import CvBridge
    bridge = CvBridge()

    sphere = None
    if args.inject:
        sphere = InjectedSphere(args.inject_start, args.inject_vel,
                                radius=args.inject_radius,
                                period=args.inject_period,
                                amplitude=args.inject_amplitude)
        print(f'INJECTING a {args.inject_radius:.2f} m sphere shuttling '
              f'{args.inject_amplitude:.2f} m from {args.inject_start} along '
              f'{sphere.dir.round(2).tolist()} (camera frame) at '
              f'{2 * args.inject_amplitude / args.inject_period:.2f} m/s, '
              f'period {args.inject_period:.1f} s')

    # ── First pass: joint states + camera info ───────────────────────────
    print('reading joint states / camera info ...', flush=True)
    K = None
    for topic, _, msg in read_bag(args.bag, {args.joint_topic, args.info_topic}):
        if topic == args.joint_topic:
            s = msg.header.stamp
            joints.add(s.sec + s.nanosec * 1e-9, msg)
        elif K is None:
            K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
    joints.finish()
    if K is None:
        raise SystemExit(f'no CameraInfo on {args.info_topic}')
    print(f'  {joints.t.size} joint samples, K fx={K[0, 0]:.1f} cx={K[0, 2]:.1f}')
    mask_builder.set_intrinsics(K)
    cx32, cy32 = np.float32(K[0, 2]), np.float32(K[1, 2])
    fxi32, fyi32 = np.float32(1.0 / K[0, 0]), np.float32(1.0 / K[1, 1])

    step = int(distance_cfg['pixel_step'])
    margin = int(distance_cfg['image_margin_px'])

    # per control-point series
    series = defaultdict(lambda: defaultdict(list))
    n_frames = n_used = n_tracked = n_no_tf = 0
    prev_shape = None

    print('replaying depth frames ...', flush=True)
    for topic, _, msg in read_bag(args.bag, {args.depth_topic}):
        n_frames += 1
        if args.max_frames and n_frames > args.max_frames:
            break
        depth = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        stamp = msg.header.stamp
        t_cap = stamp.sec + stamp.nanosec * 1e-9
        if sphere is not None:
            depth = depth.copy()      # never mutate the bag's buffer
            sphere.render(depth, t_cap, K)

        if prev_shape != depth.shape:
            prev_shape = depth.shape
            mask_builder.invalidate()
            engine.invalidate_grid_cache()
            engine.reset_lpf()
            pipeline.reset()

        qv = joints.at(t_cap)
        if qv is None:
            continue
        q, qdot = qv

        transforms = tf_mgr.lookup_all(robot_cfg['segment_links'], stamp)
        if not transforms:
            n_no_tf += 1
            continue
        cps = define_control_points(transforms, robot_cfg, distance_cfg)
        if not cps:
            continue

        H, W = depth.shape
        mask_builder.rebuild(transforms, depth.shape)
        roi = compute_roi(mask_builder.search_exclusion_mask, H, W, margin,
                          int(distance_cfg['roi_pad_px'])) or \
            (margin, margin, W - margin, H - margin)
        x0, y0, x1, y1 = roi

        cp_results, _ = engine.compute(
            depth=depth, cx_f32=cx32, cy_f32=cy32,
            fx_inv_f32=fxi32, fy_inv_f32=fyi32,
            R_base_f32=R32, t_base_f32=t32, control_points=cps,
            x=np.array([x0, x1]), y=np.array([y0, y1]), step=step,
            search_exclusion_mask=mask_builder.search_exclusion_mask,
            transforms=transforms,
            ee_source_mask=mask_builder.ee_source_mask,
            dilation_margins_px=mask_builder.dilation_margins_px,
            frame_stamp=t_cap)
        if cp_results is None:
            continue

        pipeline.update(engine.last_obstacle_cloud, R_base, t_base, stamp=t_cap)
        kin.update(q, qdot, with_jdot=False)
        n_used += 1

        seen = defaultdict(int)
        for r in cp_results:
            if not np.isfinite(r.distance) or r.direction is None \
                    or r.closest_obstacle_point is None:
                continue
            link = r.end_link
            k = seen[link]
            seen[link] += 1
            lbl = f'{link}#{k}'

            n_hat = np.asarray(r.direction, dtype=np.float64)
            nn = float(np.linalg.norm(n_hat))
            if nn < 1e-9:
                continue
            n_hat = n_hat / nn

            fid = kin.resolve_frame_id(link)
            if fid is None:
                continue
            Jp = kin.point_jacobian_pos(fid, np.asarray(r.point, dtype=np.float64))
            adotq = float((n_hat @ Jp) @ qdot)

            # (a) the shipped estimator, called on its own code
            v_res = residual._obstacle_speed(lbl, float(r.distance), t_cap, adotq)
            # (b) the tracked estimate, projected onto the same normal.
            #
            # THE SIGN, derived rather than guessed, because step 7 inherits it.
            # n_hat points OBSTACLE -> CONTROL POINT. The gap closes at
            #     ddot = n_hat^T (p_robot_dot - p_obs_dot)
            # and the residual estimator defines
            #     v_obs = a^T qdot - ddot = n_hat^T p_robot_dot - ddot
            #           = n_hat^T p_obs_dot .
            # So v_obs is the PLAIN projection, no minus sign: an obstacle
            # moving along +n_hat is moving toward the control point and yields
            # a positive (closing) v_obs, exactly as the residual does.
            tid, seen_n, v_trk, _ = pipeline.velocity_for_point(
                np.asarray(r.closest_obstacle_point, dtype=np.float64))
            v_prj = float(n_hat @ v_trk) if tid else np.nan
            if tid:
                n_tracked += 1

            # Ground truth, when a sphere with a known velocity was injected:
            # the obstacle's contribution to the closing rate is the projection
            # of its velocity onto the SAME n_hat both estimators are projected
            # onto, so the three numbers are directly comparable.
            v_true = np.nan
            on_sphere = False
            if sphere is not None:
                v_true = float(n_hat @ (R_base @ sphere.velocity(t_cap)))
                # Is THIS control point actually looking at the sphere? Scoring
                # a control point that is looking at the far wall against the
                # sphere's velocity would measure nothing but the geometry.
                obs_cam = R_base.T @ (np.asarray(r.closest_obstacle_point,
                                                 dtype=np.float64) - t_base)
                on_sphere = bool(np.linalg.norm(obs_cam - sphere.centre(t_cap))
                                 < sphere.radius + 0.05)

            is_self = self_detect.update(
                lbl, np.asarray(r.point, dtype=np.float64),
                np.asarray(r.closest_obstacle_point, dtype=np.float64))

            s = series[lbl]
            s['self'].append(1.0 if is_self else 0.0)
            for _k, _v in (('pr', r.point), ('ph', r.closest_obstacle_point)):
                for _a in range(3):
                    s[f'{_k}{_a}'].append(float(np.asarray(_v)[_a]))
            s['v_true'].append(v_true)
            s['on_sphere'].append(1.0 if on_sphere else 0.0)
            s['t'].append(t_cap)
            s['d'].append(float(r.distance))
            s['adotq'].append(adotq)
            s['qdot'].append(float(np.linalg.norm(qdot)))
            s['v_res'].append(float(v_res))
            s['v_trk'].append(v_prj)
            s['track_id'].append(int(tid))
            s['frames_seen'].append(int(seen_n))

        if n_used % 100 == 0:
            print(f'  {n_used} frames | {pipeline.describe()}', flush=True)

    print(f'\n{n_frames} depth messages, {n_used} processed, '
          f'{n_no_tf} skipped for missing TF, '
          f'{n_tracked} control-point samples matched to a track')
    rep = self_detect.report()
    if rep:
        print('\n*** ' + rep + ' ***')
    return series


# ── Report ───────────────────────────────────────────────────────────────────

def report(series, out_dir, dt_nominal, args_d_gate=0.6):
    os.makedirs(out_dir, exist_ok=True)
    rows = []
    print('\n' + '=' * 96)
    print('PER CONTROL POINT'.center(96))
    print('=' * 96)
    hdr = (f'{"cp":<14}{"N":>6}{"tracked%":>10}{"RMS diff":>10}'
           f'{"RMS diff+":>11}{"lag ms":>9}{"peak r":>8}'
           f'{"noise res":>11}{"noise trk":>11}{"N static":>9}{"SELF%":>7}')
    print(hdr)
    print('-' * 96)

    for lbl in sorted(series):
        s = {k: np.asarray(v, dtype=np.float64) for k, v in series[lbl].items()}
        s.setdefault('on_sphere', np.zeros_like(s['t']))
        s.setdefault('self', np.zeros_like(s['t']))
        n = s['t'].size
        if n < 30:
            continue
        have = np.isfinite(s['v_trk'])
        frac = float(have.mean())
        # Only compare where the tracker has an estimate at all; elsewhere it
        # publishes the safe zero, which is a fallback and not a disagreement.
        both = have & np.isfinite(s['v_res'])
        diff = s['v_trk'][both] - s['v_res'][both]
        # The clamped half is the one that reaches the QP: both estimators are
        # used as max(v, 0), so disagreement while receding costs nothing.
        pos = np.maximum(s['v_trk'][both], 0.0) - np.maximum(s['v_res'][both], 0.0)

        # Lag needs a gapless series; fill the untracked samples with 0.0, which
        # is exactly what the consumer would see there.
        vt = np.where(have, np.nan_to_num(s['v_trk']), 0.0)
        lag, r = cross_correlation_lag(s['v_res'], vt, dt_nominal)

        stat = static_windows(s['d'], s['qdot'])
        nf_res = rms(s['v_res'][stat]) if stat.any() else float('nan')
        nf_trk = rms(vt[stat]) if stat.any() else float('nan')

        print(f'{lbl:<14}{n:>6}{100 * frac:>9.1f}%{rms(diff):>10.3f}'
              f'{rms(pos):>11.3f}{1000 * lag:>9.1f}{r:>8.2f}'
              f'{nf_res:>11.4f}{nf_trk:>11.4f}{int(stat.sum()):>9}'
              f'{100 * float(s["self"].mean()):>6.0f}%')
        rows.append((lbl, s, stat))

    # ── Against ground truth, when an obstacle was injected ──────────────
    truthy = [(lbl, s, st) for lbl, s, st in rows
              if 'v_true' in s and np.isfinite(s['v_true']).any()]
    if truthy:
        print('\n' + '=' * 96)
        print('AGAINST GROUND TRUTH (injected obstacle)'.center(96))
        print('=' * 96)
        print(f'{"cp":<14}{"N on obst":>12}{"RMS res":>10}{"RMS trk":>10}'
              f'{"bias res":>10}{"bias trk":>10}{"lag res ms":>12}{"lag trk ms":>12}')
        print('-' * 96)
        for lbl, s, _ in truthy:
            vt = np.where(np.isfinite(s['v_trk']),
                          np.nan_to_num(s['v_trk']), 0.0)
            tru = s['v_true']
            # Score only where the obstacle is actually the nearest thing to
            # this control point: elsewhere both estimators are correctly
            # reporting something else, and scoring them against the sphere's
            # velocity would be scoring them on the wrong obstacle.
            m = (np.isfinite(tru) & np.isfinite(s['v_res'])
                 & (s.get('on_sphere', np.zeros_like(tru)) > 0.5))
            if m.sum() < 30:
                continue
            lr, _ = cross_correlation_lag(tru[m], s['v_res'][m], dt_nominal)
            lt, _ = cross_correlation_lag(tru[m], vt[m], dt_nominal)
            print(f'{lbl:<14}{int(m.sum()):>12}{rms(s["v_res"][m] - tru[m]):>10.3f}'
                  f'{rms(vt[m] - tru[m]):>10.3f}'
                  f'{np.mean(s["v_res"][m] - tru[m]):>10.3f}'
                  f'{np.mean(vt[m] - tru[m]):>10.3f}'
                  f'{1000 * lr:>12.1f}{1000 * lt:>12.1f}')
        print('-' * 96)
        print('RMS/bias : error of each estimate against n_hat^T v_sphere, over')
        print('           the samples where this control point\'s reported')
        print('           closest_point_human actually lies ON the sphere.')
        print('lag ms   : POSITIVE = the estimate LEADS the truth, NEGATIVE =')
        print('           it trails it. This is the number the 0.7 EMA costs.')

    # ── Aggregate ────────────────────────────────────────────────────────
    if rows:
        allr = np.concatenate([r[1]['v_res'] for r in rows])
        allt = np.concatenate([np.where(np.isfinite(r[1]['v_trk']),
                                        np.nan_to_num(r[1]['v_trk']), 0.0)
                               for r in rows])
        allstat = np.concatenate([r[2] for r in rows])
        have = np.concatenate([np.isfinite(r[1]['v_trk']) for r in rows])
        print('-' * 96)
        print(f'{"ALL":<14}{allr.size:>6}{100 * have.mean():>9.1f}%'
              f'{rms(allt[have] - allr[have]):>10.3f}'
              f'{rms(np.maximum(allt[have], 0) - np.maximum(allr[have], 0)):>11.3f}'
              f'{"":>9}{"":>8}{rms(allr[allstat]):>11.4f}'
              f'{rms(allt[allstat]):>11.4f}{int(allstat.sum()):>9}')
        print('=' * 96)
        print('RMS diff   : |v_trk - v_res| over samples where a track exists.')
        print('RMS diff+  : same, after the max(v, 0) clamp both estimates get')
        print('             downstream -- the only half that reaches the QP.')
        print('lag ms     : cross-correlation peak, POSITIVE = tracker LEADS.')
        print('noise res/trk: RMS of each estimate on STATIC-obstacle windows,')
        print('             where the true obstacle speed is ~0 by construction')
        print('             (gap steady to <4 mm AND ||qdot|| < 0.05 rad/s), so')
        print('             the number is the estimator\'s own noise floor.')

    _plot(rows, out_dir)
    np.savez_compressed(os.path.join(out_dir, 'compare_vobs.npz'),
                        **{f'{lbl}|{k}': v for lbl, s, _ in rows
                           for k, v in s.items()})
    print(f'\nwrote {out_dir}/compare_vobs.png and compare_vobs.npz')


def _plot(rows, out_dir):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    if not rows:
        return
    rows = sorted(rows, key=lambda r: -np.nanmax(np.abs(r[1]['v_res'])))[:4]
    fig, axes = plt.subplots(len(rows), 1, figsize=(13, 3.0 * len(rows)),
                             sharex=True, squeeze=False)
    for ax, (lbl, s, stat) in zip(axes[:, 0], rows):
        t = s['t'] - s['t'][0]
        ax.plot(t, s['v_res'], lw=1.0, color='tab:orange',
                label='residual  a·qdot − ddot  (today)')
        ax.plot(t, s['v_trk'], lw=1.2, color='tab:blue',
                label='n̂ᵀ v_track  (tracker)')
        ax.fill_between(t, -2, 2, where=stat, color='0.85', step='mid',
                        zorder=0, label='static-obstacle window')
        ax2 = ax.twinx()
        ax2.plot(t, s['d'], lw=0.8, color='0.4', ls=':')
        ax2.set_ylabel('gap [m]', color='0.4', fontsize=8)
        ax.set_ylim(-1.5, 1.5)
        ax.set_ylabel('v_obs [m/s]')
        ax.set_title(lbl, fontsize=10, loc='left')
        ax.grid(alpha=0.3)
    axes[0, 0].legend(loc='upper right', fontsize=8)
    axes[-1, 0].set_xlabel('time [s]')
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, 'compare_vobs.png'), dpi=110)


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--bag', required=True)
    p.add_argument('--robot-config', required=True)
    p.add_argument('--extrinsics', required=True)
    p.add_argument('--depth-topic', default='/camera/camera/aligned_depth_to_color/image_raw')
    p.add_argument('--info-topic', default='/camera/camera/aligned_depth_to_color/camera_info')
    p.add_argument('--joint-topic', default='/NS_1/joint_states')
    p.add_argument('--out', default='/tmp/compare_vobs')
    p.add_argument('--max-frames', type=int, default=0)
    p.add_argument('--dt', type=float, default=1.0 / 30.0,
                   help='nominal perception period, for the lag axis only')
    # Estimator (a) — must mirror fr3_control.yaml to be a fair comparison.
    p.add_argument('--alpha', type=float, default=0.7)
    p.add_argument('--vmax', type=float, default=2.0)
    # Estimator (b)
    p.add_argument('--voxel-m', type=float, default=0.02)
    p.add_argument('--min-cluster-points', type=int, default=10)
    p.add_argument('--depth-jump', type=float, default=0.10,
                   help='[m] depth discontinuity that separates two objects')
    p.add_argument('--max-cluster-radius', type=float, default=None,
                   help='[m] drop clusters bigger than this (scene guard); '
                        'omit for no limit')
    p.add_argument('--contains-tol', type=float, default=0.05)
    p.add_argument('--q-jerk', type=float, default=2.0)
    p.add_argument('--sigma-meas', type=float, default=0.01)
    # Ground-truth injection
    p.add_argument('--inject', action='store_true',
                   help='render a sphere with a known trajectory into every '
                        'depth frame, so both estimators can be scored against '
                        'truth instead of only against each other')
    p.add_argument('--inject-start', type=float, nargs=3, default=(0.0, 0.0, 1.6),
                   help='sphere start centre in CAMERA frame [m]')
    p.add_argument('--inject-vel', type=float, nargs=3, default=(0.0, 0.0, -0.5),
                   help='sphere velocity in CAMERA frame [m/s]')
    p.add_argument('--inject-radius', type=float, default=0.15)
    p.add_argument('--inject-period', type=float, default=2.0)
    p.add_argument('--inject-amplitude', type=float, default=0.5,
                   help='[m] half-stroke of the shuttle')
    p.add_argument('--truth-gap-gate', type=float, default=0.6,
                   help='[m] score against truth only where the reported gap is '
                        'below this, i.e. where the sphere is what the control '
                        'point is actually looking at')
    p.add_argument('--verbose', action='store_true')
    args = p.parse_args()
    report(replay(args), args.out, args.dt, args.truth_gap_gate)


if __name__ == '__main__':
    main()
