#!/usr/bin/env python3
"""Latency budget of the depth → distance → tracker → CBF → torque chain.

WHY THIS EXISTS
---------------
Every conservative constant in the safety filter that has a time in it —
``link_speed_reaction_s``, the uncertainty margin's ``t_latency``, the blind
time a fast obstacle can exploit — was a guess. This script MEASURES the hops,
one at a time, with real stamps where a stamp exists and with the shipped code
paths where the hop is compute. Nothing here publishes to the robot.

The chain, and which sub-command measures which hop:

    camera exposure ─┐
    camera stamp ────┼─► depth msg receipt        `bag`   (rosbag receipt stamps)
                     │                            `cam`   (live, read-only probe)
    depth receipt ───┼─► per_link_distances pub   `perception` (shipped code, offline
                     │        [tf, mask, distance, cluster+track, message build])
    distances pub ───┼─► CBF receipt              `dds`   (transport probe, no SHM)
    CBF receipt ─────┼─► constraint snapshot      `cbf`   (ConstraintBuilder.build)
    snapshot ────────┼─► qddot_safe pub           `cbf`   (the whole _qp_tick)
    qddot_safe ──────┼─► torque_cmd               `dds` + `cbf` (M·q̈ + C·q̇)
    torque_cmd ──────┴─► torque write             1 kHz controller: 0–1 ms

The rate-induced waits (a 50 Hz rebuild polling a 30 Hz stream, a 100 Hz tick
polling a 50 Hz snapshot) are not measured — they are uniform on [0, period]
by construction and the report adds them analytically.

USAGE (inside the ROS container)
--------------------------------
    python3 scripts/latency_budget.py bag        rosbag/arm_complex
    python3 scripts/latency_budget.py cam        /cams/d455/depth/image_rect_raw
    python3 scripts/latency_budget.py perception --bag rosbag/arm_complex
    python3 scripts/latency_budget.py cbf
    ROS_DOMAIN_ID=77 python3 scripts/latency_budget.py dds --role pub &
    ROS_DOMAIN_ID=77 python3 scripts/latency_budget.py dds --role sub

`bag` needs only numpy; the others need the sourced workspace.
"""
from __future__ import annotations

import argparse
import glob
import os
import sqlite3
import struct
import sys
import time
from collections import defaultdict

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)


# ── Reporting ────────────────────────────────────────────────────────────────

def stats(x):
    x = np.asarray(x, dtype=np.float64)
    if x.size == 0:
        return None
    return dict(n=int(x.size), med=float(np.median(x)),
                p95=float(np.percentile(x, 95)), p99=float(np.percentile(x, 99)),
                min=float(x.min()), max=float(x.max()))


def line(name, x, unit='ms', width=34):
    s = stats(x)
    if s is None:
        return f'  {name:<{width}s} (no samples)'
    return (f'  {name:<{width}s} n={s["n"]:5d}  med={s["med"]:8.2f}  '
            f'p95={s["p95"]:8.2f}  p99={s["p99"]:8.2f}  max={s["max"]:8.2f} {unit}')


class _Log:
    def __init__(self, verbose=False):
        self.verbose = verbose

    def _e(self, lvl, m):
        if self.verbose:
            print(f'[{lvl}] {m}', file=sys.stderr)

    def info(self, m, **k):    self._e('info', m)
    def warn(self, m, **k):    self._e('warn', m)
    def warning(self, m, **k): self._e('warn', m)
    def error(self, m, **k):   self._e('error', m)
    def debug(self, m, **k):   self._e('debug', m)


# ═════════════════════════════════════════════════════════════════════════════
#  bag — header stamp vs recorder receipt, straight off the sqlite file
# ═════════════════════════════════════════════════════════════════════════════

def _hdr_stamp(blob, off):
    sec, nsec = struct.unpack_from('<iI', blob, off)
    return sec + nsec * 1e-9


def cmd_bag(args):
    """No ROS needed: the CDR encapsulation is 4 bytes, then std_msgs/Header
    starts with the stamp (int32 sec, uint32 nanosec). For TFMessage there is
    a uint32 sequence length first."""
    dbs = sorted(glob.glob(os.path.join(args.bag, '*.db3')))
    if not dbs:
        raise SystemExit(f'no .db3 under {args.bag}')
    for db in dbs:
        con = sqlite3.connect(f'file:{db}?mode=ro', uri=True)
        topics = {r[0]: (r[1], r[2]) for r in con.execute('select id,name,type from topics')}
        print(f'== {db}')
        for tid, (name, typ) in topics.items():
            if not any(k in name for k in ('image', 'joint_states', '/tf', 'distance', 'qddot', 'torque')):
                continue
            if typ.endswith('CameraInfo') or name.endswith('_static'):
                continue
            off = 8 if typ.endswith('TFMessage') else 4
            if typ.endswith('Float64MultiArray'):
                continue                                  # no header
            ages, hdr = [], []
            for ts, blob in con.execute(
                    'select timestamp, data from messages where topic_id=? order by timestamp', (tid,)):
                t_h = _hdr_stamp(blob, off)
                ages.append(ts * 1e-9 - t_h)
                hdr.append(t_h)
            if len(hdr) < 2:
                continue
            ages = np.array(ages) * 1e3
            per = np.diff(np.array(hdr)) * 1e3
            # A recording that was paused leaves one giant gap; report the
            # period without it so the frame rate is readable.
            per_ok = per[per < 10 * np.median(per)]
            print(line(f'{name} stamp→receipt', ages))
            print(line(f'{name} header period', per_ok))
        con.close()


# ═════════════════════════════════════════════════════════════════════════════
#  cam — live header-stamp age at receipt (read-only subscription)
# ═════════════════════════════════════════════════════════════════════════════

def cmd_cam(args):
    import json

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import Image

    class Probe(Node):
        def __init__(self):
            super().__init__('latency_budget_cam_probe')
            self.ages = defaultdict(list)
            self.hdr = defaultdict(list)
            self.meta = []
            qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
            for t in args.topics:
                self.create_subscription(Image, t, lambda m, t=t: self.cb(m, t), qos)
            if args.metadata:
                from realsense2_camera_msgs.msg import Metadata
                self.create_subscription(Metadata, args.metadata, self.mcb, qos)

        def now(self):
            return self.get_clock().now().nanoseconds * 1e-9

        def cb(self, m, t):
            self.ages[t].append(self.now() - (m.header.stamp.sec + m.header.stamp.nanosec * 1e-9))
            self.hdr[t].append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
            if len(self.hdr[t]) == 1:
                print(f'  {t}: {m.width}x{m.height} {m.encoding}')

        def mcb(self, m):
            try:
                d = json.loads(m.json_data)
            except Exception:
                return
            self.meta.append((self.now(), m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, d))

    rclpy.init()
    n = Probe()
    t0 = time.time()
    while time.time() - t0 < args.duration:
        rclpy.spin_once(n, timeout_sec=0.1)
    for t in args.topics:
        a = np.array(n.ages[t]) * 1e3
        print(line(f'{t} stamp→receipt', a))
        if len(n.hdr[t]) > 2:
            print(line(f'{t} header period', np.diff(np.array(n.hdr[t])) * 1e3))
    if n.meta:
        rows = []
        for now, th, d in n.meta:
            try:
                rows.append((now, th, float(d['frame_timestamp']), float(d['time_of_arrival']),
                             float(d.get('actual_exposure', 'nan'))))
            except (KeyError, ValueError):
                pass
        if rows:
            r = np.array(rows)
            # librealsense metadata timestamps are in MILLISECONDS (epoch ms under
            # global_time); actual_exposure is in microseconds.
            print(line('metadata: time_of_arrival − header.stamp', r[:, 3] - r[:, 1] * 1e3))
            print(line('metadata: frame_timestamp − header.stamp', r[:, 2] - r[:, 1] * 1e3))
            print(line('metadata: receipt − time_of_arrival', r[:, 0] * 1e3 - r[:, 3]))
            print(line('metadata: actual_exposure', r[:, 4] * 1e-3))
    n.destroy_node()
    rclpy.shutdown()


# ═════════════════════════════════════════════════════════════════════════════
#  perception — the shipped perception code, offline, timed per stage
# ═════════════════════════════════════════════════════════════════════════════

def cmd_perception(args):
    sys.path.insert(0, HERE)
    import compare_vobs as cv                     # bag reading + TF + joints
    import trimesh
    from ament_index_python.packages import get_package_share_directory
    from cv_bridge import CvBridge
    from rclpy.serialization import serialize_message

    from franka_experiments.utils.distance_engine import DistanceEngine
    from franka_experiments.utils.distance_utils import (
        compute_roi, define_control_points, load_extrinsics, load_robot_config)
    from franka_experiments.utils.mask_builder import MaskBuilder
    from franka_experiments.utils.obstacle_sim import InjectedSphere
    from franka_experiments.utils.obstacle_track_pipeline import ObstacleTrackPipeline
    from franka_experiments.utils.perception_msgs import (
        annotate_track_fields, build_cp_messages)
    from franka_experiments.utils.tf_manager import TFManager

    log = _Log(args.verbose)
    cfg = load_robot_config(args.robot_config)
    robot_cfg, mask_cfg, mesh_cfg = cfg['robot'], cfg['mask'], cfg['meshes']
    distance_cfg = dict(cfg['distance'])
    distance_cfg['export_obstacle_cloud'] = True
    trk = cfg.get('tracking', {}) or {}
    R_base, t_base = load_extrinsics(args.extrinsics)
    R32, t32 = R_base.astype(np.float32), t_base.astype(np.float32)

    tf_buf, _ = cv.build_tf_buffer(args.bag)
    ee_link = robot_cfg.get('ee_link', 'fr3_link8')
    tf_mgr = TFManager(tf_buffer=tf_buf, base_frame=robot_cfg['base_frame'],
                       critical_links=robot_cfg.get('critical_links', [ee_link]),
                       cache_max_age_s=None, logger=log)
    mesh_dir = get_package_share_directory(mesh_cfg.get('package', 'franka_description'))
    samples = {n: trimesh.load(os.path.join(mesh_dir, rel), force='mesh')
               .sample(int(mesh_cfg.get('sample_points_per_link', 300)))
               for n, rel in mesh_cfg['files'].items()}
    mask_builder = MaskBuilder(link_mesh_samples=samples, R_base=R_base, t_base=t_base,
                               ee_link=ee_link, mask_cfg=mask_cfg, logger=log)
    engine = DistanceEngine(distance_cfg=distance_cfg, logger=log)
    # The shipped tracker settings, exactly as real_time_distance builds them.
    pipeline = ObstacleTrackPipeline(
        voxel_m=float(trk.get('cluster_voxel_m', 0.02)),
        min_cluster_points=int(trk.get('cluster_min_points', 10)),
        max_clusters=int(trk.get('max_clusters', 16)),
        max_cluster_radius=(float(trk['cluster_max_radius_m'])
                            if trk.get('cluster_max_radius_m') else None),
        depth_jump=float(trk.get('cluster_depth_jump_m', 0.10)),
        contains_tol=float(trk.get('cluster_contains_tol_m', 0.05)),
        q_jerk=float(trk.get('q_jerk', 2.0)), sigma_meas=float(trk.get('sigma_meas_m', 0.01)),
        gate_mahalanobis=float(trk.get('gate_mahalanobis', 3.0)),
        gate_max_m=float(trk.get('gate_max_m', 0.5)),
        confirm_hits=int(trk.get('confirm_hits', 3)), confirm_window=int(trk.get('confirm_window', 5)),
        max_missed=int(trk.get('max_missed', 5)), max_tracks=int(trk.get('max_tracks', 12)))
    sphere = None
    if args.inject:
        sphere = InjectedSphere([0.0, 0.0, 1.6], [0.0, 0.0, -0.5], radius=0.15,
                                period=2.0, amplitude=0.5)
    bridge = CvBridge()

    K = None
    for topic, _, msg in cv.read_bag(args.bag, {args.info_topic}):
        K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        break
    if K is None:
        raise SystemExit('no CameraInfo in the bag')
    mask_builder.set_intrinsics(K)
    cx32, cy32 = np.float32(K[0, 2]), np.float32(K[1, 2])
    fxi32, fyi32 = np.float32(1.0 / K[0, 0]), np.float32(1.0 / K[1, 1])
    step = int(distance_cfg['pixel_step'])
    margin = int(distance_cfg['image_margin_px'])
    thresholds = distance_cfg['thresholds']

    T = defaultdict(list)
    n_frames = n_used = 0
    shape = None
    n_clusters, n_tracks = [], []
    for topic, _, msg in cv.read_bag(args.bag, {args.depth_topic}):
        n_frames += 1
        if args.max_frames and n_frames > args.max_frames:
            break
        t_all = time.perf_counter()
        t0 = time.perf_counter()
        depth = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        T['0 cv_bridge'].append(time.perf_counter() - t0)
        stamp = msg.header.stamp
        t_cap = stamp.sec + stamp.nanosec * 1e-9
        if sphere is not None:
            depth = depth.copy()
            sphere.render(depth, t_cap, K)
        if shape != depth.shape:
            shape = depth.shape
            mask_builder.invalidate(); engine.invalidate_grid_cache()
            engine.reset_lpf(); pipeline.reset()

        t0 = time.perf_counter()
        transforms = tf_mgr.lookup_all(robot_cfg['segment_links'], stamp)
        cps = define_control_points(transforms, robot_cfg, distance_cfg) if transforms else []
        T['1 tf+control points'].append(time.perf_counter() - t0)
        if not cps:
            continue
        H, W = depth.shape
        t0 = time.perf_counter()
        mask_builder.rebuild(transforms, depth.shape)
        roi = compute_roi(mask_builder.search_exclusion_mask, H, W, margin,
                          int(distance_cfg['roi_pad_px'])) or (margin, margin, W - margin, H - margin)
        T['2 mask+roi'].append(time.perf_counter() - t0)
        x0, y0, x1, y1 = roi
        t0 = time.perf_counter()
        cp_results, n_pts = engine.compute(
            depth=depth, cx_f32=cx32, cy_f32=cy32, fx_inv_f32=fxi32, fy_inv_f32=fyi32,
            R_base_f32=R32, t_base_f32=t32, control_points=cps,
            x=np.array([x0, x1]), y=np.array([y0, y1]), step=step,
            search_exclusion_mask=mask_builder.search_exclusion_mask, transforms=transforms,
            ee_source_mask=mask_builder.ee_source_mask,
            dilation_margins_px=mask_builder.dilation_margins_px, frame_stamp=t_cap)
        T['3 distance'].append(time.perf_counter() - t0)
        if cp_results is None:
            continue
        t0 = time.perf_counter()
        pipeline.update(engine.last_obstacle_cloud, R_base, t_base, stamp=t_cap)
        T['4 cluster+track'].append(time.perf_counter() - t0)
        n_clusters.append(len(pipeline.last_clusters))
        n_tracks.append(len(pipeline.tracker.confirmed_tracks()))
        t0 = time.perf_counter()
        _, mld = build_cp_messages(cp_results=cp_results, n_pts=n_pts, stamp=stamp,
                                   frame_id=robot_cfg['base_frame'],
                                   segment_links=robot_cfg.get('segment_links', []),
                                   thresholds=thresholds,
                                   fallback=float(distance_cfg.get('fallback_distance', 2.0)),
                                   zones=cfg.get('zones', {}))
        annotate_track_fields(mld, pipeline, skip_keys=set())
        blob = serialize_message(mld)
        T['5 message build+serialise'].append(time.perf_counter() - t0)
        T['total (depth in hand → bytes out)'].append(time.perf_counter() - t_all)
        n_used += 1

    print(f'== perception replay: {args.bag}  frames={n_frames} used={n_used} '
          f'depth={shape[1]}x{shape[0]} pixel_step={step} inject={bool(args.inject)}')
    for k in sorted(T):
        print(line(k, np.array(T[k]) * 1e3))
    if n_clusters:
        print(f'  clusters/frame med={np.median(n_clusters):.0f} max={max(n_clusters)}  '
              f'confirmed tracks med={np.median(n_tracks):.0f} max={max(n_tracks)}  '
              f'message bytes={len(blob)}')


# ═════════════════════════════════════════════════════════════════════════════
#  cbf — ConstraintBuilder rebuild, the full QP tick, and the retreat authority
# ═════════════════════════════════════════════════════════════════════════════

def _cbf_params(overrides):
    from types import SimpleNamespace

    from franka_experiments.utils.config import load_package_yaml
    cfg = load_package_yaml('franka_experiments', 'config/fr3_control.yaml')
    P = SimpleNamespace(**cfg['params'])
    for k, v in overrides.items():
        setattr(P, k, v)
    return P


def cmd_cbf(args):
    import osqp
    import pinocchio as pin
    import scipy.sparse as sparse

    from franka_experiments.utils.cbf_hard_limits import (
        apply_slew_limit, position_velocity_accel_box)
    from franka_experiments.utils.cbf_qp_assembly import (
        build_osqp_A, build_osqp_bounds, build_row_rhs, pad_rows_to_block, tangential_bias)
    from franka_experiments.utils.cbf_state_rows import (
        FR3_JOINT_KEYS, G_CAP, G_OBS, G_QLIM, G_SC, G_SING, G_SPD, NV, NX, N_SLACK,
        ConstraintBuilder, JointSnap, NomSnap, Obstacle, ObstacleSnap,
        build_optional_row_builders, retreat_accel_available, retreat_speed_available)
    from franka_experiments.utils.config import load_franka_joint_limits
    from franka_experiments.utils.kinematics import CBFKinematics, build_urdf_no_hand

    log = _Log(args.verbose)
    P = _cbf_params(dict(obstacle_velocity_source=args.source,
                         enable_lateral_evasion=bool(args.evasion),
                         enable_uncertainty_margin=bool(args.uncertainty),
                         enable_velocity_feedforward=bool(args.feedforward)))
    jl = load_franka_joint_limits(FR3_JOINT_KEYS)
    lb, ub = -jl['decel_max'], jl['decel_max']
    qdot_max, q_min, q_max = jl['qdot_max'], jl['q_min'], jl['q_max']
    kin = CBFKinematics(pin.buildModelFromUrdf(build_urdf_no_hand()))
    opt = build_optional_row_builders(P, kin, log)
    builder = ConstraintBuilder(P, kin, q_min=q_min, q_max=q_max, acc_lb=lb, acc_ub=ub,
                                logger=log, **opt)

    # ── Control points at a pose, the way real_time_distance lays them out ──
    segs = [('fr3_link3', 'fr3_link4', 2), ('fr3_link4', 'fr3_link5', 2),
            ('fr3_link5', 'fr3_link6', 2), ('fr3_link6', 'fr3_link7', 2),
            ('fr3_link7', 'fr3_link8', 3)]

    def control_points(q):
        kin.update(q, np.zeros(NV), with_jdot=True)
        pos = {n: kin.data.oMf[kin.resolve_frame_id(n)].translation.copy()
               for n in [f'fr3_link{i}' for i in range(3, 9)]}
        cps = []
        for s, e, n in segs:
            ts = [(k + 1) / n for k in range(n)] if e == 'fr3_link8' else \
                 [(k + 1) / (n + 1) for k in range(n)]
            for t in ts:
                cps.append((e, pos[s] + t * (pos[e] - pos[s])))
        return cps

    poses = {'home-ish (bag start)': np.array([0.0, -0.70, 0.0, -2.35, 0.0, 1.57, 0.70]),
             'reaching forward':     np.array([0.0,  0.30, 0.0, -1.20, 0.0, 1.50, 0.70]),
             'near stretched':       np.array([0.0,  0.90, 0.0, -0.60, 0.0, 1.50, 0.70])}

    # ── 0b: retreat authority on the real robot ─────────────────────────────
    v_box = P.velocity_box_margin * qdot_max
    rng = np.random.default_rng(0)
    dirs = np.vstack([np.eye(3), -np.eye(3), rng.normal(size=(200, 3))])
    dirs /= np.linalg.norm(dirs, axis=1, keepdims=True)
    print('== retreat authority (velocity box = velocity_box_margin·qdot_max, static accel box)')
    print('   per control point: v_avail [m/s] min / median over 206 normals, a_avail [m/s²] min / median')
    overall = []
    for name, q in poses.items():
        cps = control_points(q)
        print(f'  -- pose {name}: q={np.round(q, 2).tolist()}')
        for (link, p), k in zip(cps, range(len(cps))):
            fid = kin.resolve_frame_id(link)
            Jp, _ = kin.point_jacobian(fid, p)
            va = np.array([retreat_speed_available(n @ Jp, v_box) for n in dirs])
            aa = np.array([retreat_accel_available(n @ Jp, lb, ub) for n in dirs])
            worst = dirs[int(np.argmin(va))]
            overall.append((va.min(), np.median(va), aa.min(), np.median(aa)))
            print(f'     {link}#{k:<2d} v_avail min={va.min():5.2f} med={np.median(va):5.2f}  '
                  f'a_avail min={aa.min():6.2f} med={np.median(aa):6.2f}  '
                  f'worst n̂={np.round(worst, 2).tolist()}  σ_min(Jp)={np.linalg.svd(Jp, compute_uv=False)[-1]:.3f}')
    o = np.array(overall)
    print(f'  ALL poses/CPs: v_avail min={o[:, 0].min():.2f}  median-of-medians={np.median(o[:, 1]):.2f} m/s ; '
          f'a_avail min={o[:, 2].min():.2f}  median-of-medians={np.median(o[:, 3]):.2f} m/s²')

    # ── Rebuild timing: 11 obstacle rows from a person beside the arm ────────
    q0 = poses['home-ish (bag start)']
    cps = control_points(q0)
    n_side = np.array([0.0, 1.0, 0.0])           # obstacle on the −y side
    v_obs = np.array([0.0, 0.6, 0.0])            # closing along +n̂ at 0.6 m/s
    gaps = np.linspace(0.12, 0.55, len(cps))

    def obstacles(k):
        items = []
        for (link, p), d in zip(cps, gaps):
            items.append(Obstacle(link=link, d=float(d), pr=p.copy(), ph=p - (d + 0.05) * n_side,
                                  conf=1.0, v_vec=v_obs.copy(), frames_seen=6,
                                  vel_cov=(0.05 ** 2) * np.eye(3), track_id=1))
        return tuple(items)

    dt_rebuild = 1.0 / P.cbf_update_rate_hz
    times = []
    con = None
    for k in range(args.iters):
        t = k * dt_rebuild
        q = q0 + 0.02 * np.sin(0.5 * t) * np.ones(NV)
        js = JointSnap(q=q, qdot=0.05 * np.ones(NV), stamp=t)
        obs = ObstacleSnap(items=obstacles(k), stamp=t, t_cap=t - 0.01)
        t0 = time.perf_counter()
        con = builder.build(js, obs, t)
        times.append(time.perf_counter() - t0)
    n_c = con.A.shape[0]
    fam = {g: int(np.count_nonzero(con.group == g)) for g in range(N_SLACK)}
    print(f'== ConstraintBuilder.build  (source={args.source} evasion={bool(args.evasion)} '
          f'uncertainty={bool(args.uncertainty)})  rows n_c={n_c} '
          f'obs={fam[G_OBS]} sc={fam[G_SC]} qlim={fam[G_QLIM]} sing={fam[G_SING]} '
          f'cap={fam[G_CAP]} spd={fam[G_SPD]}')
    print(line('rebuild (FK + all row families)', np.array(times) * 1e3))

    # ── The full QP tick, exactly as _qp_tick assembles it ───────────────────
    P_mat = np.eye(NX)
    for g, rho in ((G_OBS, P.rho_slack), (G_SC, P.rho_slack_self_collision),
                   (G_QLIM, P.rho_slack_joint_limit), (G_SING, P.rho_slack_singularity),
                   (G_CAP, P.rho_slack_retreat), (G_SPD, P.rho_slack_link_speed)):
        P_mat[NV + g, NV + g] = rho
    P_csc = sparse.csc_matrix(P_mat)
    qvec = np.zeros(NX)
    box_lb = np.concatenate([lb, np.zeros(N_SLACK)])
    box_ub = np.concatenate([ub, np.full(N_SLACK, 1e6)])
    dt_qp = 1.0 / P.qp_rate_hz
    qdot = 0.05 * np.ones(NV)
    qdot_cbf = qdot.copy()
    qddot_prev = np.zeros(NV)
    tan = np.zeros(NV)
    esc = np.zeros(NV)
    prob = None
    prev_rows = -1
    t_tick, t_solve, t_setup = [], [], []
    for k in range(args.iters):
        qddot_nom = 0.5 * np.sin(0.01 * k + np.arange(NV))
        t0 = time.perf_counter()
        h_qp, _ = build_row_rhs(con, qdot, qdot_cbf, k0=P.k0_cbf, k1=P.k1_cbf,
                                retreat_horizon=P.retreat_cap_horizon_s,
                                speed_horizon=P.link_speed_horizon_s)
        raw = tangential_bias(qddot_nom, qdot_cbf, con, gain=P.cbf_tangential_gain,
                              engage_margin=P.cbf_tangential_engage_margin,
                              max_bias=P.cbf_tangential_max_bias)
        a_t = P.cbf_tangential_filter_alpha
        tan = a_t * tan + (1 - a_t) * raw
        qddot_nom = qddot_nom + tan
        if con.esc_bias is not None:
            esc = a_t * esc + (1 - a_t) * con.esc_bias
            qddot_nom = qddot_nom + esc
        qvec[:NV] = -qddot_nom
        position_velocity_accel_box(q0, qdot, acc_lb=lb, acc_ub=ub, qdot_max=qdot_max,
                                    v_margin=P.velocity_box_margin, q_min=q_min, q_max=q_max,
                                    q_margin=P.position_margin_rad, brake_eta=P.position_brake_eta,
                                    dt=dt_qp, relax_dt=P.state_box_relax_s,
                                    out_lb=box_lb[:NV], out_ub=box_ub[:NV])
        if P.slew_box_enabled:
            box_lb[:NV], box_ub[:NV] = apply_slew_limit(box_lb[:NV], box_ub[:NV],
                                                        qddot_prev, P.max_qddot_delta)
        G, h = pad_rows_to_block(con.G, h_qp, P.qp_row_block)
        n_rows = G.shape[0]
        l, u = build_osqp_bounds(G, h, box_lb, box_ub)
        if n_rows != prev_rows or prob is None:
            ts = time.perf_counter()
            prev_rows = n_rows
            prob = osqp.OSQP()
            prob.setup(P=P_csc, q=qvec, A=build_osqp_A(G, NV, N_SLACK), l=l, u=u,
                       warm_start=True, max_iter=P.osqp_max_iter, verbose=False)
            t_setup.append(time.perf_counter() - ts)
        else:
            prob.update(q=qvec, l=l, u=u, Ax=build_osqp_A(G, NV, N_SLACK).data)
        ts = time.perf_counter()
        res = prob.solve()
        t_solve.append(time.perf_counter() - ts)
        qddot_prev[:] = res.x[:NV]
        t_tick.append(time.perf_counter() - t0)
    print(f'== QP tick (rows padded {n_c}→{prev_rows}, block={P.qp_row_block}, status={res.info.status})')
    print(line('whole tick: rhs+bias+box+assemble+solve', np.array(t_tick[1:]) * 1e3))
    print(line('  of which OSQP solve', np.array(t_solve[1:]) * 1e3))
    print(line('  OSQP setup() when the block changes', np.array(t_setup) * 1e3))

    # ── qddot_to_torque: M(q)·q̈ + C(q,q̇)·q̇ on the hand model ───────────────
    try:
        from franka_experiments.utils.kinematics import (
            generate_urdf_from_xacro, load_pinocchio_model)
        model, data = load_pinocchio_model(generate_urdf_from_xacro())
        qf = pin.neutral(model)
        vf = np.zeros(model.nv)
        tt = []
        for _ in range(args.iters):
            t0 = time.perf_counter()
            pin.computeAllTerms(model, data, qf, vf)
            _ = np.asarray(data.M) @ np.ones(model.nv) + np.asarray(data.C) @ vf
            tt.append(time.perf_counter() - t0)
        print(line('qddot_to_torque computeAllTerms + M·q̈ + C·q̇', np.array(tt) * 1e3))
    except Exception as exc:                            # pragma: no cover
        print(f'  (torque model timing skipped: {exc})')


# ═════════════════════════════════════════════════════════════════════════════
#  dds — transport latency of the three message shapes, no shared memory
# ═════════════════════════════════════════════════════════════════════════════

def cmd_dds(args):
    import rclpy
    from franka_msgs.msg import LinkDistance, MultiLinkDistance
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import Image
    from std_msgs.msg import Float64MultiArray

    W, H = args.image
    be = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)

    if args.role == 'pub':
        class Pub(Node):
            def __init__(self):
                super().__init__('latency_budget_pub')
                # Same QoS as the real publishers: image reliable (RealSense
                # default), distances best-effort depth 1, qddot reliable.
                self.p_img = self.create_publisher(Image, '/lb/depth', 10)
                self.p_mld = self.create_publisher(MultiLinkDistance, '/lb/per_link', be)
                self.p_qdd = self.create_publisher(Float64MultiArray, '/lb/qddot', 10)
                self.img = Image(height=H, width=W, encoding='16UC1', step=2 * W,
                                 data=bytes(2 * W * H))
                self.mld = MultiLinkDistance()
                self.mld.links = [LinkDistance() for _ in range(11)]
                self.create_timer(1.0 / 30.0, self.slow)
                self.create_timer(1.0 / 100.0, self.fast)

            def slow(self):
                st = self.get_clock().now().to_msg()
                self.img.header.stamp = st
                self.p_img.publish(self.img)
                self.mld.header.stamp = self.get_clock().now().to_msg()
                self.p_mld.publish(self.mld)

            def fast(self):
                m = Float64MultiArray()
                m.data = [self.get_clock().now().nanoseconds * 1e-9] + [0.0] * 6
                self.p_qdd.publish(m)

        rclpy.init()
        n = Pub()
        t0 = time.time()
        while time.time() - t0 < args.duration:
            rclpy.spin_once(n, timeout_sec=0.05)
        n.destroy_node(); rclpy.shutdown()
        return

    class Sub(Node):
        def __init__(self):
            super().__init__('latency_budget_sub')
            self.a = defaultdict(list)
            self.create_subscription(Image, '/lb/depth', self.on_img, 10)
            self.create_subscription(MultiLinkDistance, '/lb/per_link', self.on_mld, be)
            self.create_subscription(Float64MultiArray, '/lb/qddot', self.on_qdd, 10)

        def now(self):
            return self.get_clock().now().nanoseconds * 1e-9

        def on_img(self, m):
            self.a[f'Image {m.width}x{m.height} 16UC1 @30 Hz (reliable)'].append(
                self.now() - (m.header.stamp.sec + m.header.stamp.nanosec * 1e-9))

        def on_mld(self, m):
            self.a['MultiLinkDistance 11 CPs @30 Hz (best-effort)'].append(
                self.now() - (m.header.stamp.sec + m.header.stamp.nanosec * 1e-9))

        def on_qdd(self, m):
            self.a['Float64MultiArray(7) @100 Hz (reliable)'].append(self.now() - m.data[0])

    rclpy.init()
    n = Sub()
    t0 = time.time()
    while time.time() - t0 < args.duration:
        rclpy.spin_once(n, timeout_sec=0.05)
    print(f'== DDS transport, {os.environ.get("RMW_IMPLEMENTATION") or "default rmw"}, '
          f'profiles={os.path.basename(os.environ.get("FASTRTPS_DEFAULT_PROFILES_FILE", "") or "-")}')
    for k in sorted(n.a):
        print(line(k, np.array(n.a[k]) * 1e3, width=48))
    n.destroy_node(); rclpy.shutdown()


# ═════════════════════════════════════════════════════════════════════════════

def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='cmd', required=True)

    p = sub.add_parser('bag'); p.add_argument('bag'); p.set_defaults(fn=cmd_bag)

    p = sub.add_parser('cam')
    p.add_argument('topics', nargs='+')
    p.add_argument('--metadata', default='', help='realsense .../depth/metadata topic')
    p.add_argument('--duration', type=float, default=20.0)
    p.set_defaults(fn=cmd_cam)

    p = sub.add_parser('perception')
    p.add_argument('--bag', required=True)
    p.add_argument('--robot-config', default=os.path.join(PKG, 'config', 'fr3_complete.yaml'))
    p.add_argument('--extrinsics', default=os.path.join(PKG, 'config', 'camera_extrinsics.yaml'))
    p.add_argument('--depth-topic', default='/camera/camera/aligned_depth_to_color/image_raw')
    p.add_argument('--info-topic', default='/camera/camera/aligned_depth_to_color/camera_info')
    p.add_argument('--max-frames', type=int, default=600)
    p.add_argument('--no-inject', dest='inject', action='store_false')
    p.add_argument('--verbose', action='store_true')
    p.set_defaults(fn=cmd_perception)

    p = sub.add_parser('cbf')
    p.add_argument('--source', default='tracker', choices=('residual', 'tracker'))
    p.add_argument('--no-evasion', dest='evasion', action='store_false')
    p.add_argument('--no-uncertainty', dest='uncertainty', action='store_false')
    p.add_argument('--feedforward', action='store_true')
    p.add_argument('--iters', type=int, default=500)
    p.add_argument('--verbose', action='store_true')
    p.set_defaults(fn=cmd_cbf)

    p = sub.add_parser('dds')
    p.add_argument('--role', required=True, choices=('pub', 'sub'))
    p.add_argument('--duration', type=float, default=12.0)
    p.add_argument('--image', type=int, nargs=2, default=(848, 480), metavar=('W', 'H'))
    p.set_defaults(fn=cmd_dds)

    args = ap.parse_args()
    args.fn(args)


if __name__ == '__main__':
    main()
