#!/usr/bin/env python3
"""Read-only measurements behind docs/approach_speed_envelope.md (hypotheses H1-H4, H6).

Nothing here publishes, writes into the package, or touches the safety path: it
reads recorded bags and, for ``depth``, runs the SHIPPED real_time_distance node
in-process on the raw depth stream.

    rates   <bag>                 H3  frame rate / bursts / jitter / latency, recorded
    depth   <bag> <truth.npz>     H1+H2  does the depth image contain the ball, and is it a
                                  cluster of its own (inside the node, per frame, per pass)
    assoc                         H4  tracker association ceiling vs v*dt (synthetic, shipped TrackManager)
    causes  <eval output .txt>    classify every pass of ball_throw_eval.py's table into the 4 causes

Run inside the container (sourced), PYTHONPATH=<package source>:$PYTHONPATH.
"""
from __future__ import annotations

import argparse
import os
import re
import sqlite3
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
sys.path.insert(0, HERE)

DEPTH = '/camera/camera/depth/image_rect_raw'
DIST = '/cbf/per_link_distances'


def _pct(x, ps=(50, 95, 99)):
    x = np.asarray(x, float)
    return ' '.join(f'p{p}={np.percentile(x, p):7.2f}' for p in ps) + f' max={x.max():7.2f} (n={len(x)})'


def _stamps(bag, topic):
    """[(recv_s, stamp_s)] in log order for one topic (header stamps only: no deserialisation of images)."""
    import glob
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    out = []
    for db in sorted(glob.glob(os.path.join(bag, '*.db3'))):
        con = sqlite3.connect(f'file:{db}?mode=ro', uri=True)
        row = con.execute('SELECT id, type FROM topics WHERE name=?', (topic,)).fetchone()
        if row:
            cls = get_message(row[1])
            for ts, blob in con.execute('SELECT timestamp, substr(data,1,12) FROM messages WHERE topic_id=? ORDER BY timestamp', (row[0],)):
                # CDR header: 4 bytes encapsulation, then stamp sec(int32) nanosec(uint32) for every stamped msg here
                sec, nsec = np.frombuffer(bytes(blob[4:12]), dtype='<i4,<u4')[0]
                out.append((ts * 1e-9, sec + nsec * 1e-9))
        con.close()
    return np.array(out)


# ═════════════════════════════════════════════════════════════════════════════
def cmd_rates(a):
    D = _stamps(a.bag, DEPTH)
    R = _stamps(a.bag, DIST)
    print(f'== {a.bag}')
    if len(D):
        recv, st = D[:, 0], D[:, 1]
        first = np.r_[True, np.diff(st) != 0]
        U = D[first]
        dur = U[-1, 1] - U[0, 1]
        print(f'depth {DEPTH}: {len(D)} msgs, {len(U)} distinct capture stamps over {dur:.1f} s -> '
              f'{len(D) / dur:.1f} msg/s, {len(U) / dur:.1f} distinct frames/s (duplicates {100 * (1 - len(U) / len(D)):.0f} %)')
        dts = np.diff(U[:, 1]) * 1e3
        print(f'  capture-stamp period of distinct frames [ms]: {_pct(dts)}')
        dr = np.diff(U[:, 0]) * 1e3
        print(f'  RECEIPT inter-arrival of distinct frames [ms]: {_pct(dr)}')
        # bursts: frames received less than 4 ms after the previous one
        burst = (dr < 4.0).mean() * 100
        print(f'  {burst:.0f} % of distinct frames arrive <4 ms after the previous one (a burst); '
              f'receipt gaps >25 ms: {(dr > 25).mean() * 100:.0f} %')
        print(f'  capture -> receipt latency, distinct frames [ms]: {_pct((U[:, 0] - U[:, 1]) * 1e3)}')
    if len(R):
        recv, st = R[:, 0], R[:, 1]
        dur = st[-1] - st[0]
        # the node publishes per processed frame; a gap in capture stamps = frames the compute loop dropped
        print(f'distances {DIST}: {len(R)} msgs over {dur:.1f} s -> {len(R) / dur:.1f} Hz '
              f'(vs {len(D) and (len(U) / (U[-1, 1] - U[0, 1])):.1f} Hz distinct camera frames)')
        dts = np.diff(st) * 1e3
        print(f'  capture-stamp period [ms]: {_pct(dts)}')
        print(f'  receipt inter-arrival [ms]: {_pct(np.diff(recv) * 1e3)}')
        print(f'  capture -> recorder receipt [ms]: {_pct((recv - st) * 1e3)}')
        if len(D):
            # fraction of camera frames that reached a published message
            cam = set(np.round(U[:, 1], 4))
            pub = set(np.round(st, 4))
            print(f'  published frames that are camera frames: {len(cam & pub)} of {len(pub)}; '
                  f'camera frames never published (dropped or empty-heartbeat): {len(cam - pub)} of {len(cam)}')


# ═════════════════════════════════════════════════════════════════════════════
def _ball_at(tt, ok, P, t, tol=0.04):
    j = int(np.clip(np.searchsorted(tt, t), 1, len(tt) - 1))
    good = [k for k in (j - 1, j) if ok[k] and abs(tt[k] - t) < tol]
    if not good:
        return None
    if len(good) == 2:
        w = (t - tt[j - 1]) / max(tt[j] - tt[j - 1], 1e-9)
        return (1 - w) * P[j - 1] + w * P[j]
    return P[good[0]]


def _passes_of(bag, truth):
    """The ball passes of a LIVE bag, found exactly the way ball_throw_eval.py ``score`` finds them."""
    from ball_throw_eval import passes, read_bag, _stamp
    T = np.load(truth)
    rows_t, cpr = [], []
    for _, _, m in read_bag(bag, {DIST}):
        rows_t.append(_stamp(m))
        cpr.append(np.array([[l.closest_point_robot.x, l.closest_point_robot.y, l.closest_point_robot.z]
                             for l in m.links if l.valid]) if m.links else None)
    return T['t'], T['ok'], T['p'], passes(T['t'], T['ok'], T['p'], np.array(rows_t), cpr, 0.6, 1.5)


def cmd_depth(a):
    """H1 + H2, per frame, per pass, INSIDE the shipped real_time_distance node (every distinct raw
    depth frame is processed: no queue drops, so this is the information question, not the timing one).

    For each raw depth frame around a pass, the colour-tracked ball (truth, base frame) is moved to the
    camera frame and looked up in the node's own products:
      roi      the ball's pixel is inside the ROI box the node searched (roi_pad_px)
      excl     ... and not under the robot exclusion mask
      px       depth pixels at the ball's distance (|D - z| < 0.10 m) inside a ball-sized disc, and what a
               ball of 3.6 cm radius should cover there; grid = how many land on the pixel_step grid
      cluster  the smallest cluster whose covering sphere contains the ball: (points, radius);
               'own' = a ball-sized cluster (<= --ball-r) within 0.12 m of the ball, 'merged' = only larger
               clusters contain it, 'none' = nothing within 0.15 m
      track    nearest confirmed track within 0.15 m: frames_seen and speed
    """
    import rclpy
    import yaml
    from cv_bridge import CvBridge

    import compare_vobs as cv
    from ball_throw_eval import read_bag
    from franka_experiments.utils.tf_manager import TFManager

    tt, ok, PB, ps = _passes_of(a.bag, a.truth)
    print(f'{len(ps)} passes in {a.bag}')
    import importlib.util
    spec = importlib.util.spec_from_file_location('tcs', os.path.join(PKG, 'launch', 'torque_control_stack.launch.py'))
    tcs = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(tcs)
    cfg_path = tcs._rtd_config_with_overrides(
        os.path.join(PKG, 'config', 'fr3_complete.yaml'), tracking=True, sim_obstacle=False,
        depth_rate_hz=tcs._profile_fps(tcs._DEFAULTS['camera_depth_profile']), visualize=False)
    if a.perception_overrides:
        import tempfile
        c = yaml.safe_load(open(cfg_path))
        def merge(d, s):
            for k, v in s.items():
                if isinstance(v, dict) and isinstance(d.get(k), dict):
                    merge(d[k], v)
                else:
                    d[k] = v
        merge(c, yaml.safe_load(open(a.perception_overrides)) or {})
        f = tempfile.NamedTemporaryFile('w', suffix='.yaml', delete=False)
        yaml.safe_dump(c, f); f.close(); cfg_path = f.name
    rclpy.init(args=['--ros-args', '-p', f'robot_config_path:={cfg_path}',
                     '-p', f'camera_extrinsics_path:={os.path.join(PKG, "config", "camera_extrinsics.yaml")}',
                     '-p', f'multi_obstacle_k:={int(tcs._DEFAULTS["multi_obstacle_k"])}', '-p', 'publish_overlay_image:=false'])
    from franka_experiments.nodes.real_time_distance import RealTimeDistance
    node = RealTimeDistance()
    # the node reads the same parameters the launch gives it; only its inputs are replaced
    buf, _ = cv.build_tf_buffer(a.bag)
    node.tf_mgr = TFManager(tf_buffer=buf, base_frame=node.robot_cfg['base_frame'],
                            critical_links=node.robot_cfg.get('critical_links', [node.ee_link]),
                            cache_max_age_s=None, logger=node.get_logger())
    caught = []
    node.per_link_dist_pub = type('S', (), {'publish': lambda s, m: caught.append(m)})()
    for _, _, m in read_bag(a.bag, {'/camera/camera/depth/camera_info'}):
        node.camera_info_callback(m)
        break
    K = node.K
    R, t = node.R_base, node.t_base               # camera -> base
    step = int(node.distance_cfg['pixel_step'])
    kmin = int(node.config['tracking']['cluster_min_points'])
    bridge = CvBridge()
    out = []
    for p in ps:
        lo, hi = p['tc'] - a.pre, p['tc'] + 0.05
        rows = []
        seen_stamp = None
        for _, _, msg in read_bag(a.bag, {DEPTH}):
            st = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            if st < lo - 0.2:
                continue
            if st > hi:
                break
            if st == seen_stamp:
                continue
            seen_stamp = st
            depth = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            caught.clear()
            node._process_depth_impl((depth, msg))
            if st < lo:
                continue                      # warm-up frames so the tracker state is the one of a running node
            b = _ball_at(tt, ok, PB, st)
            r = dict(t=st - p['tc'], ball=b is not None)
            if b is not None:
                pc = R.T @ (b - t)
                z = float(pc[2])
                u = K[0, 0] * pc[0] / z + K[0, 2]
                v = K[1, 1] * pc[1] / z + K[1, 2]
                H, W = depth.shape
                x0, y0, x1, y1 = node.roi_bounds
                r.update(z=z, u=u, v=v, inroi=bool(x0 <= u < x1 and y0 <= v < y1))
                iu, iv = int(round(u)), int(round(v))
                if 0 <= iu < W and 0 <= iv < H:
                    r['excl'] = bool(node.mask_builder.search_exclusion_mask[iv, iu])
                rpx = K[0, 0] * a.ball_r_m / z
                rr = int(np.ceil(rpx * 1.3)) + 1
                v0, v1, u0, u1 = max(iv - rr, 0), min(iv + rr + 1, H), max(iu - rr, 0), min(iu + rr + 1, W)
                win = depth[v0:v1, u0:u1].astype(np.float32) * 1e-3
                vv, uu = np.mgrid[v0:v1, u0:u1]
                disc = (vv - v) ** 2 + (uu - u) ** 2 <= (1.3 * rpx) ** 2
                r['holes'] = int(((win == 0) & disc).sum())
                like = disc & (np.abs(win - z) < 0.10) & (win > 0)
                r['px'] = int(like.sum())
                r['px_exp'] = int(np.pi * rpx ** 2)
                if r['inroi']:
                    gv = (vv - y0) % step == 0
                    gu = (uu - x0) % step == 0
                    r['grid'] = int((like & gv & gu).sum())
                else:
                    r['grid'] = 0
                # clusters of the node, camera frame
                cl = node.track_pipeline.last_clusters if node.track_pipeline is not None else []
                r['ncl'] = len(cl)
                inside = [c for c in cl if np.linalg.norm(c.centroid_cam - pc) <= c.radius + 0.05]
                near = [c for c in cl if np.linalg.norm(c.centroid_cam - pc) <= 0.12]
                own = [c for c in near if c.radius <= a.ball_cluster_r]
                if own:
                    c = min(own, key=lambda c: np.linalg.norm(c.centroid_cam - pc))
                    r['cl'] = 'own'; r['cl_n'] = c.n_points; r['cl_r'] = c.radius
                elif inside:
                    c = min(inside, key=lambda c: c.radius)
                    r['cl'] = 'merged'; r['cl_n'] = c.n_points; r['cl_r'] = c.radius
                else:
                    r['cl'] = 'none'
                # nearest confirmed track
                best = None
                for tr in node.track_pipeline.tracker.confirmed_tracks():
                    d = np.linalg.norm(tr.position - b)
                    if d < 0.15 and (best is None or d < best[0]):
                        best = (d, tr)
                if best:
                    r['trk_fs'] = int(best[1].frames_seen)
                    r['trk_v'] = float(np.linalg.norm(best[1].velocity))
            rows.append(r)
        out.append((p, rows))
    # ── report ────────────────────────────────────────────────────────────────
    print('\n pass  v    | first frame (s before closest approach) at which ... | longest merged run')
    print('            ball in ROI | grid>=kmin | own cluster | track | track |v|>=.5*v_true | merged before own')
    def first(rows, f):
        for r in rows:
            if r['ball'] and f(r):
                return -r['t']
        return float('nan')
    summ = []
    for n, (p, rows) in enumerate(out, 1):
        v = p['v']
        t_roi = first(rows, lambda r: r.get('inroi'))
        t_grid = first(rows, lambda r: r.get('grid', 0) >= kmin)
        t_own = first(rows, lambda r: r.get('cl') == 'own')
        t_trk = first(rows, lambda r: r.get('trk_fs', 0) >= 1)
        t_vel = first(rows, lambda r: r.get('trk_fs', 0) >= 3 and r.get('trk_v', 0) >= 0.5 * v)
        mer = [r for r in rows if r['ball'] and r.get('cl') == 'merged' and -r['t'] > (t_own if np.isfinite(t_own) else 0)]
        t_mer = (max(-r['t'] for r in mer) - min(-r['t'] for r in mer)) if mer else 0.0
        print(f' {n:3d} {v:4.1f} |  {t_roi:8.2f}   {t_grid:8.2f}   {t_own:8.2f}   {t_trk:8.2f}   {t_vel:8.2f}   | merged span {t_mer:5.2f} s')
        summ.append((t_roi, t_grid, t_own, t_trk, t_vel))
        if a.csv:
            with open(a.csv, 'a') as f:
                for r in rows:
                    f.write(f'{n},{r["t"]:.4f},' + ','.join(str(r.get(k, '')) for k in
                            ('ball', 'z', 'inroi', 'excl', 'holes', 'px', 'px_exp', 'grid', 'ncl', 'cl', 'cl_n', 'cl_r', 'trk_fs', 'trk_v')) + '\n')
    S = np.array(summ, float)
    print(' median ' + ' '.join(f'{np.nanmedian(S[:, i]):9.2f}' for i in range(5)))
    # H1: pixel statistics on the last 0.6 s before closest approach
    px = [(r['z'], r['px'], r['px_exp'], r['grid'], r['holes']) for _, rows in out for r in rows
          if r['ball'] and r.get('inroi') and -0.6 <= r['t'] <= 0]
    if px:
        X = np.array(px, float)
        print(f'\nH1 in the last 0.6 s: ball range median {np.median(X[:, 0]):.2f} m; depth pixels at the ball range inside the disc '
              f'{np.median(X[:, 1]):.0f} (a ball would cover {np.median(X[:, 2]):.0f}); on the pixel_step grid {np.median(X[:, 3]):.1f} '
              f'(cluster_min_points {kmin}); zero-depth holes in the disc {np.median(X[:, 4]):.0f}')
        for lo_, hi_ in ((0, 1.0), (1.0, 1.5), (1.5, 2.0), (2.0, 2.5), (2.5, 4.5)):
            m = (X[:, 0] >= lo_) & (X[:, 0] < hi_)
            if m.any():
                print(f'   range {lo_:.1f}-{hi_:.1f} m: n={int(m.sum()):4d} expected px {np.median(X[m, 2]):5.0f}  seen {np.median(X[m, 1]):5.0f} '
                      f'({100 * np.median(X[m, 1] / np.maximum(X[m, 2], 1)):.0f} %)  grid {np.median(X[m, 3]):4.1f}  P(grid>={kmin}) {100 * np.mean(X[m, 3] >= kmin):.0f} %')


# ═════════════════════════════════════════════════════════════════════════════
def cmd_assoc(a):
    """H4: the tracker's association ceiling. One object, centroid measured every ``dt`` with ``noise``
    (isotropic, metres), moving at ``v`` m/s from rest-at-birth; how long until the shipped
    TrackManager holds one confirmed track on it, and how often it is lost/respawned (new id)."""
    import yaml
    from franka_experiments.utils.obstacle_track_pipeline import ObstacleTrackPipeline  # noqa: F401
    from franka_experiments.utils.obstacle_tracker import TrackManager
    cfg = yaml.safe_load(open(os.path.join(PKG, 'config', 'fr3_complete.yaml')))['tracking']
    nominal = a.fps

    from franka_experiments.utils.rate_scaling import frames_for

    def counts():          # the node's own rule (real_time_distance._track_counts)
        window = frames_for(cfg['confirm_window_s'], nominal, minimum=3)
        hits = max(2, min(window, int(round(window * cfg['confirm_hits_frac']))))
        return hits, window, frames_for(cfg['max_coast_s'], nominal, minimum=2)
    hits, win, coast = counts()

    def mk():
        return TrackManager(
            q_jerk=cfg['q_jerk'], sigma_meas=cfg['sigma_meas_m'], sigma_v0=cfg.get('sigma_v0', 1.0),
            sigma_a0=cfg.get('sigma_a0', 5.0), gate_mahalanobis=cfg['gate_mahalanobis'],
            gate_max_m=cfg['gate_max_m'], confirm_hits=hits, confirm_window=win, max_missed=coast,
            max_tracks=cfg.get('max_tracks', 12), evict_stale_tentative=cfg.get('evict_stale_tentative', False),
            imm_enabled=cfg.get('imm_enabled', False))
    rng = np.random.default_rng(0)
    print(f'TrackManager as shipped: confirm {hits}-of-{win}, coast {coast} frames, gate_mahalanobis '
          f'{cfg["gate_mahalanobis"]}, gate_max_m {cfg["gate_max_m"]}, sigma_v0 {cfg.get("sigma_v0")}, '
          f'imm {cfg.get("imm_enabled")} | frame period {1000 / nominal:.1f} ms, centroid noise {a.noise * 100:.1f} cm')
    print(' v[m/s]  v*dt[cm]  ids_used  first_confirmed[frames]  frames_with_track/frames  |v_est|/v at +10 frames')
    for v in a.speeds:
        res = []
        for rep in range(a.reps):
            tm = mk()
            dt = 1.0 / nominal
            p0 = np.array([0.5, 1.5, 0.8])
            vec = np.array([0.0, -1.0, 0.0]) * v
            ids, first, with_trk, N = set(), None, 0, int(a.frames)
            vest = np.nan
            for k in range(N):
                z = p0 + vec * k * dt + rng.normal(0, a.noise, 3)
                tm.step([z], dt)
                conf = tm.confirmed_tracks()
                if conf:
                    with_trk += 1
                    ids.update(int(t.track_id) for t in conf)
                    if first is None:
                        first = k + 1
                    if k == (first or 0) + 9:
                        vest = float(np.linalg.norm(conf[0].velocity)) / max(v, 1e-9)
            res.append((len(ids), first if first is not None else np.nan, with_trk / N, vest))
        r = np.array(res, float)
        print(f' {v:5.1f}  {100 * v / nominal:7.1f}  {np.nanmedian(r[:, 0]):6.0f}   {np.nanmedian(r[:, 1]):8.1f}  '
              f'{np.nanmean(r[:, 2]) * 100:14.0f} %          {np.nanmedian(r[:, 3]):.2f}')


# ═════════════════════════════════════════════════════════════════════════════
def cmd_causes(a):
    """Classify every pass in the table ball_throw_eval.py ``score`` prints (stdin or file).

    Order = the order the chain can fail in; a pass gets the FIRST cause that applies:
      1 not-in-rows     no distance row on the ball at all (row_lead nan)
      2 untracked       rows on the ball, none carrying a track id (tracked == 0)
      3 velocity-untrusted   tracked, but neither the span gate nor the fast-track test ever passed
      4 too-late        the CBF acted, but with less than --need-ms before closest approach
      -  ok             the CBF acted with at least --need-ms of warning
    """
    txt = open(a.file).read() if a.file != '-' else sys.stdin.read()
    rows = []
    for ln in txt.splitlines():
        m = re.match(r'\s*(\d+)\s+([\d.]+)\s+([\d.]+)\s+([\d.]+)\s+\|\s+(\w+)ms\s+(\d+)\s+(\d+)\s+(\d+)\s+([\d.]+)\s+\|\s+(\w+)ms\s+(\w+)ms\s+\|\s+(\w+)ms\s+([\d.]+)', ln)
        if m:
            f = lambda s: float('nan') if s == 'nan' else float(s)
            rows.append(dict(n=int(m[1]), v=float(m[3]), gap=float(m[4]), row=f(m[5]), on=int(m[6]), trk=int(m[7]),
                             fs=int(m[8]), trust=f(m[10]), fast=f(m[11]), cbf=f(m[12])))
    cnt = {}
    for r in rows:
        if not np.isfinite(r['row']):
            c = '1 not-in-rows'
        elif r['trk'] == 0:
            c = '2 untracked'
        elif not (np.isfinite(r['trust']) or np.isfinite(r['fast'])):
            c = '3 velocity-untrusted'
        elif not np.isfinite(r['cbf']) or r['cbf'] < a.need_ms:
            c = '4 too-late'
        else:
            c = '- ok'
        cnt[c] = cnt.get(c, 0) + 1
        r['cause'] = c
    print(f'{len(rows)} passes (need {a.need_ms:.0f} ms of CBF lead): ' + ', '.join(f'{k}: {v}' for k, v in sorted(cnt.items())))
    return rows


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='cmd', required=True)
    s = sub.add_parser('rates'); s.add_argument('bag'); s.set_defaults(fn=cmd_rates)
    s = sub.add_parser('depth'); s.add_argument('bag'); s.add_argument('truth')
    s.add_argument('--pre', type=float, default=1.2); s.add_argument('--ball-r-m', type=float, default=0.036)
    s.add_argument('--ball-cluster-r', type=float, default=0.10); s.add_argument('--csv', default='')
    s.add_argument('--perception-overrides', default=''); s.set_defaults(fn=cmd_depth)
    s = sub.add_parser('assoc')
    s.add_argument('--fps', type=float, default=90.0)
    s.add_argument('--noise', type=float, default=0.01)
    s.add_argument('--speeds', type=float, nargs='*', default=[1, 2, 3, 4, 5, 6, 8, 10, 15])
    s.add_argument('--frames', type=int, default=60)
    s.add_argument('--reps', type=int, default=20)
    s.set_defaults(fn=cmd_assoc)
    s = sub.add_parser('causes'); s.add_argument('file'); s.add_argument('--need-ms', type=float, default=450.0)
    s.set_defaults(fn=cmd_causes)
    a = ap.parse_args()
    a.fn(a)


if __name__ == '__main__':
    main()
