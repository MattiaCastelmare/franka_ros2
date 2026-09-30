#!/usr/bin/env python3
"""Score the depth → tracker → CBF chain against a thrown ball it can SEE in colour.

WHY THIS SCRIPT EXISTS
----------------------
"The arm does not dodge a thrown ball" has four possible causes — the ball is
not in the distance rows, it is in the rows but never gets a track, the track
has a velocity the CBF does not trust yet, or the CBF acts but too late — and
the launch log alone cannot tell them apart. A brightly coloured ball can: its
position is found in the RGB frame with nothing but a colour threshold, lifted
to 3D with the aligned depth, and moved to the base frame with the same
camera_extrinsics.yaml the perception uses. That is a ground truth that owes
nothing to the pipeline under test, so every stage of it can be timed against
where the ball actually was.

TWO STEPS
---------
``truth``  reads the ORIGINAL recording (color + aligned depth) once and writes
           the ball trajectory to an .npz. Slow (it touches every colour frame)
           and only has to run once per recording.
``score``  reads a bag holding /cbf/per_link_distances, /NS_1/qddot_safe and
           /NS_1/qddot_nom — the original recording, or the output of
           ``bag_replay.launch.py`` after a change — and prints one line per
           pass of the ball near the arm, plus a summary.

Per pass it reports:
  * ``row``    first distance row whose obstacle point is on the ball (within
               --on-ball-m), as lead time before the closest approach;
  * ``trk``    how many of those rows carry a track id, the largest
               frames_seen, the largest |v| the track reported;
  * ``trust``  first row whose track has frames_seen >= the CBF's evidence gate;
  * ``cbf``    first tick where |q̈_safe − q̈_nom| exceeds --act-thr, i.e. the
               filter overrode the nominal command, as lead time.

A replayed CBF runs OPEN LOOP: q and q̇ are the recorded ones, so the arm in
the replay never moves out of the way. ``cbf`` lead time is still meaningful —
it is when the filter WOULD have started to act — but the closest-approach
distance is the recorded one, not what a better filter would have achieved.

USAGE (inside the ROS container, sourced)
-----------------------------------------
    # once: the bag is zstd FILE-compressed, so give it a decompressed copy
    zstd -d rosbag/ball_throws_2/ball_throws_2_0.db3.zstd -o /tmp/bt2/bt2.db3
    python3 scripts/ball_throw_eval.py truth /tmp/bt2 -o rosbag/ball_throws_2_truth.npz
    python3 scripts/ball_throw_eval.py score rosbag/ball_throws_2 \\
        --truth rosbag/ball_throws_2_truth.npz          # the live run itself
    python3 scripts/ball_throw_eval.py score rosbag/replay_xxx \\
        --truth rosbag/ball_throws_2_truth.npz          # a replay after a change

The default colour threshold is for the PINK ball used on 2026-09-30.
"""
from __future__ import annotations

import argparse
import glob
import os
import sqlite3
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)

COLOR = '/camera/camera/color/image_raw'
COLOR_INFO = '/camera/camera/color/camera_info'
ALIGNED = '/camera/camera/aligned_depth_to_color/image_raw'
DIST = '/cbf/per_link_distances'
QDD_SAFE = '/NS_1/qddot_safe'
QDD_NOM = '/NS_1/qddot_nom'


def read_bag(bag_dir, topics):
    """Yield ``(topic, recv_s, msg)`` in log order from every .db3 under bag_dir."""
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    dbs = sorted(glob.glob(os.path.join(bag_dir, '*.db3')))
    if not dbs:
        zst = glob.glob(os.path.join(bag_dir, '*.db3.zstd'))
        hint = (f' (it is zstd-compressed: zstd -d {zst[0]} -o <dir>/x.db3 and pass <dir>)'
                if zst else '')
        raise SystemExit(f'no .db3 under {bag_dir}{hint}')
    for db in dbs:
        con = sqlite3.connect(f'file:{db}?mode=ro', uri=True)
        try:
            meta = {tid: (name, get_message(ttype)) for tid, name, ttype in
                    con.execute('SELECT id, name, type FROM topics') if name in topics}
            if not meta:
                continue
            q = ('SELECT topic_id, timestamp, data FROM messages '
                 f'WHERE topic_id IN ({",".join("?" * len(meta))}) ORDER BY timestamp')
            for tid, ts, blob in con.execute(q, tuple(meta)):
                name, cls = meta[tid]
                yield name, ts * 1e-9, deserialize_message(bytes(blob), cls)
        finally:
            con.close()


def _stamp(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


# ═════════════════════════════════════════════════════════════════════════════
#  truth
# ═════════════════════════════════════════════════════════════════════════════

def ball_mask(rgb, r_min, rg_min, bg_min, g_max):
    r, g, b = (rgb[..., i].astype(np.int16) for i in range(3))
    return (r > r_min) & (r - g > rg_min) & (b - g > bg_min) & (g < g_max)


def cmd_truth(args):
    sys.path.insert(0, PKG)
    from franka_experiments.utils.distance_utils import load_extrinsics

    R, t = load_extrinsics(args.extrinsics)
    K = None
    for _, _, m in read_bag(args.bag, {COLOR_INFO}):
        K = np.array(m.k, float).reshape(3, 3)
        break
    if K is None:
        raise SystemExit(f'no {COLOR_INFO} in {args.bag}')
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]

    depth_by_stamp, pending = {}, []
    out = []                                # stamp, ok, x, y, z, npix
    def flush():
        keep = []
        for st, u, v, ys, xs, n in pending:
            D = depth_by_stamp.get(st)
            if D is None:
                keep.append((st, u, v, ys, xs, n))
                continue
            zz = D[ys, xs]
            zz = zz[zz > 0]
            if len(zz) == 0:
                out.append((st, 0, np.nan, np.nan, np.nan, n))
                continue
            z = float(np.median(zz)) * 1e-3
            pc = np.array([(u - cx) / fx * z, (v - cy) / fy * z, z])
            pb = R @ pc + t
            out.append((st, 1, *pb, n))
        pending[:] = keep

    for topic, _, m in read_bag(args.bag, {COLOR, ALIGNED}):
        st = round(_stamp(m), 4)
        if topic == ALIGNED:
            depth_by_stamp[st] = np.frombuffer(m.data, np.uint16).reshape(m.height, -1)[:, :m.width]
            if len(depth_by_stamp) > 20:
                depth_by_stamp.pop(next(iter(depth_by_stamp)))
        else:
            rgb = np.frombuffer(m.data, np.uint8).reshape(m.height, -1)[:, :3 * m.width]
            rgb = rgb.reshape(m.height, m.width, 3)
            if m.encoding == 'bgr8':
                rgb = rgb[..., ::-1]
            mk = ball_mask(rgb, args.r_min, args.rg_min, args.bg_min, args.g_max)
            n = int(mk.sum())
            if n < args.min_px:
                out.append((st, 0, np.nan, np.nan, np.nan, n))
            else:
                ys, xs = np.nonzero(mk)
                pending.append((st, xs.mean(), ys.mean(), ys, xs, n))
        flush()
    flush()
    A = np.array(sorted(out), float)
    np.savez(args.out, t=A[:, 0], ok=A[:, 1] > 0, p=A[:, 2:5], npix=A[:, 5])
    print(f'{args.out}: {len(A)} colour frames, ball in {int((A[:, 1] > 0).sum())}')


# ═════════════════════════════════════════════════════════════════════════════
#  score
# ═════════════════════════════════════════════════════════════════════════════

def _trust_frames(rate_hz):
    """The CBF's frames_seen gate, as cbf_state_rows derives it."""
    sys.path.insert(0, PKG)
    import yaml
    with open(os.path.join(PKG, 'config', 'fr3_control.yaml')) as f:
        P = yaml.safe_load(f)['params']
    span = float(P.get('obstacle_velocity_min_span_s', 0.0) or 0.0)
    n = int(P.get('obstacle_velocity_min_frames', 1))
    return max(n, int(np.ceil(span * rate_hz))) if span > 0 else n


def passes(tt, ok, P, rows_t, rows_cpr, near_m, min_speed):
    """Contiguous stretches where the MOVING ball is within near_m of a CP."""
    sp = np.full(len(tt), np.nan)
    for i in range(2, len(tt)):
        if ok[i] and ok[i - 2] and tt[i] > tt[i - 2]:
            sp[i] = np.linalg.norm(P[i] - P[i - 2]) / (tt[i] - tt[i - 2])
    near = np.zeros(len(tt), bool)
    dmin = np.full(len(tt), np.nan)
    for i in np.nonzero(ok)[0]:
        k = np.searchsorted(rows_t, tt[i])
        k = min(max(k, 0), len(rows_t) - 1)
        c = rows_cpr[k]
        if c is None or not len(c):
            continue
        dmin[i] = np.min(np.linalg.norm(c - P[i], axis=1))
        near[i] = dmin[i] < near_m
    out, i = [], 0
    while i < len(tt):
        if near[i]:
            j = i
            while j + 1 < len(tt) and (near[j + 1] or (j + 3 < len(tt) and near[j + 1:j + 4].any())):
                j += 1
            k = i + int(np.nanargmin(dmin[i:j + 1]))
            vmax = np.nanmax(sp[max(0, k - 15):k + 1]) if np.isfinite(sp[max(0, k - 15):k + 1]).any() else 0
            if vmax >= min_speed:
                out.append(dict(t0=tt[i], t1=tt[j], tc=tt[k], dmin=dmin[k], v=vmax))
            i = j + 1
        else:
            i += 1
    return out


def truth_velocity(tt, ok, P, half_window=2):
    """Ball velocity from the truth trajectory: a least-squares line over
    ±half_window colour frames (±67 ms at 30 Hz), NaN where the ball is missing."""
    V = np.full((len(tt), 3), np.nan)
    for i in range(len(tt)):
        idx = [j for j in range(i - half_window, i + half_window + 1) if 0 <= j < len(tt) and ok[j]]
        if len(idx) < 4 or not ok[i]:
            continue
        t = tt[idx] - tt[i]
        A = np.c_[t, np.ones_like(t)]
        V[i] = np.linalg.lstsq(A, P[idx], rcond=None)[0][0]
    return V


def cmd_score(args):
    T = np.load(args.truth)
    tt, ok, P = T['t'], T['ok'], T['p']

    rows = []                          # (recv, stamp, [links])
    safe, nom = [], []
    for topic, recv, m in read_bag(args.bag, {DIST, QDD_SAFE, QDD_NOM}):
        if topic == DIST:
            L = []
            for l in m.links:
                if not l.valid:
                    continue
                n = np.array([l.direction.x, l.direction.y, l.direction.z])
                v = np.array([l.obstacle_velocity.x, l.obstacle_velocity.y, l.obstacle_velocity.z])
                C = np.array(l.velocity_covariance, float).reshape(3, 3)
                L.append((np.array([l.closest_point_robot.x, l.closest_point_robot.y, l.closest_point_robot.z]),
                          np.array([l.closest_point_human.x, l.closest_point_human.y, l.closest_point_human.z]),
                          l.distance, l.track_id, l.frames_seen, float(np.linalg.norm(v)),
                          float(n @ v), float(np.sqrt(max(n @ C @ n, 0.0))), v))
            rows.append((recv, _stamp(m), L))
        elif topic == QDD_SAFE:
            safe.append((recv, np.array(m.data[:7])))
        else:
            nom.append((recv, np.array(m.data[:7])))
    if not rows:
        raise SystemExit(f'no {DIST} in {args.bag}')
    rows_t = np.array([r[1] for r in rows])
    rows_cpr = [np.array([l[0] for l in r[2]]) if r[2] else None for r in rows]

    trust_n = args.trust_frames or _trust_frames(args.rate_hz)
    ps = passes(tt, ok, P, rows_t, rows_cpr, args.near_m, args.min_speed)

    t_ref = rows_t[0]
    ts_safe = np.array([s[0] for s in safe]) if safe else np.zeros(0)
    ts_nom = np.array([s[0] for s in nom]) if nom else np.zeros(0)
    dev = np.full(len(safe), np.nan)
    for i, (t_s, q_s) in enumerate(safe):
        k = np.searchsorted(ts_nom, t_s) - 1
        if k >= 0:
            dev[i] = np.linalg.norm(q_s - nom[k][1])

    print(f'== {args.bag}: {len(rows)} distance msgs, {len(ps)} ball passes '
          f'(moving >= {args.min_speed} m/s, within {args.near_m} m of a CP), trust gate {trust_n} frames')
    fast = lambda l: l[3] > 0 and l[6] - args.fast_k * l[7] >= args.fast_v
    print(' pass  t_close  v[m/s] min_gap |  row_lead  on_ball  tracked  max_fs  max|v| | trust_lead  fast_lead | cbf_lead  max_dev')
    S = dict(row=[], trk=0, trust=[], cbf=[], v_ok=0, fast=[])
    vt = truth_velocity(tt, ok, P)
    verr = []            # (|v_track - v_true|, |v_track|/|v_true|, angle deg) on ball rows
    in_pass = np.zeros(len(rows), bool)
    for n, p in enumerate(ps, 1):
        # rows (by capture stamp) in the window before the closest approach
        w = [(r, i) for i, r in enumerate(rows) if p['t0'] - 0.5 <= r[1] <= p['tc'] + 0.05]
        for _, i in w:
            in_pass[i] = True
        on, on_fast = [], []
        for r, i in w:
            k = np.searchsorted(tt, r[1])
            cand = [j for j in (k - 1, k) if 0 <= j < len(tt) and ok[j] and abs(tt[j] - r[1]) < 0.04]
            if not cand or not r[2]:
                continue
            pb = P[min(cand, key=lambda j: abs(tt[j] - r[1]))]
            best = min(r[2], key=lambda l: np.linalg.norm(l[1] - pb))
            if np.linalg.norm(best[1] - pb) < args.on_ball_m:
                on.append((r[0], r[1], best))
                j = min(cand, key=lambda j: abs(tt[j] - r[1]))
                for l in r[2]:
                    if l[3] > 0 and np.linalg.norm(l[1] - pb) < args.on_ball_m and np.isfinite(vt[j]).all():
                        vtr, vtk = vt[j], l[8]
                        sp = np.linalg.norm(vtr)
                        if sp > args.min_speed and np.linalg.norm(vtk) > 1e-6:
                            ang = np.degrees(np.arccos(np.clip(vtk @ vtr / (np.linalg.norm(vtk) * sp), -1, 1)))
                            verr.append((np.linalg.norm(vtk - vtr), np.linalg.norm(vtk) / sp, ang, l[4]))
            if any(np.linalg.norm(l[1] - pb) < args.on_ball_m and fast(l) for l in r[2]):
                on_fast.append(r[0])
        # recv time of the closest approach ≈ capture stamp + the recording's latency
        lat = np.median([r[0] - r[1] for r, _ in w]) if w else 0.0
        tc_recv = p['tc'] + lat
        row_lead = (tc_recv - on[0][0]) * 1e3 if on else np.nan
        tracked = sum(1 for o in on if o[2][3] > 0)
        max_fs = max((o[2][4] for o in on if o[2][3] > 0), default=0)
        max_v = max((o[2][5] for o in on), default=0.0)
        tr = [o for o in on if o[2][3] > 0 and o[2][4] >= trust_n]
        trust_lead = (tc_recv - tr[0][0]) * 1e3 if tr else np.nan
        fast_lead = (tc_recv - on_fast[0]) * 1e3 if on_fast else np.nan
        wi = (ts_safe >= tc_recv - 0.6) & (ts_safe <= tc_recv + 0.1)
        act = ts_safe[wi][dev[wi] > args.act_thr] if wi.any() else []
        cbf_lead = (tc_recv - act[0]) * 1e3 if len(act) else np.nan
        mdev = np.nanmax(dev[wi]) if wi.any() else np.nan
        print(f' {n:4d}  {p["tc"] - t_ref:7.2f}  {p["v"]:5.1f}  {p["dmin"]:6.2f} | {row_lead:7.0f}ms  {len(on):6d}  '
              f'{tracked:6d}  {max_fs:6d}  {max_v:5.2f} | {trust_lead:8.0f}ms  {fast_lead:7.0f}ms | {cbf_lead:6.0f}ms  {mdev:6.1f}')
        S['row'].append(row_lead); S['trust'].append(trust_lead); S['cbf'].append(cbf_lead)
        S['fast'].append(fast_lead)
        S['trk'] += tracked > 0
        S['v_ok'] += max_v >= 0.5 * p['v']
    if ps:
        f = lambda x: np.nanmedian(x) if np.isfinite(x).any() else np.nan
        print(f'  summary: ball in rows {np.isfinite(S["row"]).sum()}/{len(ps)} (median lead {f(S["row"]):.0f} ms) | '
              f'tracked {S["trk"]}/{len(ps)} | |v|>=half true speed {S["v_ok"]}/{len(ps)} | '
              f'trusted {np.isfinite(S["trust"]).sum()}/{len(ps)} (median lead {f(S["trust"]):.0f} ms) | '
              f'CBF acted {np.isfinite(S["cbf"]).sum()}/{len(ps)} (median lead {f(S["cbf"]):.0f} ms)')
        print(f'  fast-track (v_n - {args.fast_k}σ >= {args.fast_v} m/s) on the ball in '
              f'{np.isfinite(S["fast"]).sum()}/{len(ps)} passes (median lead {f(S["fast"]):.0f} ms)')
    if verr:
        E = np.array(verr)
        print(f'  track velocity vs truth on the ball ({len(E)} rows): |error| median {np.median(E[:, 0]):.2f} m/s, '
              f'speed ratio median {np.median(E[:, 1]):.2f} (p10 {np.percentile(E[:, 1], 10):.2f}), '
              f'direction error median {np.median(E[:, 2]):.0f} deg')
        for lo, hi in ((0, 6), (6, 10), (10, 15), (15, 25), (25, 10 ** 6)):
            m = (E[:, 3] >= lo) & (E[:, 3] < hi)
            if m.any():
                print(f'    frames_seen {lo:2d}-{min(hi, 999) - 1:3d}: n={m.sum():4d}  |err| {np.median(E[m, 0]):.2f} m/s  '
                      f'ratio {np.median(E[m, 1]):.2f}  dir {np.median(E[m, 2]):3.0f} deg')
    # How often the same test fires on something that is NOT a ball pass:
    # a fast-trusted track anywhere else is a candidate false trigger.
    dur = rows[-1][1] - rows[0][1]
    fp = [r for r, ip in zip(rows, in_pass) if not ip and any(fast(l) for l in r[2])]
    fp_close = [r for r in fp if any(fast(l) and l[2] < 0.3 for l in r[2])]
    print(f'  fast-track rows outside the passes: {len(fp)} msgs ({len(fp) / dur * 60:.0f}/min), '
          f'{len(fp_close)} with gap < 0.3 m — people moving count too, not only noise')


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    sub = ap.add_subparsers(dest='cmd', required=True)
    a = sub.add_parser('truth')
    a.add_argument('bag')
    a.add_argument('-o', '--out', required=True)
    a.add_argument('--extrinsics', default=os.path.join(PKG, 'config', 'camera_extrinsics.yaml'))
    a.add_argument('--r-min', type=int, default=140)
    a.add_argument('--rg-min', type=int, default=80)
    a.add_argument('--bg-min', type=int, default=20)
    a.add_argument('--g-max', type=int, default=120)
    a.add_argument('--min-px', type=int, default=8)
    a.set_defaults(fn=cmd_truth)
    s = sub.add_parser('score')
    s.add_argument('bag')
    s.add_argument('--truth', required=True)
    s.add_argument('--near-m', type=float, default=0.6, help='ball-to-CP-centre distance that counts as a pass')
    s.add_argument('--min-speed', type=float, default=1.5, help='ignore the ball when slower than this [m/s]')
    s.add_argument('--on-ball-m', type=float, default=0.12, help='row obstacle point this close to the ball = on the ball')
    s.add_argument('--act-thr', type=float, default=5.0, help='|q̈_safe − q̈_nom| that counts as the CBF acting [rad/s²]')
    s.add_argument('--rate-hz', type=float, default=90.0)
    s.add_argument('--fast-k', type=float, default=2.0, help='fast-track test: v_n - k*sigma_n >= fast-v')
    s.add_argument('--fast-v', type=float, default=1.0)
    s.add_argument('--trust-frames', type=int, default=0, help='0 = derive from fr3_control.yaml')
    s.set_defaults(fn=cmd_score)
    args = ap.parse_args()
    args.fn(args)


if __name__ == '__main__':
    main()
