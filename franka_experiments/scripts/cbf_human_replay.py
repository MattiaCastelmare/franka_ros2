#!/usr/bin/env python3
"""Recorded people against the CBF filter, closed loop: what the human prediction changes.

Paired comparison of filter configurations on the people of the lab bags. Every
configuration sees the same person, the same reference motion and the same perception
output, so the differences between them are the configuration's alone.

INPUT
-----
A replay .npz of ``human_kf_consistency.py replay``: the real HumanTracker (MediaPipe and
the Kalman filter, current config) run frame by frame on a bag. Per camera frame it holds
the HumanArmState the tracker published (estimate, velocity, covariance blocks,
frames_seen), the measurements the filter took, and the robot joints of the recording.

METHOD
------
arm      Double integrator at qp_rate_hz, as cbf_scenarios.py. The nominal q̈ tracks the
         RECORDED joint trajectory (PD + feedforward): the motion the person reacted to.
person   The published states, delivered T_CAM after their capture stamp, turned into
         per-control-point rows as human_distance builds them (fr3_complete.yaml control
         points, HumanArmGeometry capsules, fill_track_fields) and parsed as
         cbf_safety_filter parses them (labelled_links, latency fields, src 1). Rows are
         rebuilt at cbf_update_rate_hz at the CURRENT q, on the latest delivered state.
filter   The shipped ConstraintBuilder and QP (cbf_scenarios.Filter), with fr3_control.yaml
         and the overrides easy_torque passes (cbf_human_scenarios.easy_torque_params).
truth    The measured keypoints (accepted, own depth), median of 3, interpolated across
         gaps up to MAX_GAP_S: no filter, no lag. Every configuration is scored against it.

Approach events are the intervals where the measured person comes within EVENT_GAP of the
RECORDED arm. They do not depend on the configuration, so the smallest true gap of each
event is compared configuration by configuration (paired).

The person is open loop: in the lab they reacted to the arm of the recording, not to the
one replayed here. The comparison is between configurations, not with the lab run.

USAGE (inside the ROS container, workspace sourced, this checkout on PYTHONPATH)
--------------------------------------------------------------------------------
    python3 scripts/human_kf_consistency.py replay /bags/cbf_human --out /tmp/cbf_human.npz
    python3 scripts/cbf_human_replay.py /tmp/cbf_human.npz --configs off soft
--shift-m D moves the recorded person D metres toward the robot (a stress test: the lab
person kept clear of the arm, so the barrier hardly worked).

Configs: off | soft[:T[:rho[:k_sigma]]] (defaults from fr3_control.yaml) |
hard:T (the removed rigid prediction: run a copy of the old package) |
nohuman (no human rows: how far the reference tracking alone strays).
"""
from __future__ import annotations

import argparse
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import cbf_scenarios as cs  # noqa: E402
from cbf_human_scenarios import easy_torque_params  # noqa: E402
from franka_msgs.msg import LinkDistance, MultiLinkDistance  # noqa: E402
from franka_experiments.nodes.cbf_safety_filter import CBFSafetyFilter  # noqa: E402
from franka_experiments.utils.capsule_geometry import (  # noqa: E402
    HumanArmGeometry, RobotGeometry)
from franka_experiments.utils.cbf_state_rows import (  # noqa: E402
    GROUP_NAMES, NV, N_SLACK, JointSnap, Obstacle, ObstacleSnap)
from franka_experiments.utils.distance_utils import load_robot_config  # noqa: E402
from franka_experiments.utils.human_utils import (  # noqa: E402
    define_control_points, fill_track_fields)
from franka_experiments.utils.perception_msgs import labelled_links  # noqa: E402

T_CAM = 0.06          # [s] capture -> per_link_distances at the filter
MAX_GAP_S = 0.25      # [s] longest measurement gap the truth interpolates across
EVENT_GAP = 0.30      # [m] an approach: the person within this of the recorded arm
KP, KD = 100.0, 20.0  # nominal tracking of the recorded joints (critically damped)
SRC_HUMAN = 1         # the human topic is the first extra_distance_topics entry
HUMAN_RADII = dict(upper_arm_radius=0.075, forearm_radius=0.065, hand_radius=0.075)
LINKS = [f'fr3_link{i}' for i in range(9)]


# ═════════════════════════════════════════════════════════════════════════════
#  The recording
# ═════════════════════════════════════════════════════════════════════════════

class Recording:
    """A tracker replay: published states, measured truth and recorded joints."""

    def __init__(self, path):
        d = np.load(path)
        self.name = os.path.splitext(os.path.basename(path))[0]
        self.t = d['t']
        self.sides = [str(s) for s in d['sides']]
        need = [f'{self.sides[0]}/pub', 'joints_t']
        missing = [k for k in need if k not in d.files]
        if missing:
            raise SystemExit(f'{path}: no {missing} (replay it again with this checkout)')
        self.pub = {}
        self.truth_src = {}
        for s in self.sides:
            vis = d[f'{s}/vis']
            self.pub[s] = dict(pub=d[f'{s}/pub'], pos=d[f'{s}/pub_pos'], vel=d[f'{s}/pub_vel'],
                               valid=d[f'{s}/pub_valid'], cov=d[f'{s}/pub_cov'],
                               seen=d[f'{s}/pub_seen'], conf=vis.mean(axis=1))
            ok = (d[f'{s}/measured'] & (d[f'{s}/own_depth'] > 0.0)
                  & np.isfinite(d[f'{s}/z']).all(axis=2))
            self.truth_src[s] = (d[f'{s}/z'], ok)
        jt = d['joints_t']
        order = np.argsort(jt)
        self.jt, self.jq, self.jdq = jt[order], d['joints_q'][order], d['joints_dq'][order]
        self.t0 = max(self.t[0], self.jt[0])
        self.t1 = min(self.t[-1], self.jt[-1])

    def shift_toward_robot(self, distance):
        """Move the whole person rigidly by `distance` [m], horizontally, toward the robot
        base: the same motion, closer approaches. Exact for the filter's state and the
        truth alike (the filter is translation invariant; its covariances are kept)."""
        pts = np.concatenate([self.pub[s]['pos'][self.pub[s]['valid']] for s in self.sides])
        centre = np.median(pts, axis=0)
        u = -np.array([centre[0], centre[1], 0.0])
        u /= max(np.linalg.norm(u), 1e-9)
        for s in self.sides:
            self.pub[s]['pos'] = self.pub[s]['pos'] + distance * u
            z, ok = self.truth_src[s]
            self.truth_src[s] = (z + distance * u, ok)
        return u

    def reference(self, ts):
        """Recorded q, q̇ and a smoothed q̈ at the times ts (absolute stamps)."""
        q = np.stack([np.interp(ts, self.jt, self.jq[:, j]) for j in range(NV)], axis=1)
        dq = np.stack([np.interp(ts, self.jt, self.jdq[:, j]) for j in range(NV)], axis=1)
        dt = ts[1] - ts[0]
        w = max(1, int(round(0.02 / dt)))           # 20 ms box on the velocity first
        kern = np.ones(w) / w
        dq_s = np.stack([np.convolve(dq[:, j], kern, mode='same') for j in range(NV)], axis=1)
        ddq = np.gradient(dq_s, dt, axis=0)
        return q, dq, ddq

    def truth(self, ts):
        """Measured keypoints at ts per side: (n, 4, 3) positions and (n, 4) validity."""
        out = {}
        for s in self.sides:
            z, ok = self.truth_src[s]
            pos = np.full((len(ts), 4, 3), np.nan)
            valid = np.zeros((len(ts), 4), dtype=bool)
            for i in range(4):
                idx = np.flatnonzero(ok[:, i])
                if idx.size < 3:
                    continue
                tk = self.t[idx]
                zk = z[idx, i]
                # median of 3 consecutive measurements: one outlier never makes the truth
                zm = zk.copy()
                zm[1:-1] = np.median(np.stack([zk[:-2], zk[1:-1], zk[2:]]), axis=0)
                j = np.clip(np.searchsorted(tk, ts), 1, len(tk) - 1)
                t_lo, t_hi = tk[j - 1], tk[j]
                inside = (ts >= t_lo) & (ts <= t_hi) & (t_hi - t_lo <= MAX_GAP_S)
                a = np.clip((ts - t_lo) / np.maximum(t_hi - t_lo, 1e-9), 0.0, 1.0)[:, None]
                pos[:, i] = (1 - a) * zm[j - 1] + a * zm[j]
                valid[:, i] = inside
            out[s] = (pos, valid)
        return out


# ═════════════════════════════════════════════════════════════════════════════
#  human_distance, offline
# ═════════════════════════════════════════════════════════════════════════════

class HumanDistance:
    """The rows human_distance publishes and cbf_safety_filter parses, without ROS."""

    def __init__(self, F, sides):
        cfg = load_robot_config(os.path.join(cs.PKG, 'config', 'fr3_complete.yaml'))
        self.robot_cfg, self.dist_cfg = cfg['robot'], cfg['distance']
        self.F = F
        self.fids = {n: F.kin.resolve_frame_id(n) for n in LINKS}
        self.geom = RobotGeometry(definitions=[])
        self.human = HumanArmGeometry(**HUMAN_RADII)
        self.sides = sides

    def control_points(self):
        """Control points at the kinematics' current q (the caller updated it)."""
        tr = {n: (self.F.kin.data.oMf[f].rotation.copy(), self.F.kin.data.oMf[f].translation.copy())
              for n, f in self.fids.items()}
        return define_control_points(tr, self.robot_cfg, self.dist_cfg)

    def truth_capsules(self, truth, j):
        """Capsules of the measured person at scoring instant j (geometry only)."""
        return [cap for s in self.sides
                for cap in self.human.build_capsules(truth[s][0][j], valid=truth[s][1][j])]

    def capsules(self, states):
        """states: side -> (pos, vel, valid, cov, seen) or None. As distance_loop."""
        caps = []
        for side_index, side in enumerate(self.sides):
            st = states.get(side)
            if st is None or not np.any(st[2]):
                continue
            pos, vel, valid, cov, seen = st
            prefix = f'{side}_' if len(self.sides) > 1 else ''
            for cap in self.human.build_capsules(pos, valid=valid):
                cap['name'] = f"{prefix}{cap['name']}"
                cap['track_id'] = 1 + 3 * side_index + cap['indices'][0]
                cap['velocities'] = vel
                cap['covariances'] = tuple(cov)
                cap['frames_seen'] = seen
                caps.append(cap)
        return caps

    def message(self, cps, caps, confidence):
        msg = MultiLinkDistance()
        for cp in cps:
            info = self.geom.minimum_distance_to_human([cp], caps)
            ld = LinkDistance()
            ld.robot_link_name = cp['source_capsule']
            ld.distance = max(0.0, float(info['distance']))
            fill_track_fields(ld, info['capsule'], info['alpha'])
            (ld.closest_point_robot.x, ld.closest_point_robot.y,
             ld.closest_point_robot.z) = map(float, info['robot_position'])
            (ld.closest_point_human.x, ld.closest_point_human.y,
             ld.closest_point_human.z) = map(float, info['closest_human_point'])
            ld.valid = True
            ld.confidence = float(confidence)
            msg.links.append(ld)
        return msg

    @staticmethod
    def obstacles(msg):
        """As CBFSafetyFilter._on_distances, for the human source."""
        out = []
        for cp_label, ld in labelled_links(msg):
            if not ld.valid:
                continue
            out.append(Obstacle(
                link=ld.robot_link_name, d=float(ld.distance),
                pr=np.array([ld.closest_point_robot.x, ld.closest_point_robot.y,
                             ld.closest_point_robot.z]),
                ph=np.array([ld.closest_point_human.x, ld.closest_point_human.y,
                             ld.closest_point_human.z]),
                conf=float(ld.confidence),
                v_vec=np.array([ld.obstacle_velocity.x, ld.obstacle_velocity.y,
                                ld.obstacle_velocity.z]),
                frames_seen=int(ld.frames_seen),
                vel_cov=np.asarray(ld.velocity_covariance, dtype=np.float64).reshape(3, 3),
                track_id=int(ld.track_id), cp_label=f'{SRC_HUMAN}:{cp_label}', src=SRC_HUMAN,
                **CBFSafetyFilter._latency_fields(ld), **CBFSafetyFilter._range_field(ld)))
        return tuple(out)

    def gap(self, cps, caps):
        """Smallest signed surface gap between the control points and the capsules."""
        if not caps:
            return np.inf
        return float(self.geom.minimum_distance_to_human(cps, caps)['distance'])


# ═════════════════════════════════════════════════════════════════════════════
#  One closed-loop run
# ═════════════════════════════════════════════════════════════════════════════

def run(rec, config, *, t_start=0.0, duration=0.0):
    # 'nohuman': the filter without the person, i.e. the reference tracking alone
    use_human = config != 'nohuman'
    P = easy_torque_params(config if use_human else 'off')
    F = cs.Filter(P)
    hd = HumanDistance(F, rec.sides)
    dt = F.dt
    every = max(1, int(round(P.qp_rate_hz / P.cbf_update_rate_hz)))
    t0 = rec.t0 + t_start
    t1 = rec.t1 if not duration else min(rec.t1, t0 + duration)
    ts = t0 + np.arange(int((t1 - t0) / dt)) * dt
    q_ref, dq_ref, ddq_ref = rec.reference(ts)
    tb = ts[::every]                                  # rebuild (and scoring) instants
    truth = rec.truth(tb)

    q, qdot = q_ref[0].copy(), dq_ref[0].copy()
    frame = 0                                         # next camera frame to deliver
    latest = {s: None for s in rec.sides}
    latest_cap, latest_conf = None, {s: 0.0 for s in rec.sides}
    con = None
    out = dict(gap=[], gap_ref=[], dev=[], qdd=[], nom=[], slack=[])
    fl = F.fids['fr3_link8']
    for k, t in enumerate(ts):
        # ── perception: tracker states delivered T_CAM after their capture stamp
        while frame < len(rec.t) and rec.t[frame] + T_CAM <= t:
            for s in rec.sides:
                p = rec.pub[s]
                if p['pub'][frame]:
                    latest[s] = (p['pos'][frame], p['vel'][frame], p['valid'][frame],
                                 p['cov'][frame], p['seen'][frame])
                    latest_conf[s] = p['conf'][frame]
                    latest_cap = rec.t[frame]
            frame += 1
        if k % every == 0:
            j = k // every
            # the recorded arm against the measured person: the configuration-free events
            F.kin.update(q_ref[k], dq_ref[k], with_jdot=True)
            cps_ref = hd.control_points()
            flange_ref = F.kin.data.oMf[fl].translation.copy()
            true_caps = hd.truth_capsules(truth, j)
            out['gap_ref'].append(hd.gap(cps_ref, true_caps))
            # the replayed arm
            F.kin.update(q, qdot, with_jdot=True)
            cps = hd.control_points()
            out['gap'].append(hd.gap(cps, true_caps))
            out['dev'].append(float(np.linalg.norm(F.kin.data.oMf[fl].translation - flange_ref)))
            caps = hd.capsules(latest) if use_human and latest_cap is not None else []
            if caps:
                valid_sides = [s for s in rec.sides
                               if latest[s] is not None and np.any(latest[s][2])]
                conf = np.mean([latest_conf[s] for s in valid_sides])
                obs = hd.obstacles(hd.message(cps, caps, conf))
                con = F.builder.build(JointSnap(q=q, qdot=qdot, stamp=t),
                                      ObstacleSnap(items=obs, stamp=t, t_cap=latest_cap), t)
            else:
                con = None
        qddot_nom = ddq_ref[k] + KP * (q_ref[k] - q) + KD * (dq_ref[k] - qdot)
        qddot, slack, _, _, _ = F.tick(t, con, q, qdot, qddot_nom)
        if k % every == 0:
            out['qdd'].append(qddot.copy())
            out['nom'].append(qddot_nom.copy())
            out['slack'].append(np.asarray(slack).copy())
        qdot = qdot + qddot * dt
        q = q + qdot * dt
    res = {key: np.asarray(v) for key, v in out.items()}
    res.update(dt_score=dt * every, d_safe=P.d_safe, levels=list(F.levels), config=config)
    return res


# ═════════════════════════════════════════════════════════════════════════════
#  Scores
# ═════════════════════════════════════════════════════════════════════════════

def events(gap_ref, dt):
    """[start, end) index pairs where the measured person is within EVENT_GAP of the
    recorded arm, merged across dips shorter than 0.3 s."""
    close = np.isfinite(gap_ref) & (gap_ref < EVENT_GAP)
    idx = np.flatnonzero(np.diff(np.concatenate([[0], close.astype(int), [0]])))
    spans = list(zip(idx[::2], idx[1::2]))
    merged = []
    for a, b in spans:
        if merged and (a - merged[-1][1]) * dt < 0.3:
            merged[-1] = (merged[-1][0], b)
        else:
            merged.append((a, b))
    return merged


def score(r, ev):
    dt, d_safe = r['dt_score'], r['d_safe']
    gap = r['gap']
    fin = np.isfinite(gap)
    inside = fin & (gap < d_safe)
    qdd = r['qdd']
    jerk = np.linalg.norm(np.diff(qdd, axis=0), axis=1) / dt
    pad = int(round(0.5 / dt))
    ev_min = np.array([np.nanmin(np.where(fin[a:b + pad], gap[a:b + pad], np.inf))
                       for a, b in ev]) if ev else np.zeros(0)
    sl = r['slack']
    return dict(
        min_gap=float(np.min(gap[fin])) if fin.any() else np.inf,
        inside_s=float(inside.sum() * dt),
        depth_s=float(np.sum(np.where(inside, d_safe - gap, 0.0)) * dt),
        dev_max=float(np.max(r['dev'])), dev_mean=float(np.mean(r['dev'])),
        effort=float(np.mean(np.linalg.norm(qdd - r['nom'], axis=1))),
        jerk_p99=float(np.percentile(jerk, 99)),
        ev_min=ev_min,
        slack={GROUP_NAMES[g]: float(sl[:, g].max()) for g in range(min(N_SLACK, sl.shape[1]))},
        fallbacks=int(r['levels'][1] + r['levels'][2]))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('replays', nargs='+', help='human_kf_consistency.py replay .npz files')
    ap.add_argument('--configs', nargs='+', default=['off', 'soft'])
    ap.add_argument('--start-s', type=float, default=0.0)
    ap.add_argument('--duration-s', type=float, default=0.0)
    ap.add_argument('--shift-m', type=float, default=0.0,
                    help='stress test: move the person this far toward the robot base')
    ap.add_argument('--json', default='', help='write every score here')
    args = ap.parse_args(argv)
    dump = {}
    for path in args.replays:
        rec = Recording(path)
        print(f'\n== {rec.name}: {rec.t1 - rec.t0:.0f} s, sides {rec.sides}')
        if args.shift_m:
            u = rec.shift_toward_robot(args.shift_m)
            print(f'   person moved {100 * args.shift_m:.0f} cm toward the robot base '
                  f'(direction {np.round(u, 2).tolist()})')
        results = {c: run(rec, c, t_start=args.start_s, duration=args.duration_s)
                   for c in args.configs}
        first = results[args.configs[0]]
        ev = events(first['gap_ref'], first['dt_score'])
        print(f'   {len(ev)} approach events (measured person within {EVENT_GAP:.2f} m of '
              f'the recorded arm), d_safe {first["d_safe"]:.2f} m')
        print(f"   {'config':18s} {'min gap':>8s} {'in d_safe':>9s} {'depth*t':>8s} "
              f"{'dev max':>8s} {'dev mean':>8s} {'effort':>7s} {'jerk p99':>9s} "
              f"{'s_obs':>6s} {'s_pred':>6s} {'fallbk':>6s}")
        scores = {c: score(r, ev) for c, r in results.items()}
        for c, s in scores.items():
            print(f"   {c:18s} {100 * s['min_gap']:+7.1f}c {s['inside_s']:8.2f}s "
                  f"{100 * s['depth_s']:7.2f}c {100 * s['dev_max']:6.1f}cm "
                  f"{100 * s['dev_mean']:6.1f}cm {s['effort']:7.2f} {s['jerk_p99']:9.0f} "
                  f"{s['slack'].get('obs', 0.0):6.3f} {s['slack'].get('pred', 0.0):6.3f} "
                  f"{s['fallbacks']:6d}")
        base = scores[args.configs[0]]['ev_min']
        for c in args.configs[1:]:
            diff = scores[c]['ev_min'] - base
            fin = np.isfinite(diff)
            if not fin.any():
                continue
            diff = diff[fin]
            print(f"   events, {c} - {args.configs[0]}: min gap {100 * np.mean(diff):+.1f} cm "
                  f"on average (median {100 * np.median(diff):+.1f}, worst event "
                  f"{100 * np.min(diff):+.1f}, best {100 * np.max(diff):+.1f}); "
                  f"larger in {int(np.sum(diff > 0.005))}, smaller in "
                  f"{int(np.sum(diff < -0.005))} of {diff.size}")
        dump[rec.name] = {c: {k: (v.tolist() if isinstance(v, np.ndarray) else v)
                              for k, v in s.items()} for c, s in scores.items()}
        dump[rec.name]['events'] = [[int(a), int(b)] for a, b in ev]
    if args.json:
        with open(args.json, 'w') as f:
            json.dump(dump, f, indent=1)


if __name__ == '__main__':
    main()
