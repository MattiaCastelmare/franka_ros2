#!/usr/bin/env python3
"""Closed-loop human-arm scenarios for the CBF filter: how the human rows use prediction.

Same headless filter as scripts/cbf_scenarios.py (ConstraintBuilder, the QP as the node
assembles it, OSQP with its fallback ladder, FR3 kinematics, a double integrator for the
joints), with the HUMAN perception chain emulated the way easy_torque runs it:

    the hand's true position, measured at 30 Hz with white noise (sigma_meas),
    -> an ArmKalmanFilter (the shipped constant-velocity filter and its config)
    -> delivered to the CBF t_cam later, as human_distance rows (src = 1) carrying the
       filter's position, velocity and covariance blocks.

The robot holds its pose (or follows a slow Cartesian line); the hand reaches toward the
flange and stops short of it, reaches through it, passes by, or stands still. Every
configuration sees the same hand and the same noise (same seed).

Reported per run: the TRUE minimum barrier (gap - d_safe) and time below zero, how far the
flange was pushed off its task (max deviation), the commanded jerk, the slack paid per
family, and the task error at the end.

    python3 scripts/cbf_human_scenarios.py                       # every scenario, default configs
    python3 scripts/cbf_human_scenarios.py --configs off soft:0.3:1 --only reach_stop
Config syntax: off | hard:<horizon> (the removed rigid prediction, for old code only) |
soft[:<horizon>[:<rho_slack_prediction>[:<k_sigma>]]] (values left out: fr3_control.yaml).
"""
from __future__ import annotations

import argparse
import os
import sys

import numpy as np
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import cbf_scenarios as cs  # noqa: E402
from franka_experiments.utils.arm_kf import ArmKalmanFilter  # noqa: E402
from franka_experiments.utils.cbf_state_rows import (  # noqa: E402
    GROUP_NAMES, NV, N_SLACK, JointSnap, Obstacle, ObstacleSnap)

R_HAND = 0.04          # [m] the hand as a sphere
FPS = 30.0
T_CAM = 0.06           # [s] capture -> CBF (MediaPipe + transport)


def easy_torque_params(config: str):
    """fr3_control.yaml with the overrides easy_torque passes to cbf_safety_filter."""
    with open(os.path.join(cs.PKG, 'config', 'fr3_control.yaml')) as f:
        raw = yaml.safe_load(f)['params']
    P = cs.SimpleNamespace(**raw)
    P.obstacle_velocity_source = 'tracker'
    P.enable_vobs_in_hdot = False          # easy_torque's default
    parts = config.split(':')
    if parts[0] == 'hard':
        if not hasattr(P, 'extra_prediction_horizon_s'):
            raise SystemExit('this code has no rigid prediction (extra_prediction_horizon_s)')
        P.extra_prediction_horizon_s = float(parts[1])
    elif parts[0] == 'soft':
        if not hasattr(P, 'enable_human_prediction'):
            raise SystemExit('this code has no soft prediction (enable_human_prediction)')
        P.enable_human_prediction = True
        # Values left out keep fr3_control.yaml's
        for name, value in zip(('human_prediction_horizon_s', 'rho_slack_prediction',
                                'human_prediction_k_sigma'), parts[1:]):
            setattr(P, name, float(value))
    elif parts[0] == 'off':
        if hasattr(P, 'enable_human_prediction'):
            P.enable_human_prediction = False
    else:
        raise SystemExit(f'unknown config {config}')
    return P


def min_jerk(s):
    s = np.clip(s, 0.0, 1.0)
    return 10 * s**3 - 15 * s**4 + 6 * s**5, (30 * s**2 - 60 * s**3 + 30 * s**4)


class Hand:
    """Scripted hand: a list of (duration, start, end) min-jerk segments (holds have
    start == end)."""

    def __init__(self, segments):
        self.segments = segments

    def state(self, t):
        t0 = 0.0
        for dur, a, b in self.segments:
            if t <= t0 + dur:
                s, ds = min_jerk((t - t0) / dur)
                return a + s * (b - a), ds / dur * (b - a)
            t0 += dur
        return self.segments[-1][2].copy(), np.zeros(3)


def scenario(name, flange0):
    """The hand's script for a scenario, relative to the flange's rest position."""
    out = flange0.copy()
    out[2] = 0.0
    out = out / np.linalg.norm(out)                    # horizontal, away from the base
    lat = np.cross([0.0, 0.0, 1.0], out)               # horizontal, sideways
    reach = R_HAND + cs.R_CAP                          # centre distance at zero gap
    far = flange0 + out * (reach + 0.70)
    at = lambda gap: flange0 + out * (reach + gap)     # noqa: E731
    hold = lambda p, t: (t, p, p)                      # noqa: E731
    if name == 'reach_stop':          # reaches fast, stops 30 cm short: no danger
        return Hand([hold(far, 0.5), (0.75, far, at(0.30)), hold(at(0.30), 2.0),
                     (0.75, at(0.30), far), hold(far, 0.5)])
    if name == 'reach_stop_close':    # stops 20 cm short: inside the anticipation band
        return Hand([hold(far, 0.5), (0.75, far, at(0.20)), hold(at(0.20), 2.0),
                     (0.75, at(0.20), far), hold(far, 0.5)])
    if name == 'reach_through':       # keeps coming to where the flange was
        return Hand([hold(far, 0.5), (1.2, far, at(0.0)), hold(at(0.0), 2.0)])
    if name == 'fast_reach':          # same at ~1.5 m/s peak
        return Hand([hold(far, 0.5), (0.85, far, at(0.0)), hold(at(0.0), 2.0)])
    if name == 'pass_by':             # crosses in front at 25 cm, never closing on it
        a = at(0.25) - lat * 0.8
        b = at(0.25) + lat * 0.8
        return Hand([hold(a, 0.5), (1.6, a, b), hold(b, 1.0)])
    if name == 'still_near':          # stands 25 cm away: only the filter's noise moves it
        return Hand([hold(at(0.25), 4.0)])
    raise SystemExit(f'unknown scenario {name}')


SCENARIOS = ('reach_stop', 'reach_stop_close', 'reach_through', 'fast_reach', 'pass_by',
             'still_near')


def run(name, config, *, seed=0, sigma_meas=0.02):
    P = easy_torque_params(config)
    F = cs.Filter(P)
    rng = np.random.default_rng(seed)
    q, qdot = cs.Q0.copy(), np.zeros(NV)
    fl = F.fids['fr3_link8']
    F.control_points(q, qdot)
    flange0 = F.kin.data.oMf[fl].translation.copy()
    hand = scenario(name, flange0)
    T = sum(seg[0] for seg in hand.segments)
    kf = ArmKalmanFilter(dt=1.0 / FPS, process_accel_std=3.0, measurement_std=0.03,
                         initial_position_std=0.05, initial_velocity_std=1.0,
                         visibility_threshold=0.0)
    dt = F.dt
    n = int(round(T / dt))
    next_frame, pending, msg, con = 0.0, [], None, None
    rec = dict(h=[], dev=[], jerk=[], slack=[], qdd=[])
    for k in range(n):
        t = k * dt
        cps = F.control_points(q, qdot)
        flange = F.kin.data.oMf[fl].translation.copy()
        p_true, _ = hand.state(t)
        # ── human perception: 30 Hz measurement -> Kalman filter -> delivered T_CAM later
        if t >= next_frame - 1e-9:
            next_frame += 1.0 / FPS
            z = p_true + rng.normal(0.0, sigma_meas, 3)
            kf.step(np.tile(z, (4, 1)), np.ones(4), np.ones(4), dt=1.0 / FPS)
            P_pp, P_vv, P_pv = (blk[0] for blk in kf.get_covariances())
            pending.append((t + T_CAM, t, kf.x[0].copy(), P_pp, P_pv, P_vv,
                            int(kf.update_count[0])))
        while pending and pending[0][0] <= t:
            msg = pending.pop(0)
        # ── rebuild at 50 Hz, rows built at the CURRENT q on the latest message
        if k % 2 == 0 and msg is not None:
            _, t_cap, x, P_pp, P_pv, P_vv, seen = msg
            items = []
            for j, (link, p_cp) in enumerate(cps):
                delta = p_cp - x[:3]
                dist = float(np.linalg.norm(delta))
                gap = dist - R_HAND - cs.R_CAP
                if gap > 0.7:
                    continue
                ph = x[:3] + R_HAND * delta / max(dist, 1e-9)
                items.append(Obstacle(link=link, d=max(gap, 0.0), pr=p_cp, ph=ph, conf=1.0,
                                      v_vec=x[3:].copy(), frames_seen=seen, vel_cov=P_vv,
                                      track_id=1, a_vec=None, pos_cov=P_pp, pv_cov=P_pv,
                                      src=1, cp_label=f'1:{link}#{j}'))
            con = (F.builder.build(JointSnap(q=q, qdot=qdot, stamp=t),
                                   ObstacleSnap(items=tuple(items), stamp=t, t_cap=t_cap), t)
                   if items else None)
        # ── task: hold the rest pose
        qddot_nom = -12.0 * (q - cs.Q0) - 2.0 * np.sqrt(12.0) * qdot
        qddot, slack, _, _, _ = F.tick(t, con, q, qdot, qddot_nom)
        # ── truth
        h_true = min(float(np.linalg.norm(p_cp - p_true)) - R_HAND - cs.R_CAP - P.d_safe
                     for _, p_cp in cps)
        rec['h'].append(h_true)
        rec['dev'].append(float(np.linalg.norm(flange - flange0)))
        rec['jerk'].append(float(np.linalg.norm(qddot - rec['qdd'][-1])) / dt if rec['qdd'] else 0.0)
        rec['qdd'].append(qddot.copy())
        rec['slack'].append(np.asarray(slack).copy())
        qdot = qdot + qddot * dt
        q = q + qdot * dt
    h = np.array(rec['h'])
    sl = np.array(rec['slack'])
    return dict(
        scenario=name, config=config, min_h=float(h.min()),
        neg_s=float((h < 0.0).sum() * dt), dev_max=float(np.max(rec['dev'])),
        dev_mean=float(np.mean(rec['dev'])), jerk_p99=float(np.percentile(rec['jerk'][1:], 99)),
        slack={GROUP_NAMES[g]: float(sl[:, g].max()) for g in range(N_SLACK)},
        levels=list(F.levels), final_err=float(np.linalg.norm(q - cs.Q0)))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--configs', nargs='+', default=['off'])
    ap.add_argument('--only', nargs='+', default=list(SCENARIOS))
    ap.add_argument('--seed', type=int, default=0)
    args = ap.parse_args(argv)
    print(f"{'scenario':17s} {'config':16s} {'min h':>7s} {'h<0':>6s} {'dev max':>8s} "
          f"{'dev mean':>8s} {'jerk p99':>9s} {'s_obs':>6s} {'s_pred':>6s} {'fallbk':>7s}")
    for name in args.only:
        for config in args.configs:
            r = run(name, config, seed=args.seed)
            print(f"{name:17s} {config:16s} {100 * r['min_h']:+6.1f}c {r['neg_s']:5.2f}s "
                  f"{100 * r['dev_max']:6.1f}cm {100 * r['dev_mean']:6.1f}cm {r['jerk_p99']:9.1f} "
                  f"{r['slack']['obs']:6.3f} {r['slack'].get('pred', 0.0):6.3f} "
                  f"{r['levels'][1] + r['levels'][2]:7d}")


if __name__ == '__main__':
    main()
