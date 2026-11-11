#!/usr/bin/env python3
"""F-2-11 downstream (F-3-2, answers Q-2-1): what the REAL obstacle-row builder does with a held entry.

Pipeline exercised, all baseline source: `distance_utils.define_control_points` (11 control points
at q_home) -> `perception_msgs.build_cp_messages` -> `labelled_links` -> the `_on_distances`
mapping (transcribed in `_builder3.py`, checked by `ast`) -> `ConstraintBuilder.build` with the
hardware-default parameters and the real `CBFKinematics` / `control_point_jacobian`.

Fingertip control point at q_home; obstacle point 0.21 m in front of it along +x (surface gap
d = 0.16 m, true normal n_true = (-1, 0, 0), obstacle -> control point). Perception at 90 Hz.
Every other control point publishes no obstacle (invalid entry).

Scenario A (residual path): untracked static obstacle; the arm approaches it at 0.10 m/s along +x
  (q_dot = J^+ (0.1, 0, 0); the configuration q is kept at q_home, a snapshot approximation:
  the builder only sees (q, q_dot) per call). 0.5 s normal frames, then the bounded LPF hold:
  9 frames (0.10 s) with d frozen and closest_point_human = 0 (F-2-11), then 2 invalid frames,
  then the obstacle reappears.
Scenario B (tracker path): the arm is still; a confirmed track (frames_seen >= 60) closes at
  0.5 m/s from a gap of 0.40 m (0.15 m when the hold starts). Same hold sequence; during the hold the track fields are zero (F-3-2).

Checks:
  1. the held entry passes every gate: valid, conf >= min_confidence, d < horizon         (FACT)
  2. a row is built for it, with normal pr/|pr| (base-radial), 122 deg from n_true         (NUM)
  3. the identity guard fires entering and leaving the hold (diag_ident_reset)              (FACT)
  4. A: the held row's normal sees the true approach as SEPARATION: n_held^T v_tip > 0       (NUM)
  5. A: the k1 term's h_dot, A q_dot - v_obs, before vs during the hold                (REPRO/FACT)
  6. B: v_obs of the row drops to 0 for the whole hold; the true gap shrinks by 5 cm while the
     row's h stays at the held value                                                       (FACT)
Observed per-frame traces are printed (INFO) and recorded in results/.

Needs: the ROS 2 container (franka_msgs, builtin_interfaces, xacro, franka_description,
Pinocchio). Read-only.
"""
import os
import sys

sys.dont_write_bytecode = True
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _builder3 as B  # noqa: E402

C3, C = B.C3, B.C
import numpy as np  # noqa: E402
from builtin_interfaces.msg import Time  # noqa: E402
from franka_experiments.utils.distance_engine import ControlPointResult  # noqa: E402
from franka_experiments.utils.distance_utils import define_control_points  # noqa: E402
from franka_experiments.utils.perception_msgs import build_cp_messages, labelled_links  # noqa: E402

R = C.Report('check_held_row (chapter 07 §8.1; F-2-11 -> F-3-2, Q-2-1)')
R.fact('_builder3 transcription matches the Obstacle(...) keywords of _on_distances', B.check_mapping())
P = C3.params_namespace('hw_default')
cfg = B.robot_cfg()
HZ, GAP, D_OFF = 90.0, 0.16, 0.21
DT = 1.0 / HZ
N_NORM, N_HOLD, N_GONE, N_BACK = 45, 9, 2, 20   # 0.5 s, the 0.10 s hold, 2 invalid, reappearance


def tf_at(kin, q):
    kin.update(q, np.zeros(7), with_jdot=False)
    m, d = kin.model, kin.data
    return {n: (d.oMf[m.getFrameId(n)].rotation.copy(), d.oMf[m.getFrameId(n)].translation.copy())
            for n in cfg['robot']['segment_links']}


def message(cps, tip_i, gap, ph, direction, t, track=None):
    res = []
    for i, cp in enumerate(cps):
        if i == tip_i:
            res.append(ControlPointResult(point=cp['point'], seg_idx=cp['seg_idx'], cp_idx=cp['cp_idx'],
                                          radius=0.05, start_link=cp['start_link'], end_link=cp['end_link'],
                                          distance=gap, direction=direction, closest_obstacle_point=ph,
                                          closest_pixel=None if ph is None else (1, 1), range_m=None))
        else:
            res.append(ControlPointResult(point=cp['point'], seg_idx=cp['seg_idx'], cp_idx=cp['cp_idx'],
                                          radius=0.05, start_link=cp['start_link'], end_link=cp['end_link']))
    stamp = Time(sec=int(t), nanosec=int(round((t - int(t)) * 1e9)))
    _, mld = build_cp_messages(res, 400, stamp, 'fr3_link0', cfg['robot']['segment_links'],
                               {'min_thresh': 0.08, 'max_thresh': 0.7}, 2.0, {})
    if track is not None:                       # what annotate_track_fields writes for a matched track
        for _, ld in labelled_links(mld):
            if ld.valid:
                tid, n, v, cov = track
                ld.track_id, ld.frames_seen = tid, n
                ld.obstacle_velocity.x, ld.obstacle_velocity.y, ld.obstacle_velocity.z = map(float, v)
                ld.velocity_covariance = [float(x) for x in np.asarray(cov).ravel()]
    return mld


def run(scenario, gap0=GAP):
    builder, kin, log = B.make_builder(P)
    q = C.Q_HOME.copy()
    cps = define_control_points(tf_at(kin, q), cfg['robot'], cfg['distance'])
    tip_i = len(cps) - 1
    pr = np.asarray(cps[tip_i]['point'], dtype=np.float64)
    n_true = np.array([-1.0, 0.0, 0.0])
    kin.update(q, np.zeros(7))
    d_off = gap0 + (D_OFF - GAP)
    labels = [lb for lb, _ in labelled_links(message(cps, tip_i, gap0, pr + d_off * np.array([1, 0, 0]),
                                                     n_true, 1.0))]
    tip_lbl = labels[tip_i]
    J, _ = kin.control_point_jacobian(tip_lbl)
    if scenario == 'A':
        qdot = np.linalg.pinv(J) @ np.array([0.10, 0.0, 0.0])
        v_obst = np.zeros(3)
        track = None
    else:
        qdot = np.zeros(7)
        v_obst = np.array([-0.5, 0.0, 0.0])            # moving toward the tip (-x)
    js = B.JointSnap(q, qdot, 0.0)
    trace = []
    ph0 = pr + d_off * np.array([1.0, 0.0, 0.0])
    gap_true = gap0
    held_gap = None
    t0 = 100.0
    for k in range(N_NORM + N_HOLD + N_GONE + N_BACK):
        t = t0 + k * DT
        v_tip = J @ qdot
        gap_true = gap0 + float(n_true @ (v_tip - v_obst)) * (k * DT)
        ph_now = ph0 + v_obst * (k * DT)
        phase = ('normal' if k < N_NORM else 'hold' if k < N_NORM + N_HOLD
                 else 'gone' if k < N_NORM + N_HOLD + N_GONE else 'back')
        track = (7, 60 + k, v_obst, 1e-4 * np.eye(3)) if (scenario == 'B' and phase in ('normal', 'back')) else None
        if phase in ('normal', 'back'):
            msg = message(cps, tip_i, gap_true, ph_now, n_true, t, track)
            held_gap = gap_true
        elif phase == 'hold':
            msg = message(cps, tip_i, held_gap, None, n_true, t)     # F-2-11: point None, direction kept
        else:
            msg = message(cps, tip_i, float('inf'), None, None, t)   # hold expired: invalid
        items = B.obstacles_from_msg(msg)
        before = builder.diag_ident_reset
        con = builder.build(B.JointSnap(q, qdot, t), B.snap(items, t), t)
        rows = [r for r in B.obstacle_rows(con) if r[0] == tip_lbl]
        row = rows[0] if rows else None
        rec = dict(k=k, t=k * DT, phase=phase, gap_true=gap_true, items=items, reset=builder.diag_ident_reset - before)
        if row is not None:
            lbl, a, h, vobs, c = row
            n_row = None
            for ob in items:
                if ob.cp_label == tip_lbl:
                    dl = ob.pr - ob.ph
                    n_row = dl / np.linalg.norm(dl)
            rec.update(h=h, v_obs=vobs, hdot_row=float(a @ qdot) - vobs, n_row=n_row, a=a)
        trace.append(rec)
    return dict(trace=trace, pr=pr, J=J, qdot=qdot, tip_lbl=tip_lbl, n_true=n_true, log=log)


A = run('A')
tr = A['trace']
hold = [r for r in tr if r['phase'] == 'hold']
norm = [r for r in tr if r['phase'] == 'normal']
R.info(f'fingertip label {A["tip_lbl"]}, pr = {np.round(A["pr"], 4)} m')
h0 = hold[0]
ob = [o for o in h0['items'] if o.cp_label == A['tip_lbl']][0]
R.fact('1. held entry is valid and reaches the builder', True)
R.close('1. held entry closest_point_human [m]', ob.ph, [0, 0, 0], 0, kind='FACT')
R.fact(f'1. held entry passes the confidence gate: conf {ob.conf:.3f} >= min_confidence {P.min_confidence}',
       ob.conf >= P.min_confidence)
R.fact(f'1. held entry inside the horizon: d {ob.d:.4f} <= {P.cbf_obstacle_horizon} m', ob.d <= P.cbf_obstacle_horizon)
R.fact('1. held entry track fields are zero (track_id 0, frames_seen 0, v_vec = 0 array, not None)',
       ob.track_id == 0 and ob.frames_seen == 0 and ob.v_vec is not None and not np.any(ob.v_vec))
R.fact('2. a row is built for the fingertip on EVERY held frame', all('h' in r for r in hold),
       f'{sum("h" in r for r in hold)}/{len(hold)}')
n_held = h0['n_row']
R.close('2. held row normal = pr/|pr| (base-radial)', n_held, A['pr'] / np.linalg.norm(A['pr']), 1e-12, kind='NUM')
ang = float(np.degrees(np.arccos(np.clip(n_held @ A['n_true'], -1, 1))))
R.close('2. angle between the held row normal and the true normal [deg]', ang, 122.0, 0.06)
R.close('2. held row a = n_held^T J (row of A)', h0['a'], n_held @ A['J'], 1e-12, kind='NUM')
resets = [(r['phase'], r['k']) for r in tr if r['reset']]
R.info(f'identity resets (phase, frame): {resets}')
R.fact('3. identity guard fires on the first held frame (ph jumps to 0)', hold[0]['reset'] == 1)
R.fact('3. identity guard fires again when the obstacle reappears (ph jumps back)',
       [r for r in tr if r['phase'] == 'back'][0]['reset'] == 1)
v_tip = A['J'] @ A['qdot']
R.close('4. A: true closing rate n_true^T v_tip [m/s]', float(A['n_true'] @ v_tip), -0.10, 1e-9, kind='NUM')
R.close('4. A: the same motion along the held normal, n_held^T v_tip [m/s] (> 0: read as separation)',
        float(n_held @ v_tip), 0.10 * n_held[0], 1e-9, kind='NUM')
pre = norm[-1]
R.info('A: frame  phase   gap_true   h_row     v_obs    A q_dot - v_obs')
for r in tr[N_NORM - 3:N_NORM + N_HOLD + N_GONE + 3]:
    if 'h' in r:
        R.info(f'A: {r["k"]:3d}  {r["phase"]:6s} {r["gap_true"]:8.4f} {r["h"]:9.4f} {r["v_obs"]:8.4f} {r["hdot_row"]:9.4f}')
    else:
        R.info(f'A: {r["k"]:3d}  {r["phase"]:6s} {r["gap_true"]:8.4f}   (no fingertip row)')
R.fact(f'5. A: before the hold the k1 term sees the approach: A q_dot - v_obs = {pre["hdot_row"]:+.4f} m/s < 0',
       pre['hdot_row'] < -0.05)
R.fact('5. A: during the hold A q_dot - v_obs >= 0 on every frame (the approach is not seen)',
       all(r['hdot_row'] >= -1e-9 for r in hold), str([round(r['hdot_row'], 4) for r in hold]))
R.fact('5. A: rows vanish when the entry turns invalid (hold expired)',
       all('h' not in r for r in tr if r['phase'] == 'gone'))

Bs = run('B', gap0=0.40)   # 0.40 - 0.5 x 0.5 s = 0.15 m when the hold starts
trb = Bs['trace']
holdb = [r for r in trb if r['phase'] == 'hold']
preb = [r for r in trb if r['phase'] == 'normal'][-1]
R.info('B: frame  phase   gap_true   h_row     v_obs')
for r in trb[N_NORM - 3:N_NORM + N_HOLD + N_GONE + 3]:
    if 'h' in r:
        R.info(f'B: {r["k"]:3d}  {r["phase"]:6s} {r["gap_true"]:8.4f} {r["h"]:9.4f} {r["v_obs"]:8.4f}')
    else:
        R.info(f'B: {r["k"]:3d}  {r["phase"]:6s} {r["gap_true"]:8.4f}   (no fingertip row)')
R.fact(f'6. B: before the hold the row carries the closing speed (v_obs = {preb["v_obs"]:.3f} m/s)',
       preb['v_obs'] > 0.3)
R.fact('6. B: v_obs = 0 on every held frame (frames_seen 0 -> tracker 0; residual sees a frozen d)',
       all(abs(r['v_obs']) < 1e-12 for r in holdb), str([round(r['v_obs'], 4) for r in holdb]))
R.close('6. B: true gap lost between the first and last held frame [m]', holdb[0]['gap_true'] - holdb[-1]['gap_true'],
        0.5 * (N_HOLD - 1) * DT, 1e-9, kind='NUM', note='0.5 m/s x 8 frame intervals; ~5 cm over the full 0.10 s hold, invisible to the row')
R.info(f'B: row h over the hold: {[round(r["h"], 4) for r in holdb]}')
dh = holdb[-1]['h'] - preb['h']
dg = holdb[-1]['gap_true'] - preb['gap_true']
R.fact(f'6. B: the row LOOSENS while the obstacle closes: h {preb["h"]:+.4f} -> {holdb[-1]["h"]:+.4f} m '
       f'(+{dh:.4f}) while the true gap changes by {dg:+.4f} m', dh > 0.1 and dg < -0.04,
       'latency margin (gated on n_seen) drops at once; standoff decays by its EMA; v_obs -> 0')
sys.exit(R.finish())
