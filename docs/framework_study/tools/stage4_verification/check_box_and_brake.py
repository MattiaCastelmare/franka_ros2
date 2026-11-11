#!/usr/bin/env python3
"""The hard joint box and the braking fallbacks (chapter 08 §2.2–§2.4, §6).

Production functions: `cbf_hard_limits.hard_accel_box`, `position_velocity_accel_box`,
`apply_slew_limit`, `fr3_velocity_envelope`; `cbf_qp_assembly.braking_command`. Parameters: the
hardware-default set, limits derived as `cbf_safety_filter.__init__` derives them.

  1. (8.5)–(8.7) hard_accel_box against an independent implementation of its formulas,
     2000 random states (inside and outside the limits)                                        (NUM)
  2. the box is never empty (lb ≤ ub), including states past the limits                        (NUM)
  3. position protection: a double integrator at the QP rate, always commanded at the box's
     upper edge toward q_max, never passes q_max − position_margin by more than the
     discretisation residue                                                                   (NUM)
  4. the QP-failure brake, clip(−k_brake q̇, state box ∩ slew window), from the fastest state the
     box allows near the limit, stays inside q_max                                            (NUM)
  5. the STALE-JOINT-STATE brake (`_qp_tick` L1099–1104) is clip(−k_brake·q̇_stale, static box),
     re-published with the SAME stale q̇ every tick: an integrator of it reverses after 1/k_brake s,
     and a constant deceleration k·v0 stops after v0/(2k); from the box's fastest state
     v0 = √(2 a_auth h) that exceeds the remaining margin h whenever h < a_auth/(2k²)  (Derived + NUM)
  6. slew window δ_eff = max_qddot_delta · min(Δt/dt_nominal, slew_dt_ratio_max)               (FACT)

Needs: the ROS 2 container (`config.load_franka_joint_limits` uses ament_index), numpy, PyYAML.
Read-only. Item 5 idealises tracking (realised q̈ = commanded).
"""
import os
import sys

sys.dont_write_bytecode = True
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _common4 as C4  # noqa: E402

C, C3 = C4.C, C4.C3
import numpy as np  # noqa: E402
from franka_experiments.utils.cbf_hard_limits import (  # noqa: E402
    FR3_VEL_LIMIT, FR3_VEL_OFFSET, FR3_VEL_Q_REF_LOWER, FR3_VEL_Q_REF_UPPER, FR3_VEL_SLOPE,
    apply_slew_limit, fr3_velocity_envelope, hard_accel_box)
from franka_experiments.utils.cbf_qp_assembly import braking_command  # noqa: E402

R = C.Report('check_box_and_brake (chapter 08 §2.2-§2.4, §6)')
P = C3.params_namespace('hw_default')
L = C4.static_limits(P)
lb0, ub0, brake = L['lb'], L['ub'], L['brake']
qmin, qmax, qdmax = L['q_min'], L['q_max'], L['qdot_max']
dt = 1.0 / P.qp_rate_hz
kw = dict(acc_lb=lb0, acc_ub=ub0, qdot_max=qdmax, v_margin=P.velocity_box_margin, q_min=qmin, q_max=qmax,
          q_margin=P.position_margin_rad, brake_eta=P.position_brake_eta, dt=dt, relax_dt=P.state_box_relax_s,
          clip_to_limits=P.accel_box_clip_to_limits, brake_acc=brake)
R.info(f'acc box ±{np.round(ub0, 3)}; brake authority a_auth = η·brake = {np.round(P.position_brake_eta * brake, 3)}')


def indep_box(q, qd):
    """Independent re-implementation from the formulas of 08 §2.2 (not a copy of the code)."""
    a_auth = P.position_brake_eta * brake
    h_up = np.maximum(qmax - P.position_margin_rad - q, 0.0)
    h_lo = np.maximum(q - P.position_margin_rad - qmin, 0.0)
    m = P.velocity_box_margin
    env_up = m * np.minimum(FR3_VEL_LIMIT, np.maximum(0.0, -FR3_VEL_OFFSET + np.sqrt(np.maximum(0.0, FR3_VEL_SLOPE * (FR3_VEL_Q_REF_UPPER - q)))))
    env_lo = m * np.maximum(-FR3_VEL_LIMIT, np.minimum(0.0, FR3_VEL_OFFSET - np.sqrt(np.maximum(0.0, FR3_VEL_SLOPE * (q - FR3_VEL_Q_REF_LOWER)))))
    v_ub = np.minimum.reduce([m * qdmax, np.sqrt(2 * a_auth * h_up), env_up])
    v_lb = np.maximum.reduce([-m * qdmax, -np.sqrt(2 * a_auth * h_lo), env_lo])
    T_up = np.where(v_ub >= qd, P.state_box_relax_s, dt)
    T_lo = np.where(v_lb <= qd, P.state_box_relax_s, dt)
    ub = np.minimum(ub0, (v_ub - qd) / T_up)
    lb = np.maximum(lb0, (v_lb - qd) / T_lo)
    ub = np.maximum(ub, lb)
    lb = np.minimum(lb, ub0)
    ub = np.maximum(np.minimum(ub, ub0), lb)
    return lb, ub


rng = np.random.default_rng(5)
err, empty = 0.0, 0
for _ in range(2000):
    q = rng.uniform(qmin - 0.1, qmax + 0.1)
    qd = rng.uniform(-1.3 * qdmax, 1.3 * qdmax)
    lb, ub = hard_accel_box(q, qd, **kw)
    li, ui = indep_box(q, qd)
    err = max(err, float(np.abs(lb - li).max()), float(np.abs(ub - ui).max()))
    empty += int(np.any(lb > ub + 1e-12))
R.below('1. hard_accel_box == independent formulas (2000 random states)', err, 1e-12)
R.fact('2. box never empty (lb ≤ ub), also past the limits', empty == 0, f'{empty} empty boxes')

# 3) position protection under the most aggressive admissible command
worst = -np.inf
for j in range(7):
    q = C.Q_HOME.copy()
    q[j] = qmax[j] - 0.6
    qd = np.zeros(7)
    for _ in range(int(4.0 / dt)):
        lb, ub = hard_accel_box(q, qd, **kw)
        a = np.zeros(7)
        a[j] = ub[j]
        qd = qd + a * dt
        q = q + qd * dt
    worst = max(worst, float(q[j] - (qmax[j] - P.position_margin_rad)))
R.below('3. max over joints of q − (q_max − margin) when always accelerating at the box edge [rad]', worst, 1e-3,
        note='semi-implicit Euler at the QP rate; ideal tracking')


# 4) QP-failure brake from the box's fastest state 0.1 rad before the margin
def run_brake(j, h0, stale):
    a_auth = P.position_brake_eta * brake[j]
    q = C.Q_HOME.copy()
    q[j] = qmax[j] - P.position_margin_rad - h0
    v0 = min(np.sqrt(2 * a_auth * h0), P.velocity_box_margin * qdmax[j],
             fr3_velocity_envelope(q, margin=P.velocity_box_margin)[0][j])
    qd = np.zeros(7)
    qd[j] = v0
    qd_stale = qd.copy()
    prev = np.zeros(7)
    qmax_seen, t_rev = -np.inf, None
    for k in range(int(2.0 / dt)):
        if stale:
            cmd = np.clip(-P.k_brake * qd_stale, lb0, ub0)            # _qp_tick L1102: static box, stale q̇
        else:
            lb, ub = hard_accel_box(q, qd, **kw)
            lo, hi = apply_slew_limit(lb, ub, prev, P.max_qddot_delta, yield_to_state_box=True)
            cmd = braking_command(qd, lo, hi, k_brake=P.k_brake)       # _solve brake rung with _brake_box
        prev = cmd
        qd = qd + cmd * dt
        q = q + qd * dt
        qmax_seen = max(qmax_seen, q[j])
        if t_rev is None and qd[j] < 0.0:
            t_rev = (k + 1) * dt
    return v0, qmax_seen - (qmax[j] - P.position_margin_rad), qmax_seen - qmax[j], t_rev


for j in (3, 0):
    v0, over_m, over_lim, _ = run_brake(j, 0.10, stale=False)
    R.fact(f'4. joint {j + 1}: QP-failure brake from v0 = {v0:.3f} rad/s, h = 0.10 rad: stays inside q_max '
           f'(peak {over_m:+.4f} rad vs q_max − margin, {over_lim:+.4f} vs q_max)', over_lim <= 0.0)

# 5) stale-joint-state brake: every joint, from h = 0.10 rad at the fastest speed the box allows there
over_joints = []
for j in range(7):
    a_auth = P.position_brake_eta * brake[j]
    v0, over_m, over_lim, t_rev = run_brake(j, 0.10, stale=True)
    pred = v0 / (2 * P.k_brake) - 0.10
    R.close(f'5. joint {j + 1}: v0 = {v0:.3f} rad/s; stale-JS brake peak past q_max − margin = v0/(2k) − h [rad]',
            over_m, pred, 2e-3, kind='NUM', note=f'reverses at t = {t_rev:.3f} s (1/k = {1 / P.k_brake:.3f})')
    if over_m > 0.0:
        over_joints.append((j + 1, over_m, over_lim))
R.info('5. Derived: a constant deceleration k·v0 stops after v0/(2k), so the stale brake overshoots the margin '
       'iff v0 > 2k·h; with v0 = √(2 a_auth h) (box binding) iff h < a_auth/(2k²); where the firmware '
       'envelope binds first, v0 is smaller and there is no overshoot')
R.fact('5. from h = 0.10 rad, joints that overshoot q_max − margin under the stale-JS brake: '
       + (', '.join(f'j{j} ({o:+.4f} rad, {ol:+.4f} vs q_max)' for j, o, ol in over_joints) or 'none'),
       len(over_joints) > 0, 'ideal tracking; whether a 0.1 s+ joint-state stall coincides with such a state is runtime')

# 6) slew window
for ratio in (1.0, 3.0, 50.0):
    eff = P.max_qddot_delta * min(ratio, P.slew_dt_ratio_max)
    R.info(f'6. tick gap {ratio:>4.0f}×dt → δ_eff = {eff:.2f} rad/s² per tick')
R.fact(f'6. nominal slew {P.max_qddot_delta} rad/s² per {1e3 * dt:.2f} ms tick = {P.max_qddot_delta / dt:.0f} rad/s³, '
       f'stretched up to ×{P.slew_dt_ratio_max:g} on a late tick', P.slew_rate_compensation)
sys.exit(R.finish())
