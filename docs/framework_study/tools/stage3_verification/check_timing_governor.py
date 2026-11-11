#!/usr/bin/env python3
"""Reference generation of pentagon_qddot_commander: timing law, path, phase governor (chapter 06).

Uses the SOURCE classes `utils/timing_law.py::LinearTimingLaw` and `utils/trajectory.py::
CircularPath / LinearSegmentPath / CompositePath`, built exactly as `_start_trajectory` and
`_build_timing_law` build them from the shipped parameters (fr3_control.yaml: circle, centre
(0.4, 0, 0.45), r 0.25, cycle 6 s; node defaults: plane yz, approach_time 2.5 s, timing linear).

  1. (6.1)-(6.3) chain rule: with sigma = 1, v_d and a_d equal the central differences of p_d(t)  (NUM)
  2. soft start: s_dot, s_ddot continuous; phase at the end of the ramp = S_a / 2               (NUM)
  3. F-3-3: the governor calls timing.step(dt * sigma). For sigma < 1, v_d / (dp_d/dt) = 1/sigma and
     the centripetal part of a_d is 1/sigma^2 too large                                         (NUM)
  4. F-3-3 consequence on an idealised task-space double integrator x'' = a_d + Kp e + Kd e'
     (commander gains, cart_err_max clip): the steady lead of x over p_d at sigma = 0.3
     vs (Kd/Kp)(1 - sigma)|v_full|                                                    (illustrative)
  5. F-3-9: velocity step of v_d at the lead-in/cycle junction s = S_a, from the home TCP
     (0.3069, 0, 0.4869) m (flange of 04 §6 + 0.1034 m along z8 = -z)                (illustrative)
  6. the commander's governor sigma(d) at the shipped radii (1.5 / 0.8 x d_safe)          (FACT)

Needs: Python 3, numpy, PyYAML. Host. Read-only.
"""
import math
import os
import sys

sys.dont_write_bytecode = True
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _common3 as C3  # noqa: E402

C = C3.C
C.use_source_package()
import numpy as np  # noqa: E402
from franka_experiments.utils.timing_law import LinearTimingLaw  # noqa: E402
from franka_experiments.utils.trajectory import (  # noqa: E402
    CircularPath, CompositePath, LinearSegmentPath)

R = C.Report('check_timing_governor (chapter 06 §3, §5; F-3-3, F-3-9)')
y = C3.yaml_params()
center, radius, cycle = np.array(y['path_center_xyz'], float), float(y['path_radius']), float(y['path_cycle_time'])
kp, kd = float(y['path_kp_cart']), float(y['path_kd_cart'])
d_safe = float(y['d_safe'])
APPROACH, PLANE, CART_ERR_MAX = 2.5, 'yz', 0.08          # node defaults (declare_parameter)
R.close('shipped path: centre, radius, cycle, Kp, Kd', [*center, radius, cycle, kp, kd],
        [0.4, 0.0, 0.45, 0.25, 6.0, 20.0, 9.0], 0, kind='FACT')
P_HOME_TCP = np.array([0.3069, 0.0, 0.4869])


def build(p0):
    cyc = CircularPath(center=center, radius=radius, plane=PLANE)
    S_a = APPROACH / cycle
    path = CompositePath([(LinearSegmentPath(p0, cyc.position(0.0).copy(), span=S_a), S_a), (cyc, math.inf)])
    return path, LinearTimingLaw(rate=1.0 / cycle, soft_start_s=APPROACH), S_a


def run(sigma_of_t, T=12.0, dt=1e-3, p0=P_HOME_TCP):
    """Real-time samples of (t, s, p_d, v_d, a_d), stepping the law as _tick does."""
    path, law, S_a = build(p0)
    law.reset(0.0)
    out = []
    for k in range(int(T / dt)):
        t = (k + 1) * dt
        s, sd, sdd = law.step(dt * sigma_of_t(t))
        out.append((t, s, path.position(s).copy(), path.velocity(s, sd).copy(),
                    path.acceleration(s, sd, sdd).copy()))
    return out, S_a


# 1) chain rule with sigma = 1, on the cyclic part (t in [5, 11] s: after the junction at ~4.25 s)
dt = 1e-3
tr, S_a = run(lambda t: 1.0, dt=dt)
t = np.array([r[0] for r in tr]); P = np.array([r[2] for r in tr])
V = np.array([r[3] for r in tr]); A = np.array([r[4] for r in tr])
sel = (t > 5.0) & (t < 11.0)
i = np.flatnonzero(sel)[1:-1]
v_fd = (P[i + 1] - P[i - 1]) / (2 * dt)
a_fd = (P[i + 1] - 2 * P[i] + P[i - 1]) / dt ** 2
R.below('(6.2) sigma=1: |v_d - dp_d/dt| on the cycle [m/s]', np.abs(V[i] - v_fd).max(), 1e-5)
R.below('(6.3) sigma=1: |a_d - d2p_d/dt2| on the cycle [m/s^2]', np.abs(A[i] - a_fd).max(), 1e-3)
R.close('cycle speed 2 pi r / T [m/s]', np.linalg.norm(V[i], axis=1).mean(), 2 * np.pi * radius / cycle, 1e-9,
        kind='NUM')

# 2) soft start
law = LinearTimingLaw(rate=1.0 / cycle, soft_start_s=APPROACH)
law.reset(0.0)
for _ in range(int(APPROACH / 1e-4)):
    s_end, sd_end, _ = law.step(1e-4)
R.close('phase at the end of the soft start = S_a/2 (smootherstep integrates to 1/2)', s_end, S_a / 2, 2e-5,
        kind='NUM')
R.close('s_dot at the end of the soft start = rate', sd_end, 1 / cycle, 1e-6, kind='NUM')
t_junction = APPROACH + (S_a / 2) / (1 / cycle)
R.close('time at which s reaches S_a (junction) [s]', t_junction, 3.75, 1e-9, kind='NUM',
        note='ramp 2.5 s + the remaining S_a/2 at full rate')

# 3) F-3-3: governor
for sig in (0.3, 0.05):
    tr, _ = run(lambda t, sg=sig: 1.0 if t < 5.0 else sg, T=11.0, dt=dt)
    t = np.array([r[0] for r in tr]); P = np.array([r[2] for r in tr]); V = np.array([r[3] for r in tr])
    A = np.array([r[4] for r in tr])
    i = np.flatnonzero((t > 6.0) & (t < 10.5))[1:-1]
    v_true = (P[i + 1] - P[i - 1]) / (2 * dt)
    ratio = np.linalg.norm(V[i], axis=1).mean() / np.linalg.norm(v_true, axis=1).mean()
    R.close(f'F-3-3 sigma={sig}: |v_d| / |dp_d/dt| (should be 1; code gives 1/sigma)', ratio, 1 / sig, 1e-3,
            kind='NUM')
    a_true_n = np.linalg.norm(v_true, axis=1).mean() ** 2 / radius
    R.close(f'F-3-3 sigma={sig}: |a_d| / true centripetal (code gives 1/sigma^2)',
            np.linalg.norm(A[i], axis=1).mean() / a_true_n, 1 / sig ** 2, 1e-2 / sig ** 2, kind='NUM')

# 4) consequence on an ideal double integrator in task space (commander gains and error clip)
def lead_run(sig, T=16.0):
    tr, _ = run(lambda t: 1.0 if t < 5.0 else sig, T=T, dt=dt)
    x, xd = tr[0][2].copy(), np.zeros(3)
    lead, enorm = [], []
    for (t, s, p, v, a) in tr:
        e = p - x
        ne = np.linalg.norm(e)
        if ne > CART_ERR_MAX:
            e *= CART_ERR_MAX / ne
        xdd = a + kp * e + kd * (v - xd)
        xd = xd + xdd * dt
        x = x + xd * dt
        if t > T - 4.0:
            tang = v / np.linalg.norm(v)
            lead.append(float((x - p) @ tang))
            enorm.append(min(ne, 1e9))
    return float(np.mean(lead)), float(np.mean(enorm))


v_full = 2 * np.pi * radius / cycle
for sig in (0.5, 0.3):
    pred = (kd / kp) * (1 - sig) * v_full
    lead, en = lead_run(sig)
    if pred < CART_ERR_MAX:
        R.close(f'F-3-3 illustrative: steady tangential lead of x over p_d at sigma={sig} [m]', lead, pred, 6e-3,
                kind='NUM', note=f'linear prediction (Kd/Kp)(1-sigma)|v_full| = {pred:.4f} m below the clip; '
                'double-integrator idealisation, not the robot')
    else:
        R.info(f'F-3-3 illustrative sigma={sig}: linear prediction {pred:.4f} m exceeds cart_err_max '
               f'{CART_ERR_MAX} m, so the clip binds; simulated lead {lead:.4f} m, mean |e| {en:.4f} m')

# 5) F-3-9 junction (derivatives() returns internal buffers: copy them)
path, law, S_a = build(P_HOME_TCP)
eps = 1e-9
Pp_before = path.derivatives(S_a - eps)[1].copy()
Pp_after = path.derivatives(S_a + eps)[1].copy()
v_before, v_after = Pp_before / cycle, Pp_after / cycle
entry = CircularPath(center=center, radius=radius, plane=PLANE).position(0.0).copy()
R.info(f'entry point {np.round(entry, 4)}; lead-in velocity {np.round(v_before, 4)} m/s, '
       f'cycle velocity {np.round(v_after, 4)} m/s at s = S_a')
L = float(np.linalg.norm(entry - P_HOME_TCP))
R.close('lead-in at full rate: |dP/ds|/T = |entry - p0| / approach_time [m/s]', np.linalg.norm(v_before),
        L / APPROACH, 1e-9, kind='NUM', note=f'|entry - p0| = {L:.4f} m from the home TCP')
R.close('F-3-9 velocity step |dv_d| at the junction [m/s]', np.linalg.norm(v_after - v_before),
        np.linalg.norm(v_full * (Pp_after / np.linalg.norm(Pp_after)) - (entry - P_HOME_TCP) / APPROACH), 1e-9,
        kind='NUM', note='= |v_cycle - (entry - p0)/approach_time|; depends on where the arm starts')
R.info(f'F-3-9 magnitude from the home TCP: |dv_d| = {np.linalg.norm(v_after - v_before):.4f} m/s, '
       f'one-tick a_d impulse not represented (a_d of the lead-in is 0)')

# 6) governor sigma(d) as declared from the YAML ratios
d_full, d_stop, s_min = float(y['governor_r_full']) * d_safe, float(y['governor_r_stop']) * d_safe, 0.05


def sigma(d):
    if d >= d_full:
        return 1.0
    if d <= d_stop:
        return s_min
    return s_min + (1 - s_min) * (d - d_stop) / (d_full - d_stop)


R.close('governor radii d_full, d_stop [m]', [d_full, d_stop], [0.15, 0.08], 1e-12, kind='FACT')
R.close('sigma at d = 0.15 / 0.115 / 0.08 m', [sigma(0.15), sigma(0.115), sigma(0.08)], [1.0, 0.525, 0.05], 1e-12,
        kind='NUM')
sys.exit(R.finish())
