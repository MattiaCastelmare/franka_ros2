#!/usr/bin/env python3
"""Obstacle tracker: process noise (5.8), KF/IMM behaviour (5.9)-(5.11), coast vs gate (Q-2-2).

Preserved from the Stage 2 `kf.py` with the same seed and the same order of random draws, so the
printed statistics reproduce the table in chapter 05 §7.3 exactly (same numpy version).
  1. Q(dt) of `obstacle_tracker.process_noise` equals q * integral_0^dt F G G^T F^T  (NUM)
  2. static target, 90 Hz, 1 cm white noise: spurious |v| and sqrt(P_vv)            (illustrative)
  3. new track on a 1 m/s approach: v_x at 56 / 100 / 200 ms                        (illustrative)
  4. thrown ball under gravity: IMM ballistic mode probability                     (illustrative)
  5. a converged static track coasting 0.8 s: position sigma and the 4-sigma gate  (Q-2-2)
"Illustrative" rows are single-seed simulations with the shipped tuning, not benchmarks.

Needs: Python 3, numpy, SciPy. Imports `franka_experiments.utils.obstacle_tracker` (pure numpy)
from the source tree. Runs on the host.
"""
import os
import sys

sys.dont_write_bytecode = True   # read-only: never write __pycache__ into the repository
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _common as C  # noqa: E402

C.use_source_package()
import numpy as np  # noqa: E402
from scipy.integrate import quad_vec  # noqa: E402
from franka_experiments.utils.obstacle_tracker import (  # noqa: E402
    IMMTrack, KalmanTrack, process_noise, transition)

R = C.Report('check_tracker (chapter 05 §7; Q-2-2)')
cfg = C.load_yaml(os.path.join(C.CONFIG, 'fr3_complete.yaml'))['tracking']
q, sm, sv0, sa0 = float(cfg['q_jerk']), float(cfg['sigma_meas_m']), float(cfg['sigma_v0']), 5.0
R.close('shipped tuning q_jerk, sigma_meas_m, sigma_v0', [q, sm, sv0], [2.0, 0.01, 2.0], 0, kind='FACT')
dt = 1 / 90

# 1) exact integral
G = np.zeros((9, 3))
G[6:9] = np.eye(3)
Qn = quad_vec(lambda t: transition(t) @ G @ G.T @ transition(t).T, 0, dt)[0] * q
R.below('(5.8) process_noise vs exact integral of F G G^T F^T', np.abs(Qn - process_noise(dt, q)).max(), 1e-15,
        note='reported 3.5e-18')

# 2) static target
rng = np.random.default_rng(0)
expect_static = {'KalmanTrack': (0.072, 0.120, 0.064), 'IMMTrack': (0.072, 0.122, 0.064)}
for cls in (KalmanTrack, IMMTrack):
    t = cls(np.zeros(3), q_jerk=q, sigma_meas=sm, sigma_v0=sv0, sigma_a0=sa0)
    sp = []
    for k in range(900):
        t.predict(dt)
        t.update(rng.normal(0, 0.01, 3))
        if k > 450:
            sp.append(np.linalg.norm(t.velocity))
    obs = (np.mean(sp), np.percentile(sp, 95), np.sqrt(t.velocity_cov[0, 0]))
    R.close(f'{cls.__name__} static: |v| mean, |v| p95, sqrt(P_vv,xx) [m/s]', obs, expect_static[cls.__name__],
            6e-4, note='illustrative, one seed')
    if cls is IMMTrack:
        R.close('IMMTrack static: mode probabilities stay neutral', t.mode_prob, [0.5, 0.5], 6e-3)

# 3) new track, constant 1 m/s
expect_new = {'KalmanTrack': (1.22, 0.81, 0.95), 'IMMTrack': (1.05, 1.13, 0.86)}
for cls in (KalmanTrack, IMMTrack):
    t = cls(np.zeros(3), q_jerk=q, sigma_meas=sm, sigma_v0=sv0, sigma_a0=sa0)
    out = []
    for k in range(1, 19):
        t.predict(dt)
        t.update(np.array([k * dt * 1.0, 0, 0]) + rng.normal(0, 0.01, 3))
        if k in (5, 9, 18):
            out.append(t.velocity[0])
    R.close(f'{cls.__name__} new track at 1 m/s: v_x at 56 / 100 / 200 ms', out, expect_new[cls.__name__], 6e-3,
            note='illustrative, one seed')

# 4) thrown ball
t = IMMTrack(np.zeros(3), q_jerk=q, sigma_meas=sm, sigma_v0=sv0)
p, v = np.zeros(3), np.array([-3.5, 0, 2.0])
g = np.array([0, 0, -9.81])
mu, az = [], []
for k in range(1, 31):
    p = p + v * dt + 0.5 * g * dt * dt
    v = v + g * dt
    t.predict(dt)
    t.update(p + rng.normal(0, 0.01, 3))
    if k in (5, 15, 30):
        mu.append(t.mode_prob[1])
        az.append(t.acceleration[2])
R.close('IMM throw: ballistic mode probability at 56 / 167 / 333 ms', mu, [0.47, 0.88, 0.81], 6e-3,
        note='illustrative, one seed')
R.close('IMM throw: estimated a_z at 167 / 333 ms [m/s^2]', az[1:], [-10.0, -9.9], 0.06)

# 5) coast vs gate (Q-2-2)
t = KalmanTrack(np.zeros(3), q_jerk=q, sigma_meas=sm, sigma_v0=sv0)
for k in range(450):
    t.predict(dt)
    t.update(rng.normal(0, 0.01, 3))
sig, gate = {}, {}
cross = None
for k in range(1, 73):
    t.predict(dt)
    if cross is None and 4 * np.sqrt(t.innovation_cov()[0, 0]) > float(cfg['gate_max_m']):
        cross = k
    if k in (9, 36, 72):
        sig[k] = np.sqrt(t.position_cov[0, 0])
        gate[k] = 4 * np.sqrt(t.innovation_cov()[0, 0])
R.close('coasting static track: sigma_p after 0.1 / 0.4 / 0.8 s [m]', [sig[9], sig[36], sig[72]],
        [0.013, 0.080, 0.298], 6e-4)
R.close('coasting static track: 4-sigma gate after 0.1 / 0.4 / 0.8 s [m]', [gate[9], gate[36], gate[72]],
        [0.065, 0.321, 1.191], 6e-4)
R.close('coast time at which the 4-sigma gate exceeds gate_max_m [s]', cross * dt if cross else np.inf, 0.5, 0.06,
        note='05 §7.4 says "after about 0.5 s"')
R.info(f'gate_max_m = {cfg["gate_max_m"]} m; gate_mahalanobis = {cfg["gate_mahalanobis"]} '
       f'(the check above uses 4 sigma per axis, the shipped Mahalanobis gate)')
sys.exit(R.finish())
