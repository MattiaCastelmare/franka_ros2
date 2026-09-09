"""CBF guarantee validation on a reduced double-integrator barrier.

Isolates the SHIELD from FR3 kinematics: one joint drives a scalar barrier
h = d − d_safe with d = d0 + q0 (so ḋ = q̇0, i.e. leverage a = e0). A constant
nominal command pushes the robot INTO the obstacle (q̈_nom0 < 0). We integrate
the closed loop and compare:

    * CBF ON  : q̈ = AccelCBFFilter.filter(..., [obstacle row a=e0])
    * PASSTHRU: q̈ = filter(..., []) — same hard state box, NO obstacle row

The HOCBF must keep d ≥ ~d_safe (it decelerates and stops at the barrier),
while passthrough drives d < 0 (penetration). This is the exact math the real
cbf_safety_filter.py enforces, so a PASS here certifies the sim shield.

Run:  cd /ros2_ws/src && PYTHONPATH=/ros2_ws/src \
          python3 -m franka_sim.scripts.validate_cbf
"""

from __future__ import annotations

import numpy as np
import yaml

from franka_sim.envs.cbf_filter import AccelCBFFilter, Obstacle
from franka_sim.envs.franka_cbf_env import _DEFAULT_CONFIG

NV = 7


def run(cbf_on: bool, d0: float = 0.45, steps: int = 600):
    cfg = yaml.safe_load(open(_DEFAULT_CONFIG))
    cbf_c, lim_c = cfg['cbf'], cfg['joint_limits']
    keys = [f'joint{i}' for i in range(1, 8)]
    q_min     = np.array([lim_c[k][0] for k in keys])
    q_max     = np.array([lim_c[k][1] for k in keys])
    qdot_max  = np.array([lim_c[k][2] for k in keys])
    qddot_max = np.array([lim_c[k][3] for k in keys])
    dt = 1.0 / float(cfg['env']['control_rate_hz'])
    d_safe = float(cbf_c['d_safe'])
    # Disable the workspace box for this reduced test (no EE geometry here).
    cbf_c = dict(cbf_c); cbf_c['ws_enable'] = False
    filt = AccelCBFFilter(cbf_c, qddot_max, qdot_max, q_min, q_max, dt)
    filt.reset()

    # Joint0 sits mid-range so its hard position box never interferes; d = d0 + q0.
    q = np.zeros(NV); q[0] = 0.5 * (q_min[0] + q_max[0])
    qdot = np.zeros(NV)
    q0_ref = q[0]
    a = np.zeros(NV); a[0] = 1.0                 # leverage: ḋ = a·q̇ = q̇0
    push = -0.8 * qddot_max[0]                    # constant approach accel (q̈0 < 0)

    d_hist, h_hist = [], []
    for _ in range(steps):
        d = d0 + (q[0] - q0_ref)                  # barrier distance
        qddot_nom = np.zeros(NV); qddot_nom[0] = push
        obstacles = [Obstacle('x', d, a.copy(), 0.0)] if cbf_on else []
        qddot_safe, info = filt.filter(q, qdot, qddot_nom, obstacles)
        qdot = qdot + qddot_safe * dt
        q = q + qdot * dt
        d_hist.append(d)
        h_hist.append(d - d_safe)
    return dict(d0=d0, min_d=float(min(d_hist)), final_d=float(d_hist[-1]),
                penetrated=bool(min(d_hist) < 0.0),
                held_at_dsafe=bool(min(d_hist) >= d_safe - 0.02))


if __name__ == '__main__':
    d_safe = yaml.safe_load(open(_DEFAULT_CONFIG))['cbf']['d_safe']
    on = run(cbf_on=True)
    off = run(cbf_on=False)
    print(f'\nReduced-model CBF test  (constant push INTO obstacle, d_safe={d_safe} m)\n')
    print(f'{"":22s}{"init d":>10s}{"min d":>10s}{"final d":>10s}{"penetrated?":>13s}')
    print(f'{"CBF ON":22s}{on["d0"]:10.3f}{on["min_d"]:10.3f}{on["final_d"]:10.3f}'
          f'{str(on["penetrated"]):>13s}')
    print(f'{"PASSTHROUGH":22s}{off["d0"]:10.3f}{off["min_d"]:10.3f}{off["final_d"]:10.3f}'
          f'{str(off["penetrated"]):>13s}')
    ok = on['held_at_dsafe'] and (not on['penetrated']) and off['penetrated']
    print('\nRESULT:', 'PASS — CBF stops the robot at d_safe; passthrough penetrates'
          if ok else 'FAIL — inspect numbers above')
