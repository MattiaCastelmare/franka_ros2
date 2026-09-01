"""Regression guard: the action must actually control the arm.

This exists because it once did not. The env used to drive MuJoCo's POSITION
servos from a state-seeded reference (``q_des = q + (q̇ + q̈·dt)·dt``). Because
the reference was re-anchored to the measurement every tick it never
integrated: the commanded lead saturated at ``q̈·dt²`` (~6e-4 rad) and the servo
settled at ``q̇ ≈ 0.004 rad/s`` regardless of the commanded acceleration. The
CBF, the observation and the reward were all fine — but a zero-action policy, a
random policy and a trained policy produced the *same* trajectory, so training
could not work and no metric revealed it.

The env now mirrors the deployment chain instead: ``q̈_safe → τ = M q̈ + C q̇ +
g → joint torque`` (``mj_inverse``, recomputed every substep like
``rt_torque_controller`` does at 1 kHz).

Checks
------
1. **Inverse-dynamics fidelity** — commanding q̈ yields q̈ in the forward pass.
2. **Torque limits** — clipped with the JOINT's ``actuatorfrcrange`` (±87/±12
   N·m), never the actuators' position ctrlrange (the original bug).
3. **Control authority** — a unit action moves the arm far more than the
   zero-action baseline over the same horizon.
4. **Action discrimination** — opposite actions produce opposite joint motion.

    cd /ros2_ws/src && PYTHONPATH=/ros2_ws/src MUJOCO_GL=egl \
        python3 -m franka_sim.scripts.validate_actuation
"""

from __future__ import annotations

import numpy as np
import mujoco

from franka_sim.envs.franka_cbf_env import FrankaCBFEnv

NV = 7
HORIZON = 50          # 0.5 s at 100 Hz
AUTHORITY_RATIO = 5.0  # a unit action must move ≥5x the do-nothing drift


def _run(env, action, seed=3, steps=HORIZON):
    env.reset(seed=seed)
    q0 = env._q.copy()
    for _ in range(steps):
        env.step(action)
    return env._q - q0, env._qdot.copy()


def main() -> int:
    env = FrankaCBFEnv()
    failures = []

    # ── 1. Inverse-dynamics fidelity ────────────────────────────────────────
    print('── inverse dynamics ────────────────────────────────────')
    worst = 0.0
    for target in (np.zeros(NV), np.eye(NV)[0], -2.0 * np.eye(NV)[3],
                   np.array([0.5, -0.5, 0.5, -0.5, 1.0, -1.0, 1.0])):
        env.reset(seed=3)
        env.data.ctrl[env._act] = env._inverse_dynamics(target)
        mujoco.mj_forward(env.model, env.data)
        err = float(np.max(np.abs(env.data.qacc[env._dadr] - target)))
        worst = max(worst, err)
    print(f'  max |realized q̈ − commanded q̈| = {worst:.3e} rad/s²')
    if worst > 1e-6:
        failures.append(f'inverse dynamics inexact (err={worst:.2e})')

    # ── 2. Torque limits come from the joints, not the position ctrlrange ───
    print('── torque limits ───────────────────────────────────────')
    print(f'  lo = {env._tau_lo}\n  hi = {env._tau_hi}')
    if not np.all(env._tau_hi >= 10.0):
        failures.append(
            f'torque upper limits look like a position range: {env._tau_hi}')
    if not np.all(env._tau_lo <= -10.0):
        failures.append(
            f'torque lower limits look like a position range: {env._tau_lo}')

    # ── 3 & 4. Authority and discrimination ─────────────────────────────────
    print('── control authority (0.5 s) ───────────────────────────')
    dq_zero, _ = _run(env, np.zeros(NV))
    dq_pos, qd_pos = _run(env, np.eye(NV)[0])
    dq_neg, qd_neg = _run(env, -np.eye(NV)[0])
    n_zero = float(np.linalg.norm(dq_zero))
    n_pos = float(np.linalg.norm(dq_pos))
    print(f'  |Δq| zero-action  = {n_zero:.4f} rad   (drift)')
    print(f'  |Δq| unit action  = {n_pos:.4f} rad')
    print(f'  q̇₁: +action {qd_pos[0]:+.3f} rad/s   −action {qd_neg[0]:+.3f} rad/s')
    if n_pos < AUTHORITY_RATIO * max(n_zero, 1e-6):
        failures.append(
            f'no control authority: unit action moves {n_pos:.4f} rad vs '
            f'{n_zero:.4f} rad of drift (need {AUTHORITY_RATIO}x)')
    if not (qd_pos[0] > 0.1 > -0.1 > qd_neg[0]):
        failures.append(
            f'action sign not reflected in motion: q̇₁ = {qd_pos[0]:+.3f} / '
            f'{qd_neg[0]:+.3f}')

    env.close()
    print()
    if failures:
        print('FAIL:\n  ' + '\n  '.join(failures))
        return 1
    print('RESULT: PASS — the action controls the arm through the torque chain')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
