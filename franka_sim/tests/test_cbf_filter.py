"""Guards for the sim shield (franka_sim/envs/cbf_filter.py).

ROS-free on purpose: franka_sim must stay importable without a ROS install
(see franka_sim_to_real_implementation_status.md §8 gotcha 7), so these run
under plain pytest and not under `colcon test`.

    cd /ros2_ws/src && PYTHONPATH=/ros2_ws/src python3 -m pytest franka_sim/tests -q

Every test here corresponds to something that actually went wrong, or to a
property whose silent loss would not show up in any training curve.
"""

import os

import numpy as np
import pytest
import yaml

from franka_sim.envs.cbf_filter import (
    AccelCBFFilter,
    FR3_VEL_Q_REF_LOWER,
    FR3_VEL_Q_REF_UPPER,
    Obstacle,
    fr3_velocity_envelope,
    hard_accel_box,
)

_CFG = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'config.yaml')
NV = 7


@pytest.fixture(scope='module')
def cfg():
    with open(_CFG) as fh:
        return yaml.safe_load(fh)


@pytest.fixture(scope='module')
def limits(cfg):
    keys = [f'joint{i}' for i in range(1, 8)]
    lim = cfg['joint_limits']
    return dict(
        q_min=np.array([lim[k][0] for k in keys]),
        q_max=np.array([lim[k][1] for k in keys]),
        qdot_max=np.array([lim[k][2] for k in keys]),
        qddot_max=np.array([lim[k][3] for k in keys]),
    )


def _filter(cfg, limits, **over):
    c = dict(cfg['cbf'])
    c.update(over)
    return AccelCBFFilter(c, limits['qddot_max'], limits['qdot_max'],
                          limits['q_min'], limits['q_max'], 0.01)


# ── Acceleration authority ───────────────────────────────────────────────────

def test_accel_cap_bounds_the_box_not_the_action(cfg, limits):
    """qddot_max_abs caps the QP box; the action scale keeps the raw limit.

    Mirrors the robot exactly: rl_policy_commander scales by fr3_control.yaml's
    joint_limits (17 rad/s² on joints 5 and 7) and cbf_safety_filter's box then
    clips to 10. Capping both would change what a = 1 means.
    """
    f = _filter(cfg, limits)
    cap = float(cfg['cbf']['qddot_max_abs'])
    assert np.allclose(f.qddot_max, limits['qddot_max'])      # action scale raw
    assert np.all(f.qddot_box <= cap + 1e-12)                 # box capped
    assert np.allclose(f.qddot_box, np.minimum(limits['qddot_max'], cap))
    # The cap must actually bite, or it is not reproducing hardware.
    assert np.any(limits['qddot_max'] > cap)


def test_qp_output_never_exceeds_the_capped_box(cfg, limits):
    """Even asking for the uncapped q̈_max comes back inside the cap."""
    f = _filter(cfg, limits)
    q = np.zeros(NV)
    q[3] = -1.5                       # joint4 lives in [-3.08, -0.12]
    q[5] = 1.5                        # joint6 lives in [0.44, 4.62]
    qddot_safe, _ = f.filter(q, np.zeros(NV), limits['qddot_max'], [])
    assert np.all(np.abs(qddot_safe) <= f.qddot_box + 1e-6)


# ── Firmware velocity envelope ───────────────────────────────────────────────

def test_envelope_collapses_to_zero_at_the_reference_position():
    """At and past q_ref the firmware admits no further motion that way."""
    up, lo = fr3_velocity_envelope(FR3_VEL_Q_REF_UPPER.copy())
    assert np.allclose(up, 0.0)
    up, lo = fr3_velocity_envelope(FR3_VEL_Q_REF_LOWER.copy())
    assert np.allclose(lo, 0.0)


def test_envelope_is_tighter_than_the_flat_limit_near_a_stop():
    """The whole point: the flat limit is not what the reflex enforces.

    Run 20260915_080040 aborted with joint6 at 4.4644 rad and +0.489 rad/s,
    where the firmware admitted +0.435 — 0.85 of the flat limit, so no
    flat-limit guard could have caught it.
    """
    q = np.zeros(NV)
    q[5] = 4.4644
    up, _ = fr3_velocity_envelope(q)
    assert up[5] < 0.489, 'envelope must reject the speed that aborted the run'
    assert up[5] == pytest.approx(0.435, abs=0.01)


def test_effective_position_limits_are_inside_the_mechanical_ones(cfg, limits):
    """A barrier anchored at the mechanical stop sits behind the real wall."""
    f = _filter(cfg, limits, firmware_envelope=True)
    assert np.all(f.q_max <= limits['q_max'] + 1e-12)
    assert np.all(f.q_min >= limits['q_min'] - 1e-12)
    assert np.any(f.q_max < limits['q_max'] - 1e-9), 'envelope never bit'

    raw = _filter(cfg, limits, firmware_envelope=False)
    assert np.allclose(raw.q_max, limits['q_max'])
    assert np.allclose(raw.q_min, limits['q_min'])


def _mid_pose():
    """A configuration inside every joint's range (q=0 is outside joint4's)."""
    q = np.zeros(NV)
    q[3] = -1.5          # joint4 lives in [-3.077, -0.117]
    q[5] = 1.5           # joint6 lives in [ 0.440,  4.622]
    return q


def test_envelope_narrows_the_accel_box_near_a_position_limit(limits):
    """With the envelope on, a joint near its stop gets a tighter box.

    The joint must be MOVING toward the bound for this to be observable: with
    q̇ = 0 the velocity headroom is large next to q̈_max·dt and both branches
    simply saturate at acc_ub.
    """
    q = _mid_pose()
    q[5] = 4.45                                   # joint6, close to q_ref 4.5205
    qdot = np.zeros(NV)
    qdot[5] = 0.45                                # just under the envelope bound
    kw = dict(acc_lb=-limits['qddot_max'], acc_ub=limits['qddot_max'],
              qdot_max=limits['qdot_max'], v_margin=0.9,
              q_min=limits['q_min'], q_max=limits['q_max'],
              q_margin=0.05, brake_eta=0.6, dt=0.01)
    _, ub_on = hard_accel_box(q, qdot, firmware_envelope=True, **kw)
    _, ub_off = hard_accel_box(q, qdot, firmware_envelope=False, **kw)
    assert ub_on[5] < ub_off[5], (
        f'envelope did not tighten joint6: {ub_on[5]:.3f} vs {ub_off[5]:.3f}')


# ── Box shape knobs ──────────────────────────────────────────────────────────

def test_clip_to_limits_keeps_the_box_inside_physical_authority(limits):
    """Without it the guard can hand the QP a box the joint cannot execute."""
    q = np.zeros(NV)
    q[3] = -3.05                     # hard against joint4's lower limit
    qdot = np.full(NV, -2.0)         # and still diving into it
    kw = dict(acc_lb=-limits['qddot_max'], acc_ub=limits['qddot_max'],
              qdot_max=limits['qdot_max'], v_margin=0.9,
              q_min=limits['q_min'], q_max=limits['q_max'],
              q_margin=0.05, brake_eta=0.6, dt=0.01)
    lb_on, ub_on = hard_accel_box(q, qdot, clip_to_limits=True, **kw)
    assert np.all(ub_on <= limits['qddot_max'] + 1e-9)
    assert np.all(lb_on <= limits['qddot_max'] + 1e-9)
    assert np.all(ub_on >= lb_on - 1e-9), 'box must stay ordered'


def test_relax_dt_bites_before_the_cap(limits):
    """A longer approach horizon means a smaller allowed q̈ below the cap.

    q̇ has to be inside the final `acc_ub · relax_dt` band of the velocity cap,
    which is the whole point: with dt = 10 ms the one-step bound only bites in
    a 0.17 rad/s sliver on joint5, small enough that hardware ramped to 70 % of
    its limit with the box never once engaging.
    """
    q = _mid_pose()
    qdot = np.full(NV, 2.0)          # approaching the 0.9·2.62 = 2.358 cap
    kw = dict(acc_lb=-limits['qddot_max'], acc_ub=limits['qddot_max'],
              qdot_max=limits['qdot_max'], v_margin=0.9,
              q_min=limits['q_min'], q_max=limits['q_max'],
              q_margin=0.05, brake_eta=0.6, dt=0.01)
    _, ub_relaxed = hard_accel_box(q, qdot, relax_dt=0.10, **kw)
    _, ub_onestep = hard_accel_box(q, qdot, relax_dt=None, **kw)
    assert np.all(ub_relaxed <= ub_onestep + 1e-9)
    assert np.any(ub_relaxed < ub_onestep - 1e-9), 'relax_dt had no effect'


def test_slew_limit_bounds_tick_to_tick_change(cfg, limits):
    """Acceleration continuity: |q̈ − q̈_prev| ≤ max_qddot_delta every tick."""
    f = _filter(cfg, limits)
    delta = float(cfg['cbf']['max_qddot_delta'])
    q = np.zeros(NV); q[3] = -1.5; q[5] = 1.5
    prev = np.zeros(NV)
    for _ in range(5):
        out, _ = f.filter(q, np.zeros(NV), limits['qddot_max'], [])
        assert np.all(np.abs(out - prev) <= delta + 1e-6)
        prev = out


# ── Barrier behaviour ────────────────────────────────────────────────────────

def test_obstacle_row_opposes_motion_into_the_barrier(cfg, limits):
    """The shield must bend a nominal that drives through d_safe."""
    f = _filter(cfg, limits, ws_enable=False)
    d_safe = float(cfg['cbf']['d_safe'])
    q = np.zeros(NV); q[3] = -1.5; q[5] = 1.5
    a = np.zeros(NV); a[0] = 1.0          # ḋ = a·q̇ = q̇₀
    nom = np.zeros(NV); nom[0] = -limits['qddot_max'][0]   # drive INTO it

    free, _ = f.filter(q, np.zeros(NV), nom, [])
    f.reset()
    shielded, info = f.filter(q, np.zeros(NV), nom,
                              [Obstacle('cp', d_safe * 0.5, a, 0.0)])
    assert info.n_obs == 1
    assert shielded[0] > free[0], 'obstacle row did not oppose the approach'
    assert info.intervention > 0.0


def test_reset_clears_the_slew_anchor(cfg, limits):
    """Episode boundaries must not inherit the previous episode's q̈."""
    f = _filter(cfg, limits)
    q = np.zeros(NV); q[3] = -1.5; q[5] = 1.5
    f.filter(q, np.zeros(NV), limits['qddot_max'], [])
    assert np.any(f._qddot_prev != 0.0)
    f.reset()
    assert np.all(f._qddot_prev == 0.0)


def test_solver_is_reproducible(cfg, limits):
    """The same inputs must give the same q̈_safe, every time.

    Guards the `adaptive_rho_interval` pin in cbf_filter.py. With OSQP's
    default of 0 the solver re-adapts rho on a schedule derived from measured
    SETUP TIME, which makes its output a function of wall-clock timing rather
    than only of its inputs — and `--seed` stops meaning anything.
    """
    q = np.zeros(NV); q[3] = -1.5; q[5] = 1.5
    qdot = np.full(NV, 0.3)
    nom = np.array([3.0, -1.0, 2.0, -2.0, 5.0, 1.0, -4.0])
    a = np.zeros(NV); a[0] = 1.0
    outs = []
    for _ in range(6):
        f = _filter(cfg, limits)
        f.reset()
        out, _ = f.filter(q, qdot, nom, [Obstacle('cp', 0.10, a, 0.0)])
        outs.append(out)
    for i, o in enumerate(outs[1:], 1):
        assert np.array_equal(o, outs[0]), (
            f'solve {i} differs from solve 0 by '
            f'{np.abs(o - outs[0]).max():.3e} rad/s^2 on identical inputs')
