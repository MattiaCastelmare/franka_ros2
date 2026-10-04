"""Shared pick-and-place trajectory and the task clock (utils.pick_place_task).

Pure numpy.
"""

import numpy as np
import pytest

from franka_experiments.utils.pick_place_task import PickPlaceTrajectory, TaskClock

DT = 0.01


def _trajectory():
    # The values of config/pick_place.yaml
    return PickPlaceTrajectory(
        home=np.array([0.3, 0.0, 0.5]), lateral_offset=0.24, down_offset=0.20, drop_offset=0.06,
        move_time=2.5, drop_time=0.8, transfer_time=2.5, return_time=2.5,
        wait_pick=0.8, wait_drop=0.8, wait_transfer=0.8, wait_home=0.8,
    )


def test_cycle_has_fourteen_phases_and_lasts_16_3_s():
    traj = _trajectory()
    assert len(traj.phases) == 14
    assert traj.cycle_time == pytest.approx(16.3)


def test_reference_at_unit_rate_is_the_timed_trajectory():
    traj = _trajectory()
    for s in np.linspace(0.0, 2 * traj.cycle_time, 97):
        p, v, a, phase, cycle = traj.evaluate(s)
        p, v, a = p.copy(), v.copy(), a.copy()
        p_d, v_d, a_d, phase_d, cycle_d = traj.reference(s, 1.0, 0.0)
        assert np.allclose(p, p_d) and np.allclose(v, v_d) and np.allclose(a, a_d)
        assert (phase, cycle) == (phase_d, cycle_d)


def test_reference_applies_the_chain_rule():
    traj = _trajectory()
    s, s_dot, s_ddot = 1.1, 0.4, -0.7     # inside MOVE_TO_PICK
    _, v, a, _, _ = traj.evaluate(s)
    v, a = v.copy(), a.copy()
    _, v_d, a_d, _, _ = traj.reference(s, s_dot, s_ddot)
    assert np.allclose(v_d, v * s_dot)
    assert np.allclose(a_d, a * s_dot**2 + v * s_ddot)


def test_clock_runs_at_real_time_while_tracking_is_good():
    clock = TaskClock(e_ok=0.03, e_stop=0.10, s_ddot_max=2.0)
    for _ in range(500):
        s, s_dot, _ = clock.step(DT, tracking_error=0.01)
    assert s == pytest.approx(5.0) and s_dot == 1.0


def test_clock_stops_when_the_arm_is_held_back_and_resumes_after():
    clock = TaskClock(e_ok=0.03, e_stop=0.10, s_ddot_max=2.0)
    clock.step(DT, 0.0)
    prev = clock.s_dot
    for _ in range(100):                      # 1 s held 15 cm off the reference
        clock.step(DT, tracking_error=0.15)
        assert abs(clock.s_dot - prev) <= 2.0 * DT + 1e-12   # rate limit
        prev = clock.s_dot
    assert clock.s_dot == 0.0
    s_held = clock.s
    clock.step(DT, tracking_error=0.15)
    assert clock.s == s_held                  # the reference waits
    for _ in range(100):
        clock.step(DT, tracking_error=0.0)
    assert clock.s_dot == 1.0


def test_clock_rate_is_linear_between_the_thresholds():
    clock = TaskClock(e_ok=0.03, e_stop=0.10, s_ddot_max=2.0)
    assert clock.target_rate(0.065) == pytest.approx(0.5)
    assert clock.target_rate(0.065, rate_cap=0.2) == pytest.approx(0.2)


def test_disabled_clock_is_wall_time():
    clock = TaskClock(e_ok=0.03, e_stop=0.10, s_ddot_max=2.0, enabled=False)
    for _ in range(100):
        s, s_dot, s_ddot = clock.step(DT, tracking_error=1.0)
    assert s == pytest.approx(1.0) and s_dot == 1.0 and s_ddot == 0.0


def test_reference_velocity_stays_continuous_when_the_clock_brakes():
    """No jump in v_d: the rate limit makes the slowdown smooth (transfer peak 0.36 m/s)."""
    traj = _trajectory()
    clock = TaskClock(e_ok=0.03, e_stop=0.10, s_ddot_max=2.0)
    clock.s = 7.5 + 1.25                      # middle of LATERAL_TRANSFER
    prev_v = None
    for k in range(200):
        s, s_dot, s_ddot = clock.step(DT, tracking_error=0.2 if k < 100 else 0.0)
        _, v_d, _, _, _ = traj.reference(s, s_dot, s_ddot)
        if prev_v is not None:
            assert np.linalg.norm(v_d - prev_v) < 0.02
        prev_v = v_d


def test_invalid_thresholds_are_rejected():
    with pytest.raises(ValueError):
        TaskClock(e_ok=0.1, e_stop=0.05, s_ddot_max=2.0)
    with pytest.raises(ValueError):
        TaskClock(e_ok=0.03, e_stop=0.10, s_ddot_max=0.0)