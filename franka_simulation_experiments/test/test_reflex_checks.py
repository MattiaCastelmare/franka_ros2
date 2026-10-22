"""Where the real FR3 would have stopped (plan H5): the checks, on constructed states."""
import numpy as np
import pytest

from simexp import reflex_checks as R

Q0 = np.array([0.0, -0.70, 0.0, -2.35, 0.0, 1.57, 0.70])


@pytest.fixture()
def chk():
    L = R.robot_limits()
    return R.ReflexChecker(L['q_min'], L['q_max'], L['tau_max'])


def test_limits_are_the_robots(chk):
    np.testing.assert_allclose(chk.tau_max, [87, 87, 87, 87, 12, 12, 12])
    assert chk.rate_max == pytest.approx(1000.0, abs=1e-2)


def test_a_still_arm_at_q0_trips_nothing(chk):
    for k in range(10):
        assert chk.check(k * 1e-3, Q0, np.zeros(7), np.zeros(7), np.zeros(7)) == []
    assert chk.n_trips == 0 and chk.n_samples == 10


def test_velocity_against_the_position_dependent_envelope(chk):
    up, lo = R.envelope(Q0)
    qd = np.zeros(7)
    qd[3] = up[3] + 0.01
    ev = chk.check(0.0, Q0, qd, np.zeros(7), np.zeros(7))
    assert [(k, j) for k, j, *_ in ev] == [('velocity', 3)]
    # near the j4 lower reference the curve is ~0: a SLOW joint trips there
    q = Q0.copy()
    q[3] = -3.04
    qd = np.zeros(7)
    qd[3] = -0.15
    assert ('velocity', 3) in [(k, j) for k, j, *_ in chk.check(1.0, q, qd, np.zeros(7), np.zeros(7))]


def test_position_and_torque_limits(chk):
    q = Q0.copy()
    q[5] = chk.q_max[5] + 1e-3
    tau = np.zeros(7)
    tau[4] = 12.5
    ev = {(k, j) for k, j, *_ in chk.check(0.0, q, np.zeros(7), tau, tau)}
    assert ('position', 5) in ev and ('torque', 4) in ev
    assert chk.n_trips == 2


def test_torque_rate_is_a_clip_not_a_trip(chk):
    """franka_hardware's libfranka limiter clips rate (robot.hpp:318): counted apart."""
    tau0, tau1 = np.zeros(7), np.zeros(7)
    tau1[1] = 1.5                                     # 1.5 N·m in 1 ms = 1500 N·m/s
    chk.check(0.000, Q0, np.zeros(7), tau0, tau0)
    ev = chk.check(0.001, Q0, np.zeros(7), tau1, tau1)
    assert [(k, j) for k, j, *_ in ev] == [(R.CLIP_KIND, 1)]
    assert chk.n_trips == 0 and chk.counts[R.CLIP_KIND] == 1
    assert chk.worst['rate_nms'] == pytest.approx(1500.0)


def test_a_sample_gap_is_not_read_as_a_torque_rate(chk):
    tau1 = np.zeros(7)
    tau1[1] = 1.5
    chk.check(0.000, Q0, np.zeros(7), np.zeros(7), np.zeros(7))
    assert chk.check(0.010, Q0, np.zeros(7), tau1, tau1) == []


def test_rtf_meter():
    m = R.RtfMeter(1.0)
    for k in range(51):
        m.push(sim_s=0.5 * k * 0.1, wall_s=k * 0.1)     # sim at half speed
    s = m.summary()
    assert s['rtf_run'] == pytest.approx(0.5) and s['flagged'] is True
    assert m.windows and all(w == pytest.approx(0.5) for w in m.windows)
    assert R.RtfMeter().summary()['rtf_run'] is None
