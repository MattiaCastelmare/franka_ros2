"""cbf.enable_vobs_in_hdot — tracked obstacle velocity inside ḣ.

Row convention: build_row_rhs gives h_qp with aᵀq̈ + s ≥ −h_qp, so the row's
b = −h_qp and a LARGER b is a MORE stringent row. Pure numpy, no ROS.
"""
import numpy as np
import pytest

from _cbf_builder_harness import make_builder, make_obstacle, run
from franka_experiments.utils.cbf_qp_assembly import build_row_rhs

K0, K1 = 25.0, 10.5
QD = 0.05
# Default harness geometry: pr - ph = (0, 0.25, 0), so n̂ = +y (obstacle → CP).
N_HAT = np.array([0.0, 1.0, 0.0])


def _builder(vobs, **over):
    b = make_builder(**over)
    if vobs is not None:
        b._P.enable_vobs_in_hdot = vobs
        b._P.vobs_hdot_max = 2.0
    return b


def _b(con):
    h_qp, _ = build_row_rhs(con, np.full(7, QD), np.full(7, QD), k0=K0, k1=K1,
                            retreat_horizon=0.15, speed_horizon=0.10)
    return -h_qp


def _snap(vobs, obstacles, **over):
    b = _builder(vobs, **over)
    return run(b, obstacles, qdot=QD), b


def _tracked(v, frames=5):
    return [make_obstacle(v=v, frames_seen=frames, track_id=7,
                          cp_label='fr3_link5#0')]


# Velocity estimators off in these: flag OFF then has ḣ = aᵀq̇ exactly, so the
# ON-OFF difference is the new term and nothing else.
NO_VEST = dict(obstacle_velocity_enabled=False)


def test_flag_off_is_bit_identical():
    over = dict(obstacle_velocity_source='tracker', enable_velocity_feedforward=True)
    obs = _tracked([0.0, 0.4, 0.1])
    today, _ = _snap(None, obs, **over)          # attribute absent = today's code
    off, _ = _snap(False, obs, **over)
    assert np.array_equal(_b(today), _b(off))
    assert np.array_equal(today.v_obs, off.v_obs)
    assert np.array_equal(today.b_ff, off.b_ff)
    assert np.array_equal(today.h_bar, off.h_bar)


def test_approaching_tightens_by_exactly_k1_vn():
    v = 0.5
    off, _ = _snap(False, _tracked(v * N_HAT), **NO_VEST)
    on, _ = _snap(True, _tracked(v * N_HAT), **NO_VEST)
    assert np.array_equal(on.A, off.A)
    assert np.array_equal(on.h_bar, off.h_bar)
    assert (_b(on) - _b(off))[0] == pytest.approx(K1 * v, rel=1e-12)


def test_receding_relaxes_by_exactly_k1_vn():
    v = 0.5
    off, _ = _snap(False, _tracked(-v * N_HAT), **NO_VEST)
    on, _ = _snap(True, _tracked(-v * N_HAT), **NO_VEST)
    assert (_b(on) - _b(off))[0] == pytest.approx(-K1 * v, rel=1e-12)


@pytest.mark.parametrize('obs', [
    [make_obstacle(cp_label='fr3_link5#0')],                            # no track
    [make_obstacle(v=[0, 0.5, 0], frames_seen=2, track_id=7,
                   cp_label='fr3_link5#0')],                            # unconfirmed
])
def test_no_confirmed_track_is_identical_to_flag_off(obs):
    over = dict(obstacle_velocity_source='tracker', enable_velocity_feedforward=True)
    off, _ = _snap(False, obs, **over)
    on, b = _snap(True, obs, **over)
    assert np.array_equal(_b(on), _b(off))
    assert np.array_equal(on.b_ff, off.b_ff)
    assert b.diag_vobs_hdot_n == 0


def test_speed_clamped_to_vobs_hdot_max():
    off, _ = _snap(False, _tracked(10.0 * N_HAT), **NO_VEST)
    on, b = _snap(True, _tracked(10.0 * N_HAT), **NO_VEST)
    assert (_b(on) - _b(off))[0] == pytest.approx(K1 * 2.0, rel=1e-12)
    assert b.diag_vobs_hdot == 2.0 and b.diag_vobs_hdot_n == 1


def test_feedforward_rhs_term_dropped_on_tracked_row():
    over = dict(obstacle_velocity_source='tracker', obstacle_velocity_track_deadband=0.0,
                enable_velocity_feedforward=True)
    v = 0.5
    off, _ = _snap(False, _tracked(v * N_HAT), **over)
    on, _ = _snap(True, _tracked(v * N_HAT), **over)
    assert off.b_ff[0] == pytest.approx(-1.0 * v)     # today: -k_ff·v_app
    assert on.b_ff[0] == 0.0                          # not double counted
    assert np.array_equal(on.h_bar, off.h_bar)        # braking tightening kept
    a = on.A[0]
    expected = -(K1 * (a @ np.full(7, QD) - v) + K0 * on.h_bar[0] + on.jdot_qdot[0])
    assert _b(on)[0] == pytest.approx(expected, rel=1e-12)


def test_two_rows_same_cp_different_tracks():
    # What step A publishes with multi_obstacle_k = 2: one control point (same
    # pr), two obstacles, two labels, two tracks moving differently.
    pr = (0.5, 0.0, 0.5)
    ph1, ph2 = (0.5, -0.25, 0.5), (0.25, 0.0, 0.5)
    n1 = np.array([0.0, 1.0, 0.0])
    n2 = np.array([1.0, 0.0, 0.0])
    obs = [make_obstacle(d=0.25, pr=pr, ph=ph1, v=0.6 * n1, frames_seen=5,
                         track_id=1, cp_label='fr3_link5#0'),
           make_obstacle(d=0.25, pr=pr, ph=ph2, v=-0.3 * n2, frames_seen=5,
                         track_id=2, cp_label='fr3_link5#0.1')]
    off, _ = _snap(False, obs, **NO_VEST)
    on, b = _snap(True, obs, **NO_VEST)
    assert on.links == ('fr3_link5#0', 'fr3_link5#0.1')
    assert on.v_obs[0] == pytest.approx(0.6) and on.v_obs[1] == pytest.approx(-0.3)
    d = _b(on) - _b(off)
    assert d[0] == pytest.approx(K1 * 0.6, rel=1e-12)
    assert d[1] == pytest.approx(-K1 * 0.3, rel=1e-12)
    assert b.diag_vobs_hdot_n == 2 and b.diag_vobs_hdot == pytest.approx(0.6)
