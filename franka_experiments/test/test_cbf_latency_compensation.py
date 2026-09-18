"""Phase 4: move the obstacle forward by the blind time, tighten by the
propagated uncertainty, and never loosen anything.

* flag off → every QP array bit-identical, even with a full track on the wire;
* a constant-velocity track, predicted forward by t_blind from the real
  KalmanTrack state, lands on the ground truth within the filter's own
  tolerance;
* a growing covariance yields a strictly larger margin, never a smaller one;
* a receding obstacle predicts NO tightening; the clamp binds.

Pure numpy; drives the real ConstraintBuilder through the harness.
"""

import numpy as np

from _cbf_builder_harness import (make_builder, make_obstacle, run)
from franka_experiments.utils.cbf_state_rows import latency_compensation_terms
from franka_experiments.utils.obstacle_tracker import KalmanTrack

PR = (0.5, 0.0, 0.5)
PH = (0.5, -0.25, 0.5)
N_HAT = np.array([0.0, 1.0, 0.0])
T = 0.085
KW = dict(t_blind=T, k_sigma=2.0, margin_max=0.25)


def _identical(a, b):
    np.testing.assert_array_equal(a.A, b.A)
    np.testing.assert_array_equal(a.h_bar, b.h_bar)
    np.testing.assert_array_equal(a.jdot_qdot, b.jdot_qdot)
    np.testing.assert_array_equal(a.G, b.G)


def _track_kw():
    return dict(v=(0.0, 0.8, 0.0), a=(0.0, 0.5, 0.0), frames_seen=20, track_id=5,
                cov=np.eye(3) * 0.04, pos_cov=np.eye(3) * 1e-4, pv_cov=np.eye(3) * 1e-3)


# ── Flag off: zero effect ────────────────────────────────────────────────────

def test_flag_off_is_bit_identical_with_a_full_track_on_the_wire():
    plain = run(make_builder(obstacle_velocity_source='tracker'),
                [make_obstacle(pr=PR, ph=PH, **_track_kw())], n_frames=5, qdot=0.05)
    off = run(make_builder(obstacle_velocity_source='tracker',
                           enable_latency_compensation=False),
              [make_obstacle(pr=PR, ph=PH, **_track_kw())], n_frames=5, qdot=0.05)
    _identical(plain, off)


def test_flag_on_tightens_the_barrier_by_exactly_the_two_terms():
    kw = _track_kw()
    off = run(make_builder(obstacle_velocity_source='tracker'),
              [make_obstacle(pr=PR, ph=PH, **kw)], n_frames=5, qdot=0.05)
    on = run(make_builder(obstacle_velocity_source='tracker',
                          enable_latency_compensation=True),
             [make_obstacle(pr=PR, ph=PH, **kw)], n_frames=5, qdot=0.05)
    h_pred, h_unc = latency_compensation_terms(
        N_HAT, kw['v'], kw['a'], kw['pos_cov'], kw['pv_cov'], kw['cov'], **KW)
    assert h_pred > 0.0 and h_unc > 0.0
    assert np.isclose(off.h_bar[0] - on.h_bar[0], min(h_pred + h_unc, 0.25))
    np.testing.assert_array_equal(off.A, on.A)          # only h moves
    np.testing.assert_array_equal(off.jdot_qdot, on.jdot_qdot)


def test_flag_on_with_no_track_is_bit_identical():
    off = run(make_builder(obstacle_velocity_source='tracker'),
              [make_obstacle(pr=PR, ph=PH)], n_frames=5, qdot=0.05)
    on = run(make_builder(obstacle_velocity_source='tracker',
                          enable_latency_compensation=True),
             [make_obstacle(pr=PR, ph=PH)], n_frames=5, qdot=0.05)
    _identical(off, on)


def test_a_young_track_is_gated_out():
    kw = _track_kw(); kw['frames_seen'] = 2
    off = run(make_builder(obstacle_velocity_source='tracker'),
              [make_obstacle(pr=PR, ph=PH, **kw)], n_frames=5, qdot=0.05)
    on = run(make_builder(obstacle_velocity_source='tracker',
                          enable_latency_compensation=True),
             [make_obstacle(pr=PR, ph=PH, **kw)], n_frames=5, qdot=0.05)
    _identical(off, on)


# ── The prediction against ground truth, through the real Kalman filter ─────

def test_constant_velocity_prediction_matches_truth_within_kf_tolerance():
    """A track fed 30 noisy centroids of a constant-velocity obstacle, then
    propagated by t_blind with its own (p, v, a): the predicted position must
    be within the filter's admitted 2σ of the true position at t + t_blind —
    and the gap tightening along n̂ must be the true closing displacement."""
    rng = np.random.default_rng(0)
    dt, v_true, p0 = 1.0 / 30.0, np.array([0.1, -0.9, 0.05]), np.array([0.3, 0.6, 0.4])
    trk = KalmanTrack(p0 + rng.normal(0, 0.01, 3))
    t = 0.0
    for k in range(1, 31):
        t = k * dt
        trk.predict(dt)
        trk.update(p0 + v_true * t + rng.normal(0, 0.01, 3))
    p_pred = trk.position + trk.velocity * T + 0.5 * trk.acceleration * T * T
    p_true = p0 + v_true * (t + T)
    P_t = trk.position_cov + T * (trk.pos_vel_cov + trk.pos_vel_cov.T) + T * T * trk.velocity_cov
    err = p_pred - p_true
    for i in range(3):
        assert abs(err[i]) < 2.0 * np.sqrt(P_t[i, i]) + 0.005, (i, err[i], np.sqrt(P_t[i, i]))
    n = np.array([0.0, -1.0, 0.0])          # obstacle moving along −y closes on a CP at −y
    h_pred, h_unc = latency_compensation_terms(
        n, trk.velocity, trk.acceleration, trk.position_cov, trk.pos_vel_cov,
        trk.velocity_cov, **KW)
    assert np.isclose(h_pred, float(n @ (v_true * T)), atol=0.01)
    assert h_unc > 0.0


# ── Uncertainty only ever grows the margin ───────────────────────────────────

def test_growing_covariance_gives_a_strictly_larger_margin():
    v = np.array([0.0, 0.5, 0.0])
    prev = -1.0
    for scale in (0.5, 1.0, 2.0, 4.0, 8.0):
        _, h_unc = latency_compensation_terms(
            N_HAT, v, None, scale * np.eye(3) * 1e-4, scale * np.eye(3) * 1e-3,
            scale * np.eye(3) * 0.01, **KW)
        assert h_unc > prev
        prev = h_unc


def test_the_cross_term_and_the_velocity_term_both_add():
    v = np.array([0.0, 0.5, 0.0])
    base = latency_compensation_terms(N_HAT, v, None, np.eye(3) * 1e-4, None, None, **KW)[1]
    with_v = latency_compensation_terms(N_HAT, v, None, np.eye(3) * 1e-4, None, np.eye(3) * 0.01, **KW)[1]
    with_pv = latency_compensation_terms(N_HAT, v, None, np.eye(3) * 1e-4, np.eye(3) * 1e-3, np.eye(3) * 0.01, **KW)[1]
    assert base < with_v < with_pv
    # exact propagation of the diagonal case
    var = 1e-4 + T * 2e-3 + T * T * 0.01
    assert np.isclose(with_pv, 2.0 * np.sqrt(var))


def test_the_margin_is_clamped():
    _, h_unc = latency_compensation_terms(N_HAT, np.zeros(3), None, np.eye(3) * 100.0,
                                          None, None, **KW)
    assert h_unc == 0.25


# ── Never loosen ─────────────────────────────────────────────────────────────

def test_a_receding_obstacle_predicts_no_tightening():
    h_pred, _ = latency_compensation_terms(N_HAT, np.array([0.0, -1.0, 0.0]),
                                           np.array([0.0, -2.0, 0.0]), None, None, None, **KW)
    assert h_pred == 0.0


def test_a_lateral_obstacle_predicts_no_tightening():
    h_pred, _ = latency_compensation_terms(N_HAT, np.array([2.0, 0.0, 1.0]), None,
                                           None, None, None, **KW)
    assert abs(h_pred) < 1e-12


def test_both_terms_are_never_negative_on_random_inputs():
    rng = np.random.default_rng(1)
    for _ in range(200):
        n = rng.normal(size=3); n /= np.linalg.norm(n)
        M = rng.normal(size=(3, 3)); Ppp = M @ M.T * 1e-3
        M = rng.normal(size=(3, 3)); Pvv = M @ M.T * 1e-2
        Ppv = rng.normal(size=(3, 3)) * 1e-3
        hp, hu = latency_compensation_terms(n, rng.normal(size=3), rng.normal(size=3),
                                            Ppp, Ppv, Pvv, **KW)
        assert hp >= 0.0 and 0.0 <= hu <= 0.25


def test_zero_blind_time_predicts_nothing_but_keeps_the_position_uncertainty():
    h_pred, h_unc = latency_compensation_terms(
        N_HAT, np.array([0.0, 3.0, 0.0]), np.array([0.0, 9.0, 0.0]),
        np.eye(3) * 1e-4, np.eye(3), np.eye(3), t_blind=0.0, k_sigma=2.0, margin_max=0.25)
    assert h_pred == 0.0
    assert np.isclose(h_unc, 2.0 * 0.01)
