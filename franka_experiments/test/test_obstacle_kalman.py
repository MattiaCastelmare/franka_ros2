"""The constant-acceleration Kalman track: does it actually beat a difference?

The whole justification for replacing ``_obstacle_speed`` is that a Kalman
filter smooths and differentiates in ONE step, so it can be both less noisy AND
less laggy than the current "finite difference, then EMA at alpha=0.7" chain.
That is a falsifiable claim and this file falsifies it: the central test
compares the filter's velocity against a plain finite difference of the SAME
noisy samples and asserts, explicitly, both that it converges to the truth and
that its variance is lower.

Everything here is synthetic and seeded. Pure numpy, no ROS.
"""

import numpy as np

from franka_experiments.utils.obstacle_tracker import (
    H, KalmanTrack, process_noise, transition)

DT = 1.0 / 30.0          # the perception rate this runs at
SIGMA = 0.01             # centroid measurement noise [m], one axis


def _run(traj, *, sigma=SIGMA, seed=0, dt=DT, **kw):
    """Feed a noisy version of `traj` (K,3) to a fresh track.

    Returns (track, v_est (K,3), z (K,3)) — the per-frame velocity estimate and
    the noisy measurements, so a finite difference can be taken over exactly the
    same data the filter saw.
    """
    rng = np.random.default_rng(seed)
    z = np.asarray(traj, dtype=np.float64) + rng.normal(scale=sigma, size=np.shape(traj))
    trk = KalmanTrack(z[0], sigma_meas=sigma, **kw)
    v = np.empty_like(z)
    v[0] = trk.velocity
    for k in range(1, len(z)):
        trk.predict(dt)
        trk.update(z[k])
        v[k] = trk.velocity
    return trk, v, z


def _cv_traj(v_true, n=300, dt=DT, p0=(0.0, 0.0, 1.0)):
    t = np.arange(n) * dt
    return np.asarray(p0) + np.outer(t, np.asarray(v_true, dtype=np.float64))


def _finite_difference(z, dt=DT):
    """The estimator this replaces, in its most favourable form: a plain central
    difference of the same samples, with no EMA lag added on top."""
    return (z[2:] - z[:-2]) / (2.0 * dt)


# ── The central claim ───────────────────────────────────────────────────────

def test_velocity_converges_within_5_percent_on_a_constant_velocity_target():
    v_true = np.array([0.6, -0.3, 0.2])
    _, v, _ = _run(_cv_traj(v_true))
    v_settled = v[100:].mean(axis=0)
    err = np.linalg.norm(v_settled - v_true) / np.linalg.norm(v_true)
    assert err < 0.05, f'relative velocity error {err:.3%}'


def test_the_estimate_is_less_noisy_than_a_finite_difference_of_the_same_data():
    """THE comparison. Differencing noisy positions amplifies noise by
    sqrt(2)*sigma/dt -- at sigma=2 cm and 30 Hz that is 0.85 m/s of standard
    deviation on a signal whose whole useful range is a couple of m/s. That is
    why the current pipeline has to EMA the residual, and why the EMA costs
    75 ms of lag. The filter must remove the noise WITHOUT paying that."""
    v_true = np.array([0.6, -0.3, 0.2])
    _, v, z = _run(_cv_traj(v_true))

    fd = _finite_difference(z)
    kf_var = float(np.var(v[100:], axis=0).sum())
    fd_var = float(np.var(fd[100:], axis=0).sum())
    assert kf_var < fd_var, f'KF var {kf_var:.4f} !< FD var {fd_var:.4f}'
    # Not marginally: the point of the exercise is an order-of-magnitude class
    # improvement, and a marginal win would not justify the pipeline.
    assert kf_var < 0.1 * fd_var, f'KF var {kf_var:.4f} vs FD var {fd_var:.4f}'


def test_the_estimate_is_also_more_ACCURATE_than_the_finite_difference():
    """Lower variance alone could be bought with lag or with bias. RMS error
    against the truth is the honest figure, and it must also improve."""
    v_true = np.array([0.6, -0.3, 0.2])
    _, v, z = _run(_cv_traj(v_true))
    fd = _finite_difference(z)
    kf_rms = float(np.sqrt(((v[100:] - v_true) ** 2).sum(axis=1).mean()))
    fd_rms = float(np.sqrt(((fd[100:] - v_true) ** 2).sum(axis=1).mean()))
    assert kf_rms < fd_rms
    assert kf_rms < 0.35 * fd_rms


# ── Zero-velocity target ────────────────────────────────────────────────────

def test_a_stationary_target_keeps_its_speed_near_zero():
    """A static obstacle that reads as moving is the failure that matters most:
    v_obs only ever TIGHTENS the barrier, so a fabricated speed is a permanent,
    silent tax on the workspace. The centroid noise must not be differentiated
    into motion.

    Two separate claims, because they fail differently. The estimate must be
    UNBIASED -- a systematic drift would tighten every barrier in the workspace
    forever -- and its excursions must stay small compared with the speeds the
    barrier is sized for (obstacle_velocity_max = 2.0 m/s)."""
    _, v, _ = _run(_cv_traj([0.0, 0.0, 0.0]), seed=1)
    speed = np.linalg.norm(v[100:], axis=1)
    assert np.abs(v[100:].mean(axis=0)).max() < 0.02, 'the estimate must be unbiased'
    assert speed.mean() < 0.15, f'mean spurious speed {speed.mean():.4f} m/s'
    assert speed.max() < 0.40, f'peak spurious speed {speed.max():.4f} m/s'


def test_the_conservative_clamp_costs_little_on_a_static_obstacle():
    """What the consumer actually sees. v_obs = max(n_hat^T v, 0) discards the
    receding half, so the spurious tightening from a static obstacle is the mean
    of a half-normal, sigma/sqrt(2*pi) -- a fraction of the raw noise, not the
    peak of it. This is the number that matters for "does noise tax the
    workspace", and it is what justifies tuning q_jerk to the responsive side."""
    _, v, _ = _run(_cv_traj([0.0, 0.0, 0.0]), seed=1)
    n_hat = np.array([0.0, 0.0, 1.0])
    v_obs = np.maximum(v[100:] @ n_hat, 0.0)
    assert v_obs.mean() < 0.05, f'mean spurious tightening {v_obs.mean():.4f} m/s'


def test_a_stationary_target_is_quieter_than_its_own_finite_difference():
    _, v, z = _run(_cv_traj([0.0, 0.0, 0.0]), seed=1)
    fd = _finite_difference(z)
    assert np.linalg.norm(v[100:], axis=1).mean() < \
           0.4 * np.linalg.norm(fd[100:], axis=1).mean()


# ── Acceleration: the reason the state carries `a` at all ───────────────────

def test_an_accelerating_target_is_tracked_without_a_growing_lag():
    """A human reach is an acceleration burst, and a constant-VELOCITY filter
    answers one by lagging further behind every frame. Carrying `a` in the state
    means the approach is extrapolated instead of chased."""
    dt, n = DT, 200
    t = np.arange(n) * dt
    acc = np.array([0.0, 0.0, -1.5])
    traj = np.array([0.0, 0.0, 1.5]) + 0.5 * np.outer(t ** 2, acc)
    _, v, _ = _run(traj, seed=3)
    v_true = np.outer(t, acc)
    late_err = np.linalg.norm(v[150:] - v_true[150:], axis=1).mean()
    assert late_err < 0.1, f'lag on a 1.5 m/s^2 ramp: {late_err:.3f} m/s'


def test_acceleration_is_estimated_with_the_right_sign_and_scale():
    dt, n = DT, 250
    t = np.arange(n) * dt
    acc = np.array([0.0, 0.0, -1.5])
    traj = np.array([0.0, 0.0, 1.5]) + 0.5 * np.outer(t ** 2, acc)
    trk, _, _ = _run(traj, seed=4)
    assert np.linalg.norm(trk.acceleration - acc) < 0.5


# ── Covariance: what step 8 consumes ────────────────────────────────────────

def test_velocity_covariance_shrinks_as_evidence_accumulates():
    """A new track knows nothing about its velocity and must SAY so -- step 8
    turns this covariance into a barrier margin, so a filter that reports
    false confidence would silently drop that margin."""
    trk, _, z = _run(_cv_traj([0.5, 0.0, 0.0]))
    fresh = KalmanTrack(z[0], sigma_meas=SIGMA)
    assert np.trace(trk.velocity_cov) < 0.05 * np.trace(fresh.velocity_cov)


def test_coasting_inflates_the_velocity_covariance():
    """A track that has not been measured for three frames is less certain, not
    equally certain. Everything downstream -- the association gate and step 8's
    margin -- depends on that being true."""
    trk, _, _ = _run(_cv_traj([0.5, 0.0, 0.0]))
    before = np.trace(trk.velocity_cov)
    for _ in range(3):
        trk.predict(DT)
    assert np.trace(trk.velocity_cov) > before


def test_speed_variance_along_a_direction_is_the_quadratic_form():
    trk, _, _ = _run(_cv_traj([0.5, 0.0, 0.0]))
    n = np.array([0.6, 0.8, 0.0])
    assert np.isclose(trk.speed_variance_along(n), float(n @ trk.velocity_cov @ n))
    assert trk.speed_variance_along(n) >= 0.0


def test_the_covariance_stays_symmetric_over_a_long_run():
    """Joseph form + explicit symmetrisation. The short (I-KH)P update loses
    symmetry to round-off over minutes of 30 Hz operation, and an indefinite P
    makes step 8's sqrt return NaN."""
    trk, _, _ = _run(_cv_traj([0.4, 0.1, -0.2], n=2000), seed=7)
    assert np.allclose(trk.P, trk.P.T, atol=1e-12)
    assert np.linalg.eigvalsh(trk.P).min() > -1e-12


# ── Model algebra ───────────────────────────────────────────────────────────

def test_transition_is_the_constant_acceleration_kinematics():
    dt = 0.1
    x = np.array([1.0, 2.0, 3.0, 0.5, -0.5, 0.0, 1.0, 0.0, -2.0])
    y = transition(dt) @ x
    assert np.allclose(y[0:3], x[0:3] + dt * x[3:6] + 0.5 * dt ** 2 * x[6:9])
    assert np.allclose(y[3:6], x[3:6] + dt * x[6:9])
    assert np.allclose(y[6:9], x[6:9])


def test_process_noise_is_psd_and_scales_with_the_jerk_psd():
    Q = process_noise(DT, 20.0)
    assert np.allclose(Q, Q.T)
    assert np.linalg.eigvalsh(Q).min() > -1e-18
    assert np.allclose(process_noise(DT, 40.0), 2.0 * Q)


def test_measurement_matrix_selects_position_only():
    x = np.arange(9, dtype=np.float64)
    assert np.allclose(H @ x, x[0:3])


# ── Robustness of predict() against a bad clock ─────────────────────────────

def test_a_nonpositive_or_absurd_dt_is_ignored():
    """Camera stamps duplicate and occasionally go backwards. A negative dt runs
    the model backwards; a 2 s gap inflates P by dt^5 (1e7 against the nominal
    33 ms) and the association gate afterwards accepts anything."""
    trk = KalmanTrack(np.array([0.0, 0.0, 1.0]))
    x0, P0 = trk.x.copy(), trk.P.copy()
    for bad in (0.0, -0.03, 5.0, 1.0):
        trk.predict(bad)
    assert np.array_equal(trk.x, x0) and np.array_equal(trk.P, P0)
    assert trk.age == 1 and trk.missed == 0


def test_update_resets_missed_and_counts_a_frame():
    trk = KalmanTrack(np.array([0.0, 0.0, 1.0]))
    trk.predict(DT)
    trk.predict(DT)
    assert trk.missed == 2 and trk.frames_seen == 1
    trk.update(np.array([0.0, 0.0, 1.0]))
    assert trk.missed == 0 and trk.frames_seen == 2


def test_mahalanobis_widens_while_a_track_coasts():
    """A coasting track's gate must open on its own, or a re-appearing obstacle
    spawns a duplicate id instead of being re-associated (step 3's occlusion
    test depends on exactly this)."""
    trk, _, _ = _run(_cv_traj([0.5, 0.0, 0.0]))
    z = trk.position + np.array([0.05, 0.0, 0.0])
    tight = trk.mahalanobis(z)
    for _ in range(5):
        trk.predict(DT)
    assert trk.mahalanobis(trk.position + np.array([0.05, 0.0, 0.0])) < tight


def test_getters_do_not_hand_out_the_internal_state():
    trk = KalmanTrack(np.array([0.0, 0.0, 1.0]))
    trk.velocity[:] = 99.0
    trk.velocity_cov[:] = 99.0
    assert np.allclose(trk.velocity, 0.0)
    assert np.allclose(trk.velocity_cov, trk.P[3:6, 3:6])
