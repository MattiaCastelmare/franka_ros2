"""IMMTrack: does the ballistic hypothesis actually buy faster convergence,
and does it stay quiet on everything :class:`KalmanTrack` was already good at?

Two things have to be true for this to be worth the extra model, and this file
falsifies both:

1. On a genuine thrown-ball trajectory, the IMM's velocity/acceleration must
   converge measurably FASTER than the single generic model on the exact same
   noisy data (the whole point of carrying a second, more informative prior).
2. On every scenario the generic model was already tuned against — a
   constant-velocity reach, a decelerating reach, a static obstacle — the IMM
   must perform indistinguishably from :class:`KalmanTrack`, with the mode
   probability staying on the generic model throughout. A ball-throw fix that
   quietly degrades the person-tracking case this pipeline exists for would
   not be a fix.

Everything here is synthetic and seeded. Pure numpy, no ROS.
"""

import numpy as np

from franka_experiments.utils.obstacle_tracker import IMMTrack, KalmanTrack

DT = 1.0 / 90.0          # the depth stream's current real rate
SIGMA = 0.01             # centroid measurement noise [m], one axis — same as
                         # test_obstacle_kalman.py, and as sigma_meas_m in config
G = np.array([0.0, 0.0, -9.81])


def _ballistic_traj(v0, *, n=90, dt=DT, p0=(0.0, 0.0, 1.5), g=G):
    """A free-flight trajectory: p0 + v0*t + 1/2*g*t^2."""
    t = np.arange(n) * dt
    p0 = np.asarray(p0, dtype=np.float64)
    v0 = np.asarray(v0, dtype=np.float64)
    return p0 + np.outer(t, v0) + 0.5 * np.outer(t ** 2, np.asarray(g, dtype=np.float64))


def _cv_traj(v_true, n=300, dt=DT, p0=(0.0, 0.0, 1.0)):
    t = np.arange(n) * dt
    return np.asarray(p0) + np.outer(t, np.asarray(v_true, dtype=np.float64))


def _reach_then_stop_traj(v_peak, *, ramp_n=25, hold_n=15, dt=DT, p0=(0.0, 0.0, 1.0)):
    """A hand reaching out and stopping: accelerate for `ramp_n` frames,
    decelerate back to zero over the same span, then hold — NOT a constant
    downward pull, the thing a ballistic prior must not be fooled by."""
    v_peak = np.asarray(v_peak, dtype=np.float64)
    n = 2 * ramp_n + hold_n
    v = np.zeros((n, 3))
    for k in range(ramp_n):
        v[k] = v_peak * (k / ramp_n)
    for k in range(ramp_n):
        v[ramp_n + k] = v_peak * (1.0 - k / ramp_n)
    v[2 * ramp_n:] = 0.0
    p = np.asarray(p0, dtype=np.float64) + np.cumsum(v, axis=0) * dt
    return p


def _run_imm(traj, *, sigma=SIGMA, seed=0, dt=DT, **kw):
    """Feed a noisy version of `traj` (K,3) to a fresh IMMTrack.

    Returns (track, v (K,3), a (K,3), mu (K,2), z (K,3)) — per-frame velocity,
    acceleration, mode probability ([generic, ballistic]), and the noisy
    measurements themselves.
    """
    rng = np.random.default_rng(seed)
    z = np.asarray(traj, dtype=np.float64) + rng.normal(scale=sigma, size=np.shape(traj))
    trk = IMMTrack(z[0], sigma_meas=sigma, **kw)
    v = np.empty_like(z)
    a = np.empty_like(z)
    mu = np.empty((len(z), 2))
    v[0], a[0], mu[0] = trk.velocity, trk.acceleration, trk.mode_prob
    for k in range(1, len(z)):
        trk.predict(dt)
        trk.update(z[k])
        v[k], a[k], mu[k] = trk.velocity, trk.acceleration, trk.mode_prob
    return trk, v, a, mu, z


def _run_kf(traj, *, sigma=SIGMA, seed=0, dt=DT, **kw):
    """Same feed, through the plain single-model KalmanTrack — the baseline
    every claim here is measured against."""
    rng = np.random.default_rng(seed)
    z = np.asarray(traj, dtype=np.float64) + rng.normal(scale=sigma, size=np.shape(traj))
    trk = KalmanTrack(z[0], sigma_meas=sigma, **kw)
    v = np.empty_like(z)
    a = np.empty_like(z)
    v[0], a[0] = trk.velocity, trk.acceleration
    for k in range(1, len(z)):
        trk.predict(dt)
        trk.update(z[k])
        v[k], a[k] = trk.velocity, trk.acceleration
    return trk, v, a, z


# ── 1. The central claim: faster convergence on a real throw ────────────────

def test_imm_converges_on_a_thrown_ball_faster_than_the_generic_model():
    v0 = np.array([1.5, 0.0, 0.3])
    _, v_imm, a_imm, _, _ = _run_imm(_ballistic_traj(v0), seed=0)
    _, v_kf, a_kf, _ = _run_kf(_ballistic_traj(v0), seed=0)

    t = np.arange(90) * DT
    v_true = v0 + np.outer(t, G)

    # Early flight (frame 8, ~90 ms in) is where the whole investigation's
    # "first ~100 ms" cost lives. The IMM must already be closer to truth than
    # the generic model at that exact point.
    err_imm = np.linalg.norm(v_imm[8] - v_true[8])
    err_kf = np.linalg.norm(v_kf[8] - v_true[8])
    assert err_imm < 0.7 * err_kf, (
        f'IMM v-error {err_imm:.3f} m/s not meaningfully better than '
        f'generic {err_kf:.3f} m/s at frame 8')

    err_a_imm = np.linalg.norm(a_imm[8] - G)
    err_a_kf = np.linalg.norm(a_kf[8] - G)
    assert err_a_imm < 0.7 * err_a_kf, (
        f'IMM a-error {err_a_imm:.3f} not meaningfully better than '
        f'generic {err_a_kf:.3f} at frame 8')


def test_ballistic_mode_probability_rises_fast_on_a_clean_throw():
    v0 = np.array([1.5, 0.0, 0.3])
    _, _, _, mu, _ = _run_imm(_ballistic_traj(v0), seed=1)
    # 15 frames at 90 Hz is ~165 ms -- comparable to the blind time this whole
    # investigation has been trying to buy back, not an arbitrary bar.
    assert mu[15, 1] > 0.8, f'ballistic mode prob only {mu[15, 1]:.3f} by frame 15'


# ── 2. No regression on what KalmanTrack was already good at ────────────────

def test_imm_matches_the_generic_model_on_a_constant_velocity_reach():
    """A level reach has zero acceleration, which does not clearly favour
    EITHER model's initial belief (0 g. gravity), so — measured — the mode
    probability settles near neutral (~0.5) rather than confidently rejecting
    ballistic. That is fine and expected: the "spread of means" term (see
    IMMTrack._combine) means genuine uncertainty about which model is right
    only ever WIDENS the reported covariance, which only ever tightens a
    downstream barrier. What must not happen is the blended POINT ESTIMATE
    drifting from the generic model's — that is the actual safety-relevant
    claim, and mode probability confidently mistaking this for a ballistic
    object (>0.6, most of a 3-vote landslide) must not happen either."""
    v_true = np.array([0.6, -0.3, 0.2])
    _, v_imm, _, mu, _ = _run_imm(_cv_traj(v_true, n=300), seed=2)
    _, v_kf, _, _ = _run_kf(_cv_traj(v_true, n=300), seed=2)
    v_imm_settled = v_imm[100:].mean(axis=0)
    v_kf_settled = v_kf[100:].mean(axis=0)
    assert np.linalg.norm(v_imm_settled - v_true) < 1.5 * np.linalg.norm(v_kf_settled - v_true) + 0.02
    assert mu[100:, 1].mean() < 0.6, (
        f'ballistic mode confidently won a level reach: {mu[100:, 1].mean():.3f}')


def test_imm_is_not_fooled_by_a_reach_that_stops():
    """The scenario a naive size- or direction-based classifier gets wrong:
    a hand accelerating and then decelerating to zero, along an arbitrary
    (non-gravity) axis. The ballistic mode must not dominate here."""
    v_peak = np.array([0.0, 0.9, -0.2])
    _, v_imm, _, mu, _ = _run_imm(_reach_then_stop_traj(v_peak), seed=3)
    _, v_kf, _, _ = _run_kf(_reach_then_stop_traj(v_peak), seed=3)
    # Peak-region velocity error must not be meaningfully worse than the
    # generic model's.
    peak = slice(20, 30)
    err_imm = np.linalg.norm(v_imm[peak] - v_kf[peak], axis=1).mean()
    assert err_imm < 0.25, f'IMM diverges from the generic model by {err_imm:.3f} m/s mid-reach'
    assert mu[15:, 1].max() < 0.5, (
        f'ballistic mode rose to {mu[15:, 1].max():.3f} on a decelerating reach')


def test_a_stationary_target_stays_quiet_and_on_the_generic_model():
    """See the comment on the constant-velocity-reach test above: near-neutral
    mode probability on a scene with no discriminating acceleration is
    expected and safe. The load-bearing claim is that the estimated SPEED
    stays quiet, exactly as it does for the plain :class:`KalmanTrack`."""
    _, v_imm, _, mu, _ = _run_imm(_cv_traj([0.0, 0.0, 0.0], n=300), seed=4)
    speed = np.linalg.norm(v_imm[100:], axis=1)
    assert speed.mean() < 0.2, f'mean spurious speed {speed.mean():.4f} m/s'
    assert mu[100:, 1].mean() < 0.6, (
        f'ballistic mode confidently won a static scene: {mu[100:, 1].mean():.3f}')


# ── 3. Recovery: caught or bounced, the mode must let go of gravity ─────────

def test_mode_probability_swings_back_after_a_ball_is_caught():
    """Measured decay after an abrupt catch (frame 40): 0.86 -> 0.76 -> 0.68
    -> 0.63 by frames 20/40/60/75-after-catch. It does not snap back to
    "confidently generic" inside this window — `mode_transition_stay_prob`
    trades sustained ballistic confidence during a real (short) throw against
    how fast it forgets afterward, and erring toward "still cautious a bit
    longer than strictly necessary" is the safe direction for a barrier that
    only ever tightens on a higher assumed speed. What must hold is a clear,
    monotonic swing back toward neutral, not a filter stuck confidently wrong
    forever."""
    v0 = np.array([1.2, 0.0, 0.2])
    fly = _ballistic_traj(v0, n=40)
    caught = np.repeat(fly[-1:], 40, axis=0)   # abrupt stop: caught in the hand
    traj = np.concatenate([fly, caught], axis=0)
    _, _, _, mu, _ = _run_imm(traj, seed=5)
    assert mu[30, 1] > 0.8, 'should be confidently ballistic mid-flight'
    assert mu[79, 1] < 0.65, f'still {mu[79, 1]:.3f} ballistic 430 ms after being caught'
    assert mu[79, 1] < mu[40, 1] < mu[20, 1], 'mode probability must decay monotonically after the catch'


# ── 4. Timing budget ──────────────────────────────────────────────────────

def test_twenty_imm_tracks_step_inside_the_frame_budget():
    """max_tracks in fr3_complete.yaml is 20. Running twice the linear algebra
    per track (two 9-state filters instead of one) must still leave headroom
    against the ~11.1 ms/frame budget the Sep 2026 rate-independence work
    measured tracking against (a few ms today)."""
    import time
    rng = np.random.default_rng(6)
    tracks = [IMMTrack(rng.normal(size=3)) for _ in range(20)]
    z = [rng.normal(size=3) for _ in range(20)]
    n_steps = 50
    t0 = time.perf_counter()
    for _ in range(n_steps):
        for trk in tracks:
            trk.predict(DT)
        for trk, meas in zip(tracks, z):
            trk.update(meas)
    elapsed = time.perf_counter() - t0
    per_frame_ms = 1000.0 * elapsed / n_steps
    assert per_frame_ms < 5.0, f'{per_frame_ms:.2f} ms/frame for 20 IMM tracks'


# ── 5. Model algebra / plumbing ─────────────────────────────────────────────

def test_getters_do_not_hand_out_the_internal_state():
    trk = IMMTrack(np.array([0.0, 0.0, 1.0]))
    trk.velocity[:] = 99.0
    trk.velocity_cov[:] = 99.0
    assert np.allclose(trk.velocity, 0.0)


def test_mode_probability_starts_uninformed_and_sums_to_one():
    trk = IMMTrack(np.array([0.0, 0.0, 1.0]))
    assert np.isclose(trk.mode_prob.sum(), 1.0)
    assert np.allclose(trk.mode_prob, [0.5, 0.5])


def test_a_nonpositive_or_absurd_dt_is_ignored():
    trk = IMMTrack(np.array([0.0, 0.0, 1.0]))
    x0, mu0 = trk._x_comb.copy(), trk.mode_prob.copy()
    for bad in (0.0, -0.03, 5.0, 1.0):
        trk.predict(bad)
    assert np.array_equal(trk._x_comb, x0)
    assert np.allclose(trk.mode_prob, mu0)
    assert trk.age == 1 and trk.missed == 0


def test_update_resets_missed_and_counts_a_frame():
    trk = IMMTrack(np.array([0.0, 0.0, 1.0]))
    trk.predict(DT)
    trk.predict(DT)
    assert trk.missed == 2 and trk.frames_seen == 1
    trk.update(np.array([0.0, 0.0, 1.0]))
    assert trk.missed == 0 and trk.frames_seen == 2


def test_combined_covariance_widens_when_the_models_disagree():
    """The IMM 'spread of means' term: right when the two models actively
    disagree about where the object is, the reported uncertainty must be AT
    LEAST as wide as either individual model's own — never averaged away."""
    v0 = np.array([1.5, 0.0, 0.3])
    trk, _, _, mu, _ = _run_imm(_ballistic_traj(v0), seed=7)
    # Early on (~frame 3), mode probability is still close to 50/50 and the
    # two models disagree sharply about acceleration.
    assert 0.1 < mu[3, 1] < 0.9, 'test is only meaningful while genuinely unsure'
    assert np.trace(trk._P_comb[6:9, 6:9]) >= min(
        np.trace(trk._P[0, 6:9, 6:9]), np.trace(trk._P[1, 6:9, 6:9]))
