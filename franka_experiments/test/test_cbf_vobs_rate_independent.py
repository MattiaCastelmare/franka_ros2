"""v_obs must be conditioned the same way whatever rate perception runs at.

WHY THIS FILE EXISTS
--------------------
``v_obs`` is a DERIVATIVE of a noisy distance, it is multiplied into a binding
row by ``k1 = 10.5``, and this repo has already measured what happens when it
gets noisier: a static obstacle produced a square wave 0.000 → 0.085 → 0.000 →
0.035 m/s whose edges were the two largest single-tick jumps in the log
(``cbf_state_rows._residual_velocity``), and the arm oscillated.

Four devices keep that under control — the differencing interval, the EMA, the
median window and the rotation hold — and all four were tuned against a 30 Hz
perception stream, three of them expressed as FRAME COUNTS. The stream is a
parameter (``camera_depth_profile``), and when it went to 848x480x90 the
constraint builder started finding a new frame on every 50 Hz rebuild instead of
on two out of three: the windows shrank by 1.65x and the differencing noise grew
by the same factor. The arm oscillated against a static obstacle again.

So the tests below are about a PROPERTY, not about a number: the conditioned
estimate a static obstacle produces must not depend on the perception rate.
"""

import numpy as np
import pytest

from _cbf_builder_harness import (
    make_builder,
    make_js,
    make_obs,
    make_obstacle,
)

D0 = 0.25            # [m] the static obstacle's true gap
SIGMA = 0.004        # [m] per-frame depth noise, the scale this pipeline sees
RATE_TUNED = 30.0    # the rate every gain in fr3_control.yaml was measured at
RATE_NOW = 84.0      # what the 848x480x90 profile actually delivers

#: The durations the frame-counted numbers stood for at 30 Hz.
DURATIONS = dict(
    obstacle_velocity_dt_min_s=1.0 / RATE_TUNED,
    obstacle_velocity_tau_s=0.075,
    obstacle_velocity_median_s=5.0 / RATE_TUNED,
)


def _noise(seed=7, n=4000):
    """One fixed white sequence, sampled the same way at every rate.

    White is the point: depth noise is uncorrelated frame to frame, so a
    difference quotient over a shorter interval divides the SAME noise by a
    smaller number. A smooth test signal would show the opposite — a shorter
    interval estimates a true derivative better — and would prove nothing.
    """
    return np.random.default_rng(seed).normal(0.0, SIGMA, n)


def _v_obs_series(rate_hz, *, seconds=3.0, qdot=0.0, params=None, seed=7):
    """Conditioned v_obs, per rebuild, for a STATIC obstacle at ``rate_hz``.

    The builder is driven at its own 50 Hz, as in the node; a new perception
    sample appears whenever the stream has produced one, which is what makes
    the 30 Hz and 84 Hz cases differ in exactly the way the rig did.
    """
    b = make_builder(obstacle_velocity_median=5, **(params or {}))
    noise = _noise(seed)
    out = []
    build_dt = 1.0 / 50.0
    n_builds = int(seconds / build_dt)
    for k in range(n_builds):
        t = k * build_dt
        idx = int(t * rate_hz)                 # which perception frame is live
        t_cap = idx / rate_hz                  # and its capture stamp
        d = D0 + noise[idx % len(noise)]
        ob = make_obstacle(d=d, frames_seen=50)
        b.build(make_js(qdot=qdot, stamp=t), make_obs([ob], stamp=t,
                                                      t_cap=t_cap), t)
        out.append(float(b.diag_v_obs))
    return np.asarray(out)


# ── The regression, and the fix ──────────────────────────────────────────────

def test_frame_counted_conditioning_gets_noisier_when_the_rate_rises():
    """The regression, stated: same static obstacle, 2.8x the rate, more noise.

    Nothing about the obstacle changed. Only the camera profile did.
    """
    tuned = _v_obs_series(RATE_TUNED)
    now = _v_obs_series(RATE_NOW)
    assert now.std() > 1.5 * tuned.std(), (
        f'expected the 84 Hz series to be visibly noisier with frame-counted '
        f'conditioning, got {now.std():.4f} vs {tuned.std():.4f}')


def test_durations_restore_the_tuned_conditioning():
    """With every window a duration, the rate stops mattering."""
    tuned = _v_obs_series(RATE_TUNED)
    fixed = _v_obs_series(RATE_NOW, params=DURATIONS)
    assert fixed.std() <= 1.3 * tuned.std(), (
        f'84 Hz with durations should condition like 30 Hz did, got '
        f'{fixed.std():.4f} vs {tuned.std():.4f}')


@pytest.mark.parametrize('rate', [30.0, 45.0, 60.0, 84.0, 90.0])
def test_the_estimate_is_bounded_at_every_rate(rate):
    """A static obstacle must not manufacture approach speed at ANY rate."""
    v = _v_obs_series(rate, params=DURATIONS)
    assert np.all(np.isfinite(v))
    # The residual is clamped to the approaching half upstream, so the mean is
    # positive by construction; what must stay small is its SIZE.
    assert v.mean() < 0.05, f'fabricated {v.mean():.3f} m/s at {rate} Hz'


# ── The individual devices ──────────────────────────────────────────────────

def test_the_differencing_interval_has_a_floor():
    """Two rebuilds inside dt_min must not produce a new difference quotient."""
    # alpha 0 so the reading IS the difference quotient: what is under test is
    # the interval it was taken over, not the filter behind it.
    b = make_builder(obstacle_velocity_dt_min_s=0.033,
                     obstacle_velocity_alpha=0.0)
    ob = make_obstacle(d=D0)
    b.build(make_js(stamp=0.0), make_obs([ob], stamp=0.0, t_cap=0.0), 0.0)
    # A second sample 10 ms later: too soon, the estimate is HELD.
    b.build(make_js(stamp=0.02), make_obs([make_obstacle(d=D0 - 0.01)],
                                          stamp=0.02, t_cap=0.010), 0.02)
    held = float(b.diag_v_obs)
    assert held == 0.0, 'a sub-floor interval must hold, not difference'
    # 40 ms after the anchor: now it differences, and over the full 40 ms.
    b.build(make_js(stamp=0.05), make_obs([make_obstacle(d=D0 - 0.01)],
                                          stamp=0.05, t_cap=0.040), 0.05)
    assert float(b.diag_v_obs) == pytest.approx(0.01 / 0.040, rel=0.15)


def test_without_the_floor_the_same_pair_differences_over_10_ms():
    """The contrast: the legacy path divides by whatever interval it is given."""
    b = make_builder(obstacle_velocity_dt_min_s=0.0,
                     obstacle_velocity_alpha=0.0)
    b.build(make_js(stamp=0.0), make_obs([make_obstacle(d=D0)],
                                         stamp=0.0, t_cap=0.0), 0.0)
    b.build(make_js(stamp=0.02), make_obs([make_obstacle(d=D0 - 0.01)],
                                          stamp=0.02, t_cap=0.010), 0.02)
    assert float(b.diag_v_obs) == pytest.approx(0.01 / 0.010, rel=0.15)


def test_the_ema_keeps_its_time_constant_across_rates():
    """A 75 ms lag must be 75 ms at 30 Hz and at 84 Hz.

    Driven with a step in the distance rate, the estimate's rise after one time
    constant is the same fraction at both rates — which a per-frame weight
    cannot do.
    """
    def rise(rate):
        b = make_builder(obstacle_velocity_tau_s=0.075,
                         obstacle_velocity_dt_min_s=0.0)
        dt = 1.0 / rate
        d, v = D0, []
        for k in range(int(0.075 / dt) + 1):
            d -= 0.30 * dt                     # a steady 0.30 m/s approach
            t = k * dt
            b.build(make_js(stamp=t), make_obs([make_obstacle(d=d)], stamp=t,
                                               t_cap=t), t)
            v.append(float(b.diag_v_obs))
        return v[-1] / 0.30                    # fraction of the true speed

    assert rise(30.0) == pytest.approx(rise(84.0), abs=0.12)


def test_the_median_window_is_a_duration():
    """5 taps at 30 Hz is 167 ms; at 84 Hz the window must still be 167 ms."""
    b = make_builder(obstacle_velocity_median=5,
                     obstacle_velocity_median_s=0.167,
                     obstacle_velocity_dt_min_s=0.0)
    dt = 1.0 / 84.0
    for k in range(40):
        t = k * dt
        b.build(make_js(stamp=t), make_obs([make_obstacle(d=D0)], stamp=t,
                                           t_cap=t), t)
    hist = next(iter(b._obs_vmed.values()))
    span = hist[-1][0] - hist[0][0]
    assert span == pytest.approx(0.167, abs=0.02), (
        f'the window spans {span * 1e3:.0f} ms, not the configured 167')
    assert len(hist) > 5, 'at 84 Hz a 167 ms window holds more than 5 taps'


def test_the_rotation_hold_is_a_duration():
    """The guard may hold a stale estimate for a TIME, not for a frame count.

    With the normal flipping on every frame the guard rejects until it gives
    up; where it gives up must be the same instant at 30 Hz and at 84 Hz.
    """
    def break_through(rate):
        b = make_builder(obstacle_velocity_normal_rot_max=0.15,
                         obstacle_velocity_rot_hold_s=0.10,
                         obstacle_velocity_dt_min_s=0.0)
        dt = 1.0 / rate
        # Alternate the closest point across the arm so n̂ hops every frame.
        for k in range(int(0.4 / dt)):
            t = k * dt
            ph = (0.5, -0.25, 0.5) if k % 2 else (0.5, 0.25, 0.5)
            b.build(make_js(stamp=t),
                    make_obs([make_obstacle(d=D0, ph=ph)], stamp=t, t_cap=t), t)
            if b.diag_rot_reject and k * dt >= 0.10:
                return b.diag_rot_reject, k * dt
        return b.diag_rot_reject, None

    n30, t30 = break_through(30.0)
    n84, t84 = break_through(84.0)
    # The COUNT of rejections scales with the rate (more frames in the same
    # window); the WINDOW does not, which is the whole point.
    assert n84 > n30
    assert t30 is not None and t84 is not None


# ── The evidence gates, derived from the measured rate ──────────────────────

def test_the_evidence_gate_is_derived_from_the_measured_rate():
    """8 measurements at 30 Hz and 22 at 84 Hz are the same 265 ms of evidence."""
    b = make_builder(obstacle_velocity_min_span_s=0.265,
                     velocity_feedforward_min_span_s=0.10)
    assert b._min_frames_eff == 8            # sized from obstacle_input_rate_hz
    assert b.set_input_rate(84.0) is True
    assert b._min_frames_eff == 22
    assert b._ff_min_frames_eff == 8
    assert b.set_input_rate(84.0) is False   # idempotent
    assert b.set_input_rate(30.0) is True
    assert b._min_frames_eff == 8


def test_an_unmigrated_config_keeps_its_frame_counts():
    """With no span configured the count is the config's, at any rate."""
    b = make_builder(obstacle_velocity_min_frames=8,
                     obstacle_velocity_min_span_s=0.0)
    assert b._min_frames_eff == 8
    b.set_input_rate(84.0)
    assert b._min_frames_eff == 8


def test_a_nonsense_rate_is_refused():
    b = make_builder(obstacle_velocity_min_span_s=0.265)
    before = b._min_frames_eff
    assert b.set_input_rate(0.0) is False
    assert b.set_input_rate(-5.0) is False
    assert b._min_frames_eff == before


# ── Back-compat ─────────────────────────────────────────────────────────────

def test_the_durations_off_reproduce_the_legacy_path():
    """Every new key at 0 must be the pre-2026-09-21 behaviour, value for value."""
    legacy = _v_obs_series(RATE_NOW)
    again = _v_obs_series(RATE_NOW, params=dict(
        obstacle_velocity_dt_min_s=0.0, obstacle_velocity_tau_s=0.0,
        obstacle_velocity_median_s=0.0, obstacle_velocity_rot_hold_s=0.0))
    assert np.array_equal(legacy, again)


def test_the_shipped_control_config_states_the_durations():
    import os
    import yaml
    here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    with open(os.path.join(here, 'config', 'fr3_control.yaml')) as fh:
        p = yaml.safe_load(fh)['params']
    assert p['obstacle_velocity_dt_min_s'] == pytest.approx(0.033, abs=1e-3)
    assert p['obstacle_velocity_tau_s'] == pytest.approx(0.075)
    # 0.30, not the 0.167 the 5 taps stood for: widened against the
    # static-obstacle artefact, see test_the_shipped_median_window_matches_the_measurement.
    assert p['obstacle_velocity_median_s'] == pytest.approx(0.30)
    assert p['obstacle_velocity_rot_hold_s'] == pytest.approx(0.10)
    assert p['obstacle_velocity_min_span_s'] == pytest.approx(0.265)
    # The nominal input rate must not be hard-coded anywhere; it is a parameter
    # and it has to agree with the profile the camera is launched with.
    assert p['obstacle_input_rate_hz'] == pytest.approx(90.0)


# ── One source of truth for the rate ────────────────────────────────────────

def test_the_profile_string_is_where_the_rate_comes_from():
    """848x480x90 -> 90.0, and nothing else in the stack hard-codes it."""
    import importlib.util
    import os
    here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    spec = importlib.util.spec_from_file_location(
        'tcs_launch', os.path.join(here, 'launch',
                                   'torque_control_stack.launch.py'))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)

    assert mod._profile_fps('848x480x90') == pytest.approx(90.0)
    assert mod._profile_fps('640x480x30') == pytest.approx(30.0)
    assert mod._profile_fps('848x480x15') == pytest.approx(15.0)
    # Empty = the driver chooses, so neither config is overridden and each
    # keeps its own value.
    assert mod._profile_fps('') is None
    assert mod._profile_fps('848x480') is None
    assert mod._profile_fps('848x480xfast') is None
    assert mod._profile_fps('848x480x0') is None


def test_the_two_configs_agree_with_the_launched_profile():
    """The claim in each config must match the profile the stack launches.

    They are pushed from the profile at launch, so a mismatch here is only a
    stale default — but a stale default is what sizes the gates in any run that
    does not go through the launch file, including every replay harness.
    """
    import os
    import yaml
    here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    load = lambda n: yaml.safe_load(open(os.path.join(here, 'config', n)))
    profile = load('launch_defaults.yaml')['camera_depth_profile']
    fps = float(profile.strip().split('x')[2])
    assert load('fr3_complete.yaml')['distance']['depth_rate_hz'] == fps
    assert load('fr3_control.yaml')['params']['obstacle_input_rate_hz'] == fps


# ── The static-obstacle artefact, and the price of filtering it ─────────────
# The reproduction: a STATIC obstacle, the arm sweeping past it, and the
# nearest point hopping between two surface patches 5 cm apart — which is the
# mechanism `_residual_velocity` names for the square wave it measured on
# hardware. These tests pin BOTH sides of the median's trade-off, because the
# window was widened on the strength of exactly these two numbers and a later
# retune must move them both on purpose.

HOP_EVERY = 4            # perception frames between patch hops
HOP_M = 0.03             # [m] the step in d a hop produces


def _artefact_steps(median_s, *, seconds=6.0, rate=84.0):
    """p95 of the frame-to-frame step in the CONSUMED v_obs, on the artefact."""
    b = make_builder(obstacle_velocity_median=5,
                     obstacle_velocity_dt_min_s=0.033,
                     obstacle_velocity_tau_s=0.075,
                     obstacle_velocity_rot_hold_s=0.10,
                     obstacle_velocity_normal_rot_max=0.15,
                     obstacle_velocity_median_s=median_s)
    noise = _noise(3, 20000)
    v = []
    for k in range(int(seconds * 50.0)):
        t = k / 50.0
        idx = int(t * rate)
        hop = (idx // HOP_EVERY) % 2
        d = D0 + (HOP_M if hop else 0.0) + noise[idx]
        ph = (0.5 + (0.05 if hop else -0.05), -0.25, 0.5)
        con = b.build(make_js(qdot=0.25, stamp=t),
                      make_obs([make_obstacle(d=d, ph=ph, frames_seen=60)],
                               stamp=t, t_cap=idx / rate), t)
        v.append(float(con.v_obs[0]) if con.v_obs.size else 0.0)
    return float(np.percentile(np.abs(np.diff(v)), 95))


def _onset_lag(median_s, *, v_true=0.30, rate=84.0):
    """Seconds from a genuine approach starting to v_obs reaching half of it.

    The window is PRELOADED with the obstacle standing still: an empty median
    window tracks a step instantly, so without the preload this measures
    nothing and a long window looks free.
    """
    b = make_builder(obstacle_velocity_median=5,
                     obstacle_velocity_dt_min_s=0.033,
                     obstacle_velocity_tau_s=0.075,
                     obstacle_velocity_rot_hold_s=0.10,
                     obstacle_velocity_normal_rot_max=0.15,
                     obstacle_velocity_median_s=median_s)
    noise = _noise(11, 20000)
    pre = 1.0
    for k in range(int((pre + 1.5) * 50.0)):
        t = k / 50.0
        idx = int(t * rate)
        t_cap = idx / rate
        d = (0.60 + noise[idx] if t_cap < pre
             else 0.60 - v_true * (t_cap - pre) + noise[idx])
        con = b.build(make_js(stamp=t),
                      make_obs([make_obstacle(d=d, frames_seen=60)],
                               stamp=t, t_cap=t_cap), t)
        v = float(con.v_obs[0]) if con.v_obs.size else 0.0
        if t_cap >= pre and v >= 0.5 * v_true:
            return t_cap - pre
    return None


def test_widening_the_median_reduces_the_artefact_step():
    """The reason the window was widened: 0.30 s must beat 0.167 s here."""
    assert _artefact_steps(0.30) < 0.85 * _artefact_steps(0.167)


def test_the_median_still_detects_a_genuine_approach_in_time():
    """And the price it was widened at: the onset lag must stay under 0.3 s.

    0.3 s is the barrier's own time constant; an estimator slower than that
    would be reacting after the row it feeds has already moved. At 0.3 m/s it
    is also ~9 cm, against a `danger` zone of 0.2 m.
    """
    lag = _onset_lag(0.30)
    assert lag is not None, 'a genuine approach must be detected at all'
    assert lag < 0.30, f'onset lag {lag * 1e3:.0f} ms is too slow'


def test_a_longer_window_would_cost_more_than_it_buys():
    """0.50 s is the option NOT taken, and this is why: the lag crosses 0.3 s."""
    assert _onset_lag(0.50) > _onset_lag(0.30)
    assert _onset_lag(0.50) >= 0.30


def test_the_shipped_median_window_matches_the_measurement():
    import os
    import yaml
    here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    with open(os.path.join(here, 'config', 'fr3_control.yaml')) as fh:
        p = yaml.safe_load(fh)['params']
    assert p['obstacle_velocity_median_s'] == pytest.approx(0.30)
    # The hold stays short on purpose: a 0.30 s hold was measured at 798 ms of
    # lag on a diagonal approach. See its note in the config.
    assert p['obstacle_velocity_rot_hold_s'] == pytest.approx(0.10)
