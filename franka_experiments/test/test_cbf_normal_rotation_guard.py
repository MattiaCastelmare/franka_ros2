"""The residual's rotation guard: reject the frame, never the approach.

The residual estimator computes v_obs = a^T qdot - ddot and assumes n_hat is
the SAME direction in both frames it differences. When closest_point_human hops
to a neighbouring surface patch the normal rotates, the two halves stop
referring to the same axis, and the difference is the projection error of a
rotation rather than an obstacle velocity. Clamped to the approaching half and
combined with max(), that error has a positive mean, so a STATIONARY obstacle
gets a fabricated closing speed — measured on hardware as a square wave
0.000 -> 0.085 -> 0.000 -> 0.035 m/s, worth ~0.9 rad/s^2 per edge through
k1 = 10.5 in a row that was binding on every tick.

What must be true of any fix:

1. off (rot_max = 0) it changes nothing at all;
2. with a STABLE normal it changes nothing, whatever the obstacle is doing —
   otherwise it is a deadband on v_obs by another name, and a deadband on
   v_obs breaks the B2 requirement to react to a 0.1 m/s approach;
3. on a rotated frame it HOLDS the previous estimate rather than decaying it,
   because the estimator has learned nothing and every quantity on this path
   is held to "may only tighten, never loosen";
4. it is counted, so the next hardware run can falsify the hypothesis behind
   it instead of the hypothesis quietly becoming a permanent parameter.
"""

import numpy as np
import pytest

from _cbf_builder_harness import make_builder, make_obstacle, make_js, make_obs

DT = 1.0 / 30.0
GUARD = 0.15                      # rad, the shipped launch value


def _speeds(builder, gaps, normals, *, adotq=0.0, t0=0.0):
    """Drive _obstacle_speed directly over a frame sequence and collect v_obs.

    Direct rather than through build(): the guard is a property of the
    estimator, and driving it here lets a test rotate n_hat by an exact angle
    instead of hoping a Jacobian happens to produce one.

    ``t0`` continues an earlier sequence on the same builder. Restarting the
    clock instead would make every frame land on ``dt <= 0``, which the
    estimator answers by holding BEFORE the guard is even consulted — a test
    written that way passes or fails for the wrong reason.
    """
    out = []
    for k, (d, n) in enumerate(zip(gaps, normals)):
        out.append(builder._obstacle_speed('cp', float(d), t0 + k * DT,
                                           float(adotq),
                                           None if n is None else np.asarray(n)))
    return np.asarray(out)


def _rot(theta):
    return np.array([np.cos(theta), np.sin(theta), 0.0])


# ── 1. inert when off ────────────────────────────────────────────────────────

def test_the_guard_is_inert_when_the_threshold_is_zero():
    gaps = [0.30, 0.29, 0.28, 0.27]
    spin = [_rot(0.0), _rot(0.9), _rot(0.0), _rot(0.9)]   # violent rotation
    off = _speeds(make_builder(obstacle_velocity_normal_rot_max=0.0), gaps, spin)
    none = _speeds(make_builder(obstacle_velocity_normal_rot_max=0.0), gaps,
                   [None] * 4)
    np.testing.assert_array_equal(off, none)
    assert make_builder(obstacle_velocity_normal_rot_max=0.0).diag_rot_reject == 0


# ── 2. a stable normal is never touched ──────────────────────────────────────

def test_a_stable_normal_is_left_completely_alone():
    """The guard must not become a deadband. Same gaps, same everything, the
    only difference is whether the guard is armed."""
    gaps = [0.30, 0.29, 0.28, 0.27, 0.26, 0.25]
    stable = [_rot(0.0)] * len(gaps)
    off = _speeds(make_builder(obstacle_velocity_normal_rot_max=0.0), gaps, stable)
    on = _speeds(make_builder(obstacle_velocity_normal_rot_max=GUARD), gaps, stable)
    np.testing.assert_array_equal(off, on)


def test_a_genuine_slow_approach_still_gets_through():
    """B2 covers 0.1 to 1.0 m/s and the slow end is where a deadband would
    silently eat the signal. 0.15 m/s of real closing on a stable normal must
    survive the guard untouched — this is the test that failed when a deadband
    was tried on the total v_obs, and it is kept pointed at the same place."""
    v_true = 0.15
    gaps = [0.30 - v_true * DT * k for k in range(8)]
    stable = [_rot(0.0)] * len(gaps)
    b = make_builder(obstacle_velocity_normal_rot_max=GUARD,
                     obstacle_velocity_alpha=0.0)     # no EMA, read it straight
    v = _speeds(b, gaps, stable)
    assert v[-1] == pytest.approx(v_true, abs=1e-9)
    assert b.diag_rot_reject == 0


def test_a_rotation_below_the_threshold_is_not_rejected():
    """The arm's own motion turns the normal a little every frame: 4 mm of
    tangential travel at a 0.2 m gap is 0.02 rad. The guard must sit well above
    that or it would fire continuously on honest motion."""
    gaps = [0.30, 0.29, 0.28, 0.27]
    creep = [_rot(0.02 * k) for k in range(4)]
    b = make_builder(obstacle_velocity_normal_rot_max=GUARD)
    _speeds(b, gaps, creep)
    assert b.diag_rot_reject == 0


# ── 3. a rotated frame holds, and is counted ─────────────────────────────────

def test_a_hopped_normal_holds_the_previous_estimate():
    """A patch 5 cm away at a 0.2 m gap turns the normal by 0.25 rad. On that
    frame the estimate must be the PREVIOUS one, exactly.

    A hop AWAY and the hop BACK are two rejections, not one, and that is the
    correct reading: returning to the old patch is as much a change of geometry
    as leaving it, and the frame that spans the return is just as unreadable.
    """
    v_true = 0.20
    gaps = [0.30 - v_true * DT * k for k in range(6)]
    normals = [_rot(0.0)] * 6
    normals[4] = _rot(0.30)                      # the hop
    b = make_builder(obstacle_velocity_normal_rot_max=GUARD)
    v = _speeds(b, gaps, normals)
    assert b.diag_rot_reject == 2                # out, and back
    assert v[4] == v[3]                          # held, bit for bit
    assert v[5] == v[3]


def test_a_hopped_normal_is_held_and_never_decayed():
    """Held, not faded toward zero. Decaying would LOOSEN the barrier on the
    strength of a frame the estimator has already declared unreadable."""
    v_true = 0.30
    b = make_builder(obstacle_velocity_normal_rot_max=GUARD,
                     obstacle_velocity_alpha=0.0)
    gaps = [0.40 - v_true * DT * k for k in range(4)]
    v_before = _speeds(b, gaps, [_rot(0.0)] * 4)[-1]
    v_after = _speeds(b, [gaps[-1] - v_true * DT], [_rot(0.8)])[0]
    assert v_after == v_before
    assert v_after > 0.0


def test_the_fabricated_speed_from_a_pure_rotation_is_suppressed():
    """The artefact itself, reproduced. The obstacle is STATIC (the gap is
    constant) and the arm is moving, so a^T qdot is non-zero while ddot is not:
    without the guard the estimator reports the arm's own speed as the
    obstacle's. With the guard the rotated frame is discarded.
    """
    adotq = 0.12                     # the arm moving, gap unchanged
    gaps = [0.20] * 4
    normals = [_rot(0.0), _rot(0.0), _rot(0.5), _rot(0.0)]
    off = make_builder(obstacle_velocity_normal_rot_max=0.0,
                       obstacle_velocity_alpha=0.0)
    on = make_builder(obstacle_velocity_normal_rot_max=GUARD,
                      obstacle_velocity_alpha=0.0)
    v_off = _speeds(off, gaps, normals, adotq=adotq)
    v_on = _speeds(on, gaps, normals, adotq=adotq)
    # Unguarded, the estimator reports the ARM's speed as the obstacle's, on a
    # gap that never changed. That is the artefact, in one line.
    assert v_off[2] == pytest.approx(adotq)
    assert on.diag_rot_reject == 2               # the hop and the hop back
    assert v_on[2] == v_on[1]
    assert v_on[3] == v_on[1]


# ── 4. counted, so the hypothesis is falsifiable ─────────────────────────────

def test_the_rejection_count_is_cumulative_and_survives_rebuilds():
    """A per-rebuild counter would only ever be read as 0 or 1. The question
    this number answers is "does this guard ever fire on the real robot", and
    that question needs a total."""
    b = make_builder(obstacle_velocity_normal_rot_max=GUARD)
    _speeds(b, [0.3, 0.3, 0.3], [_rot(0.0), _rot(0.9), _rot(0.0)])
    first = b.diag_rot_reject
    assert first > 0
    _speeds(b, [0.3, 0.3, 0.3], [_rot(0.0), _rot(0.9), _rot(0.0)], t0=10.0)
    assert b.diag_rot_reject > first


def test_the_guard_runs_through_the_shipped_builder():
    """Not just the private helper: the real build() path must pass n_hat, or
    the guard would be dead code that only the unit test exercises."""
    b = make_builder(obstacle_velocity_normal_rot_max=GUARD,
                     obstacle_velocity_source='residual')
    for k in range(4):
        t = k * DT
        # Swing the obstacle around the control point so n_hat rotates hard
        # while the GAP stays constant — a static obstacle, a moving normal.
        ang = 0.0 if k % 2 == 0 else 1.0
        ph = (0.5 + 0.25 * np.sin(ang), -0.25 * np.cos(ang), 0.5)
        b.build(make_js(q=0.1, qdot=0.05, stamp=t),
                make_obs([make_obstacle(d=0.25, ph=ph)], stamp=t), t)
    assert b.diag_rot_reject > 0


def test_the_guard_cannot_freeze_an_estimate_forever():
    """A nearest point alternating between two patches every frame has NO pair
    of consecutive frames sharing a normal, so an unbounded guard would reject
    every frame and hold v_obs at whatever it was — indefinitely.

    Held is the right answer for one bad frame and the wrong answer for a
    permanent one: a stale-low estimate under-tightens the barrier exactly when
    the geometry is too confusing to read. _ROT_HOLD_MAX bounds it.
    """
    from franka_experiments.utils.cbf_state_rows import _ROT_HOLD_MAX
    n = 12
    gaps = [0.30 - 0.25 * DT * k for k in range(n)]
    flip = [_rot(0.0 if k % 2 == 0 else 0.9) for k in range(n)]
    b = make_builder(obstacle_velocity_normal_rot_max=GUARD,
                     obstacle_velocity_alpha=0.0)
    v = _speeds(b, gaps, flip)
    # It did keep rejecting...
    assert b.diag_rot_reject >= _ROT_HOLD_MAX
    # ...but it did NOT reject every frame, so the estimate is refreshed.
    assert b.diag_rot_reject < n - 1
    assert len(set(np.round(v[1:], 9))) > 1, 'v_obs never moved: guard froze it'
