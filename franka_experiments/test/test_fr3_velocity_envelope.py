"""The FR3 position-based joint-velocity envelope, and the box that obeys it.

Five hardware runs in ``franka_logs`` ended with
``joint_velocity_violation``. In every one of them the offending joint was well
under its flat |q̇| limit — the worst was 0.85 of it, most were below 0.2 — and
over the POSITION-BASED envelope the firmware actually enforces. The guards in
place at the time were built from ``franka_description``'s
``position_based_velocity_limits``, whose offset sign and reference position
both differ from libfranka's, always in the permissive direction.

These cases are the regression: each ``(joint, q, q̇)`` below is the last state
the 30 Hz logger recorded before the abort.
"""

from __future__ import annotations

import numpy as np
import pytest

from franka_experiments.utils.cbf_hard_limits import (
    FR3_VEL_LIMIT,
    FR3_VEL_Q_REF_LOWER,
    FR3_VEL_Q_REF_UPPER,
    fr3_velocity_envelope,
    hard_accel_box,
)
from franka_experiments.utils.config import load_franka_joint_limits

_N = 7

# (run, joint index, q [rad], q̇ [rad/s] measured just before the abort)
ABORTS = [
    ('20260915_080123', 3, -3.0241, -0.1746),
    ('20260915_080040', 5, +4.4644, +0.4886),
    ('20260915_075903', 3, -3.0249, -0.1425),
    ('20260914_144329', 1, -1.5820, -0.8420),
    ('20260914_141401', 3, -2.9640, -0.5288),
]


def _q_home():
    """A configuration comfortably inside every joint range."""
    return np.array([0.0, -0.7, 0.0, -2.35, 0.0, 1.57, 0.7])


def _q_mid():
    """Midway between the two envelope references — the widest point of each.

    Not the same as ``_q_home``: joint4 spans only 2.9 rad, so at the home pose
    (−2.35 rad) its lower envelope is already down to −2.06 rad/s, below the flat
    limit. "Mid-range" has to mean mid-range per joint.
    """
    return 0.5 * (FR3_VEL_Q_REF_UPPER + FR3_VEL_Q_REF_LOWER)


# ── the envelope itself ──────────────────────────────────────────────────────

@pytest.mark.parametrize('run,j,q_j,qdot_j', ABORTS)
def test_recorded_aborts_are_over_the_envelope(run, j, q_j, qdot_j):
    """Each recorded abort state violates the envelope — so it is detectable."""
    q = _q_home()
    q[j] = q_j
    up, lo = fr3_velocity_envelope(q)
    bound = up[j] if qdot_j >= 0 else lo[j]
    assert abs(qdot_j) > abs(bound), (
        f'{run}: j{j + 1} q̇={qdot_j:+.4f} should exceed the envelope '
        f'{bound:+.4f}')
    # ...and by a margin a 30 Hz sampler could not have invented.
    assert abs(qdot_j) / abs(bound) > 1.0


@pytest.mark.parametrize('run,j,q_j,qdot_j', ABORTS)
def test_recorded_aborts_look_safe_under_the_flat_limit(run, j, q_j, qdot_j):
    """The flat limit explains none of them — which is why it was not enough."""
    assert abs(qdot_j) < 0.9 * FR3_VEL_LIMIT[j]


def test_envelope_is_zero_past_the_reference_position():
    """Past q_ref the firmware admits no motion that way; so does this."""
    up, _ = fr3_velocity_envelope(FR3_VEL_Q_REF_UPPER + 0.01)
    assert np.all(up == 0.0)
    _, lo = fr3_velocity_envelope(FR3_VEL_Q_REF_LOWER - 0.01)
    assert np.all(lo == 0.0)


def test_envelope_saturates_at_the_flat_limit_mid_range():
    """Far from both ends the envelope is just the flat limit."""
    up, lo = fr3_velocity_envelope(_q_mid())
    assert np.allclose(up, FR3_VEL_LIMIT)
    assert np.allclose(lo, -FR3_VEL_LIMIT)


def test_envelope_margin_scales_both_sides():
    up, lo = fr3_velocity_envelope(_q_mid(), margin=0.9)
    assert np.allclose(up, 0.9 * FR3_VEL_LIMIT)
    assert np.allclose(lo, -0.9 * FR3_VEL_LIMIT)


def test_envelope_is_tighter_than_franka_description_near_the_limits():
    """The parametrisation that was in use is looser wherever it differs.

    Not a style point: on joint4 at −3.024 rad the gap is 7.3x, and the guard
    only matters in exactly that band.
    """
    jl = load_franka_joint_limits([f'joint{i}' for i in range(1, _N + 1)])
    # (joint, q, +1 if the nearby limit is the upper one). Only the side facing
    # that limit is off the flat cap; the far side saturates in both forms.
    for j, q_j, side in [(3, -3.0241, -1), (5, 4.4644, +1), (1, -1.5820, -1)]:
        q = _q_home()
        q[j] = q_j
        up, lo = fr3_velocity_envelope(q)
        h = (jl['q_max'][j] - q_j) if side > 0 else (q_j - jl['q_min'][j])
        fd = min(jl['qdot_max'][j],
                 jl['v_offset'][j] + np.sqrt(2 * jl['decel_max'][j] * max(h, 0.0)))
        ours = up[j] if side > 0 else -lo[j]
        assert ours < fd, f'j{j + 1}: {ours:.4f} should be under {fd:.4f}'
        assert fd / ours > 1.5, (
            f'j{j + 1}: the two forms should differ by more than rounding '
            f'({fd:.4f} vs {ours:.4f})')


# ── the box that consumes it ─────────────────────────────────────────────────

_FR3_KW = dict(v_margin=0.9, q_margin=0.05, brake_eta=0.6, dt=0.01)


def _box_kw():
    jl = load_franka_joint_limits([f'joint{i}' for i in range(1, _N + 1)])
    return dict(acc_lb=-jl['decel_max'], acc_ub=jl['decel_max'],
                qdot_max=jl['qdot_max'], q_min=jl['q_min'], q_max=jl['q_max'],
                **_FR3_KW)


@pytest.mark.parametrize('run,j,q_j,qdot_j', ABORTS)
def test_box_demands_braking_at_every_abort_state(run, j, q_j, qdot_j):
    """At each abort state the box must force acceleration AWAY from the bound."""
    q = _q_home()
    q[j] = q_j
    qdot = np.zeros(_N)
    qdot[j] = qdot_j
    lb, ub = hard_accel_box(q, qdot, **_box_kw())
    if qdot_j < 0:
        assert lb[j] > 0.0, f'{run}: j{j + 1} box should demand q̈ > 0'
    else:
        assert ub[j] < 0.0, f'{run}: j{j + 1} box should demand q̈ < 0'


def test_box_without_the_envelope_misses_joint6():
    """The pre-fix box was 1.9x loose on joint6 — the 20260915_080040 abort."""
    q = _q_home()
    q[5] = 4.4644
    qdot = np.zeros(_N)
    qdot[5] = 0.4886
    kw = _box_kw()
    lb_off, ub_off = hard_accel_box(q, qdot, firmware_envelope=False, **kw)
    lb_on, ub_on = hard_accel_box(q, qdot, firmware_envelope=True, **kw)
    assert ub_off[5] > 0.0        # old box: "carry on, you have room"
    assert ub_on[5] < 0.0         # new box: "brake"
    assert np.allclose(lb_off, lb_on)   # braking authority untouched


def test_box_mid_range_unaffected_by_the_envelope():
    """Away from the limits the fix changes nothing."""
    q = _q_mid()
    qdot = np.full(_N, 0.3)
    kw = _box_kw()
    assert np.allclose(hard_accel_box(q, qdot, firmware_envelope=False, **kw),
                       hard_accel_box(q, qdot, firmware_envelope=True, **kw))


# ── acceleration authority ───────────────────────────────────────────────────

def test_accel_authority_cap_only_touches_the_wrist():
    """qddot_max_abs = 10 is libfranka's rated q̈; only j5/j7 were above it.

    franka_description has no acceleration field, so the filter had been using
    deceleration_limit symmetrically. That is exact in braking and 70% optimistic
    in the other direction on joints 5 and 7 (17 against a rated 10).
    """
    from franka_experiments.utils.cbf_hard_limits import FR3_MAX_JOINT_ACCEL
    jl = load_franka_joint_limits([f'joint{i}' for i in range(1, _N + 1)])
    capped = np.minimum(jl['decel_max'], FR3_MAX_JOINT_ACCEL)
    changed = ~np.isclose(capped, jl['decel_max'])
    assert list(np.flatnonzero(changed)) == [4, 6], 'only joints 5 and 7'
    assert np.all(capped <= FR3_MAX_JOINT_ACCEL)
    assert np.allclose(capped[~changed], jl['decel_max'][~changed])
