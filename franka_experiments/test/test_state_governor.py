"""The state governor: task attenuation keyed on the arm's own state.

Three margins — FR3 velocity envelope, sigma_min, self-collision gap — folded
into one weight in [0, 1]. The properties that matter are not the exact ramp
shape but the four that make it safe to leave on: it engages BEFORE the hard
box clamps, it never touches the rows, it cannot latch on a state the arm
cannot leave, and it is inert when every margin is full.
"""

from __future__ import annotations

import numpy as np
import pytest

from franka_experiments.utils.cbf_hard_limits import (
    FR3_VEL_Q_REF_LOWER,
    FR3_VEL_Q_REF_UPPER,
    fr3_velocity_envelope,
    hard_accel_box,
)
from franka_experiments.utils.state_governor import StateGovernor

_N = 7


def _gov(**kw):
    base = dict(qdot_band=0.30, sigma_floor=0.05, sigma_band=0.04,
                sc_margin=0.02, sc_band=0.04, resume_s=0.5,
                envelope_margin=0.85)
    base.update(kw)
    return StateGovernor(**base)


def _q_mid():
    return 0.5 * (FR3_VEL_Q_REF_UPPER + FR3_VEL_Q_REF_LOWER)


# ── inert when nothing is tight ──────────────────────────────────────────────

def test_full_margins_leave_the_task_untouched():
    g = _gov()
    st = g.weight(q=_q_mid(), qdot=np.full(_N, 0.2), sigma=0.5, d_sc=0.5,
                  dt=0.01)
    assert st.w == 1.0
    assert st.binding == '-'


def test_missing_diagnostics_are_not_treated_as_danger():
    """sigma=None / d_sc=inf means "no measurement", not "zero margin"."""
    g = _gov()
    st = g.weight(q=_q_mid(), qdot=np.zeros(_N), sigma=None, d_sc=float('inf'),
                  dt=0.01)
    assert st.w == 1.0
    st = g.weight(q=_q_mid(), qdot=np.zeros(_N), sigma=float('nan'),
                  d_sc=None, dt=0.01)
    assert st.w == 1.0


# ── each term binds, and says so ─────────────────────────────────────────────

def test_velocity_term_binds_and_is_named():
    g = _gov()
    q = _q_mid()
    q[3] = -2.90                      # joint4, 0.15 rad from its envelope ref
    qdot = np.zeros(_N)
    up, lo = fr3_velocity_envelope(q, margin=0.85)
    qdot[3] = lo[3] + 0.05            # 0.05 rad/s of headroom left
    st = g.weight(q=q, qdot=qdot, sigma=0.5, d_sc=0.5, dt=0.01)
    assert st.binding == 'vel'
    assert 0.0 < st.w < 1.0
    assert abs(st.margins[0] - 0.05) < 1e-9


def test_singularity_term_binds():
    g = _gov()
    st = g.weight(q=_q_mid(), qdot=np.zeros(_N), sigma=0.06, d_sc=0.5, dt=0.01)
    assert st.binding == 'sing'
    assert 0.0 < st.w < 1.0
    st = g.weight(q=_q_mid(), qdot=np.zeros(_N), sigma=0.04, d_sc=0.5, dt=0.01)
    assert st.w == 0.0


def test_self_collision_term_binds():
    g = _gov()
    st = g.weight(q=_q_mid(), qdot=np.zeros(_N), sigma=0.5, d_sc=0.04, dt=0.01)
    assert st.binding == 'sc'
    assert 0.0 < st.w < 1.0
    st = g.weight(q=_q_mid(), qdot=np.zeros(_N), sigma=0.5, d_sc=0.01, dt=0.01)
    assert st.w == 0.0


def test_worst_term_wins():
    g = _gov()
    st = g.weight(q=_q_mid(), qdot=np.zeros(_N), sigma=0.07, d_sc=0.03,
                  dt=0.01)
    assert st.binding == 'sc'
    assert st.w == min(st.weights)


# ── the ordering the governor exists to create ───────────────────────────────

def test_governor_engages_before_the_hard_box_clamps():
    """A joint closing on its envelope must meet the fade before the clamp.

    This is the whole point of ``governor_envelope_margin`` sitting under
    ``velocity_box_margin``: if the box bit first there would be nothing left
    for the governor to prevent, and the QP would again be resolving a
    single-joint clamp by dumping the demand on the other six.
    """
    g = _gov(envelope_margin=0.85)
    decel = np.array([6.0, 2.585, 3.5, 4.0, 10.0, 5.5, 10.0])
    q_mech_min = np.array([-2.9007, -1.8361, -2.9007, -3.0770,
                           -2.8763, 0.4398, -3.0508])
    q_mech_max = np.array([2.9007, 1.8361, 2.9007, -0.1169,
                           2.8763, 4.6216, 3.0508])
    q = _q_mid()
    q[3] = -2.90
    up, lo = fr3_velocity_envelope(q, margin=0.85)

    engaged_at = clamped_at = None
    for frac in np.linspace(0.30, 1.05, 200):
        qdot = np.zeros(_N)
        qdot[3] = lo[3] * frac
        st = g.weight(q=q, qdot=qdot, sigma=None, d_sc=None, dt=0.0)
        g.reset(1.0)                       # each probe independent of the ramp
        lb, _ = hard_accel_box(
            q, qdot, acc_lb=-decel, acc_ub=decel,
            qdot_max=np.array([2.62, 2.62, 2.62, 2.62, 5.26, 4.18, 5.26]),
            v_margin=0.9,
            q_min=np.maximum(q_mech_min, FR3_VEL_Q_REF_LOWER),
            q_max=np.minimum(q_mech_max, FR3_VEL_Q_REF_UPPER),
            q_margin=0.05, brake_eta=0.6, dt=0.01)
        if engaged_at is None and st.target < 1.0:
            engaged_at = frac
        if clamped_at is None and lb[3] > -decel[3] + 1e-9:
            clamped_at = frac
    assert engaged_at is not None, 'the governor never engaged'
    assert clamped_at is not None, 'the box never clamped'
    assert engaged_at < clamped_at, (
        f'governor engaged at {engaged_at:.3f} of the envelope, box clamped at '
        f'{clamped_at:.3f} — the governor must come first')


# ── no latch ─────────────────────────────────────────────────────────────────

def test_a_stopped_joint_on_its_envelope_has_full_margin():
    """Direction matters: the bound ahead is the one that counts.

    A joint parked on (or past) its lower envelope with q̇ = 0 is in no danger,
    and suspending the task there would be a latch — nothing would move, so
    nothing would recover.
    """
    g = _gov()
    q = _q_mid()
    q[3] = FR3_VEL_Q_REF_LOWER[3] - 0.01       # past the reference, stopped
    st = g.weight(q=q, qdot=np.zeros(_N), sigma=None, d_sc=None, dt=0.01)
    assert st.w == 1.0


def test_velocity_term_releases_once_the_joint_reverses():
    g = _gov()
    q = _q_mid()
    q[3] = -3.00
    up, lo = fr3_velocity_envelope(q, margin=0.85)
    qdot = np.zeros(_N)
    qdot[3] = lo[3]                            # hard against the envelope
    assert g.weight(q=q, qdot=qdot, sigma=None, d_sc=None, dt=0.01).w == 0.0
    qdot[3] = +0.05                            # braking reversed it
    st = g.weight(q=q, qdot=qdot, sigma=None, d_sc=None, dt=0.01)
    assert st.target == 1.0, 'margin must be restored the moment q̇ reverses'


# ── the ramp ─────────────────────────────────────────────────────────────────

def test_fall_is_instant_and_climb_is_rate_limited():
    g = _gov(resume_s=0.5)
    st = g.weight(q=_q_mid(), qdot=np.zeros(_N), sigma=0.0, d_sc=None, dt=0.01)
    assert st.w == 0.0
    st = g.weight(q=_q_mid(), qdot=np.zeros(_N), sigma=1.0, d_sc=None, dt=0.01)
    assert st.target == 1.0
    assert abs(st.w - 0.02) < 1e-9             # one tick of 0.01 / 0.5 s
    for _ in range(60):
        st = g.weight(q=_q_mid(), qdot=np.zeros(_N), sigma=1.0, d_sc=None,
                      dt=0.01)
    assert st.w == 1.0


def test_zero_resume_disables_the_ramp():
    g = _gov(resume_s=0.0)
    g.weight(q=_q_mid(), qdot=np.zeros(_N), sigma=0.0, d_sc=None, dt=0.01)
    st = g.weight(q=_q_mid(), qdot=np.zeros(_N), sigma=1.0, d_sc=None, dt=0.01)
    assert st.w == 1.0


def test_bands_must_be_positive():
    with pytest.raises(ValueError):
        _gov(qdot_band=0.0)
    with pytest.raises(ValueError):
        _gov(sigma_band=-1.0)
