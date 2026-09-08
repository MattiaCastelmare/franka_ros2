"""Phase-1 obstacle-velocity feedforward: the two terms, and the flag-off path.

The HOCBF row already carries the obstacle's closing speed through the ``k1``
anticipation term. These two terms are a SECOND, separately tunable use of the
same estimate:

* ``h_eff = h − min(v_app²/(2·a_obs), brake_max)`` — the barrier is measured to
  where the obstacle could still be after decelerating, not to where it is;
* ``h_qp += −k_ff·v_app`` — a direct RHS feedforward whose gain is decoupled
  from ``k1`` (which also sets the barrier's damping and cannot be raised for
  reactivity without the row oscillating).

The requirement that dominates this file: **with the flag off the QP rows must
be bit-identical to the pre-Phase-1 build.** Both terms are exactly zero at
``v_app = 0``, so that reduces to arithmetic identity, which is asserted here
rather than assumed.

Pure numpy.
"""

import numpy as np

from franka_experiments.utils.cbf_state_rows import velocity_feedforward_terms

KW = dict(decel=4.0, gain=1.0, brake_max=0.25)


# ── The flag-off / no-approach path ──────────────────────────────────────────

def test_a_stationary_obstacle_contributes_exactly_nothing():
    """The whole regression requirement rests on this: both terms are exactly
    0.0 at v_app = 0, so `h - 0.0` and `h_qp + 0.0` are bit-identical to not
    evaluating them at all."""
    h_brake, b_ff = velocity_feedforward_terms(0.0, **KW)
    assert h_brake == 0.0
    assert b_ff == 0.0


def test_a_receding_obstacle_contributes_exactly_nothing():
    """v_app is clamped to the approaching half at the call site, but this must
    hold on a raw estimate too: a receding obstacle may never LOOSEN the row by
    producing a negative braking distance or a positive RHS term."""
    for v in (-0.01, -0.5, -5.0):
        h_brake, b_ff = velocity_feedforward_terms(v, **KW)
        assert h_brake == 0.0
        assert b_ff == 0.0


def test_adding_the_terms_at_zero_is_a_bitwise_no_op():
    """Directly what the node does with the flag off vs on-but-idle."""
    rng = np.random.default_rng(0)
    h_bar = rng.normal(size=16)
    h_qp = rng.normal(size=16) * 10.0
    h_brake, b_ff = velocity_feedforward_terms(0.0, **KW)
    assert np.array_equal(h_bar - h_brake, h_bar)
    assert np.array_equal(h_qp + b_ff, h_qp)


# ── Term (a): braking distance ───────────────────────────────────────────────

def test_braking_distance_is_the_kinematic_stopping_distance():
    assert np.isclose(velocity_feedforward_terms(0.8, **KW)[0], 0.8**2 / 8.0)


def test_braking_distance_is_quadratic_in_the_approach_speed():
    """Doubling the closing speed must quadruple the tightening — that is the
    whole point of the term versus the linear k1 path."""
    lo = velocity_feedforward_terms(0.3, **KW)[0]
    hi = velocity_feedforward_terms(0.6, **KW)[0]
    assert np.isclose(hi / lo, 4.0)


def test_a_lower_assumed_deceleration_is_more_conservative():
    slow_stop = velocity_feedforward_terms(0.8, decel=2.0, gain=1.0, brake_max=1.0)[0]
    fast_stop = velocity_feedforward_terms(0.8, decel=8.0, gain=1.0, brake_max=1.0)[0]
    assert slow_stop > fast_stop


def test_the_clamp_binds_before_the_term_exceeds_d_safe():
    """v_app is capped at obstacle_velocity_max = 2.0 m/s upstream. Unclamped
    the term would reach 2²/(2·4) = 0.5 m, over 3x d_safe = 0.15 — one depth
    artefact would drive the barrier deeply negative and the QP would answer
    with a maximal retreat. The clamp is what stops that."""
    unclamped = 2.0**2 / 8.0
    assert unclamped > 3 * 0.15, 'fixture assumption'
    assert velocity_feedforward_terms(2.0, **KW)[0] == KW['brake_max']


def test_braking_distance_is_monotone_and_bounded():
    caps = [velocity_feedforward_terms(v, **KW)[0] for v in np.linspace(0, 3, 61)]
    assert np.all(np.diff(caps) >= -1e-15)
    assert max(caps) <= KW['brake_max']


# ── Term (b): RHS feedforward, and its sign ──────────────────────────────────

def test_the_feedforward_tightens_the_row():
    """THE sign test. This repo assembles G x ≤ h_qp with G[:, :nv] = −A, i.e.
    aᵀq̈ + s ≥ −h_qp, so a LARGER h_qp is a LOOSER row. A closing obstacle must
    therefore push h_qp DOWN. Getting this backwards would make a fast approach
    relax the barrier — silently, and only under the flag."""
    _, b_ff = velocity_feedforward_terms(0.5, **KW)
    assert b_ff < 0.0
    h_qp_before = 3.0
    assert h_qp_before + b_ff < h_qp_before


def test_the_feedforward_is_linear_in_the_gain():
    _, b1 = velocity_feedforward_terms(0.5, decel=4.0, gain=1.0, brake_max=0.25)
    _, b2 = velocity_feedforward_terms(0.5, decel=4.0, gain=2.0, brake_max=0.25)
    assert np.isclose(b2, 2.0 * b1)


def test_the_gain_is_independent_of_the_braking_term():
    """The two terms must be separately tunable: changing k_ff must not move the
    barrier, and changing a_obs must not move the RHS."""
    hb1, ff1 = velocity_feedforward_terms(0.5, decel=4.0, gain=1.0, brake_max=0.25)
    hb2, ff2 = velocity_feedforward_terms(0.5, decel=4.0, gain=9.0, brake_max=0.25)
    hb3, ff3 = velocity_feedforward_terms(0.5, decel=9.0, gain=1.0, brake_max=0.25)
    assert hb1 == hb2 and ff1 != ff2
    assert ff1 == ff3 and hb1 != hb3


# ── What the flag is supposed to buy ─────────────────────────────────────────

def test_a_fast_approach_activates_the_row_earlier():
    """The success criterion, stated as an assertion on the row RHS.

    Same geometry, same robot motion, same gains; only the obstacle's closing
    speed differs. A row is DEMANDING retreat when h_qp < 0 (it then forces
    aᵀq̈ ≥ −h_qp > 0). With the feedforward on, the fast approach must cross
    that threshold at a LARGER surface gap than the slow one — i.e. earlier in
    time along an approach trajectory.
    """
    k0, k1, d_safe = 25.0, 10.5, 0.15
    a_dot_q, jdq = 0.0, 0.0             # robot still, no drift term

    def h_qp(d, v_app, flag_on):
        h = d - d_safe
        hb, ff = velocity_feedforward_terms(v_app, **KW) if flag_on else (0.0, 0.0)
        return k1 * (a_dot_q - v_app) + k0 * (h - hb) + jdq + ff

    def activation_gap(v_app, flag_on):
        """Largest surface gap at which the row already demands retreat."""
        gaps = np.linspace(0.60, 0.0, 601)
        active = [d for d in gaps if h_qp(d, v_app, flag_on) < 0.0]
        return max(active) if active else 0.0

    slow_off = activation_gap(0.10, False)
    fast_off = activation_gap(0.80, False)
    slow_on = activation_gap(0.10, True)
    fast_on = activation_gap(0.80, True)

    # The k1 path alone already reacts to speed — the feedforward must add to
    # it, not replace it.
    assert fast_off > slow_off
    assert fast_on > fast_off, 'the flag must buy earlier activation'
    # ...and it must buy MORE for a fast approach than for a slow one.
    assert (fast_on - fast_off) > (slow_on - slow_off)
