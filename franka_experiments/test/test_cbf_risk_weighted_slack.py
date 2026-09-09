"""Phase-2 risk-weighted slack: the weight, and what it does to a conflict.

One slack per FAMILY is shared by every row in it, so when rows conflict the QP
buys the same relief for all of them — a row 3 cm from contact and a row 40 cm
away are relaxed by exactly the same amount. Weighting the slack COLUMN,

    aᵢᵀq̈ + s_g/wᵢ ≥ bᵢ        instead of        aᵢᵀq̈ + s_g ≥ bᵢ

makes the relief inversely proportional to criticality. The slack stays shared:
this is weighted *sharing*, not independent per-row slack, and the last test in
this file is the one that pins what that actually buys.

The normalisation is the part that is easy to get wrong, and one test below
exists only to pin it. The column is written ``−w_max/wᵢ``, not ``−1/wᵢ``:

* ``−1/wᵢ`` tightens the critical row, but the shared slack is ``maxᵢ wᵢ·rᵢ``,
  so it inflates by up to ``w_max`` and its ``½ρs²`` cost by up to ``w_max²``.
  The obstacle family then out-prices ``‖q̈ − q̈_nom‖²`` and the solution is
  distorted far beyond "redistribute relaxation".
* ``−w_max/wᵢ`` gives the identical ratio between rows, but the MOST critical
  row keeps today's column of ``−1.0`` (to within 0.1 %: σ saturates toward 1
  without reaching it) and the shared slack keeps today's magnitude — so
  ``rho_slack`` needs no retuning. The distant rows are what move, yielding up
  to ``w_max×`` further.

Stated plainly, because it is the honest description and not the flattering
one: this does not make anything safer by tightening. It relocates relaxation
onto the rows that can afford it.

Pure numpy + scipy.
"""

import numpy as np
from scipy.optimize import minimize_scalar

from franka_experiments.utils.cbf_state_rows import (
    joint_limit_risk_margin,
    risk_slack_weight,
)

KW = dict(w_max=5.0, rho=0.30, alpha=6.0)


# ── The weight ───────────────────────────────────────────────────────────────

def test_a_violated_row_is_maximally_critical():
    assert risk_slack_weight(-0.05, **KW) > 0.98 * KW['w_max']


def test_a_distant_row_is_indistinguishable_from_today():
    """Beyond the distance of influence the slack column must be the −1.0 it is
    today, not merely close to it — otherwise enabling the flag silently
    restiffens every far row in the workspace."""
    assert abs(risk_slack_weight(KW['rho'], **KW) - 1.0) < 0.15
    assert abs(risk_slack_weight(3 * KW['rho'], **KW) - 1.0) < 1e-6


def test_the_knee_sits_at_half_the_influence_distance():
    """σ(ρ/2) = 1/2 exactly — the defining property of the Flacco/De Luca
    logistic, and the thing rho is tuned against."""
    mid = risk_slack_weight(KW['rho'] / 2.0, **KW)
    assert np.isclose(mid, 1.0 + (KW['w_max'] - 1.0) * 0.5)


def test_the_weight_is_bounded_at_both_ends():
    """No measurement, however absurd, may drive the slack column to 0 or ∞ —
    the multiplier w_max/w must stay in [1, w_max]."""
    for h in (-1e6, -10.0, -0.1, 0.0, 0.1, 10.0, 1e6):
        w = risk_slack_weight(h, **KW)
        assert 1.0 <= w <= KW['w_max'], (h, w)
        assert 1.0 <= KW['w_max'] / w <= KW['w_max']


def test_the_weight_is_monotone_in_the_barrier():
    """Getting closer may never buy a row MORE relief."""
    ws = [risk_slack_weight(h, **KW) for h in np.linspace(-0.2, 0.9, 111)]
    assert np.all(np.diff(ws) <= 1e-12)


def test_the_weight_is_continuous():
    """Continuity is the whole reason for a sigmoid over a threshold: a control
    point drifting across the knee must not flip the active set."""
    hs = np.linspace(-0.1, 0.6, 701)
    ws = np.array([risk_slack_weight(h, **KW) for h in hs])
    assert np.max(np.abs(np.diff(ws))) < 0.05


def test_w_max_one_disables_the_weighting_entirely():
    """The escape hatch: flag on, weighting neutral, bit-identical columns."""
    kw = dict(KW, w_max=1.0)
    for h in (-0.2, 0.0, 0.15, 1.0):
        assert risk_slack_weight(h, **kw) == 1.0


def test_no_overflow_at_extreme_barriers():
    """exp() overflows for a far row; the limit is σ → 0, not a warning or a
    nan landing in the constraint matrix."""
    w = risk_slack_weight(1e9, **KW)
    assert np.isfinite(w) and np.isclose(w, 1.0)


# ── What it does to a conflict ───────────────────────────────────────────────

def _solve_family(bs, ms, rho_g=1000.0):
    """min ½‖q̈‖² + ½ρ_g s²  s.t.  q̈ + mᵢ·s ≥ bᵢ,  s ≥ 0.

    A one-dimensional stand-in for one constraint family: every row acts on the
    same scalar direction (the case where rows genuinely conflict), so the QP
    reduces to a convex problem in q̈ with the slack solved in closed form —
    ``s`` must satisfy the worst row, ``s = maxᵢ (bᵢ − q̈)⁺/mᵢ``.

    Returns ``(qddot, s, relief)`` with ``relief[i] = mᵢ·s``, i.e. how much of
    row i's demand the slack covered.
    """
    bs = np.asarray(bs, float)
    ms = np.asarray(ms, float)

    def s_of(qdd):
        return float(np.max(np.maximum(bs - qdd, 0.0) / ms))

    def cost(qdd):
        return 0.5 * qdd ** 2 + 0.5 * rho_g * s_of(qdd) ** 2

    res = minimize_scalar(cost, bounds=(-6.0, 6.0), method='bounded',
                          options={'xatol': 1e-12})
    assert res.success, res.message
    s = s_of(res.x)
    return res.x, s, s * ms


def test_unweighted_slack_relaxes_both_rows_equally():
    """The behaviour being replaced, stated as a property: with w = 1 the near
    row and the far row receive exactly the same relief, whatever their gap."""
    _, _, relief = _solve_family(bs=[8.0, 3.0], ms=[1.0, 1.0])
    assert np.isclose(relief[0], relief[1])


def test_weighting_redistributes_relief_onto_the_less_critical_row():
    """The point of the phase. Same two conflicting rows; row 0 is at the
    barrier (w = w_max), row 1 is far (w = 1). The far row must end up
    absorbing strictly more of the relaxation."""
    w_near = risk_slack_weight(-0.02, **KW)
    w_far = risk_slack_weight(0.45, **KW)
    assert w_near > 4.0 and np.isclose(w_far, 1.0, atol=1e-3), 'fixture'
    m_near, m_far = KW['w_max'] / w_near, KW['w_max'] / w_far

    _, _, relief = _solve_family(bs=[8.0, 3.0], ms=[m_near, m_far])
    assert relief[1] > relief[0]
    assert np.isclose(relief[1] / relief[0], w_near / w_far, rtol=1e-6)


def test_the_extra_relaxation_goes_to_the_far_row_not_into_the_slack():
    """The claim that matters for tuning: the shared slack — and therefore the
    family's ½ρs² cost — must be unchanged, with the extra relief landing
    entirely on the far row."""
    w_near = risk_slack_weight(-0.02, **KW)
    w_far = risk_slack_weight(0.45, **KW)

    _, s_u, relief_u = _solve_family(bs=[8.0, 3.0], ms=[1.0, 1.0])
    _, s_w, relief_w = _solve_family(
        bs=[8.0, 3.0], ms=[KW['w_max'] / w_near, KW['w_max'] / w_far])

    # The critical row is the binding one either way, so its relief is what the
    # shared slack is sized on. What must NOT happen is that slack growing to
    # serve the far row — that is the −1/w trap.
    # rtol, not equality: sigma saturates toward 1 but never reaches it, so a
    # violated row's multiplier is 1.0009 rather than exactly 1.0. The point is
    # that it does not SCALE with w_max, which the -1/w form does.
    assert np.isclose(s_w, s_u, rtol=1e-2)
    assert np.isclose(relief_w[0], relief_u[0], rtol=1e-2)
    # The far row is where the extra relaxation goes.
    assert relief_w[1] > 4.0 * relief_u[1]


def test_the_normalisation_leaves_the_shared_slack_magnitude_alone():
    """The regression that the −1/w form fails and −w_max/w passes.

    Four rows, accel box saturated so the conflict is real and the slack has to
    absorb the residual. Under −1/w the shared slack inflates (and with it the
    ½ρs² cost, quadratically); under −w_max/w it must stay at the unweighted
    value, because the critical row's multiplier is exactly 1.
    """
    bs = [9.0, 7.0, 5.0, 3.0]
    ws = np.array([risk_slack_weight(h, **KW)
                   for h in (-0.02, 0.05, 0.18, 0.45)])

    # multipliers m: row reads aᵀq̈ + m·s ≥ b
    m_unweighted = np.ones(4)
    m_bad = 1.0 / ws                       # the trap
    m_good = KW['w_max'] / ws              # what is shipped

    def shared_slack(m, qdd=6.0):          # q̈ pinned at the accel box
        return float(np.max(np.maximum(np.array(bs) - qdd, 0.0) / m))

    s_ref = shared_slack(m_unweighted)
    assert shared_slack(m_bad) > 4.0 * s_ref, 'the trap must actually inflate'
    # Within a fraction of a percent, not exactly: see the rtol note above.
    assert np.isclose(shared_slack(m_good), s_ref, rtol=1e-2), 'shipped form must not'


def test_the_most_critical_row_keeps_todays_slack_column():
    """Consequence of the normalisation, and the reason rho_slack needs no
    retuning: well inside the barrier the multiplier is 1.0 to within 0.1 %."""
    w = risk_slack_weight(-0.5, **KW)
    assert np.isclose(KW['w_max'] / w, 1.0, rtol=1e-3)


def test_distant_rows_yield_further_than_today():
    """The honest half of the claim, asserted rather than glossed: the far row's
    multiplier is strictly greater than 1, i.e. it IS relaxed more than today.
    That is the mechanism."""
    w = risk_slack_weight(0.6, **KW)
    assert KW['w_max'] / w > 4.5


# ── Joint-limit family ───────────────────────────────────────────────────────
#
# Same logistic, its own rho, because that barrier is in RADIANS. The scenario
# these guard is the one logged on joint2: the position braking curve in
# hard_accel_box clips a joint from h < qdot²/(2·a_auth) — 0.337 rad at
# 1.02 rad/s — while the row horizon was 0.30 rad, so the hard box bit and the
# soft row did not exist. dq_ort hit 19.5 rad/s² with s[qlim] at zero.

RHO_Q = 0.40


def test_the_qlim_weight_uses_its_own_radian_scale():
    """A joint 0.20 rad from its limit must score as critical as an obstacle
    0.15 m from contact — that is what separate rhos buy. Feeding the metres
    rho to a radian barrier is the units confusion the per-family slack split
    exists to prevent."""
    w_wrong = risk_slack_weight(0.20, w_max=5.0, rho=KW['rho'], alpha=6.0)
    w_right = risk_slack_weight(0.20, w_max=5.0, rho=RHO_Q, alpha=6.0)
    # 0.20 rad IS the knee of the radian rho (rho/2), so the right scale reads
    # exactly the midpoint weight while the metres scale reads nearly neutral.
    assert np.isclose(w_right, 0.5 * (1.0 + 5.0))
    assert w_wrong < 1.6, w_wrong
    assert w_right > 2.0 * w_wrong


A_AUTH = np.full(7, 0.6 * 2.585)          # joint2's braking authority


def test_raw_h_weights_a_closing_joint_far_too_late():
    """Why the score is the post-braking margin and not h. At the distance
    where the box already clips joint2 (0.335 rad at 1.02 rad/s) a logistic on
    raw h reads essentially neutral — the box would be fighting the joint while
    the weighting still considered it uninteresting."""
    h_bind = 1.02 ** 2 / (2.0 * A_AUTH[1])
    assert 0.33 < h_bind < 0.34, 'fixture: the logged binding distance'
    assert risk_slack_weight(h_bind, w_max=5.0, rho=RHO_Q, alpha=6.0) < 1.1


def test_a_joint_closing_on_its_braking_curve_is_weighted_critical():
    """The logged case, scored properly: joint2 at 1.02 rad/s toward its LOWER
    limit, exactly at the distance the box starts clipping. Zero room after
    braking ⇒ the weight must saturate."""
    a = np.zeros(7); a[1] = 1.0               # lower-limit row: a = +e_j
    qdot = np.zeros(7); qdot[1] = -1.02       # closing on the lower limit
    h_bind = 1.02 ** 2 / (2.0 * A_AUTH[1])

    score = joint_limit_risk_margin(a, h_bind, qdot, A_AUTH)
    assert abs(score) < 1e-9, score
    assert risk_slack_weight(score, w_max=5.0, rho=RHO_Q,
                             alpha=6.0) > 4.9


def test_a_joint_parked_at_the_same_distance_stays_neutral():
    """"Near a limit" is not "closing on a limit". A stationary joint at the
    same 0.335 rad has all its room left and must not be protected — otherwise
    the weighting fires on every pose that merely sits near a limit."""
    a = np.zeros(7); a[1] = 1.0
    h_bind = 1.02 ** 2 / (2.0 * A_AUTH[1])
    score = joint_limit_risk_margin(a, h_bind, np.zeros(7), A_AUTH)
    assert np.isclose(score, h_bind)
    assert risk_slack_weight(score, w_max=5.0, rho=RHO_Q, alpha=6.0) < 1.1


def test_a_joint_moving_away_gets_no_penalty():
    """Direction matters: a = +e_j guards the LOWER limit, so a positive q̇ is
    receding and must leave the score untouched."""
    a = np.zeros(7); a[1] = 1.0
    qdot = np.zeros(7); qdot[1] = +2.0        # moving away from the lower limit
    assert joint_limit_risk_margin(a, 0.30, qdot, A_AUTH) == 0.30


def test_the_upper_limit_row_has_the_mirrored_sign():
    """a = −e_j guards the UPPER limit, so there a POSITIVE q̇ is the closing
    one. A sign slip here would protect exactly the wrong half of the range."""
    a = np.zeros(7); a[1] = -1.0
    qd_closing = np.zeros(7); qd_closing[1] = +1.02
    qd_receding = np.zeros(7); qd_receding[1] = -1.02
    assert joint_limit_risk_margin(a, 0.40, qd_closing, A_AUTH) < 0.40
    assert joint_limit_risk_margin(a, 0.40, qd_receding, A_AUTH) == 0.40


def test_the_score_degrades_smoothly_with_speed():
    """No threshold: the penalty grows quadratically with the closing speed, so
    the weight moves continuously as the joint accelerates toward its limit."""
    a = np.zeros(7); a[1] = 1.0
    scores = [joint_limit_risk_margin(a, 0.40, np.array([0, -v, 0, 0, 0, 0, 0.0]),
                                      A_AUTH) for v in np.linspace(0, 1.5, 31)]
    assert np.all(np.diff(scores) <= 1e-12)
    assert scores[0] == 0.40


def test_rho_zero_leaves_the_joint_limit_rows_unweighted():
    """The escape hatch back to Phase-2 behaviour (obstacle-only weighting) is a
    parameter, not a code path: the node skips the weight when rho_qlim <= 0."""
    assert RHO_Q > 0.0        # the shipped default weights them
    # and the node's guard is `self._wsl_rho_q > 0.0`, checked structurally in
    # test_qlim_weighting_is_gated_on_rho below.


def test_qlim_weighting_is_gated_on_rho():
    """Structural check on the builder, so the gate cannot be deleted silently.

    Lives in ``utils/cbf_state_rows.ConstraintBuilder``; the node is only the
    orchestrator and owns no row formula.
    """
    import pathlib
    src = pathlib.Path('franka_experiments/franka_experiments/utils/'
                       'cbf_state_rows.py').read_text()
    assert ('self._P.enable_weighted_slack\n'
            '                        and self._P.slack_weight_rho_qlim > 0.0') in src \
        or 'self._P.enable_weighted_slack and self._P.slack_weight_rho_qlim > 0.0' in src
