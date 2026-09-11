"""The retreat cap: an upper bound on how fast the filter may run away.

The HOCBF obstacle row is one-sided. It puts a FLOOR under the separation rate
(``−k1·ḣ − k0·h̄ − ċ``) and says nothing above it, so once the barrier is
violated the demanded floor grows with ``k0·|h̄|`` and the QP accelerates the arm
away for as long as that lasts. Observed on hardware as a retreat violent enough
to be a hazard of its own next to a person — and it leaves the arm with a large
velocity and a large tracking error once the obstacle is gone.

The cap rows bound the SAME direction ``n̂ᵀJ`` from above. Two things have to be
true for that to be safe rather than just slower:

1. the bound must COLLAPSE back to a walking pace the moment the obstacle is
   outside ``d_safe`` and no longer closing — otherwise it is just a second
   tuning knob for how fast the arm runs away;
2. it must LOSE to the barrier when the two genuinely conflict — a comfort
   bound must never be able to hold the arm inside a closing obstacle.

Both are tested here, the second against a real QP solve.

Pure numpy + scipy.
"""

import numpy as np
from scipy.optimize import minimize_scalar

from franka_experiments.utils.cbf_state_rows import (
    retreat_cap_rhs,
    retreat_cap_speed,
)

NV = 7
# Mirrors cbf_safety_filter's families; the cap is the last one.
G_OBS, G_CAP = 0, 1
N_SLACK = 2
NX = NV + N_SLACK

CAP_KW = dict(base=0.05, obs_gain=1.0, depth_gain=2.0,
              depth_speed_ref=0.30, engage_gap=0.15, max_speed=0.6)
# At or inside the barrier the relief term is zero and the cap is purely
# proportional. Most tests here are about that regime, so they pass h̄ <= 0.
AT_BARRIER = 0.0


# ── The cap value ────────────────────────────────────────────────────────────

def test_cap_is_the_base_speed_when_the_episode_is_over():
    """Obstacle outside d_safe (h̄ > 0) and not closing (v_obs = 0): nothing to
    escalate for, so the arm is back to a controlled creep. This is the whole
    "appena l'ostacolo è fuori dalla d_safe torna a comportamento safe"."""
    assert retreat_cap_speed(0.0, AT_BARRIER, **CAP_KW) == 0.05


def test_no_depth_escalation_outside_d_safe():
    """The depth term is max(−h̄, 0): outside d_safe it contributes nothing, so
    the only thing separating two points out there is the relief ramp."""
    kw = dict(CAP_KW, engage_gap=0.0)      # relief pinned, isolate the depth term
    assert (retreat_cap_speed(0.3, +0.01, **kw)
            == retreat_cap_speed(0.3, +5.00, **kw))


def test_far_from_the_barrier_the_cap_does_not_bind():
    """Ordinary task motion must not be throttled because a person happens to
    be standing somewhere in the 1.2 m obstacle horizon. Logged as
    retreat=+0.161/0.150 with d_min=0.389 m and no avoidance happening."""
    assert retreat_cap_speed(0.03, +0.24, **CAP_KW) == CAP_KW['max_speed']


def test_the_cap_tightens_monotonically_as_the_barrier_approaches():
    """The relief ramp must be monotone in h̄: no distance at which getting
    CLOSER buys the arm more retreat."""
    caps = [retreat_cap_speed(0.05, h, **CAP_KW)
            for h in np.linspace(0.30, 0.0, 31)]
    assert np.all(np.diff(caps) <= 1e-12)
    assert np.isclose(caps[-1], 0.05 + 0.05)   # at the barrier: purely proportional


def test_cap_tracks_the_obstacle_speed():
    """obs_gain = 1 ⇒ the arm may back off exactly as fast as the obstacle
    comes in: it holds the gap without outrunning anything."""
    assert np.isclose(retreat_cap_speed(0.30, AT_BARRIER, **CAP_KW), 0.05 + 0.30)


def test_cap_ignores_a_receding_obstacle():
    """v_obs is clamped to the approaching half upstream, but the cap must not
    depend on that: a negative value here can only mean 'moving away'."""
    assert retreat_cap_speed(-0.40, AT_BARRIER, **CAP_KW) == 0.05


def test_cap_escalates_with_barrier_depth():
    """Inside d_safe the escalation is proportional to how far inside — but
    only in proportion to an approach that is actually happening, so the ramp
    has to be at full strength for the raw depth term to appear."""
    assert np.isclose(retreat_cap_speed(0.30, -0.05, **CAP_KW),
                      0.05 + 0.30 + 0.10)


def test_cap_is_ceilinged():
    """Whatever the terms add up to, max_speed is the last word."""
    assert retreat_cap_speed(2.0, -1.0, **CAP_KW) == 0.6


def test_cap_is_monotone_in_both_drivers():
    """A faster approach or a deeper violation may never BUY LESS retreat —
    a sign slip in either term would show up as a non-monotone cap."""
    caps_v = [retreat_cap_speed(v, 0.0, **CAP_KW) for v in np.linspace(0, 1, 11)]
    caps_h = [retreat_cap_speed(0.3, -d, **CAP_KW) for d in np.linspace(0, 0.3, 11)]
    assert np.all(np.diff(caps_v) >= 0.0)
    assert np.all(np.diff(caps_h) >= 0.0)


# ── Proportionality to the approach speed ────────────────────────────────────
#
# The property the user asked for, and the one the depth term used to break:
# "la velocità di avoidance deve essere proporzionale alla velocità di
# avvicinamento dell'ostacolo — magari l'ostacolo si muove lentamente ma il
# robot si allontana troppo velocemente ed è rischioso."

def test_a_slow_obstacle_never_buys_a_fast_retreat_however_deep():
    """The regression. A crawling obstacle (0.02 m/s) that has ended up 15 cm
    inside d_safe: geometry alarming, approach negligible. Before the ramp this
    authorised 0.05 + 0.02 + 0.30 = 0.37 m/s — an arm bolting away from
    something barely moving. Now the depth term is scaled by how fast the thing
    is actually closing."""
    slow_deep = retreat_cap_speed(0.02, -0.15, **CAP_KW)
    assert slow_deep < 0.10, slow_deep
    # ...while the SAME depth with a genuine approach still escalates fully.
    assert retreat_cap_speed(0.40, -0.15, **CAP_KW) > 4 * slow_deep


def test_the_cap_is_dominated_by_the_proportional_term():
    """Away from the ceiling, doubling the approach speed must roughly double
    the retreat allowance. If some speed-independent term dominated, this ratio
    would flatten out — which is exactly what 'si allontana troppo velocemente'
    looks like from the outside."""
    lo = retreat_cap_speed(0.10, 0.0, **CAP_KW) - CAP_KW['base']
    hi = retreat_cap_speed(0.20, 0.0, **CAP_KW) - CAP_KW['base']
    assert np.isclose(hi / lo, 2.0)


def test_a_stationary_obstacle_leaves_only_the_creep():
    """Nothing is closing, so nothing is getting worse: whatever the geometry,
    the arm may only walk out of the violation. This is what makes 'base' a
    creep rather than a retreat speed."""
    for depth in (0.0, 0.05, 0.20, 1.0):
        assert retreat_cap_speed(0.0, -depth, **CAP_KW) == CAP_KW['base']


def test_the_ramp_saturates_at_the_reference_speed():
    """Above depth_speed_ref the escalation is at full strength and stops
    growing with speed on its own account — the obs_gain term carries it from
    there, so the two do not compound quadratically."""
    # max_speed lifted out of the way: this test is about the ramp, and at the
    # shipped 0.6 m/s ceiling both samples would simply saturate there.
    kw = dict(CAP_KW, max_speed=10.0)
    ref = kw['depth_speed_ref']
    at_ref = retreat_cap_speed(ref, -0.05, **kw)
    above = retreat_cap_speed(2 * ref, -0.05, **kw)
    # The whole difference is the linear obs_gain term, not the depth term.
    assert np.isclose(above - at_ref, kw['obs_gain'] * ref)


def test_zero_reference_restores_the_speed_independent_behaviour():
    """Escape hatch: depth_speed_ref <= 0 must mean 'no ramp', not a division
    by zero, so the older behaviour stays reachable from configuration."""
    kw = dict(CAP_KW, depth_speed_ref=0.0)
    assert np.isclose(retreat_cap_speed(0.0, -0.05, **kw),
                      CAP_KW['base'] + 0.10)


# ── The row's sign convention ────────────────────────────────────────────────

def _n_dot_J(rng):
    """A stand-in for n̂ᵀJ: the direction whose rate the cap bounds."""
    return rng.normal(size=NV)


def test_rhs_encodes_the_one_step_bound():
    """The cap row is stored as a = −n̂ᵀJ so the node's generic G = [−A | −e]
    turns it into an UPPER bound. Check the algebra end to end: the resulting
    row must be exactly n̂ᵀJ(q̇ + q̈·T) ≤ v_cap."""
    rng = np.random.default_rng(0)
    aJ = _n_dot_J(rng)
    qdot = rng.normal(size=NV) * 0.3
    T, v_cap = 0.15, 0.25

    A_cap = -aJ[None, :]
    h_qp = retreat_cap_rhs(A_cap, qdot, np.array([v_cap]), T)

    # The QP row, as assembled by the node: G[:, :NV] = -A  ⇒  +aJ.
    G = np.zeros((1, NX))
    G[:, :NV] = -A_cap
    G[0, NV + G_CAP] = -1.0

    # Pick a q̈ that sits exactly on the bound implied by the row (s = 0).
    qddot = aJ * ((h_qp[0] - 0.0) / float(aJ @ aJ))
    assert np.isclose(float(G[0, :NV] @ qddot), h_qp[0])
    # ...and that q̈ must make the one-step rate land exactly on v_cap.
    assert np.isclose(float(aJ @ (qdot + qddot * T)), v_cap)


def test_slack_relaxes_the_cap_upward():
    """s_cap must open the bound, not close it: with the slack column at −1 the
    row is +aᵀq̈ − s ≤ h_qp, so paying slack allows MORE retreat."""
    rng = np.random.default_rng(1)
    aJ = _n_dot_J(rng)
    G = np.zeros((1, NX))
    G[:, :NV] = aJ
    G[0, NV + G_CAP] = -1.0
    qddot = aJ * 10.0
    x0 = np.concatenate([qddot, [0.0, 0.0]])
    x1 = np.concatenate([qddot, [0.0, 5.0]])
    assert float(G[0] @ x1) < float(G[0] @ x0)


# ── Against a real QP ────────────────────────────────────────────────────────

def _solve(rows, rho, qddot_nom, aJ, box=50.0):
    """The filter's QP, reduced to the one direction every row here acts on.

        min ½‖q̈ − q̈_nom‖² + ½Σ ρ_g s_g²   s.t.  G x ≤ u,  s ≥ 0,  |q̈| ≤ box

    Two reductions, both exact for the problems in this file:

    * every row constrains the same ``n̂ᵀJ``, so writing ``q̈ = α·â + w`` with
      ``â = aJ/‖aJ‖`` leaves ``w`` unconstrained (``w = w_nom``) and one scalar
      ``α`` under constraint;
    * each slack appears in exactly one row, so for a given ``α`` its optimum is
      ``s_k = max(0, g_k·α − u_k)`` in closed form.

    What is left is a convex, C¹, one-dimensional minimisation — solved exactly
    instead of handed to SLSQP, which on these instances converges to the right
    point and then reports "positive directional derivative for linesearch"
    depending on ``ftol``. Same reasoning as ``_solve_row_closed_form`` in
    test_cbf_multi_cp_qp, and the same reason neither file depends on OSQP.

    The reduction is NOT a shortcut around the assertions: the scalar
    coefficients are read out of the assembled ``G`` rows, so a sign slip in the
    row convention still fails the test.
    """
    a_hat = aJ / float(np.linalg.norm(aJ))
    alpha_nom = float(a_hat @ qddot_nom)
    w_nom = qddot_nom - alpha_nom * a_hat

    # Row k, in the reduced variables: g_k·α − s_{fam(k)} ≤ u_k.
    g = np.array([float(r[0][0, :NV] @ a_hat) for r in rows])
    fam = [int(np.argmin(r[0][0, NV:])) for r in rows]      # the −1 column
    u = np.array([r[1] for r in rows])
    assert len(set(fam)) == len(fam), 'this reduction assumes one row per family'

    def slacks(alpha):
        s = np.zeros(N_SLACK)
        for k, f in enumerate(fam):
            s[f] = max(0.0, g[k] * alpha - u[k])
        return s

    def cost(alpha):
        s = slacks(alpha)
        return (0.5 * (alpha - alpha_nom) ** 2
                + 0.5 * sum(rho[f] * s[f] ** 2 for f in fam))

    res = minimize_scalar(cost, bounds=(-box, box), method='bounded',
                          options={'xatol': 1e-12})
    assert res.success, res.message
    return res.x * a_hat + w_nom, slacks(res.x)


def _cap_row(aJ, qdot, v_cap, T=0.15):
    A_cap = -aJ[None, :]
    u = retreat_cap_rhs(A_cap, qdot, np.array([v_cap]), T)[0]
    G = np.zeros((1, NX))
    G[:, :NV] = -A_cap
    G[0, NV + G_CAP] = -1.0
    return G, u


def _barrier_row(aJ, bound):
    """HOCBF row aᵀq̈ + s ≥ bound, in the node's G x ≤ u form."""
    G = np.zeros((1, NX))
    G[:, :NV] = -aJ
    G[0, NV + G_OBS] = -1.0
    return G, -bound


def test_cap_holds_a_nominal_that_wants_to_bolt():
    """A nominal command demanding a huge retreat, with no barrier pushing: the
    cap alone must bring the one-step separation rate down to v_cap."""
    rng = np.random.default_rng(2)
    aJ = _n_dot_J(rng)
    qdot = np.zeros(NV)
    T, v_cap = 0.15, 0.20
    qddot_nom = aJ * 40.0                      # ≈ 40·‖aJ‖² m/s² of pure retreat

    # Unfiltered, the nominal alone blows straight through the cap.
    assert float(aJ @ (qdot + qddot_nom * T)) > 5 * v_cap

    qddot, slack = _solve([_cap_row(aJ, qdot, v_cap, T)],
                          {G_OBS: 1e3, G_CAP: 1e2}, qddot_nom, aJ)
    rate = float(aJ @ (qdot + qddot * T))
    # The cap is soft (it has a slack), so it is approached, not enforced to the
    # metre. What matters is the ORDER of magnitude of the reduction.
    assert rate < 0.5 * float(aJ @ (qdot + qddot_nom * T))
    assert slack[G_CAP] > 0.0                  # and the QP paid for what it kept


def test_barrier_outranks_the_cap_when_they_conflict():
    """The case that must never regress: an obstacle closing faster than the cap
    allows. The barrier (rho 1000) and the cap (rho 100) bound the same
    direction from opposite sides; the CAP is the one that has to yield."""
    rng = np.random.default_rng(3)
    aJ = _n_dot_J(rng)
    qdot = np.zeros(NV)
    demanded = 8.0 * float(aJ @ aJ)            # barrier: aᵀq̈ ≥ this
    rows = [_barrier_row(aJ, demanded), _cap_row(aJ, qdot, 0.05, 0.15)]
    qddot, slack = _solve(rows, {G_OBS: 1000.0, G_CAP: 100.0},
                          np.zeros(NV), aJ)

    assert float(aJ @ qddot) > 0.8 * demanded  # barrier essentially satisfied
    assert slack[G_CAP] > slack[G_OBS]         # ...and the cap is what paid


def test_cap_wins_when_the_barrier_is_not_pushing():
    """Same two rows, but with the barrier slack (obstacle far): now nothing
    fights the cap and it must bind hard."""
    rng = np.random.default_rng(4)
    aJ = _n_dot_J(rng)
    qdot = aJ * 0.5 / float(aJ @ aJ)           # already separating at 0.5 m/s
    rows = [_barrier_row(aJ, -50.0),           # non-binding lower bound
            _cap_row(aJ, qdot, 0.15, 0.15)]
    qddot, slack = _solve(rows, {G_OBS: 1000.0, G_CAP: 100.0},
                          aJ * 20.0, aJ)

    assert float(aJ @ qddot) < 0.0             # told to DECELERATE the retreat
    assert slack[G_OBS] == 0.0                 # barrier never paid a thing


def test_cap_never_forces_motion_toward_the_obstacle():
    """A cap is an upper bound, not a target: a robot already separating more
    slowly than v_cap must be left alone by this row."""
    rng = np.random.default_rng(5)
    aJ = _n_dot_J(rng)
    qdot = aJ * 0.02 / float(aJ @ aJ)          # creeping away at 0.02 m/s
    G, u = _cap_row(aJ, qdot, 0.30, 0.15)
    qddot, slack = _solve([(G, u)], {G_OBS: 1e3, G_CAP: 1e2},
                          np.zeros(NV), aJ)
    np.testing.assert_allclose(qddot, np.zeros(NV), atol=1e-6)
    assert slack[G_CAP] < 1e-6
