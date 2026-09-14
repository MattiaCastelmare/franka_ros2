"""Tangential steering: go AROUND a close obstacle, not just back off it.

``tangential_bias`` biases the QP's target (q̈_nom) sideways so the solver has
an actual incentive to use the FREE part of q̈ — the part orthogonal to every
active barrier row, which the barrier itself has no opinion about — instead of
letting it sit at zero while the blocked normal component is clipped.

v1 of this (single hard branch switch between "amplify q̈_nom's own sideways
part" and "fall back to qdot's", no cap, no cross-tick filter) measured as
visibly oscillatory on hardware. This file is the regression for v2's fix:
every blend is a smoothstep, the qdot fallback carries its own confidence
instead of being normalised outright, and the total is norm-capped. The
cross-tick EMA lives in the node (cbf_safety_filter._qp_tick) and is not pure
numpy, so it is not exercised here — this file covers the stateless per-tick
map only.

Pure numpy.
"""

import numpy as np

from franka_experiments.utils.cbf_qp_assembly import tangential_bias
from franka_experiments.utils.cbf_state_rows import (
    G_OBS,
    G_QLIM,
    G_SC,
    ConstraintSnap,
)

NV = 7


def _con(a_rows, h_bar, group=None):
    """A minimal ConstraintSnap — only the fields tangential_bias reads."""
    A = np.array(a_rows, dtype=np.float64)
    n_c = A.shape[0]
    if group is None:
        group = np.full(n_c, G_OBS)
    return ConstraintSnap(
        A=A, h_bar=np.array(h_bar, dtype=np.float64), jdot_qdot=np.zeros(n_c),
        G=None, t_dist=0.0, links=tuple(f'l#{i}' for i in range(n_c)),
        d_obs_min=float(np.min(h_bar)) if n_c else float('inf'),
        group=np.asarray(group), d_sc_min=float('inf'), v_obs=np.zeros(n_c),
        b_ff=None, cap_v=np.zeros(0), n_cap=0, n_rtr=0)


def _single_row(a=(1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0), h=0.05, group=G_OBS):
    return _con([a], [h], group=[group])


KW = dict(gain=1.0, engage_margin=0.30)


# ── Disable / no-op paths ─────────────────────────────────────────────────────

def test_zero_gain_returns_exact_zero():
    con = _single_row()
    qn = np.array([1.0, 2.0, 0, 0, 0, 0, 0])
    bias = tangential_bias(qn, np.zeros(NV), con, gain=0.0, engage_margin=0.30)
    np.testing.assert_array_equal(bias, np.zeros(NV))


def test_no_constraint_snapshot_returns_exact_zero():
    qn = np.array([1.0, 2.0, 0, 0, 0, 0, 0])
    bias = tangential_bias(qn, np.zeros(NV), None, **KW)
    np.testing.assert_array_equal(bias, np.zeros(NV))


def test_empty_row_set_returns_exact_zero():
    con = _con([], [])
    qn = np.array([1.0, 2.0, 0, 0, 0, 0, 0])
    bias = tangential_bias(qn, np.zeros(NV), con, **KW)
    np.testing.assert_array_equal(bias, np.zeros(NV))


def test_row_outside_engage_margin_contributes_nothing():
    con = _single_row(h=0.9)                    # far outside 0.30 m margin
    qn = np.array([1.0, 2.0, 0, 0, 0, 0, 0])
    bias = tangential_bias(qn, np.zeros(NV), con, **KW)
    np.testing.assert_array_equal(bias, np.zeros(NV))


def test_joint_limit_and_singularity_rows_do_not_engage():
    """Only OBSTACLE/SELF-COLLISION rows are geometric proximities this
    function knows how to interpret; joint-limit rows are in radians, not
    metres, and must not be swept in by an accidental h̄ < 0.30 match."""
    con = _single_row(h=0.05, group=G_QLIM)
    qn = np.array([1.0, 2.0, 0, 0, 0, 0, 0])
    bias = tangential_bias(qn, np.zeros(NV), con, **KW)
    np.testing.assert_array_equal(bias, np.zeros(NV))


def test_zero_nominal_returns_exact_zero():
    """No q̈_nom means no scale to amplify or fall back to — must not divide
    by ~0 and produce a spurious full-scale direction."""
    con = _single_row()
    bias = tangential_bias(np.zeros(NV), np.zeros(NV), con, **KW)
    np.testing.assert_array_equal(bias, np.zeros(NV))


# ── Safety invariant: never touches the normal direction ────────────────────

def test_bias_is_orthogonal_to_the_active_normal():
    """The one property that must never regress: whatever this function
    returns, it cannot change the QP's normal-direction retreat, because that
    would mean fighting the barrier via the cost instead of respecting it."""
    rng = np.random.default_rng(0)
    a = rng.normal(size=NV)
    con = _single_row(a=a, h=0.05)
    qn = rng.normal(size=NV) * 2.0
    qdot = rng.normal(size=NV) * 0.3
    bias = tangential_bias(qn, qdot, con, **KW)
    a_hat = a / np.linalg.norm(a)
    assert abs(float(bias @ a_hat)) < 1e-9


def test_multi_row_orthogonal_to_each_active_normal_when_directions_agree():
    """Two rows with the SAME â: the sum is still exactly orthogonal to it
    (the multi-row non-orthogonality caveat only applies across DIFFERENT
    âᵢ, documented in the function's own docstring)."""
    rng = np.random.default_rng(1)
    a = rng.normal(size=NV)
    con = _con([a, a], [0.05, 0.10])
    qn = rng.normal(size=NV) * 2.0
    bias = tangential_bias(qn, np.zeros(NV), con, **KW)
    a_hat = a / np.linalg.norm(a)
    assert abs(float(bias @ a_hat)) < 1e-9


# ── Amplify-the-nominal branch ───────────────────────────────────────────────

def test_amplifies_existing_sideways_intent():
    """q̈_nom already has a large component orthogonal to â (joint 1): that
    component must grow, and the â-aligned component (joint 0) must be exactly
    unchanged by the ADDITION (bias ⟂ â)."""
    con = _single_row(a=(1, 0, 0, 0, 0, 0, 0), h=0.05)
    qn = np.array([1.0, 2.0, 0, 0, 0, 0, 0])
    bias = tangential_bias(qn, np.zeros(NV), con, **KW)
    assert bias[1] > 0.0
    assert abs(bias[0]) < 1e-9


def test_farther_inside_engage_margin_biases_harder():
    """The per-row weight ramps with (engage_margin - h̄)/engage_margin: a row
    just inside the margin must bias less than one deep inside it."""
    con_near_edge = _single_row(a=(1, 0, 0, 0, 0, 0, 0), h=0.29)
    con_deep      = _single_row(a=(1, 0, 0, 0, 0, 0, 0), h=0.01)
    qn = np.array([1.0, 2.0, 0, 0, 0, 0, 0])
    b_edge = tangential_bias(qn, np.zeros(NV), con_near_edge, **KW)
    b_deep = tangential_bias(qn, np.zeros(NV), con_deep, **KW)
    assert np.linalg.norm(b_deep) > np.linalg.norm(b_edge)


# ── Continuity: the v1 regression this file exists for ──────────────────────

def test_no_jump_as_nominal_sideways_component_shrinks_through_the_blend():
    """The v1 bug: a hard switch between the "amplify q̈_nom" and "fall back
    to qdot" branches. Sweep q̈_nom's orthogonal component down through the
    blend band with a FIXED, small qdot drift and check the bias magnitude
    changes smoothly — no discontinuity at any sample."""
    a = np.array([1.0, 0, 0, 0, 0, 0, 0])
    con = _single_row(a=a, h=0.05)
    qdot = np.array([0.0, 0.02, 0, 0, 0, 0, 0])   # small, fixed lateral drift
    norms = []
    for perp_mag in np.linspace(0.30, 0.0, 61):    # spans the 0.15*nom_norm band
        qn = np.array([1.0, perp_mag, 0, 0, 0, 0, 0])
        bias = tangential_bias(qn, qdot, con, **KW)
        norms.append(float(np.linalg.norm(bias)))
    norms = np.array(norms)
    step = np.abs(np.diff(norms))
    # 61 samples over the sweep: any single-sample jump much larger than the
    # typical step is exactly the discontinuity this test is written to catch.
    assert np.max(step) < 8 * np.median(step[step > 1e-9])


def test_no_jump_as_qdot_drift_direction_rotates():
    """Same continuity property from the OTHER side: with q̈_nom pinned to
    have (near) zero sideways component — so the fallback branch dominates —
    rotating qdot's drift direction must rotate the bias smoothly, not flip
    it."""
    a = np.array([1.0, 0, 0, 0, 0, 0, 0])
    con = _single_row(a=a, h=0.05)
    qn = np.array([1.0, 0, 0, 0, 0, 0, 0])         # purely head-on
    prev = None
    for theta in np.linspace(0.0, np.pi, 37):
        qdot = np.array([0.0, 0.05 * np.cos(theta), 0.05 * np.sin(theta),
                         0, 0, 0, 0])
        bias = tangential_bias(qn, qdot, con, **KW)
        if prev is not None:
            # Consecutive samples 5 deg apart must stay close in direction —
            # a flip would show up as an near-180 deg jump between neighbours.
            cos_step = float(bias @ prev) / (
                np.linalg.norm(bias) * np.linalg.norm(prev) + 1e-12)
            assert cos_step > 0.9, (theta, cos_step)
        prev = bias


def test_tiny_qdot_drift_fades_out_instead_of_normalising_to_full_scale():
    """The v1 bug's second half: normalising a near-zero, noise-scale qdot
    component produced a FULL-SCALE bias in a direction rounding error picked.
    A drift far below _TB_DRIFT_REF must contribute a correspondingly small
    bias, not a nom_norm-sized one."""
    a = np.array([1.0, 0, 0, 0, 0, 0, 0])
    con = _single_row(a=a, h=0.05)
    qn = np.array([1.0, 0, 0, 0, 0, 0, 0])          # no nominal sideways part
    tiny_qdot = np.array([0.0, 1e-6, 0, 0, 0, 0, 0])
    bias = tangential_bias(qn, tiny_qdot, con, **KW)
    assert np.linalg.norm(bias) < 1e-3 * np.linalg.norm(qn)


# ── Cap ───────────────────────────────────────────────────────────────────────

def test_max_bias_caps_the_norm():
    con = _single_row(a=(1, 0, 0, 0, 0, 0, 0), h=0.0)   # deepest possible weight
    qn = np.array([1.0, 100.0, 0, 0, 0, 0, 0])           # huge sideways intent
    bias = tangential_bias(qn, np.zeros(NV), con, gain=5.0,
                           engage_margin=0.30, max_bias=2.0)
    assert np.linalg.norm(bias) <= 2.0 + 1e-9


def test_max_bias_none_leaves_it_unbounded():
    con = _single_row(a=(1, 0, 0, 0, 0, 0, 0), h=0.0)
    qn = np.array([1.0, 100.0, 0, 0, 0, 0, 0])
    bias = tangential_bias(qn, np.zeros(NV), con, gain=5.0,
                           engage_margin=0.30, max_bias=None)
    assert np.linalg.norm(bias) > 2.0


# ── Self-collision rows engage the same as obstacle rows ────────────────────

def test_self_collision_rows_engage_like_obstacle_rows():
    con = _single_row(a=(1, 0, 0, 0, 0, 0, 0), h=0.05, group=G_SC)
    qn = np.array([1.0, 2.0, 0, 0, 0, 0, 0])
    bias = tangential_bias(qn, np.zeros(NV), con, **KW)
    assert np.linalg.norm(bias) > 0.0
