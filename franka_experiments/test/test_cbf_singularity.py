"""The singularity HOCBF row: gradient, activation window, and drift sign.

The row this guards was added after two hardware aborts with the SAME shape:
``n_act=0``, the closest obstacle a comfortable 0.26–0.33 m away, and the arm
nevertheless riding its joint-velocity limit until the firmware fired
``joint_velocity_violation``. Near a singularity ``‖a‖ = ‖n̂ᵀJ‖`` collapses, so
the obstacle row has to buy a modest Cartesian retreat with an ever larger
``q̈`` — the velocity box underneath can only truncate that after the fact, it
cannot steer the pose out of the ill-conditioned region. ``σ_min(J̃)`` is the
amplification factor itself, so a floor on it is a ceiling on that blow-up.

Pinocchio is only needed for ``sigma()``; every test here overrides it with an
analytic surrogate, so the numerics are checked against a closed form instead of
against the robot model. Pure numpy otherwise.
"""

import numpy as np

from franka_experiments.utils.cbf_singularity import SingularityRowBuilder


NV = 7


class _FakeModel:
    """Minimal stand-in: the builder only needs ``nv`` and ``createData()``."""

    nv = NV

    def createData(self):
        return object()


class _AnalyticSigma(SingularityRowBuilder):
    """σ(q) with a closed-form gradient, so ∇σ can be checked exactly.

    σ(q) = base + cᵀq  is linear: its finite-difference gradient must reproduce
    ``c`` to machine precision up to the FD truncation term (zero for a linear
    function, so this is an exact check of the differencing code itself).
    """

    def __init__(self, c, base=0.0, **kw):
        super().__init__(_FakeModel(), 0, **kw)
        self.c = np.asarray(c, dtype=np.float64)
        self.base = float(base)
        self.n_sigma_calls = 0

    def sigma(self, q):
        self.n_sigma_calls += 1
        return self.base + float(self.c @ np.asarray(q))


def _q(v=0.0):
    return np.full(NV, v)


# ── Activation window ────────────────────────────────────────────────────────

def test_no_row_when_well_conditioned():
    """h = σ − floor ≥ horizon ⇒ no row at all (n_c must not grow for nothing)."""
    b = _AnalyticSigma(np.ones(NV) * 0.1, base=0.30,
                       sigma_floor=0.05, horizon=0.08)
    assert b.build(_q(), np.zeros(NV), 0.0) == []
    # ...and the barrier value is still published for the diagnostic.
    assert b.last_sigma == 0.30


def test_row_appears_inside_the_horizon():
    b = _AnalyticSigma(np.ones(NV) * 0.1, base=0.10,
                       sigma_floor=0.05, horizon=0.08)
    rows = b.build(_q(), np.zeros(NV), 0.0)
    assert len(rows) == 1
    a, h, jdq, label = rows[0]
    assert a.shape == (NV,)
    assert h == 0.10 - 0.05
    assert jdq == 0.0                     # first rebuild: no previous gradient
    assert label.startswith('sing')


def test_row_survives_a_negative_barrier():
    """σ already BELOW the floor is the case the row exists for — h < 0, still
    a row, and the QP sees a strongly negative barrier through k0·h."""
    b = _AnalyticSigma(np.ones(NV) * 0.1, base=0.02,
                       sigma_floor=0.05, horizon=0.08)
    rows = b.build(_q(), np.zeros(NV), 0.0)
    assert len(rows) == 1
    assert rows[0][1] < 0.0


# ── Gradient ─────────────────────────────────────────────────────────────────

def test_gradient_matches_the_closed_form():
    c = np.array([0.3, -0.2, 0.5, 0.1, -0.4, 0.05, 0.0])
    b = _AnalyticSigma(c, base=0.10, sigma_floor=0.05, horizon=0.08, eps=1e-4)
    a = b.build(_q(), np.zeros(NV), 0.0)[0][0]
    np.testing.assert_allclose(a, c, rtol=0, atol=1e-8)


def test_gradient_costs_nv_plus_one_evaluations():
    """Forward (not central) differences: the cost budget the 50 Hz rebuild was
    sized for. A silent switch to central differencing would double it."""
    b = _AnalyticSigma(np.ones(NV) * 0.1, base=0.10,
                       sigma_floor=0.05, horizon=0.08)
    b.build(_q(), np.zeros(NV), 0.0)
    assert b.n_sigma_calls == NV + 1


def test_row_dropped_when_gradient_has_no_leverage():
    """σ_min is non-smooth where the two smallest singular values cross; there
    the finite difference is noise. No row is the honest answer — the
    velocity/position box underneath is untouched either way."""
    b = _AnalyticSigma(np.zeros(NV), base=0.10, sigma_floor=0.05,
                       horizon=0.08, min_leverage=1e-3)
    assert b.build(_q(), np.zeros(NV), 0.0) == []


# ── Drift term ───────────────────────────────────────────────────────────────

def _rows_with_moving_gradient(drift_relaxes):
    """Two rebuilds whose gradients differ, so ċ = (Δ∇σ/Δt)ᵀq̇ is nonzero.

    ∇σ goes from c0 to c1 over 20 ms with q̇ = 1 on joint 0, so
    ċ = (c1[0] − c0[0]) / 0.02, whose SIGN is what the clamp is about.
    """
    c0 = np.zeros(NV)
    c1 = np.zeros(NV)
    c0[0], c1[0] = 0.20, 0.40          # gradient GROWING ⇒ ċ > 0 ⇒ would relax
    qdot = np.zeros(NV)
    qdot[0] = 1.0
    b = _AnalyticSigma(c0, base=0.10, sigma_floor=0.05, horizon=0.08,
                       drift_relaxes=drift_relaxes)
    b.build(_q(), qdot, 0.00)
    b.c = c1
    return b.build(_q(), qdot, 0.02)[0][2]


def test_drift_cannot_relax_the_row_by_default():
    """ċ enters h_qp additively, so a positive ċ LOOSENS the constraint. It is a
    finite difference of a finite difference — the noisiest term in the row —
    so by default it may only ever tighten."""
    assert _rows_with_moving_gradient(drift_relaxes=False) == 0.0


def test_drift_is_passed_through_when_explicitly_allowed():
    jdq = _rows_with_moving_gradient(drift_relaxes=True)
    assert np.isclose(jdq, (0.40 - 0.20) / 0.02)


def test_tightening_drift_is_kept_under_the_clamp():
    """The clamp is one-sided: a NEGATIVE ċ (barrier curving the wrong way) is
    exactly what the HOCBF needs and must survive."""
    c0, c1 = np.zeros(NV), np.zeros(NV)
    c0[0], c1[0] = 0.40, 0.20          # gradient SHRINKING ⇒ ċ < 0
    qdot = np.zeros(NV)
    qdot[0] = 1.0
    b = _AnalyticSigma(c0, base=0.10, sigma_floor=0.05, horizon=0.08)
    b.build(_q(), qdot, 0.00)
    b.c = c1
    assert np.isclose(b.build(_q(), qdot, 0.02)[0][2], (0.20 - 0.40) / 0.02)


def test_drift_memory_resets_when_the_row_deactivates():
    """Leaving and re-entering the horizon must not difference against a stale
    gradient from before the gap — that would fabricate a huge ċ."""
    b = _AnalyticSigma(np.ones(NV) * 0.2, base=0.10,
                       sigma_floor=0.05, horizon=0.08)
    b.build(_q(), np.ones(NV), 0.0)
    b.base = 0.30                       # well conditioned again → no row
    assert b.build(_q(), np.ones(NV), 0.1) == []
    b.base = 0.10                       # back inside the horizon
    assert b.build(_q(), np.ones(NV), 5.0)[0][2] == 0.0


# ── The property the row is for ──────────────────────────────────────────────

def test_row_pushes_away_from_the_singularity():
    """Sanity on the SIGN convention shared with every other row:

        aᵀq̈ + s ≥ −k1·(aᵀq̇) − k0·h − ċ

    With h < 0 (already past the floor) and q̇ = 0, the bound is −k0·h > 0, so
    the QP is forced to pick q̈ with a POSITIVE component along ∇σ — i.e. to
    accelerate in the direction that INCREASES σ_min. That is the whole point:
    the filter steers out of the ill-conditioned region instead of waiting for
    the velocity box to truncate the blow-up it causes.
    """
    k0 = 25.0
    c = np.array([0.3, -0.2, 0.5, 0.1, -0.4, 0.05, 0.0])
    b = _AnalyticSigma(c, base=0.02, sigma_floor=0.05, horizon=0.08)
    a, h, jdq, _ = b.build(_q(), np.zeros(NV), 0.0)[0]

    bound = -k0 * h - jdq               # right-hand side with q̇ = 0
    assert bound > 0.0
    # Least-norm q̈ satisfying aᵀq̈ = bound, i.e. what the QP converges to when
    # the nominal is zero: it lies ALONG +∇σ.
    qddot = a * (bound / float(a @ a))
    assert float(a @ qddot) > 0.0
    assert float(np.dot(qddot, c)) > 0.0
