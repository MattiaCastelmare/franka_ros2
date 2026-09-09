"""Quantising the QP's row count, so OSQP stops re-factorizing every tick.

OSQP's sparsity pattern is a function of the row count, so the node must build
a fresh ``osqp.OSQP()`` and factorize whenever ``n_c`` changes — and n_c changes
almost every rebuild (22 -> 29 -> 30 -> 34 -> 36 -> 37 across a few hundred ms
of one hardware log) as perception's control points and the barrier rows engage
and disengage. In a 100 Hz Python node that is a per-tick allocate-and-factorize
spike, and the executor's IO thread is what starves: the logged
"joint state stale -> braking on last known q̇" immediately preceding a
``joint_velocity_violation``.

``pad_rows_to_block`` absorbs the churn by rounding the row count up to a fixed
block. The property that makes this safe — and the only one worth testing — is
that the padding rows are INERT: the padded problem must have exactly the same
solution as the unpadded one. That is asserted here against a real solve, not
argued.

Pure numpy + scipy.
"""

import numpy as np
from scipy.optimize import minimize

from franka_experiments.utils.cbf_qp_assembly import (
    build_osqp_A,
    build_osqp_bounds,
    pad_rows_to_block,
)

NV = 7
N_SLACK = 2
NX = NV + N_SLACK


def _rows(rng, n_c):
    """A random but well-posed CBF row block, in the node's [-A | -e] form."""
    G = np.zeros((n_c, NX))
    G[:, :NV] = rng.normal(size=(n_c, NV))
    for i in range(n_c):
        G[i, NV + (i % N_SLACK)] = -1.0
    h = rng.normal(size=n_c) * 0.5
    return G, h


# ── Shape arithmetic ─────────────────────────────────────────────────────────

def test_rounds_up_to_the_block():
    rng = np.random.default_rng(0)
    for n_c, expect in ((1, 16), (15, 16), (16, 16), (17, 32), (33, 48), (48, 48)):
        G, h = _rows(rng, n_c)
        Gp, hp = pad_rows_to_block(G, h, 16)
        assert Gp.shape == (expect, NX), (n_c, Gp.shape)
        assert hp.shape == (expect,)


def test_every_count_inside_one_block_yields_one_pattern():
    """The whole point: the shape OSQP sees must be constant across the range
    of n_c the filter actually walks through, so setup() runs once."""
    rng = np.random.default_rng(1)
    shapes = set()
    for n_c in range(33, 49):
        G, h = _rows(rng, n_c)
        Gp, _ = pad_rows_to_block(G, h, 16)
        shapes.add(Gp.shape)
    assert len(shapes) == 1, shapes


def test_exact_multiple_is_returned_by_reference():
    """No allocation on the common steady-state path."""
    rng = np.random.default_rng(2)
    G, h = _rows(rng, 32)
    Gp, hp = pad_rows_to_block(G, h, 16)
    assert Gp is G and hp is h


def test_block_of_one_or_less_disables_padding():
    """Escape hatch back to the exact previous behaviour."""
    rng = np.random.default_rng(3)
    G, h = _rows(rng, 7)
    for block in (1, 0, -4):
        Gp, hp = pad_rows_to_block(G, h, block)
        assert Gp is G and hp is h


def test_no_rows_stays_no_rows():
    """n_c == 0 keeps the node's allocation-free "box bounds only" fast path;
    padding it would create constraints where the design has none."""
    Gp, hp = pad_rows_to_block(None, None, 16)
    assert Gp is None and hp is None


# ── The padding rows are inert ───────────────────────────────────────────────

def test_padding_rows_are_never_binding():
    """A padded row is ``-s <= +inf`` for a slack already bounded below by 0:
    satisfiable by every feasible point, so it cannot cut anything away."""
    rng = np.random.default_rng(4)
    G, h = _rows(rng, 20)
    Gp, hp = pad_rows_to_block(G, h, 16)
    pad = Gp[20:]
    assert np.all(np.isinf(hp[20:])) and np.all(hp[20:] > 0)
    assert np.all(pad[:, :NV] == 0.0)          # no opinion about any joint
    assert np.all(pad[:, -1] == -1.0)          # non-zero row, so no zero-row
    assert np.count_nonzero(pad) == pad.shape[0]


def test_padded_rows_survive_into_the_osqp_structures():
    """Shape agreement end to end: the A matrix and the (l, u) pair the node
    hands OSQP must both carry the padded count, or setup() would reject it."""
    rng = np.random.default_rng(5)
    G, h = _rows(rng, 20)
    Gp, hp = pad_rows_to_block(G, h, 16)
    A = build_osqp_A(Gp, NV, N_SLACK)
    l, u = build_osqp_bounds(Gp, hp, np.full(NX, -50.0), np.full(NX, 50.0))
    assert A.shape == (32 + NX, NX)
    assert l.shape == u.shape == (32 + NX,)


#: Stand-in for +inf when handing a padded row to the optimiser. The node puts
#: a true +inf in h_qp (OSQP's own convention for "no upper bound"), but a
#: constraint function returning inf gives a numerical optimiser nothing to
#: work with — so the row is passed as an enormous FINITE bound instead. That
#: keeps the padding rows genuinely present in the problem the solver sees,
#: which is the whole point: filtering them out by ``isfinite`` would make the
#: "padding changes nothing" assertion tautological.
_BIG = 1e6


def _solve(G, h, qddot_nom, rho):
    """The filter's QP: min ½‖q̈ − q̈_nom‖² + ½Σρ_g s_g²  s.t. Gx ≤ h, s ≥ 0.

    Solved with scipy rather than OSQP so this file — like the rest of the CBF
    tests — needs no solver beyond scipy.

    ``trust-constr``, not SLSQP: on the hard-binding fixtures below SLSQP
    reaches the right point and then reports "Positive directional derivative
    for linesearch" (status 8), the same convergence artefact
    ``test_cbf_retreat_cap`` documents and works around. This is a small,
    strictly convex QP; trust-constr solves it without the artefact.
    """
    def cost(x):
        d = x[:NV] - qddot_nom
        return 0.5 * float(d @ d) + 0.5 * float(sum(
            rho[g] * x[NV + g] ** 2 for g in range(N_SLACK)))

    ub = np.where(np.isfinite(h), h, _BIG)
    cons = [{'type': 'ineq', 'fun': (lambda x, i=i: float(ub[i] - G[i] @ x))}
            for i in range(G.shape[0])]
    bounds = [(-50.0, 50.0)] * NV + [(0.0, None)] * N_SLACK
    res = minimize(cost, np.zeros(NX), constraints=cons, bounds=bounds,
                   method='trust-constr', options={'maxiter': 3000,
                                                   'gtol': 1e-12,
                                                   'xtol': 1e-12})
    assert res.status in (1, 2), (res.status, res.message)
    return res.x


#: Tolerance for "the padded problem has the same solution". trust-constr's own
#: convergence floor on these fixtures is ~1e-5 absolute on components of
#: magnitude ~20, so anything tighter tests the optimiser rather than the
#: padding. That it is nonetheless a strong bound is asserted directly by
#: test_the_comparison_would_catch_a_binding_padding_row below: a padding row
#: that actually bound moves the solution ORDERS of magnitude further than
#: this.
_SOLVE_ATOL = 1e-4


def test_padding_does_not_change_the_solution():
    """The property the whole change rests on. Same problem, padded and not:
    the optimiser must land on the same point, or the quantisation would be
    silently altering the command sent to the arm."""
    rng = np.random.default_rng(6)
    rho = {0: 1000.0, 1: 200.0}
    for trial in range(5):
        n_c = int(rng.integers(3, 15))
        G, h = _rows(rng, n_c)
        qddot_nom = rng.normal(size=NV) * 2.0
        Gp, hp = pad_rows_to_block(G, h, 16)
        assert Gp.shape[0] == 16 and Gp is not G      # padding really happened
        x_raw = _solve(G, h, qddot_nom, rho)
        x_pad = _solve(Gp, hp, qddot_nom, rho)
        np.testing.assert_allclose(x_pad, x_raw, atol=_SOLVE_ATOL,
                                   err_msg=f'trial {trial}, n_c={n_c}')


def test_padding_does_not_change_the_solution_when_rows_bind_hard():
    """Same check with the constraints actually active — a nominal command
    that drives straight into every row, so the solution sits on the boundary
    where an accidentally-binding padding row would show up immediately."""
    rng = np.random.default_rng(7)
    rho = {0: 1000.0, 1: 200.0}
    G, h = _rows(rng, 9)
    h[:] = -0.5                                   # every row demanding
    qddot_nom = rng.normal(size=NV) * 20.0
    Gp, hp = pad_rows_to_block(G, h, 16)
    x_raw = _solve(G, h, qddot_nom, rho)
    x_pad = _solve(Gp, hp, qddot_nom, rho)
    np.testing.assert_allclose(x_pad, x_raw, atol=_SOLVE_ATOL)
    assert np.any(x_raw[NV:] > 1e-6), 'the fixture should be paying slack'


def test_the_comparison_would_catch_a_binding_padding_row():
    """Test of the test: prove _SOLVE_ATOL is not so loose that it hides a
    padding row with real teeth. Take the same fixture and corrupt exactly the
    thing the implementation is trusted to get right — the padding row's RHS —
    and the comparison above must fail."""
    rng = np.random.default_rng(8)
    rho = {0: 1000.0, 1: 200.0}
    G, h = _rows(rng, 9)
    h[:] = -0.5
    qddot_nom = rng.normal(size=NV) * 20.0
    Gp, hp = pad_rows_to_block(G, h, 16)

    corrupt = Gp.copy()
    corrupt[9:, :NV] = 1.0          # a padding row that DOES constrain q̈...
    hp_corrupt = hp.copy()
    hp_corrupt[9:] = -1.0           # ...and demands something of it

    x_raw = _solve(G, h, qddot_nom, rho)
    x_bad = _solve(corrupt, hp_corrupt, qddot_nom, rho)
    assert np.max(np.abs(x_bad - x_raw)) > 100 * _SOLVE_ATOL, (
        'the inert-padding assertion has no teeth')
