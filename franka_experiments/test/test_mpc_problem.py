"""MPC QP of utils/mpc_problem.py: horizon model, cost terms, constraint rows, OSQP solve.

The tests pin the conventions of the module and method docstrings:

- the closed form q_free + G U, qd_free + V U is the step-by-step rollout;
- every cost term is the exact quadratic expansion of what its docstring says;
- every row block means the constraint its docstring says, for any U;
- solve() returns the QP minimiser, keeps the box, and stays right across updates;
- the diagnostics report what the solution did with the human rows, the whole cost and its
  stage and terminal parts, and the multipliers of each row family.

The solve tests run with every installed solver (OSQP always, DAQP if present).
Pure numpy + QP solvers, no ROS.
"""

import importlib.util

import numpy as np
import pytest

from franka_experiments.utils.constants import NUM_JOINTS
from franka_experiments.utils.mpc_problem import MPCProblem
from franka_experiments.utils.mpc_problem import MPCSettings

INF = 1e20  # OSQP treats anything beyond 1e30 as infinite; np.inf is fine too
SOLVERS = ['osqp'] + (['daqp'] if importlib.util.find_spec('daqp') else [])


def _problem(*, q_lim=2.5, qdot_max=2.0, qddot_max=5.0, **settings) -> MPCProblem:
    n = NUM_JOINTS
    return MPCProblem(MPCSettings(**settings), q_min=-q_lim * np.ones(n), q_max=q_lim * np.ones(n),
                      qdot_max=qdot_max * np.ones(n), qddot_max=qddot_max * np.ones(n))


def _state(p: MPCProblem, q0=None, qd0=None, u_prev=None) -> None:
    """What begin_cycle does before building the QP, so each part is tested on its own."""
    n = p.n_joints
    p.q0 = np.zeros(n) if q0 is None else np.asarray(q0, dtype=float)
    p.qd0 = np.zeros(n) if qd0 is None else np.asarray(qd0, dtype=float)
    p.u_prev = np.zeros(n) if u_prev is None else np.asarray(u_prev, dtype=float)
    p.q_free, p.qd_free = p.free_response(p.q0, p.qd0)
    p.H[:] = 0.0
    p.g[:] = 0.0
    p.A[:] = 0.0


def _z(p: MPCProblem, U, s=None) -> np.ndarray:
    return np.concatenate([U, np.zeros(p.ns) if s is None else s])


def _quad(H, g, z) -> float:
    return 0.5 * z @ H @ z + g @ z


def _assert_expansion(p: MPCProblem, H, g, cost, rng) -> None:
    """½zᵀHz + gᵀz equals cost(U) up to a constant, checked on random pairs of U."""
    U0 = rng.normal(size=p.nu)
    for _ in range(3):
        U1 = rng.normal(size=p.nu)
        expected = cost(U1) - cost(U0)
        got = _quad(H, g, _z(p, U1)) - _quad(H, g, _z(p, U0))
        assert got == pytest.approx(expected, rel=1e-9, abs=1e-9)


def _brake_to_stop(p: MPCProblem) -> np.ndarray:
    """Hardest braking that stops every joint and then holds it: q̇ never reverses."""
    U = np.zeros((p.n_steps, p.n_joints))
    qd = p.qd0.copy()
    for k in range(p.n_steps):
        U[k] = -np.clip(qd / p.dt, -p.qddot_max, p.qddot_max)
        qd = qd + p.dt * U[k]
    return U.ravel()


# ── Horizon model ─────────────────────────────────────────────────────────

def test_system_matrices_are_the_exact_double_integrator():
    p = _problem()
    q, qd, u = np.full(7, 0.1), np.full(7, -0.4), np.full(7, 2.0)
    x = p.A_dyn @ np.concatenate([q, qd]) + p.B_dyn @ u
    assert np.allclose(x[:7], q + p.dt * qd + 0.5 * p.dt ** 2 * u)
    assert np.allclose(x[7:], qd + p.dt * u)


def test_closed_form_matches_the_rollout():
    p = _problem()
    rng = np.random.default_rng(0)
    for _ in range(5):
        q0, qd0, U = rng.normal(size=7), rng.normal(size=7), rng.normal(size=p.nu)
        q, qd = p.rollout(q0, qd0, U)
        q_free, qd_free = p.free_response(q0, qd0)
        for k in range(p.n_steps):
            assert np.allclose(q_free[k] + p.G[k] @ U, q[k], atol=1e-12)
            assert np.allclose(qd_free[k] + p.V[k] @ U, qd[k], atol=1e-12)


def test_free_response_is_the_rollout_with_no_input():
    p = _problem()
    q0, qd0 = np.full(7, 0.3), np.full(7, -0.2)
    q, qd = p.rollout(q0, qd0, np.zeros(p.nu))
    q_free, qd_free = p.free_response(q0, qd0)
    assert np.allclose(q_free, q) and np.allclose(qd_free, qd)


def test_shift_plan_drops_the_first_input_and_repeats_the_last():
    p = _problem()
    U = np.arange(p.nu, dtype=float)
    shifted = p.shift_plan(U).reshape(p.n_steps, 7)
    steps = U.reshape(p.n_steps, 7)
    assert p.shift_plan(U).shape == (p.nu,)
    assert np.array_equal(shifted[:-1], steps[1:])
    assert np.array_equal(shifted[-1], steps[-1])


@pytest.mark.parametrize('terminal', ['soft', 'hard', 'none'])
def test_row_blocks_have_fixed_sizes(terminal):
    p = _problem(terminal=terminal)
    n, N, K = 7, p.n_steps, p.n_pairs
    sizes = {name: r.stop - r.start for name, r in p.rows.items()}
    expected = {'u': n * N, 's': K * N, 'q': n * N, 'qd': n * N, 'human': K * N}
    if terminal == 'hard':
        expected['term'] = n
    assert sizes == expected
    assert p.n_rows == sum(expected.values()) == p.A.shape[0]


def test_invalid_settings_are_rejected():
    with pytest.raises(ValueError):
        _problem(terminal='sometimes')
    with pytest.raises(ValueError):
        _problem(solver='scipy')
    with pytest.raises(ValueError):
        MPCProblem(MPCSettings(), q_min=np.zeros(6), q_max=np.ones(7),
                   qdot_max=np.ones(7), qddot_max=np.ones(7))


# ── Cost terms ──────────────────────────────────────────────────────────────

@pytest.mark.parametrize('k', [1, 4, 10])
def test_tracking_term_is_the_exact_quadratic_expansion(k):
    p = _problem()
    rng = np.random.default_rng(k)
    _state(p, q0=rng.normal(size=7), qd0=rng.normal(size=7))
    C, r = rng.normal(size=(3, 7)), rng.normal(size=3)
    W = np.diag(rng.uniform(1.0, 5.0, size=3))
    p.add_tracking_term(k, C, r, W)

    def cost(U):
        q, _ = p.rollout(p.q0, p.qd0, U)
        e = C @ q[k - 1] - r
        return e @ W @ e

    assert np.allclose(p.H, p.H.T)
    assert not p.H[p.iS].any() and not p.g[p.iS].any()  # the task never touches the slacks
    _assert_expansion(p, p.H, p.g, cost, rng)


@pytest.mark.parametrize('terminal', ['soft', 'hard', 'none'])
def test_regularisation_is_the_exact_quadratic_expansion(terminal):
    p = _problem(terminal=terminal)
    rng = np.random.default_rng(1)
    _state(p, q0=rng.normal(size=7), qd0=rng.normal(size=7), u_prev=rng.normal(size=7))
    p._add_regularisation()
    cfg = p.cfg

    def cost(U):
        _, qd = p.rollout(p.q0, p.qd0, U)
        u = U.reshape(p.n_steps, 7)
        jerk = np.diff(np.vstack([p.u_prev, u]), axis=0) / p.dt
        J = cfg.w_velocity * np.sum(qd ** 2) + cfg.w_accel * np.sum(u ** 2) + cfg.w_jerk * np.sum(jerk ** 2)
        if terminal == 'soft':
            J += cfg.w_terminal * np.sum(qd[-1] ** 2)
        return J

    _assert_expansion(p, p.H, p.g, cost, rng)
    # Slacks: ρ·1ᵀs + ½σ‖s‖², decoupled from U
    assert np.allclose(p.H[p.iS, p.iS], cfg.slack_quadratic * np.eye(p.ns))
    assert np.allclose(p.g[p.iS], cfg.slack_linear)
    assert not p.H[p.iU, p.iS].any()


def test_hessian_is_positive_definite():
    p = _problem()
    _state(p)
    p._add_regularisation()
    assert np.allclose(p.H, p.H.T)
    assert np.linalg.eigvalsh(p.H).min() > 0.0


# ── Constraint rows ─────────────────────────────────────────────────────────

def test_limit_rows_mean_the_joint_limits():
    p = _problem()
    _state(p, q0=np.full(7, 0.2), qd0=np.full(7, 0.1))
    p._set_limit_rows()
    rng = np.random.default_rng(2)
    U = rng.normal(size=p.nu)
    q, qd = p.rollout(p.q0, p.qd0, U)
    c, m = p.cfg.qdot_scale, p.cfg.joint_margin

    rq, rqd, rs, ru = p.rows['q'], p.rows['qd'], p.rows['s'], p.rows['u']
    # Rows in step-major order: row (k−1)·7 + i is joint i at step k
    assert np.allclose(p.A[rq, p.iU] @ U + p.q_free.ravel(), q.ravel())
    assert np.allclose(p.A[rqd, p.iU] @ U + p.qd_free.ravel(), qd.ravel())
    assert not p.A[rq, p.iS].any() and not p.A[rqd, p.iS].any()
    # Far from the limits the bounds are the plain ones
    assert np.allclose(p.l[rq], np.tile(p.q_min + m, p.n_steps) - p.q_free.ravel())
    assert np.allclose(p.u[rq], np.tile(p.q_max - m, p.n_steps) - p.q_free.ravel())
    assert np.allclose(p.l[rqd], np.tile(-c * p.qdot_max, p.n_steps) - p.qd_free.ravel())
    assert np.allclose(p.u[rqd], np.tile(c * p.qdot_max, p.n_steps) - p.qd_free.ravel())
    # Slacks are non-negative; the q̈ rows are the identity (bounded in solve)
    assert np.allclose(p.A[rs, p.iS], np.eye(p.ns)) and not p.A[rs, p.iU].any()
    assert np.allclose(p.l[rs], 0.0) and np.all(p.u[rs] >= INF)
    assert np.allclose(p.A[ru, p.iU], np.eye(p.nu)) and not p.A[ru, p.iS].any()


@pytest.mark.parametrize('case', ['at_margin_moving_out', 'too_fast', 'beyond_limit'])
def test_braking_to_a_stop_satisfies_the_limit_rows(case):
    """The two traps of _set_limit_rows: from any state, stopping must be feasible."""
    p = _problem()
    c, m = p.cfg.qdot_scale, p.cfg.joint_margin
    q0, qd0 = {
        'at_margin_moving_out': (p.q_max - m, np.full(7, 0.5)),
        'too_fast':             (np.zeros(7), 1.5 * c * p.qdot_max),
        'beyond_limit':         (p.q_max + 0.02, np.zeros(7)),
    }[case]
    _state(p, q0=q0, qd0=qd0)
    p._set_limit_rows()
    z = _z(p, _brake_to_stop(p))
    for name in ('q', 'qd'):
        r = p.rows[name]
        Az = p.A[r] @ z
        assert np.all(p.l[r] <= Az + 1e-9) and np.all(Az <= p.u[r] + 1e-9), name


def test_terminal_row_is_the_stop():
    p = _problem(terminal='hard')
    qd0 = np.linspace(-0.5, 0.5, 7)
    _state(p, qd0=qd0)
    p._set_limit_rows()
    rt = p.rows['term']
    assert np.allclose(p.l[rt], p.u[rt])
    stop = np.tile(-qd0 / (p.n_steps * p.dt), p.n_steps)  # constant deceleration to rest at N
    assert np.allclose(p.A[rt, p.iU] @ stop, p.l[rt])
    U = np.random.default_rng(3).normal(size=p.nu)
    _, qd = p.rollout(p.q0, p.qd0, U)
    assert np.allclose(p.A[rt, p.iU] @ U - p.l[rt], qd[-1])  # residual = q̇_N


def test_human_rows_start_inert():
    p = _problem()
    _state(p)
    p._reset_human_rows()
    rh = p.rows['human']
    assert not p.A[rh, p.iU].any()
    assert np.allclose(p.A[rh, p.iS], np.eye(p.ns))  # row (k−1)·K + j ↔ slack (k−1)·K + j
    assert np.all(p.l[rh] <= -INF) and np.all(p.u[rh] >= INF)


def test_human_row_is_the_linearised_distance():
    p = _problem()
    rng = np.random.default_rng(4)
    _state(p, q0=rng.normal(size=7), qd0=rng.normal(size=7))
    p._reset_human_rows()
    k = 3
    a = rng.normal(size=(2, 7))
    d_bar, margin, q_bar = np.array([0.30, 0.25]), np.array([0.02, 0.05]), rng.normal(size=7)
    p.set_human_rows(k, a, d_bar, margin, q_bar)

    rh, K = p.rows['human'], p.n_pairs
    for j in range(2):
        idx = (k - 1) * K + j
        row = rh.start + idx
        # Row j means d̄ + a·(q_k − q̄) + s ≥ d_safe + δ + m, up to a positive scale of the row
        ratios = []
        for _ in range(3):
            U, s = rng.normal(size=p.nu), rng.uniform(0.0, 0.1, size=p.ns)
            q, _ = p.rollout(p.q0, p.qd0, U)
            lhs = d_bar[j] + a[j] @ (q[k - 1] - q_bar) + s[idx]
            ratios.append((p.A[row] @ _z(p, U, s) - p.l[row]) / (lhs - (p.cfg.d_safe + p.cfg.delta + margin[j])))
        assert ratios[0] > 0.0 and np.allclose(ratios, ratios[0])
        assert p.u[row] >= INF
    # The unused rows of step k and every other step stay inert
    used = {rh.start + (k - 1) * K + j for j in range(2)}
    for row in set(range(rh.start, rh.stop)) - used:
        assert not p.A[row, p.iU].any() and p.l[row] <= -INF
    # The next cycle starts inert again
    p._reset_human_rows()
    assert not p.A[rh, p.iU].any() and np.all(p.l[rh] <= -INF)


# ── Solve ───────────────────────────────────────────────────────────────────

def _track_joint_goal(p: MPCProblem, goal, weight=1000.0) -> None:
    for k in range(1, p.n_steps + 1):
        p.add_tracking_term(k, np.eye(7), goal, weight * np.eye(7))


@pytest.mark.parametrize('solver', SOLVERS)
def test_solve_returns_the_qp_minimiser_when_nothing_binds(solver):
    p = _problem(solver=solver, q_lim=10.0, qdot_max=10.0, qddot_max=100.0, trust_region=0.0)
    p.begin_cycle(np.zeros(7), np.zeros(7), np.zeros(7))
    _track_joint_goal(p, np.full(7, 0.05))
    U = p.solve(np.zeros(p.nu))
    assert U is not None and U.shape == (p.nu,)
    assert p.status and np.isfinite(p.solve_time_ms)
    U_star = -np.linalg.solve(p.H[p.iU, p.iU], p.g[p.iU])  # slacks sit at 0 (ρ > 0, s ≥ 0)
    assert np.allclose(U, U_star, atol=1e-2 * max(1.0, np.abs(U_star).max()))


@pytest.mark.parametrize('solver', SOLVERS)
def test_solve_keeps_the_acceleration_box_and_the_trust_region(solver):
    p = _problem(solver=solver, qddot_max=1.0, trust_region=0.3)
    p.begin_cycle(np.zeros(7), np.zeros(7), np.zeros(7))
    _track_joint_goal(p, np.full(7, 1.0))  # far goal: unconstrained q̈ would be large
    U_bar = np.full(p.nu, 0.2)
    U = p.solve(U_bar)
    assert U is not None
    assert np.all(np.abs(U) <= 1.0 + 1e-3)
    assert np.all(np.abs(U - U_bar) <= 0.3 + 1e-3)


@pytest.mark.parametrize('solver', SOLVERS)
def test_a_human_row_holds_in_the_solution(solver):
    p = _problem(solver=solver, trust_region=0.0)
    p.begin_cycle(np.zeros(7), np.zeros(7), np.zeros(7))
    goal = np.zeros(7)
    goal[0] = -0.1                   # the task pulls joint 1 negative...
    _track_joint_goal(p, goal)
    a = np.zeros((1, 7))
    a[0, 0] = 1.0                    # ...the human row wants q_N[0] ≥ 0
    d_bar = np.array([p.cfg.d_safe + p.cfg.delta])
    p.set_human_rows(p.n_steps, a, d_bar, np.zeros(1), np.zeros(7))
    U = p.solve(np.zeros(p.nu))
    assert U is not None
    q, _ = p.rollout(np.zeros(7), np.zeros(7), U)
    assert q[-1, 0] >= -2e-3
    # The diagnostics see one row in use, at its bound, held without slack
    d = p.diag
    assert d.solved and d.iterations > 0 and np.isfinite(d.cost)
    assert d.human_rows == 1 and d.human_binding == 1
    assert d.human_clearance == pytest.approx(0.0, abs=2e-3)
    assert d.max_slack < 1e-3


@pytest.mark.parametrize('terminal', ['soft', 'none'])
def test_cost_constant_stage_and_terminal_terms_add_up(terminal):
    """½zᵀHz + gᵀz + cost_constant is the whole cost (not up to a constant); the kept stage
    terms are step 1's share of it (q_1, q̇_1, u_0, u_0 − u_prev), the terminal ones w_terminal‖q̇_N‖²."""
    p = _problem(terminal=terminal)
    rng = np.random.default_rng(3)
    q0, qd0, u_prev, goal = (rng.normal(size=7) for _ in range(4))
    p.begin_cycle(q0, qd0, u_prev)
    _track_joint_goal(p, goal, weight=100.0)
    cfg = p.cfg
    for _ in range(3):
        U = rng.normal(size=p.nu)
        q, qd = p.rollout(q0, qd0, U)
        u = U.reshape(p.n_steps, 7)
        jerk = np.diff(np.vstack([u_prev, u]), axis=0) / p.dt
        terminal_cost = cfg.w_terminal * np.sum(qd[-1] ** 2) if terminal == 'soft' else 0.0
        full = (100.0 * np.sum((q - goal) ** 2) + cfg.w_velocity * np.sum(qd ** 2) + terminal_cost
                + cfg.w_accel * np.sum(u ** 2) + cfg.w_jerk * np.sum(jerk ** 2))
        stage = (100.0 * np.sum((q[0] - goal) ** 2) + cfg.w_velocity * np.sum(qd[0] ** 2)
                 + cfg.w_accel * np.sum(u[0] ** 2) + cfg.w_jerk * np.sum(jerk[0] ** 2))
        assert _quad(p.H, p.g, _z(p, U)) + p.cost_constant == pytest.approx(full, rel=1e-9)
        assert p._terms_cost(p._stage_terms, U) == pytest.approx(stage, rel=1e-9)
        assert p._terms_cost(p._terminal_terms, U) == pytest.approx(terminal_cost, rel=1e-9, abs=1e-12)


@pytest.mark.parametrize('solver', SOLVERS)
def test_diagnostics_report_the_cost_parts_at_the_solution(solver):
    p = _problem(solver=solver, q_lim=10.0, qdot_max=10.0, qddot_max=100.0)
    rng = np.random.default_rng(4)
    q0, qd0, u_prev = 0.1 * rng.normal(size=7), 0.2 * rng.normal(size=7), rng.normal(size=7)
    p.begin_cycle(q0, qd0, u_prev)
    _track_joint_goal(p, 0.2 * rng.normal(size=7))
    U = p.solve(np.zeros(p.nu))
    d = p.diag
    assert U is not None
    assert d.cost_full == pytest.approx(d.cost + p.cost_constant) and d.cost_full >= 0.0
    # Without human rows the slacks sit at 0 (OSQP: within its tolerance, times ρ)
    assert d.stage_cost == pytest.approx(p._terms_cost(p._stage_terms, U), abs=1e-2)
    assert d.terminal_cost == pytest.approx(p._terms_cost(p._terminal_terms, U))
    assert 0.0 < d.stage_cost < d.cost_full and 0.0 <= d.terminal_cost < d.cost_full
    # Nothing binds: every multiplier is 0 (OSQP: about)
    assert max(d.lambda_human, d.lambda_accel, d.lambda_position, d.lambda_velocity) < 1e-2


@pytest.mark.parametrize('solver, rho', [(s, 1e3) for s in SOLVERS]
                         + ([('daqp', 1e4)] if 'daqp' in SOLVERS else []))
def test_multipliers_of_a_held_and_of_a_slackened_human_row(solver, rho):
    """KKT of a slack (s ≥ 0, cost ρs + ½σs²): a row held without slack has λ < ρ, a row that
    needs slack has λ = ρ + σs. λ is the unscaled multiplier, in cost per metre.
    ρ = 1e4 (mpc.yaml) with DAQP only: OSQP's ADMM stalls there."""
    def solve(need, qddot_max):
        # The task pulls joint 1 to −0.1; the row wants q_N[0] ≥ need
        p = _problem(solver=solver, qddot_max=qddot_max, slack_linear=rho)
        p.begin_cycle(np.zeros(7), np.zeros(7), np.zeros(7))
        goal = np.zeros(7)
        goal[0] = -0.1
        _track_joint_goal(p, goal)
        a = np.zeros((1, 7))
        a[0, 0] = 1.0
        d_bar = np.array([p.cfg.d_safe + p.cfg.delta - need])
        p.set_human_rows(p.n_steps, a, d_bar, np.zeros(1), np.zeros(7))
        assert p.solve(np.zeros(p.nu)) is not None
        return p.cfg, p.diag

    cfg, held = solve(0.0, 5.0)
    assert held.max_slack < 1e-3 and held.human_binding == 1
    assert 10.0 < held.lambda_human < cfg.slack_linear
    assert held.lambda_position < 1e-2 and held.lambda_velocity < 1e-2   # limits far away
    # 0.5 rad is out of reach in 0.5 s with |q̈| ≤ 1: the row leans on its slack, q̈ on its box
    cfg, pushed = solve(0.5, 1.0)
    assert pushed.max_slack > 0.1 and pushed.lambda_accel > 1.0
    assert pushed.lambda_human == pytest.approx(cfg.slack_linear + cfg.slack_quadratic * pushed.max_slack,
                                                rel=1e-2)


@pytest.mark.parametrize('solver', SOLVERS)
def test_an_infeasible_cycle_returns_none_and_says_so(solver):
    # Stop in N steps from 1 rad/s, but q̈ may only move 0.1 around a plan of zeros
    p = _problem(solver=solver, terminal='hard', trust_region=0.1)
    p.begin_cycle(np.zeros(7), np.full(7, 1.0), np.zeros(7))
    assert p.solve(np.zeros(p.nu)) is None
    assert not p.diag.solved and p.diag.status
    assert np.isnan(p.diag.cost_full) and np.isnan(p.diag.lambda_human)


@pytest.mark.parametrize('solver', SOLVERS)
def test_second_cycle_matches_a_fresh_problem(solver):
    """OSQP update path: the pattern stays fixed and only the values change."""
    q0, qd0 = np.full(7, 0.1), np.full(7, -0.05)
    a = np.zeros((1, 7))
    a[0, 2] = 1.0                    # the row wants q_5[2] ≥ q0[2], the second goal pulls it down

    def cycle(p, goal, with_human):
        p.begin_cycle(q0, qd0, np.zeros(7))
        _track_joint_goal(p, goal)
        if with_human:
            d_bar = np.array([p.cfg.d_safe + p.cfg.delta])
            p.set_human_rows(5, a, d_bar, np.zeros(1), q0)
        return p.solve(np.zeros(p.nu))

    persistent = _problem(solver=solver, trust_region=0.0)
    cycle(persistent, np.full(7, 0.15), with_human=False)
    second = cycle(persistent, np.full(7, 0.05), with_human=True)
    fresh = cycle(_problem(solver=solver, trust_region=0.0), np.full(7, 0.05), with_human=True)
    assert second is not None and fresh is not None
    # Same optimum within OSQP's accuracy (eps 1e-4); a broken pattern is off by far more
    assert np.allclose(second, fresh, atol=2e-2 * max(1.0, np.abs(fresh).max()))


@pytest.mark.parametrize('solver', SOLVERS)
def test_closed_loop_reaches_a_joint_goal_and_stops(solver):
    p = _problem(solver=solver)
    goal = np.linspace(-0.3, 0.3, 7)
    q, qd, u_prev = np.zeros(7), np.zeros(7), np.zeros(7)
    U = np.zeros(p.nu)
    for _ in range(100):             # 5 s at dt = 0.05
        p.begin_cycle(q, qd, u_prev)
        _track_joint_goal(p, goal)
        U = p.solve(p.shift_plan(U))
        assert U is not None
        u_prev = p.first_input(U)
        x = p.A_dyn @ np.concatenate([q, qd]) + p.B_dyn @ u_prev
        q, qd = x[:7], x[7:]
    assert np.allclose(q, goal, atol=1e-2)
    assert np.allclose(qd, 0.0, atol=1e-2)


def test_mpc_yaml_sets_every_setting():
    """config/mpc.yaml is what mpc_commander loads: every MPCSettings field, valid values."""
    import dataclasses
    import os
    import yaml
    path = os.path.join(os.path.dirname(__file__), '..', 'config', 'mpc.yaml')
    with open(path) as f:
        params = {name: value for section in yaml.safe_load(f).values() for name, value in section.items()}
    fields = [field.name for field in dataclasses.fields(MPCSettings)]
    assert set(fields) <= set(params), set(fields) - set(params)
    settings = MPCSettings(**{name: params[name] for name in fields})
    ones = np.ones(NUM_JOINTS)
    MPCProblem(settings, -ones, ones, ones, ones)
    assert len(params['q_home']) == NUM_JOINTS
