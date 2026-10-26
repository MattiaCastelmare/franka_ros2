"""Condensed QP of the FR3 MPC: double integrator in joint space, input q̈.

Pure NumPy + OSQP, no ROS and no Pinocchio: the node computes the kinematics,
the human forecast and the distances, and passes numbers in. Notation and
formulas follow "MPC per la tesi — guida di studio", sections 7.2–7.6.

Horizon
    x_k = (q_k, q̇_k), u_k = q̈_k (k = 0..N-1). x_0 is measured, x_1..x_N are predicted.
        q_k = q_free,k  + G_k U,      q_free,k  = q_0 + k·dt·q̇_0
        q̇_k = qd_free,k + V_k U,      qd_free,k = q̇_0
    U = (u_0, ..., u_{N-1}) ∈ R^{7N}. G_k, V_k (7 × 7N) are the block rows k of
    Γ_q ⊗ I_7 and Γ_v ⊗ I_7.

Decision vector
    z = (U, s), with s ∈ R^{K·N} one slack per human row (K pairs per step, padded).

OSQP form
    min ½ zᵀ P z + qᵀ z   s.t.   l ≤ A z ≤ u

Row blocks of A, in this order and with a FIXED size, so that every cycle only
updates values (OSQP keeps its factorisation):
    'u'      I on U          |u_k| ≤ q̈_max, intersected with the trust region around Ū
    's'      I on s          s ≥ 0
    'q'      G_1..G_N        q_min + margin ≤ q_k ≤ q_max − margin
    'qd'     V_1..V_N        |q̇_k| ≤ qdot_scale · q̇_max
    'term'   V_N             q̇_N = 0                               (terminal == 'hard')
    'human'      a_jᵀ G_k | +1             d̄_j + a_jᵀ(q_k − q̄_k) ≥ d_safe + δ + m_j − s_jk
    'lookahead'  a_jᵀ(G_k + T V_k) | +1    the same + T·a_jᵀq̇_k       (human_lookahead_s T > 0)
The two human blocks share the slacks: together they hold d + T·min(ḣ, 0) ≥ d_safe + δ + m.

One cycle (the node calls them in this order):
    begin_cycle(q0, qd0, u_prev)
    add_tracking_term(k, C, r, W)          any number, k = 1..N
    set_human_rows(k, a, d_bar, margin, q_bar_k)   k = 1..N
    U = solve(U_bar)                       None if the QP was not solved
    diag                                   MPCDiagnostics of that solve

Solvers (MPCSettings.solver), both in the image: 'osqp' (ADMM, warm-started) or 'daqp'
(dense active set). On scripts/mpc_problem_demo.py, with several distance rows active
together, OSQP needs up to ~10000 iterations (100+ ms) where DAQP solves the same QPs
in ~1.5 ms: config/mpc.yaml uses 'daqp'.
"""

from dataclasses import dataclass
from typing import Optional, Tuple

import daqp
import numpy as np
import osqp
from scipy import sparse

from franka_experiments.utils.constants import NUM_JOINTS


# Setting names of osqp 0.6 (the Dockerfile's); osqp 1.x accepts them with a deprecation warning.
# A cold start can need ~1000 iterations, a warm-started cycle far fewer.
OSQP_SETTINGS = dict(warm_start=True, polish=False, verbose=False,
                     eps_abs=1e-4, eps_rel=1e-4, max_iter=2000)
OSQP_SOLVED = (1, 2)  # status_val of 'solved' and 'solved inaccurate', in osqp 0.6 and 1.x
DAQP_INF = 1e30       # DAQP's infinite bound
DAQP_EQUALITY = 5     # DAQP sense of a row with l = u
DAQP_STATUS = {1: 'solved', 2: 'solved soft', -1: 'primal infeasible', -2: 'cycling',
               -3: 'unbounded', -4: 'iteration limit', -5: 'nonconvex', -6: 'overdetermined'}
BINDING_TOL = 1e-3    # [m] a human row within this of its bound counts as binding


@dataclass
class MPCSettings:
    """Starting values from the guide (section 8); the weights are to tune on the replays."""
    dt: float = 0.05              # [s]
    n_steps: int = 10             # N: horizon N·dt = 0.5 s
    pairs_per_step: int = 4       # K human rows per step, fixed (unused rows are padded)

    # Safety: same d_safe as the CBF, delta on top so the shield rarely has to act
    d_safe: float = 0.15          # [m]
    delta: float = 0.05           # [m]
    # T [s]: next to d_k ≥ d_safe + δ + m, a second row d_k + T·ḣ_k ≥ the same (ḣ_k = a·q̇_k, the
    # robot's own approach speed): the robot slows down T·|ḣ| before the distance, as the shield's
    # HOCBF does with T = k1_cbf/k0_cbf. Only tighter, never looser. 0 = the position rows only
    human_lookahead_s: float = 0.0
    # rho must stay above the multipliers of the human rows (~450 with w_position 1000:
    # they grow with the task weights) for the penalty to be exact. 1e4 is not safer:
    # it makes OSQP's ADMM stall (4000 iterations without converging on a plain case).
    slack_linear: float = 1e3     # rho: exact L1 penalty
    slack_quadratic: float = 1e3  # sigma: keeps the slack block strictly convex

    # Limits
    qdot_scale: float = 0.7       # |q̇_k| ≤ qdot_scale · q̇_max (headroom for the shield)
    joint_margin: float = 0.05    # [rad]
    terminal: str = 'soft'        # 'soft' (w_terminal on q̇_N) | 'hard' (q̇_N = 0) | 'none'
    # [rad/s²] |U − Ū|∞ around the shifted plan, 0 = off. Off by default: it also caps the
    # braking the relaxed limit rows count on, and on the demo it made a QP infeasible.
    trust_region: float = 0.0
    solver: str = 'osqp'          # 'osqp' | 'daqp' (see the module docstring)

    # Task weights (used by the caller to build the tracking terms)
    w_position: float = 1000.0
    w_orientation: float = 300.0
    w_posture: float = 1.0

    # Weights of the terms built here
    w_velocity: float = 0.1       # ‖q̇_k‖²
    w_accel: float = 0.01         # ‖u_k‖²
    w_jerk: float = 1e-4          # ‖(u_k − u_{k−1}) / dt‖²
    w_terminal: float = 10.0      # ‖q̇_N‖² with terminal == 'soft'


@dataclass
class MPCDiagnostics:
    """What one solve did: published as franka_msgs/MpcDiagnostics (human_utils.mpc_diagnostics_msg)
    for the CSV, summarised once a second in the terminal (logging_utils.format_mpc_summary)."""
    status: str = ''
    solved: bool = False
    iterations: int = 0
    solve_time_ms: float = float('nan')
    cost: float = float('nan')            # ½zᵀPz + qᵀz at the solution (no constant term)
    human_rows: int = 0                   # human rows in use this cycle (the others are padding)
    human_binding: int = 0                # of those, at their bound or violated
    human_clearance: float = float('nan') # [m] min over rows of d_lin + T·ḣ − (d_safe + δ + m); < 0 = slack used
    max_slack: float = 0.0                # [m]
    accel_saturation: float = 0.0         # fraction of the planned q̈ at the q̈ box
    # For the a-posteriori checks (analysis/mpc_analysis.py): the optimal cost V and two of its parts
    cost_full: float = float('nan')       # V = cost + the constant term (≥ 0): comparable across cycles
    stage_cost: float = float('nan')      # ℓ(x_0, u_0): the terms of step 1 (q_1, q̇_1, u_0, u_0 − u_prev, s_1)
    terminal_cost: float = float('nan')   # w_terminal·‖q̇_N‖² (terminal 'soft'), included in cost_full
    # Largest |multiplier| per row family: how much V would drop per unit the bound gave way
    lambda_human: float = float('nan')    # [cost/m] unscaled: the L1 penalty is exact while < slack_linear
    lambda_accel: float = float('nan')    # [cost/(rad/s²)] q̈ box (and trust region)
    lambda_position: float = float('nan') # [cost/rad] joint position limits
    lambda_velocity: float = float('nan') # [cost/(rad/s)] joint velocity limits


class MPCProblem:
    """Builds and solves the MPC QP of one cycle (real-time iteration: one QP per cycle)."""

    def __init__(self, settings: MPCSettings, q_min, q_max, qdot_max, qddot_max):
        self.cfg = settings
        self.dt = float(settings.dt)
        self.n_steps = int(settings.n_steps)
        self.n_joints = NUM_JOINTS
        self.n_pairs = int(settings.pairs_per_step)
        if settings.terminal not in ('soft', 'hard', 'none'):
            raise ValueError("terminal must be 'soft', 'hard' or 'none'")
        if settings.solver not in ('osqp', 'daqp'):
            raise ValueError("solver must be 'osqp' or 'daqp'")

        self.q_min = self._joint_vector(q_min, 'q_min')
        self.q_max = self._joint_vector(q_max, 'q_max')
        self.qdot_max = self._joint_vector(qdot_max, 'qdot_max')
        self.qddot_max = self._joint_vector(qddot_max, 'qddot_max')

        # Sizes and column layout of z = (U, s)
        N, n, K = self.n_steps, self.n_joints, self.n_pairs
        self.nu = n * N
        self.ns = K * N
        self.nz = self.nu + self.ns
        self.iU = slice(0, self.nu)
        self.iS = slice(self.nu, self.nz)

        self.A_dyn, self.B_dyn = self.get_system_matrices()
        # G[k-1], V[k-1] map U to q_k, q̇_k (k = 1..N), each (7, 7N)
        self.G, self.V = self._build_prediction_matrices()
        # D U − (u_prev, 0, ..., 0) = (u_0 − u_prev, u_1 − u_0, ...)
        self.D = np.eye(self.nu) - np.eye(self.nu, k=-n)
        self.rows = self._row_layout()
        self.human_blocks = [b for b in ('human', 'lookahead') if b in self.rows]
        self.n_rows = max(r.stop for r in self.rows.values())
        self._A_pattern, self._P_pattern = self._build_patterns()

        # Per-cycle data, reset by begin_cycle and filled by the set_/add_ methods
        self.q0 = np.zeros(n)
        self.qd0 = np.zeros(n)
        self.u_prev = np.zeros(n)
        self.q_free = np.zeros((N, n))
        self.qd_free = np.zeros((N, n))
        self.H = np.zeros((self.nz, self.nz))     # full symmetric Hessian, P = triu(H)
        self.g = np.zeros(self.nz)
        self.A = np.zeros((self.n_rows, self.nz))  # dense here, sparse only for OSQP
        self.l = np.zeros(self.n_rows)
        self.u = np.zeros(self.n_rows)
        self.cost_constant = 0.0                  # Σ cᵀWc of the cost terms: the solver does without it
        self._stage_terms = []                    # (M, c, W) of step 1's terms, see _add_quadratic
        self._terminal_terms = []

        self._human_scale = np.ones((len(self.human_blocks), self.ns))   # row scaling, see set_human_rows
        self._osqp = None                         # osqp.OSQP, created on the first solve
        self.diag = MPCDiagnostics()

    @property
    def status(self) -> str:
        return self.diag.status

    @property
    def solve_time_ms(self) -> float:
        return self.diag.solve_time_ms

    # ── Dynamics ────────────────────────────────────────────────────────────

    def _joint_vector(self, value, name: str) -> np.ndarray:
        v = np.asarray(value, dtype=np.float64).ravel()
        if v.shape != (self.n_joints,):
            raise ValueError(f'{name} must have {self.n_joints} entries, got {v.shape}')
        return v

    def get_system_matrices(self) -> Tuple[np.ndarray, np.ndarray]:
        """Exact zero-order-hold discretisation of q̈ = u: x_{k+1} = A x_k + B u_k."""
        n, dt = self.n_joints, self.dt
        I, Z = np.eye(n), np.zeros((n, n))
        A = np.block([[I, dt * I], [Z, I]])
        B = np.vstack([0.5 * dt ** 2 * I, dt * I])
        return A, B

    def rollout(self, q0, qd0, U) -> Tuple[np.ndarray, np.ndarray]:
        """Step-by-step simulation of the horizon: (q_1..q_N, q̇_1..q̇_N), each (N, 7).

        It does not use G and V, so it is the reference to test them against
        (and the way to get q̄_k from a shifted plan Ū).
        """
        n = self.n_joints
        U = np.asarray(U, dtype=np.float64).reshape(self.n_steps, n)
        x = np.concatenate([np.asarray(q0, dtype=np.float64), np.asarray(qd0, dtype=np.float64)])
        q = np.empty((self.n_steps, n))
        qd = np.empty((self.n_steps, n))
        for k in range(self.n_steps):
            x = self.A_dyn @ x + self.B_dyn @ U[k]
            q[k], qd[k] = x[:n], x[n:]
        return q, qd

    def _build_prediction_matrices(self) -> Tuple[np.ndarray, np.ndarray]:
        """G, V of shape (N, 7, 7N): q_k = q_free,k + G[k-1] U, q̇_k = qd_free,k + V[k-1] U."""
        G = np.zeros((self.n_steps, self.n_joints, self.nu))
        V = np.zeros((self.n_steps, self.n_joints, self.nu))
        for k in range(self.n_steps):
            step = k + 1  # G[k], V[k] map U to q_{k+1}, q̇_{k+1}
            for i in range(self.n_steps):
                if i < step:
                    G[k, :, self.n_joints*i:self.n_joints*(i+1)] = self.dt ** 2 * (step-i-0.5) * np.eye(self.n_joints)
                    V[k, :, self.n_joints*i:self.n_joints*(i+1)] = self.dt * np.eye(self.n_joints)
        return G, V

    def free_response(self, q0, qd0) -> Tuple[np.ndarray, np.ndarray]:
        """(q_free, qd_free), each (N, 7): the horizon with U = 0."""
        q_free = np.zeros((self.n_steps, self.n_joints))
        qd_free = np.zeros((self.n_steps, self.n_joints))
        for k in range(self.n_steps):
            step = k + 1  # q_free[k], qd_free[k] are the free response at step k+1
            q_free[k] = q0 + step * self.dt * qd0
            qd_free[k] = qd0
        return q_free, qd_free

    def shift_plan(self, U) -> np.ndarray:
        """Previous solution moved one step forward: Ū = (u_1, ..., u_{N-1}, u_{N-1})."""
        U = np.asarray(U, dtype=np.float64).reshape(self.n_steps, self.n_joints)
        return np.vstack([U[1:], U[-1:]]).ravel()

    def _row_layout(self) -> dict:
        """Row slices of A, in the order of the module docstring."""
        N, n = self.n_steps, self.n_joints
        sizes = [('u', self.nu), ('s', self.ns), ('q', n * N), ('qd', n * N)]
        if self.cfg.terminal == 'hard':
            sizes.append(('term', n))
        sizes.append(('human', self.ns))
        if self.cfg.human_lookahead_s > 0.0:
            sizes.append(('lookahead', self.ns))
        rows, start = {}, 0
        for name, size in sizes:
            rows[name] = slice(start, start + size)
            start += size
        return rows

    def _build_patterns(self):
        """CSC sparsity of A and of P = triu(H): every entry that can EVER be non-zero.

        Built once. OSQP keeps its factorisation only if the pattern never changes,
        and scipy drops explicit zeros, so the values are always written through
        these patterns (csc_values), never through a fresh sparse matrix.
        """
        R, N, K = self.rows, self.n_steps, self.n_pairs
        a_mask = np.zeros((self.n_rows, self.nz), dtype=bool)
        a_mask[R['u'], self.iU] = np.eye(self.nu, dtype=bool)
        a_mask[R['s'], self.iS] = np.eye(self.ns, dtype=bool)
        a_mask[R['q'], self.iU] = self.G.reshape(-1, self.nu) != 0
        a_mask[R['qd'], self.iU] = self.V.reshape(-1, self.nu) != 0
        if 'term' in R:
            a_mask[R['term'], self.iU] = self.V[-1] != 0
        for block in self.human_blocks:
            a_mask[R[block], self.iS] = np.eye(self.ns, dtype=bool)
            for k in range(N):
                # A human row of step k+1 depends on the inputs before it, like q_{k+1} and q̇_{k+1}
                inputs = self.G[k].any(axis=0)
                a_mask[R[block].start + k * K:R[block].start + (k + 1) * K, self.iU] = inputs

        p_mask = np.zeros((self.nz, self.nz), dtype=bool)
        p_mask[self.iU, self.iU] = True
        p_mask[self.iS, self.iS] = np.eye(self.ns, dtype=bool)
        return csc_pattern(a_mask), csc_pattern(np.triu(p_mask))

    # ── One cycle ───────────────────────────────────────────────────────────

    def begin_cycle(self, q0, qd0, u_prev) -> None:
        """Start a new QP from the state where the plan starts and the q̈ in force then (u_prev:
        the jerk term's starting point)."""
        self.q0 = self._joint_vector(q0, 'q0')
        self.qd0 = self._joint_vector(qd0, 'qd0')
        self.u_prev = self._joint_vector(u_prev, 'u_prev')
        self.q_free, self.qd_free = self.free_response(self.q0, self.qd0)
        self.H[:] = 0.0
        self.g[:] = 0.0
        self.A[:] = 0.0
        self.cost_constant = 0.0
        self._stage_terms, self._terminal_terms = [], []
        self._add_regularisation()
        self._set_limit_rows()
        self._reset_human_rows()

    def _add_quadratic(self, M, c, W, part: Optional[list] = None) -> None:
        """Add ‖M U - c‖²_W to the cost: H[U,U] += 2 MᵀWM, g[U] -= 2 MᵀWc, cost_constant += cᵀWc.
        W: matrix or scalar. part: a list that also keeps (M, c, W), so the diagnostics can
        evaluate that share of the cost at the solution (stage and terminal cost)."""
        MtW = W * M.T if np.ndim(W) == 0 else M.T @ W
        self.H[self.iU, self.iU] += 2.0 * MtW @ M
        self.g[self.iU] -= 2.0 * MtW @ c
        self.cost_constant += float(c @ (W * c if np.ndim(W) == 0 else W @ c))
        if part is not None:
            part.append((M, c, W))

    def add_tracking_term(self, k: int, C, r, W) -> None:
        """Add ‖C·q_k - r‖²_W to the cost, k = 1..N.

        Every task term the node needs has this form:
            joint target       C = I_7,  r = q_ref,k
            posture            C = I_7,  r = q_home,             W = w_posture·I
            TCP position       C = J_v(q̄_k),  r = p_ref,k - p(q̄_k) + J_v(q̄_k) q̄_k
            TCP orientation    C = -J_ω(q̄_k), r = -e_R(q̄_k) - J_ω(q̄_k) q̄_k,
                               with e_R = log(R_d R(q̄_k)ᵀ)^v as in the commander
        (the last two are the Gauss-Newton linearisation around q̄_k).

        With q_k = q_free,k + G_k U the residual is (C G_k) U - (r - C q_free,k).
        H stays constant only with C constant (joint target, posture); with the TCP
        terms it changes every cycle, which solve() handles (update of Px).
        """
        if not 1 <= k <= self.n_steps:
            raise IndexError(f'step k must be in 1..{self.n_steps}, got {k}')
        C = np.atleast_2d(np.asarray(C, dtype=np.float64))
        r = np.asarray(r, dtype=np.float64)
        self._add_quadratic(C @ self.G[k - 1], r - C @ self.q_free[k - 1], np.asarray(W, dtype=np.float64),
                            self._stage_terms if k == 1 else None)

    def _add_regularisation(self) -> None:
        """Velocity, effort, jerk, soft terminal stop and slack penalties.

            ‖q̇_k‖² w_velocity, k = 1..N        with q̇_k = qd_free,k + V_k U
            ‖q̇_N‖² w_terminal                  only with terminal == 'soft'
            ‖u_k‖² w_accel
            ‖(DU - c)/dt‖² w_jerk               c = (u_prev, 0, ..., 0)
            ρ·1ᵀs + ½σ‖s‖²
        With w_accel > 0 the Hessian is positive definite: the solution is unique.
        """
        cfg, n = self.cfg, self.n_joints
        for k in range(self.n_steps):
            self._add_quadratic(self.V[k], -self.qd_free[k], cfg.w_velocity, self._stage_terms if k == 0 else None)
        if cfg.terminal == 'soft':
            self._add_quadratic(self.V[-1], -self.qd_free[-1], cfg.w_terminal, self._terminal_terms)
        self.H[self.iU, self.iU] += 2.0 * cfg.w_accel * np.eye(self.nu)
        c = np.zeros(self.nu)
        c[:n] = self.u_prev
        self._add_quadratic(self.D, c, cfg.w_jerk / self.dt ** 2)
        # Step 1's share of the two terms that span all of U: u_0 and u_0 − u_prev
        self._stage_terms += [(np.eye(n, self.nu), np.zeros(n), cfg.w_accel),
                              (self.D[:n], c[:n], cfg.w_jerk / self.dt ** 2)]
        self.H[self.iS, self.iS] += cfg.slack_quadratic * np.eye(self.ns)
        self.g[self.iS] += cfg.slack_linear

    def _set_limit_rows(self) -> None:
        """Rows 'u', 's', 'q', 'qd' (and 'term') of A with their bounds; 'u' is bounded in solve().
            's'    0 ≤ s ≤ +inf
            'q'    q_min + margin - q_free,k  ≤ G_k U ≤  q_max - margin - q_free,k
            'qd'   -v_max,k - qd_free,k       ≤ V_k U ≤  v_max,k - qd_free,k
            'term' V_N U = -qd_free,N   (l = u)
        Two relaxations keep the box non-empty from any state:
          - a free motion already past a position limit only has to get no worse:
            the bound is clipped at 0 (as utils/cbf_qp.py does);
          - with |q̇_0| above qdot_scale·q̇_max, q̇_k can only come down by k·dt·q̈_max:
            v_max,k = max(qdot_scale·q̇_max, |q̇_0| - k·dt·q̈_max).
        """
        cfg, R, N, n = self.cfg, self.rows, self.n_steps, self.n_joints
        self.A[R['u'], self.iU] = np.eye(self.nu)
        self.A[R['s'], self.iS] = np.eye(self.ns)
        self.l[R['s']], self.u[R['s']] = 0.0, np.inf

        q_free = self.q_free.ravel()
        margin = cfg.joint_margin
        self.A[R['q'], self.iU] = self.G.reshape(N * n, self.nu)
        self.l[R['q']] = np.minimum(np.tile(self.q_min + margin, N) - q_free, 0.0)
        self.u[R['q']] = np.maximum(np.tile(self.q_max - margin, N) - q_free, 0.0)

        steps = np.arange(1, N + 1)[:, None]
        reachable = np.abs(self.qd0) - steps * self.dt * self.qddot_max
        v_max = np.maximum(cfg.qdot_scale * self.qdot_max, reachable).ravel()
        qd_free = self.qd_free.ravel()
        self.A[R['qd'], self.iU] = self.V.reshape(N * n, self.nu)
        self.l[R['qd']] = -v_max - qd_free
        self.u[R['qd']] = v_max - qd_free

        if 'term' in R:
            self.A[R['term'], self.iU] = self.V[-1]
            self.l[R['term']] = self.u[R['term']] = -self.qd_free[-1]

    def _reset_human_rows(self) -> None:
        """Every human row starts inert: a = 0, lower bound -inf, the slack coefficient kept."""
        for block in self.human_blocks:
            R = self.rows[block]
            self.A[R, self.iU] = 0.0
            self.A[R, self.iS] = np.eye(self.ns)
            self.l[R], self.u[R] = -np.inf, np.inf
        self._human_scale[:] = 1.0

    def set_human_rows(self, k: int, a, d_bar, margin, q_bar_k) -> None:
        """Up to K distance rows of step k (k = 1..N), linearised around q̄_k.

        a       (m, 7)  n̂ᵀ J_c(q̄_k) of each robot-point / human-capsule pair
        d_bar   (m,)    distance at q̄_k from the human capsule predicted at τ_k
        margin  (m,)    m(τ_k) of each pair (prediction uncertainty)
        q_bar_k (7,)    linearisation point (the shifted plan)
        with m ≤ K; the caller keeps the K pairs with the smallest d_bar.

        Pair j holds d_k ≥ d_safe + δ + m_j ('human' block) and, with T = human_lookahead_s > 0,
        d_k + T·ḣ_k ≥ d_safe + δ + m_j ('lookahead' block), with the linearised distance
        d_k = d̄_j + a_j·(q_k − q̄_k) and ḣ_k = a_j·q̇_k, the robot's own approach speed (as the
        shield's ḣ: the human's motion is in d̄_j). Both rows have index (k-1)·K + j in their
        block and share the slack s_jk (t = 0 in the 'human' block, T in 'lookahead'):
            A[row, U] = a_j (G_k + t V_k),   A[row, s_jk] = 1
            l[row]    = d_safe + δ + m_j - d̄_j + a_j·(q̄_k - q_free,k) - t a_j·qd_free,k,   u[row] = +inf
        The whole row is then divided by ‖a_j (G_k + t V_k)‖: the same constraint, with the slack
        still in metres. Unscaled, the U entries (dt²-sized, 1e-3 at k = 1) next to the
        slack's 1 stalled OSQP for thousands of iterations (scripts/mpc_problem_demo.py).
        """
        if not 1 <= k <= self.n_steps:
            raise IndexError(f'step k must be in 1..{self.n_steps}, got {k}')
        a = np.atleast_2d(np.asarray(a, dtype=np.float64))
        if a.shape[0] > self.n_pairs:
            raise ValueError(f'at most {self.n_pairs} pairs per step, got {a.shape[0]}')
        cfg, first = self.cfg, (k - 1) * self.n_pairs
        offset = self._joint_vector(q_bar_k, 'q_bar_k') - self.q_free[k - 1]
        for b, block in enumerate(self.human_blocks):
            t = cfg.human_lookahead_s if block == 'lookahead' else 0.0
            for j, (a_j, d_j, m_j) in enumerate(zip(a, np.ravel(d_bar), np.ravel(margin))):
                row = self.rows[block].start + first + j
                row_u = a_j @ (self.G[k - 1] + t * self.V[k - 1])
                norm = float(np.linalg.norm(row_u))
                scale = 1.0 / norm if norm > 1e-12 else 1.0
                self.A[row, self.iU] = scale * row_u
                self.A[row, self.nu + first + j] = scale
                self.l[row] = scale * (cfg.d_safe + cfg.delta + m_j - d_j + a_j @ offset
                                       - t * a_j @ self.qd_free[k - 1])
                self.u[row] = np.inf
                self._human_scale[b, first + j] = scale

    # ── Solve ───────────────────────────────────────────────────────────────

    def solve(self, U_bar) -> Optional[np.ndarray]:
        """Solve the QP of this cycle; U (7N,) or None if it was not solved.

        U_bar is the shifted previous plan: the centre of the trust region and
        OSQP's warm start (DAQP starts from scratch, it is fast enough).
        """
        U_bar = np.asarray(U_bar, dtype=np.float64).ravel()
        R = self.rows['u']
        bound = np.tile(self.qddot_max, self.n_steps)
        lo, hi = -bound, bound
        if self.cfg.trust_region > 0.0:
            lo = np.maximum(lo, U_bar - self.cfg.trust_region)
            hi = np.minimum(hi, U_bar + self.cfg.trust_region)
        self.l[R], self.u[R] = lo, hi

        solve = self._solve_daqp if self.cfg.solver == 'daqp' else self._solve_osqp
        x, duals, status, solved, iterations, solve_time = solve(U_bar)
        solved = solved and x is not None and bool(np.all(np.isfinite(x)))
        self.diag = self._diagnose(x, duals, status, solved, iterations, solve_time)
        return x[:self.nu].copy() if solved else None

    def _solve_osqp(self, U_bar):
        """Set OSQP up once on the fixed patterns, then only update values and warm-start."""
        Ax = csc_values(self._A_pattern, self.A)
        Px = csc_values(self._P_pattern, self.H)
        if self._osqp is None:
            self._osqp = osqp.OSQP()
            self._osqp.setup(P=with_values(self._P_pattern, Px), q=self.g,
                             A=with_values(self._A_pattern, Ax), l=self.l, u=self.u,
                             **OSQP_SETTINGS)
        else:
            self._osqp.update(q=self.g, l=self.l, u=self.u, Px=Px, Ax=Ax)
        self._osqp.warm_start(x=np.concatenate([U_bar, np.zeros(self.ns)]))
        res = self._osqp.solve()
        return (res.x, res.y, str(res.info.status), res.info.status_val in OSQP_SOLVED,
                int(res.info.iter), float(res.info.solve_time))

    def _solve_daqp(self, U_bar):
        """Dense active set on the same H, g, A, l, u (rows with l = u as equalities)."""
        lower, upper = np.maximum(self.l, -DAQP_INF), np.minimum(self.u, DAQP_INF)
        sense = np.where(lower == upper, DAQP_EQUALITY, 0).astype(np.int32)
        x, _, flag, info = daqp.solve(self.H, self.g, self.A, upper, lower, sense)
        return (x, info.get('lam'), DAQP_STATUS.get(flag, f'exitflag {flag}'), flag in (1, 2),
                int(info['iterations']), float(info['solve_time']))

    def _diagnose(self, x, duals, status: str, solved: bool, iterations: int,
                  solve_time: float) -> MPCDiagnostics:
        """duals: one multiplier per row of A (sign: + at the upper bound, − at the lower)."""
        diag = MPCDiagnostics(status=status, solved=solved, iterations=iterations,
                              solve_time_ms=1e3 * solve_time)
        if not solved:
            return diag
        cfg, U, s = self.cfg, x[self.iU], x[self.iS]
        diag.cost = float(0.5 * x @ self.H @ x + self.g @ x)
        diag.cost_full = diag.cost + self.cost_constant
        s_1 = s[:self.n_pairs]
        diag.stage_cost = (self._terms_cost(self._stage_terms, U)
                           + float(cfg.slack_linear * s_1.sum() + 0.5 * cfg.slack_quadratic * s_1 @ s_1))
        diag.terminal_cost = self._terms_cost(self._terminal_terms, U)
        if duals is not None and len(duals) == self.n_rows and np.all(np.isfinite(duals)):
            lam = np.abs(np.asarray(duals, dtype=np.float64))
            diag.lambda_accel = float(lam[self.rows['u']].max())
            diag.lambda_position = float(lam[self.rows['q']].max())
            diag.lambda_velocity = float(lam[self.rows['qd']].max())
            # A human row was divided by its norm (scale): its multiplier per metre is λ·scale.
            # The two rows of a pair share a slack: their multipliers add up against ρ
            diag.lambda_human = float(sum(lam[self.rows[block]] * self._human_scale[b]
                                          for b, block in enumerate(self.human_blocks)).max())
        in_use = np.isfinite(self.l[self.rows['human']])
        diag.human_rows = int(in_use.sum())
        if diag.human_rows:
            # Per pair, the tighter of its rows: (lookahead) distance minus what the row asks
            # for, without the slack's help [m]
            clearance = np.min([(self.A[self.rows[block], self.iU] @ x[self.iU] - self.l[self.rows[block]])
                                / self._human_scale[b] for b, block in enumerate(self.human_blocks)], axis=0)[in_use]
            diag.human_binding = int(np.sum(clearance <= BINDING_TOL))
            diag.human_clearance = float(clearance.min())
        diag.max_slack = float(max(0.0, x[self.iS].max())) if self.ns else 0.0
        at_box = np.abs(x[self.iU]) >= np.tile(self.qddot_max, self.n_steps) - 1e-3
        diag.accel_saturation = float(at_box.mean())
        return diag

    @staticmethod
    def _terms_cost(terms, U) -> float:
        """Σ ‖M U − c‖²_W over the (M, c, W) kept by _add_quadratic."""
        total = 0.0
        for M, c, W in terms:
            e = M @ U - c
            total += float(e @ (W * e if np.ndim(W) == 0 else W @ e))
        return total

    def first_input(self, U) -> np.ndarray:
        """u_0 of a solution, the input to apply now."""
        return np.asarray(U, dtype=np.float64)[:self.n_joints].copy()


# ── Fixed-pattern CSC matrices ──────────────────────────────────────────────

def csc_pattern(mask: np.ndarray):
    """(pattern, rows, cols): a CSC matrix of the mask and the (row, col) of each stored entry."""
    pattern = sparse.csc_matrix(mask.astype(np.float64))
    pattern.sort_indices()
    cols = np.repeat(np.arange(mask.shape[1]), np.diff(pattern.indptr))
    return pattern, pattern.indices.copy(), cols


def csc_values(pattern, dense: np.ndarray) -> np.ndarray:
    """The entries of a dense matrix at the pattern's positions, in CSC order."""
    _, rows, cols = pattern
    return dense[rows, cols]


def with_values(pattern, values: np.ndarray) -> sparse.csc_matrix:
    """A copy of the pattern holding these values (zeros stay stored)."""
    matrix, _, _ = pattern
    return sparse.csc_matrix((values, matrix.indices, matrix.indptr), shape=matrix.shape)