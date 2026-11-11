"""Shared helpers for the Stage 4 verification scripts (chapters 08 and 09). Read-only.

Re-uses the Stage 3 helpers:
  * `../stage3_verification/_common3.py` — effective `cbf_safety_filter` parameters per launch
    configuration (launch helpers executed via `ast`), on top of `../stage2_verification/_common.py`
    (paths, the source-import guard, the PASS/FAIL `Report`);
  * `../stage3_verification/_builder3.py` (container only) — the real `ConstraintBuilder` with the
    real `CBFKinematics`, wired as `cbf_safety_filter.__init__` wires it.

Adds `QPHarness`: the QP of `cbf_safety_filter`, built ONLY from the production functions
(`build_qp_objective`, `build_osqp_A`, `build_osqp_bounds`, `pad_rows_to_block`, `build_row_rhs`,
`accept_iterate`, `position_velocity_accel_box`, `apply_slew_limit`) with the same OSQP `setup`
arguments as `_solve` (L1684–1708 at the baseline). The node itself needs a ROS graph, so the
sequence of calls in `_qp_tick`/`_solve` is transcribed here; every numeric formula comes from the
production module.
"""
import os
import sys

sys.dont_write_bytecode = True
HERE4 = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE4, '..', 'stage3_verification'))
import _common3 as C3  # noqa: E402

C = C3.C
C.use_source_package()
import numpy as np  # noqa: E402

FILTER_SRC = os.path.join(C.PKG_SRC, 'franka_experiments', 'nodes', 'cbf_safety_filter.py')
RT_SRC = os.path.join(C.REPO, 'franka_rt_controllers', 'src')


def static_limits(P):
    """acc box and brake authority exactly as cbf_safety_filter.__init__ L152–168 derives them."""
    from franka_experiments.utils.cbf_state_rows import FR3_JOINT_KEYS
    from franka_experiments.utils.config import load_franka_joint_limits
    from franka_experiments.utils.cbf_hard_limits import FR3_VEL_Q_REF_LOWER, FR3_VEL_Q_REF_UPPER
    jl = load_franka_joint_limits(FR3_JOINT_KEYS)
    qdd_cap = np.minimum(jl['decel_max'], P.qddot_max_abs)
    brake = qdd_cap.copy()
    acc = getattr(P, 'qddot_accel_limits', None)
    if acc:
        acc = np.asarray(acc, dtype=np.float64).reshape(-1)
        qdd_cap = np.minimum(acc, P.qddot_max_abs)
        brake = np.minimum(brake, qdd_cap)
    return dict(lb=-qdd_cap, ub=qdd_cap, brake=brake, qdot_max=jl['qdot_max'],
                q_min=np.maximum(jl['q_min'], FR3_VEL_Q_REF_LOWER),
                q_max=np.minimum(jl['q_max'], FR3_VEL_Q_REF_UPPER), jl=jl)


class QPHarness:
    """The filter's QP from production functions (needs `osqp`; run where the deployed version is)."""

    def __init__(self, P, n_slot=None):
        from franka_experiments.utils.cbf_qp_assembly import build_qp_objective
        from franka_experiments.utils.cbf_state_rows import (
            G_CAP, G_OBS, G_QLIM, G_SC, G_SING, G_SPD, G_SSB, N_SLACK, NV, NX, row_slack_slots)
        self.P = P
        self.NV, self.NX, self.N_SLACK = NV, NX, N_SLACK
        self.n_slot = row_slack_slots(P) if n_slot is None else n_slot
        self.n_slack = N_SLACK + self.n_slot
        self.nx = NX + self.n_slot
        self.rho = {G_OBS: P.rho_slack, G_SC: P.rho_slack_self_collision,
                    G_QLIM: P.rho_slack_joint_limit, G_SING: P.rho_slack_singularity,
                    G_CAP: P.rho_slack_retreat, G_SPD: P.rho_slack_link_speed,
                    G_SSB: P.rho_slack_ssb}
        self.P_csc, self.qvec = build_qp_objective(
            NV, N_SLACK, self.rho, n_slot=self.n_slot, rho_l1=P.rho_slack_l1,
            eps=P.obstacle_row_slack_eps)
        L = static_limits(P)
        self.lb, self.ub = L['lb'], L['ub']
        self.box_lb = np.concatenate([self.lb, np.zeros(self.n_slack)])
        self.box_ub = np.concatenate([self.ub, np.full(self.n_slack, 1e6)])
        self.box_ub[NV + G_SSB] = float(P.ssb_slack_max)
        self.box_ub[NV + G_OBS] = float(P.obstacle_slack_max)
        self.L = L

    def solve(self, G, h_qp, qddot_nom, *, box_lb=None, box_ub=None, block=None, osqp_kw=None):
        """One `_solve` call: pad, bounds, fresh OSQP setup (as on a row-count change), solve."""
        import osqp
        from franka_experiments.utils.cbf_qp_assembly import (
            accept_iterate, build_osqp_A, build_osqp_bounds, pad_rows_to_block)
        qvec = self.qvec.copy()
        qvec[:self.NV] = -np.asarray(qddot_nom, dtype=np.float64)
        blb = self.box_lb.copy() if box_lb is None else box_lb
        bub = self.box_ub.copy() if box_ub is None else box_ub
        Gp, hp = pad_rows_to_block(G, h_qp, self.P.qp_row_block if block is None else block)
        l, u = build_osqp_bounds(Gp, hp, blb, bub)
        prob = osqp.OSQP()
        tol = ({'eps_rel': float(self.P.obstacle_row_osqp_eps_rel)} if self.n_slot else {})
        tol['max_iter'] = self.P.osqp_max_iter
        tol.update(osqp_kw or {})          # a test may override the production settings explicitly
        prob.setup(P=self.P_csc, q=qvec, A=build_osqp_A(Gp, self.NV, self.n_slack),
                   l=l, u=u, warm_start=True, verbose=False, **tol)
        res = prob.solve()
        self.last_exc = None
        try:                               # _solve L1683–1712 wraps the same calls in try/except
            x = accept_iterate(res.x, res.info.status_val, blb, bub, self.NV,
                               solved=osqp.constant('OSQP_SOLVED'),
                               inaccurate=(osqp.constant('OSQP_SOLVED_INACCURATE')
                                           if self.P.accept_inaccurate_qp else -999))
        except Exception as exc:           # noqa: BLE001 — mirrors the node
            self.last_exc = exc
            x = None
        return x, res, prob, qvec


def independent_qp(Pd, q, G, h, lb, ub):
    """Independent solve of min ½xᵀPx + qᵀx s.t. Gx ≤ h, lb ≤ x ≤ ub with scipy SLSQP (dense)."""
    from scipy.optimize import minimize
    n = q.size
    cons = [] if G is None else [{'type': 'ineq', 'fun': lambda x: h - G @ x, 'jac': lambda x: -G}]
    x0 = np.clip(np.zeros(n), lb, ub)
    lb_ = np.where(np.isfinite(lb), lb, None)
    ub_ = np.where(np.isfinite(ub), ub, None)
    r = minimize(lambda x: 0.5 * x @ Pd @ x + q @ x, x0, jac=lambda x: Pd @ x + q,
                 constraints=cons, bounds=list(zip(lb_, ub_)), method='SLSQP',
                 options={'ftol': 1e-12, 'maxiter': 2000})
    return r.x, r
