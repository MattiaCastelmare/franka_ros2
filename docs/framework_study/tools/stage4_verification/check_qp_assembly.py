#!/usr/bin/env python3
"""The QP of cbf_safety_filter, assembled from the production functions on a real snapshot (ch. 08 §2–§4).

Snapshot: the real `ConstraintBuilder` (hardware-default parameters, real kinematics, the optional
self-collision and singularity builders from `build_optional_row_builders`) at a pose near q_home
with joint 7 at 2.90 rad (a joint-limit row) and one obstacle 0.13 m in front of the fingertip
(an obstacle row + its retreat cap). The arm moves toward the obstacle; the nominal pushes it on.

Then the `_qp_tick` sequence (transcribed): `build_row_rhs` → `position_velocity_accel_box` →
`apply_slew_limit` → `_solve` (`pad_rows_to_block`, `build_osqp_bounds`, `build_osqp_A`,
OSQP 0.6.x `setup` with the node's arguments, `accept_iterate`).

  1. objective: P = blkdiag(I_7, diag ρ_g), q = [−q̈_nom; 0]  ⇒ ½‖q̈ − q̈_nom‖² + Σ ½ρ_g s_g²      (FACT)
  2. A = [G_padded; I]; the sparsity pattern is DENSE and value-independent (explicit zeros kept),
     so `update(Ax=…)` is valid whenever the padded row count is unchanged                    (NUM)
  3. updated (warm) problem == freshly set-up problem                                          (NUM)
  4. padding rows (block 16) do not change the solution                                        (NUM)
  5. OSQP solution vs an independent SLSQP solve of the same problem                            (NUM)
  6. box satisfied exactly after `accept_iterate`; relaxed rows satisfied to OSQP tolerance;
     which rows actually use slack (raw violation)                                       (NUM / FACT)

Needs: the ROS 2 container (Pinocchio, xacro, franka_description, osqp 0.6.x, scipy). Read-only.
"""
import os
import sys

sys.dont_write_bytecode = True
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _common4 as C4  # noqa: E402

C, C3 = C4.C, C4.C3
import _builder3 as B  # noqa: E402
import numpy as np  # noqa: E402
import osqp  # noqa: E402
from franka_experiments.utils.cbf_hard_limits import apply_slew_limit, position_velocity_accel_box  # noqa: E402
from franka_experiments.utils.cbf_qp_assembly import build_osqp_A, build_row_rhs  # noqa: E402
from franka_experiments.utils.cbf_state_rows import (  # noqa: E402
    GROUP_NAMES, ConstraintBuilder, Obstacle, ObstacleSnap, JointSnap, build_optional_row_builders)

R = C.Report('check_qp_assembly (chapter 08 §2-§4)')
R.info(f'osqp {osqp.__version__}')
P = C3.params_namespace('hw_default')
log = B.Log()
kin = B.make_kin()
opt = build_optional_row_builders(P, kin, log)
L = C4.static_limits(P)
builder = ConstraintBuilder(P, kin, q_min=L['q_min'], q_max=L['q_max'], acc_lb=L['lb'], acc_ub=L['ub'],
                            logger=log, brake_acc=L['brake'], qdot_max=L['qdot_max'],
                            control_point_jacobian=kin.control_point_jacobian, **opt)
R.info('optional builders: ' + ', '.join(f'{k}={"on" if v is not None else "off"}'
                                         for k, v in opt.items() if k in ('sc_rows', 'sing_rows', 'ssb_rows')))
q = C.Q_HOME.copy()
q[6] = 2.90
kin.update(q, np.zeros(7))
lbl = 'fr3_link8#2'
pr = kin.control_point_position(lbl)
J, _ = kin.control_point_jacobian(lbl)
qdot = np.linalg.pinv(J) @ np.array([0.15, 0.0, 0.0])
qddot_nom = np.linalg.pinv(J) @ np.array([2.0, 0.0, 0.0])
ph = pr + np.array([0.18, 0.0, 0.0])
ob = Obstacle(link='fr3_link8', d=0.13, pr=pr, ph=ph, conf=0.9, v_vec=np.zeros(3), frames_seen=0,
              vel_cov=np.zeros((3, 3)), track_id=0, cp_label=lbl)
t = 50.0
con = None
for k in range(3):     # a few rebuilds so the per-label filters settle (static obstacle, residual 0)
    con = builder.build(JointSnap(q, qdot, t + k * 0.02), ObstacleSnap((ob,), t + k * 0.02, t + k * 0.02),
                        t + k * 0.02)
R.fact('snapshot built', con is not None, f'{con.A.shape[0]} rows: ' + ', '.join(
    f'{GROUP_NAMES[g]}:{int((con.group == g).sum())}' for g in sorted(set(con.group.tolist()))))

# _qp_tick transcription ---------------------------------------------------------------
H = C4.QPHarness(P)
h_qp, _ = build_row_rhs(con, qdot, qdot, k0=P.k0_cbf, k1=P.k1_cbf,
                        retreat_horizon=P.retreat_cap_horizon_s, speed_horizon=P.link_speed_horizon_s)
dt = 1.0 / P.qp_rate_hz
blb, bub = H.box_lb.copy(), H.box_ub.copy()
position_velocity_accel_box(q, qdot, acc_lb=H.lb, acc_ub=H.ub, qdot_max=L['qdot_max'],
                            v_margin=P.velocity_box_margin, q_min=L['q_min'], q_max=L['q_max'],
                            q_margin=P.position_margin_rad, brake_eta=P.position_brake_eta, dt=dt,
                            relax_dt=P.state_box_relax_s, out_lb=blb[:7], out_ub=bub[:7],
                            clip_to_limits=P.accel_box_clip_to_limits, brake_acc=L['brake'])
state_lb, state_ub = blb[:7].copy(), bub[:7].copy()
qdd_prev = qddot_nom.copy()          # previous command = the nominal: the slew window is centred on it
blb[:7], bub[:7] = apply_slew_limit(blb[:7], bub[:7], qdd_prev, P.max_qddot_delta,
                                    yield_to_state_box=P.slew_yields_to_state_box)
x, res, prob, qvec = H.solve(con.G, h_qp, qddot_nom, box_lb=blb, box_ub=bub)
R.fact('OSQP status solved', x is not None and res.info.status == 'solved', res.info.status)

# 1) objective
Pd = H.P_csc.toarray()
diag_exp = np.concatenate([np.ones(7), [H.rho[g] for g in range(H.N_SLACK)], np.full(H.n_slot, P.obstacle_row_slack_eps)])
R.fact('P is diagonal: I_7 on q̈, ρ_g on the family slacks', np.array_equal(Pd, np.diag(diag_exp)),
       f'ρ = {dict((GROUP_NAMES[g], H.rho[g]) for g in range(H.N_SLACK))}')
R.close('q[:7] = −q̈_nom, q[7:] = 0 (family mode)', qvec, np.concatenate([-qddot_nom, np.zeros(H.n_slack)]), 0,
        kind='FACT')
xs = x
obj_osqp = 0.5 * xs @ Pd @ xs + qvec @ xs
obj_math = 0.5 * np.sum((xs[:7] - qddot_nom) ** 2) + sum(0.5 * H.rho[g] * xs[7 + g] ** 2 for g in range(H.N_SLACK)) \
    - 0.5 * qddot_nom @ qddot_nom
R.close('objective identity ½xᵀPx + qᵀx = ½‖q̈−q̈_nom‖² + Σ½ρ s² − ½‖q̈_nom‖²', obj_osqp, obj_math, 1e-9, kind='NUM')

# 2) sparsity pattern
from franka_experiments.utils.cbf_qp_assembly import pad_rows_to_block  # noqa: E402
Gp, hp = pad_rows_to_block(con.G, h_qp, P.qp_row_block)
A1 = build_osqp_A(Gp, 7, H.n_slack)
G2 = Gp.copy()
G2[:, :7] *= 0.0
G2[0, :7] = 1.0
A2 = build_osqp_A(G2, 7, H.n_slack)
R.fact(f'padded rows: {con.A.shape[0]} → {Gp.shape[0]} (qp_row_block {P.qp_row_block})',
       Gp.shape[0] % P.qp_row_block == 0)
R.fact('A pattern is dense and independent of the values (nnz, indptr, indices identical with many zeros)',
       A1.nnz == A2.nnz == Gp.shape[0] * H.nx + H.nx and np.array_equal(A1.indptr, A2.indptr)
       and np.array_equal(A1.indices, A2.indices), f'nnz {A1.nnz} = {Gp.shape[0]}·{H.nx} + {H.nx}')

# 3) update path == fresh setup
from franka_experiments.utils.cbf_qp_assembly import build_osqp_bounds  # noqa: E402
h2 = hp.copy()
h2[:con.A.shape[0]] += 0.3
l2, u2 = build_osqp_bounds(Gp, h2, blb, bub)
TIGHT = dict(eps_abs=1e-9, eps_rel=1e-9, polish=True)
p_up = osqp.OSQP()
p_up.setup(P=H.P_csc, q=qvec, A=A1, l=build_osqp_bounds(Gp, hp, blb, bub)[0],
           u=build_osqp_bounds(Gp, hp, blb, bub)[1], warm_start=True, max_iter=P.osqp_max_iter, verbose=False, **TIGHT)
p_up.solve()
p_up.update(q=qvec, l=l2, u=u2, Ax=A1.data)
r_up = p_up.solve()
fresh = osqp.OSQP()
fresh.setup(P=H.P_csc, q=qvec, A=A1, l=l2, u=u2, warm_start=True, max_iter=P.osqp_max_iter, verbose=False, **TIGHT)
r_fr = fresh.solve()
R.below('update(q, l, u, Ax) then solve == fresh setup then solve (eps 1e-9): max |Δx|',
        float(np.abs(r_up.x - r_fr.x).max()), 1e-6)
prob.update(q=qvec, l=l2, u=u2, Ax=A1.data)
r_up_d = prob.solve()
f_d = osqp.OSQP()
f_d.setup(P=H.P_csc, q=qvec, A=A1, l=l2, u=u2, warm_start=True, max_iter=P.osqp_max_iter, verbose=False)
r_fr_d = f_d.solve()
R.info(f'at the PRODUCTION tolerance (OSQP default) warm-updated vs fresh differ by '
       f'{np.abs(r_up_d.x[:7] - r_fr_d.x[:7]).max():.2e} rad/s² on q̈ and '
       f'{np.abs(r_up_d.x[7:] - r_fr_d.x[7:]).max():.2e} on the slacks')

# 4) padding inert
x1, _, _, _ = H.solve(con.G, h_qp, qddot_nom, box_lb=blb, box_ub=bub, block=1, osqp_kw=TIGHT)
x16, _, _, _ = H.solve(con.G, h_qp, qddot_nom, box_lb=blb, box_ub=bub, block=16, osqp_kw=TIGHT)
R.below('padding to blocks of 16 vs no padding (eps 1e-9): max |Δx|', float(np.abs(x1 - x16).max()), 1e-6)

# 5) independent solver
xi, ri = C4.independent_qp(Pd, qvec, con.G, h_qp, blb, bub)
oi = 0.5 * xi @ Pd @ xi + qvec @ xi
R.fact(f'SLSQP converged ({ri.message})', ri.success)
o16 = 0.5 * x16 @ Pd @ x16 + qvec @ x16
R.close('objective: OSQP (eps 1e-9) vs independent SLSQP', o16, oi, 1e-5 * abs(oi), kind='NUM')
R.below('x: OSQP (eps 1e-9) vs independent SLSQP, max |Δ|', float(np.abs(x16 - xi).max()), 1e-4)
R.info(f'at the PRODUCTION tolerance: q̈ differs from SLSQP by {np.abs(x[:7] - xi[:7]).max():.2e} rad/s², '
       f'the objective by {obj_osqp - oi:+.3f} ({(obj_osqp - oi) / oi:+.2e} relative; dominated by ½ρs² with s_obs = {x[7]:.3f})')

# 6) feasibility and slack use
R.fact('box (state box ∩ slew window) satisfied exactly after accept_iterate',
       bool(np.all(x[:7] >= blb[:7]) and np.all(x[:7] <= bub[:7])))
viol_relaxed = con.G @ x - h_qp
R.below('relaxed rows: max(Gx − h_qp)', float(viol_relaxed.max()), 5e-3, note='OSQP primal tolerance')
raw = -(con.A @ x[:7]) - h_qp          # > 0 ⇔ the row WITHOUT its slack is violated
for i in range(con.A.shape[0]):
    R.info(f'row {i:2d} {con.links[i]:18s} group {GROUP_NAMES[con.group[i]]:4s}  h_qp {h_qp[i]:+9.3f}  '
           f'aᵀq̈ {con.A[i] @ x[:7]:+9.3f}  raw violation {raw[i]:+.2e}')
R.info('slack per family: ' + ', '.join(f'{GROUP_NAMES[g]}={x[7 + g]:.2e}' for g in range(H.N_SLACK)))
R.info(f'q̈_nom = {np.round(qddot_nom, 3)}; q̈_safe = {np.round(x[:7], 3)}')
R.info(f'box after slew: lb {np.round(blb[:7], 3)} ub {np.round(bub[:7], 3)}')
sys.exit(R.finish())
