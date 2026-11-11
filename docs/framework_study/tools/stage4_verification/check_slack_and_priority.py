#!/usr/bin/env python3
"""Slack semantics and constraint priority of the filter's QP (chapter 08 §3, §5).

All problems are built with the production objective/bounds/A builders through `_common4.QPHarness`
(OSQP 0.6.x, tight tolerance 1e-9 unless stated) and compared with closed-form KKT solutions.

  A. (8.12) quadratic family slack: one row aᵀq̈ + s ≥ b inside the box. Closed form
     λ = (b − aᵀq̈_n)/(‖a‖² + 1/ρ), q̈ = q̈_n + λa, s = λ/ρ: the unrelaxed row is ALWAYS violated
     by s = (b − aᵀq̈_n)/(1 + ρ‖a‖²) > 0, i.e. the soft barrier is a penalty, not a constraint  (NUM)
  B. shared family slack couples rows: a second, easily satisfiable obstacle row is also violated
     when another row of the same family needs slack                                          (NUM)
  C. (8.14) row_l1 (exact penalty): with per-row slots and ρ_l1 = 1000 the same row as A is met
     exactly (s = 0) because its multiplier λ < ρ_l1; slack appears only once λ would exceed it  (NUM)
  D. hard box beats soft rows: a row whose demand lies outside the slew window is relaxed by
     exactly the shortfall left after the box-constrained optimum                             (NUM)
  E. (8.13) obstacle row vs retreat cap on the same normal: closed-form split of the conflict,
     ρ_obs·s_obs vs ρ_cap·s_cap                                                               (NUM)
  F. feasibility by construction: 200 random snapshots with conflicting rows (family mode) all solve;
     with obstacle_slack_max = 0 (hard obstacle rows) a conflict is reported infeasible        (NUM)

Needs: the ROS 2 container (osqp 0.6.x). Read-only.
"""
import os
import sys

sys.dont_write_bytecode = True
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _common4 as C4  # noqa: E402

C, C3 = C4.C, C4.C3
import numpy as np  # noqa: E402
import osqp  # noqa: E402
from franka_experiments.utils.cbf_hard_limits import apply_slew_limit  # noqa: E402
from franka_experiments.utils.cbf_state_rows import G_CAP, G_OBS, NV, NX  # noqa: E402

R = C.Report('check_slack_and_priority (chapter 08 §3, §5)')
TIGHT = dict(eps_abs=1e-10, eps_rel=1e-10, polish=True)
P = C3.params_namespace('hw_default')
H = C4.QPHarness(P)
rng = np.random.default_rng(11)
a = np.array([0.0, -0.18, 0.0, 0.47, 0.0, 0.18, 0.0])      # the held-row a of 07 §8.1 (scale of a real row)
qn = np.zeros(NV)


def row(a_, grp, nx, slot=None, m=1.0):
    g = np.zeros(nx)
    g[:NV] = -a_
    g[NX + slot if slot is not None else NV + grp] = -m
    return g


# A ------------------------------------------------------------------------------------------
rho = P.rho_slack
b = 0.8                                       # demanded aᵀq̈ (m/s²); well inside the box
G = row(a, G_OBS, H.nx)[None, :]
x, res, _, _ = H.solve(G, np.array([-b]), qn, osqp_kw=TIGHT)
lam = (b - a @ qn) / (a @ a + 1.0 / rho)
s_cf = lam / rho
R.close('A. q̈ = q̈_n + λa (closed form)', x[:NV], qn + lam * a, 1e-7, kind='NUM')
R.close('A. family slack s = (b − aᵀq̈_n)/(1 + ρ‖a‖²)', x[NV + G_OBS], s_cf, 1e-8, kind='NUM')
R.close('A. unrelaxed violation b − aᵀq̈ equals s', b - a @ x[:NV], s_cf, 1e-8, kind='NUM',
        note=f'{100 * s_cf / b:.2f} % of the demand with ρ = {rho:g}, ‖a‖ = {np.linalg.norm(a):.3f}')

# B ------------------------------------------------------------------------------------------
a2 = np.array([0.3, 0.0, 0.2, 0.0, 0.1, 0.0, 0.1])          # a second row, nearly orthogonal
b_big, b2 = 40.0, 0.5
G = np.vstack([row(a, G_OBS, H.nx), row(a2, G_OBS, H.nx)])
x, _, _, _ = H.solve(G, np.array([-b_big, -b2]), qn, osqp_kw=TIGHT)
s = x[NV + G_OBS]
v2 = b2 - a2 @ x[:NV]
R.fact(f'B. row 1 (demand {b_big} m/s², beyond the box) needs slack s = {s:.3f}', s > 1.0)
R.fact(f'B. row 2 (demand {b2}, satisfiable alone) is ALSO violated by {v2:.3f} m/s²: the shared slack '
       'relaxes every row of the family', v2 > 1e-3)
x2, _, _, _ = H.solve(row(a2, G_OBS, H.nx)[None, :], np.array([-b2]), qn, osqp_kw=TIGHT)
R.info(f'B. row 2 alone: violation {b2 - a2 @ x2[:NV]:.2e} (the A-type penalty residue only)')

# C ------------------------------------------------------------------------------------------
P1 = C3.params_namespace('sim:minimal_l1')
H1 = C4.QPHarness(P1)
R.fact(f'C. row_l1 harness: {H1.n_slot} slots, ρ_l1 = {P1.rho_slack_l1:g}, slot quadratic ε = '
       f'{P1.obstacle_row_slack_eps:g}', H1.n_slot == 32)
G = row(a, G_OBS, H1.nx, slot=0)[None, :]
x, _, _, _ = H1.solve(G, np.array([-b]), qn, osqp_kw=TIGHT)
lam_hard = (b - a @ qn) / (a @ a)
R.close(f'C. row met exactly: slot slack (hard multiplier λ = {lam_hard:.2f} < ρ_l1)', x[NX], 0.0, 1e-7, kind='NUM')
R.close('C. q̈ equals the HARD-constraint solution q̈_n + (b/‖a‖²)a', x[:NV], qn + lam_hard * a, 1e-6, kind='NUM')
b_hi = 1.2 * P1.rho_slack_l1 * (a @ a)        # multiplier 1.2·ρ_l1 → exact penalty must give way
x, _, _, _ = H1.solve(G, np.array([-b_hi]), qn, osqp_kw=dict(TIGHT, max_iter=200000))
s_slot = x[NX]
R.info(f'C. demand {b_hi:.1f}: slot slack {s_slot:.4f}, box-limited q̈ {np.round(x[:NV], 3)}')
R.fact('C. beyond the exact-penalty threshold (or the box) the slot slack becomes positive', s_slot > 1e-6)

# D ------------------------------------------------------------------------------------------
prev = np.zeros(NV)
blb, bub = H.box_lb.copy(), H.box_ub.copy()
blb[:NV], bub[:NV] = apply_slew_limit(blb[:NV], bub[:NV], prev, P.max_qddot_delta, yield_to_state_box=True)
G = row(a, G_OBS, H.nx)[None, :]
x, _, _, _ = H.solve(G, np.array([-b]), qn, box_lb=blb, box_ub=bub, osqp_kw=TIGHT)
reach = float(np.sum(np.abs(a)) * P.max_qddot_delta)        # max aᵀq̈ inside ±δ
R.close(f'D. slew window ±{P.max_qddot_delta} rad/s² caps aᵀq̈ at Σ|aᵢ|·δ = {reach:.4f}', a @ x[:NV], reach, 1e-6,
        kind='NUM')
R.close('D. the row is relaxed by the shortfall b − Σ|aᵢ|δ', x[NV + G_OBS], b - reach, 1e-6, kind='NUM',
        note='the hard box (state ∩ slew) has priority over every CBF row')

# E ------------------------------------------------------------------------------------------
b_obs, b_cap = 1.0, 0.4                     # obstacle wants aᵀq̈ ≥ 1.0, cap allows ≤ 0.4
G = np.vstack([row(a, G_OBS, H.nx), np.concatenate([a, np.zeros(H.nx - NV)])])
G[1, NV + G_CAP] = -1.0                      # cap row: +aᵀq̈ − s_cap ≤ b_cap
x, _, _, _ = H.solve(G, np.array([-b_obs, b_cap]), qn, osqp_kw=TIGHT)
ro, rc, aa = P.rho_slack, P.rho_slack_retreat, a @ a
M = np.array([[1.0 + aa * ro, -aa * rc], [1.0, 1.0]])
s_o, s_c = np.linalg.solve(M, [b_obs - a @ qn, b_obs - b_cap])
R.close('E. (s_obs, s_cap) from the KKT system', [x[NV + G_OBS], x[NV + G_CAP]], [s_o, s_c], 1e-7, kind='NUM')
R.info(f'E. the cap gives way: s_cap = {s_c:.4f} vs s_obs = {s_o:.4f}; aᵀq̈ = {a @ x[:NV]:.4f} '
       f'(ρ_obs = {ro:g}, ρ_cap = {rc:g})')

# F ------------------------------------------------------------------------------------------
ok = 0
for _ in range(200):
    n = int(rng.integers(2, 12))
    A_ = rng.normal(size=(n, NV)) * rng.uniform(0.05, 1.0, size=(n, 1))
    grp = rng.choice([0, 1, 2, 3, 4], size=n)
    G = np.zeros((n, H.nx))
    G[:, :NV] = -A_
    G[np.arange(n), NV + grp] = -1.0
    h = rng.normal(scale=20.0, size=n)
    x, res, _, _ = H.solve(G, h, rng.normal(scale=3.0, size=NV))
    ok += x is not None
R.fact(f'F. 200 random conflicting snapshots (family mode, production tolerance): {ok}/200 solved', ok == 200)
Ph = C3.params_namespace('hw_default', obstacle_slack_max=0.0)
Hh = C4.QPHarness(Ph)
G = row(a, G_OBS, Hh.nx)[None, :]
x, res, _, _ = Hh.solve(G, np.array([-40.0]), qn)
R.fact(f'F. obstacle_slack_max = 0 and an unreachable demand → status "{res.info.status}", '
       'iterate rejected (the fallback ladder takes over)', x is None)
R.fact(f'F. OSQP 0.6 returns x as an object array of None when infeasible (dtype {np.asarray(res.x).dtype}); '
       f'accept_iterate then RAISES ({type(Hh.last_exc).__name__}) instead of returning None — the node '
       'reaches the fallback only through the try/except in _solve (F-4-x)',
       isinstance(Hh.last_exc, TypeError) and np.asarray(res.x).dtype == object)
sys.exit(R.finish())
