"""Acceleration-level HOCBF safety filter — standalone sim mirror of the real node.

This module re-implements, in pure NumPy + raw OSQP, the SAME math the real
ROS 2 node ``franka_experiments/nodes/cbf_safety_filter.py`` solves on the
robot, so that a policy trained against this shield in simulation meets the
identical constraint on hardware (safe exploration, Sim-to-Real shielding).

It is intentionally DUPLICATED (not imported from the ROS package) so that
``franka_sim`` stays a standalone training module with no ROS 2 dependency —
the roadmap's "franka_sim/ indipendente da ROS 2". The class-K gains, d_safe,
slack penalty ρ, hard state-limit box and workspace box all read from the same
numbers in ``franka_sim/config.yaml`` that ``fr3_control.yaml`` uses on the
robot; keep the two in sync when either changes.

Per active obstacle i (barrier h̄ = d − d_safe has relative degree 2 → HOCBF):

    aᵢ  = n̂ᵢᵀ Jᵢ                                    (built by the env from MuJoCo)
    bᵢ  = −k1·(aᵢᵀ q̇) − k0·h̄ᵢ − ċᵢ                  ċᵢ = n̂ᵢᵀ(J̇ᵢ q̇) drift term
    row : aᵢᵀ q̈ + s ≥ bᵢ            (SOFT: slack s ≥ 0 can relax obstacle rows)

QP solved each control tick (same objective as the real node):

    min  ½‖q̈ − q̈_nom‖²  +  ½ρ s²
    s.t. box(q, q̇):  hard state-limit box (static decel ∩ one-step velocity
                     bound ∩ position braking curve √(2ηa·h)) ∩ slew |q̈−q̈_prev|≤Δ
         aᵢᵀ q̈ + s ≥ bᵢ    ∀ obstacle          (soft, slack-relaxable)
         aⱼᵀ q̈     ≥ bⱼ    ∀ near workspace face (hard, slack col = 0)

OSQP is driven with the raw setup/update/solve API exactly as the real node
(``osqp.OSQP``), not through qpsolvers, so the numerical behaviour matches.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional

import numpy as np
import osqp
import scipy.sparse as sparse

NV = 7


# ── Constraint ingredients handed in by the environment ──────────────────────

@dataclass
class Obstacle:
    """One soft HOCBF row, geometry already reduced to the QP row by the env."""
    name: str
    d: float                 # surface distance  d = ‖p_cp − p_obs‖ − r_obs  [m]
    a: np.ndarray            # (NV,)  aᵢ = n̂ᵢᵀ Jpᵢ   (CBF leverage row)
    cdot: float = 0.0        # ċᵢ = n̂ᵢᵀ(J̇pᵢ q̇)  drift term (0 if unavailable)


@dataclass
class CBFInfo:
    """Diagnostics returned alongside q̈_safe (mirrors cbf_status + CBFDIAG)."""
    n_c: int = 0             # total active rows (obstacles + workspace)
    n_obs: int = 0           # obstacle (soft) rows among them
    slack: float = 0.0       # QP slack s (>0 ⇒ a soft row is being relaxed)
    min_h: float = 99.0      # smallest obstacle h̄ (99 = none active); <0 ⇒ inside d_safe
    solved: bool = True      # QP solved cleanly this tick
    braking: bool = False    # fell back to −k_brake·q̇ (QP fail / infeasible)
    intervention: float = 0.0  # ‖q̈_safe − q̈_nom‖ how hard the shield bent nominal


# ── Ported hard state-limit helpers (see utils/cbf_hard_limits.py) ───────────

def hard_accel_box(q, qdot, *, acc_lb, acc_ub, qdot_max, v_margin,
                   q_min, q_max, q_margin, brake_eta, dt):
    """Per-joint q̈ box enforcing joint velocity AND position limits. (lb, ub)."""
    h_up = np.maximum(q_max - q_margin - q, 0.0)
    h_lo = np.maximum(q - q_margin - q_min, 0.0)
    a_auth = brake_eta * np.minimum(np.abs(acc_lb), np.abs(acc_ub))
    v_cap = v_margin * qdot_max
    v_ub = np.minimum(v_cap, np.sqrt(2.0 * a_auth * h_up))
    v_lb = np.maximum(-v_cap, -np.sqrt(2.0 * a_auth * h_lo))
    ub = np.minimum(acc_ub, (v_ub - qdot) / dt)
    lb = np.maximum(acc_lb, (v_lb - qdot) / dt)
    ub = np.maximum(ub, lb)          # feasibility guard (priority to lb)
    return lb, ub


def apply_slew_limit(lb, ub, qddot_prev, delta):
    """Intersect (lb, ub) with the accel-continuity box q̈_prev ± Δ (non-empty)."""
    lo_s = qddot_prev - delta
    hi_s = qddot_prev + delta
    lo = np.maximum(lb, lo_s)
    hi = np.minimum(ub, hi_s)
    above = lb > hi_s
    below = ub < lo_s
    lo = np.where(above, hi_s, np.where(below, lo_s, lo))
    hi = np.where(above, hi_s, np.where(below, lo_s, hi))
    return lo, hi


def workspace_face_rows(p, Jp, Jpd_qd, ws_min, ws_max, margin, horizon):
    """HOCBF row ingredients (a, h, jdq, label) for an axis-aligned Cartesian box."""
    rows = []
    axes = ('x', 'y', 'z')
    for k in range(3):
        h_lo = float(p[k] - ws_min[k] - margin)
        if h_lo < horizon:
            rows.append((Jp[k].copy(), h_lo, float(Jpd_qd[k]), f'ws:{axes[k]}-'))
        h_hi = float(ws_max[k] - margin - p[k])
        if h_hi < horizon:
            rows.append((-Jp[k], h_hi, float(-Jpd_qd[k]), f'ws:{axes[k]}+'))
    return rows


# ── Filter ───────────────────────────────────────────────────────────────────

class AccelCBFFilter:
    """Acceleration-level HOCBF QP shield, config-driven, raw-OSQP solved."""

    def __init__(self, cbf_cfg: dict, qddot_max, qdot_max, q_min, q_max,
                 dt: float):
        p = cbf_cfg
        self.d_safe   = float(p.get('d_safe', 0.20))
        self.k0       = float(p.get('k0_cbf', 25.0))
        self.k1       = float(p.get('k1_cbf', 10.5))
        self.rho      = float(p.get('rho_slack', 1000.0))
        self.horizon  = float(p.get('cbf_obstacle_horizon', 1.2))
        self.a_min    = float(p.get('cbf_min_leverage', 0.05))
        self.k_brake  = float(p.get('k_brake', 3.0))
        self.max_iter = int(p.get('osqp_max_iter', 20000))

        # Hard state-limit box + slew continuity.
        self.hard_v_margin  = float(p.get('hard_v_margin', 0.9))
        self.hard_q_margin  = float(p.get('hard_q_margin', 0.05))
        self.hard_brake_eta = float(p.get('hard_brake_eta', 0.7))
        self.slew_delta     = float(p.get('max_qddot_delta', 5.0))
        self.dt = float(dt)

        # Workspace box (hard rows on the EE point).
        self.ws_enable  = bool(p.get('ws_enable', True))
        self.ws_min     = np.asarray(p.get('ws_min', [0.05, -0.60, 0.05]), float)
        self.ws_max     = np.asarray(p.get('ws_max', [0.75, 0.60, 0.95]), float)
        self.ws_margin  = float(p.get('ws_margin', 0.02))
        self.ws_horizon = float(p.get('ws_horizon', 0.25))

        self.qddot_max = np.asarray(qddot_max, float)
        self.qdot_max  = np.asarray(qdot_max, float)
        self.q_min     = np.asarray(q_min, float)
        self.q_max     = np.asarray(q_max, float)
        self._acc_lb   = -self.qddot_max
        self._acc_ub   =  self.qddot_max

        # Constant QP cost P = diag(I_7, ρ); box bounds get the slack tail.
        self._P = np.eye(NV + 1)
        self._P[-1, -1] = self.rho
        self._P_csc = sparse.csc_matrix(self._P)
        self._qvec = np.zeros(NV + 1)
        self._box_lb = np.append(self._acc_lb, 0.0)
        self._box_ub = np.append(self._acc_ub, 1e6)

        # One persistent OSQP problem per constraint count (fixed sparsity per
        # n_c) → setup() paid once per n_c, update() every other tick.
        self._probs: dict[int, osqp.OSQP] = {}
        self.reset()

    # ── Episode lifecycle ────────────────────────────────────────────────────

    def reset(self):
        """Clear slew anchor + warm-started OSQP state at the start of an episode."""
        self._qddot_prev = np.zeros(NV)
        self._probs.clear()

    # ── OSQP assembly (identical layout to the real node) ────────────────────

    @staticmethod
    def _osqp_A(G: Optional[np.ndarray]) -> sparse.csc_matrix:
        box = sparse.identity(NV + 1, format='csc')
        if G is None:
            return box
        n_c = G.shape[0]
        rows = np.repeat(np.arange(n_c), NV + 1)
        cols = np.tile(np.arange(NV + 1), n_c)
        cbf = sparse.csc_matrix((G.ravel(), (rows, cols)), shape=(n_c, NV + 1))
        return sparse.vstack([cbf, box], format='csc')

    def _osqp_lu(self, G, h_qp):
        if G is None:
            return self._box_lb, self._box_ub
        n_c = G.shape[0]
        l = np.concatenate([np.full(n_c, -np.inf), self._box_lb])
        u = np.concatenate([h_qp, self._box_ub])
        return l, u

    def _solve(self, G, h_qp, n_c):
        A = self._osqp_A(G)
        l, u = self._osqp_lu(G, h_qp)
        prob = self._probs.get(n_c)
        if prob is None:
            prob = osqp.OSQP()
            prob.setup(P=self._P_csc, q=self._qvec, A=A, l=l, u=u,
                       warm_start=True, max_iter=self.max_iter, verbose=False)
            self._probs[n_c] = prob
        elif n_c > 0:
            prob.update(q=self._qvec, l=l, u=u, Ax=A.data)
        else:
            prob.update(q=self._qvec, l=l, u=u)
        return prob.solve()

    # ── Main entry point ─────────────────────────────────────────────────────

    def filter(self, q, qdot, qddot_nom, obstacles: List[Obstacle],
               ee_pos=None, ee_Jp=None, ee_jd_qd=None) -> tuple:
        """Shield q̈_nom into q̈_safe. Returns (q̈_safe (NV,), CBFInfo).

        obstacles : soft HOCBF rows (built by the env from MuJoCo geometry).
        ee_pos/ee_Jp/ee_jd_qd : EE point + its (3,NV) Jacobian + J̇q̇ drift, used
            for the hard workspace-box rows (skip if ws disabled or None).
        """
        q = np.asarray(q, float)
        qdot = np.asarray(qdot, float)
        qddot_nom = np.asarray(qddot_nom, float)
        info = CBFInfo()

        # Hard state-limit ∩ slew box → written into the QP box bounds.
        h_lb, h_ub = hard_accel_box(
            q, qdot, acc_lb=self._acc_lb, acc_ub=self._acc_ub,
            qdot_max=self.qdot_max, v_margin=self.hard_v_margin,
            q_min=self.q_min, q_max=self.q_max, q_margin=self.hard_q_margin,
            brake_eta=self.hard_brake_eta, dt=self.dt)
        box_lo, box_hi = apply_slew_limit(h_lb, h_ub, self._qddot_prev,
                                          self.slew_delta)
        self._box_lb[:NV] = box_lo
        self._box_ub[:NV] = box_hi

        # ── Build rows: obstacles (soft) then workspace faces (hard) ─────────
        rows_a, rows_b, rows_soft, rows_h = [], [], [], []
        for ob in obstacles:
            if ob.d > self.horizon:
                continue
            a = np.asarray(ob.a, float)
            if float(np.linalg.norm(a)) < self.a_min:
                continue
            h = ob.d - self.d_safe
            b = -self.k1 * float(a @ qdot) - self.k0 * h - float(ob.cdot)
            if not (np.all(np.isfinite(a)) and np.isfinite(b)):
                continue
            rows_a.append(a); rows_b.append(b); rows_soft.append(1.0); rows_h.append(h)
        n_obs = len(rows_a)

        if self.ws_enable and ee_pos is not None and ee_Jp is not None:
            jd = np.zeros(3) if ee_jd_qd is None else np.asarray(ee_jd_qd, float)
            for a_row, h_ws, jdq_ws, _label in workspace_face_rows(
                    np.asarray(ee_pos, float), np.asarray(ee_Jp, float), jd,
                    self.ws_min, self.ws_max, self.ws_margin, self.ws_horizon):
                b = -self.k1 * float(a_row @ qdot) - self.k0 * h_ws - jdq_ws
                if np.all(np.isfinite(a_row)) and np.isfinite(b):
                    rows_a.append(a_row); rows_b.append(b)
                    rows_soft.append(0.0); rows_h.append(h_ws)

        n_c = len(rows_a)
        info.n_c = n_c
        info.n_obs = n_obs
        if n_obs > 0:
            info.min_h = float(min(rows_h[:n_obs]))

        # ── Assemble G, h and solve ──────────────────────────────────────────
        self._qvec[:NV] = -qddot_nom
        if n_c == 0:
            G = None; h_qp = None
        else:
            A = np.vstack(rows_a)                      # (n_c, NV)
            b = np.asarray(rows_b, float)
            soft = np.asarray(rows_soft, float)
            G = np.empty((n_c, NV + 1))
            G[:, :NV] = -A
            G[:, -1] = -soft                            # obstacle rows soft, ws hard
            h_qp = -b                                   # −A q̈ − soft·s ≤ −b

        res = self._solve(G, h_qp, n_c)
        x = res.x
        solved = (res.info.status_val == osqp.constant('OSQP_SOLVED')
                  and x is not None and np.all(np.isfinite(x)))

        if solved:
            qddot_safe = np.asarray(x[:NV], float)
            info.slack = float(x[-1]) if n_c > 0 else 0.0
        else:
            # QP failure → conservative braking; drop the poisoned warm start.
            self._probs.pop(n_c, None)
            qddot_safe = -self.k_brake * qdot
            info.solved = False
            info.braking = True

        # Every path passes through the hard box+slew clip (as the real node's
        # _finalize_and_publish) so velocity/position/continuity always hold.
        qddot_safe = np.clip(qddot_safe, self._box_lb[:NV], self._box_ub[:NV])
        info.intervention = float(np.linalg.norm(qddot_safe - qddot_nom))
        self._qddot_prev[:] = qddot_safe
        return qddot_safe, info
