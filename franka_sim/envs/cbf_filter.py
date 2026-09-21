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


# ── FR3 firmware velocity envelope ───────────────────────────────────────────
#
# COPIED, not imported, from franka_experiments/utils/cbf_hard_limits.py:
# franka_sim must stay importable with no ROS installed. Keep the two in step —
# test_real_configs_are_in_sync checks the parameters, and the constants below
# are libfranka's own (rate_limiting.h, computeUpper/LowerLimitsJointVelocity).
#
# The firmware does NOT enforce a flat |q̇| ≤ q̇_max. Near a position limit the
# admissible speed collapses along
#
#     q̇_max,i(q) = min( q̇_lim,i , max(0, −v_off,i + sqrt(2·a_i·(q_ref,i − q))) )
#
# and it is THIS curve that the `joint_velocity_violation` reflex applies. The
# robot adopted it in commit f5a59f8 after five logged hardware aborts, none of
# which was anywhere near its flat q̇_max (worst case 0.85 of it). Training
# without it lets the policy explore joint states the firmware simply refuses.

FR3_VEL_LIMIT  = np.array([2.62, 2.62, 2.62, 2.62, 5.26, 4.18, 5.26])
FR3_VEL_OFFSET = np.array([0.30, 0.20, 0.20, 0.30, 0.35, 0.35, 0.35])
FR3_VEL_SLOPE  = np.array([12.0, 5.17, 7.00, 8.00, 34.0, 11.0, 34.0])
FR3_VEL_Q_REF_UPPER = np.array(
    [2.75010, 1.79180, 2.90650, -0.14580, 2.81010, 4.52050, 3.01960])
FR3_VEL_Q_REF_LOWER = np.array(
    [-2.75010, -1.79180, -2.90650, -3.04810, -2.81010, 0.54092, -3.01960])
"""q_ref lies INSIDE the mechanical stop, so the last ~0.03–0.10 rad of travel
admit no motion at all. That is why the robot also clamps its effective
position limits to these values (cbf_safety_filter, commit f5a59f8): anchoring
a barrier at the mechanical limit puts it behind the wall."""


def fr3_velocity_envelope(q, margin: float = 1.0):
    """Firmware-admissible ``(q̇_upper, q̇_lower)`` at configuration *q*."""
    up = np.minimum(FR3_VEL_LIMIT, np.maximum(
        0.0, -FR3_VEL_OFFSET + np.sqrt(np.maximum(
            0.0, FR3_VEL_SLOPE * (FR3_VEL_Q_REF_UPPER - q)))))
    lo = np.maximum(-FR3_VEL_LIMIT, np.minimum(
        0.0, FR3_VEL_OFFSET - np.sqrt(np.maximum(
            0.0, FR3_VEL_SLOPE * (q - FR3_VEL_Q_REF_LOWER)))))
    return margin * up, margin * lo


# ── Ported hard state-limit helpers (see utils/cbf_hard_limits.py) ───────────

def hard_accel_box(q, qdot, *, acc_lb, acc_ub, qdot_max, v_margin,
                   q_min, q_max, q_margin, brake_eta, dt,
                   relax_dt=None, clip_to_limits=False,
                   firmware_envelope=True):
    """Per-joint q̈ box enforcing joint velocity AND position limits. (lb, ub).

    Mirrors ``cbf_hard_limits.hard_accel_box`` term for term, including the
    three arguments the robot gained in f5a59f8 / 4606e39:

    * ``firmware_envelope`` intersects the velocity bound with
      :func:`fr3_velocity_envelope`. NOT a duplicate of the braking curve
      below: that one is built from ``q_min``/``q_max`` and ``brake_eta``, i.e.
      from what WE think the joint needs to stop, and on joint6 it comes out
      1.9x looser than the firmware's.
    * ``relax_dt`` decouples APPROACHING a cap from being over it — below the
      cap the horizon is ``relax_dt`` (the bound bites early and gently), past
      it the one-step ``dt`` (braking authority is never softened when it is
      actually needed). ``None`` reproduces the legacy one-step behaviour.
    * ``clip_to_limits`` keeps the box inside the physical acceleration limits
      after the feasibility guard, so the QP is never handed a box demanding
      more than the joint can produce.
    """
    h_up = np.maximum(q_max - q_margin - q, 0.0)
    h_lo = np.maximum(q - q_margin - q_min, 0.0)
    a_auth = brake_eta * np.minimum(np.abs(acc_lb), np.abs(acc_ub))
    v_cap = v_margin * qdot_max
    v_ub = np.minimum(v_cap, np.sqrt(2.0 * a_auth * h_up))
    v_lb = np.maximum(-v_cap, -np.sqrt(2.0 * a_auth * h_lo))

    # The firmware's own envelope, on top of ours and under the same margin.
    if firmware_envelope:
        env_up, env_lo = fr3_velocity_envelope(q, margin=v_margin)
        v_ub = np.minimum(v_ub, env_up)
        v_lb = np.maximum(v_lb, env_lo)

    rdt = dt if relax_dt is None else float(relax_dt)
    dt_ub = np.where(v_ub >= qdot, rdt, dt)
    dt_lb = np.where(v_lb <= qdot, rdt, dt)

    ub = np.minimum(acc_ub, (v_ub - qdot) / dt_ub)
    lb = np.maximum(acc_lb, (v_lb - qdot) / dt_lb)
    ub = np.maximum(ub, lb)          # feasibility guard (priority to lb)
    if clip_to_limits:
        lb = np.minimum(lb, acc_ub)  # never ask for more than the joint has
        ub = np.minimum(ub, acc_ub)
        ub = np.maximum(ub, lb)      # keep the (possibly degenerate) box ordered
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
        self.d_safe   = float(p.get('d_safe', 0.15))
        self.k0       = float(p.get('k0_cbf', 25.0))
        self.k1       = float(p.get('k1_cbf', 10.5))
        self.rho      = float(p.get('rho_slack', 1000.0))
        self.horizon  = float(p.get('cbf_obstacle_horizon', 1.2))
        self.a_min    = float(p.get('cbf_min_leverage', 0.05))
        self.k_brake  = float(p.get('k_brake', 3.0))
        self.max_iter = int(p.get('osqp_max_iter', 20000))

        # Hard state-limit box + slew continuity.  The robot renamed these three
        # (hard_v_margin → velocity_box_margin, hard_q_margin →
        # position_margin_rad, hard_brake_eta → position_brake_eta); the old
        # spelling is still read so a config.yaml frozen next to an older model
        # keeps reproducing the shield it was trained under instead of silently
        # falling back to the default.
        self.hard_v_margin  = float(p.get('velocity_box_margin',
                                          p.get('hard_v_margin', 0.9)))
        self.hard_q_margin  = float(p.get('position_margin_rad',
                                          p.get('hard_q_margin', 0.05)))
        self.hard_brake_eta = float(p.get('position_brake_eta',
                                          p.get('hard_brake_eta', 0.6)))
        self.slew_delta     = float(p.get('max_qddot_delta', 5.0))
        self.dt = float(dt)

        # Box shape knobs the robot gained in f5a59f8 / 4606e39. Defaults are
        # the robot's shipped values, so a config.yaml frozen next to an older
        # model still reproduces the shield it was trained under only if it
        # carries them explicitly — which is why they are in the sync test.
        self.relax_dt        = p.get('state_box_relax_s', 0.10)
        self.clip_to_limits  = bool(p.get('accel_box_clip_to_limits', True))
        self.fw_envelope     = bool(p.get('firmware_envelope', True))

        # Workspace box (hard rows on the EE point).
        self.ws_enable  = bool(p.get('ws_enable', True))
        self.ws_min     = np.asarray(p.get('ws_min', [0.05, -0.60, 0.05]), float)
        self.ws_max     = np.asarray(p.get('ws_max', [0.75, 0.60, 0.95]), float)
        self.ws_margin  = float(p.get('ws_margin', 0.02))
        self.ws_horizon = float(p.get('ws_horizon', 0.25))

        self.qddot_max = np.asarray(qddot_max, float)
        self.qdot_max  = np.asarray(qdot_max, float)

        # EFFECTIVE position limits, not the mechanical ones (robot: commit
        # f5a59f8, cbf_safety_filter.__init__). The firmware's velocity
        # envelope reaches zero at a reference position INSIDE the mechanical
        # stop — 4.5205 rad on joint6 against a 4.6216 limit — so a barrier
        # anchored at the mechanical limit sits behind the wall the robot
        # actually has.
        if self.fw_envelope:
            self.q_min = np.maximum(np.asarray(q_min, float), FR3_VEL_Q_REF_LOWER)
            self.q_max = np.minimum(np.asarray(q_max, float), FR3_VEL_Q_REF_UPPER)
        else:
            self.q_min = np.asarray(q_min, float)
            self.q_max = np.asarray(q_max, float)

        # ACCELERATION AUTHORITY, capped at what the arm can actually produce.
        #
        # The robot builds its box from franka_description's `deceleration_limit`
        # — the only q̈ scale the vendor publishes — which reads 17 rad/s² on
        # joints 5 and 7, 70 % above libfranka's kMaxJointAcceleration of 10.
        # Commit 4606e39 capped it at `qddot_max_abs: 10.0` after measuring
        # q̈_safe saturating at ±17 on the wrist while the realised acceleration
        # lagged by up to 18 rad/s².
        #
        # NOTE the asymmetry, and that it is deliberate: the cap applies to the
        # BOX only. `qddot_max` (the action scale, q̈_nom = a·q̈_max) keeps the
        # uncapped 17, exactly as on the robot, where rl_policy_commander scales
        # by fr3_control.yaml's joint_limits and cbf_safety_filter's box then
        # clips. Capping both would change what a = 1 means and would NOT
        # mirror hardware.
        cap = p.get('qddot_max_abs')
        qdd_box = (np.minimum(self.qddot_max, float(cap)) if cap is not None
                   else self.qddot_max.copy())
        self._acc_lb   = -qdd_box
        self._acc_ub   =  qdd_box
        self.qddot_box = qdd_box

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
            # adaptive_rho_interval PINNED, and this is a reproducibility fix,
            # not a tuning choice.
            #
            # OSQP's default is 0, which means "re-adapt rho on a schedule
            # derived from the measured SETUP TIME" (verified against the
            # installed osqp 0.6.7: adaptive_rho=1, adaptive_rho_interval=0,
            # adaptive_rho_fraction=0.4). The solver's iteration path is then
            # a function of wall-clock timing, not only of its inputs — so two
            # runs of the same seed on the same machine can return different
            # q̈_safe, and `--seed` stops guaranteeing a reproducible run.
            #
            # Measured: with the default, a no-op regression test comparing two
            # identical 40-step rollouts failed about 2 runs in 5 under pytest
            # (max divergence 1.6e-4, amplified from ~5e-6 by the closed loop),
            # while never failing in a plain script — pytest's capture and
            # import work perturb exactly the timing OSQP samples. With the
            # interval pinned it passes bit-exactly, repeatedly.
            #
            # 25 is OSQP's own `check_termination` default, i.e. the cadence it
            # already evaluates residuals on. This changes the iteration
            # SCHEDULE, never the problem: the solution still satisfies the same
            # eps_abs/eps_rel = 1e-3 tolerance.
            #
            # NOTE: cbf_safety_filter.py on the robot still uses the default.
            # The QP it solves is therefore reproducible only up to solver
            # tolerance — worth knowing when a CBFDIAG line is compared across
            # runs, though it is well below every safety margin here.
            prob.setup(P=self._P_csc, q=self._qvec, A=A, l=l, u=u,
                       warm_start=True, max_iter=self.max_iter, verbose=False,
                       adaptive_rho_interval=25)
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
            brake_eta=self.hard_brake_eta, dt=self.dt,
            relax_dt=self.relax_dt, clip_to_limits=self.clip_to_limits,
            firmware_envelope=self.fw_envelope)
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
