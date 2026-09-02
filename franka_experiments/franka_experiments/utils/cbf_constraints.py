
# TODO[LEGACY]: no importer; velocity-era ZCBF rows (b = -gamma*h), not the HOCBF the stack uses | confidence: high | superseded-by: cbf_safety_filter HOCBF rows | flagged: 2026-09-01
from dataclasses import dataclass, field
import numpy as np
from franka_experiments.utils.cbf_kinematics import CBFKinematics


# ── Simple first-order acceleration-space CBF ─────────────────────────────────
#
# Barrier:    h = d - d_safe
# Constraint: aᵀ(qdot + qddot·dt) >= -gamma·h
# where:      a = nᵀ·J_point
#
# Rearranged into QP form: (a·dt)·qddot >= -gamma·h - a·qdot
#
# This is a ZCBF lifted to acceleration space.
# No Hessians, no second-order Lie derivatives, no c_i term.
# ─────────────────────────────────────────────────────────────────────────────

def build_simple_accel_cbf_constraints(
    kin: CBFKinematics,
    q: np.ndarray,
    qdot: np.ndarray,
    link_distances: list,
    dt: float,
    d_safe: float,
    gamma: float,
) -> tuple[np.ndarray, np.ndarray, list]:
    """Build simple acceleration-space CBF constraints.

    For each closest-point pair (robot link ↔ obstacle):
      h     = d - d_safe
      a     = n @ J_point          (nv,)
      A_row = a * dt               (nv,)
      b_row = -gamma*h - a @ qdot  scalar

    Returns:
      A    (n_c × nv) — constraint matrix, A @ qddot >= b
      b    (n_c,)     — constraint RHS
      meta list of dicts with: link, h, hdot, distance, condJ, zone
    """
    kin.update(q, qdot)
    rows_A, rows_b, meta = [], [], []

    for item in link_distances:
        if not item.get('valid', False):
            continue
        if item.get('confidence', 1.0) < 0.2:
            continue

        # --- geometry ---
        p_robot = np.asarray(item['closest_point_robot'], dtype=np.float64)
        p_human = np.asarray(item['closest_point_human'], dtype=np.float64)
        delta_p = p_robot - p_human
        d_num   = float(np.linalg.norm(delta_p))
        if d_num < 1e-8:
            continue
        n_world = delta_p / d_num   # unit normal pointing away from obstacle

        link_name = item['link_name']
        fid = kin.resolve_frame_id(link_name)
        if fid is None:
            continue

        # --- Jacobian at closest point (Jpd ignored — no second-order terms) ---
        Jp, _ = kin.point_jacobian(fid, p_robot)

        condJ = float(np.linalg.cond(Jp))
        if condJ > 1e5:
            continue   # near-singular — skip this constraint

        # --- CBF constraint ---
        h    = d_num - d_safe
        a    = (n_world @ Jp).astype(np.float64)   # (nv,)
        hdot = float(a @ qdot)                     # approximate distance rate

        A_row = a * dt
        b_row = float(-gamma * h - a @ qdot)

        if not (np.all(np.isfinite(A_row)) and np.isfinite(b_row)):
            continue

        rows_A.append(A_row)
        rows_b.append(b_row)
        meta.append({
            'link':     link_name,
            'h':        h,
            'hdot':     hdot,
            'distance': d_num,
            'condJ':    condJ,
            'zone':     item.get('zone', ''),
        })

    nv = kin.model.nv
    if not rows_A:
        return np.zeros((0, nv)), np.zeros(0), meta

    return np.vstack(rows_A), np.asarray(rows_b, dtype=np.float64), meta


# ── HOCBF (second-order) — preserved for future reintroduction ───────────────
# The functions below implement the full Higher-Order CBF (HOCBF / ECBF).
# cbf_safety_filter.py currently does NOT use these.
# ─────────────────────────────────────────────────────────────────────────────

@dataclass
class CBFParams:
    k0: float = 25.0
    k1: float = 10.0
    d_safe_default: float = 0.15
    d_safe_per_link: dict = field(default_factory=dict)
    dynamic_human: bool = False
    delta_base: float = 0.0      # robust margin base [m]
    k_delta_vel: float = 0.0     # velocity-dependent margin gain [m·s/rad]
    h_activation_threshold: float = 0.5  # [m] skip constraint when h > threshold


def build_hocbf_constraints(
    kin: CBFKinematics,
    q: np.ndarray,
    qdot: np.ndarray,
    link_distances: list,
    p_h_dot_est: dict | None,
    p_h_ddot_est: dict | None,
    cbf_params: CBFParams,
) -> tuple[np.ndarray, np.ndarray, list]:
    """Build HOCBF constraint matrix A and vector b such that A @ qddot >= b.

    link_distances: list of dicts with keys:
        link_name, closest_point_robot, closest_point_human,
        distance, valid, confidence, zone
    """
    kin.update(q, qdot)
    rows_A, rows_b, meta = [], [], []

    delta = cbf_params.delta_base + cbf_params.k_delta_vel * float(np.linalg.norm(qdot))

    for item in link_distances:
        if not item.get('valid', False):
            continue
        if item.get('confidence', 1.0) < 0.2:
            continue

        p_i = np.asarray(item['closest_point_robot'], dtype=np.float64)
        p_h = np.asarray(item['closest_point_human'], dtype=np.float64)
        Delta = p_i - p_h
        d_num = float(np.linalg.norm(Delta))
        if d_num < 1e-8:
            continue
        n_i = Delta / d_num

        link_name = item['link_name']
        fid = kin.resolve_frame_id(link_name)
        if fid is None:
            continue

        Jp, Jpd = kin.point_jacobian(fid, p_i)

        condJ = float(np.linalg.cond(Jp))
        if condJ > 1e5:
            continue   # numerically singular — skip this constraint

        p_i_dot = Jp @ qdot
        p_h_dot = np.asarray(
            (p_h_dot_est or {}).get(link_name, np.zeros(3)), dtype=np.float64)

        D_s = cbf_params.d_safe_per_link.get(link_name, cbf_params.d_safe_default)
        h    = d_num - D_s - delta
        if h > cbf_params.h_activation_threshold:
            continue
        rel_vel = p_i_dot - p_h_dot
        hdot = float(n_i @ rel_vel)

        n_dot = (rel_vel - n_i * (n_i @ rel_vel)) / max(d_num, 0.05)
        p_h_ddot = np.asarray(
            (p_h_ddot_est or {}).get(link_name, np.zeros(3)), dtype=np.float64)
        c_i = float(np.clip(
            n_i @ (Jpd @ qdot - p_h_ddot) + n_dot @ rel_vel,
            -5.0, 5.0,
        ))

        A_row = (n_i @ Jp).astype(np.float64)   # shape (nv,)
        b_val = float(-c_i - cbf_params.k1 * hdot - cbf_params.k0 * h)

        if not (np.all(np.isfinite(A_row)) and np.isfinite(b_val)):
            continue

        rows_A.append(A_row)
        rows_b.append(b_val)
        meta.append({
            'link': link_name, 'h': h, 'hdot': hdot,
            'c_i': c_i, 'distance': d_num, 'zone': item.get('zone', ''),
            'condJ': condJ,
        })

    nv = kin.model.nv
    if not rows_A:
        return np.zeros((0, nv)), np.zeros(0), meta

    return np.vstack(rows_A), np.asarray(rows_b, dtype=np.float64), meta


def predict_state(
    q: np.ndarray,
    qdot: np.ndarray,
    qddot_last: np.ndarray,
    delay: float = 0.04,
) -> tuple[np.ndarray, np.ndarray]:
    """Predict joint state after a communication delay Δ.

    q_pred    = q + qdot·Δ + ½·qddot_last·Δ²
    qdot_pred = qdot + qddot_last·Δ

    With delay=0 returns (q, qdot) unchanged.
    """
    q_pred    = q    + qdot      * delay + 0.5 * qddot_last * delay ** 2
    qdot_pred = qdot + qddot_last * delay
    return q_pred, qdot_pred
