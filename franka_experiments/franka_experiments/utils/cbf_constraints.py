from dataclasses import dataclass, field
import numpy as np
from franka_experiments.utils.cbf_kinematics import CBFKinematics


@dataclass
class CBFParams:
    k0: float = 25.0
    k1: float = 10.0
    d_safe_default: float = 0.15
    d_safe_per_link: dict = field(default_factory=dict)
    dynamic_human: bool = False


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
        p_i_dot = Jp @ qdot
        p_h_dot = np.asarray(
            (p_h_dot_est or {}).get(link_name, np.zeros(3)), dtype=np.float64)

        D_s = cbf_params.d_safe_per_link.get(link_name, cbf_params.d_safe_default)
        h    = d_num - D_s
        rel_vel = p_i_dot - p_h_dot
        hdot = float(n_i @ rel_vel)

        n_dot = (rel_vel - n_i * (n_i @ rel_vel)) / d_num
        p_h_ddot = np.asarray(
            (p_h_ddot_est or {}).get(link_name, np.zeros(3)), dtype=np.float64)
        c_i = float(n_i @ (Jpd @ qdot - p_h_ddot) + n_dot @ rel_vel)

        A_row = (n_i @ Jp).astype(np.float64)   # shape (nv,)
        b_val = float(-c_i - cbf_params.k1 * hdot - cbf_params.k0 * h)

        if not (np.all(np.isfinite(A_row)) and np.isfinite(b_val)):
            continue

        rows_A.append(A_row)
        rows_b.append(b_val)
        meta.append({
            'link': link_name, 'h': h, 'hdot': hdot,
            'c_i': c_i, 'distance': d_num, 'zone': item.get('zone', ''),
        })

    nv = kin.model.nv
    if not rows_A:
        return np.zeros((0, nv)), np.zeros(0), meta

    return np.vstack(rows_A), np.asarray(rows_b, dtype=np.float64), meta
