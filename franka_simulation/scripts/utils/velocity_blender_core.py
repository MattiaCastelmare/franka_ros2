"""Core logic for `velocity_control_blender.py` (ROS-agnostic).

Goal
----
Keep the ROS2 node readable at a high level by moving the low-level math and
polyline bookkeeping here.

Design constraints
------------------
This module is written to preserve the original behavior as closely as possible:
- same formulas
- same try/except guards
- same thresholds and corner-case handling

It deliberately does **not** import rclpy or ROS message types.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy as np

from .avoidance_math import enforce_halfspace_with_box
from .risk_staging import compute_risk_staging, ramp_down_distance


def compute_polyline_arc_lengths(points: Sequence[np.ndarray]) -> Optional[np.ndarray]:
    """Compute cumulative arc-length for a joint-space polyline.

    Mirrors the try/except behavior of the node.
    """
    try:
        pts = list(points)
        s = [0.0]
        for i in range(1, len(pts)):
            ds = float(np.linalg.norm(pts[i] - pts[i - 1]))
            s.append(s[-1] + ds)
        return np.array(s, dtype=float)
    except Exception:
        return None


def nearest_point_on_polyline(
    *,
    q: np.ndarray,
    pts: Sequence[np.ndarray],
    s_cum: Optional[np.ndarray],
    i0: int,
    i1: int,
) -> tuple[int, float, float, np.ndarray, float]:
    """Nearest point projection of q onto polyline segments [i0..i1].

    Returns (best_i, best_alpha, best_s, best_qproj, best_d2)
    where the nearest point lies on segment i->i+1 at interpolation alpha in [0,1].

    This is the extracted version of `SimpleVelocityBlender._nearest_point_on_polyline`.
    """
    pts = list(pts)
    n = int(len(pts))
    if n <= 0:
        return 0, 0.0, 0.0, q.copy(), float("inf")
    if n == 1:
        dq = pts[0] - q
        return (
            0,
            0.0,
            float(s_cum[0]) if s_cum is not None else 0.0,
            pts[0].copy(),
            float(dq @ dq),
        )

    i0 = int(max(0, min(n - 2, i0)))
    i1 = int(max(i0, min(n - 2, i1)))

    best_i = i0
    best_a = 0.0
    best_d2 = float("inf")
    best_qp = pts[i0].copy()
    best_s = float(s_cum[i0]) if s_cum is not None else 0.0

    qv = q.reshape(-1)

    for i in range(i0, i1 + 1):
        p0 = pts[i]
        p1 = pts[i + 1]
        v = p1 - p0
        vv = float(v @ v)
        if vv < 1e-12:
            a = 0.0
            qp = p0
        else:
            a = float(((qv - p0) @ v) / vv)
            a = float(np.clip(a, 0.0, 1.0))
            qp = p0 + a * v

        d = qp - qv
        d2 = float(d @ d)
        if d2 < best_d2:
            best_d2 = d2
            best_i = i
            best_a = a
            best_qp = qp
            if s_cum is not None:
                seg_len = float(np.linalg.norm(v))
                best_s = float(s_cum[i]) + a * seg_len
            else:
                best_s = float(i) + a

    return best_i, best_a, best_s, best_qp, best_d2


def interpolate_at_s(
    *,
    pts: Sequence[np.ndarray],
    s_cum: Optional[np.ndarray],
    s_query: float,
    n_dof: int = 7,
) -> np.ndarray:
    """Interpolate polyline at arc-length s_query (joint-space).

    Extracted from `SimpleVelocityBlender._interpolate_at_s`.
    """
    pts = list(pts)
    n = int(len(pts))
    if n <= 0:
        return np.zeros(int(n_dof), dtype=float)
    if n == 1 or s_cum is None or len(s_cum) != n:
        return pts[-1].copy()

    s0 = float(s_cum[0])
    sN = float(s_cum[-1])
    s = float(np.clip(s_query, s0, sN))
    j = int(np.searchsorted(s_cum, s, side="right") - 1)
    j = int(max(0, min(n - 2, j)))

    sj0 = float(s_cum[j])
    sj1 = float(s_cum[j + 1])
    if (sj1 - sj0) < 1e-12:
        return pts[j + 1].copy()
    a = (s - sj0) / (sj1 - sj0)
    return (1.0 - a) * pts[j] + a * pts[j + 1]


@dataclass
class EmergencyRecoveryState:
    emergency_active: bool = False
    recovery_until_wall: float = 0.0


@dataclass
class EmergencyParams:
    emergency_enable: bool
    emergency_enter_m: float
    emergency_exit_m: float
    emergency_d_dot: float
    emergency_max_vel_fraction: float
    emergency_hazard_prefixes: Sequence[str]

    recovery_enable: bool
    recovery_time_s: float


def emergency_override(
    *,
    now_wall: float,
    d: float,
    j_row: np.ndarray,
    hazard: str,
    j_norm: float,
    max_vel: float,
    state: EmergencyRecoveryState,
    params: EmergencyParams,
) -> tuple[bool, Optional[np.ndarray], EmergencyRecoveryState, bool]:
    """Emergency override step.

    Returns:
      (handled, qdot_cmd, new_state, reset_smoothing)

    - handled=True with qdot_cmd means: publish immediately and return.
    - reset_smoothing=True means the caller should reset any velocity LPF state.

    Mirrors the original logic block in the node.
    """
    hazard = str(hazard or "none")

    hazard_ok = True
    if params.emergency_hazard_prefixes and hazard != "none":
        hazard_ok = any(hazard.startswith(p) for p in params.emergency_hazard_prefixes)

    reset_smoothing = False

    if params.emergency_enable and hazard_ok and (float(j_norm) > 1e-6):
        # Hysteresis on emergency state
        if (not state.emergency_active) and (float(d) <= float(params.emergency_enter_m)):
            state.emergency_active = True
            reset_smoothing = True

        if state.emergency_active:
            # Exit condition
            if float(d) >= float(params.emergency_exit_m):
                state.emergency_active = False
                if params.recovery_enable and (float(params.recovery_time_s) > 0.0):
                    state.recovery_until_wall = float(now_wall) + float(params.recovery_time_s)
            else:
                # Escape velocity: minimal-norm solution to enforce d_dot >= emergency_d_dot
                d_dot_des = max(0.0, float(params.emergency_d_dot))
                alpha = d_dot_des / (float(j_norm) * float(j_norm) + 1e-8)
                qdot_escape = float(alpha) * np.array(j_row, dtype=float).reshape(-1)

                # Stronger escape if deeper than enter threshold
                try:
                    if float(params.emergency_enter_m) > 1e-6:
                        depth = max(0.0, float(params.emergency_enter_m) - float(d))
                        depth_gain = 1.0 + 4.0 * (depth / float(params.emergency_enter_m))
                        qdot_escape *= float(np.clip(depth_gain, 1.0, 5.0))
                except Exception:
                    pass

                maxv = float(max_vel) * float(max(0.1, float(params.emergency_max_vel_fraction)))
                qdot_escape = np.clip(qdot_escape, -maxv, +maxv)

                return True, qdot_escape, state, reset_smoothing

    return False, None, state, reset_smoothing


@dataclass
class InfluenceParams:
    n_dof: int
    d_infl: float
    d_safe: float

    max_vel: float

    # Blend shaping
    avoidance_weight_max: float
    slowdown_factor_max: float
    slowdown_gamma_min: float

    # Unified risk staging thresholds (30/20/10/5 cm) + conservative inflation
    distance_inflation: float
    risk_d_far: float
    risk_d_mid: float
    risk_d_near: float
    stop_distance: float

    d_dot_min_far: float
    d_dot_min_close: float

    cbf_enable: bool
    cbf_kappa: float
    cbf_projection_iters: int
    cbf_eps: float

    d_dot_push_gain: float
    d_dot_push_max: float

    use_avoidance_velocity: bool
    avoidance_normal_only: bool
    avoidance_tangent_weight: float

    null_boost_max: float
    avoidance_ratio_max: float
    avoidance_repulsion_cap_fraction: float
    ns_floor_fraction: float

    normal_correction_max: float

    recovery_enable: bool
    recovery_tangent_speed: float

    tangent_escape_enable: bool
    tangent_escape_speed: float
    tangent_escape_err_min: float

    tangential_cmd_max_fraction: float

    diag_cmd_norm_eps: float


def compute_influence_zone_command(
    *,
    now_wall: float,
    d: float,
    j_row: np.ndarray,
    j_norm: float,
    qdot_tracking: np.ndarray,
    qdot_avoid: np.ndarray,
    avoid_norm: float,
    error_norm: float,
    threshold: float,
    recovery_until_wall: float,
    stall_detected: bool = False,
    params: InfluenceParams,
) -> tuple[np.ndarray, Dict[str, Any]]:
    """Compute qdot_des in the influence zone.

    Returns (qdot_des, dbg). The `dbg` dict mirrors the node fields.

    This function contains the extracted contents of the big `else:` block.
    """
    n_dof = int(params.n_dof)

    dbg: Dict[str, Any] = {
        "active": True,
        "err_norm": float(error_norm),
        "d": float(d),
        "j_norm": float(j_norm),
        "w_d": 0.0,
        "gamma": 1.0,
        "d_dot": 0.0,
        "d_dot_min": 0.0,
        "track_norm": float(np.linalg.norm(qdot_tracking)),
        "avoid_norm": float(avoid_norm),
    }

    # If no sensible avoidance info -> tracking only (caller typically checks too)
    if (float(d) >= float(params.d_infl)) or (float(j_norm) < 1e-6):
        return np.array(qdot_tracking, dtype=float).reshape(n_dof), dbg

    d_safe = float(params.d_safe)
    d_infl = float(params.d_infl)

    # Projectors for the distance normal and its nullspace
    J = np.array(j_row, dtype=float).reshape(1, n_dof)
    JT = J.T
    denom = float(J @ JT) + 1e-8
    P = (JT @ J) / denom
    N = np.eye(n_dof) - P

    # Unified risk staging (based on *effective* distance).
    d_eff = float(d) - float(max(0.0, float(params.distance_inflation)))
    staging = compute_risk_staging(
        d_eff=float(d_eff),
        d_far=float(params.risk_d_far),
        d_mid=float(params.risk_d_mid),
        d_near=float(params.risk_d_near),
        d_stop=float(params.stop_distance),
    )
    w_d = float(staging.w_total)
    dbg["w_d"] = float(w_d)
    dbg["d_eff"] = float(d_eff)
    dbg["w_far"] = float(staging.w_far)
    dbg["w_mid"] = float(staging.w_mid)
    dbg["w_near"] = float(staging.w_near)

    # 1) Base: keep tracking (towards goal)
    qdot_des = np.array(qdot_tracking, dtype=float).reshape(n_dof).copy()

    # 2) Add avoidance contribution (repulsive normal + tangential)
    if params.use_avoidance_velocity and (float(avoid_norm) > 1e-6):
        qdot_avoid_v = np.array(qdot_avoid, dtype=float).reshape(n_dof)

        # Split avoidance into normal/tangential w.r.t. the current distance gradient
        qdot_avoid_n = P @ qdot_avoid_v
        qdot_avoid_t = N @ qdot_avoid_v

        qdot_avoid_use_n = qdot_avoid_n if params.avoidance_normal_only else qdot_avoid_v

        w_rep = float(params.avoidance_weight_max) * float(w_d)
        w_tan = float(params.avoidance_tangent_weight) * float(w_d)

        ns_norm = float(np.linalg.norm(N @ qdot_des))
        rep_vec = float(w_rep) * qdot_avoid_use_n
        rep_norm = float(np.linalg.norm(rep_vec))

        # Absolute cap on repulsion magnitude
        try:
            cap_frac = float(np.clip(float(params.avoidance_repulsion_cap_fraction), 0.0, 2.0))
            rep_cap = cap_frac * float(params.max_vel)
            if (rep_cap > 0.0) and (rep_norm > rep_cap) and (rep_norm > 1e-9):
                rep_vec *= (rep_cap / rep_norm)
                rep_norm = float(np.linalg.norm(rep_vec))
        except Exception:
            pass

        # Ratio limiter with floor
        ns_floor = max(float(params.ns_floor_fraction) * float(params.max_vel), 1e-3)
        rep_max = float(params.avoidance_ratio_max) * (max(ns_norm, ns_floor) + 1e-6)
        if rep_norm > rep_max and rep_norm > 1e-9:
            rep_vec *= (rep_max / rep_norm)

        qdot_des = qdot_des + rep_vec + (float(w_tan) * qdot_avoid_t)

    # 3) Optional: increase tangential progress near obstacle
    null_boost = 1.0 + float(params.null_boost_max) * float(w_d)
    qdot_des = (P @ qdot_des) + float(null_boost) * (N @ qdot_des)

    # 4) Enforce lower bound on d_dot across the influence region (risk-staged)
    d_dot_min = (1.0 - float(w_d)) * float(params.d_dot_min_far) + float(w_d) * float(params.d_dot_min_close)

    if params.cbf_enable:
        try:
            cbf_min = -float(params.cbf_kappa) * float(float(d) - float(d_safe))
            d_dot_min = max(float(d_dot_min), float(cbf_min))
        except Exception:
            pass

    # Extra push only when inside the safety margin.
    # Use a smooth ramp so we don't introduce a discontinuity at d_safe (reduces jitter).
    if float(d_eff) < float(d_safe):
        pen = max(0.0, float(d_safe) - float(d_eff))
        # 0 at d_safe, 1 near stop_distance.
        try:
            r = float(
                ramp_down_distance(
                    d=float(d_eff),
                    d_hi=float(d_safe),
                    d_lo=float(params.stop_distance),
                )
            )
        except Exception:
            r = 1.0
        push = float(params.d_dot_push_gain) * float(pen) * float(np.clip(r, 0.0, 1.0))
        push = float(np.clip(push, 0.0, float(params.d_dot_push_max)))
        d_dot_min = max(float(d_dot_min), float(push))

    d_dot = float(np.array(j_row, dtype=float).reshape(-1) @ qdot_des)
    dbg["d_dot"] = float(d_dot)
    dbg["d_dot_min"] = float(d_dot_min)

    if float(d_dot) < float(d_dot_min):
        lambda_corr = (float(d_dot_min) - float(d_dot)) / (float(j_norm) * float(j_norm) + 1e-8)
        corr = float(lambda_corr) * np.array(j_row, dtype=float).reshape(-1)
        corr_norm = float(np.linalg.norm(corr))
        if corr_norm > float(params.normal_correction_max):
            corr *= float(params.normal_correction_max) / (corr_norm + 1e-9)
        qdot_des = qdot_des + corr

    # 5) Global slowdown
    gamma = 1.0 - float(params.slowdown_factor_max) * float(w_d)
    gamma = max(float(params.slowdown_gamma_min), float(gamma))
    qdot_des *= float(gamma)
    dbg["gamma"] = float(gamma)

    # 5b) Recovery tangential injection
    if params.recovery_enable and (float(now_wall) < float(recovery_until_wall)):
        try:
            t_vec = N @ np.array(qdot_tracking, dtype=float).reshape(n_dof)
            t_n = float(np.linalg.norm(t_vec))
            if t_n < 1e-9 and (float(avoid_norm) > 1e-9):
                t_vec = N @ np.array(qdot_avoid, dtype=float).reshape(n_dof)
                t_n = float(np.linalg.norm(t_vec))
            if t_n < 1e-9:
                t_vec = np.zeros(n_dof)
                for k in range(n_dof):
                    ei = np.zeros(n_dof)
                    ei[k] = 1.0
                    cand = N @ ei
                    cn = float(np.linalg.norm(cand))
                    if cn > 1e-6:
                        t_vec = cand
                        t_n = cn
                        break
            if t_n > 1e-9:
                t_dir = t_vec / (t_n + 1e-9)
                qdot_des = qdot_des + (float(params.recovery_tangent_speed) * float(w_d)) * t_dir
        except Exception:
            pass

    # 6) Anti-stall tangential escape
    # IMPORTANT: gate on progress-based stall detection to avoid injecting tangential motion
    # during normal smooth avoidance.
    if params.tangent_escape_enable and bool(stall_detected):
        try:
            if (float(w_d) > 1e-3) and (float(error_norm) > float(max(float(threshold), float(params.tangent_escape_err_min)))):
                des_n = float(np.linalg.norm(qdot_des))
                if des_n < float(params.diag_cmd_norm_eps):
                    t_vec = N @ np.array(qdot_tracking, dtype=float).reshape(n_dof)
                    t_n = float(np.linalg.norm(t_vec))
                    if t_n < 1e-9:
                        try:
                            t_vec = N @ np.array(qdot_avoid, dtype=float).reshape(n_dof)
                            t_n = float(np.linalg.norm(t_vec))
                        except Exception:
                            t_vec = np.zeros(n_dof)
                            t_n = 0.0

                    if t_n < 1e-9:
                        t_vec = np.zeros(n_dof)
                        for k in range(n_dof):
                            ei = np.zeros(n_dof)
                            ei[k] = 1.0
                            cand = N @ ei
                            cn = float(np.linalg.norm(cand))
                            if cn > 1e-6:
                                t_vec = cand
                                t_n = cn
                                break

                    if t_n > 1e-9:
                        t_dir = t_vec / (t_n + 1e-9)
                        qdot_des = qdot_des + (float(params.tangent_escape_speed) * float(w_d)) * t_dir
        except Exception:
            pass

    # Final: cap tangential component in the distance-jacobian nullspace.
    qdot_des = _cap_tangential_component(
        qdot=np.array(qdot_des, dtype=float).reshape(n_dof),
        N=N,
        max_vel=float(params.max_vel),
        w_d=float(w_d),
        tangential_cmd_max_fraction=float(params.tangential_cmd_max_fraction),
    )

    return qdot_des, dbg


def _cap_tangential_component(
    *,
    qdot: np.ndarray,
    N: np.ndarray,
    max_vel: float,
    w_d: float,
    tangential_cmd_max_fraction: float,
) -> np.ndarray:
    """Cap the nullspace (tangential) component norm to avoid dominance/chattering."""
    try:
        qdot = np.array(qdot, dtype=float).reshape(-1)
        t = N @ qdot
        t_n = float(np.linalg.norm(t))

        frac = float(np.clip(float(tangential_cmd_max_fraction), 0.0, 2.0))
        # Allow more tangential near obstacles, but still bounded.
        cap = float(frac) * float(max_vel) * float(0.30 + 0.70 * float(np.clip(w_d, 0.0, 1.0)))
        if (cap > 0.0) and (t_n > cap) and (t_n > 1e-9):
            t = (cap / t_n) * t
        return (qdot - (N @ qdot)) + t
    except Exception:
        return np.array(qdot, dtype=float).reshape(-1)


def apply_output_filter_and_constraints(
    *,
    qdot_des: np.ndarray,
    qdot_prev: np.ndarray,
    velocity_filter_beta: float,
    velocity_filter_beta_near: float,
    max_vel: float,
    # constraint inputs
    d: float,
    d_infl: float,
    j_row: np.ndarray,
    j_norm: float,
    cbf_projection_iters: int,
    cbf_eps: float,
    normal_correction_max: float,
    qdot_tracking_hint: Optional[np.ndarray],
    dbg: Dict[str, Any],
) -> tuple[np.ndarray, np.ndarray, Dict[str, Any]]:
    """LPF + saturation + robust halfspace enforcement.

    Extracted from the final section of the node.

    Returns (qdot, qdot_prev_new, dbg).
    """
    # Risk-scaled LPF: close to hazards -> less lag (beta -> beta_near).
    w = float(np.clip(float(dbg.get("w_d", 0.0)), 0.0, 1.0))
    b_far = float(np.clip(float(velocity_filter_beta), 0.0, 1.0))
    b_near = float(np.clip(float(velocity_filter_beta_near), 0.0, 1.0))
    beta = float(b_far + (b_near - b_far) * w)
    qdot = beta * np.array(qdot_des, dtype=float).reshape(-1) + (1.0 - beta) * np.array(qdot_prev, dtype=float).reshape(-1)
    qdot_prev_new = qdot.copy()

    qdot = np.clip(qdot, -float(max_vel), float(max_vel))

    if (float(d) < float(d_infl)) and (float(j_norm) > 1e-6):
        try:
            j = np.array(j_row, dtype=float).reshape(-1)
            d_dot_min_eff = float(dbg.get("d_dot_min", 0.0))

            # Feasibility guard: under the box constraint |qdot_i|<=max_vel, the maximum
            # achievable distance rate is max_vel * sum(|j_i|).
            d_dot_max = float(max_vel) * float(np.sum(np.abs(j)))
            if d_dot_max <= 1e-9:
                d_dot_min_eff = float(d_dot_min_eff)
            else:
                if float(d_dot_min_eff) > float(d_dot_max):
                    # Soft-infeasible: request the maximum achievable bound and keep going.
                    # This avoids an abrupt jump to qdot = max_vel*sign(j) (which can look like "folding").
                    d_dot_min_eff = float(d_dot_max) - 1e-6
                    dbg["cbf_infeasible"] = True
                else:
                    # Otherwise clamp slightly below the true maximum to help convergence.
                    d_dot_min_eff = float(min(float(d_dot_min_eff), float(d_dot_max) - 1e-6))

            qdot, ok = enforce_halfspace_with_box(
                qdot_des=qdot,
                j_row=j,
                d_dot_min=float(d_dot_min_eff),
                max_abs_vel=float(max_vel),
                iters=int(cbf_projection_iters),
                eps=float(cbf_eps),
                correction_l2_max=float(normal_correction_max),
            )
            # If we failed, try again without a correction cap (safety-first near obstacles).
            if not bool(ok):
                qdot2, ok2 = enforce_halfspace_with_box(
                    qdot_des=qdot,
                    j_row=j,
                    d_dot_min=float(d_dot_min_eff),
                    max_abs_vel=float(max_vel),
                    iters=max(2, int(cbf_projection_iters) * 2),
                    eps=float(cbf_eps),
                    correction_l2_max=None,
                )
                if bool(ok2):
                    qdot = qdot2
                    ok = True

            # If still not ok: safety-first escape (maximize d_dot) + optional nullspace tracking.
            if not bool(ok):
                qdot = float(max_vel) * np.sign(j)
                if qdot_tracking_hint is not None:
                    try:
                        qt = np.array(qdot_tracking_hint, dtype=float).reshape(-1)
                        jn2 = float(j @ j) + 1e-8
                        N = np.eye(int(qt.shape[0])) - (np.outer(j, j) / jn2)
                        t = N @ qt
                        t_n = float(np.linalg.norm(t))
                        t_cap = 0.5 * float(max_vel)
                        if (t_cap > 0.0) and (t_n > t_cap) and (t_n > 1e-9):
                            t *= (t_cap / t_n)
                        qdot = np.clip(qdot + t, -float(max_vel), float(max_vel))
                    except Exception:
                        pass
                dbg["cbf_ok"] = False
                dbg["cbf_infeasible_hard"] = True
                return qdot, qdot_prev_new, dbg

            dbg["cbf_ok"] = bool(ok)
        except Exception:
            dbg["cbf_ok"] = False

    return qdot, qdot_prev_new, dbg
