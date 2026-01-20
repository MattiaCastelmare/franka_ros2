"""CBF-QP safety filter (controller safety layer).

This module implements the safety filtering stage used by the online avoidance
controller:
- min-distance smoothing (for risk scaling)
- stop gate hysteresis
- CBF activation hysteresis
- constraint construction (via avoidance_math.build_cbf_constraints)
- QP solve (OSQP) with deterministic fallback projection
- output smoothing (risk-scaled LPF) and optional accel limiting

It is intentionally kept independent from rclpy.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, List, Optional, Tuple

import numpy as np

from .avoidance_math import (
    beta_lpf_from_distance,
    build_cbf_constraints,
    pocs_project_halfspaces_with_box,
    posture_reference,
    qp_gamma_from_distance,
    staged_risk_weight,
)


@dataclass(frozen=True)
class CbfFilterParams:
    # Timing / limits
    rate: float
    max_qdot: float

    # CBF distances
    cbf_d_safe: float
    cbf_d_buffer_in: float
    cbf_d_buffer_out: float

    # Risk zones
    risk_d_far: float
    risk_d_mid: float
    risk_d_near: float
    stop_d_in: float
    stop_d_out: float

    # Constraint building
    cbf_eps: float
    cbf_K: int
    cbf_approach_speed_limit: float
    cbf_alpha_min: float
    cbf_alpha_max: float

    # QP / fallback
    cbf_use_qp: bool
    cbf_qp_damping_min: float
    cbf_qp_damping_max: float

    # Output smoothing
    beta_lpf_far: float
    beta_lpf_near: float
    min_distance_lpf: float
    output_accel_limit: float

    # Optional posture bias
    posture_bias_gain: float
    posture_reference_param: list


@dataclass
class CbfFilterState:
    cbf_active: bool = False
    stop_gate_active: bool = False

    qdot_out_prev: np.ndarray = field(default_factory=lambda: np.zeros(7, dtype=float))
    qdot_pub_prev: np.ndarray = field(default_factory=lambda: np.zeros(7, dtype=float))
    qdot_qp_prev: np.ndarray = field(default_factory=lambda: np.zeros(7, dtype=float))

    d_min_filt: float = 999.0

    qp_last_status: str = "disabled"
    qp_last_slack_max: float = 0.0

    last_debug_log_ns: int = 0


def apply_cbf_qp_safety_filter(
    *,
    qdot_nom: np.ndarray,
    d_min: float,
    candidates: List[dict],
    model: Any,
    data: Any,
    q: np.ndarray,
    params: CbfFilterParams,
    state: CbfFilterState,
    qp_solver: Any = None,
    qp_available: bool = False,
) -> Tuple[np.ndarray, np.ndarray, int, Optional[dict], str, float, CbfFilterState]:
    """Apply CBF-QP safety filtering.

    Returns:
      (qdot_out, G, m_active, active_best, qp_status, slack_max, updated_state)
    """
    qdot_nom = np.array(qdot_nom, dtype=float).reshape(7)
    d_min = float(d_min)

    # --- Filter the min distance signal (used for risk scaling / diagnostics).
    d_beta = float(np.clip(float(params.min_distance_lpf), 0.0, 1.0))
    state.d_min_filt = float(d_beta * float(d_min) + (1.0 - d_beta) * float(state.d_min_filt))

    # --- Hard stop gate with hysteresis.
    if (not state.stop_gate_active) and (float(d_min) <= float(params.stop_d_in)):
        state.stop_gate_active = True
    elif state.stop_gate_active and (float(d_min) >= float(params.stop_d_out)):
        state.stop_gate_active = False

    # --- Optional posture bias near hazards.
    if float(params.posture_bias_gain) > 0.0:
        q_ref = posture_reference(params.posture_reference_param, model=model)
        if (q_ref is not None) and isinstance(q, np.ndarray) and (q.shape[0] >= 7):
            w_post = float(
                staged_risk_weight(
                    float(state.d_min_filt),
                    d_far=float(params.risk_d_far),
                    d_mid=float(params.risk_d_mid),
                    d_near=float(params.risk_d_near),
                    d_stop=float(params.stop_d_in),
                )
            )
            q_cur = np.array(q, dtype=float).reshape(-1)[:7]
            qdot_post = float(params.posture_bias_gain) * w_post * (q_ref - q_cur)
            qdot_nom = qdot_nom + qdot_post

    # --- CBF activation hysteresis (avoid chatter) using RAW effective distance.
    # This avoids a delayed reaction caused by min-distance filtering.
    d_act = float(d_min)
    d_in = float(max(float(params.cbf_d_buffer_in), float(params.risk_d_far)))
    d_out = float(max(float(params.cbf_d_buffer_out), float(params.risk_d_far)))

    if (not state.cbf_active) and (d_act <= d_in):
        state.cbf_active = True
    elif state.cbf_active and (d_act >= d_out):
        state.cbf_active = False

    active_thr = float(d_out) if state.cbf_active else float(d_in)

    # --- Build CBF constraints.
    G, b_cbf, m_active, active_best = build_cbf_constraints(
        list(candidates),
        float(active_thr),
        K=int(params.cbf_K),
        cbf_eps=float(params.cbf_eps),
        cbf_d_safe=float(params.cbf_d_safe),
        approach_speed_limit=float(params.cbf_approach_speed_limit),
        alpha_min=float(params.cbf_alpha_min),
        alpha_max=float(params.cbf_alpha_max),
        risk_d_far=float(params.risk_d_far),
        risk_d_mid=float(params.risk_d_mid),
        risk_d_near=float(params.risk_d_near),
        stop_distance=float(params.stop_d_in),
        model=model,
        data=data,
        q=q,
    )

    # --- Solve (QP preferred) or robust fallback.
    qdot_safe = None
    slack_max = 0.0
    qp_status = "inactive"

    if state.stop_gate_active:
        qdot_safe = np.zeros(7, dtype=float)
        slack_max = 0.0
        qp_status = "stop_gate"
    elif m_active <= 0:
        qdot_safe = qdot_nom
        qp_status = "no_constraints"
    else:
        gamma = float(
            qp_gamma_from_distance(
                float(state.d_min_filt),
                gamma_min=float(params.cbf_qp_damping_min),
                gamma_max=float(params.cbf_qp_damping_max),
                d_far=float(params.risk_d_far),
                d_mid=float(params.risk_d_mid),
                d_near=float(params.risk_d_near),
                d_stop=float(params.stop_d_in),
            )
        )

        if bool(params.cbf_use_qp) and bool(qp_available) and (qp_solver is not None):
            qp_res = qp_solver.solve(qdot_nom, G, b_cbf, gamma=gamma, qdot_prev=state.qdot_qp_prev)
            if qp_res is not None:
                qdot_safe, slack_max, qp_status = qp_res
            else:
                qdot_safe = None

        if qdot_safe is None:
            qdot_safe = pocs_project_halfspaces_with_box(
                qdot_nom=qdot_nom,
                G=G[:m_active, :],
                b=b_cbf[:m_active],
                max_abs_vel=float(params.max_qdot),
                iters=3,
                eps=float(params.cbf_eps),
            )
            slack_max = 0.0
            qp_status = "fallback_projection"

    # Remember previous (pre-LPF) safe output for QP damping
    try:
        state.qdot_qp_prev = np.array(qdot_safe, dtype=float).reshape(7)
    except Exception:
        pass

    # Joint velocity limits (box)
    qdot_safe = np.clip(qdot_safe, -float(params.max_qdot), +float(params.max_qdot))

    # Low-pass filter on output to reduce jitter (risk-scaled)
    beta = float(
        beta_lpf_from_distance(
            float(state.d_min_filt),
            beta_far=float(params.beta_lpf_far),
            beta_near=float(params.beta_lpf_near),
            d_far=float(params.risk_d_far),
            d_mid=float(params.risk_d_mid),
            d_near=float(params.risk_d_near),
            d_stop=float(params.stop_d_in),
        )
    )

    qdot_out = beta * qdot_safe + (1.0 - beta) * state.qdot_out_prev
    state.qdot_out_prev = np.array(qdot_out, dtype=float).reshape(7)

    # Optional acceleration (rate) limiting on the published command.
    acc_lim = float(params.output_accel_limit)
    if acc_lim > 0.0:
        dt = 1.0 / float(max(1.0, float(params.rate)))
        dq = qdot_out - state.qdot_pub_prev
        dq_max = float(acc_lim) * float(dt)
        dq = np.clip(dq, -dq_max, +dq_max)
        qdot_out = state.qdot_pub_prev + dq

    state.qdot_pub_prev = np.array(qdot_out, dtype=float).reshape(7)

    state.qp_last_status = str(qp_status)
    state.qp_last_slack_max = float(slack_max)

    return (
        np.array(qdot_out, dtype=float).reshape(7),
        np.array(G, dtype=float),
        int(m_active),
        active_best,
        str(qp_status),
        float(slack_max),
        state,
    )


def debug_throttled(
    *,
    logger: Any,
    now_ns: int,
    d_min_raw: float,
    m_active: int,
    params: CbfFilterParams,
    state: CbfFilterState,
) -> None:
    """Emit a 1Hz debug line matching the controller's previous output."""
    try:
        now_ns = int(now_ns)
        if now_ns - int(state.last_debug_log_ns) < 1_000_000_000:
            return
        state.last_debug_log_ns = now_ns

        w_dbg = float(
            staged_risk_weight(
                float(state.d_min_filt),
                d_far=float(params.risk_d_far),
                d_mid=float(params.risk_d_mid),
                d_near=float(params.risk_d_near),
                d_stop=float(params.stop_d_in),
            )
        )
        gamma_dbg = float(
            qp_gamma_from_distance(
                float(state.d_min_filt),
                gamma_min=float(params.cbf_qp_damping_min),
                gamma_max=float(params.cbf_qp_damping_max),
                d_far=float(params.risk_d_far),
                d_mid=float(params.risk_d_mid),
                d_near=float(params.risk_d_near),
                d_stop=float(params.stop_d_in),
            )
        )

        logger.debug(
            f"CBF-QP: d_min_raw={float(d_min_raw):.4f} d_min_filt={float(state.d_min_filt):.4f} "
            f"w={w_dbg:.2f} gamma={gamma_dbg:.2f} stop={state.stop_gate_active} "
            f"active={state.cbf_active} m_active={int(m_active)} status={state.qp_last_status} "
            f"slack_max={state.qp_last_slack_max:.3e}"
        )
    except Exception:
        pass
