"""ROS-facing helpers for `online_avoidance_controller.py`.

This module exists to keep the controller node readable at a high level.
It contains ROS-dependent glue and parameter boilerplate, while the algorithmic
parts remain in `avoidance_core.py` and `cbf_filter.py`.

Scope
-----
- declare/load parameters (with explicit casts, matching previous behavior)
- create CBF filter params/state and optional QP solver
- publish diagnostics and coherent "closest constraint" pair

Intentionally *not* included:
- distance computation / kinematics / potential-field math (kept ROS-agnostic)
"""

# NOTE
# ----
# This module is kept ONLY for backward compatibility with older imports.
# The controller has been refactored to use the split helpers:
#   - utils/params.py
#   - utils/logging.py
#   - utils/ros_publishers.py
#   - utils/closest_constraint.py
#   - utils/diagnostics.py
# New code should not import this module.

import warnings

warnings.warn(
    "utils.avoidance_ros_helpers is deprecated; use utils.params/utils.logging/utils.ros_publishers/utils.closest_constraint/utils.diagnostics",
    DeprecationWarning,
    stacklevel=2,
)

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy as np

from std_msgs.msg import Float64MultiArray, String

from .avoidance_math import OsqpCbfQpSolver, build_cbf_constraints
from .cbf_filter import CbfFilterParams, CbfFilterState


# -----------------------------------------------------------------------------
# Parameters
# -----------------------------------------------------------------------------


DEFAULT_NULLSPACE_AVOIDANCE_PARAMS: Dict[str, Any] = {
    # Control
    "control_rate": 100.0,

    # Distances / gains
    "influence_distance": 0.30,
    "safety_margin": 0.08,
    # When closer than this distance, avoidance becomes intentionally more aggressive.
    # This is *not* the same as safety_margin: it is a "start pushing hard" threshold.
    "aggressive_distance": 0.20,
    "aggressive_gain_scale": 3.0,
    # Overall scaling for the avoidance twist (used for both repulsive and tangential components)
    "nullspace_gain": 0.15,
    # Extra tangential (swirl) component to break local minima near obstacles.
    "tangential_gain": 0.20,
    "max_joint_velocity": 0.25,
    "excluded_obstacles": ["ground_plane", "ground", "floor", "plane"],

    # Capsule geometry tuning (m)
    "capsule_radii": [0.15, 0.12, 0.13],
    # Format: [p0_0, p1_0, p0_1, p1_1, p0_2, p1_2]
    "capsule_fractions": [0.00, 0.35, 0.25, 0.75, 0.60, 0.95],

    # Distance model knobs
    "box_projection_iters": 8,
    "repulsion_spread_enable": True,
    "repulsion_spread_samples": 5,  # odd number recommended (e.g., 3/5/7)
    "repulsion_spread_half_length": 0.10,  # m along the capsule segment

    # Extra safety layers (approximate but fast): ground + self-collision
    "enable_ground_avoidance": True,
    "ground_z": 0.0,  # world Z of the floor plane
    "ground_influence_distance": 0.15,
    "ground_safety_margin": 0.05,
    "ground_gain": 0.25,

    "enable_self_collision_avoidance": True,
    "self_influence_distance": 0.12,
    "self_safety_margin": 0.03,
    "self_gain": 0.25,
    # Skip capsule pairs belonging to links closer than this in the kinematic chain
    "self_skip_adjacent_links": 1,

    # ================= CBF-QP SAFETY FILTER =================
    # Barrier function: h = d - d_safe
    # Constraint: g^T qdot >= v_obs_proj - alpha*(d - d_safe)
    "d_safe": 0.08,  # [m] safety distance
    "d_buffer": 0.30,  # [m] activation distance (influence zone for the filter)
    "d_buffer_out": 0.0,  # [m] hysteresis exit threshold (0 -> auto)
    "alpha": 5.0,  # [1/s] CBF gain
    "max_constraints": 5,  # K (top-K closest hazards)
    "lambda_reg": 1e-6,  # regularization on qdot
    "rho_slack": 100.0,  # slack penalty
    "beta_lpf": 0.80,  # LPF on output qdot (kept for backward compatibility)
    "output_accel_limit": 0.0,  # [rad/s^2] 0 disables rate limiting
    "approach_speed_limit": 0.0,  # [m/s] 0 disables extra cap on negative d_dot
    "use_qp": True,  # try OSQP if available
    "eps": 1e-9,  # numerical epsilon
    "qp_weight_diag": [1.0] * 7,  # diagonal W in ||qdot-qdot_nom||_W

    # ===== Risk-scaled zones (30/20/10/5 cm) + Stop Gate =====
    "risk_d_far": 0.30,  # [m] start reacting
    "risk_d_mid": 0.20,  # [m] medium zone
    "risk_d_near": 0.10,  # [m] strong zone
    "stop_distance": 0.05,  # [m] hard stop enter
    "stop_release_distance": 0.06,  # [m] hard stop exit (hysteresis)

    # Risk-scaled CBF alpha: alpha(d) = lerp(alpha_min, alpha_max, w(d))
    # Defaults keep legacy behavior (alpha_min==alpha_max==alpha).
    "alpha_min": 5.0,  # [1/s]
    "alpha_max": 5.0,  # [1/s]

    # QP damping term: gamma(d) * ||qdot - qdot_prev||^2
    "qp_damping_min": 0.0,  # >= 0
    "qp_damping_max": 0.0,  # >= 0

    # Risk-scaled output LPF (beta near should be smaller for smoother motion)
    "beta_lpf_far": 0.80,  # 0..1
    "beta_lpf_near": 0.80,  # 0..1

    # Smoothing for published min distance signal (for downstream blending/visualization)
    "min_distance_lpf": 0.50,  # 0..1 (1.0 = no filtering)

    # Optional posture bias (OFF by default)
    "posture_bias_gain": 0.0,  # [1/s]
    "posture_reference": [],  # 7 values (radians)
}


def declare_nullspace_avoidance_parameters(node: Any) -> None:
    """Declare parameters (with defaults) on the node."""
    node.declare_parameters("", [(k, v) for k, v in DEFAULT_NULLSPACE_AVOIDANCE_PARAMS.items()])


@dataclass
class QpSetup:
    qp_solver: Any = None
    qp_available: bool = False


def load_nullspace_avoidance_parameters(node: Any, target: Any) -> None:
    """Load parameters into `target` fields (explicit casts preserved)."""

    p = lambda n: node.get_parameter(n).value
    p_float = lambda n: float(p(n))
    p_int = lambda n: int(p(n))
    p_bool = lambda n: bool(p(n))
    p_list_float = lambda n: [float(x) for x in list(p(n))]
    p_list_str = lambda n: [str(x) for x in list(p(n))]

    target.rate = p_float("control_rate")
    target.influence_distance = p_float("influence_distance")
    target.safety_margin = p_float("safety_margin")
    target.d_aggr = p_float("aggressive_distance")
    target.k_aggr = p_float("aggressive_gain_scale")
    target.k_null = p_float("nullspace_gain")
    target.k_tan = p_float("tangential_gain")
    target.max_qdot = p_float("max_joint_velocity")
    target.excluded = p_list_str("excluded_obstacles")

    target.capsule_radii = p_list_float("capsule_radii")
    target.capsule_fractions = p_list_float("capsule_fractions")
    target.box_projection_iters = p_int("box_projection_iters")

    target.repulsion_spread_enable = p_bool("repulsion_spread_enable")
    target.repulsion_spread_samples = p_int("repulsion_spread_samples")
    target.repulsion_spread_half_length = p_float("repulsion_spread_half_length")

    target.enable_ground = p_bool("enable_ground_avoidance")
    target.ground_z = p_float("ground_z")
    target.ground_infl = p_float("ground_influence_distance")
    target.ground_safe = p_float("ground_safety_margin")
    target.k_ground = p_float("ground_gain")

    target.enable_self = p_bool("enable_self_collision_avoidance")
    target.self_infl = p_float("self_influence_distance")
    target.self_safe = p_float("self_safety_margin")
    target.k_self = p_float("self_gain")
    target.self_skip_adjacent = p_int("self_skip_adjacent_links")

    # --- CBF-QP params (with backward compatible defaults) ---
    target.cbf_d_safe = float(node.get_parameter("d_safe").value)
    target.cbf_d_buffer_in = float(node.get_parameter("d_buffer").value)
    target.cbf_d_buffer_out = float(node.get_parameter("d_buffer_out").value)
    if target.cbf_d_buffer_out <= target.cbf_d_buffer_in + 1e-12:
        target.cbf_d_buffer_out = float(1.10 * target.cbf_d_buffer_in)

    target.cbf_alpha = float(node.get_parameter("alpha").value)
    target.cbf_K = int(node.get_parameter("max_constraints").value)
    target.cbf_lambda_reg = float(node.get_parameter("lambda_reg").value)
    target.cbf_rho_slack = float(node.get_parameter("rho_slack").value)
    target.cbf_beta_lpf = float(node.get_parameter("beta_lpf").value)
    target.cbf_output_accel_limit = float(node.get_parameter("output_accel_limit").value)
    target.cbf_approach_speed_limit = float(node.get_parameter("approach_speed_limit").value)
    target.cbf_use_qp = bool(node.get_parameter("use_qp").value)
    target.cbf_eps = float(node.get_parameter("eps").value)

    target.cbf_W_diag = np.array(list(node.get_parameter("qp_weight_diag").value), dtype=float).reshape(-1)
    if target.cbf_W_diag.shape[0] != 7:
        node.get_logger().warn("qp_weight_diag must have length 7; falling back to ones")
        target.cbf_W_diag = np.ones(7, dtype=float)
    target.cbf_W_diag = np.maximum(target.cbf_W_diag, 1e-9)

    # --- Risk-scaled staging / stop gate ---
    target.risk_d_far = float(node.get_parameter("risk_d_far").value)
    target.risk_d_mid = float(node.get_parameter("risk_d_mid").value)
    target.risk_d_near = float(node.get_parameter("risk_d_near").value)
    target.stop_d_in = float(node.get_parameter("stop_distance").value)
    target.stop_d_out = float(node.get_parameter("stop_release_distance").value)

    # Ensure monotonic thresholds
    target.risk_d_far = max(target.risk_d_far, target.risk_d_mid + 1e-6)
    target.risk_d_mid = max(target.risk_d_mid, target.risk_d_near + 1e-6)
    target.risk_d_near = max(target.risk_d_near, target.stop_d_in + 1e-6)
    target.stop_d_out = max(target.stop_d_out, target.stop_d_in + 1e-6)

    target.cbf_alpha_min = float(node.get_parameter("alpha_min").value)
    target.cbf_alpha_max = float(node.get_parameter("alpha_max").value)

    # Backward compatible: if user didn't configure min/max, treat legacy 'alpha' as both.
    if (target.cbf_alpha_min <= 0.0) and (target.cbf_alpha_max <= 0.0):
        target.cbf_alpha_min = float(target.cbf_alpha)
        target.cbf_alpha_max = float(target.cbf_alpha)

    target.cbf_alpha_min = max(0.0, float(target.cbf_alpha_min))
    target.cbf_alpha_max = max(0.0, float(target.cbf_alpha_max))
    if target.cbf_alpha_max < target.cbf_alpha_min:
        target.cbf_alpha_max = target.cbf_alpha_min

    target.cbf_qp_damping_min = float(node.get_parameter("qp_damping_min").value)
    target.cbf_qp_damping_max = float(node.get_parameter("qp_damping_max").value)

    target.cbf_beta_lpf_far = float(node.get_parameter("beta_lpf_far").value)
    target.cbf_beta_lpf_near = float(node.get_parameter("beta_lpf_near").value)
    target.min_distance_lpf = float(node.get_parameter("min_distance_lpf").value)

    # These two params were added later; when running with an older installed YAML
    # (or a different overlay) they may not be initialized. Default to "off".
    try:
        target.posture_bias_gain = float(node.get_parameter("posture_bias_gain").value)
    except Exception:
        target.posture_bias_gain = 0.0

    try:
        target.posture_reference_param = list(node.get_parameter("posture_reference").value)
    except Exception:
        target.posture_reference_param = []


def build_cbf_params(target: Any) -> CbfFilterParams:
    """Bundle safety-filter params in one place (keeps controller loop uncluttered)."""
    return CbfFilterParams(
        rate=float(target.rate),
        max_qdot=float(target.max_qdot),
        cbf_d_safe=float(target.cbf_d_safe),
        cbf_d_buffer_in=float(target.cbf_d_buffer_in),
        cbf_d_buffer_out=float(target.cbf_d_buffer_out),
        risk_d_far=float(target.risk_d_far),
        risk_d_mid=float(target.risk_d_mid),
        risk_d_near=float(target.risk_d_near),
        stop_d_in=float(target.stop_d_in),
        stop_d_out=float(target.stop_d_out),
        cbf_eps=float(target.cbf_eps),
        cbf_K=int(target.cbf_K),
        cbf_approach_speed_limit=float(target.cbf_approach_speed_limit),
        cbf_alpha_min=float(target.cbf_alpha_min),
        cbf_alpha_max=float(target.cbf_alpha_max),
        cbf_use_qp=bool(target.cbf_use_qp),
        cbf_qp_damping_min=float(target.cbf_qp_damping_min),
        cbf_qp_damping_max=float(target.cbf_qp_damping_max),
        beta_lpf_far=float(target.cbf_beta_lpf_far),
        beta_lpf_near=float(target.cbf_beta_lpf_near),
        min_distance_lpf=float(target.min_distance_lpf),
        output_accel_limit=float(target.cbf_output_accel_limit),
        posture_bias_gain=float(target.posture_bias_gain),
        posture_reference_param=list(target.posture_reference_param),
    )


def setup_optional_qp_solver(*, target: Any) -> QpSetup:
    """Initialize the optional OSQP wrapper exactly as the controller did."""
    qp = QpSetup(qp_solver=None, qp_available=False)

    if bool(getattr(target, "cbf_use_qp", False)) and int(getattr(target, "cbf_K", 0)) > 0:
        solver = OsqpCbfQpSolver(
            K=int(target.cbf_K),
            W_diag=np.array(target.cbf_W_diag, dtype=float).reshape(-1),
            lambda_reg=float(target.cbf_lambda_reg),
            rho_slack=float(target.cbf_rho_slack),
            max_abs_vel=float(target.max_qdot),
            max_iter=100,
        )
        qp.qp_solver = solver
        qp.qp_available = bool(getattr(solver, "available", False))

        try:
            target._cbf_state.qp_last_status = str(getattr(solver, "init_status", "disabled"))
        except Exception:
            pass

    return qp


def log_loaded_parameters(node: Any, target: Any, *, qp_available: bool) -> None:
    """Emit the same parameter log lines as before."""
    node.get_logger().info("📊 Parametri CARICATI (da file YAML o default):")
    node.get_logger().info(f"   influence_distance: {target.influence_distance}")
    node.get_logger().info(f"   safety_margin: {target.safety_margin}")
    node.get_logger().info(f"   k_null (nullspace_gain): {target.k_null}")
    node.get_logger().info(f"   k_tan (tangential_gain): {target.k_tan}")
    node.get_logger().info(f"   max_qdot (max_joint_velocity): {target.max_qdot}")
    node.get_logger().info(f"   d_aggr (aggressive_distance): {target.d_aggr}")
    node.get_logger().info(f"   k_aggr (aggressive_gain_scale): {target.k_aggr}")
    node.get_logger().info(
        "   capsule geometry: "
        f"radii={target.capsule_radii} | fractions={target.capsule_fractions}"
    )
    node.get_logger().info(
        "   box distance: "
        f"iters={target.box_projection_iters} | spread(enable={target.repulsion_spread_enable}, samples={target.repulsion_spread_samples}, half_len={target.repulsion_spread_half_length})"
    )
    node.get_logger().info(
        "   extra safety: "
        f"ground(enable={target.enable_ground}, z={target.ground_z}, d_infl={target.ground_infl}, d_safe={target.ground_safe}, k={target.k_ground}) | "
        f"self(enable={target.enable_self}, d_infl={target.self_infl}, d_safe={target.self_safe}, k={target.k_self}, skip_adj={target.self_skip_adjacent})"
    )
    node.get_logger().info(
        "   CBF-QP safety filter: "
        f"d_safe={target.cbf_d_safe}, d_buffer_in={target.cbf_d_buffer_in}, d_buffer_out={target.cbf_d_buffer_out}, "
        f"alpha={target.cbf_alpha} (risk-scaled [{target.cbf_alpha_min},{target.cbf_alpha_max}]), K={target.cbf_K}, use_qp={target.cbf_use_qp} (available={qp_available}), beta_lpf={target.cbf_beta_lpf}"
    )
    node.get_logger().info(
        "   Risk zones: "
        f"far={target.risk_d_far:.3f} mid={target.risk_d_mid:.3f} near={target.risk_d_near:.3f} "
        f"stop_in={target.stop_d_in:.3f} stop_out={target.stop_d_out:.3f} | "
        f"qp_damping=[{target.cbf_qp_damping_min},{target.cbf_qp_damping_max}] | "
        f"beta_lpf_far={target.cbf_beta_lpf_far} beta_lpf_near={target.cbf_beta_lpf_near}"
    )


# -----------------------------------------------------------------------------
# Publishing helpers
# -----------------------------------------------------------------------------


def publish_stale_outputs(
    *,
    min_dist_raw_pub: Any,
    min_dist_pub: Any,
    closest_constraint_pub: Any,
    closest_hazard_pub: Any,
) -> None:
    """Publish safe defaults when controller is not ready (best-effort)."""
    try:
        min_dist_raw_pub.publish(Float64MultiArray(data=[999.0]))
        min_dist_pub.publish(Float64MultiArray(data=[999.0]))
        closest_constraint_pub.publish(Float64MultiArray(data=[999.0] + [0.0] * 7))
        msg = String()
        msg.data = "none"
        closest_hazard_pub.publish(msg)
    except Exception:
        pass


def publish_cbf_diagnostics(
    *,
    jac_zero: Float64MultiArray,
    G: Optional[np.ndarray],
    m_active: int,
    active_best: Optional[dict],
    d_min_raw: float,
    cbf_state: CbfFilterState,
    min_dist_raw_pub: Any,
    min_dist_pub: Any,
    hazard_pub: Any,
    jac_pub: Any,
) -> None:
    """Publish min-distance, hazard string, and active constraint Jacobian row."""
    hazard_msg = String()

    # Publish BOTH raw and filtered global min distance.
    min_dist_raw_pub.publish(Float64MultiArray(data=[float(d_min_raw)]))
    min_dist_pub.publish(Float64MultiArray(data=[float(cbf_state.d_min_filt)]))

    if bool(cbf_state.stop_gate_active):
        hazard_msg.data = "stop_gate"
        hazard_pub.publish(hazard_msg)
        jac_pub.publish(jac_zero)
        return

    if (active_best is None) or (int(m_active) <= 0) or (G is None):
        hazard_msg.data = "none"
        hazard_pub.publish(hazard_msg)
        jac_pub.publish(jac_zero)
        return

    hazard_msg.data = str(active_best.get("hazard", "none"))
    hazard_pub.publish(hazard_msg)
    jac_pub.publish(Float64MultiArray(data=np.array(G, dtype=float)[0, :].reshape(-1).tolist()))


def publish_closest_constraint_pair(
    *,
    candidates: Sequence[dict],
    d_min: float,
    cbf_params: CbfFilterParams,
    model: Any,
    data: Any,
    q: np.ndarray,
    closest_constraint_pub: Any,
    closest_hazard_pub: Any,
) -> None:
    """Publish coherent pair (d_closest_raw, j_row_closest) for downstream blending."""
    try:
        if len(list(candidates)) <= 0:
            closest_constraint_pub.publish(Float64MultiArray(data=[999.0] + [0.0] * 7))
            msg = String(); msg.data = "none"
            closest_hazard_pub.publish(msg)
            return

        Gc, _, mc, best_c = build_cbf_constraints(
            list(candidates),
            float(1e9),
            K=1,
            cbf_eps=float(cbf_params.cbf_eps),
            cbf_d_safe=float(cbf_params.cbf_d_safe),
            approach_speed_limit=float(cbf_params.cbf_approach_speed_limit),
            alpha_min=float(cbf_params.cbf_alpha_min),
            alpha_max=float(cbf_params.cbf_alpha_max),
            risk_d_far=float(cbf_params.risk_d_far),
            risk_d_mid=float(cbf_params.risk_d_mid),
            risk_d_near=float(cbf_params.risk_d_near),
            stop_distance=float(cbf_params.stop_d_in),
            model=model,
            data=data,
            q=q,
        )

        if (int(mc) > 0) and (best_c is not None):
            d_closest = float(best_c.get("d", float(d_min)))
            j_row_closest = np.array(Gc[0, :], dtype=float).reshape(-1)
            closest_constraint_pub.publish(Float64MultiArray(data=[float(d_closest)] + j_row_closest.tolist()))
            msg = String(); msg.data = str(best_c.get("hazard", "none"))
            closest_hazard_pub.publish(msg)
            return

        closest_constraint_pub.publish(Float64MultiArray(data=[999.0] + [0.0] * 7))
        msg = String(); msg.data = "none"
        closest_hazard_pub.publish(msg)
    except Exception:
        pass
