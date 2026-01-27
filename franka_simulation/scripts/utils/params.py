"""Parameter declaration + loading for controllers (ROS2 Humble).

Goal
----
Keep node scripts short and readable by moving:
- default parameter tables
- declaration (with defaults)
- parsing / casting
- backward-compatible fallbacks and validations

IMPORTANT: behavior should remain identical to the original node code.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, List, Optional

import numpy as np

from .cbf_filter import CbfFilterParams


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
    # Per-obstacle clamp on J^T xdot contribution (0.0 = off)
    "avoidance_contrib_max_ratio": 0.0,
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

    # Conservative distance option (subtract from measured distances for safety decisions)
    "distance_inflation": 0.0,  # [m]

    # Reduce hazard switching (especially important for downstream blending)
    "hazard_hold_time_s": 0.20,      # [s]
    "hazard_switch_delta_m": 0.010,  # [m] keep previous if new is within this margin

    # Architecture switch
    # - True  -> controller applies CBF/QP safety filtering to /avoidance/velocity
    # - False -> controller publishes nominal avoidance, and ALL safety is enforced in the blender
    #           (recommended to avoid double-safety conservatism).
    "controller_safety_filter_enable": True,
}


@dataclass(frozen=True)
class NullSpaceAvoidanceParams:
    # Control
    rate: float

    # External/nominal avoidance
    d_infl: float
    d_safe: float
    d_aggr: float
    k_aggr: float
    k_null: float
    k_tan: float
    max_qdot: float
    avoidance_contrib_max_ratio: float
    excluded: List[str]

    # Geometry
    capsule_radii: List[float]
    capsule_fractions: List[float]

    # Distance model
    box_projection_iters: int
    repulsion_spread_enable: bool
    repulsion_spread_samples: int
    repulsion_spread_half_length: float

    # Extra safety
    enable_ground: bool
    ground_z: float
    ground_infl: float
    ground_safe: float
    k_ground: float

    enable_self: bool
    self_infl: float
    self_safe: float
    k_self: float
    self_skip_adjacent: int

    # CBF/QP knobs (kept because used by QP setup + logging)
    cbf_d_safe: float
    cbf_d_buffer_in: float
    cbf_d_buffer_out: float

    cbf_alpha: float
    cbf_K: int
    cbf_lambda_reg: float
    cbf_rho_slack: float
    cbf_beta_lpf: float
    cbf_output_accel_limit: float
    cbf_approach_speed_limit: float
    cbf_use_qp: bool
    cbf_eps: float
    cbf_W_diag: np.ndarray

    risk_d_far: float
    risk_d_mid: float
    risk_d_near: float
    stop_d_in: float
    stop_d_out: float

    cbf_alpha_min: float
    cbf_alpha_max: float

    cbf_qp_damping_min: float
    cbf_qp_damping_max: float

    cbf_beta_lpf_far: float
    cbf_beta_lpf_near: float
    min_distance_lpf: float

    posture_bias_gain: float
    posture_reference_param: List[float]

    # Conservative distance handling + switching stabilization
    distance_inflation: float
    hazard_hold_time_s: float
    hazard_switch_delta_m: float

    # Architecture switch
    controller_safety_filter_enable: bool

    # Ready-to-use bundle for the safety filter stage
    cbf_params: CbfFilterParams


def load_controller_params(node: Any) -> NullSpaceAvoidanceParams:
    """Declare + load all controller parameters from a ROS node.

    Node usage:
        self.params = load_controller_params(self)

    This function declares parameters with defaults (single source of truth)
    and then loads them with explicit casts and the same guards as the original.
    """

    node.declare_parameters("", [(k, v) for k, v in DEFAULT_NULLSPACE_AVOIDANCE_PARAMS.items()])

    p = lambda n: node.get_parameter(n).value
    p_float = lambda n: float(p(n))
    p_int = lambda n: int(p(n))
    p_bool = lambda n: bool(p(n))
    p_list_float = lambda n: [float(x) for x in list(p(n))]
    p_list_str = lambda n: [str(x) for x in list(p(n))]

    rate = p_float("control_rate")
    d_infl = p_float("influence_distance")
    d_safe = p_float("safety_margin")
    d_aggr = p_float("aggressive_distance")
    k_aggr = p_float("aggressive_gain_scale")
    k_null = p_float("nullspace_gain")
    k_tan = p_float("tangential_gain")
    max_qdot = p_float("max_joint_velocity")
    avoidance_contrib_max_ratio = p_float("avoidance_contrib_max_ratio")
    excluded = p_list_str("excluded_obstacles")

    capsule_radii = p_list_float("capsule_radii")
    capsule_fractions = p_list_float("capsule_fractions")

    box_projection_iters = p_int("box_projection_iters")
    repulsion_spread_enable = p_bool("repulsion_spread_enable")
    repulsion_spread_samples = p_int("repulsion_spread_samples")
    repulsion_spread_half_length = p_float("repulsion_spread_half_length")

    enable_ground = p_bool("enable_ground_avoidance")
    ground_z = p_float("ground_z")
    ground_infl = p_float("ground_influence_distance")
    ground_safe = p_float("ground_safety_margin")
    k_ground = p_float("ground_gain")

    enable_self = p_bool("enable_self_collision_avoidance")
    self_infl = p_float("self_influence_distance")
    self_safe = p_float("self_safety_margin")
    k_self = p_float("self_gain")
    self_skip_adjacent = p_int("self_skip_adjacent_links")

    # --- CBF-QP params (with backward compatible defaults) ---
    cbf_d_safe = float(node.get_parameter("d_safe").value)
    cbf_d_buffer_in = float(node.get_parameter("d_buffer").value)
    cbf_d_buffer_out = float(node.get_parameter("d_buffer_out").value)
    if cbf_d_buffer_out <= cbf_d_buffer_in + 1e-12:
        cbf_d_buffer_out = float(1.10 * cbf_d_buffer_in)

    cbf_alpha = float(node.get_parameter("alpha").value)
    cbf_K = int(node.get_parameter("max_constraints").value)
    cbf_lambda_reg = float(node.get_parameter("lambda_reg").value)
    cbf_rho_slack = float(node.get_parameter("rho_slack").value)
    cbf_beta_lpf = float(node.get_parameter("beta_lpf").value)
    cbf_output_accel_limit = float(node.get_parameter("output_accel_limit").value)
    cbf_approach_speed_limit = float(node.get_parameter("approach_speed_limit").value)
    cbf_use_qp = bool(node.get_parameter("use_qp").value)
    cbf_eps = float(node.get_parameter("eps").value)

    cbf_W_diag = np.array(list(node.get_parameter("qp_weight_diag").value), dtype=float).reshape(-1)
    if cbf_W_diag.shape[0] != 7:
        node.get_logger().warn("qp_weight_diag must have length 7; falling back to ones")
        cbf_W_diag = np.ones(7, dtype=float)
    cbf_W_diag = np.maximum(cbf_W_diag, 1e-9)

    # --- Risk-scaled staging / stop gate ---
    # Derive risk thresholds from influence_distance for a single-parameter workflow.
    # far = d_infl, mid = 2/3 * d_infl, near = 1/3 * d_infl
    risk_d_far = float(d_infl)
    risk_d_mid = float(d_infl) * (2.0 / 3.0)
    risk_d_near = float(d_infl) * (1.0 / 3.0)
    stop_d_in = float(node.get_parameter("stop_distance").value)
    stop_d_out = float(node.get_parameter("stop_release_distance").value)

    # Ensure monotonic thresholds
    risk_d_far = max(risk_d_far, risk_d_mid + 1e-6)
    risk_d_mid = max(risk_d_mid, risk_d_near + 1e-6)
    risk_d_near = max(risk_d_near, stop_d_in + 1e-6)
    stop_d_out = max(stop_d_out, stop_d_in + 1e-6)

    cbf_alpha_min = float(node.get_parameter("alpha_min").value)
    cbf_alpha_max = float(node.get_parameter("alpha_max").value)

    # Backward compatible: if user didn't configure min/max, treat legacy 'alpha' as both.
    if (cbf_alpha_min <= 0.0) and (cbf_alpha_max <= 0.0):
        cbf_alpha_min = float(cbf_alpha)
        cbf_alpha_max = float(cbf_alpha)

    cbf_alpha_min = max(0.0, float(cbf_alpha_min))
    cbf_alpha_max = max(0.0, float(cbf_alpha_max))
    if cbf_alpha_max < cbf_alpha_min:
        cbf_alpha_max = cbf_alpha_min

    cbf_qp_damping_min = float(node.get_parameter("qp_damping_min").value)
    cbf_qp_damping_max = float(node.get_parameter("qp_damping_max").value)

    cbf_beta_lpf_far = float(node.get_parameter("beta_lpf_far").value)
    cbf_beta_lpf_near = float(node.get_parameter("beta_lpf_near").value)
    min_distance_lpf = float(node.get_parameter("min_distance_lpf").value)

    # These two params were added later; when running with an older installed YAML
    # (or a different overlay) they may not be initialized. Default to "off".
    try:
        posture_bias_gain = float(node.get_parameter("posture_bias_gain").value)
    except Exception:
        posture_bias_gain = 0.0

    try:
        posture_reference_param = [float(x) for x in list(node.get_parameter("posture_reference").value)]
    except Exception:
        posture_reference_param = []

    distance_inflation = float(node.get_parameter("distance_inflation").value)
    hazard_hold_time_s = float(node.get_parameter("hazard_hold_time_s").value)
    hazard_switch_delta_m = float(node.get_parameter("hazard_switch_delta_m").value)

    controller_safety_filter_enable = bool(node.get_parameter("controller_safety_filter_enable").value)

    cbf_params = CbfFilterParams(
        rate=float(rate),
        max_qdot=float(max_qdot),
        cbf_d_safe=float(cbf_d_safe),
        cbf_d_buffer_in=float(cbf_d_buffer_in),
        cbf_d_buffer_out=float(cbf_d_buffer_out),
        risk_d_far=float(risk_d_far),
        risk_d_mid=float(risk_d_mid),
        risk_d_near=float(risk_d_near),
        stop_d_in=float(stop_d_in),
        stop_d_out=float(stop_d_out),
        cbf_eps=float(cbf_eps),
        cbf_K=int(cbf_K),
        cbf_approach_speed_limit=float(cbf_approach_speed_limit),
        cbf_alpha_min=float(cbf_alpha_min),
        cbf_alpha_max=float(cbf_alpha_max),
        cbf_use_qp=bool(cbf_use_qp),
        cbf_qp_damping_min=float(cbf_qp_damping_min),
        cbf_qp_damping_max=float(cbf_qp_damping_max),
        beta_lpf_far=float(cbf_beta_lpf_far),
        beta_lpf_near=float(cbf_beta_lpf_near),
        min_distance_lpf=float(min_distance_lpf),
        output_accel_limit=float(cbf_output_accel_limit),
        posture_bias_gain=float(posture_bias_gain),
        posture_reference_param=list(posture_reference_param),
    )

    return NullSpaceAvoidanceParams(
        rate=float(rate),
        d_infl=float(d_infl),
        d_safe=float(d_safe),
        d_aggr=float(d_aggr),
        k_aggr=float(k_aggr),
        k_null=float(k_null),
        k_tan=float(k_tan),
        max_qdot=float(max_qdot),
        avoidance_contrib_max_ratio=float(avoidance_contrib_max_ratio),
        excluded=list(excluded),
        capsule_radii=list(capsule_radii),
        capsule_fractions=list(capsule_fractions),
        box_projection_iters=int(box_projection_iters),
        repulsion_spread_enable=bool(repulsion_spread_enable),
        repulsion_spread_samples=int(repulsion_spread_samples),
        repulsion_spread_half_length=float(repulsion_spread_half_length),
        enable_ground=bool(enable_ground),
        ground_z=float(ground_z),
        ground_infl=float(ground_infl),
        ground_safe=float(ground_safe),
        k_ground=float(k_ground),
        enable_self=bool(enable_self),
        self_infl=float(self_infl),
        self_safe=float(self_safe),
        k_self=float(k_self),
        self_skip_adjacent=int(self_skip_adjacent),
        cbf_d_safe=float(cbf_d_safe),
        cbf_d_buffer_in=float(cbf_d_buffer_in),
        cbf_d_buffer_out=float(cbf_d_buffer_out),
        cbf_alpha=float(cbf_alpha),
        cbf_K=int(cbf_K),
        cbf_lambda_reg=float(cbf_lambda_reg),
        cbf_rho_slack=float(cbf_rho_slack),
        cbf_beta_lpf=float(cbf_beta_lpf),
        cbf_output_accel_limit=float(cbf_output_accel_limit),
        cbf_approach_speed_limit=float(cbf_approach_speed_limit),
        cbf_use_qp=bool(cbf_use_qp),
        cbf_eps=float(cbf_eps),
        cbf_W_diag=np.array(cbf_W_diag, dtype=float).reshape(-1),
        risk_d_far=float(risk_d_far),
        risk_d_mid=float(risk_d_mid),
        risk_d_near=float(risk_d_near),
        stop_d_in=float(stop_d_in),
        stop_d_out=float(stop_d_out),
        cbf_alpha_min=float(cbf_alpha_min),
        cbf_alpha_max=float(cbf_alpha_max),
        cbf_qp_damping_min=float(cbf_qp_damping_min),
        cbf_qp_damping_max=float(cbf_qp_damping_max),
        cbf_beta_lpf_far=float(cbf_beta_lpf_far),
        cbf_beta_lpf_near=float(cbf_beta_lpf_near),
        min_distance_lpf=float(min_distance_lpf),
        posture_bias_gain=float(posture_bias_gain),
        posture_reference_param=list(posture_reference_param),
        distance_inflation=float(distance_inflation),
        hazard_hold_time_s=float(hazard_hold_time_s),
        hazard_switch_delta_m=float(hazard_switch_delta_m),
        controller_safety_filter_enable=bool(controller_safety_filter_enable),
        cbf_params=cbf_params,
    )


def setup_optional_qp_solver(*, params: NullSpaceAvoidanceParams, cbf_state: Any) -> tuple[Any, bool]:
    """Initialize the optional OSQP wrapper (same behavior as the original node).

    Returns: (qp_solver, qp_available)
    """
    qp_solver = None
    qp_available = False

    if bool(params.cbf_use_qp) and int(params.cbf_K) > 0:
        # Lazy import to keep import-time deps minimal.
        from .avoidance_math import OsqpCbfQpSolver

        qp_solver = OsqpCbfQpSolver(
            K=int(params.cbf_K),
            W_diag=np.array(params.cbf_W_diag, dtype=float).reshape(-1),
            lambda_reg=float(params.cbf_lambda_reg),
            rho_slack=float(params.cbf_rho_slack),
            max_abs_vel=float(params.max_qdot),
            max_iter=100,
        )
        qp_available = bool(getattr(qp_solver, "available", False))
        try:
            cbf_state.qp_last_status = str(getattr(qp_solver, "init_status", "disabled"))
        except Exception:
            pass

    return qp_solver, bool(qp_available)
