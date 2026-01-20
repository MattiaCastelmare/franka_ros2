"""ROS-facing helpers for `velocity_control_blender.py`.

Why this exists
---------------
`velocity_blender_core.py` already contains the algorithmic (ROS-agnostic) pieces.
This module contains the *ROS-dependent glue* that was still making the node
large:
- parameter declaration/loading
- JointTrajectory parsing into canonical joint arrays
- safety-signal selection (closest constraint vs legacy topics)
- polyline progress bookkeeping (rejoin + lookahead target selection)

Keep this module lightweight: it can depend on rclpy and ROS messages, but it
should not contain heavy math.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy as np

# Core (ROS-agnostic) control primitives.
from .velocity_blender_core import (
    EmergencyParams,
    EmergencyRecoveryState,
    InfluenceParams,
    apply_output_filter_and_constraints,
    compute_influence_zone_command,
    emergency_override,
)

from .risk_staging import compute_risk_staging


# -----------------------------------------------------------------------------
# Parameters
# -----------------------------------------------------------------------------


DEFAULT_VELOCITY_BLENDER_PARAMS: Dict[str, Any] = {
    # Core tracking
    "control_period_s": 0.01,
    "kp": 20.0,
    "max_vel": 0.4,
    "waypoint_threshold": 0.05,
    "final_threshold": 0.01,

    # Path rejoin / lookahead
    "rejoin_enable": True,
    "rejoin_lookahead_points": 5,
    "rejoin_lookahead_distance_rad": 0.25,
    "rejoin_search_ahead_points": 0,

    # Distance / avoidance inputs
    "influence_distance": 0.30,
    "safety_margin": 0.08,

    # Conservative distance option (match controller semantics)
    "distance_inflation": 0.0,  # [m]

    # Unified risk staging (30/20/10/5 cm) for gating + risk-scaled filtering
    "risk_d_far": 0.30,
    "risk_d_mid": 0.20,
    "risk_d_near": 0.10,
    "stop_distance": 0.05,

    # Blending / constraints
    "avoidance_weight_max": 1.0,
    "slowdown_factor_max": 0.5,
    "d_dot_min_close": 0.0,
    "d_dot_push_gain": 0.5,
    "d_dot_push_max": 0.10,
    "d_dot_min_far": -0.05,
    "cbf_enable": True,
    "cbf_kappa": 2.0,
    "cbf_projection_iters": 4,
    "cbf_eps": 1e-4,

    # Emergency stop/escape
    "emergency_enable": True,
    "emergency_enter_m": 0.05,
    "emergency_exit_m": 0.10,
    "emergency_d_dot": 0.06,
    "emergency_max_vel_fraction": 1.0,
    "emergency_hazard_prefixes": ["external:"],

    # Recovery
    "recovery_enable": True,
    "recovery_time_s": 1.0,
    "recovery_tangent_speed": 0.08,

    # Blending shaping / anti-stall
    "avoidance_tangent_weight": 0.4,
    "normal_correction_max": 0.25,
    "use_avoidance_velocity": True,
    "avoidance_normal_only": True,
    "null_boost_max": 3.0,
    "avoidance_ratio_max": 1.2,
    "avoidance_input_filter_beta": 0.55,
    "avoidance_input_filter_beta_near": 0.90,
    "avoidance_repulsion_cap_fraction": 0.65,
    "velocity_filter_beta": 0.7,
    "velocity_filter_beta_near": 0.90,
    "slowdown_gamma_min": 0.6,
    "ns_floor_fraction": 0.25,
    "diag_enable": True,
    "diag_period_s": 1.0,
    "diag_cmd_norm_eps": 0.005,
    "tangent_escape_enable": True,
    "tangent_escape_speed": 0.03,
    "tangent_escape_err_min": 0.03,

    # Extra cap to prevent tangential dominance (applied in j-row nullspace)
    "tangential_cmd_max_fraction": 0.60,

    # Pause / reactive
    "pause_enable": True,
    "reactive_enable": True,
    "reactive_deadband": 1e-3,
    "hold_position_without_trajectory": True,
}


def make_emergency_params_from_attrs(node_like: Any) -> EmergencyParams:
    """Build EmergencyParams from an object exposing the node attributes.

    This is pure refactor glue: it preserves the exact casts used in the node.
    """

    return EmergencyParams(
        emergency_enable=bool(getattr(node_like, "emergency_enable")),
        emergency_enter_m=float(getattr(node_like, "emergency_enter_m")),
        emergency_exit_m=float(getattr(node_like, "emergency_exit_m")),
        emergency_d_dot=float(getattr(node_like, "emergency_d_dot")),
        emergency_max_vel_fraction=float(getattr(node_like, "emergency_max_vel_fraction")),
        emergency_hazard_prefixes=list(getattr(node_like, "emergency_hazard_prefixes")),
        recovery_enable=bool(getattr(node_like, "recovery_enable")),
        recovery_time_s=float(getattr(node_like, "recovery_time_s")),
    )


def make_influence_params_from_attrs(node_like: Any) -> InfluenceParams:
    """Build InfluenceParams from an object exposing the node attributes.

    Pure refactor only: preserves exact parameter wiring and casts.
    """

    return InfluenceParams(
        n_dof=int(getattr(node_like, "n_dof")),
        d_infl=float(getattr(node_like, "d_infl")),
        d_safe=float(getattr(node_like, "d_safe")),
        max_vel=float(getattr(node_like, "max_vel")),
        avoidance_weight_max=float(getattr(node_like, "avoidance_weight_max")),
        slowdown_factor_max=float(getattr(node_like, "slowdown_factor_max")),
        slowdown_gamma_min=float(getattr(node_like, "slowdown_gamma_min")),
        distance_inflation=float(getattr(node_like, "distance_inflation")),
        risk_d_far=float(getattr(node_like, "risk_d_far")),
        risk_d_mid=float(getattr(node_like, "risk_d_mid")),
        risk_d_near=float(getattr(node_like, "risk_d_near")),
        stop_distance=float(getattr(node_like, "stop_distance")),
        d_dot_min_far=float(getattr(node_like, "d_dot_min_far")),
        d_dot_min_close=float(getattr(node_like, "d_dot_min_close")),
        cbf_enable=bool(getattr(node_like, "cbf_enable")),
        cbf_kappa=float(getattr(node_like, "cbf_kappa")),
        cbf_projection_iters=int(getattr(node_like, "cbf_projection_iters")),
        cbf_eps=float(getattr(node_like, "cbf_eps")),
        d_dot_push_gain=float(getattr(node_like, "d_dot_push_gain")),
        d_dot_push_max=float(getattr(node_like, "d_dot_push_max")),
        use_avoidance_velocity=bool(getattr(node_like, "use_avoidance_velocity")),
        avoidance_normal_only=bool(getattr(node_like, "avoidance_normal_only")),
        avoidance_tangent_weight=float(getattr(node_like, "avoidance_tangent_weight")),
        null_boost_max=float(getattr(node_like, "null_boost_max")),
        avoidance_ratio_max=float(getattr(node_like, "avoidance_ratio_max")),
        avoidance_repulsion_cap_fraction=float(getattr(node_like, "avoidance_repulsion_cap_fraction")),
        ns_floor_fraction=float(getattr(node_like, "ns_floor_fraction")),
        normal_correction_max=float(getattr(node_like, "normal_correction_max")),
        recovery_enable=bool(getattr(node_like, "recovery_enable")),
        recovery_tangent_speed=float(getattr(node_like, "recovery_tangent_speed")),
        tangent_escape_enable=bool(getattr(node_like, "tangent_escape_enable")),
        tangent_escape_speed=float(getattr(node_like, "tangent_escape_speed")),
        tangent_escape_err_min=float(getattr(node_like, "tangent_escape_err_min")),
        diag_cmd_norm_eps=float(getattr(node_like, "diag_cmd_norm_eps")),
        tangential_cmd_max_fraction=float(getattr(node_like, "tangential_cmd_max_fraction")),
    )


def declare_velocity_blender_parameters(node: Any) -> None:
    """Declare all parameters on a ROS2 node."""
    for k, v in DEFAULT_VELOCITY_BLENDER_PARAMS.items():
        node.declare_parameter(k, v)


def load_velocity_blender_parameters(node: Any, target: Any) -> None:
    """Load parameters from node into `target` attributes (keeps explicit casts)."""

    def p(name: str) -> Any:
        return node.get_parameter(name).value

    target.control_period_s = float(p("control_period_s"))
    target.kp = float(p("kp"))
    target.max_vel = float(p("max_vel"))
    target.waypoint_threshold = float(p("waypoint_threshold"))
    target.final_threshold = float(p("final_threshold"))

    target.rejoin_enable = bool(p("rejoin_enable"))
    target.rejoin_lookahead_points = int(p("rejoin_lookahead_points"))
    target.rejoin_lookahead_distance_rad = float(p("rejoin_lookahead_distance_rad"))
    target.rejoin_search_ahead_points = int(p("rejoin_search_ahead_points"))

    target.d_infl = float(p("influence_distance"))
    target.d_safe = float(p("safety_margin"))

    target.distance_inflation = float(p("distance_inflation"))

    target.risk_d_far = float(p("risk_d_far"))
    target.risk_d_mid = float(p("risk_d_mid"))
    target.risk_d_near = float(p("risk_d_near"))
    target.stop_distance = float(p("stop_distance"))

    target.avoidance_weight_max = float(p("avoidance_weight_max"))
    target.slowdown_factor_max = float(p("slowdown_factor_max"))
    target.d_dot_min_close = float(p("d_dot_min_close"))
    target.d_dot_push_gain = float(p("d_dot_push_gain"))
    target.d_dot_push_max = float(p("d_dot_push_max"))
    target.d_dot_min_far = float(p("d_dot_min_far"))

    target.cbf_enable = bool(p("cbf_enable"))
    target.cbf_kappa = float(p("cbf_kappa"))
    target.cbf_projection_iters = int(p("cbf_projection_iters"))
    target.cbf_eps = float(p("cbf_eps"))

    target.emergency_enable = bool(p("emergency_enable"))
    target.emergency_enter_m = float(p("emergency_enter_m"))
    target.emergency_exit_m = float(p("emergency_exit_m"))
    target.emergency_d_dot = float(p("emergency_d_dot"))
    target.emergency_max_vel_fraction = float(p("emergency_max_vel_fraction"))
    target.emergency_hazard_prefixes = [str(x) for x in list(p("emergency_hazard_prefixes"))]

    target.recovery_enable = bool(p("recovery_enable"))
    target.recovery_time_s = float(p("recovery_time_s"))
    target.recovery_tangent_speed = float(p("recovery_tangent_speed"))

    target.avoidance_tangent_weight = float(p("avoidance_tangent_weight"))
    target.normal_correction_max = float(p("normal_correction_max"))
    target.use_avoidance_velocity = bool(p("use_avoidance_velocity"))
    target.avoidance_normal_only = bool(p("avoidance_normal_only"))
    target.null_boost_max = float(p("null_boost_max"))
    target.avoidance_ratio_max = float(p("avoidance_ratio_max"))

    target.avoidance_input_filter_beta = float(p("avoidance_input_filter_beta"))
    target.avoidance_input_filter_beta_near = float(p("avoidance_input_filter_beta_near"))
    target.avoidance_repulsion_cap_fraction = float(p("avoidance_repulsion_cap_fraction"))

    target.velocity_filter_beta = float(p("velocity_filter_beta"))
    target.velocity_filter_beta_near = float(p("velocity_filter_beta_near"))
    target.slowdown_gamma_min = float(p("slowdown_gamma_min"))
    target.ns_floor_fraction = float(p("ns_floor_fraction"))

    target.diag_enable = bool(p("diag_enable"))
    target.diag_period_s = float(p("diag_period_s"))
    target.diag_cmd_norm_eps = float(p("diag_cmd_norm_eps"))

    target.tangent_escape_enable = bool(p("tangent_escape_enable"))
    target.tangent_escape_speed = float(p("tangent_escape_speed"))
    target.tangent_escape_err_min = float(p("tangent_escape_err_min"))

    target.tangential_cmd_max_fraction = float(p("tangential_cmd_max_fraction"))

    target.pause_enable = bool(p("pause_enable"))
    target.paused = False

    target.reactive_enable = bool(p("reactive_enable"))
    target.reactive_deadband = float(p("reactive_deadband"))
    target.hold_position_without_trajectory = bool(p("hold_position_without_trajectory"))


# -----------------------------------------------------------------------------
# JointTrajectory parsing
# -----------------------------------------------------------------------------


def joint_trajectory_to_points(
    *,
    msg: Any,
    joint_names: Sequence[str],
    n_dof: int,
    logger: Optional[Any] = None,
) -> Optional[List[np.ndarray]]:
    """Convert a JointTrajectory message into a list of joint-position waypoints.

    - Preserves the node's fallback behavior when `msg.joint_names` is empty.
    - Returns None on mismatch/unusable messages.
    """
    if msg is None or (not getattr(msg, "points", None)):
        return None

    joint_names = list(joint_names)
    n_dof = int(n_dof)

    index_map: Dict[int, int] = {}
    if getattr(msg, "joint_names", None) and len(msg.joint_names) > 0:
        for i, name in enumerate(joint_names):
            if name in msg.joint_names:
                index_map[i] = msg.joint_names.index(name)

    use_direct_positions = False
    if len(index_map) != n_dof:
        ok_shape = True
        for p in msg.points:
            if (not hasattr(p, "positions")) or (len(p.positions) != n_dof):
                ok_shape = False
                break

        if ok_shape:
            use_direct_positions = True
            if logger is not None:
                try:
                    logger.warn("JointTrajectory joint_names missing/mismatched; assuming canonical FR3 joint order.")
                except Exception:
                    pass
        else:
            if logger is not None:
                try:
                    logger.error(f"Joint names mismatch in trajectory! joint_names={list(getattr(msg, 'joint_names', []))}")
                except Exception:
                    pass
            return None

    new_points: List[np.ndarray] = []
    for point in msg.points:
        if use_direct_positions:
            q_target = np.array(point.positions[:n_dof], dtype=float)
        else:
            q_target = np.array([point.positions[index_map[i]] for i in range(n_dof)], dtype=float)
        new_points.append(q_target)

    return new_points


def is_degenerate_trajectory(points: Sequence[np.ndarray], *, span_eps: float = 1e-3) -> bool:
    """Return True if the trajectory span is ~0 (all points nearly identical)."""
    try:
        pts = list(points)
        if len(pts) < 2:
            return False
        span = float(np.linalg.norm(pts[-1] - pts[0]))
        return span < float(span_eps)
    except Exception:
        return False


# -----------------------------------------------------------------------------
# JointState parsing (kept here to keep the node minimal)
# -----------------------------------------------------------------------------


def build_name_to_index(joint_names: Sequence[str]) -> Dict[str, int]:
    """Build a joint-name -> joint-index map once."""
    return {str(name): int(i) for i, name in enumerate(list(joint_names))}


def update_joint_positions_inplace(q: np.ndarray, msg: Any, name_to_i: Dict[str, int]) -> None:
    """Update q in-place from a JointState-like message (O(n))."""
    try:
        for name, pos in zip(msg.name, msg.position):
            i = name_to_i.get(str(name))
            if i is not None:
                q[int(i)] = pos
    except Exception:
        return


# -----------------------------------------------------------------------------
# Safety signal selection
# -----------------------------------------------------------------------------


def pick_safety_signal(
    *,
    closest_d: float,
    closest_j_row: np.ndarray,
    closest_hazard: str,
) -> Tuple[float, np.ndarray, str, float]:
    """Use the coherent closest constraint as the ONLY safety source.

    Returns: (d_raw, j_row, hazard, j_norm)

    Notes
    -----
    - We intentionally do *not* combine `/avoidance/min_distance` with `/avoidance/jacobian`.
      That pair can become incoherent due to filtering/switching.
    - If the closest constraint is not valid, we return a safe 'no hazard' signal.
    """
    try:
        d_raw = float(closest_d)
        j_row = np.array(closest_j_row, dtype=float).reshape(-1)
        hazard_for_safety = str(closest_hazard or "none")
        j_norm = float(np.linalg.norm(j_row))

        if (j_norm <= 1e-6) or (d_raw >= 1e6):
            return 999.0, np.zeros_like(j_row), "none", 0.0

        return float(d_raw), j_row, hazard_for_safety, float(j_norm)
    except Exception:
        return 999.0, np.zeros_like(np.array(closest_j_row, dtype=float).reshape(-1)), "none", 0.0


@dataclass
class SafetySignalContext:
    d_raw: float
    j_row: np.ndarray
    hazard: str
    j_norm: float

    inflation: float
    d_eff: float
    staging: Any


def compute_safety_signal_context(
    *,
    closest_d: float,
    closest_j_row: np.ndarray,
    closest_hazard: str,
    distance_inflation: float,
    risk_d_far: float,
    risk_d_mid: float,
    risk_d_near: float,
    stop_distance: float,
) -> SafetySignalContext:
    """Compute all safety-related signals used by the node in one place."""

    d_raw, j_row, hazard_for_safety, j_norm = pick_safety_signal(
        closest_d=float(closest_d),
        closest_j_row=np.array(closest_j_row, dtype=float).reshape(-1),
        closest_hazard=str(closest_hazard or "none"),
    )

    inflation, d_eff, staging = compute_effective_distance_and_staging(
        d_raw=float(d_raw),
        distance_inflation=float(distance_inflation),
        risk_d_far=float(risk_d_far),
        risk_d_mid=float(risk_d_mid),
        risk_d_near=float(risk_d_near),
        stop_distance=float(stop_distance),
    )

    return SafetySignalContext(
        d_raw=float(d_raw),
        j_row=np.array(j_row, dtype=float).reshape(-1),
        hazard=str(hazard_for_safety),
        j_norm=float(j_norm),
        inflation=float(inflation),
        d_eff=float(d_eff),
        staging=staging,
    )


def compute_effective_distance_and_staging(
    *,
    d_raw: float,
    distance_inflation: float,
    risk_d_far: float,
    risk_d_mid: float,
    risk_d_near: float,
    stop_distance: float,
) -> Tuple[float, float, Any]:
    """Compute inflation, effective distance and risk staging (pure refactor)."""

    inflation = float(max(0.0, float(distance_inflation)))
    d_eff = float(d_raw) - float(inflation)
    staging = compute_risk_staging(
        d_eff=float(d_eff),
        d_far=float(risk_d_far),
        d_mid=float(risk_d_mid),
        d_near=float(risk_d_near),
        d_stop=float(stop_distance),
    )
    return float(inflation), float(d_eff), staging


# -----------------------------------------------------------------------------
# Progress / rejoin bookkeeping
# -----------------------------------------------------------------------------


@dataclass
class PolylineProgress:
    """Keep progress along a polyline monotonic (index + arc-length)."""

    progress_index: int = 0
    progress_s: float = 0.0


def update_polyline_progress(
    *,
    q: np.ndarray,
    pts: Sequence[np.ndarray],
    s_cum: Optional[np.ndarray],
    last_idx: int,
    progress: PolylineProgress,
    rejoin_enable: bool,
    rejoin_search_ahead_points: int,
    nearest_point_fn: Any,
) -> PolylineProgress:
    """Project q onto the polyline and update progress monotonically."""

    n_pts = int(len(list(pts)))
    if n_pts <= 0:
        return progress

    start = int(max(0, progress.progress_index))
    if int(rejoin_search_ahead_points) > 0:
        end = int(min(int(last_idx), start + int(rejoin_search_ahead_points)))
    else:
        end = int(last_idx)

    if bool(rejoin_enable):
        try:
            seg_end = int(max(0, min(end - 1, int(last_idx) - 1)))
            seg_start = int(max(0, min(start, seg_end)))
            bi, _ba, bs, _bq, _ = nearest_point_fn(
                q=np.array(q, dtype=float).reshape(-1),
                pts=pts,
                s_cum=s_cum,
                i0=seg_start,
                i1=seg_end,
            )
            if float(bs) > float(progress.progress_s):
                progress.progress_s = float(bs)
                progress.progress_index = int(max(progress.progress_index, int(bi)))
        except Exception:
            pass

    return progress


def select_lookahead_target(
    *,
    q: np.ndarray,
    pts: Sequence[np.ndarray],
    s_cum: Optional[np.ndarray],
    last_idx: int,
    progress: PolylineProgress,
    rejoin_lookahead_distance_rad: float,
    rejoin_lookahead_points: int,
    interpolate_fn: Any,
) -> Tuple[np.ndarray, int]:
    """Return (q_target, current_index) using distance-based or index-based lookahead."""

    pts_l = list(pts)
    if len(pts_l) <= 0:
        return np.zeros_like(np.array(q, dtype=float).reshape(-1)), 0

    lookahead_s = float(rejoin_lookahead_distance_rad)
    if (lookahead_s > 1e-6) and (s_cum is not None):
        s_target = float(progress.progress_s) + float(lookahead_s)
        q_target = interpolate_fn(pts=pts_l, s_cum=s_cum, s_query=s_target, n_dof=int(np.array(q).reshape(-1).shape[0]))
        try:
            current_index = int(np.searchsorted(s_cum, float(progress.progress_s), side="right") - 1)
            current_index = int(max(0, min(int(last_idx), current_index)))
        except Exception:
            current_index = int(max(0, min(int(last_idx), int(progress.progress_index))))
        return np.array(q_target, dtype=float).reshape(-1), int(current_index)

    lookahead = max(0, int(rejoin_lookahead_points))
    current_index = int(max(0, min(int(last_idx), int(progress.progress_index))))
    target_index = int(min(int(last_idx), current_index + lookahead))
    return np.array(pts_l[target_index], dtype=float).reshape(-1), int(current_index)


def update_progress_and_select_target(
    *,
    q: np.ndarray,
    pts: Sequence[np.ndarray],
    s_cum: Optional[np.ndarray],
    last_idx: int,
    current_index: int,
    progress: PolylineProgress,
    rejoin_enable: bool,
    rejoin_search_ahead_points: int,
    rejoin_lookahead_distance_rad: float,
    rejoin_lookahead_points: int,
    nearest_point_fn: Any,
    interpolate_fn: Any,
) -> Tuple[PolylineProgress, int, np.ndarray]:
    """Pipeline wrapper: update progress (with rejoin semantics) and pick target."""

    if not bool(rejoin_enable):
        progress.progress_index = int(current_index)

    progress = update_polyline_progress(
        q=np.array(q, dtype=float).reshape(-1),
        pts=pts,
        s_cum=s_cum,
        last_idx=int(last_idx),
        progress=progress,
        rejoin_enable=bool(rejoin_enable),
        rejoin_search_ahead_points=int(rejoin_search_ahead_points),
        nearest_point_fn=nearest_point_fn,
    )

    q_target, current_index_new = select_lookahead_target(
        q=np.array(q, dtype=float).reshape(-1),
        pts=pts,
        s_cum=s_cum,
        last_idx=int(last_idx),
        progress=progress,
        rejoin_lookahead_distance_rad=float(rejoin_lookahead_distance_rad),
        rejoin_lookahead_points=int(rejoin_lookahead_points),
        interpolate_fn=interpolate_fn,
    )

    return progress, int(current_index_new), np.array(q_target, dtype=float).reshape(-1)


def check_trajectory_completion(
    *,
    q: np.ndarray,
    q_final: np.ndarray,
    final_threshold: float,
) -> Tuple[float, bool]:
    """Return (final_err, complete) using the node's exact norm/threshold rule."""

    final_err = float(np.linalg.norm(np.array(q_final, dtype=float).reshape(-1) - np.array(q, dtype=float).reshape(-1)))
    return float(final_err), bool(final_err < float(final_threshold))


# -----------------------------------------------------------------------------
# Control-flow patterns (pause/reactive/emergency)
# -----------------------------------------------------------------------------


def handle_pause_mode(*, pause_enable: bool, paused: bool, n_dof: int) -> Tuple[bool, Optional[np.ndarray], bool]:
    """Return (handled, qdot_cmd, reset_smoothing)."""
    if bool(pause_enable) and bool(paused):
        return True, np.zeros(int(n_dof), dtype=float), True
    return False, None, False


def handle_no_trajectory_mode(
    *,
    active: bool,
    trajectory_points_len: int,
    hold_position_without_trajectory: bool,
    reactive_enable: bool,
    qdot_avoid: np.ndarray,
    reactive_deadband: float,
    max_vel: float,
    n_dof: int,
) -> Tuple[bool, Optional[np.ndarray], bool]:
    """Return (handled, qdot_cmd, reset_smoothing) for the no-trajectory branch."""

    if bool(active) and int(trajectory_points_len) > 0:
        return False, None, False

    if bool(hold_position_without_trajectory):
        return True, np.zeros(int(n_dof), dtype=float), False

    if bool(reactive_enable):
        qdot = np.array(qdot_avoid, dtype=float).reshape(int(n_dof))
        if float(np.linalg.norm(qdot)) < float(reactive_deadband):
            qdot = np.zeros(int(n_dof), dtype=float)
        qdot = np.clip(qdot, -float(max_vel), float(max_vel))
        return True, qdot, False

    return True, np.zeros(int(n_dof), dtype=float), False


def handle_emergency_override(
    *,
    now_wall: float,
    d_eff: float,
    j_row: np.ndarray,
    hazard: str,
    j_norm: float,
    max_vel: float,
    state: EmergencyRecoveryState,
    params: EmergencyParams,
) -> Tuple[bool, Optional[np.ndarray], EmergencyRecoveryState, bool, bool]:
    """Wrapper around emergency_override() to keep the node flat.

    Returns: (handled, qdot_cmd, state_new, reset_smoothing, emergency_active_now)
    """

    handled, qdot_em, state_new, reset_smoothing = emergency_override(
        now_wall=float(now_wall),
        d=float(d_eff),
        j_row=np.array(j_row, dtype=float).reshape(-1),
        hazard=str(hazard),
        j_norm=float(j_norm),
        max_vel=float(max_vel),
        state=state,
        params=params,
    )
    try:
        emergency_now = bool(state_new.emergency_active)
    except Exception:
        emergency_now = False
    return bool(handled), qdot_em, state_new, bool(reset_smoothing), bool(emergency_now)


def update_lpf(prev: np.ndarray, x: np.ndarray, beta: float) -> np.ndarray:
    """One-step LPF update with the same semantics used in the node."""
    b = float(beta)
    if b >= 0.999:
        return np.array(x, dtype=float).copy()
    return b * np.array(x, dtype=float) + (1.0 - b) * np.array(prev, dtype=float)


def compute_tracking_command(*, kp: float, q_target: np.ndarray, q: np.ndarray) -> np.ndarray:
    return float(kp) * (np.array(q_target, dtype=float).reshape(-1) - np.array(q, dtype=float).reshape(-1))


def compute_avoidance_input_beta(*, beta_far: float, beta_near: float, w: float) -> float:
    try:
        b_far = float(np.clip(float(beta_far), 0.0, 1.0))
        b_near = float(np.clip(float(beta_near), 0.0, 1.0))
        return float(b_far + (b_near - b_far) * float(w))
    except Exception:
        return 1.0


def blend_tracking_and_avoidance(
    *,
    now_wall: float,
    d_raw: float,
    d_eff: float,
    d_infl: float,
    j_row: np.ndarray,
    j_norm: float,
    staging: Any,
    qdot_tracking: np.ndarray,
    qdot_avoid_filt: np.ndarray,
    error_norm: float,
    threshold: float,
    recovery_until_wall: float,
    influence_params: InfluenceParams,
    stall_detected: bool = False,
    idx: int,
    n_points: int,
) -> Tuple[np.ndarray, Dict[str, Any]]:
    """Return (qdot_des, dbg) preserving the node's exact dbg fields."""

    avoid_norm = float(np.linalg.norm(np.array(qdot_avoid_filt, dtype=float).reshape(-1)))

    dbg: Dict[str, Any] = {
        "active": True,
        "idx": int(idx),
        "n_points": int(n_points),
        "err_norm": float(error_norm),
        "d_raw": float(d_raw),
        "d": float(d_eff),
        "j_norm": float(j_norm),
        "w_d": float(getattr(staging, "w_total", 0.0)),
        "gamma": 1.0,
        "d_dot": 0.0,
        "d_dot_min": 0.0,
        "track_norm": float(np.linalg.norm(np.array(qdot_tracking, dtype=float).reshape(-1))),
        "avoid_norm": float(avoid_norm),
        "stall_detected": bool(stall_detected),
    }

    if (float(d_eff) >= float(d_infl)) or (float(j_norm) < 1e-6):
        return np.array(qdot_tracking, dtype=float).reshape(-1), dbg

    qdot_des, dbg2 = compute_influence_zone_command(
        now_wall=float(now_wall),
        d=float(d_eff),
        j_row=np.array(j_row, dtype=float).reshape(-1),
        j_norm=float(j_norm),
        qdot_tracking=np.array(qdot_tracking, dtype=float).reshape(-1),
        qdot_avoid=np.array(qdot_avoid_filt, dtype=float).reshape(-1),
        avoid_norm=float(avoid_norm),
        error_norm=float(error_norm),
        threshold=float(threshold),
        recovery_until_wall=float(recovery_until_wall),
        stall_detected=bool(stall_detected),
        params=influence_params,
    )
    dbg.update(dbg2)
    return np.array(qdot_des, dtype=float).reshape(-1), dbg


def apply_final_filters_and_limits(
    *,
    qdot_des: np.ndarray,
    qdot_prev: np.ndarray,
    velocity_filter_beta: float,
    velocity_filter_beta_near: float,
    max_vel: float,
    d_eff: float,
    d_infl: float,
    j_row: np.ndarray,
    j_norm: float,
    cbf_projection_iters: int,
    cbf_eps: float,
    normal_correction_max: float,
    qdot_tracking_hint: np.ndarray,
    dbg: Dict[str, Any],
) -> Tuple[np.ndarray, np.ndarray, Dict[str, Any]]:
    return apply_output_filter_and_constraints(
        qdot_des=np.array(qdot_des, dtype=float).reshape(-1),
        qdot_prev=np.array(qdot_prev, dtype=float).reshape(-1),
        velocity_filter_beta=float(velocity_filter_beta),
        velocity_filter_beta_near=float(velocity_filter_beta_near),
        max_vel=float(max_vel),
        d=float(d_eff),
        d_infl=float(d_infl),
        j_row=np.array(j_row, dtype=float).reshape(-1),
        j_norm=float(j_norm),
        cbf_projection_iters=int(cbf_projection_iters),
        cbf_eps=float(cbf_eps),
        normal_correction_max=float(normal_correction_max),
        qdot_tracking_hint=np.array(qdot_tracking_hint, dtype=float).reshape(-1),
        dbg=dbg,
    )


# -----------------------------------------------------------------------------
# Diagnostics bookkeeping and throttled logging
# -----------------------------------------------------------------------------


def _throttle(now_wall: float, last_wall: float, period_s: float) -> Tuple[bool, float]:
    if (float(now_wall) - float(last_wall)) >= float(period_s):
        return True, float(now_wall)
    return False, float(last_wall)


@dataclass
class VelocityBlenderDiagnostics:
    """All diagnostic counters + throttling timestamps for the velocity blender."""

    last_diag_wall: float = 0.0
    last_robust_wall: float = 0.0

    infeasible_count: int = 0
    emergency_enter_count: int = 0
    stop_gate_count: int = 0

    prev_emergency: bool = False
    prev_stop_gate: bool = False

    def update_edge_counters(self, *, emergency_now: bool, stop_gate_now: bool) -> None:
        if (not bool(self.prev_emergency)) and bool(emergency_now):
            self.emergency_enter_count = int(self.emergency_enter_count) + 1
        self.prev_emergency = bool(emergency_now)

        if (not bool(self.prev_stop_gate)) and bool(stop_gate_now):
            self.stop_gate_count = int(self.stop_gate_count) + 1
        self.prev_stop_gate = bool(stop_gate_now)

    def count_infeasible(self, dbg: Dict[str, Any]) -> None:
        try:
            if bool(dbg.get("cbf_infeasible", False)):
                self.infeasible_count = int(self.infeasible_count) + 1
        except Exception:
            return

    def maybe_log_stall(
        self,
        logger: Any,
        *,
        diag_enable: bool,
        now_wall: float,
        diag_period_s: float,
        dbg: Dict[str, Any],
        cmd_norm: float,
        d_infl: float,
        diag_cmd_norm_eps: float,
    ) -> None:
        if not bool(diag_enable):
            return

        try:
            avoid_active = bool((float(dbg["d"]) < float(d_infl)) and (float(dbg["j_norm"]) > 1e-6))
            if avoid_active and (float(cmd_norm) <= float(diag_cmd_norm_eps)):
                should, last_new = _throttle(
                    now_wall=float(now_wall),
                    last_wall=float(self.last_diag_wall),
                    period_s=max(0.2, float(diag_period_s)),
                )
                if should:
                    self.last_diag_wall = float(last_new)
                    logger.warn(
                        "[BLENDER-STALL] "
                        f"idx={dbg['idx']}/{max(0, dbg['n_points']-1)} err_norm={dbg['err_norm']:.3f} "
                        f"d_eff={dbg.get('d_eff', dbg['d']):.3f} j_norm={dbg['j_norm']:.3e} w_d={dbg['w_d']:.3f} gamma={dbg['gamma']:.3f} "
                        f"d_dot={dbg['d_dot']:.4f} d_dot_min={dbg['d_dot_min']:.4f} "
                        f"|track|={dbg['track_norm']:.3f} |avoid_in|={dbg['avoid_norm']:.3f} |cmd|={float(cmd_norm):.3f} "
                        f"cbf_ok={bool(dbg.get('cbf_ok', True))} infeasible={bool(dbg.get('cbf_infeasible', False))} "
                        f"inf_cnt={int(self.infeasible_count)} emg_cnt={int(self.emergency_enter_count)} stop_cnt={int(self.stop_gate_count)}"
                    )
        except Exception:
            return

    def maybe_log_robust(
        self,
        logger: Any,
        *,
        diag_enable: bool,
        now_wall: float,
        d_raw: float,
        d_eff: float,
        inflation: float,
        hazard: str,
    ) -> None:
        if not bool(diag_enable):
            return

        try:
            should, last_new = _throttle(
                now_wall=float(now_wall),
                last_wall=float(self.last_robust_wall),
                period_s=1.0,
            )
            if should:
                self.last_robust_wall = float(last_new)
                logger.info(
                    "[BLENDER-ROBUST] "
                    f"d_raw={float(d_raw):.3f} d_eff={float(d_eff):.3f} infl={float(inflation):.3f} "
                    f"hazard='{str(hazard)}' "
                    f"infeasible={int(self.infeasible_count)} emergency_enter={int(self.emergency_enter_count)} stop_enter={int(self.stop_gate_count)}"
                )
        except Exception:
            return
