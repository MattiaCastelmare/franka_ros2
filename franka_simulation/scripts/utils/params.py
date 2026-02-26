"""Parameter declaration + loading for avoidance controller.

This module provides the parameter set for the avoidance controller
(online_avoidance_controller.py).

The controller computes:
- Distances to external obstacles
- Closest constraint Jacobian
- Closest hazard label
- Visualization markers
- CBF-QP velocity avoidance (when obstacles are within influence distance)
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, List


__all__ = [
    "AvoidanceControllerParams",
    "load_avoidance_controller_params",
]


# ============================================================================
# DISTANCE-ONLY AVOIDANCE CONTROLLER
# ============================================================================

DEFAULT_AVOIDANCE_CONTROLLER_PARAMS: Dict[str, Any] = {
    # Control rate
    "control_rate": 100.0,

    # External obstacle detection
    "influence_distance": 0.30,
    "excluded_obstacles": ["ground_plane", "ground", "floor", "plane"],

    # Capsule geometry (radii + fractions)
    "capsule_radii": [0.15, 0.12, 0.13],
    "capsule_fractions": [0.00, 0.35, 0.25, 0.75, 0.60, 0.95],
    "debug_capsule_index": -1,  # -1 = all capsules; 0-7 = single capsule debug

    # Capsule distance filter
    "min_capsule_index_for_distance": 3,  # only capsules >= this index contribute to distances/CBF

    # Distance computation tuning
    "box_projection_iters": 8,
    "repulsion_spread_enable": True,
    "repulsion_spread_samples": 5,
    "repulsion_spread_half_length": 0.10,

    # Visualization
    "distance_inflation": 0.0,  # [m] conservative offset for display

    # CBF velocity avoidance
    "d_safe": 0.05,            # [m] safety margin (h = d - d_safe)
    "k_alpha": 1.0,            # CBF class-K linear gain (alpha(h) = k_alpha * h)
    "k_rep": 0.5,              # repulsion gain for nominal velocity
    "max_joint_vel": 0.5,      # [rad/s] symmetric per-joint velocity bound

    # Capsule risk weights (end-effector priority)
    "capsule_weight_last2": 2.0,               # last 2 capsules (highest priority)
    "capsule_weight_last3": 1.5,               # 3rd from end
    "capsule_weight_default": 1.0,             # all other capsules
    "capsule_influence_scale_enable": True,     # scale influence_distance by weight
    "risk_selection_enable": True,              # select candidate by min(d/w)
}


@dataclass(frozen=True)
class AvoidanceControllerParams:
    """Parameter set for avoidance controller with CBF-QP velocity generation.
    
    Computes distances to obstacles and generates repulsive joint velocities
    via a Control Barrier Function (CBF) safety filter when obstacles are
    within influence distance.
    """
    # Control
    rate: float

    # External obstacles
    influence_distance: float
    excluded: List[str]

    # Capsule geometry
    capsule_radii: List[float]
    capsule_fractions: List[float]
    debug_capsule_index: int

    # Capsule distance filter
    min_capsule_index_for_distance: int

    # Distance model
    box_projection_iters: int
    repulsion_spread_enable: bool
    repulsion_spread_samples: int
    repulsion_spread_half_length: float

    # Visualization
    distance_inflation: float

    # CBF velocity avoidance
    d_safe: float
    k_alpha: float
    k_rep: float
    max_joint_vel: float

    # Capsule risk weights
    capsule_weight_last2: float
    capsule_weight_last3: float
    capsule_weight_default: float
    capsule_influence_scale_enable: bool
    risk_selection_enable: bool


def load_avoidance_controller_params(node: Any) -> AvoidanceControllerParams:
    """Declare + load parameters for distance-only avoidance controller.
    
    Usage:
        self.params = load_avoidance_controller_params(self)
    """
    node.declare_parameters(
        "",
        [(k, v) for k, v in DEFAULT_AVOIDANCE_CONTROLLER_PARAMS.items()]
    )

    p = lambda n: node.get_parameter(n).value
    p_float = lambda n: float(p(n))
    p_int = lambda n: int(p(n))
    p_bool = lambda n: bool(p(n))
    p_list_float = lambda n: [float(x) for x in list(p(n))]
    p_list_str = lambda n: [str(x) for x in list(p(n))]

    rate = p_float("control_rate")
    influence_distance = p_float("influence_distance")
    excluded = p_list_str("excluded_obstacles")

    capsule_radii = p_list_float("capsule_radii")
    capsule_fractions = p_list_float("capsule_fractions")
    debug_capsule_index = p_int("debug_capsule_index")

    min_capsule_index_for_distance = p_int("min_capsule_index_for_distance")

    box_projection_iters = p_int("box_projection_iters")
    repulsion_spread_enable = p_bool("repulsion_spread_enable")
    repulsion_spread_samples = p_int("repulsion_spread_samples")
    repulsion_spread_half_length = p_float("repulsion_spread_half_length")

    distance_inflation = p_float("distance_inflation")

    d_safe = p_float("d_safe")
    k_alpha = p_float("k_alpha")
    k_rep = p_float("k_rep")
    max_joint_vel = p_float("max_joint_vel")

    capsule_weight_last2 = p_float("capsule_weight_last2")
    capsule_weight_last3 = p_float("capsule_weight_last3")
    capsule_weight_default = p_float("capsule_weight_default")
    capsule_influence_scale_enable = p_bool("capsule_influence_scale_enable")
    risk_selection_enable = p_bool("risk_selection_enable")

    return AvoidanceControllerParams(
        rate=float(rate),
        influence_distance=float(influence_distance),
        excluded=list(excluded),
        capsule_radii=list(capsule_radii),
        capsule_fractions=list(capsule_fractions),
        debug_capsule_index=int(debug_capsule_index),
        min_capsule_index_for_distance=int(min_capsule_index_for_distance),
        box_projection_iters=int(box_projection_iters),
        repulsion_spread_enable=bool(repulsion_spread_enable),
        repulsion_spread_samples=int(repulsion_spread_samples),
        repulsion_spread_half_length=float(repulsion_spread_half_length),
        distance_inflation=float(distance_inflation),
        d_safe=float(d_safe),
        k_alpha=float(k_alpha),
        k_rep=float(k_rep),
        max_joint_vel=float(max_joint_vel),
        capsule_weight_last2=float(capsule_weight_last2),
        capsule_weight_last3=float(capsule_weight_last3),
        capsule_weight_default=float(capsule_weight_default),
        capsule_influence_scale_enable=bool(capsule_influence_scale_enable),
        risk_selection_enable=bool(risk_selection_enable),
    )
