"""Parameter declaration + loading for distance-only avoidance controller.

This module provides ONLY the minimal parameter set for the distance-only
avoidance controller (online_avoidance_controller.py).

The controller computes:
- Distances to external obstacles
- Closest constraint Jacobian
- Closest hazard label
- Visualization markers

NO velocity computation, NO CBF/QP, NO self-collision.
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

    # Distance computation tuning
    "box_projection_iters": 8,
    "repulsion_spread_enable": True,
    "repulsion_spread_samples": 5,
    "repulsion_spread_half_length": 0.10,

    # Visualization
    "distance_inflation": 0.0,  # [m] conservative offset for display
}


@dataclass(frozen=True)
class AvoidanceControllerParams:
    """Parameter set for distance-only avoidance controller.
    
    Computes distances to obstacles but does NOT generate velocities.
    Publishes: min distance, closest constraint Jacobian, hazard label, markers.
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

    # Distance model
    box_projection_iters: int
    repulsion_spread_enable: bool
    repulsion_spread_samples: int
    repulsion_spread_half_length: float

    # Visualization
    distance_inflation: float


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

    box_projection_iters = p_int("box_projection_iters")
    repulsion_spread_enable = p_bool("repulsion_spread_enable")
    repulsion_spread_samples = p_int("repulsion_spread_samples")
    repulsion_spread_half_length = p_float("repulsion_spread_half_length")

    distance_inflation = p_float("distance_inflation")

    return AvoidanceControllerParams(
        rate=float(rate),
        influence_distance=float(influence_distance),
        excluded=list(excluded),
        capsule_radii=list(capsule_radii),
        capsule_fractions=list(capsule_fractions),
        debug_capsule_index=int(debug_capsule_index),
        box_projection_iters=int(box_projection_iters),
        repulsion_spread_enable=bool(repulsion_spread_enable),
        repulsion_spread_samples=int(repulsion_spread_samples),
        repulsion_spread_half_length=float(repulsion_spread_half_length),
        distance_inflation=float(distance_inflation),
    )
