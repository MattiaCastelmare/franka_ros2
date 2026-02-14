"""ROS publishing utilities for controllers.

Goal
----
Keep node code short by centralizing repetitive publishing patterns.

IMPORTANT: topic names and message types are owned by the node; this module only
publishes using publishers that the node provides.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from std_msgs.msg import Float64MultiArray, String


@dataclass(frozen=True)
class PublishersBundle:
    """All publishers used by the controller node."""

    pub: Any
    min_dist_pub: Any
    min_dist_raw_pub: Any
    closest_constraint_pub: Any
    closest_hazard_pub: Any
    constraints_pub: Any
    jac_pub: Any
    hazard_pub: Any
    capsule_marker_pub: Any


def publish_not_ready_outputs(*, pubs: PublishersBundle) -> None:
    """Publish zeros + reset diagnostics when the controller is not ready.

    This covers the 'pinocchio not initialized / q not received yet' case.
    """
    # Joint velocity command and jacobian (zero)
    try:
        pubs.pub.publish(Float64MultiArray(data=[0.0] * 7))
        pubs.jac_pub.publish(Float64MultiArray(data=[0.0] * 7))
    except Exception:
        pass

    # Distance diagnostics
    try:
        pubs.min_dist_raw_pub.publish(Float64MultiArray(data=[999.0]))
        pubs.min_dist_pub.publish(Float64MultiArray(data=[999.0]))
    except Exception:
        pass

    # Coherent closest constraint pair
    try:
        pubs.closest_constraint_pub.publish(Float64MultiArray(data=[999.0] + [0.0] * 7))
        msg = String(); msg.data = "none"
        pubs.closest_hazard_pub.publish(msg)
    except Exception:
        pass

    # Multi-constraint list (avoidance-only blender; [N, d1, j1..] )
    try:
        pubs.constraints_pub.publish(Float64MultiArray(data=[0.0]))
    except Exception:
        pass

    # Hazard string (best-effort reset)
    try:
        msg = String(); msg.data = "none"
        pubs.hazard_pub.publish(msg)
    except Exception:
        pass


def publish_minimal_avoidance_outputs(
    *,
    pubs: PublishersBundle,
    qdot: Any,
    d_min_raw: float,
    closest_j_row: Any,
    closest_label: str,
) -> None:
    """Publish standard minimal avoidance outputs.
    
    This consolidates all repetitive publishing in the minimal (avoidance-only) controller.
    
    Args:
        pubs: PublishersBundle with all publishers.
        qdot: Joint velocity command (n_dof,).
        d_min_raw: Raw minimum distance [m].
        closest_j_row: Distance Jacobian row for closest constraint (n_dof,).
        closest_label: Human-readable hazard label.
    """
    import numpy as np
    
    try:
        qdot_arr = np.array(qdot, dtype=float).reshape(-1)
        pubs.pub.publish(Float64MultiArray(data=qdot_arr.tolist()))
    except Exception:
        pass
    
    try:
        pubs.min_dist_raw_pub.publish(Float64MultiArray(data=[float(d_min_raw)]))
        pubs.min_dist_pub.publish(Float64MultiArray(data=[float(d_min_raw)]))
    except Exception:
        pass
    
    try:
        j_row_arr = np.array(closest_j_row, dtype=float).reshape(-1)
        closest_payload = [float(d_min_raw)] + j_row_arr.tolist()
        pubs.closest_constraint_pub.publish(Float64MultiArray(data=closest_payload))
        pubs.jac_pub.publish(Float64MultiArray(data=j_row_arr.tolist()))
    except Exception:
        pass
    
    try:
        hazard_msg = String(data=str(closest_label))
        pubs.hazard_pub.publish(hazard_msg)
        pubs.closest_hazard_pub.publish(hazard_msg)
    except Exception:
        pass
    
    try:
        pubs.constraints_pub.publish(Float64MultiArray(data=[0.0]))
    except Exception:
        pass
