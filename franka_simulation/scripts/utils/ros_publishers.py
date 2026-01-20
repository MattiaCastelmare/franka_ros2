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

    # Hazard string (best-effort reset)
    try:
        msg = String(); msg.data = "none"
        pubs.hazard_pub.publish(msg)
    except Exception:
        pass
