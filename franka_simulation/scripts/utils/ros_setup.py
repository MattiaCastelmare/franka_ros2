"""ROS-facing helpers for the online avoidance controller.

We keep these bits out of `online_avoidance_controller.py` to keep the node
readable at a high level.

Unlike `avoidance_core.py` / `cbf_filter.py`, this module is intentionally
*ROS-dependent* (rclpy + ROS message/service types).
"""

from __future__ import annotations

from typing import Any, Callable, Optional, Sequence, Tuple

import rclpy
from rcl_interfaces.srv import GetParameters
from sensor_msgs.msg import JointState
from moveit_msgs.msg import PlanningScene

from .avoidance_math import (
    build_capsules_for_link_pairs,
    build_reduced_pinocchio_model_from_urdf,
    filtered_collision_objects_from_planning_scene,
    ordered_joint_positions_from_joint_state,
)


def fetch_robot_description(
    node: Any,
    *,
    service_name: str = "/robot_state_publisher/get_parameters",
    timeout_sec: float = 5.0,
) -> Optional[str]:
    """Fetch `robot_description` (URDF XML string) from robot_state_publisher."""
    client = node.create_client(GetParameters, service_name)
    if not client.wait_for_service(timeout_sec=float(timeout_sec)):
        node.get_logger().error(f"{service_name} service not available")
        return None

    req = GetParameters.Request()
    req.names = ["robot_description"]

    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=float(timeout_sec))
    if not future.done():
        node.get_logger().error("Timeout while requesting robot_description")
        return None

    try:
        res = future.result()
    except Exception as e:
        node.get_logger().error(f"Failed to get robot_description: {e}")
        return None

    try:
        if res is None or len(res.values) <= 0:
            return None
        return str(res.values[0].string_value)
    except Exception:
        return None


def init_pinocchio_and_capsules(
    node: Any,
    *,
    link_pairs: Sequence[Tuple[str, str]],
    capsule_fractions: Sequence[float],
    capsule_radii: Sequence[float],
) -> tuple[bool, Any, Any, dict[str, int], dict[str, list[dict[str, Any]]]]:
    """Initialize Pinocchio model/data and capsule geometry.

    Returns:
      (pin_ok, model, data, frame_ids, capsules)
    """
    urdf_xml = fetch_robot_description(node)
    if not urdf_xml:
        node.get_logger().error("robot_description is empty or missing")
        return False, None, None, {}, {}

    try:
        model, data = build_reduced_pinocchio_model_from_urdf(urdf_xml)
        frame_ids, capsules = build_capsules_for_link_pairs(
            model=model,
            data=data,
            link_pairs=list(link_pairs),
            capsule_fractions=list(capsule_fractions),
            capsule_radii=list(capsule_radii),
        )
        return True, model, data, frame_ids, capsules
    except Exception as e:
        node.get_logger().error(f"Failed to initialize Pinocchio/capsules: {e}")
        return False, None, None, {}, {}


def make_joint_state_callback(
    *,
    controller: Any,
    joint_names: Sequence[str],
) -> Callable[[JointState], None]:
    """Create a JointState callback that updates `controller.q`."""

    joint_names = list(joint_names)

    def _cb(msg: JointState) -> None:
        q = ordered_joint_positions_from_joint_state(msg, joint_names)
        if q is None:
            return
        controller.q = q

    return _cb


def make_planning_scene_callback(
    *,
    controller: Any,
    excluded_substrings: Sequence[str],
) -> Callable[[PlanningScene], None]:
    """Create a PlanningScene callback that updates `controller.obstacles`."""

    excluded_substrings = list(excluded_substrings)

    def _cb(msg: PlanningScene) -> None:
        controller.obstacles = filtered_collision_objects_from_planning_scene(msg, excluded_substrings)

    return _cb
