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
    build_joint_to_joint_capsules,
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
    capsule_radii: Sequence[float],
) -> tuple[bool, Any, Any, dict[str, int], dict[str, list[dict[str, Any]]]]:
    """Initialize Pinocchio model/data and joint-to-joint capsule geometry.
    
    Creates 8 capsules connecting consecutive joints:
    - cap 0: fr3_link0  → fr3_joint1
    - cap 1: fr3_joint1 → fr3_joint2
    - cap 2: fr3_joint2 → fr3_joint3
    - cap 3: fr3_joint3 → fr3_joint4
    - cap 4: fr3_joint4 → fr3_joint5
    - cap 5: fr3_joint5 → fr3_joint6
    - cap 6: fr3_joint6 → fr3_joint7
    - cap 7: fr3_joint7 → fr3_link8

    Returns:
      (pin_ok, model, data, frame_ids, capsules)
      
    Note: frame_ids is now an empty dict since capsules store joint/frame IDs directly.
    """
    urdf_xml = fetch_robot_description(node)
    if not urdf_xml:
        node.get_logger().error("robot_description is empty or missing")
        return False, None, None, {}, {}

    try:
        import pinocchio as pin
        model, data = build_reduced_pinocchio_model_from_urdf(urdf_xml)
        capsules = build_joint_to_joint_capsules(
            model=model,
            capsule_radii=list(capsule_radii),
        )
        # frame_ids is now empty since each capsule stores its own joint/frame IDs
        frame_ids: dict[str, int] = {}
        
        node.get_logger().info(f"✓ Created {len(capsules)} joint-to-joint capsules")
        
        # Debug: log target lengths for modified capsules
        # Use a neutral configuration to compute actual capsule lengths
        import numpy as np
        q_neutral = pin.neutral(model)
        pin.forwardKinematics(model, data, q_neutral)
        pin.updateFramePlacements(model, data)
        
        for cap_name in ["fr3_cap_2", "fr3_cap_4", "fr3_cap_5"]:
            if cap_name in capsules:
                cap = capsules[cap_name][0]
                # Get positions
                start_id, end_id = int(cap["start_id"]), int(cap["end_id"])
                start_type, end_type = cap["start_type"], cap["end_type"]
                
                if start_type == "joint":
                    p0 = np.array(data.oMi[start_id].translation).reshape(3)
                else:
                    p0 = np.array(data.oMf[start_id].translation).reshape(3)
                
                if end_type == "joint":
                    p1 = np.array(data.oMi[end_id].translation).reshape(3)
                else:
                    p1 = np.array(data.oMf[end_id].translation).reshape(3)
                
                # Apply shortening if specified
                if "target_length" in cap:
                    direction = p1 - p0
                    current_len = np.linalg.norm(direction)
                    target_len = float(cap["target_length"])
                    if current_len > 1e-6:
                        p1 = p0 + (direction / current_len) * target_len
                
                final_len = float(np.linalg.norm(p1 - p0))
                target_str = f" (target={cap['target_length']:.4f}m)" if "target_length" in cap else ""
                node.get_logger().info(f"  {cap_name}: length={final_len:.4f}m{target_str}")
        
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
