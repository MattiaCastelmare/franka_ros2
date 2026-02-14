#!/usr/bin/env python3
"""
ONLINE AVOIDANCE CONTROLLER — MINIMAL DISTANCE & MARKER VERSION
===============================================================
"""

import time
from typing import List

import numpy as np
import pinocchio as pin
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String
from visualization_msgs.msg import MarkerArray
from moveit_msgs.msg import PlanningScene

from utils.avoidance_core import (
    iter_world_capsule_segments,
    scan_external,
)
from utils.avoidance_math import compute_closest_constraint_jacobian
from utils.closest_constraint import select_closest_candidate, format_closest_label
from utils.logging import log_capsule_distances_throttled
from utils.params import AvoidanceControllerParams, load_avoidance_controller_params
from utils.rviz_markers import build_marker_array
from utils.ros_publishers import (
    PublishersBundle,
    publish_not_ready_outputs,
    publish_minimal_avoidance_outputs,
)
from utils.ros_setup import (
    init_pinocchio_and_capsules,
    make_joint_state_callback,
    make_planning_scene_callback,
)


class AvoidanceController(Node):

    def __init__(self):
        super().__init__("online_avoidance_controller")

        self.joint_names = [
            "fr3_joint1", "fr3_joint2", "fr3_joint3", "fr3_joint4",
            "fr3_joint5", "fr3_joint6", "fr3_joint7",
        ]
        self.n_dof = len(self.joint_names)
        self.params: AvoidanceControllerParams = load_avoidance_controller_params(self)

        self.q = None
        self.frame_ids = {}
        self.obstacles: List[dict] = []
        self.capsules = {}
        self.pin_ok = False
        self.distances_data: List[dict] = []
        self.last_marker_array = MarkerArray()
        self._scene_log_last_wall = 0.0
        self._distance_log_last_wall = 0.0

        # Initialize Pinocchio model and joint-to-joint capsules
        # Capsules are now built directly from joint positions (8 capsules total)
        self.pin_ok, self.model, self.data, self.frame_ids, self.capsules = init_pinocchio_and_capsules(
            self,
            capsule_radii=list(self.params.capsule_radii),
        )

        marker_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.capsule_marker_pub = self.create_publisher(
            MarkerArray,
            "/robot_capsules_markers",
            marker_qos,
        )

        self.create_subscription(
            JointState,
            "/joint_states",
            make_joint_state_callback(controller=self, joint_names=self.joint_names),
            10,
        )
        self.create_subscription(
            PlanningScene,
            "/obstacle_scene",
            make_planning_scene_callback(controller=self, excluded_substrings=list(self.params.excluded)),
            1,
        )

        self.pub = self.create_publisher(Float64MultiArray, "/avoidance/velocity", 10)
        self.min_dist_pub = self.create_publisher(Float64MultiArray, "/avoidance/min_distance", 10)
        self.min_dist_raw_pub = self.create_publisher(Float64MultiArray, "/avoidance/min_distance_raw", 10)
        self.closest_constraint_pub = self.create_publisher(Float64MultiArray, "/avoidance/closest_constraint", 10)
        self.closest_hazard_pub = self.create_publisher(String, "/avoidance/closest_hazard", 10)
        self.constraints_pub = self.create_publisher(Float64MultiArray, "/avoidance/constraints", 10)
        self.jac_pub = self.create_publisher(Float64MultiArray, "/avoidance/jacobian", 10)
        self.hazard_pub = self.create_publisher(String, "/avoidance/hazard", 10)

        self._pubs = PublishersBundle(
            pub=self.pub,
            min_dist_pub=self.min_dist_pub,
            min_dist_raw_pub=self.min_dist_raw_pub,
            closest_constraint_pub=self.closest_constraint_pub,
            closest_hazard_pub=self.closest_hazard_pub,
            constraints_pub=self.constraints_pub,
            jac_pub=self.jac_pub,
            hazard_pub=self.hazard_pub,
            capsule_marker_pub=self.capsule_marker_pub,
        )

        self.create_timer(1.0 / float(self.params.rate), self._control_loop)
        self.create_timer(0.1, self._publish_markers_only)

        self.get_logger().info("🟢 Null-Space Avoidance Controller READY (minimal mode)")

    def _control_loop(self):
        now_wall = float(time.time())

        if not (self.pin_ok and isinstance(self.q, np.ndarray)):
            publish_not_ready_outputs(pubs=self._pubs)
            return

        pin.forwardKinematics(self.model, self.data, self.q)
        pin.updateFramePlacements(self.model, self.data)

        segments = iter_world_capsule_segments(
            capsules=self.capsules,
            frame_ids=self.frame_ids,
            data=self.data,
            debug_capsule_index=int(self.params.debug_capsule_index),
        )

        (
            d_min,
            external_best,
            dist_ext_ground,
            external_candidates,
            tip_to_obstacle_distances,
        ) = scan_external(
            segments=segments,
            obstacles=list(self.obstacles),
            box_projection_iters=int(self.params.box_projection_iters),
            repulsion_spread_enable=bool(self.params.repulsion_spread_enable),
            repulsion_spread_samples=int(self.params.repulsion_spread_samples),
            repulsion_spread_half_length=float(self.params.repulsion_spread_half_length),
            influence_distance=float(self.params.influence_distance),
            debug_stats=None,
        )

        self.distances_data = list(dist_ext_ground)
        if tip_to_obstacle_distances:
            self.distances_data.extend(tip_to_obstacle_distances)

        best_candidates: List[dict] = []
        if isinstance(external_best, dict):
            best_candidates.extend(list(external_best.values()))

        # A) Select closest candidate
        closest_candidate, closest_distance, d_min_raw = select_closest_candidate(best_candidates)

        # B) Format closest label
        closest_label = format_closest_label(closest_candidate)

        # C) Compute Jacobian row for closest constraint
        closest_j_row = compute_closest_constraint_jacobian(
            closest_candidate,
            self.model,
            self.data,
            self.q,
            n_dof=self.n_dof,
        )

        # D) Count active candidates (external obstacles only)
        active_candidates = list(external_candidates)
        active_count = len(active_candidates)
        
        d_for_log = d_min_raw if np.isfinite(d_min_raw) else 999.0
        
        # F) Throttled logging of capsule-obstacle distances (every 0.5s)
        self._distance_log_last_wall = log_capsule_distances_throttled(
            distances_data=dist_ext_ground,
            obstacles=self.obstacles,
            logger=self.get_logger(),
            last_wall=self._distance_log_last_wall,
            now_wall=now_wall,
            throttle_s=0.5,
        )
        
        if (now_wall - float(self._scene_log_last_wall)) >= 0.5:
            self._scene_log_last_wall = now_wall
            self.get_logger().info(
                f"[SCENE-MIN] obs={len(self.obstacles)} active={active_count} closest='{closest_label}' d_min={d_for_log:.3f}"
            )

        # E) Publish all outputs via consolidated helper
        qdot_zeros = np.zeros(self.n_dof, dtype=float)
        publish_minimal_avoidance_outputs(
            pubs=self._pubs,
            qdot=qdot_zeros,
            d_min_raw=d_min_raw,
            closest_j_row=closest_j_row,
            closest_label=closest_label,
        )

        self.last_marker_array = build_marker_array(
            capsules=self.capsules,
            frame_ids=self.frame_ids,
            data=self.data,
            distances_data=self.distances_data,
            influence_distance=float(self.params.influence_distance),
            distance_inflation=float(self.params.distance_inflation),
            stamp_msg=self.get_clock().now().to_msg(),
            logger=self.get_logger(),
            debug_capsule_index=int(self.params.debug_capsule_index),
        )

    def _publish_markers_only(self):
        if len(self.last_marker_array.markers) > 0:
            self.capsule_marker_pub.publish(self.last_marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = AvoidanceController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()