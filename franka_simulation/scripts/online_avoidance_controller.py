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
    scan_external_and_ground,
    scan_self_collision,
)
from utils.avoidance_math import point_jacobian_world
from utils.params import NullSpaceAvoidanceParams, load_controller_params
from utils.rviz_markers import build_marker_array
from utils.ros_publishers import PublishersBundle, publish_not_ready_outputs
from utils.ros_setup import (
    init_pinocchio_and_capsules,
    make_joint_state_callback,
    make_planning_scene_callback,
)


class NullSpaceAvoidance(Node):

    def __init__(self):
        super().__init__("online_avoidance_controller")

        self.joint_names = [
            "fr3_joint1", "fr3_joint2", "fr3_joint3", "fr3_joint4",
            "fr3_joint5", "fr3_joint6", "fr3_joint7",
        ]
        self.n_dof = len(self.joint_names)
        self.params: NullSpaceAvoidanceParams = load_controller_params(self)

        self.q = None
        self.frame_ids = {}
        self.obstacles: List[dict] = []
        self.capsules = {}
        self.pin_ok = False
        self.distances_data: List[dict] = []
        self.last_marker_array = MarkerArray()
        self._scene_log_last_wall = 0.0

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
            _,
            d_min,
            external_best,
            ground_best,
            dist_ext_ground,
            external_candidates,
            tip_to_obstacle_distances,
        ) = scan_external_and_ground(
            segments=segments,
            obstacles=list(self.obstacles),
            model=self.model,
            data=self.data,
            q=self.q,
            box_projection_iters=int(self.params.box_projection_iters),
            repulsion_spread_enable=bool(self.params.repulsion_spread_enable),
            repulsion_spread_samples=int(self.params.repulsion_spread_samples),
            repulsion_spread_half_length=float(self.params.repulsion_spread_half_length),
            influence_distance=float(self.params.influence_distance),
            d_aggr=float(self.params.d_aggr),
            safety_margin=float(self.params.safety_margin),
            k_aggr=float(self.params.k_aggr),
            k_null=float(self.params.k_null),
            k_tan=float(self.params.k_tan),
            max_qdot=float(self.params.max_qdot),
            avoidance_contrib_max_ratio=float(getattr(self.params, "avoidance_contrib_max_ratio", 0.0)),
            enable_ground=bool(self.params.enable_ground),
            ground_z=float(self.params.ground_z),
            ground_infl=float(self.params.ground_infl),
            ground_safe=float(self.params.ground_safe),
            k_ground=float(self.params.k_ground),
            debug_stats=None,
        )

        (
            _,
            d_min,
            self_best,
            dist_self,
            self_candidates,
        ) = scan_self_collision(
            segments=segments,
            model=self.model,
            data=self.data,
            q=self.q,
            enable_self=bool(self.params.enable_self),
            self_skip_adjacent_links=int(self.params.self_skip_adjacent),
            self_infl=float(self.params.self_infl),
            self_safe=float(self.params.self_safe),
            k_self=float(self.params.k_self),
            d_min_in=float(d_min),
        )

        self.distances_data = list(dist_ext_ground)
        if dist_self:
            self.distances_data.extend(dist_self)
        if tip_to_obstacle_distances:
            self.distances_data.extend(tip_to_obstacle_distances)

        best_candidates: List[dict] = []
        if isinstance(external_best, dict):
            best_candidates.extend(list(external_best.values()))
        if ground_best is not None:
            best_candidates.append(ground_best)
        if self_best is not None:
            best_candidates.append(self_best)

        closest_candidate = None
        closest_distance = float("inf")
        for candidate in best_candidates:
            d_val = float(candidate.get("d", float("inf")))
            if not np.isfinite(d_val):
                continue
            if d_val < closest_distance:
                closest_distance = d_val
                closest_candidate = candidate

        d_min_raw = float(d_min) if np.isfinite(float(d_min)) else 999.0

        closest_label = "none"
        if closest_candidate is not None:
            hazard = str(closest_candidate.get("hazard", "")).strip()
            kind = str(closest_candidate.get("kind", "")).strip()
            link_desc = ""
            if kind == "self":
                link_i = str(closest_candidate.get("link_i", "?"))
                link_j = str(closest_candidate.get("link_j", "?"))
                link_desc = f"{link_i}↔{link_j}"
            else:
                link_desc = str(closest_candidate.get("link", "")).strip()
            if hazard and link_desc:
                closest_label = hazard if "@" in hazard else f"{hazard}@{link_desc}"
            elif hazard:
                closest_label = hazard
            elif link_desc:
                closest_label = link_desc
            else:
                closest_label = kind or "hazard"

        closest_j_row = np.zeros(self.n_dof, dtype=float)
        if closest_candidate is not None:
            kind = str(closest_candidate.get("kind", "")).strip()
            if kind in ("external", "ground"):
                fid = int(closest_candidate.get("fid", -1))
                p = np.array(closest_candidate.get("p", [0.0, 0.0, 0.0]), dtype=float).reshape(3)
                n = np.array(closest_candidate.get("n", [0.0, 0.0, 0.0]), dtype=float).reshape(3)
                if fid >= 0:
                    n_norm = float(np.linalg.norm(n))
                    if n_norm > 1e-9:
                        n_unit = n / n_norm
                        Jp = point_jacobian_world(self.model, self.data, self.q, fid, p)
                        j_vec = (n_unit.reshape(1, 3) @ Jp).reshape(-1)
                        if j_vec.shape[0] == self.n_dof and np.all(np.isfinite(j_vec)):
                            closest_j_row = j_vec
            elif kind == "self":
                fid_i = int(closest_candidate.get("fid_i", -1))
                fid_j = int(closest_candidate.get("fid_j", -1))
                p_i = np.array(closest_candidate.get("p_i", [0.0, 0.0, 0.0]), dtype=float).reshape(3)
                p_j = np.array(closest_candidate.get("p_j", [0.0, 0.0, 0.0]), dtype=float).reshape(3)
                n = np.array(closest_candidate.get("n", [0.0, 0.0, 0.0]), dtype=float).reshape(3)
                n_norm = float(np.linalg.norm(n))
                if fid_i >= 0 and fid_j >= 0 and n_norm > 1e-9:
                    n_unit = n / n_norm
                    J_i = point_jacobian_world(self.model, self.data, self.q, fid_i, p_i)
                    J_j = point_jacobian_world(self.model, self.data, self.q, fid_j, p_j)
                    j_vec = (n_unit.reshape(1, 3) @ (J_i - J_j)).reshape(-1)
                    if j_vec.shape[0] == self.n_dof and np.all(np.isfinite(j_vec)):
                        closest_j_row = j_vec

        active_candidates = list(external_candidates) + list(self_candidates)
        if ground_best is not None:
            active_candidates.append(ground_best)

        def _is_active(candidate: dict) -> bool:
            kind = str(candidate.get("kind", "")).strip()
            d_val = float(candidate.get("d", float("inf")))
            if not np.isfinite(d_val):
                return False
            if kind == "self":
                return d_val <= float(self.params.self_infl)
            if kind == "ground":
                return d_val <= float(self.params.ground_infl)
            return d_val <= float(self.params.influence_distance)

        active_count = sum(1 for cand in active_candidates if _is_active(cand))
        d_for_log = d_min_raw if np.isfinite(d_min_raw) else 999.0
        if (now_wall - float(self._scene_log_last_wall)) >= 0.5:
            self._scene_log_last_wall = now_wall
            self.get_logger().info(
                f"[SCENE-MIN] obs={len(self.obstacles)} active={active_count} closest='{closest_label}' d_min={d_for_log:.3f}"
            )

        qdot_zeros = np.zeros(self.n_dof, dtype=float)
        self.pub.publish(Float64MultiArray(data=qdot_zeros.tolist()))
        self.min_dist_raw_pub.publish(Float64MultiArray(data=[d_min_raw]))
        self.min_dist_pub.publish(Float64MultiArray(data=[d_min_raw]))

        closest_payload = [d_min_raw] + closest_j_row.tolist()
        self.closest_constraint_pub.publish(Float64MultiArray(data=closest_payload))
        self.jac_pub.publish(Float64MultiArray(data=closest_j_row.tolist()))
        hazard_msg = String(data=str(closest_label))
        self.hazard_pub.publish(hazard_msg)
        self.closest_hazard_pub.publish(hazard_msg)
        self.constraints_pub.publish(Float64MultiArray(data=[0.0]))

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
    node = NullSpaceAvoidance()
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