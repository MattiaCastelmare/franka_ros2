#!/usr/bin/env python3
"""
ONLINE AVOIDANCE CONTROLLER — NULL SPACE VERSION
===============================================

• Capsule-based distance estimation
• Potential field used ONLY as direction metric
• Avoidance projected in EE null space
• Tracking task is NEVER opposed
• No local minima blocking

Author: Maurizio (Null-space refactor)
"""

import os
import tempfile
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from moveit_msgs.msg import PlanningScene
from rcl_interfaces.srv import GetParameters

import pinocchio as pin


class NullSpaceAvoidance(Node):

    def __init__(self):
        super().__init__("online_avoidance_controller")

        # ================= PARAMETERS =================
        self.declare_parameter("control_rate", 100.0)
        self.declare_parameter("influence_distance", 0.30)
        self.declare_parameter("safety_margin", 0.08)
        self.declare_parameter("nullspace_gain", 0.15)
        self.declare_parameter("max_joint_velocity", 0.25)
        self.declare_parameter("excluded_obstacles", ["ground", "plane", "floor"])

        self.rate = float(self.get_parameter("control_rate").value)
        self.d_infl = float(self.get_parameter("influence_distance").value)
        self.d_safe = float(self.get_parameter("safety_margin").value)
        self.k_null = float(self.get_parameter("nullspace_gain").value)
        self.max_qdot = float(self.get_parameter("max_joint_velocity").value)
        self.excluded = list(self.get_parameter("excluded_obstacles").value)

        # ================= CAPSULE GEOMETRY =================
        self.link_pairs = [
            ("fr3_link1", "fr3_link2"),
            ("fr3_link2", "fr3_link3"),
            ("fr3_link3", "fr3_link4"),
            ("fr3_link4", "fr3_link5"),
            ("fr3_link5", "fr3_link6"),
            ("fr3_link6", "fr3_link7"),
            ("fr3_link7", "fr3_link8"),
        ]

        self.link_radius = {l: 0.08 for l, _ in self.link_pairs}
        self.capsules = {}

        # ================= STATE =================
        self.q = None
        self.frame_ids = {}
        self.obstacles = []
        self.pin_ok = False

        # ================= PINOCCHIO =================
        self._init_pinocchio_and_capsules()

        # ================= ROS =================
        self.create_subscription(JointState, "/joint_states", self._joint_cb, 10)
        self.create_subscription(PlanningScene, "/obstacle_scene", self._obstacle_cb, 1)

        self.pub = self.create_publisher(Float64MultiArray, "/avoidance/velocity", 10)
        self.min_dist_pub = self.create_publisher(Float64MultiArray, "/avoidance/min_distance", 10)

        self.create_timer(1.0 / self.rate, self._control_loop)

        self.get_logger().info("🟢 Null-Space Avoidance Controller READY")

    # ======================================================
    # PINOCCHIO + CAPSULES
    # ======================================================
    def _init_pinocchio_and_capsules(self):
        cli = self.create_client(GetParameters, "/robot_state_publisher/get_parameters")
        cli.wait_for_service()

        req = GetParameters.Request()
        req.names = ["robot_description"]
        future = cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        urdf = future.result().values[0].string_value

        with tempfile.NamedTemporaryFile(delete=False, suffix=".urdf") as f:
            f.write(urdf.encode())
            urdf_path = f.name

        model_full = pin.buildModelFromUrdf(urdf_path)
        os.unlink(urdf_path)

        lock = [model_full.getJointId(n) for n in model_full.names if "finger" in n]
        self.model = pin.buildReducedModel(model_full, lock, pin.neutral(model_full))
        self.data = self.model.createData()

        for parent, child in self.link_pairs:
            for link in (parent, child):
                if link not in self.frame_ids:
                    self.frame_ids[link] = self.model.getFrameId(link)

        q0 = pin.neutral(self.model)
        pin.forwardKinematics(self.model, self.data, q0)
        pin.updateFramePlacements(self.model, self.data)

        for parent, child in self.link_pairs:
            fid_p = self.frame_ids[parent]
            fid_c = self.frame_ids[child]

            oMp = self.data.oMf[fid_p]
            oMc = self.data.oMf[fid_c]

            p_child_local = oMp.rotation.T @ (oMc.translation - oMp.translation)

            self.capsules[parent] = {
                "p0": np.zeros(3),
                "p1": 0.95 * p_child_local,
                "radius": self.link_radius[parent],
            }

        self.pin_ok = True

    # ======================================================
    # CALLBACKS
    # ======================================================
    def _joint_cb(self, msg: JointState):
        order = [
            "fr3_joint1", "fr3_joint3", "fr3_joint2",
            "fr3_joint4", "fr3_joint6", "fr3_joint5", "fr3_joint7"
        ]

        try:
            q_ros = np.array([msg.position[msg.name.index(n)] for n in order])
        except ValueError:
            return

        self.q = np.array([
            q_ros[0], q_ros[2], q_ros[1],
            q_ros[3], q_ros[5], q_ros[4], q_ros[6]
        ])

    def _obstacle_cb(self, msg: PlanningScene):
        self.obstacles = [
            o for o in msg.world.collision_objects
            if not any(ex in o.id.lower() for ex in self.excluded)
        ]

    # ======================================================
    # CAPSULE ↔ BOX DISTANCE
    # ======================================================
    def _distance_capsule_to_box(self, p0, p1, r, obs):
        best_d = 1e6
        best_dir = None

        v = p1 - p0
        L = np.linalg.norm(v)
        if L < 1e-6:
            return best_d, None

        v /= L

        for i, prim in enumerate(obs.primitives):
            if prim.type != prim.BOX:
                continue

            pose = obs.primitive_poses[i]
            center = np.array([pose.position.x, pose.position.y, pose.position.z])
            half = np.array(prim.dimensions) / 2.0

            t = np.clip(np.dot(center - p0, v), 0, L)
            p_seg = p0 + t * v
            p_box = center + np.clip(p_seg - center, -half, +half)

            diff = p_seg - p_box
            dist = np.linalg.norm(diff) - r

            if dist < best_d:
                best_d = dist
                best_dir = diff / (np.linalg.norm(diff) + 1e-6)

        return best_d, best_dir

    # ======================================================
    # CONTROL LOOP (NULL SPACE)
    # ======================================================
    def _control_loop(self):
        zero = Float64MultiArray()
        zero.data = [0.0] * 7

        if not (self.pin_ok and isinstance(self.q, np.ndarray)):
            self.pub.publish(zero)
            return

        pin.forwardKinematics(self.model, self.data, self.q)
        pin.updateFramePlacements(self.model, self.data)

        qdot_ns = np.zeros(7)
        d_min = 999.0

        for parent in self.capsules:
            fid = self.frame_ids[parent]
            caps = self.capsules[parent]

            oMp = self.data.oMf[fid]
            p0 = oMp.translation + oMp.rotation @ caps["p0"]
            p1 = oMp.translation + oMp.rotation @ caps["p1"]

            for obs in self.obstacles:
                d, dir_vec = self._distance_capsule_to_box(p0, p1, caps["radius"], obs)
                if dir_vec is None:
                    continue

                d_min = min(d_min, d)

                if d >= self.d_infl:
                    continue

                alpha = np.clip((self.d_infl - d) / (self.d_infl - self.d_safe), 0, 1)
                xdot_avoid = self.k_null * alpha * dir_vec

                J = pin.computeFrameJacobian(
                    self.model, self.data, self.q, fid, pin.ReferenceFrame.WORLD
                )[:3, :]

                J_pinv = np.linalg.pinv(J)
                N = np.eye(7) - J_pinv @ J

                qdot_raw = J.T @ xdot_avoid
                qdot_ns += N @ qdot_raw

        if np.linalg.norm(qdot_ns) > self.max_qdot:
            qdot_ns *= self.max_qdot / np.linalg.norm(qdot_ns)

        self.pub.publish(Float64MultiArray(data=qdot_ns.tolist()))
        self.min_dist_pub.publish(Float64MultiArray(data=[float(d_min)]))


# ======================================================
# MAIN
# ======================================================
def main(args=None):
    rclpy.init(args=args)
    node = NullSpaceAvoidance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == "__main__":
    main()
