#!/usr/bin/env python3
"""
ONLINE AVOIDANCE CONTROLLER — BASELINE-1 (Potential Field Clean Version)
=======================================================================

Implementa campi potenziali ben definiti (Khatib / Palmieri & Scoccia):
 - Potenziale repulsivo continuo
 - Blending morbido sul range [safety_margin, influence_distance]
 - Forza repulsiva cartesiana
 - Mappatura a velocità joint: qdot_rep = J^T * F
 - Effetto estremamente più forte, MATEMATICAMENTE corretto
"""

import os
import tempfile
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from moveit_msgs.msg import PlanningScene, CollisionObject
from rcl_interfaces.srv import GetParameters
import pinocchio as pin
import yaml


class Baseline1Avoidance(Node):

    def __init__(self):
        super().__init__("online_avoidance_controller")

        # ===== PARAMETRI YAML (uguali ai tuoi) =====
        self.declare_parameter("control_rate", 100.0)
        self.declare_parameter("influence_distance", 0.30)
        self.declare_parameter("safety_margin", 0.08)
        self.declare_parameter("repulsive_gain", 1.0)    # più forte
        self.declare_parameter("max_joint_velocity", 1.2)

        self.declare_parameter("control_points",
            ["fr3_link4", "fr3_link5", "fr3_link6", "fr3_link7", "fr3_link8"]
        )
        self.declare_parameter("excluded_obstacles", ["ground", "plane", "floor"])
        self.declare_parameter("control_points_offsets_yaml", '')

        # ===== LETTURA PARAMETRI =====
        self.rate = self.get_parameter("control_rate").value
        self.d_infl = self.get_parameter("influence_distance").value
        self.d_safe = self.get_parameter("safety_margin").value
        self.K = self.get_parameter("repulsive_gain").value
        self.max_qdot = self.get_parameter("max_joint_velocity").value

        self.control_links = list(self.get_parameter("control_points").value)
        self.excluded = list(self.get_parameter("excluded_obstacles").value)

        # Control points offsets
        offsets_yaml = self.get_parameter("control_points_offsets_yaml").value
        self.control_offsets = {}
        if offsets_yaml:
            self.control_offsets = yaml.safe_load(offsets_yaml)
        else:
            # 1 punto centrale di default
            for link in self.control_links:
                self.control_offsets[link] = [[0.0, 0.0, 0.0]]

        # Stato
        self.joint_positions = None
        self.frame_ids = {}
        self.obstacles = []
        self.pin_ok = False
        self.loop = 0

        # ===== PINOCCHIO =====
        self._init_pinocchio()

        # ===== ROS I/O =====
        self.create_subscription(JointState, "/joint_states", self._joint_cb, 10)
        self.create_subscription(
            PlanningScene, "/obstacle_scene",
            self._obstacle_cb, 1
        )
        self.pub = self.create_publisher(Float64MultiArray, "/avoidance/velocity", 10)

        # Timer
        self.create_timer(1.0/self.rate, self._control_loop)

        self.get_logger().info("🔥 Baseline-1 Potential Field Avoidance READY")


    # ----------------------------------------------------------------------
    # PINOCCHIO INIT
    # ----------------------------------------------------------------------
    def _init_pinocchio(self):
        try:
            cli = self.create_client(GetParameters, "/robot_state_publisher/get_parameters")
            cli.wait_for_service(timeout_sec=10.0)

            req = GetParameters.Request()
            req.names = ["robot_description"]
            future = cli.call_async(req)
            rclpy.spin_until_future_complete(self, future)

            urdf_str = future.result().values[0].string_value
            with tempfile.NamedTemporaryFile(delete=False, suffix=".urdf", mode="w") as f:
                f.write(urdf_str)
                urdf_path = f.name

            model_full = pin.buildModelFromUrdf(urdf_path)
            os.unlink(urdf_path)

            # Rimuovi le dita
            lock = [model_full.getJointId(n) for n in model_full.names if "finger" in n]
            self.model = pin.buildReducedModel(model_full, lock, pin.neutral(model_full))
            self.data = self.model.createData()

            for link in self.control_links:
                try:
                    self.frame_ids[link] = self.model.getFrameId(link)
                    self.get_logger().info(f"  ✓ frame {link}: {self.frame_ids[link]}")
                except:
                    self.get_logger().warn(f"⚠️ Frame non trovato: {link}")

            self.pin_ok = True
        except Exception as e:
            self.get_logger().error(f"Pinocchio ERROR: {e}")


    # ----------------------------------------------------------------------
    # CALLBACKS
    # ----------------------------------------------------------------------
    def _joint_cb(self, msg):
        idx = [i for i, n in enumerate(msg.name) if "fr3_joint" in n and "finger" not in n]
        if len(idx) == 7:
            self.joint_positions = np.array([msg.position[i] for i in idx])


    def _obstacle_cb(self, msg):
        self.obstacles = []
        for obs in msg.world.collision_objects:
            if any(ex in obs.id.lower() for ex in self.excluded):
                continue
            self.obstacles.append(obs)


    # ----------------------------------------------------------------------
    # POTENTIAL FIELD FUNCTIONS
    # ----------------------------------------------------------------------
    def potential_force(self, d, dir_vec):
        """
        Campo potenziale continuo (Khatib-style).
        d = distanza point->box
        dir_vec = direzione normalizzata dal box verso il robot
        """
        if d >= self.d_infl:
            return np.zeros(3)

        # Se molto vicino al limite → effetto fortissimo
        if d <= self.d_safe:
            d = self.d_safe + 1e-3

        term = (1.0/(d - self.d_safe) - 1.0/(self.d_infl - self.d_safe))
        dU_dd = self.K * term * (1.0/((d - self.d_safe)**2))

        return dU_dd * dir_vec


    # ----------------------------------------------------------------------
    # CONTROL LOOP
    # ----------------------------------------------------------------------
    def _control_loop(self):
        self.loop += 1

        zero = Float64MultiArray()
        zero.data = [0.0]*7

        if not (self.pin_ok and isinstance(self.joint_positions, np.ndarray)):
            self.pub.publish(zero)
            return

        if len(self.obstacles) == 0:
            self.pub.publish(zero)
            return

        q = self.joint_positions

        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)

        q_dot_rep = np.zeros(7)
        active = 0
        global_min = 999

        # === PER OGNI LINK ===
        for link in self.control_links:
            if link not in self.frame_ids:
                continue

            fid = self.frame_ids[link]
            pose = self.data.oMf[fid]
            p_link = np.array(pose.translation)
            R_link = pose.rotation

            offsets = self.control_offsets[link]

            # === PER OGNI CONTROL POINT ===
            for offs in offsets:
                p = p_link + R_link @ np.array(offs)

                # distanza dal box più vicino
                min_d, dir_vec = self._nearest_obstacle(p)

                if min_d < global_min:
                    global_min = min_d

                if min_d < self.d_infl:
                    active += 1

                    # Forza repulsiva cartesiana
                    F = self.potential_force(min_d, dir_vec)

                    if np.linalg.norm(F) > 0:
                        # Jacobiano posizione
                        J = pin.computeFrameJacobian(
                            self.model, self.data, q, fid, pin.ReferenceFrame.WORLD
                        )
                        Jpos = J[:3, :]  # prime 3 righe = posizione

                        # qdot_rep_i = J^T * F
                        q_dot_rep += Jpos.T @ F

        # Debug
        if self.loop % 40 == 0:
            self.get_logger().info(
                f"📏 min_dist = {global_min:.3f} m (infl={self.d_infl}) | active={active}"
            )

        # === OUTPUT ===
        if active > 0:
            # Normalizza e clippa
            norm = np.linalg.norm(q_dot_rep)
            if norm > self.max_qdot:
                q_dot_rep = q_dot_rep / norm * self.max_qdot

            msg = Float64MultiArray()
            msg.data = q_dot_rep.tolist()
            self.pub.publish(msg)
        else:
            self.pub.publish(zero)


    # ----------------------------------------------------------------------
    # DISTANZA PUNTO - BOX
    # ----------------------------------------------------------------------
    def _nearest_obstacle(self, point):
        min_d = 999
        best_dir = np.array([1.0, 0.0, 0.0])

        for obs in self.obstacles:
            d, dir_vec = self._distance_to_box(point, obs)
            if d < min_d:
                min_d = d
                best_dir = dir_vec

        return min_d, best_dir


    def _distance_to_box(self, point, obs):
        """Retorna: distanza, direzione normalizzata (punto ← box)."""
        best = 999
        best_dir = np.array([1.0,0,0])

        for i, prim in enumerate(obs.primitives):
            if prim.type != prim.BOX:
                continue

            pose = obs.primitive_poses[i]
            center = np.array([pose.position.x, pose.position.y, pose.position.z])
            dims = np.array(prim.dimensions)
            half = dims/2.0

            rel = point - center
            closest = center + np.clip(rel, -half, +half)
            diff = point - closest
            d = np.linalg.norm(diff)

            if d > 1e-6:
                dir_vec = diff / d
            else:
                # dentro al box: spingi verso la faccia più vicina
                face_dir = np.zeros(3)
                axis = np.argmin(half - np.abs(rel))
                face_dir[axis] = 1.0 if rel[axis] > 0 else -1.0
                dir_vec = face_dir
                d = 1e-3

            if d < best:
                best = d
                best_dir = dir_vec

        return best, best_dir


def main(args=None):
    rclpy.init(args=args)
    node = Baseline1Avoidance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == "__main__":
    main()
