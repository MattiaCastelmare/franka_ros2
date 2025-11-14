#!/usr/bin/env python3
"""
Online Avoidance Controller - Jacobian-based Null-Space Obstacle Avoidance
===========================================================================

Versione con Pinocchio (FK/Jacobiani da URDF di robot_state_publisher).

Author: Maurizio
Package: franka_simulation
"""

import os
import tempfile
from typing import Dict, Optional
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# ROS messages
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from moveit_msgs.msg import PlanningScene, CollisionObject
from geometry_msgs.msg import Point
from rcl_interfaces.srv import GetParameters

# Pinocchio
import pinocchio as pin
from pinocchio.utils import zero


class OnlineAvoidanceController(Node):
    """Controller reattivo per obstacle avoidance basato su Jacobiani (Pinocchio)."""

    def __init__(self):
        super().__init__("online_avoidance_controller")

        # PARAMETRI
        self.declare_parameters()
        self.load_parameters()

        # STATO INTERNO
        self.current_joint_positions: Optional[np.ndarray] = None
        self.current_joint_velocities: Optional[np.ndarray] = None
        self.obstacles = []
        self.pin_initialized = False

        # MODELLO PINOCCHIO
        self.get_logger().info("🤖 Inizializzazione modello Pinocchio da robot_description...")
        if not self.initialize_pin():
            self.get_logger().error("❌ Errore inizializzazione Pinocchio")
            return

        # SUBSCRIBERS
        self.joint_state_sub = self.create_subscription(
            JointState, "/joint_states", self.joint_state_callback, 10
        )

        planning_scene_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.planning_scene_sub = self.create_subscription(
            PlanningScene, self.planning_scene_topic, self.planning_scene_callback, planning_scene_qos
        )

        # PUBLISHER
        self.velocity_cmd_pub = self.create_publisher(
            Float64MultiArray, "/fr3_velocity_controller/commands", 10
        )

        # TIMER CONTROLLO
        control_period = 1.0 / self.control_rate
        self.control_timer = self.create_timer(control_period, self.control_loop)

        self.get_logger().info(f"🚀 Online Avoidance Controller attivo @ {self.control_rate} Hz")
        self.get_logger().info(f"📍 Control points: {self.control_points}")

    # ================================================================
    # INIZIALIZZAZIONE PINOCCHIO
    # ================================================================

    def initialize_pin(self) -> bool:
        """Costruisce il modello Pinocchio dal robot_description (URDF) ottenuto da RSP."""
        try:
            import time

            # 1) Preleva robot_description da robot_state_publisher
            max_wait = 10.0
            start_time = time.time()
            robot_description = None

            cli = self.create_client(GetParameters, "/robot_state_publisher/get_parameters")
            while (time.time() - start_time) < max_wait:
                if cli.wait_for_service(timeout_sec=1.0):
                    req = GetParameters.Request()
                    req.names = ["robot_description"]
                    future = cli.call_async(req)
                    rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
                    if future.result() and len(future.result().values) > 0:
                        robot_description = future.result().values[0].string_value
                        if robot_description:
                            break
                time.sleep(0.2)

            if not robot_description:
                self.get_logger().error("❌ Impossibile ottenere robot_description")
                return False

            # 2) Scrive l'URDF in un file temporaneo (Pinocchio accetta path file)
            with tempfile.NamedTemporaryFile(delete=False, suffix=".urdf", mode="w") as f:
                f.write(robot_description)
                urdf_path = f.name

            # 3) Costruisce modello e data
            # Nota: per sole cinematiche non servono pacchetti mesh; package_dirs può essere vuoto.
            self.model = pin.buildModelFromUrdf(urdf_path)
            self.data = self.model.createData()

            # 4) Frame IDs utili (EE e punti di controllo)
            def frame_id(name: str) -> int:
                fid = self.model.getFrameId(name)
                if fid == len(self.model.frames):
                    raise ValueError(f"Frame '{name}' non trovato nel modello Pinocchio.")
                return fid

            self.ee_fid = frame_id(self.ee_frame)
            self.cp_fids: Dict[str, int] = {ln: frame_id(ln) for ln in self.control_points}

            # 5) Dimensioni configurazione
            self.nq = self.model.nq
            self.nv = self.model.nv
            if self.nq != 7 or self.nv != 7:
                self.get_logger().warn(f"Atteso manipolatore a 7 dof, ma model: nq={self.nq}, nv={self.nv}")

            # 6) Stato q default
            self.q = zero(self.nq)
            self.pin_initialized = True

            # Cleanup file temporaneo
            try:
                os.unlink(urdf_path)
            except Exception:
                pass

            self.get_logger().info(
                f"✅ Pinocchio: modello caricato (nq={self.nq}, nv={self.nv}) | base={self.base_frame} ee={self.ee_frame}"
            )
            return True

        except Exception as e:
            self.get_logger().error(f"❌ Errore Pinocchio: {e}")
            import traceback

            traceback.print_exc()
            return False

    # ================================================================
    # PARAMETRI
    # ================================================================

    def declare_parameters(self):
        self.declare_parameter("control_rate", 50.0)
        self.declare_parameter("control_points", ["fr3_link3", "fr3_link5", "fr3_link7", "fr3_link8"])
        self.declare_parameter("influence_distance", 0.25)
        self.declare_parameter("safety_margin", 0.05)
        self.declare_parameter("repulsive_gain", 0.3)
        self.declare_parameter("damping_max", 0.05)
        self.declare_parameter("singularity_threshold", 0.05)
        self.declare_parameter("max_joint_velocity", 1.0)
        self.declare_parameter("planning_scene_topic", "/planning_scene")
        self.declare_parameter("monitor_planning_scene", True)
        self.declare_parameter("verbose_logging", False)
        self.declare_parameter("base_frame", "fr3_link0")
        self.declare_parameter("ee_frame", "fr3_link8")

    def load_parameters(self):
        self.control_rate = self.get_parameter("control_rate").value
        self.control_points = self.get_parameter("control_points").value
        self.influence_distance = self.get_parameter("influence_distance").value
        self.safety_margin = self.get_parameter("safety_margin").value
        self.repulsive_gain = self.get_parameter("repulsive_gain").value
        self.damping_max = self.get_parameter("damping_max").value
        self.singularity_threshold = self.get_parameter("singularity_threshold").value
        self.max_joint_velocity = self.get_parameter("max_joint_velocity").value
        self.planning_scene_topic = self.get_parameter("planning_scene_topic").value
        self.monitor_planning_scene = self.get_parameter("monitor_planning_scene").value
        self.verbose_logging = self.get_parameter("verbose_logging").value
        self.base_frame = self.get_parameter("base_frame").value
        self.ee_frame = self.get_parameter("ee_frame").value

    # ================================================================
    # CALLBACKS
    # ================================================================

    def joint_state_callback(self, msg: JointState):
        # Prende solo i 7 giunti del braccio (esclude le dita)
        fr3_indices = [i for i, n in enumerate(msg.name) if "fr3_joint" in n and "finger" not in n]
        if len(fr3_indices) == 7:
            self.current_joint_positions = np.array([msg.position[i] for i in fr3_indices], dtype=float)
            self.current_joint_velocities = np.array([msg.velocity[i] for i in fr3_indices], dtype=float)

    def planning_scene_callback(self, msg: PlanningScene):
        if self.monitor_planning_scene:
            self.obstacles = msg.world.collision_objects

    # ================================================================
    # CONTROL LOOP
    # ================================================================

    def control_loop(self):
        if not self.pin_initialized or self.current_joint_positions is None:
            return

        # Aggiorna stato in Pinocchio
        q = np.asarray(self.current_joint_positions, dtype=float)
        if q.shape[0] != self.nq:
            return

        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)

        # Jacobiano all'EE (per proiettore di spazio nullo)
        J = self.compute_jacobian(q, self.ee_fid)
        if J is None:
            return

        # Campo repulsivo (sommatoria per link di controllo)
        distances = self.compute_control_point_distances()
        q_dot_avoid = self.compute_repulsive_velocity(distances, q)

        # Proiezione in spazio nullo rispetto al task EE
        J_pinv = self.compute_damped_pseudoinverse(J)
        N = np.eye(self.nv) - J_pinv @ J
        q_dot_null = N @ q_dot_avoid

        q_dot_total = self.saturate_velocity(q_dot_null)
        self.publish_velocity_command(q_dot_total)

    # ================================================================
    # JACOBIAN E FK (Pinocchio)
    # ================================================================

    def compute_jacobian(self, q: np.ndarray, frame_id: int) -> Optional[np.ndarray]:
        try:
            # Jacobiano di frame in riferimento al mondo
            J6xN = pin.computeFrameJacobian(
                self.model, self.data, q, frame_id, pin.ReferenceFrame.WORLD
            )
            # Assicura (6 x nv)
            return np.asarray(J6xN)
        except Exception:
            return None

    def get_link_jacobian(self, link_name: str, q: np.ndarray) -> Optional[np.ndarray]:
        try:
            fid = self.cp_fids[link_name]
            return self.compute_jacobian(q, fid)
        except Exception:
            return None

    def get_link_position(self, link_name: str) -> Optional[np.ndarray]:
        try:
            fid = self.cp_fids[link_name]
            oMf = self.data.oMf[fid]  # posa del frame nel mondo
            return np.array(oMf.translation).reshape(3)
        except Exception:
            return None

    # ================================================================
    # DISTANZE
    # ================================================================

    def compute_control_point_distances(self) -> Dict:
        distances = {}
        if not self.obstacles:
            return {ln: (float("inf"), Point(), Point()) for ln in self.control_points}

        for link_name in self.control_points:
            pos = self.get_link_position(link_name)
            if pos is None:
                continue

            min_dist, closest_pt, normal = float("inf"), Point(), Point()
            for i_obs, obs in enumerate(self.obstacles):
                d, pt, n = self.compute_distance_to_obstacle(pos, obs)
                if d < min_dist:
                    min_dist, closest_pt, normal = d, pt, n

            distances[link_name] = (min_dist, closest_pt, normal)

        return distances

    def compute_distance_to_obstacle(self, point: np.ndarray, obs: CollisionObject):
        min_dist, closest_pt, normal = float("inf"), Point(), Point()

        for i, prim in enumerate(obs.primitives):
            pos = obs.primitive_poses[i].position
            center = np.array([pos.x, pos.y, pos.z])

            if prim.type == prim.BOX:
                d, pt, n = self._dist_box(point, center, prim.dimensions)
            elif prim.type == prim.SPHERE:
                d, pt, n = self._dist_sphere(point, center, prim.dimensions[0])
            else:
                d = np.linalg.norm(point - center)
                pt = Point(x=center[0], y=center[1], z=center[2])
                n = Point(x=1.0, y=0.0, z=0.0)

            if d < min_dist:
                min_dist, closest_pt, normal = d, pt, n

        return min_dist, closest_pt, normal

    def _dist_box(self, pt, center, dims):
        half = np.array(dims) / 2.0
        rel = pt - center
        closest = center + np.clip(rel, -half, half)
        dist = np.linalg.norm(pt - closest)

        if dist > 1e-6:
            norm_vec = (pt - closest) / dist
        else:
            axis = np.argmin(half - np.abs(rel))
            norm_vec = np.zeros(3)
            norm_vec[axis] = np.sign(rel[axis]) if np.sign(rel[axis]) != 0 else 1.0

        return dist, Point(x=closest[0], y=closest[1], z=closest[2]), Point(
            x=norm_vec[0], y=norm_vec[1], z=norm_vec[2]
        )

    def _dist_sphere(self, pt, center, radius):
        vec = pt - center
        dist_center = np.linalg.norm(vec)

        if dist_center < 1e-6:
            return radius, Point(x=center[0] + radius, y=center[1], z=center[2]), Point(
                x=1.0, y=0.0, z=0.0
            )

        direction = vec / dist_center
        closest = center + direction * radius
        return abs(dist_center - radius), Point(x=closest[0], y=closest[1], z=closest[2]), Point(
            x=direction[0], y=direction[1], z=direction[2]
        )

    # ================================================================
    # VELOCITÀ REPULSIVE
    # ================================================================

    def compute_repulsive_velocity(self, distances: Dict, q: np.ndarray) -> np.ndarray:
        q_dot = np.zeros(self.nv)

        for link, (dist, _, normal) in distances.items():
            if dist > self.influence_distance:
                continue

            d = max(dist, self.safety_margin)
            intensity = self.repulsive_gain * ((1.0 / d) - (1.0 / self.influence_distance)) ** 2

            norm_vec = np.array([normal.x, normal.y, normal.z], dtype=float)
            norm_len = np.linalg.norm(norm_vec)
            if norm_len < 1e-6:
                continue

            v_rep = intensity * norm_vec / norm_len

            # Twist cartesiano target (solo linear)
            x_dot = np.zeros(6)
            x_dot[:3] = v_rep

            J_link = self.get_link_jacobian(link, q)
            if J_link is not None:
                q_dot += J_link.T @ x_dot  # proiezione cartesiano->giunti

        return q_dot

    def compute_damped_pseudoinverse(self, J: np.ndarray) -> np.ndarray:
        # J: 6 x nv
        try:
            _, sigma, _ = np.linalg.svd(J, full_matrices=False)
            sigma_min = np.min(sigma) if sigma.size > 0 else 0.0
            lam = self.damping_max * np.exp(-sigma_min / self.singularity_threshold) if sigma_min < self.singularity_threshold else 1e-6
            return J.T @ np.linalg.inv(J @ J.T + (lam ** 2) * np.eye(6))
        except Exception:
            return np.linalg.pinv(J)

    def saturate_velocity(self, q_dot: np.ndarray) -> np.ndarray:
        return np.clip(q_dot, -self.max_joint_velocity, self.max_joint_velocity)

    def publish_velocity_command(self, q_dot: np.ndarray):
        msg = Float64MultiArray()
        msg.data = q_dot.tolist()
        self.velocity_cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = OnlineAvoidanceController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
