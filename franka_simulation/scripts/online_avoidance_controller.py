#!/usr/bin/env python3
"""
Online Avoidance Controller - VERSIONE DEBUG v2
================================================

Modifiche:
1. Si attiva SOLO se distanza < 0.25m (molto vicino)
2. Pubblica SEMPRE zero se non in pericolo
3. Log della posizione HOME per verificare che sia corretta
4. Verifica che obstacle_box_lateral sia dove ci aspettiamo

"""

import os
import tempfile
from typing import Dict, Optional, List
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from moveit_msgs.msg import PlanningScene, CollisionObject
from geometry_msgs.msg import Point
from rcl_interfaces.srv import GetParameters

import pinocchio as pin
from pinocchio.utils import zero


class SimpleAvoidanceController(Node):
    """Controller SEMPLICE per debug."""

    def __init__(self):
        super().__init__("online_avoidance_controller")

        # Parametri MOLTO CONSERVATIVI per debug
        self.control_rate = 50.0  # Hz
        self.influence_distance = 0.25  # m - RIDOTTO! Solo quando molto vicino
        self.safety_margin = 0.03  # m
        self.repulsive_gain = 0.5  # RIDOTTO per movimenti più lenti
        self.max_joint_velocity = 0.2  # rad/s - molto lento per debug
        
        # Ostacoli da escludere
        self.excluded_names = ["ground", "plane", "floor"]

        # Stato
        self.joint_positions: Optional[np.ndarray] = None
        self.obstacles: List[CollisionObject] = []
        self.pin_initialized = False
        
        self.loop_count = 0
        self.logged_home = False

        # Inizializza Pinocchio
        self.get_logger().info("🤖 Inizializzazione Pinocchio...")
        if not self._init_pinocchio():
            self.get_logger().error("❌ Errore Pinocchio")
            return

        # Subscribers
        self.create_subscription(JointState, "/joint_states", self._joint_cb, 10)
        
        obstacle_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.create_subscription(PlanningScene, "/obstacle_scene", self._obstacle_cb, obstacle_qos)

        # Publisher
        self.vel_pub = self.create_publisher(Float64MultiArray, "/avoidance/velocity", 10)

        # Timer
        self.create_timer(1.0 / self.control_rate, self._control_loop)

        self.get_logger().info("=" * 60)
        self.get_logger().info("🚀 SIMPLE Avoidance Controller v2 READY")
        self.get_logger().info(f"   Influence: {self.influence_distance} m (SOLO molto vicino)")
        self.get_logger().info(f"   Gain: {self.repulsive_gain}")
        self.get_logger().info(f"   Max vel: {self.max_joint_velocity} rad/s")
        self.get_logger().info("=" * 60)

    def _init_pinocchio(self) -> bool:
        """Inizializza Pinocchio dal robot_description."""
        try:
            cli = self.create_client(GetParameters, "/robot_state_publisher/get_parameters")
            if not cli.wait_for_service(timeout_sec=10.0):
                return False
                
            req = GetParameters.Request()
            req.names = ["robot_description"]
            future = cli.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            
            if not future.result() or not future.result().values:
                return False
                
            urdf_str = future.result().values[0].string_value
            
            with tempfile.NamedTemporaryFile(delete=False, suffix=".urdf", mode="w") as f:
                f.write(urdf_str)
                urdf_path = f.name

            model_full = pin.buildModelFromUrdf(urdf_path)
            
            # Lock finger joints
            joints_to_lock = [model_full.getJointId(n) for n in model_full.names if "finger" in n]
            self.model = pin.buildReducedModel(model_full, joints_to_lock, pin.neutral(model_full))
            self.data = self.model.createData()
            
            # Frame ID per end-effector
            self.ee_frame_id = self.model.getFrameId("fr3_link8")
            
            os.unlink(urdf_path)
            
            self.pin_initialized = True
            self.get_logger().info(f"✅ Pinocchio OK: {self.model.nq} DOF")
            return True
            
        except Exception as e:
            self.get_logger().error(f"❌ Pinocchio error: {e}")
            import traceback
            traceback.print_exc()
            return False

    def _joint_cb(self, msg: JointState):
        """Callback joint states."""
        indices = [i for i, n in enumerate(msg.name) if "fr3_joint" in n and "finger" not in n]
        if len(indices) == 7:
            self.joint_positions = np.array([msg.position[i] for i in indices])

    def _obstacle_cb(self, msg: PlanningScene):
        """Callback ostacoli."""
        all_obs = msg.world.collision_objects
        self.obstacles = []
        
        for obs in all_obs:
            name_lower = obs.id.lower()
            is_excluded = any(excl in name_lower for excl in self.excluded_names)
            if not is_excluded:
                self.obstacles.append(obs)
        
        # Log ostacoli solo una volta
        if self.loop_count == 0:
            self.get_logger().info(f"📦 Obstacles loaded: {len(self.obstacles)}")
            for obs in self.obstacles:
                if obs.primitive_poses:
                    p = obs.primitive_poses[0].position
                    if obs.primitives:
                        dims = obs.primitives[0].dimensions
                        self.get_logger().info(
                            f"   - {obs.id}: center=({p.x:.2f}, {p.y:.2f}, {p.z:.2f}), "
                            f"dims=({dims[0]:.2f}, {dims[1]:.2f}, {dims[2]:.2f})"
                        )

    def _control_loop(self):
        """Loop di controllo."""
        self.loop_count += 1
        
        # Default: pubblica ZERO
        zero_vel = Float64MultiArray()
        zero_vel.data = [0.0] * 7
        
        if not self.pin_initialized or self.joint_positions is None:
            self.vel_pub.publish(zero_vel)
            return

        q = self.joint_positions
        
        # Forward kinematics
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        
        # Posizione end-effector
        ee_pose = self.data.oMf[self.ee_frame_id]
        ee_pos = np.array(ee_pose.translation).flatten()
        
        # Log posizione HOME solo una volta all'inizio
        if not self.logged_home:
            self.logged_home = True
            self.get_logger().info("=" * 60)
            self.get_logger().info(f"🏠 HOME POSITION:")
            self.get_logger().info(f"   EE: ({ee_pos[0]:.3f}, {ee_pos[1]:.3f}, {ee_pos[2]:.3f})")
            self.get_logger().info(f"   Joints: [{', '.join([f'{j:.2f}' for j in q])}]")
            self.get_logger().info("=" * 60)
        
        # Trova ostacolo più vicino
        min_dist = float('inf')
        repulsion_direction = np.zeros(3)
        closest_name = "none"
        
        for obs in self.obstacles:
            dist, direction = self._distance_to_obstacle(ee_pos, obs)
            if dist < min_dist:
                min_dist = dist
                repulsion_direction = direction
                closest_name = obs.id
        
        # Log periodico della distanza (ogni 2 sec)
        if self.loop_count % 100 == 0:
            self.get_logger().info(
                f"📏 EE: ({ee_pos[0]:.2f}, {ee_pos[1]:.2f}, {ee_pos[2]:.2f}) | "
                f"Closest: {closest_name} @ {min_dist:.3f}m | "
                f"Threshold: {self.influence_distance}m"
            )
        
        # SOLO se MOLTO vicino, attiva avoidance
        if min_dist < self.influence_distance and min_dist > 0.001:
            d = max(min_dist, self.safety_margin)
            
            # Intensità
            intensity = self.repulsive_gain * (1.0/d - 1.0/self.influence_distance)
            
            # Velocità cartesiana (LONTANO dall'ostacolo)
            v_cartesian = intensity * repulsion_direction
            
            # Jacobiano
            J = pin.computeFrameJacobian(
                self.model, self.data, q, self.ee_frame_id, 
                pin.ReferenceFrame.WORLD
            )
            J_pos = J[:3, :]
            
            # Joint velocity
            J_pinv = np.linalg.pinv(J_pos)
            q_dot = J_pinv @ v_cartesian
            
            # Saturazione
            q_dot = np.clip(q_dot, -self.max_joint_velocity, self.max_joint_velocity)
            
            # Log quando avoidance attivo
            if self.loop_count % 10 == 0:
                self.get_logger().warn(
                    f"⚠️  AVOIDANCE ACTIVE! dist={min_dist:.3f}m to {closest_name} | "
                    f"|q_dot|={np.linalg.norm(q_dot):.4f}"
                )
            
            # Pubblica velocità di avoidance
            msg = Float64MultiArray()
            msg.data = q_dot.tolist()
            self.vel_pub.publish(msg)
        else:
            # NESSUN avoidance - pubblica zero
            self.vel_pub.publish(zero_vel)

    def _distance_to_obstacle(self, point: np.ndarray, obs: CollisionObject) -> tuple:
        """Calcola distanza e direzione di repulsione."""
        min_dist = float('inf')
        best_direction = np.array([1.0, 0.0, 0.0])
        
        for i, prim in enumerate(obs.primitives):
            pose = obs.primitive_poses[i]
            center = np.array([pose.position.x, pose.position.y, pose.position.z])
            
            if prim.type == prim.BOX:
                half_size = np.array(prim.dimensions) / 2.0
                rel = point - center
                closest_on_box = center + np.clip(rel, -half_size, half_size)
                diff = point - closest_on_box
                dist = np.linalg.norm(diff)
                
                if dist > 1e-6:
                    direction = diff / dist
                else:
                    face_dists = half_size - np.abs(rel)
                    closest_face = np.argmin(face_dists)
                    direction = np.zeros(3)
                    direction[closest_face] = np.sign(rel[closest_face]) if rel[closest_face] != 0 else 1.0
                    dist = 0.001
                
            elif prim.type == prim.SPHERE:
                radius = prim.dimensions[0]
                diff = point - center
                dist_to_center = np.linalg.norm(diff)
                
                if dist_to_center > 1e-6:
                    direction = diff / dist_to_center
                    dist = max(0, dist_to_center - radius)
                else:
                    direction = np.array([1.0, 0.0, 0.0])
                    dist = 0.001
                    
            elif prim.type == prim.CYLINDER:
                height, radius = prim.dimensions[0], prim.dimensions[1]
                half_h = height / 2.0
                rel = point - center
                xy_dist = np.sqrt(rel[0]**2 + rel[1]**2)
                z_clamped = np.clip(rel[2], -half_h, half_h)
                
                if xy_dist > 1e-6:
                    xy_dir = np.array([rel[0], rel[1], 0]) / xy_dist
                else:
                    xy_dir = np.array([1.0, 0.0, 0.0])
                
                closest = center + np.array([0, 0, z_clamped])
                if xy_dist > radius:
                    closest += xy_dir * radius
                else:
                    closest += xy_dir * xy_dist
                
                diff = point - closest
                dist = np.linalg.norm(diff)
                
                if dist > 1e-6:
                    direction = diff / dist
                else:
                    direction = xy_dir
                    dist = 0.001
            else:
                diff = point - center
                dist = np.linalg.norm(diff)
                direction = diff / max(dist, 1e-6)
            
            if dist < min_dist:
                min_dist = dist
                best_direction = direction
        
        return min_dist, best_direction


def main(args=None):
    rclpy.init(args=args)
    
    print("=" * 60)
    print("🚀 SIMPLE Avoidance Controller v2")
    print("   - Influence distance: 0.25m (SOLO molto vicino)")
    print("   - Pubblica ZERO se non in pericolo")
    print("=" * 60)
    
    node = SimpleAvoidanceController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()