#!/usr/bin/env python3
"""
VELOCITY CONTROL BLENDER - VERSIONE SEMPLICE
=============================================

Segue la traiettoria punto per punto.
Non salta mai punti, anche se il robot è lento.

Logica:
1. Riceve traiettoria MoveIt
2. Punta al primo waypoint
3. Quando è vicino, passa al successivo
4. Ripete fino all'ultimo punto
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
import yaml

class SimpleVelocityBlender(Node):

    def __init__(self):
        super().__init__("velocity_control_blender")

        # Nomi giunti
        self.joint_names = [
            "fr3_joint1", "fr3_joint2", "fr3_joint3", "fr3_joint4",
            "fr3_joint5", "fr3_joint6", "fr3_joint7",
        ]
        self.n_dof = 7
        
              # ===== PARAMETRI (da ROS/YAML) =====
        self.declare_parameter('kp', 30.0)
        self.declare_parameter('max_vel', 0.5)
        self.declare_parameter('waypoint_threshold', 0.05)
        self.declare_parameter('final_threshold', 0.01)

        self.kp = self.get_parameter('kp').value
        self.max_vel = self.get_parameter('max_vel').value
        self.waypoint_threshold = self.get_parameter('waypoint_threshold').value
        self.final_threshold = self.get_parameter('final_threshold').value

        
        # ===== STATO =====
        self.q = np.zeros(self.n_dof)              # Posizione corrente
        self.qdot_avoid = np.zeros(self.n_dof)     # Velocità avoidance
        
        # Traiettoria
        self.trajectory_points = []      # Lista di configurazioni target
        self.current_index = 0           # Indice del punto corrente
        self.active = False              # Traiettoria attiva?
        
        # ===== SUBSCRIBERS =====
        self.create_subscription(
            JointState, "/joint_states",
            self.joint_state_cb, 10
        )
        
        self.create_subscription(
            JointTrajectory, "/velocity_blender/trajectory",
            self.trajectory_cb, 
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)
        )
        
        self.create_subscription(
            Float64MultiArray, "/avoidance/velocity",
            self.avoidance_cb, 10
        )
        
        # ===== PUBLISHER =====
        self.cmd_pub = self.create_publisher(
            Float64MultiArray,
            "/fr3_velocity_controller/commands",
            10
        )
        
        # ===== TIMER =====
        self.create_timer(0.01, self.control_loop)  # 100 Hz
        
        self.get_logger().info("✅ Simple Velocity Blender started")
        self.get_logger().info(f"   Kp={self.kp}, max_vel={self.max_vel}")

    def joint_state_cb(self, msg: JointState):
        """Legge la posizione corrente."""
        for i, name in enumerate(self.joint_names):
            if name in msg.name:
                idx = msg.name.index(name)
                self.q[i] = msg.position[idx]

    def trajectory_cb(self, msg: JointTrajectory):
        """Riceve una nuova traiettoria."""
        if not msg.points:
            return
        
        # Crea mapping nomi -> indici
        index_map = {}
        for i, name in enumerate(self.joint_names):
            if name in msg.joint_names:
                index_map[i] = msg.joint_names.index(name)
        
        if len(index_map) != self.n_dof:
            self.get_logger().error("Joint names mismatch!")
            return
        
        # Estrai tutti i punti della traiettoria
        self.trajectory_points = []
        for point in msg.points:
            q_target = np.array([point.positions[index_map[i]] for i in range(self.n_dof)])
            self.trajectory_points.append(q_target)
        
        # Inizia dal primo punto
        self.current_index = 0
        self.active = True
        
        self.get_logger().info(f"📈 Nuova traiettoria: {len(self.trajectory_points)} punti")

    def avoidance_cb(self, msg: Float64MultiArray):
        """Riceve velocità di avoidance."""
        if len(msg.data) == self.n_dof:
            self.qdot_avoid = np.array(msg.data)

    def control_loop(self):
        """Loop di controllo principale."""
        
        # Se non c'è traiettoria attiva, pubblica zero
        if not self.active or len(self.trajectory_points) == 0:
            self.publish_velocity(np.zeros(self.n_dof))
            return
        
        # Prendi il punto target corrente
        q_target = self.trajectory_points[self.current_index]
        
        # Calcola errore
        error = q_target - self.q
        error_norm = np.linalg.norm(error)
        
        # Siamo all'ultimo punto?
        is_last = (self.current_index == len(self.trajectory_points) - 1)
        
        # Soglia da usare
        threshold = self.final_threshold if is_last else self.waypoint_threshold
        
        # Se siamo vicini al punto corrente
        if error_norm < threshold:
            if is_last:
                # Traiettoria completata!
                self.get_logger().info(f"✅ Traiettoria completata! Errore finale: {error_norm:.4f} rad")
                self.active = False
                self.publish_velocity(np.zeros(self.n_dof))
                return
            else:
                # Passa al punto successivo
                self.current_index += 1
                self.get_logger().info(f"📍 Punto {self.current_index}/{len(self.trajectory_points)-1}")
                q_target = self.trajectory_points[self.current_index]
                error = q_target - self.q
        
        # Calcola velocità: semplice controllo P
        qdot = self.kp * error
        
        # Aggiungi avoidance
        qdot = qdot + self.qdot_avoid
        
        # Limita velocità
        qdot = np.clip(qdot, -self.max_vel, self.max_vel)
        
        # Pubblica
        self.publish_velocity(qdot)

    def publish_velocity(self, qdot: np.ndarray):
        """Pubblica comando di velocità."""
        msg = Float64MultiArray()
        msg.data = qdot.tolist()
        self.cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SimpleVelocityBlender()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Ferma il robot
        msg = Float64MultiArray()
        msg.data = [0.0] * 7
        node.cmd_pub.publish(msg)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()