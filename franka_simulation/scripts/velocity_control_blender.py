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

Integrazione con obstacle avoidance:
- Riceve qdot_avoid dai campi potenziali
- Riceve la riga di Jacobiano del punto più critico (J_avoid)
- Riceve la distanza minima dall'ostacolo (min_dist)
- Applica null-space control + blending basato su min_dist
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
        self.declare_parameter("kp", 30.0)
        self.declare_parameter("max_vel", 0.5)
        self.declare_parameter("waypoint_threshold", 0.05)
        self.declare_parameter("final_threshold", 0.01)

        self.declare_parameter("influence_distance", 0.30)
        self.declare_parameter("safety_margin", 0.08)

        self.kp = self.get_parameter("kp").value
        self.max_vel = self.get_parameter("max_vel").value
        self.waypoint_threshold = self.get_parameter("waypoint_threshold").value
        self.final_threshold = self.get_parameter("final_threshold").value

        self.d_infl = self.get_parameter("influence_distance").value
        self.d_safe = self.get_parameter("safety_margin").value

        # ===== STATO =====
        self.q = np.zeros(self.n_dof)              # Posizione corrente
        self.qdot_avoid = np.zeros(self.n_dof)     # Velocità di avoidance
        self.qdot_prev = np.zeros(self.n_dof)      # Velocità precedente (per smoothing)
        self.J_avoid = np.zeros((1, self.n_dof))   # Jacobiano del punto più critico (1x7)
        self.min_dist = 999.0                      # Distanza minima iniziale "lontana"

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
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE
            )
        )

        self.create_subscription(
            Float64MultiArray, "/avoidance/velocity",
            self.avoidance_cb, 10
        )

        self.create_subscription(
            Float64MultiArray, "/avoidance/jacobian",
            self.avoidance_jac_cb, 10
        )

        self.create_subscription(
            Float64MultiArray, "/avoidance/min_distance",
            self.min_dist_cb, 10
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
        self.get_logger().info(
            f"   Kp={self.kp}, max_vel={self.max_vel}, d_safe={self.d_safe}, d_infl={self.d_infl}"
        )

    # ======================================================================
    # CALLBACKS
    # ======================================================================

    def joint_state_cb(self, msg: JointState):
        """Legge la posizione corrente dei giunti."""
        for i, name in enumerate(self.joint_names):
            if name in msg.name:
                idx = msg.name.index(name)
                self.q[i] = msg.position[idx]

    def trajectory_cb(self, msg: JointTrajectory):
        """Riceve una nuova traiettoria da MoveIt."""
        if not msg.points:
            return

        # Mappa nomi giunti -> indici
        index_map = {}
        for i, name in enumerate(self.joint_names):
            if name in msg.joint_names:
                index_map[i] = msg.joint_names.index(name)

        if len(index_map) != self.n_dof:
            self.get_logger().error("Joint names mismatch in trajectory_cb!")
            return

        # Estrai tutti i punti della traiettoria
        self.trajectory_points = []
        for point in msg.points:
            q_target = np.array([point.positions[index_map[i]] for i in range(self.n_dof)])
            self.trajectory_points.append(q_target)

        self.current_index = 0
        self.active = True

        # Reset filtro smoothing
        self.qdot_prev = np.zeros(self.n_dof)

        self.get_logger().info(f"📈 Nuova traiettoria: {len(self.trajectory_points)} punti")

    def avoidance_cb(self, msg: Float64MultiArray):
        """Riceve velocità di avoidance (7D)."""
        if len(msg.data) == self.n_dof:
            self.qdot_avoid = np.array(msg.data)

    def avoidance_jac_cb(self, msg: Float64MultiArray):
        """Riceve la riga di Jacobiano del punto più critico (1x7)."""
        if len(msg.data) == self.n_dof:
            self.J_avoid[0, :] = np.array(msg.data)
        else:
            self.J_avoid[0, :] = np.zeros(self.n_dof)

    def min_dist_cb(self, msg: Float64MultiArray):
        """Riceve la distanza minima dall'ostacolo."""
        if len(msg.data) > 0:
            self.min_dist = float(msg.data[0])

    # ======================================================================
    # CONTROL LOOP
    # ======================================================================

    def control_loop(self):
        """Loop di controllo principale."""

        # Se non c'è traiettoria attiva, pubblica zero
        if not self.active or len(self.trajectory_points) == 0:
            self.publish_velocity(np.zeros(self.n_dof))
            return

        # Punto target corrente
        q_target = self.trajectory_points[self.current_index]

        # Errore di tracking in joint space
        error = q_target - self.q
        error_norm = np.linalg.norm(error)

        # Siamo all'ultimo punto?
        is_last = (self.current_index == len(self.trajectory_points) - 1)

        # Soglia da usare
        threshold = self.final_threshold if is_last else self.waypoint_threshold

        # Gestione del passaggio waypoint successivo
        if error_norm < threshold:
            if is_last:
                # Traiettoria completata!
                self.get_logger().info(
                    f"✅ Traiettoria completata! Errore finale: {error_norm:.4f} rad"
                )
                self.active = False
                self.publish_velocity(np.zeros(self.n_dof))
                return
            else:
                # Passa al punto successivo
                self.current_index += 1
                self.get_logger().info(
                    f"📍 Punto {self.current_index}/{len(self.trajectory_points) - 1}"
                )
                q_target = self.trajectory_points[self.current_index]
                error = q_target - self.q
                error_norm = np.linalg.norm(error)

        # ===== 1) Tracking "puro" (senza avoidance) =====
        qdot_tracking = self.kp * error

        # ===== 2) Velocità di evitamento =====
        qdot_avoid = self.qdot_avoid.copy()
        avoid_norm = np.linalg.norm(qdot_avoid)

        # ===== 3) Info per null-space e safety =====
        j_row = self.J_avoid[0, :]
        j_norm = np.linalg.norm(j_row)
        d = float(self.min_dist)

        # ------------------------------------------------------------------
        # 3.a SAFETY HARD: troppo vicino all'ostacolo → SOLO avoidance
        # ------------------------------------------------------------------
        if d <= self.d_safe:
            # Qui vogliamo che il robot si allontani dall'ostacolo.
            # Usiamo solo qdot_avoid, senza tracking.
            qdot = qdot_avoid

            # Filtro e saturazione
            beta = 0.7
            qdot = beta * qdot + (1.0 - beta) * self.qdot_prev
            self.qdot_prev = qdot.copy()
            qdot = np.clip(qdot, -self.max_vel, self.max_vel)

            # Pubblica e termina il ciclo
            self.publish_velocity(qdot)
            return

        # ------------------------------------------------------------------
        # 3.b ZONA LIBERA / NESSUNA AVOIDANCE: tracking puro
        # ------------------------------------------------------------------
        # - lontano dagli ostacoli (d >= d_infl)
        # - oppure nessuna forza di avoidance
        # - oppure Jacobiano malcondizionato
        if (d >= self.d_infl) or (avoid_norm < 1e-6) or (j_norm < 1e-6):
            qdot = qdot_tracking

            # Filtro e saturazione
            beta = 0.7
            qdot = beta * qdot + (1.0 - beta) * self.qdot_prev
            self.qdot_prev = qdot.copy()
            qdot = np.clip(qdot, -self.max_vel, self.max_vel)

            self.publish_velocity(qdot)
            return

        # ------------------------------------------------------------------
        # 3.c ZONA DI INFLUENZA: null-space control + blending
        # ------------------------------------------------------------------

        # Proiettore nel null space del vincolo di distanza
        # J: (1,7), J^T: (7,1)
        J = j_row.reshape(1, self.n_dof)
        JT = J.T
        denom = float(J @ JT) + 1e-6  # scalare
        P = (JT @ J) / denom          # (7,7)
        N = np.eye(self.n_dof) - P    # (7,7) proiettore nel null space

        # Scaling basato sulla distanza (d_safe < d < d_infl)
        d_safe = self.d_safe
        d_infl = self.d_infl

        # x in [0,1]: 0 = bordo esterno, 1 = bordo interno
        x = (d_infl - d) / (d_infl - d_safe)
        x = max(0.0, min(1.0, x))

        # Smoothstep per alpha
        alpha = 3.0 * x * x - 2.0 * x * x * x

        # Deadband per ridurre le oscillazioni ai bordi
        if x < 0.05:
            alpha = 0.0
        elif x > 0.95:
            alpha = 1.0

        # alpha:
        #  - 0.0 → solo tracking nel null-space
        #  - 1.0 → solo avoidance
        qdot = alpha * qdot_avoid + (1.0 - alpha) * (N @ qdot_tracking)

        # ------------------------------------------------------------------
        # 4) FILTRO SULLA VELOCITÀ + SATURAZIONE
        # ------------------------------------------------------------------
        beta = 0.7  # 0.7 = più reattivo, 0.5 = più liscio
        qdot = beta * qdot + (1.0 - beta) * self.qdot_prev
        self.qdot_prev = qdot.copy()

        # Saturazione delle velocità
        qdot = np.clip(qdot, -self.max_vel, self.max_vel)

        # Pubblica comando finale
        self.publish_velocity(qdot)

    # ======================================================================
    # UTILITIES
    # ======================================================================

    def publish_velocity(self, qdot: np.ndarray):
        """Pubblica comando di velocità sui giunti."""
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
