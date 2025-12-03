#!/usr/bin/env python3
"""
VELOCITY CONTROL BLENDER - VERSIONE SEMPLICE (con vincolo su ḋ)
================================================================

Segue la traiettoria punto per punto e combina il tracking con
l'avoidance usando una proiezione in spazio di giunto che garantisce:

    d_dot = j_row @ qdot >= d_dot_min(d)

Questo evita:
 - blocchi sull'ostacolo
 - collassi sull'ostacolo
 - spinte eccessive e rientri bruschi
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory


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
        self.declare_parameter("kp", 20.0)             # MODIFICA: un po' meno aggressivo (prima 30)
        self.declare_parameter("max_vel", 0.4)         # MODIFICA: limita velocità globale
        self.declare_parameter("waypoint_threshold", 0.05)
        self.declare_parameter("final_threshold", 0.01)

        self.declare_parameter("influence_distance", 0.30)
        self.declare_parameter("safety_margin", 0.08)

        # MODIFICA: nuovi parametri per blending e proiezione
        self.declare_parameter("avoidance_weight_max", 1.0)      # peso max su qdot_avoid
        self.declare_parameter("slowdown_factor_max", 0.5)       # riduzione max velocità vicini (0.5 -> velocità min 50%)
        self.declare_parameter("d_dot_min_close", 0.02)          # ḋ minima quando d <= d_safe (m/s equivalente)

        self.kp = self.get_parameter("kp").value
        self.max_vel = self.get_parameter("max_vel").value
        self.waypoint_threshold = self.get_parameter("waypoint_threshold").value
        self.final_threshold = self.get_parameter("final_threshold").value

        self.d_infl = self.get_parameter("influence_distance").value
        self.d_safe = self.get_parameter("safety_margin").value

        self.avoidance_weight_max = self.get_parameter("avoidance_weight_max").value
        self.slowdown_factor_max = self.get_parameter("slowdown_factor_max").value
        self.d_dot_min_close = self.get_parameter("d_dot_min_close").value

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

        self.get_logger().info("✅ Simple Velocity Blender (ḋ-constrained) started")
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

        # ===== 3) Info per safety =====
        j_row = self.J_avoid[0, :]
        j_norm = np.linalg.norm(j_row)
        d = float(self.min_dist)

        # Se nessuna informazione sensata di avoidance → tracking puro
        if (d >= self.d_infl) or (avoid_norm < 1e-6) or (j_norm < 1e-6):
            qdot_des = qdot_tracking
        else:
            # --------------------------------------------------------------
            # ZONA DI INFLUENZA:
            #   - decomposizione tracking in normale + tangenziale
            #   - blending morbido tra tracking e repulsione
            #   - vincolo su ḋ SOLO molto vicino all'ostacolo
            # --------------------------------------------------------------

            d_safe = self.d_safe
            d_infl = self.d_infl

            # 1) Proiettore sul normale e sul null space
            #    J: (1,7), J^+: (7,1), P = J^+ J, N = I - P
            J = j_row.reshape(1, self.n_dof)
            JT = J.T
            denom = float(J @ JT) + 1e-8          # JJ^T = ||j_row||^2 (scalare)
            P = (JT @ J) / denom                  # proiettore sulla direzione di j_row
            N = np.eye(self.n_dof) - P            # proiettore nel null space di j_row

            # 2) Decomposizione del tracking:
            #    qdot_tracking = qdot_n (normale) + qdot_tan (tangenziale)
            qdot_n   = P @ qdot_tracking          # componente che cambia d
            qdot_tan = N @ qdot_tracking          # componente che non cambia d al primo ordine

            # 3) Peso basato sulla distanza (smoothstep da d_infl -> d_safe)
            x = (d_infl - d) / (d_infl - d_safe)  # 0 al bordo esterno, 1 vicino a d_safe
            x = max(0.0, min(1.0, x))
            w_d = 3.0 * x * x - 2.0 * x * x * x   # smoothstep ∈ [0,1]

            # 4) Pesi per normal tracking e repulsione:
            #    - lontano: w_d ≈ 0  → w_n ≈ 1, w_rep ≈ 0
            #    - vicino : w_d ≈ 1  → w_n ≈ 0, w_rep ≈ max
            w_n   = 1.0 - w_d
            w_rep = self.avoidance_weight_max * w_d

            # >>> ENFASI SUL NULL SPACE <<<
            # Tracking "utile" = tutto quello che sopravvive nel null space (qdot_tan)
            # + eventualmente una piccola componente normale se non siamo proprio attaccati.
            NULL_BOOST = 2.0      # prova con 2.0 o 3.0
            qdot_ns = NULL_BOOST * qdot_tan + w_n * qdot_n


            # 5) Combinazione preliminare:
            #    - qdot_ns: sempre presente → il robot cerca comunque di progredire
            #      nel null space della distanza
            #    - qdot_avoid: componente puramente di repulsione
            qdot_des = qdot_ns + w_rep * qdot_avoid


            # 6) Rallentamento globale vicino all'ostacolo
            gamma = 1.0 - self.slowdown_factor_max * w_d
            gamma = max(0.5, gamma)
            # non rallentare oltre il 20%
            qdot_des *= gamma

            # 7) Vincolo su ḋ = j_row @ qdot SOLO quando siamo molto vicini
            #    - se d > d_safe: permettiamo anche un piccolo avvicinamento
            #    - se d <= d_safe: imponiamo ḋ >= ḋ_min_close (positivo)
            if d <= d_safe:
                # 1) ḋ attuale
                d_dot = float(j_row @ qdot_des)
                d_dot_min = self.d_dot_min_close   # ad es. 0.02 m/s equivalente

                # 2) Salva la componente tangenziale PRIMA della correzione normale
                qdot_tan_only = N @ qdot_des

                # 3) Se serve, calcola la correzione lungo la normale
                if d_dot < d_dot_min:
                    lambda_corr = (d_dot_min - d_dot) / (j_norm * j_norm + 1e-8)
                    qdot_des = qdot_des + lambda_corr * j_row

                # 4) Reintroduci la componente tangenziale (non viene toccata dal vincolo)
                qdot_des = qdot_des + qdot_tan_only


        # ------------------------------------------------------------------
        # FILTRO SULLA VELOCITÀ + SATURAZIONE
        # ------------------------------------------------------------------
        beta = 0.7  # 0.7 = più reattivo, 0.5 = più liscio
        qdot = beta * qdot_des + (1.0 - beta) * self.qdot_prev
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
