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
import time
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
        # ḋ minima quando d <= d_safe (m/s equivalente). 0.0 = evita penetrazione ma non forza "scappare".
        self.declare_parameter("d_dot_min_close", 0.0)
        # ḋ minima al bordo della zona di influenza (di solito negativa: permette avvicinarsi se lontano)
        self.declare_parameter("d_dot_min_far", -0.05)

        # Bias tangenziale per "aggirare" senza bloccare il goal
        self.declare_parameter("avoidance_tangent_weight", 0.4)
        # Limite alla correzione lungo la normale (evita scatti)
        self.declare_parameter("normal_correction_max", 0.25)
        # Strategie per evitare lo "stallo" in presenza di ostacoli
        self.declare_parameter("use_avoidance_velocity", True)        # usa /avoidance/velocity nel blending
        self.declare_parameter("avoidance_normal_only", True)         # applica repulsione solo nella normale (rispetto a j_row)
        self.declare_parameter("null_boost_max", 3.0)                 # boost progress nel nullspace vicino all'ostacolo
        self.declare_parameter("avoidance_ratio_max", 1.2)            # limite: ||w_rep*qdot_avoid|| <= ratio*(||qdot_ns||+eps)
        # Modalità B (reactive): l'avoidance può muovere anche senza traiettoria
        self.declare_parameter("reactive_enable", True)
        self.declare_parameter("reactive_deadband", 1e-3)   # sotto questa norma → fermo
        # Sicurezza/UX: per default non muovere il robot finché non arriva una traiettoria
        # (evita drift/spinte iniziali dovute a avoidance o rumore)
        self.declare_parameter("hold_position_without_trajectory", True)
        self.kp = self.get_parameter("kp").value
        self.max_vel = self.get_parameter("max_vel").value
        self.waypoint_threshold = self.get_parameter("waypoint_threshold").value
        self.final_threshold = self.get_parameter("final_threshold").value

        self.d_infl = self.get_parameter("influence_distance").value
        self.d_safe = self.get_parameter("safety_margin").value

        self.avoidance_weight_max = self.get_parameter("avoidance_weight_max").value
        self.slowdown_factor_max = self.get_parameter("slowdown_factor_max").value
        self.d_dot_min_close = self.get_parameter("d_dot_min_close").value
        self.d_dot_min_far = float(self.get_parameter("d_dot_min_far").value)

        self.avoidance_tangent_weight = float(self.get_parameter("avoidance_tangent_weight").value)
        self.normal_correction_max = float(self.get_parameter("normal_correction_max").value)

        self.use_avoidance_velocity = bool(self.get_parameter("use_avoidance_velocity").value)
        self.avoidance_normal_only = bool(self.get_parameter("avoidance_normal_only").value)
        self.null_boost_max = float(self.get_parameter("null_boost_max").value)
        self.avoidance_ratio_max = float(self.get_parameter("avoidance_ratio_max").value)

        self.reactive_enable = bool(self.get_parameter("reactive_enable").value)
        self.reactive_deadband = float(self.get_parameter("reactive_deadband").value)
        self.hold_position_without_trajectory = bool(
            self.get_parameter("hold_position_without_trajectory").value
        )

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
        self.get_logger().info(
            f"   use_avoidance_velocity={self.use_avoidance_velocity}, avoidance_normal_only={self.avoidance_normal_only}, "
            f"null_boost_max={self.null_boost_max}, avoidance_ratio_max={self.avoidance_ratio_max}"
        )
        self.get_logger().info(
            f"   d_dot_min_far={self.d_dot_min_far}, d_dot_min_close={self.d_dot_min_close}, "
            f"avoidance_tangent_weight={self.avoidance_tangent_weight}, normal_correction_max={self.normal_correction_max}"
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
        # In alcune pipeline (o per bug/bridge), JointTrajectory può arrivare con joint_names vuoti.
        # In quel caso, se positions è di dimensione 7, assumiamo l'ordine canonico.
        index_map = {}
        if msg.joint_names and len(msg.joint_names) > 0:
            for i, name in enumerate(self.joint_names):
                if name in msg.joint_names:
                    index_map[i] = msg.joint_names.index(name)

        use_direct_positions = False
        if len(index_map) != self.n_dof:
            # Fallback: assume canonical order if positions look correct
            ok_shape = all((hasattr(p, "positions") and len(p.positions) == self.n_dof) for p in msg.points)
            if ok_shape:
                use_direct_positions = True
                self.get_logger().warn(
                    "JointTrajectory joint_names missing/mismatched; assuming canonical FR3 joint order."
                )
            else:
                self.get_logger().error(
                    f"Joint names mismatch in trajectory_cb! joint_names={list(msg.joint_names)}"
                )
                return

        # Estrai tutti i punti della traiettoria (prima in una lista locale)
        new_points = []
        for point in msg.points:
            if use_direct_positions:
                q_target = np.array(point.positions[: self.n_dof], dtype=float)
            else:
                q_target = np.array([point.positions[index_map[i]] for i in range(self.n_dof)], dtype=float)
            new_points.append(q_target)

        # Guard: ignore degenerate trajectories (all points ~ identical).
        # These can appear in some edge cases and would otherwise overwrite a useful trajectory.
        try:
            if len(new_points) >= 2:
                span = float(np.linalg.norm(new_points[-1] - new_points[0]))
                if span < 1e-3:
                    self.get_logger().warn(
                        f"Ignoring degenerate trajectory (span≈{span:.2e} rad, points={len(new_points)})."
                    )
                    return
        except Exception:
            pass

        self.trajectory_points = new_points

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

        # ===== MODALITÀ REACTIVE (B) =====
        # Se non c'è traiettoria, usa direttamente qdot_avoid come comando di velocità.
        if (not self.active) or (len(self.trajectory_points) == 0):
            # Default: mantieni fermo finché non arriva una traiettoria
            if self.hold_position_without_trajectory:
                self.publish_velocity(np.zeros(self.n_dof))
                return

            if self.reactive_enable:
                qdot = self.qdot_avoid.copy()

                # deadband per evitare drift dovuto a rumore numerico
                if np.linalg.norm(qdot) < self.reactive_deadband:
                    qdot = np.zeros(self.n_dof)

                # saturazione
                qdot = np.clip(qdot, -self.max_vel, self.max_vel)

                self.publish_velocity(qdot)
            else:
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
        if (d >= self.d_infl) or (j_norm < 1e-6):
            qdot_des = qdot_tracking
        else:
            # --------------------------------------------------------------
            # ZONA DI INFLUENZA (smooth, goal-driven):
            #   - mantieni tracking verso goal
            #   - aggiungi un bias tangenziale (per aggirare)
            #   - applica un vincolo su ḋ in tutta la zona di influenza
            #     con correzione MINIMA lungo j_row (QP 1D)
            # --------------------------------------------------------------
            d_safe = self.d_safe
            d_infl = self.d_infl

            # Projectors for the distance normal and its nullspace
            J = j_row.reshape(1, self.n_dof)
            JT = J.T
            denom = float(J @ JT) + 1e-8
            P = (JT @ J) / denom
            N = np.eye(self.n_dof) - P

            # Smoothstep weight: 0 at d_infl, 1 at d_safe
            x = (d_infl - d) / (d_infl - d_safe)
            x = max(0.0, min(1.0, x))
            w_d = 3.0 * x * x - 2.0 * x * x * x

            # 1) Base: keep tracking (towards goal)
            qdot_des = qdot_tracking.copy()

            # 2) Add tangential avoidance bias (helps going around instead of pushing straight back)
            if self.use_avoidance_velocity and (avoid_norm > 1e-6):
                qdot_avoid_tan = N @ qdot_avoid
                qdot_des = qdot_des + (self.avoidance_tangent_weight * w_d) * qdot_avoid_tan

            # 3) Optional: increase tangential progress near obstacle
            null_boost = 1.0 + self.null_boost_max * w_d
            qdot_des = (P @ qdot_des) + null_boost * (N @ qdot_des)

            # 4) Enforce a smooth lower bound on d_dot across the influence region
            #    - far (d≈d_infl): allow some approach (negative)
            #    - close (d≈d_safe): require non-decreasing distance (>= d_dot_min_close, default 0)
            d_dot_min = (1.0 - w_d) * self.d_dot_min_far + w_d * self.d_dot_min_close

            d_dot = float(j_row @ qdot_des)
            if d_dot < d_dot_min:
                lambda_corr = (d_dot_min - d_dot) / (j_norm * j_norm + 1e-8)
                corr = lambda_corr * j_row
                # Cap the correction magnitude for smoothness
                corr_norm = float(np.linalg.norm(corr))
                if corr_norm > self.normal_correction_max:
                    corr *= self.normal_correction_max / (corr_norm + 1e-9)
                qdot_des = qdot_des + corr

            # 5) Global slowdown (gentle) close to obstacles
            gamma = 1.0 - self.slowdown_factor_max * w_d
            gamma = max(0.6, gamma)
            qdot_des *= gamma


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
