#!/usr/bin/env python3
"""
VELOCITY CONTROL BLENDER - VERSIONE ROBUSTA
============================================

Questa versione risolve i problemi di tracking con:

1. SETTLING PHASE: Continua a correggere dopo la fine della traiettoria
2. CONTROLLO PD: Aggiunge termine derivativo per smorzamento
3. MONITORING: Log dettagliati per debug
4. TIMEOUT INTELLIGENTE: Non si blocca mai

Il flusso è:
1. Riceve traiettoria da /velocity_blender/trajectory
2. Interpola la traiettoria nel tempo
3. Calcola qdot_track = Kp*(q_d - q) + Kd*(qdot_d - qdot)
4. Aggiunge qdot_avoid da /avoidance/velocity
5. Pubblica comando finale su /fr3_velocity_controller/commands
6. NUOVO: Dopo fine traiettoria, continua fino a raggiungere il target

Author: Versione robusta per tracking preciso
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory


class RobustVelocityBlender(Node):
    """Velocity blender robusto con settling phase."""

    def __init__(self):
        super().__init__("velocity_control_blender")

        # ===== PARAMETERS =====
        self.declare_parameter("joint_names", [
            "fr3_joint1", "fr3_joint2", "fr3_joint3", "fr3_joint4",
            "fr3_joint5", "fr3_joint6", "fr3_joint7",
        ])
        
        # Tracking gains
        self.declare_parameter("kp_tracking", 20.0)      # Proporzionale
        self.declare_parameter("kd_tracking", 10.0)      # Derivativo (smorzamento)
        
        # Velocity limits
        self.declare_parameter("max_joint_vel", 0.8)    # rad/s
        self.declare_parameter("max_acceleration", 2.0) # rad/s²
        
        # Settling phase (NUOVO)
        self.declare_parameter("settling_enabled", True)
        self.declare_parameter("settling_kp", 3.0)           # Gain più alto per settling
        self.declare_parameter("settling_error_threshold", 0.005)  # 0.005 rad ≈ 0.3°
        self.declare_parameter("settling_timeout", 15.0)     # secondi
        
        # Control rate
        self.declare_parameter("control_rate", 100.0)   # Hz
        
        # Load parameters
        self.joint_names = self.get_parameter("joint_names").value
        self.kp = self.get_parameter("kp_tracking").value
        self.kd = self.get_parameter("kd_tracking").value
        self.max_joint_vel = self.get_parameter("max_joint_vel").value
        self.max_acceleration = self.get_parameter("max_acceleration").value
        
        self.settling_enabled = self.get_parameter("settling_enabled").value
        self.settling_kp = self.get_parameter("settling_kp").value
        self.settling_error_threshold = self.get_parameter("settling_error_threshold").value
        self.settling_timeout = self.get_parameter("settling_timeout").value
        
        control_rate = self.get_parameter("control_rate").value
        
        self.n_dof = len(self.joint_names)
        
        # ===== STATE =====
        self.q = np.zeros(self.n_dof)           # Posizione corrente
        self.qdot = np.zeros(self.n_dof)        # Velocità corrente (misurata)
        self.qdot_avoid = np.zeros(self.n_dof)  # Velocità avoidance
        
        # Trajectory state
        self.traj = None
        self.traj_start_time = None
        self.traj_index_map = None
        self.q_target = None  # Target finale della traiettoria
        
        # Settling state
        self.is_settling = False
        self.settling_start_time = None
        
        # Smoothing state
        self.previous_qdot_cmd = np.zeros(self.n_dof)
        self.previous_time = None
        
        # Logging
        self.log_counter = 0
        self.last_error_norm = 0.0
        
        # ===== SUBSCRIBERS =====
        qos_reliable = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )
        
        self.create_subscription(
            JointState, "/joint_states",
            self.joint_state_callback, 10
        )
        
        self.create_subscription(
            JointTrajectory, "/velocity_blender/trajectory",
            self.trajectory_callback, qos_reliable
        )
        
        self.create_subscription(
            Float64MultiArray, "/avoidance/velocity",
            self.avoidance_callback, 10
        )
        
        # ===== PUBLISHER =====
        self.cmd_pub = self.create_publisher(
            Float64MultiArray,
            "/fr3_velocity_controller/commands",
            10
        )
        
        # ===== CONTROL TIMER =====
        period = 1.0 / control_rate
        self.timer = self.create_timer(period, self.control_loop)
        
        self.get_logger().info(f"🚀 Robust Velocity Blender started")
        self.get_logger().info(f"   Kp={self.kp}, Kd={self.kd}")
        self.get_logger().info(f"   Settling: enabled={self.settling_enabled}, Kp={self.settling_kp}")
        self.get_logger().info(f"   Error threshold: {self.settling_error_threshold} rad ({np.degrees(self.settling_error_threshold):.2f}°)")

    # ===== CALLBACKS =====
    
    def joint_state_callback(self, msg: JointState):
        """Aggiorna stato corrente del robot."""
        for i, name in enumerate(self.joint_names):
            if name in msg.name:
                idx = msg.name.index(name)
                self.q[i] = msg.position[idx]
                if len(msg.velocity) > idx:
                    self.qdot[i] = msg.velocity[idx]
    
    def trajectory_callback(self, msg: JointTrajectory):
        """Riceve nuova traiettoria."""
        if not msg.points:
            self.get_logger().warn("⚠️ Empty trajectory received")
            return
        
        # Crea mapping dei nomi
        self.traj_index_map = {}
        for i, name in enumerate(self.joint_names):
            if name in msg.joint_names:
                self.traj_index_map[i] = msg.joint_names.index(name)
        
        if len(self.traj_index_map) != self.n_dof:
            self.get_logger().error(f"❌ Joint name mismatch! Found {len(self.traj_index_map)}/{self.n_dof}")
            return
        
        self.traj = msg
        self.traj_start_time = self.get_clock().now()
        
        # Salva target finale
        last_point = msg.points[-1]
        self.q_target = np.array([
            last_point.positions[self.traj_index_map[i]] 
            for i in range(self.n_dof)
        ])
        
        # Reset settling
        self.is_settling = False
        self.settling_start_time = None
        
        duration = self._trajectory_duration()
        self.get_logger().info(
            f"📈 New trajectory: {len(msg.points)} points, {duration:.2f}s"
        )
    
    def avoidance_callback(self, msg: Float64MultiArray):
        """Riceve velocità di avoidance."""
        if len(msg.data) == self.n_dof:
            self.qdot_avoid = np.array(msg.data)

    # ===== CONTROL LOOP =====
    
    def control_loop(self):
        """Main control loop."""
        now = self.get_clock().now()
        
        # Calcola velocità di tracking
        qdot_track = self.compute_tracking_velocity(now)
        
        # Combina tracking + avoidance
        qdot_cmd = qdot_track + self.qdot_avoid
        
        # Applica limite di accelerazione
        if self.previous_time is not None:
            dt = (now - self.previous_time).nanoseconds * 1e-9
            if 0 < dt < 0.1:
                qdot_cmd = self._limit_acceleration(qdot_cmd, dt)
        
        # Saturazione velocità
        qdot_cmd = np.clip(qdot_cmd, -self.max_joint_vel, self.max_joint_vel)
        
        # Pubblica
        msg = Float64MultiArray()
        msg.data = qdot_cmd.tolist()
        self.cmd_pub.publish(msg)
        
        # Aggiorna stato
        self.previous_qdot_cmd = qdot_cmd.copy()
        self.previous_time = now
        
        # Logging periodico
        self.log_counter += 1
        if self.log_counter % 200 == 0:  # Ogni 2 secondi a 100Hz
            self._log_status(qdot_track, qdot_cmd)
    
    def compute_tracking_velocity(self, now) -> np.ndarray:
        """
        Calcola velocità di tracking.
        
        Durante la traiettoria: PD tracking
        Dopo la traiettoria: Settling phase con controllo P
        """
        
        # Nessuna traiettoria attiva
        if self.traj is None or self.traj_start_time is None:
            return np.zeros(self.n_dof)
        
        t = (now - self.traj_start_time).nanoseconds * 1e-9
        t_final = self._trajectory_duration()
        
        # ===== FASE 1: TRACKING TRAIETTORIA =====
        if t < t_final:
            # Interpola posizione e velocità desiderate
            q_d, qdot_d = self._interpolate_trajectory(t)
            
            # Controllo PD
            q_err = q_d - self.q
            qdot_err = qdot_d - self.qdot
            
            qdot_track = self.kp * q_err + self.kd * qdot_err
            
            self.last_error_norm = np.linalg.norm(q_err)
            return qdot_track
        
        # ===== FASE 2: SETTLING =====
        if not self.settling_enabled or self.q_target is None:
            # Settling disabilitato, ferma
            self._reset_trajectory()
            return np.zeros(self.n_dof)
        
        # Calcola errore verso target finale
        q_err = self.q_target - self.q
        error_norm = np.linalg.norm(q_err)
        self.last_error_norm = error_norm
        
        # Inizia settling se non già attivo
        if not self.is_settling:
            self.is_settling = True
            self.settling_start_time = now
            self.get_logger().info(
                f"🎯 Settling phase started, error: {error_norm:.4f} rad "
                f"({np.degrees(error_norm):.2f}°)"
            )
        
        # Verifica timeout
        settling_duration = (now - self.settling_start_time).nanoseconds * 1e-9
        if settling_duration > self.settling_timeout:
            self.get_logger().warn(
                f"⏱️ Settling TIMEOUT! Final error: {error_norm:.4f} rad "
                f"({np.degrees(error_norm):.2f}°)"
            )
            self._reset_trajectory()
            return np.zeros(self.n_dof)
        
        # Verifica convergenza
        if error_norm < self.settling_error_threshold:
            self.get_logger().info(
                f"✅ TARGET REACHED! Error: {error_norm:.6f} rad "
                f"({np.degrees(error_norm):.3f}°) in {settling_duration:.1f}s"
            )
            self._reset_trajectory()
            return np.zeros(self.n_dof)
        
        # Controllo P per settling (gain più alto per convergenza)
        qdot_settling = self.settling_kp * q_err
        
        return qdot_settling
    
    # ===== HELPER METHODS =====
    
    def _interpolate_trajectory(self, t: float):
        """Interpola la traiettoria al tempo t."""
        points = self.traj.points
        times = [self._point_time(p) for p in points]
        
        # Prima del primo punto
        if t <= times[0]:
            return self._extract_point(points[0])
        
        # Dopo l'ultimo punto
        if t >= times[-1]:
            return self._extract_point(points[-1])
        
        # Trova segmento
        for i in range(len(times) - 1):
            if times[i] <= t < times[i + 1]:
                # Interpolazione lineare
                t0, t1 = times[i], times[i + 1]
                alpha = (t - t0) / (t1 - t0) if t1 > t0 else 0.0
                
                q0, qdot0 = self._extract_point(points[i])
                q1, qdot1 = self._extract_point(points[i + 1])
                
                q_d = q0 + alpha * (q1 - q0)
                qdot_d = qdot0 + alpha * (qdot1 - qdot0)
                
                return q_d, qdot_d
        
        # Fallback
        return self._extract_point(points[-1])
    
    def _extract_point(self, point):
        """Estrae q e qdot da un punto della traiettoria."""
        q = np.array([
            point.positions[self.traj_index_map[i]] 
            for i in range(self.n_dof)
        ])
        
        if point.velocities:
            qdot = np.array([
                point.velocities[self.traj_index_map[i]] 
                for i in range(self.n_dof)
            ])
        else:
            qdot = np.zeros(self.n_dof)
        
        return q, qdot
    
    def _point_time(self, point) -> float:
        """Converte time_from_start in secondi."""
        return point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
    
    def _trajectory_duration(self) -> float:
        """Durata totale della traiettoria."""
        if self.traj is None or not self.traj.points:
            return 0.0
        return self._point_time(self.traj.points[-1])
    
    def _limit_acceleration(self, qdot_new: np.ndarray, dt: float) -> np.ndarray:
        """Limita l'accelerazione per evitare movimenti bruschi."""
        dv = qdot_new - self.previous_qdot_cmd
        max_dv = self.max_acceleration * dt
        
        # Limita ogni giunto
        for i in range(len(dv)):
            if abs(dv[i]) > max_dv:
                dv[i] = np.sign(dv[i]) * max_dv
        
        return self.previous_qdot_cmd + dv
    
    def _reset_trajectory(self):
        """Reset stato traiettoria."""
        self.traj = None
        self.traj_start_time = None
        self.q_target = None
        self.is_settling = False
        self.settling_start_time = None
    
    def _log_status(self, qdot_track: np.ndarray, qdot_cmd: np.ndarray):
        """Log periodico dello stato."""
        status = "SETTLING" if self.is_settling else ("TRACKING" if self.traj else "IDLE")
        avoid_norm = np.linalg.norm(self.qdot_avoid)
        
        self.get_logger().info(
            f"📊 [{status}] Error: {self.last_error_norm:.4f} rad "
            f"({np.degrees(self.last_error_norm):.2f}°), "
            f"|v_track|: {np.linalg.norm(qdot_track):.3f}, "
            f"|v_avoid|: {avoid_norm:.3f}, "
            f"|v_cmd|: {np.linalg.norm(qdot_cmd):.3f} rad/s"
        )


def main(args=None):
    rclpy.init(args=args)
    node = RobustVelocityBlender()
    
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