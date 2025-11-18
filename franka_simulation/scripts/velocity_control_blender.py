#!/usr/bin/env python3
"""
Velocity Control Blender (formal)
================================

- Riceve una JointTrajectory su /franka/velocity_trajectory
- Interpola q_d(t), qdot_d(t)
- Applica legge di tracking in velocità:
      qdot_cmd = qdot_d + Kp * (q_d - q)
- Somma qdot_avoid da /avoidance/velocity
- Pubblica su /fr3_velocity_controller/commands

Serve:
  - online_avoidance_controller (già attivo)
  - un nodo che pubblica JointTrajectory su /franka/velocity_trajectory
"""

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class VelocityControlBlender(Node):

    def __init__(self):
        super().__init__("velocity_control_blender")

        # ===== Parametri =====
        self.declare_parameter(
            "joint_names",
            [
                "fr3_joint1",
                "fr3_joint2",
                "fr3_joint3",
                "fr3_joint4",
                "fr3_joint5",
                "fr3_joint6",
                "fr3_joint7",
            ],
        )
        self.declare_parameter("kp_tracking", 3.0)      # guadagno P in velocità
        self.declare_parameter("max_joint_vel", 1.0)    # [rad/s]

        self.joint_names = (
            self.get_parameter("joint_names").get_parameter_value().string_array_value
        )
        self.kp = self.get_parameter("kp_tracking").value
        self.max_joint_vel = self.get_parameter("max_joint_vel").value

        self.n_dof = len(self.joint_names)

        # ===== Stati interni =====
        self.q = np.zeros(self.n_dof)          # stato attuale
        self.qdot_avoid = np.zeros(self.n_dof) # da avoidance
        self.traj: JointTrajectory | None = None
        self.traj_start_time = None
        self.traj_index_map = None             # mapping joint_names traj → indice

        # ===== Subscribers =====
        qos1 = QoSProfile(depth=1)
        qos10 = QoSProfile(depth=10)

        # Joint states
        self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_state_callback,
            qos10,
        )

        # Velocità di avoidance
        self.create_subscription(
            Float64MultiArray,
            "/avoidance/velocity",
            self.avoidance_callback,
            qos1,
        )

        # Traiettoria di riferimento
        self.create_subscription(
            JointTrajectory,
            "/franka/velocity_trajectory",
            self.trajectory_callback,
            qos1,
        )

        # ===== Publisher comandi velocità =====
        self.cmd_pub = self.create_publisher(
            Float64MultiArray,
            "/fr3_velocity_controller/commands",
            qos10,
        )

        # ===== Timer controllo (300 Hz) =====
        self.control_timer = self.create_timer(1.0 / 300.0, self.control_loop)

        self.get_logger().info("🚀 VelocityControlBlender (formal) avviato")
        self.get_logger().info(f"  • joint_names = {self.joint_names}")
        self.get_logger().info(f"  • kp_tracking = {self.kp}")
        self.get_logger().info(f"  • max_joint_vel = {self.max_joint_vel} rad/s")

    # =========================================================
    # CALLBACKS
    # =========================================================

    def joint_state_callback(self, msg: JointState):
        """Aggiorna lo stato q attuale (solo i 7 giunti FR3)."""
        name_to_pos = dict(zip(msg.name, msg.position))
        q_list = []
        for jn in self.joint_names:
            if jn in name_to_pos:
                q_list.append(name_to_pos[jn])
        if len(q_list) == self.n_dof:
            self.q = np.array(q_list, dtype=float)

    def avoidance_callback(self, msg: Float64MultiArray):
        """Aggiorna qdot_avoid dal controller di obstacle avoidance."""
        data = np.array(msg.data, dtype=float)
        if data.shape[0] == self.n_dof:
            self.qdot_avoid = data

    def trajectory_callback(self, msg: JointTrajectory):
        """
        Riceve una nuova JointTrajectory di riferimento.
        Assume che joint_names contenga i 7 giunti del braccio.
        """
        if not msg.points:
            self.get_logger().warn("⚠️ Trajectory vuota ricevuta, ignorata.")
            return

        # Mappa joint_names della traiettoria → ordine desiderato
        traj_names = list(msg.joint_names)
        index_map = []
        for jn in self.joint_names:
            if jn not in traj_names:
                self.get_logger().error(
                    f"❌ Joint {jn} non trovato nella JointTrajectory ricevuta."
                )
                return
            index_map.append(traj_names.index(jn))

        self.traj = msg
        self.traj_start_time = self.get_clock().now()
        self.traj_index_map = index_map

        duration = self._trajectory_total_time(msg)
        self.get_logger().info(
            f"📈 Nuova JointTrajectory ricevuta: {len(msg.points)} punti, durata ~{duration:.2f}s"
        )

    # =========================================================
    # CONTROL LOOP
    # =========================================================

    def control_loop(self):
        # Calcolo qdot_tracking da traiettoria (se presente)
        qdot_track = self.compute_tracking_velocity()

        # Somma con avoidance
        qdot_cmd = qdot_track + self.qdot_avoid

        # Saturazione
        qdot_cmd = np.clip(qdot_cmd, -self.max_joint_vel, self.max_joint_vel)

        # Pubblica
        msg = Float64MultiArray()
        msg.data = qdot_cmd.tolist()
        self.cmd_pub.publish(msg)

    # =========================================================
    # TRACKING TRAIETTORIA — PARTE "FORMALE"
    # =========================================================

    def compute_tracking_velocity(self) -> np.ndarray:
        """
        1) Se non c'è traiettoria, ritorna 0.
        2) Se c'è traiettoria:
           - calcola tempo t dal momento in cui è stata ricevuta
           - interpola q_d(t) e qdot_d(t)
           - applica legge: qdot = qdot_d + Kp * (q_d - q)
        """
        if self.traj is None or self.traj_start_time is None:
            return np.zeros(self.n_dof)

        now = self.get_clock().now()
        t = (now - self.traj_start_time).nanoseconds * 1e-9

        # Se siamo oltre la fine → stop morbido (solo termine P piccolo o 0)
        t_final = self._trajectory_total_time(self.traj)
        if t >= t_final:
            return np.zeros(self.n_dof)

        # Trova il segmento [i, i+1] in cui cade t
        times = [self._time_from_start(p) for p in self.traj.points]
        # Caso inizio
        if t <= times[0]:
            q_d, qdot_d = self._extract_point(self.traj.points[0])
        else:
            # Trova i tale che times[i] <= t < times[i+1]
            idx = None
            for i in range(len(times) - 1):
                if times[i] <= t < times[i + 1]:
                    idx = i
                    break
            if idx is None:
                idx = len(times) - 2

            t0 = times[idx]
            t1 = times[idx + 1]
            p0 = self.traj.points[idx]
            p1 = self.traj.points[idx + 1]

            alpha = (t - t0) / max(t1 - t0, 1e-6)
            q0, v0 = self._extract_point(p0)
            q1, v1 = self._extract_point(p1)

            # Interpolazione lineare posizione e velocità
            q_d = (1.0 - alpha) * q0 + alpha * q1
            qdot_d = (1.0 - alpha) * v0 + alpha * v1

        # Legge di tracking in velocità
        q_err = q_d - self.q
        qdot_track = qdot_d + self.kp * q_err
        return qdot_track

    # =========================================================
    # UTILITY TRAIETTORIA
    # =========================================================

    @staticmethod
    def _time_from_start(point: JointTrajectoryPoint) -> float:
        return point.time_from_start.sec + point.time_from_start.nanosec * 1e-9

    @staticmethod
    def _trajectory_total_time(traj: JointTrajectory) -> float:
        last = traj.points[-1]
        return last.time_from_start.sec + last.time_from_start.nanosec * 1e-9

    def _extract_point(self, point: JointTrajectoryPoint):
        """
        Estrae (q, qdot) nell'ordine self.joint_names usando la mappa traj_index_map.
        Se le velocità nella traiettoria non sono valorizzate, le approssima a zero.
        """
        if self.traj_index_map is None:
            # fallback (dovrebbe non succedere se trajectory_callback è andato a buon fine)
            q = np.zeros(self.n_dof)
            v = np.zeros(self.n_dof)
            return q, v

        q = []
        v = []
        for idx in self.traj_index_map:
            q.append(point.positions[idx])
            if point.velocities and len(point.velocities) == len(self.traj.joint_names):
                v.append(point.velocities[idx])
            else:
                v.append(0.0)

        return np.array(q, dtype=float), np.array(v, dtype=float)


def main(args=None):
    rclpy.init(args=args)
    node = VelocityControlBlender()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
