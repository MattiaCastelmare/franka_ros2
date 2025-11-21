#!/usr/bin/env python3
"""
Enhanced Velocity Control Blender with Smoothing
================================================

Improvements:
1. Smoother tracking control (lower Kp)
2. Velocity smoothing filter
3. Acceleration limiting
4. Gradual trajectory start/stop
5. Better avoidance integration

Author: Enhanced for smooth operation
"""

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class EnhancedVelocityBlender(Node):
    """Enhanced velocity blender with smoothing"""

    def __init__(self):
        super().__init__("velocity_control_blender")

        # Parameters
        self.declare_parameter(
            "joint_names",
            [
                "fr3_joint1", "fr3_joint2", "fr3_joint3", "fr3_joint4",
                "fr3_joint5", "fr3_joint6", "fr3_joint7",
            ],
        )
        # REDUCED tracking gain for smoother motion
        self.declare_parameter("kp_tracking", 1.5)  # Was 3.0
        # REDUCED max velocity for safety
        self.declare_parameter("max_joint_vel", 0.6)  # Was 1.0
        
        # NEW: Smoothing parameters
        self.declare_parameter("enable_smoothing", True)
        self.declare_parameter("smoothing_alpha", 0.4)  # Tracking smoothing
        self.declare_parameter("max_acceleration", 1.5)  # rad/s²

        self.joint_names = (
            self.get_parameter("joint_names").get_parameter_value().string_array_value
        )
        self.kp = self.get_parameter("kp_tracking").value
        self.max_joint_vel = self.get_parameter("max_joint_vel").value
        
        self.enable_smoothing = self.get_parameter("enable_smoothing").value
        self.smoothing_alpha = self.get_parameter("smoothing_alpha").value
        self.max_acceleration = self.get_parameter("max_acceleration").value

        self.n_dof = len(self.joint_names)

        # State
        self.q = np.zeros(self.n_dof)
        self.qdot_avoid = np.zeros(self.n_dof)
        self.traj: JointTrajectory | None = None
        self.traj_start_time = None
        self.traj_index_map = None

        # Smoothing state
        self.previous_qdot_cmd = np.zeros(self.n_dof)
        self.previous_qdot_track = np.zeros(self.n_dof)
        self.previous_time = None

        # Subscribers
        qos1 = QoSProfile(depth=1)
        qos10 = QoSProfile(depth=10)

        self.create_subscription(JointState, "/joint_states",
                                self.joint_state_callback, qos10)
        
        self.create_subscription(Float64MultiArray, "/avoidance/velocity",
                                self.avoidance_callback, qos1)
        
        self.create_subscription(JointTrajectory, "/velocity_blender/trajectory",
                                self.trajectory_callback, qos1)

        # Publisher
        self.cmd_pub = self.create_publisher(Float64MultiArray,
                                             "/fr3_velocity_controller/commands", qos10)

        # Control timer (higher frequency for smoothness)
        self.control_timer = self.create_timer(1.0 / 200.0, self.control_loop)  # 200 Hz

        self.get_logger().info("🚀 Enhanced Velocity Blender started")
        self.get_logger().info(f"   • Kp tracking: {self.kp}")
        self.get_logger().info(f"   • Max velocity: {self.max_joint_vel} rad/s")
        self.get_logger().info(f"   • Smoothing: {'ENABLED' if self.enable_smoothing else 'DISABLED'}")

    def joint_state_callback(self, msg: JointState):
        """Update joint state"""
        name_to_pos = dict(zip(msg.name, msg.position))
        q_list = []
        for jn in self.joint_names:
            if jn in name_to_pos:
                q_list.append(name_to_pos[jn])
        if len(q_list) == self.n_dof:
            self.q = np.array(q_list, dtype=float)

    def avoidance_callback(self, msg: Float64MultiArray):
        """Update avoidance velocity"""
        data = np.array(msg.data, dtype=float)
        if data.shape[0] == self.n_dof:
            self.qdot_avoid = data

    def trajectory_callback(self, msg: JointTrajectory):
        """Receive new trajectory"""
        if not msg.points:
            self.get_logger().warn("⚠️ Empty trajectory received")
            return

        # Map joint names
        traj_names = list(msg.joint_names)
        index_map = []
        for jn in self.joint_names:
            if jn not in traj_names:
                self.get_logger().error(f"❌ Joint {jn} not in trajectory")
                return
            index_map.append(traj_names.index(jn))

        self.traj = msg
        self.traj_start_time = self.get_clock().now()
        self.traj_index_map = index_map

        duration = self._trajectory_total_time(msg)
        self.get_logger().info(
            f"📈 New trajectory: {len(msg.points)} points, {duration:.2f}s"
        )

    def control_loop(self):
        """Main control loop with smoothing"""
        current_time = self.get_clock().now()
        
        # Compute tracking velocity
        qdot_track = self.compute_tracking_velocity()
        
        # Apply smoothing to tracking
        if self.enable_smoothing:
            qdot_track = self.apply_tracking_smoothing(qdot_track)
        
        # Combine tracking + avoidance
        qdot_cmd = qdot_track + self.qdot_avoid
        
        # Apply acceleration limiting to combined command
        if self.enable_smoothing and self.previous_time is not None:
            dt = (current_time - self.previous_time).nanoseconds * 1e-9
            if dt > 0 and dt < 0.1:  # Reasonable dt
                qdot_cmd = self.apply_acceleration_limit(qdot_cmd, dt)
        
        # Saturate
        qdot_cmd = np.clip(qdot_cmd, -self.max_joint_vel, self.max_joint_vel)
        
        # Publish
        msg = Float64MultiArray()
        msg.data = qdot_cmd.tolist()
        self.cmd_pub.publish(msg)
        
        # Update state
        self.previous_qdot_cmd = qdot_cmd.copy()
        self.previous_qdot_track = qdot_track.copy()
        self.previous_time = current_time

    def apply_tracking_smoothing(self, qdot_track_new: np.ndarray) -> np.ndarray:
        """Apply exponential smoothing to tracking velocity"""
        alpha = self.smoothing_alpha
        qdot_smoothed = alpha * qdot_track_new + (1.0 - alpha) * self.previous_qdot_track
        return qdot_smoothed

    def apply_acceleration_limit(self, qdot_new: np.ndarray, dt: float) -> np.ndarray:
        """Limit acceleration to prevent jerky motion"""
        dv = qdot_new - self.previous_qdot_cmd
        acceleration = dv / dt
        
        # Limit per joint
        for i in range(len(acceleration)):
            if abs(acceleration[i]) > self.max_acceleration:
                max_dv = self.max_acceleration * dt
                dv[i] = np.sign(dv[i]) * max_dv
        
        qdot_limited = self.previous_qdot_cmd + dv
        return qdot_limited

    def compute_tracking_velocity(self) -> np.ndarray:
        """
        Compute tracking velocity with smoother control law.
        
        Reduced Kp for smoother tracking (accepts more tracking error
        but eliminates jerky motion).
        """
        if self.traj is None or self.traj_start_time is None:
            return np.zeros(self.n_dof)

        now = self.get_clock().now()
        t = (now - self.traj_start_time).nanoseconds * 1e-9

        t_final = self._trajectory_total_time(self.traj)
        if t >= t_final:
            # Trajectory finished - smooth stop
            return np.zeros(self.n_dof)

        # Interpolate trajectory
        times = [self._time_from_start(p) for p in self.traj.points]
        
        if t <= times[0]:
            q_d, qdot_d = self._extract_point(self.traj.points[0])
        else:
            # Find segment
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

            # Smooth interpolation
            q_d = (1.0 - alpha) * q0 + alpha * q1
            qdot_d = (1.0 - alpha) * v0 + alpha * v1

        # Tracking control with REDUCED gain
        q_err = q_d - self.q
        qdot_track = qdot_d + self.kp * q_err
        
        return qdot_track

    @staticmethod
    def _time_from_start(point: JointTrajectoryPoint) -> float:
        return point.time_from_start.sec + point.time_from_start.nanosec * 1e-9

    @staticmethod
    def _trajectory_total_time(traj: JointTrajectory) -> float:
        last = traj.points[-1]
        return last.time_from_start.sec + last.time_from_start.nanosec * 1e-9

    def _extract_point(self, point: JointTrajectoryPoint):
        """Extract (q, qdot) from trajectory point"""
        if self.traj_index_map is None:
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
    node = EnhancedVelocityBlender()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()