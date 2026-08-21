#!/usr/bin/env python3
import csv
import time
from pathlib import Path
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, String

from franka_msgs.msg import HumanArmState, HumanArmPrediction


class BaseLogger:
    """Base class handling CSV file creation, header writing, and data flushing."""

    def __init__(self, filename: str, headers: list[str]):
        Path(filename).parent.mkdir(parents=True, exist_ok=True)
        self.filename = filename
        self.file = open(self.filename, mode='w', newline='')
        self.writer = csv.writer(self.file)
        self.writer.writerow(headers)

    def log(self, data: list):
        """Write a single data row to the CSV file."""
        self.writer.writerow(data)

    def close(self):
        """Flush and close the open file handle safely."""
        if not self.file.closed:
            self.file.flush()
            self.file.close()


class HumanRawLogger(BaseLogger):
    """Logger for raw 3D positions before Kalman Filtering."""
    def __init__(self, base_path: str):
        headers = ['timestamp']
        for kp in ['shoulder', 'elbow', 'wrist', 'hand']:
            headers.extend([f'{kp}_x', f'{kp}_y', f'{kp}_z'])
        super().__init__(f"{base_path}_human_raw.csv", headers)


class HumanStateLogger(BaseLogger):
    """Logger for filtered human arm state and perception diagnostics (HumanArmState.msg)."""

    def __init__(self, base_path: str):
        headers = ['timestamp']
        keypoints = ['shoulder', 'elbow', 'wrist', 'hand']
        
        # Positions and Velocities for 4 keypoints
        for kp in keypoints:
            headers.extend([f'{kp}_x', f'{kp}_y', f'{kp}_z'])
            headers.extend([f'{kp}_vx', f'{kp}_vy', f'{kp}_vz'])

        # Per-keypoint visibility and quality indicators
        for i in range(4):
            headers.extend([f'vis_{i}', f'measured_{i}', f'valid_{i}', f'age_{i}'])

        # Global metrics
        headers.extend(['confidence', 'valid', 'occluded'])
        super().__init__(f"{base_path}_human_state.csv", headers)


class HumanPredictionLogger(BaseLogger):
    """Logger for multi-step human trajectory predictions (HumanArmPrediction.msg)."""

    def __init__(self, base_path: str):
        headers = [
            'timestamp', 'num_steps', 'step_dt', 'horizon_sec',
            'hand_pred_end_x', 'hand_pred_end_y', 'hand_pred_end_z',
            'wrist_pred_end_x', 'wrist_pred_end_y', 'wrist_pred_end_z',
            'valid_shoulder', 'valid_elbow', 'valid_wrist', 'valid_hand'
        ]
        super().__init__(f"{base_path}_human_prediction.csv", headers)


class RobotStateLogger(BaseLogger):
    """Logger for 7-DoF robot joint position, velocity, acceleration, and torque (JointState)."""

    def __init__(self, base_path: str, num_joints: int = 7):
        headers = ['timestamp']
        for i in range(1, num_joints + 1):
            headers.extend([f'q_{i}', f'dq_{i}', f'tau_{i}'])
        super().__init__(f"{base_path}_robot_state.csv", headers)


class SafetyDistanceLogger(BaseLogger):
    """Logger for minimum human-robot clearance distance and closest points."""

    def __init__(self, base_path: str):
        headers = [
            'timestamp', 'min_distance', 
            'robot_closest_link', 'robot_cp_x', 'robot_cp_y', 'robot_cp_z',
            'human_closest_capsule', 'human_cp_x', 'human_cp_y', 'human_cp_z'
        ]
        super().__init__(f"{base_path}_safety_distance.csv", headers)


class ControllerDiagnosticsLogger(BaseLogger):
    """Logger for CBF vs MPC operational metrics and solve times."""

    def __init__(self, base_path: str):
        headers = [
            'timestamp', 'active_controller', 'solve_time_ms', 
            'cost_value', 'is_converged', 'tracking_error_pos'
        ]
        super().__init__(f"{base_path}_controller_diag.csv", headers)


class ExperimentLoggerNode(Node):
    """ROS 2 Node that centralizes experiment data recording across perception, control, and robot state."""

    def __init__(self):
        super().__init__('human_logger')

        # Create timestamped experiment output folder
        session_time = time.strftime("%Y%m%d_%H%M%S")
        base_log_dir = Path(f"experiment_logs/{session_time}")
        base_log_path = str(base_log_dir / "experiment")

        self.get_logger().info(f"Initializing logging node. Target directory: {base_log_dir}")

        # Instantiate loggers
        self.human_raw_logger = HumanRawLogger(base_log_path)
        self.human_state_logger = HumanStateLogger(base_log_path)
        self.human_pred_logger = HumanPredictionLogger(base_log_path)
        self.robot_logger = RobotStateLogger(base_log_path)
        self.distance_logger = SafetyDistanceLogger(base_log_path)
        self.diag_logger = ControllerDiagnosticsLogger(base_log_path)

        # State cache for diagnostics
        self.current_controller = "CBF"

        # --- Subscriptions ---
        self.human_raw_sub = self.create_subscription(
            HumanArmState, '/human/raw_state', self.human_raw_callback, 10)

        self.human_state_sub = self.create_subscription(
                    HumanArmState, '/human/arm_state', self.human_state_callback, 10)

        self.human_pred_sub = self.create_subscription(
            HumanArmPrediction, '/human/arm_prediction', self.human_prediction_callback, 10)

        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.robot_state_callback, 10)

        self.min_dist_sub = self.create_subscription(
            Float32, '/human_robot_distance/min_distance', self.min_distance_callback, 10)

        self.mux_sub = self.create_subscription(
            String, '/controller_mux/active_controller', self.active_controller_callback, 10)


    def human_raw_callback(self, msg: HumanArmState):
        t = self.get_clock().now().nanoseconds / 1e9
        row = [t]
        # Extract only the positions
        keypoints = [msg.shoulder, msg.elbow, msg.wrist, msg.hand]
        for pt in keypoints:
            row.extend([pt.x, pt.y, pt.z])
        self.human_raw_logger.log(row)
        
    def human_state_callback(self, msg: HumanArmState):
        """Log incoming filtered human joint state and perception metrics."""
        t = self.get_clock().now().nanoseconds / 1e9
        row = [t]

        # Extract positions and velocities for shoulder, elbow, wrist, hand
        keypoints = [msg.shoulder, msg.elbow, msg.wrist, msg.hand]
        velocities = [msg.shoulder_vel, msg.elbow_vel, msg.wrist_vel, msg.hand_vel]

        for pt, vel in zip(keypoints, velocities):
            row.extend([pt.x, pt.y, pt.z, vel.x, vel.y, vel.z])

        # Visibility, measured, valid, and age
        for i in range(4):
            row.extend([
                msg.visibility[i], 
                int(msg.measured[i]), 
                int(msg.keypoint_valid[i]), 
                msg.measurement_age[i]
            ])

        # Overall confidence and status flags
        row.extend([msg.confidence, int(msg.valid), int(msg.occluded)])
        self.human_state_logger.log(row)

    def human_prediction_callback(self, msg: HumanArmPrediction):
        """Log multi-step trajectory prediction summary."""
        t = self.get_clock().now().nanoseconds / 1e9
        horizon = msg.num_steps * msg.step_dt

        # End of horizon predicted positions for hand and wrist
        hand_end = msg.hand[-1] if len(msg.hand) > 0 else None
        wrist_end = msg.wrist[-1] if len(msg.wrist) > 0 else None

        row = [
            t, msg.num_steps, msg.step_dt, horizon,
            hand_end.x if hand_end else 0.0,
            hand_end.y if hand_end else 0.0,
            hand_end.z if hand_end else 0.0,
            wrist_end.x if wrist_end else 0.0,
            wrist_end.y if wrist_end else 0.0,
            wrist_end.z if wrist_end else 0.0,
            int(msg.keypoint_valid[0]),
            int(msg.keypoint_valid[1]),
            int(msg.keypoint_valid[2]),
            int(msg.keypoint_valid[3])
        ]
        self.human_pred_logger.log(row)

    def robot_state_callback(self, msg: JointState):
        """Log robot joint positions, velocities, and torques."""
        t = self.get_clock().now().nanoseconds / 1e9
        row = [t]
        
        # Log first 7 joints (Franka Emika Panda)
        n_joints = min(7, len(msg.position))
        for i in range(n_joints):
            q = msg.position[i] if i < len(msg.position) else 0.0
            dq = msg.velocity[i] if i < len(msg.velocity) else 0.0
            tau = msg.effort[i] if i < len(msg.effort) else 0.0
            row.extend([q, dq, tau])

        self.robot_logger.log(row)

    def min_distance_callback(self, msg: Float32):
        """Log minimum human-robot clearance distance."""
        t = self.get_clock().now().nanoseconds / 1e9
        # Simplified row format (expand with closest point data if using custom distance msg)
        row = [t, msg.data, "", 0.0, 0.0, 0.0, "", 0.0, 0.0, 0.0]
        self.distance_logger.log(row)

    def active_controller_callback(self, msg: String):
        """Track which controller (CBF or MPC) is currently driving the robot."""
        self.current_controller = msg.data

    def destroy_node(self):
        """Safely close all open CSV file writers upon node exit."""
        self.get_logger().info("Closing all experiment log files...")
        self.human_raw_logger.close()
        self.human_state_logger.close()
        self.human_pred_logger.close()
        self.robot_logger.close()
        self.distance_logger.close()
        self.diag_logger.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ExperimentLoggerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Experiment logger stopped by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()