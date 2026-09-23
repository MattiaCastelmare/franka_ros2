#!/usr/bin/env python3
import os
import csv
import time
from functools import partial
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory
from franka_msgs.msg import HumanArmState, HumanArmPrediction, MultiLinkDistance, KalmanDiagnostics
from franka_experiments.utils.distance_utils import load_robot_config
from franka_experiments.utils.human_utils import format_topic, stamp_to_ns
from franka_experiments.utils.constants import FR3_JOINT_NAMES


class BaseLogger:
    """Base class handling CSV file creation, header writing, and data flushing.

    Every row starts with 'timestamp' (message header stamp, for alignment) and
    'recv_time' (clock at reception); recv_time - timestamp is the pipeline latency.
    """

    def __init__(self, filename: str, headers: list[str]):
        Path(filename).parent.mkdir(parents=True, exist_ok=True)
        self.filename = filename
        self.file = open(self.filename, mode='w', newline='')
        self.writer = csv.writer(self.file)
        self.writer.writerow(['timestamp', 'recv_time'] + headers)

    def log(self, data: list):
        """Write a single data row to the CSV file."""
        self.writer.writerow(data)

    def flush(self):
        if not self.file.closed:
            self.file.flush()

    def close(self):
        """Flush and close the open file handle safely."""
        if not self.file.closed:
            self.file.flush()
            self.file.close()


class HumanRawLogger(BaseLogger):
    """Logger for raw 3D positions before Kalman Filtering."""

    def __init__(self, base_path: str):
        headers = []
        for kp in ['shoulder', 'elbow', 'wrist', 'hand']:
            headers.extend([f'{kp}_x', f'{kp}_y', f'{kp}_z'])
        super().__init__(f"{base_path}_human_raw.csv", headers)


class HumanStateLogger(BaseLogger):
    """Logger for filtered human arm state and perception diagnostics (HumanArmState.msg)."""

    def __init__(self, base_path: str):
        headers = []
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
            'num_steps', 'step_dt', 'horizon_sec',
            'hand_pred_end_x', 'hand_pred_end_y', 'hand_pred_end_z',
            'wrist_pred_end_x', 'wrist_pred_end_y', 'wrist_pred_end_z',
            'valid_shoulder', 'valid_elbow', 'valid_wrist', 'valid_hand'
        ]
        super().__init__(f"{base_path}_human_prediction.csv", headers)


class RobotStateLogger(BaseLogger):
    """Logger for 7-DoF robot joint position, velocity, acceleration, and torque (JointState)."""

    def __init__(self, base_path: str, num_joints: int = 7):
        headers = []
        for i in range(1, num_joints + 1):
            headers.extend([f'q_{i}', f'dq_{i}', f'tau_{i}'])
        super().__init__(f"{base_path}_robot_state.csv", headers)


class SafetyDistanceLogger(BaseLogger):
    """Logger for minimum human-robot clearance distance and closest points."""

    def __init__(self, base_path: str):
        headers = [
            'min_distance', 
            'robot_closest_link', 'robot_cp_x', 'robot_cp_y', 'robot_cp_z',
            'human_closest_capsule', 'human_cp_x', 'human_cp_y', 'human_cp_z'
        ]
        super().__init__(f"{base_path}_safety_distance.csv", headers)


class KalmanDiagnosticsLogger(BaseLogger):
    """Logger for Kalman Filter internal diagnostics (Whiteness Test and Covariance Trace)."""

    def __init__(self, base_path: str):
        headers = []
        keypoints = ['shoulder', 'elbow', 'wrist', 'hand']
        
        for kp in keypoints:
            headers.extend([f'{kp}_inn_x', f'{kp}_inn_y', f'{kp}_inn_z', f'{kp}_p_trace'])
            
        super().__init__(f"{base_path}_kf_diagnostics.csv", headers)


class ControllerDiagnosticsLogger(BaseLogger):
    """Logger for CBF vs MPC operational metrics and solve times."""

    def __init__(self, base_path: str):
        headers = [
            'active_controller', 'solve_time_ms', 
            'cost_value', 'is_converged', 'tracking_error_pos'
        ]
        super().__init__(f"{base_path}_controller_diag.csv", headers)


class ExperimentLoggerNode(Node):
    """ROS 2 Node that centralizes experiment data recording across perception, control, and robot state."""

    def __init__(self):
        super().__init__('human_logger')

        config_path = os.path.join(
            get_package_share_directory('franka_experiments'),
            'config',
            'human_params.yaml',
        )
        full_config = load_robot_config(config_path)
        tracker_config = full_config.get('human_tracker', {})
        self.pose_side = str(tracker_config["pose_side"]).lower()
        self.active_sides = ["left", "right"] if self.pose_side == "both" else [self.pose_side]

        # Create timestamped experiment output folder
        session_time = time.strftime("%Y%m%d_%H%M%S")
        if not self.has_parameter('run_name'):
            self.declare_parameter('run_name', session_time)
        run_name = self.get_parameter('run_name').get_parameter_value().string_value
        base_log_dir = Path(f"experiment_logs/{run_name}")
        base_log_path = str(base_log_dir / "experiment")

        self.get_logger().info(f"Initializing logging node. Target directory: {base_log_dir}, mode: {self.pose_side}")

        # Instantiate Global Loggers
        self.robot_logger = RobotStateLogger(base_log_path)
        self.distance_logger = SafetyDistanceLogger(base_log_path)
        self.diag_logger = ControllerDiagnosticsLogger(base_log_path)

        # State cache for diagnostics
        self.current_controller = "CBF"
        
        # Instantiate Dictionaries for Human Loggers
        self.human_raw_loggers = {}
        self.human_state_loggers = {}
        self.human_pred_loggers = {}
        self.kf_diag_loggers = {}
        
        # Subscriptions Dictionaries
        self.human_raw_subs = {}
        self.human_state_subs = {}
        self.human_pred_subs = {}
        self.kf_diag_subs = {}

        for side in self.active_sides:
            # Add side prefix to files only if tracking both arms, else keep standard name
            side_path = f"{base_log_path}_{side}" if len(self.active_sides) > 1 else base_log_path
            
            self.human_raw_loggers[side] = HumanRawLogger(side_path)
            self.human_state_loggers[side] = HumanStateLogger(side_path)
            self.human_pred_loggers[side] = HumanPredictionLogger(side_path)
            self.kf_diag_loggers[side] = KalmanDiagnosticsLogger(side_path)

            # Topic formatting
            prefix = f"{side}_" if self.pose_side == "both" else ""

            self.human_raw_subs[side] = self.create_subscription(
                HumanArmState, format_topic('/human/raw_state', prefix), partial(self.human_raw_callback, side=side), 10)
            
            self.human_state_subs[side] = self.create_subscription(
                HumanArmState, format_topic('/human/arm_state', prefix), partial(self.human_state_callback, side=side), 10)

            self.human_pred_subs[side] = self.create_subscription(
                HumanArmPrediction, format_topic('/human/arm_prediction', prefix), partial(self.human_prediction_callback, side=side), 10)

            self.kf_diag_subs[side] = self.create_subscription(
                KalmanDiagnostics, format_topic('/human/kf_diagnostics', prefix), partial(self.kf_diag_callback, side=side), 10)

        # Global subscriptions
        latest_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.joint_state_sub = self.create_subscription(
            JointState, '/NS_1/joint_states', self.robot_state_callback, 10)

        self.min_dist_sub = self.create_subscription(
            MultiLinkDistance, '/human/per_link_distances', self.min_distance_callback, latest_qos)

        self.mux_sub = self.create_subscription(
            String, '/controller_mux/active_controller', self.active_controller_callback, 10)

        # Periodic flush so data survives a hard kill
        self.flush_timer = self.create_timer(1.0, self.flush_all)

    def all_loggers(self) -> list:
        loggers = [self.robot_logger, self.distance_logger, self.diag_logger]
        for side in self.active_sides:
            loggers += [self.human_raw_loggers[side], self.human_state_loggers[side],
                        self.human_pred_loggers[side], self.kf_diag_loggers[side]]
        return loggers

    def flush_all(self):
        for logger in self.all_loggers():
            logger.flush()

    def times(self, msg) -> list:
        """[header stamp, reception time] in seconds."""
        return [stamp_to_ns(msg) * 1e-9, self.get_clock().now().nanoseconds * 1e-9]

    def human_raw_callback(self, msg: HumanArmState, side: str):
        row = self.times(msg)
        keypoints = [msg.shoulder, msg.elbow, msg.wrist, msg.hand]
        for pt in keypoints:
            row.extend([pt.x, pt.y, pt.z])
        self.human_raw_loggers[side].log(row)
        
    def human_state_callback(self, msg: HumanArmState, side: str):
        """Log incoming filtered human joint state and perception metrics."""
        row = self.times(msg)

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
        self.human_state_loggers[side].log(row)

    def human_prediction_callback(self, msg: HumanArmPrediction, side: str):
        """Log multi-step trajectory prediction summary."""
        horizon = msg.num_steps * msg.step_dt

        # End of horizon predicted positions for hand and wrist
        hand_end = msg.hand[-1] if len(msg.hand) > 0 else None
        wrist_end = msg.wrist[-1] if len(msg.wrist) > 0 else None

        row = self.times(msg) + [
            msg.num_steps, msg.step_dt, horizon,
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
        self.human_pred_loggers[side].log(row)

    def kf_diag_callback(self, msg: KalmanDiagnostics, side: str):
        """Log KF innovations and covariance traces."""
        row = self.times(msg)
        for i in range(4):
            inn = msg.innovations[i]
            row.extend([inn.x, inn.y, inn.z, msg.p_traces[i]])
        self.kf_diag_loggers[side].log(row)

    def robot_state_callback(self, msg: JointState):
        """Log robot joint positions, velocities, and torques."""
        row = self.times(msg)
        
        # Arm joints by name: JointState order is not guaranteed (e.g. gripper joints)
        index = {name: i for i, name in enumerate(msg.name)}
        if not all(name in index for name in FR3_JOINT_NAMES):
            return
        for name in FR3_JOINT_NAMES:
            i = index[name]
            q = msg.position[i] if i < len(msg.position) else 0.0
            dq = msg.velocity[i] if i < len(msg.velocity) else 0.0
            tau = msg.effort[i] if i < len(msg.effort) else 0.0
            row.extend([q, dq, tau])

        self.robot_logger.log(row)

    def min_distance_callback(self, msg: MultiLinkDistance):
        """Log minimum human-robot clearance distance and closest points from CBF data."""
        if not msg.links:
            return
        
        # Find global minimum
        min_link = min(msg.links, key=lambda l: l.distance)

        row = self.times(msg) + [
            min_link.distance, 
            min_link.robot_link_name, 
            min_link.closest_point_robot.x, 
            min_link.closest_point_robot.y, 
            min_link.closest_point_robot.z,
            min_link.human_capsule,
            min_link.closest_point_human.x, 
            min_link.closest_point_human.y, 
            min_link.closest_point_human.z
        ]
        self.distance_logger.log(row)

    def active_controller_callback(self, msg: String):
        """Track which controller (CBF or MPC) is currently driving the robot."""
        self.current_controller = msg.data

    def destroy_node(self):
        """Safely close all open CSV file writers upon node exit."""
        self.get_logger().info("Closing all experiment log files...")
        for logger in self.all_loggers():
            logger.close()
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
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()