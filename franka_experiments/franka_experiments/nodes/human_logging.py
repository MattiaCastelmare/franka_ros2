#!/usr/bin/env python3
"""CSV logs of an experiment, in experiment_logs/<controller>/<run_name>/experiment_*.csv.

``controller`` says what drove the robot and selects what is logged:
    human     tracking only, the robot does not move (human.launch.py)
    nominal   pick_place_qddot_commander straight to the torque chain
    cbf       the same, through cbf_safety_filter
    mpc       mpc_commander (mpc_cbf: with the CBF as shield)
The robot state is always logged; the human pipeline when ``log_human`` is true;
commands and task with any moving controller; CBF and MPC data with their controller.
Every row starts with 'timestamp' (message stamp) and 'recv_time' (clock at reception).
"""
import os
import csv
import time
from functools import partial
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String, UInt32
from trajectory_msgs.msg import MultiDOFJointTrajectory
from ament_index_python.packages import get_package_share_directory
from franka_msgs.msg import (
    HumanArmState, HumanArmPrediction, KalmanDiagnostics, MpcDiagnostics, MultiLinkDistance,
)
from franka_experiments.utils.distance_utils import load_robot_config
from franka_experiments.utils.human_utils import ee_trajectory_arrays, format_topic, stamp_to_ns
from franka_experiments.utils.constants import FR3_JOINT_NAMES


class BaseLogger:
    """Base class handling CSV file creation, header writing, and data flushing.

    Every row starts with 'timestamp' (message header stamp, for alignment) and
    'recv_time' (clock at reception); recv_time - timestamp is the pipeline latency.
    """

    def __init__(self, filename: str, headers: list[str] | None):
        """headers None: the subclass writes them itself (write_header), e.g. when they depend on the data."""
        Path(filename).parent.mkdir(parents=True, exist_ok=True)
        self.filename = filename
        self.file = open(self.filename, mode='w', newline='')
        self.writer = csv.writer(self.file)
        if headers is not None:
            self.write_header(headers)

    def write_header(self, headers: list[str]):
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
    """Logger for the minimum robot clearance from one distance source and its closest points.

    who='human': /human/per_link_distances (the arm capsules), file _safety_distance.csv;
    who='obstacle': /cbf/per_link_distances (depth obstacles), file _obstacle_distance.csv.
    """

    def __init__(self, base_path: str, who: str = 'human'):
        headers = [
            'min_distance',
            'robot_closest_link', 'robot_cp_x', 'robot_cp_y', 'robot_cp_z',
            f'{who}_closest_capsule', f'{who}_cp_x', f'{who}_cp_y', f'{who}_cp_z'
        ]
        name = 'safety_distance' if who == 'human' else f'{who}_distance'
        super().__init__(f"{base_path}_{name}.csv", headers)


class KalmanDiagnosticsLogger(BaseLogger):
    """Logger for Kalman Filter internal diagnostics (Whiteness Test and Covariance Trace)."""

    def __init__(self, base_path: str):
        headers = []
        keypoints = ['shoulder', 'elbow', 'wrist', 'hand']
        
        for kp in keypoints:
            headers.extend([f'{kp}_inn_x', f'{kp}_inn_y', f'{kp}_inn_z', f'{kp}_p_trace',
                            f'{kp}_nis', f'{kp}_gated'])

        super().__init__(f"{base_path}_kf_diagnostics.csv", headers)


class JointVectorLogger(BaseLogger):
    """Logger for a 7-element joint command (Float64MultiArray, no header: timestamp = reception)."""

    def __init__(self, base_path: str, name: str, column_prefix: str, num_joints: int = 7):
        headers = [f'{column_prefix}_{i}' for i in range(1, num_joints + 1)]
        super().__init__(f"{base_path}_{name}.csv", headers)


class JointReferenceLogger(BaseLogger):
    """Logger for the commander's integrated joint reference (/NS_1/q_des_state: q_d, q̇_d, q̈ in effort)."""

    def __init__(self, base_path: str, num_joints: int = 7):
        headers = []
        for i in range(1, num_joints + 1):
            headers.extend([f'q_d_{i}', f'dq_d_{i}', f'qdd_{i}'])
        super().__init__(f"{base_path}_q_des.csv", headers)


class CbfStatusLogger(BaseLogger):
    """Logger for /NS_1/cbf_status (Float64MultiArray, layout in cbf_safety_filter._publish_status)."""

    FIELDS = ['n_rows', 'slack', 'fault', 'n_violated', 'd_min', 'sp_min', 'v_cap_min',
              'v_closing_max', 'iso_stop', 'v_obs_cond', 'v_obs_hdot']

    def __init__(self, base_path: str):
        super().__init__(f"{base_path}_cbf_status.csv", self.FIELDS)


class TaskLogger(BaseLogger):
    """Logger for the task reference position and the task clock state."""

    def __init__(self, base_path: str):
        headers = ['ref_x', 'ref_y', 'ref_z', 's', 's_dot', 'ee_err']
        super().__init__(f"{base_path}_task.csv", headers)


class TaskEventLogger(BaseLogger):
    """Logger for task phase and cycle transitions (one row per change)."""

    def __init__(self, base_path: str):
        super().__init__(f"{base_path}_task_events.csv", ['event', 'value'])


class MpcDiagnosticsLogger(BaseLogger):
    """Logger for one row per MPC solve (franka_msgs/MpcDiagnostics on /mpc/diagnostics)."""

    FIELDS = ['status', 'solved', 'fallback', 'iterations', 'solve_time_ms', 'cost',
              'human_rows', 'human_binding', 'human_clearance', 'max_slack', 'accel_saturation']

    def __init__(self, base_path: str):
        super().__init__(f"{base_path}_mpc_diagnostics.csv", self.FIELDS)


class TcpPathLogger(BaseLogger):
    """Logger for a TCP path (MultiDOFJointTrajectory): one row per message, all its points.

    Point j is at timestamp + j * step_s. The columns depend on the number of points,
    so the header is written with the first message; later messages keep that count.
    """

    def __init__(self, base_path: str, name: str):
        super().__init__(f"{base_path}_{name}.csv", None)
        self.n_points = None

    def log_path(self, times: list, step_s: float, positions):
        if self.n_points is None:
            self.n_points = len(positions)
            self.write_header(['step_s'] + [f'{axis}{j}' for j in range(self.n_points) for axis in 'xyz'])
        row = times + [step_s]
        for j in range(self.n_points):
            row.extend(positions[j] if j < len(positions) else [float('nan')] * 3)
        self.log(row)


class ExperimentLoggerNode(Node):
    """ROS 2 Node that centralizes experiment data recording across perception, control, and robot state."""

    CONTROLLERS = ('human', 'nominal', 'cbf', 'mpc', 'mpc_cbf')

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

        session_time = time.strftime("%Y%m%d_%H%M%S")
        if not self.has_parameter('run_name'):
            self.declare_parameter('run_name', session_time)
        run_name = self.get_parameter('run_name').get_parameter_value().string_value
        # What drove the robot: the subfolder, and which groups of CSVs are written
        self.declare_parameter('controller', 'human')
        controller = str(self.get_parameter('controller').value)
        if controller not in self.CONTROLLERS:
            raise ValueError(f"controller must be one of {self.CONTROLLERS}, got '{controller}'")
        self.declare_parameter('log_human', True)
        log_human = bool(self.get_parameter('log_human').value)
        # Bags carry only the 30 Hz /NS_1/joint_states (republished with fresh stamps, unusable
        # for accelerations or jerk); on the robot log the 1 kHz /NS_1/franka/joint_states
        self.declare_parameter('robot_state_topic', '/NS_1/joint_states')
        robot_state_topic = self.get_parameter('robot_state_topic').value

        base_log_dir = Path('experiment_logs') / controller / run_name
        self.base_log_path = str(base_log_dir / "experiment")
        self.get_logger().info(
            f"Logging to {base_log_dir} (controller {controller}, human {'on' if log_human else 'off'}, "
            f"arms: {self.pose_side})")

        self.loggers: list[BaseLogger] = []
        self.subs = []
        # Best effort with a short queue: matches the 1 kHz best-effort feed and the reliable bag topic
        self.robot_logger = self.add_logger(RobotStateLogger(self.base_log_path))
        self.subscribe(JointState, robot_state_topic, self.robot_state_callback,
                       QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))

        if log_human:
            self.setup_human_logs()
        if controller != 'human':
            self.setup_command_logs()
        if 'cbf' in controller.split('_'):
            self.setup_cbf_logs()
        if 'mpc' in controller.split('_'):
            self.setup_mpc_logs()

        # Periodic flush so data survives a hard kill
        self.flush_timer = self.create_timer(1.0, self.flush_all)

    def add_logger(self, logger: BaseLogger) -> BaseLogger:
        self.loggers.append(logger)
        return logger

    def subscribe(self, msg_type, topic: str, callback, qos=10) -> None:
        self.subs.append(self.create_subscription(msg_type, topic, callback, qos))

    def setup_human_logs(self) -> None:
        """Tracker states, predictions, Kalman diagnostics per arm, and the human-robot distance."""
        self.human_raw_loggers, self.human_state_loggers = {}, {}
        self.human_pred_loggers, self.kf_diag_loggers = {}, {}
        for side in self.active_sides:
            # Side in the file name only when both arms are tracked
            side_path = f"{self.base_log_path}_{side}" if len(self.active_sides) > 1 else self.base_log_path
            self.human_raw_loggers[side] = self.add_logger(HumanRawLogger(side_path))
            self.human_state_loggers[side] = self.add_logger(HumanStateLogger(side_path))
            self.human_pred_loggers[side] = self.add_logger(HumanPredictionLogger(side_path))
            self.kf_diag_loggers[side] = self.add_logger(KalmanDiagnosticsLogger(side_path))

            prefix = f"{side}_" if self.pose_side == "both" else ""
            for msg_type, topic, callback in (
                (HumanArmState, '/human/raw_state', self.human_raw_callback),
                (HumanArmState, '/human/arm_state', self.human_state_callback),
                (HumanArmPrediction, '/human/arm_prediction', self.human_prediction_callback),
                (KalmanDiagnostics, '/human/kf_diagnostics', self.kf_diag_callback),
            ):
                self.subscribe(msg_type, format_topic(topic, prefix), partial(callback, side=side))

        distance_logger = self.add_logger(SafetyDistanceLogger(self.base_log_path, 'human'))
        self.subscribe(MultiLinkDistance, '/human/per_link_distances',
                       partial(self.min_distance_callback, logger=distance_logger),
                       QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))

    def setup_command_logs(self) -> None:
        """The torque chain's commands, the commander's joint reference and the task."""
        for topic, name, prefix in (('/NS_1/qddot_nom', 'qddot_nom', 'qddot_nom'),
                                    ('/NS_1/torque_cmd', 'torque_cmd', 'tau_ff')):
            logger = self.add_logger(JointVectorLogger(self.base_log_path, name, prefix))
            self.subscribe(Float64MultiArray, topic, partial(self.joint_vector_callback, logger=logger))
        self.q_des_logger = self.add_logger(JointReferenceLogger(self.base_log_path))
        self.subscribe(JointState, '/NS_1/q_des_state', self.q_des_callback)

        self.task_logger = self.add_logger(TaskLogger(self.base_log_path))
        self.task_event_logger = self.add_logger(TaskEventLogger(self.base_log_path))
        self.latest_task_clock = [float('nan')] * 3
        # Phase and cycle are latched by the commander
        latched_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.subscribe(String, '/task/phase', partial(self.task_event_callback, event='phase'), latched_qos)
        self.subscribe(UInt32, '/task/cycle', partial(self.task_event_callback, event='cycle'), latched_qos)
        self.subscribe(Float64MultiArray, '/task/clock', self.task_clock_callback)
        self.subscribe(PoseStamped, '/task/reference', self.task_reference_callback)

    def setup_cbf_logs(self) -> None:
        """The filtered command, the filter's status and the depth-obstacle distances."""
        logger = self.add_logger(JointVectorLogger(self.base_log_path, 'qddot_safe', 'qddot_safe'))
        self.subscribe(Float64MultiArray, '/NS_1/qddot_safe', partial(self.joint_vector_callback, logger=logger))
        self.cbf_status_logger = self.add_logger(CbfStatusLogger(self.base_log_path))
        self.subscribe(Float64MultiArray, '/NS_1/cbf_status', self.cbf_status_callback)
        obstacle_logger = self.add_logger(SafetyDistanceLogger(self.base_log_path, 'obstacle'))
        self.subscribe(MultiLinkDistance, '/cbf/per_link_distances',
                       partial(self.min_distance_callback, logger=obstacle_logger),
                       QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))

    def setup_mpc_logs(self) -> None:
        """One diagnostics row and one planned TCP path per MPC solve."""
        self.mpc_diag_logger = self.add_logger(MpcDiagnosticsLogger(self.base_log_path))
        self.mpc_plan_logger = self.add_logger(TcpPathLogger(self.base_log_path, 'mpc_plan'))
        self.subscribe(MpcDiagnostics, '/mpc/diagnostics', self.mpc_diag_callback)
        self.subscribe(MultiDOFJointTrajectory, '/mpc/plan', self.mpc_plan_callback)

    def flush_all(self):
        for logger in self.loggers:
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
        """Log KF innovations, covariance traces, NIS and gate decisions."""
        row = self.times(msg)
        for i in range(4):
            inn = msg.innovations[i]
            nis = msg.nis[i] if i < len(msg.nis) else float('nan')
            gated = int(msg.gated[i]) if i < len(msg.gated) else 0
            row.extend([inn.x, inn.y, inn.z, msg.p_traces[i], nis, gated])
        self.kf_diag_loggers[side].log(row)

    def now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def joint_vector_callback(self, msg: Float64MultiArray, logger: JointVectorLogger):
        """Log a headerless joint command; both time columns are the reception time."""
        now = self.now_s()
        logger.log([now, now] + list(msg.data))

    def task_event_callback(self, msg, event: str):
        now = self.now_s()
        self.task_event_logger.log([now, now, event, msg.data])

    def task_clock_callback(self, msg: Float64MultiArray):
        self.latest_task_clock = list(msg.data[:3])

    def task_reference_callback(self, msg: PoseStamped):
        p = msg.pose.position
        self.task_logger.log(self.times(msg) + [p.x, p.y, p.z] + self.latest_task_clock)

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

    def min_distance_callback(self, msg: MultiLinkDistance, logger: SafetyDistanceLogger):
        """Log the minimum clearance of one distance source and its closest points."""
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
        logger.log(row)

    def q_des_callback(self, msg: JointState):
        """Log the commander's joint reference: q_d, q̇_d and q̈ per joint."""
        row = self.times(msg)
        for i in range(len(msg.position)):
            row.extend([msg.position[i], msg.velocity[i], msg.effort[i]])
        self.q_des_logger.log(row)

    def cbf_status_callback(self, msg: Float64MultiArray):
        """Log the CBF status (headerless: both time columns are the reception time)."""
        now = self.now_s()
        n = len(CbfStatusLogger.FIELDS)
        values = list(msg.data[:n]) + [float('nan')] * (n - len(msg.data))
        self.cbf_status_logger.log([now, now] + values)

    def mpc_diag_callback(self, msg: MpcDiagnostics):
        """Log one MPC solve; the field order is MpcDiagnosticsLogger.FIELDS."""
        row = self.times(msg)
        for field in MpcDiagnosticsLogger.FIELDS:
            value = getattr(msg, field)
            row.append(int(value) if isinstance(value, bool) else value)
        self.mpc_diag_logger.log(row)

    def mpc_plan_callback(self, msg: MultiDOFJointTrajectory):
        """Log the TCP path the MPC planned at this solve."""
        times, positions, _ = ee_trajectory_arrays(msg)
        step = float(times[1] - times[0]) if len(times) > 1 else 0.0
        self.mpc_plan_logger.log_path(self.times(msg), step, positions.tolist())

    def destroy_node(self):
        """Safely close all open CSV file writers upon node exit."""
        self.get_logger().info("Closing all experiment log files...")
        for logger in self.loggers:
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