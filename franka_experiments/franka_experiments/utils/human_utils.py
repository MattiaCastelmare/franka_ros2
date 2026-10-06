"""Shared helper functions for human tracking and visualization.
"""

import os
import xacro
import numpy as np
from typing import Any
from geometry_msgs.msg import Point, Vector3, Point32
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud, ChannelFloat32
from franka_msgs.msg import HumanArmState, HumanArmPrediction, LinkDistance
from ament_index_python.packages import get_package_share_directory
from franka_experiments.utils.ros_setup import build_reduced_pinocchio_model_from_urdf


def init_pinocchio_from_xacro(node: Any, with_hand: bool = True) -> tuple[bool, Any, Any]:
    """
    Initialize the reduced Pinocchio model directly from the Xacro file,
    bypassing ROS 2 services for robustness with rosbag playback.
    """
    pkg_path = get_package_share_directory('franka_description')
    
    # Try common paths for the Franka Xacro file
    xacro_path = os.path.join(pkg_path, 'robots', 'fr3', 'fr3.urdf.xacro')
    if not os.path.exists(xacro_path):
        xacro_path = os.path.join(pkg_path, 'urdf', 'fr3.urdf.xacro')
        
    if not os.path.exists(xacro_path):
        if node is not None:
            node.get_logger().error(f"Could not find Xacro file at {xacro_path}")
        return False, None, None

    try:
        # Process Xacro dynamically (injecting the hand argument)
        mappings = {'hand': 'true' if with_hand else 'false'}
        doc = xacro.process_file(xacro_path, mappings=mappings)
        urdf_string = doc.toprettyxml(indent='  ')
        
        # Build the model using the existing utility
        model, data = build_reduced_pinocchio_model_from_urdf(urdf_string)
        
        if node is not None:
            node.get_logger().info("✓ Pinocchio model successfully initialized via Xacro.")
            
        return True, model, data
        
    except Exception as e:
        if node is not None:
            node.get_logger().error(f"Pinocchio initialization failed: {e}")
        return False, None, None


def format_topic(topic, prefix: str = None) -> str:
    if not prefix: return topic
    parts = topic.split('/')
    parts[-1] = f"{prefix}{parts[-1]}"
    return '/'.join(parts)


def get_side(active_sides: list, closest_capsule_name: str = "") -> str:
    side_prefix = ""
    if len(active_sides) > 1 and closest_capsule_name:
        if "left_" in closest_capsule_name:
            side_prefix = "[LEFT] "
        elif "right_" in closest_capsule_name:
            side_prefix = "[RIGHT] "
    return side_prefix


def check_engagement_start(active_sides, visibility_threshold, visibilities_dict: dict) -> bool:
    """
    Checks if at least one active arm has all 4 keypoints above the visibility threshold.
    The other arm, if any, starts being tracked as soon as its keypoints appear.
    """
    for side in active_sides:
        if np.all(visibilities_dict[side] >= visibility_threshold):
            return True
    return False


def check_engagement_loss(active_sides, validities_dict: dict) -> bool:
    """
    Checks if ALL keypoints are lost (age > reset_after_s or max_state_age_s).
    Returns True only if there is no valid keypoint in any active side.
    """
    for side in active_sides:
        if np.any(validities_dict[side]):
            return False
    return True


def quaternion_to_rotation(qx, qy, qz, qw):
    """Convert a quaternion into the same 3x3 rotation matrix used before."""
    return np.array(
        [
            [
                1 - 2 * (qy*qy + qz*qz),
                2 * (qx*qy - qz*qw),
                2 * (qx*qz + qy*qw),
            ],
            [
                2 * (qx*qy + qz*qw),
                1 - 2 * (qx*qx + qz*qz),
                2 * (qy*qz - qx*qw),
            ],
            [
                2 * (qx*qz - qy*qw),
                2 * (qy*qz + qx*qw),
                1 - 2 * (qx*qx + qy*qy),
            ],
        ],
        dtype=float,
    )


def measurement_age(last_valid_time, current_time):
    """Compute the age of the latest valid measurement for each keypoint."""
    age = np.full(4, -1.0, dtype=float)
    known = np.isfinite(last_valid_time)
    age[known] = current_time - last_valid_time[known]
    return age


def extract_human_keypoints(state_msg: HumanArmState):
    """
    Extracts 3D positions, 3D velocities, and validity masks from the state message.
    Returns:
        positions (np.ndarray): Shape (4, 3) XYZ positions.
        velocities (np.ndarray): Shape (4, 3) XYZ velocities.
        valid (np.ndarray): Shape (4,) boolean validity mask.
    """
    positions = np.array([
        [state_msg.shoulder.x, state_msg.shoulder.y, state_msg.shoulder.z],
        [state_msg.elbow.x, state_msg.elbow.y, state_msg.elbow.z],
        [state_msg.wrist.x, state_msg.wrist.y, state_msg.wrist.z],
        [state_msg.hand.x, state_msg.hand.y, state_msg.hand.z]
    ], dtype=float)
    
    velocities = np.array([
        [state_msg.shoulder_vel.x, state_msg.shoulder_vel.y, state_msg.shoulder_vel.z],
        [state_msg.elbow_vel.x, state_msg.elbow_vel.y, state_msg.elbow_vel.z],
        [state_msg.wrist_vel.x, state_msg.wrist_vel.y, state_msg.wrist_vel.z],
        [state_msg.hand_vel.x, state_msg.hand_vel.y, state_msg.hand_vel.z]
    ], dtype=float)
    
    valid = np.array(state_msg.keypoint_valid, dtype=bool)
    
    return positions, velocities, valid


def extract_human_covariances(state_msg: HumanArmState):
    """
    Extracts the Kalman covariance blocks and update counts from the state message.
    Returns:
        P_pp, P_vv, P_pv (np.ndarray): Shape (4, 3, 3) each, base frame.
        frames_seen (np.ndarray): Shape (4,) accepted measurement updates per keypoint.
    """
    P_pp = np.asarray(state_msg.position_covariance, dtype=float).reshape(4, 3, 3)
    P_vv = np.asarray(state_msg.velocity_covariance, dtype=float).reshape(4, 3, 3)
    P_pv = np.asarray(state_msg.position_velocity_covariance, dtype=float).reshape(4, 3, 3)
    frames_seen = np.asarray(state_msg.frames_seen, dtype=np.int64)
    return P_pp, P_vv, P_pv, frames_seen


def fill_track_fields(ld: LinkDistance, capsule: dict, alpha: float) -> None:
    """Velocity, covariances and evidence of the human point closest to this control point."""
    i0, i1 = capsule['indices']
    w0, w1 = 1.0 - alpha, alpha
    velocities = capsule['velocities']
    ld.obstacle_velocity = to_vector(w0 * velocities[i0] + w1 * velocities[i1])

    # Independent filters: the interpolated point's covariance has no cross terms
    P_pp, P_vv, P_pv = capsule['covariances']
    for field, P in (
        ('position_covariance', P_pp),
        ('velocity_covariance', P_vv),
        ('position_velocity_covariance', P_pv),
    ):
        setattr(ld, field, (w0**2 * P[i0] + w1**2 * P[i1]).reshape(-1).tolist())

    # Constant-velocity filter: no acceleration estimate, obstacle_acceleration stays zero
    ld.frames_seen = int(min(capsule['frames_seen'][i0], capsule['frames_seen'][i1]))
    ld.track_id = int(capsule['track_id'])


def define_control_points(transforms: dict, robot_cfg: dict, distance_cfg: dict) -> list:
    """Build the ordered list of control points along the robot kinematic chain."""
    ee_link = robot_cfg.get('ee_link', 'fr3_link8')
    ee_tip_axis = distance_cfg['ee_tip_axis']
    ee_tip_offset = distance_cfg['ee_tip_offset']
    control_points = []

    for seg in robot_cfg.get('segments', []):
        n_cp = int(seg.get('control_points', 0))
        if n_cp <= 0:
            continue

        start_link = seg['start_link']
        end_link = seg['end_link']
        if start_link not in transforms or end_link not in transforms:
            continue

        _, p0 = transforms[start_link]
        R_end, p1 = transforms[end_link]
        radius = float(seg.get('radius', 0.05))

        ts = [(k + 1) / (n_cp + 1) for k in range(n_cp)]
        if end_link == ee_link:
            ts = [1.0] if n_cp == 1 else [(k + 1) / n_cp for k in range(n_cp)]

        for k, t in enumerate(ts):
            p = p0 + t * (p1 - p0)
            if end_link == ee_link and np.isclose(t, 1.0):
                p = p1 + ee_tip_offset * R_end[:, ee_tip_axis]

            control_points.append({
                'name': f"{start_link}_cp_{k}",
                'position': p,
                'radius': radius,
                'source_capsule': start_link
            })

    return control_points


def to_point(values):
    """Convert a 3-vector into geometry_msgs/Point."""
    msg = Point()
    msg.x, msg.y, msg.z = map(float, values)
    return msg


def to_vector(values):
    """Convert a 3-vector into geometry_msgs/Vector3."""
    msg = Vector3()
    msg.x, msg.y, msg.z = map(float, values)
    return msg


def restamp(header, frame_id: str) -> Header:
    """New header with the same stamp: never mutate the shared camera header."""
    return Header(stamp=header.stamp, frame_id=frame_id)


def stamp_to_ns(msg):
    """Convert a ROS message header timestamp to integer nanoseconds."""
    stamp = msg.header.stamp
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def build_arm_state_msg(
    positions, velocities, visibilities, measured, keypoint_valid, age, header, base_frame,
    covariances=None, frames_seen=None,
) -> HumanArmState:
    """Builds the HumanArmState message.

    ``covariances`` is the (P_pp, P_vv, P_pv) tuple of ArmKalmanFilter.get_covariances();
    blocks of invalid keypoints are left at zero, as their positions are.
    """
    msg = HumanArmState()
    msg.header = restamp(header, base_frame)

    cov_fields = ("position_covariance", "velocity_covariance", "position_velocity_covariance")
    cov_values = [np.zeros((4, 3, 3)) for _ in cov_fields]

    fields = ["shoulder", "elbow", "wrist", "hand"]
    for i, field in enumerate(fields):
        if keypoint_valid[i]:
            setattr(msg, field, to_point(positions[i]))
            setattr(msg, f"{field}_vel", to_vector(velocities[i]))
            if covariances is not None:
                for values, block in zip(cov_values, covariances):
                    values[i] = block[i]

    for name, values in zip(cov_fields, cov_values):
        setattr(msg, name, values.reshape(-1).astype(float).tolist())
    if frames_seen is not None:
        msg.frames_seen = np.where(keypoint_valid, frames_seen, 0).astype(int).tolist()

    msg.visibility = visibilities.astype(float).tolist()
    msg.measured = measured.astype(bool).tolist()
    msg.keypoint_valid = keypoint_valid.astype(bool).tolist()
    msg.measurement_age = age.astype(float).tolist()
    msg.confidence = float(np.mean(visibilities))
    msg.valid = bool(np.all(keypoint_valid))
    msg.occluded = bool(not np.any(measured))
    return msg


def build_disengaged_state_msg(visibilities, header, base_frame) -> HumanArmState:
    """Heartbeat while no arm is engaged: a state with every keypoint invalid.
    human_distance turns it into an empty MultiLinkDistance, which the controller reads as
    "nothing near"; silence would read as a dead perception.
    """
    return build_arm_state_msg(
        positions=np.full((4, 3), np.nan), velocities=np.full((4, 3), np.nan),
        visibilities=visibilities, measured=np.zeros(4, dtype=bool),
        keypoint_valid=np.zeros(4, dtype=bool), age=np.full(4, -1.0),
        header=header, base_frame=base_frame,
    )


def predict_future_positions(positions, velocities, step_dt, num_steps):
    """Predict future positions using a constant-velocity model."""
    future_positions = []
    for step in range(1, num_steps + 1):
        future_time = step * step_dt
        future_positions.append(
            positions + future_time * velocities
        )
    return np.array(future_positions)


def build_prediction_msg(
    positions, velocities, valid, age, step_dt, num_steps, header, base_frame
) -> HumanArmPrediction:
    """Builds the HumanArmPrediction message."""
    msg = HumanArmPrediction()
    msg.header = restamp(header, base_frame)
    msg.step_dt = float(step_dt)
    msg.num_steps = int(num_steps)
    msg.keypoint_valid = valid.astype(bool).tolist()
    msg.measurement_age = age.astype(float).tolist()

    fields = ['shoulder', 'elbow', 'wrist', 'hand']

    for i, field in enumerate(fields):
        if valid[i]:
            future_positions = predict_future_positions(
                positions[i],
                velocities[i],
                step_dt,
                num_steps
            )
            for pos in future_positions:
                getattr(msg, field).append(to_point(pos))
        else:
            for _ in range(num_steps):
                getattr(msg, field).append(Point())
    return msg


def build_2d_landmarks_msg(landmarks, keypoint_names, header) -> PointCloud:
    """Builds the PointCloud message for the 2D landmarks extracted by MediaPipe."""
    msg = PointCloud()
    msg.header = header
    visibility = ChannelFloat32()
    visibility.name = "visibility"

    if landmarks is not None:
        for name in keypoint_names:
            landmark = landmarks[name]
            point = Point32()
            point.x = landmark["x_px"]
            point.y = landmark["y_px"]
            msg.points.append(point)
            visibility.values.append(landmark["visibility"])

    msg.channels.append(visibility)
    return msg