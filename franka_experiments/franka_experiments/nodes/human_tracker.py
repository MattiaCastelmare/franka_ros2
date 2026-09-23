#!/usr/bin/env python3
"""Human-arm tracking node.

RGB-D -> MediaPipe -> 3D keypoints in fr3_link0 -> Kalman filter
-> current arm state and constant-velocity prediction.

Robot geometry, distances, controllers and visualization are intentionally
kept outside this node.
"""

import os
import threading
import time
import cv2
import mediapipe as mp
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, PointCloud
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import Vector3

from franka_msgs.msg import HumanArmPrediction, HumanArmState, KalmanDiagnostics
from franka_experiments.utils.arm_kf import ArmKalmanFilter
from franka_experiments.utils.distance_utils import load_robot_config
from franka_experiments.utils.human_utils import (
    deproject, depth_patch_median, extract_arm_landmarks,
    measurement_age, quaternion_to_rotation, build_arm_state_msg,
    build_prediction_msg, build_2d_landmarks_msg, format_topic,
    check_engagement_start, check_engagement_loss
)


class HumanTracker(Node):
    KEYPOINT_NAMES = ("shoulder", "elbow", "wrist", "index")

    def __init__(self):
        super().__init__("human_tracker")

        # Config
        config_path = os.path.join(
            get_package_share_directory("franka_experiments"),
            "config",
            "human_params.yaml",
        )
        config = load_robot_config(config_path)["human_tracker"]
        self.base_frame = str(config["base_frame"])
        self.pose_side = str(config["pose_side"]).lower()
        if self.pose_side not in ("left", "right", "both"):
            raise ValueError("pose_side must be 'left' or 'right' or 'both'.")
        self.active_sides = ["left", "right"] if self.pose_side == "both" else [self.pose_side]

        # Inference and filtering parameters
        self.is_engaged = False
        self.first_visible_time = None
        self.first_lost_time = None
        self.engage_stability_s = float(config["engage_stability_s"])
        self.loss_stability_s = float(config["loss_stability_s"])
        self.inference_hz = max(1.0, float(config["inference_hz"]))
        self.visibility_threshold = float(config["visibility_threshold"])
        self.depth_patch_radius = int(config["depth_patch_radius"])
        self.min_depth_m = float(config["min_depth_m"])
        self.max_depth_m = float(config["max_depth_m"])
        self.max_state_age_s = float(config["max_state_age_s"])
        self.reset_after_s = max(
            self.max_state_age_s,
            float(config["reset_after_s"]),
        )
        self.max_speed_m_s = float(config["max_speed_m_s"])

        self.publish_prediction_enabled = bool(
            config["publish_prediction"]
        )
        self.prediction_dt = float(config["prediction_dt"])
        self.prediction_steps = int(config["prediction_steps"])

        # Camera state
        self.bridge = CvBridge()
        self.fx = self.fy = self.cx = self.cy = None
        self.camera_frame = None
        self.last_image = None
        self.last_depth = None
        self.image_header = None
        self.current_image_time = None
        self.last_update_time = None

        # MediaPipe runs in a separate worker so old camera frames never accumulate
        self.frame_lock = threading.Lock()
        self.pending_rgbd = None
        self.stop_event = threading.Event()

        # Static camera -> robot-base transform, cached after the first lookup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.R_camera_to_base = None
        self.t_camera_to_base = None

        # MediaPipe pose estimation
        model_complexity = int(
            np.clip(config["model_complexity"], 0, 2)
        )
        self.pose = mp.solutions.pose.Pose(
            static_image_mode=False,
            model_complexity=model_complexity,
            smooth_landmarks=True,
            enable_segmentation=False,
            min_detection_confidence=float(
                config["min_detection_confidence"]
            ),
            min_tracking_confidence=float(
                config["min_tracking_confidence"]
            ),
        )

        # Kalman filter for 3D keypoints
        self.kfs = {}
        self.last_valid_time = {}
        
        for side in self.active_sides:
            self.kfs[side] = ArmKalmanFilter(
                dt=float(config["kf_nominal_dt"]),
                process_accel_std=float(config["kf_process_accel_std"]),
                measurement_std=float(config["kf_measurement_std"]),
                visibility_threshold=self.visibility_threshold,
            )
            self.last_valid_time[side] = np.full(4, np.nan, dtype=float)

        color_topic = str(config["color_topic"])
        depth_topic = str(config["depth_topic"])
        camera_info_topic = str(config["camera_info_topic"])

        # Subscribers and Synchronizer
        self.color_sub = Subscriber(
            self, Image, color_topic, qos_profile=qos_profile_sensor_data
        )
        self.depth_sub = Subscriber(
            self, Image, depth_topic, qos_profile=qos_profile_sensor_data
        )
        self.rgbd_sync = ApproximateTimeSynchronizer(
            [self.color_sub, self.depth_sub],
            queue_size=max(1, int(config["sync_queue_size"])),
            slop=max(0.0, float(config["sync_slop_s"])),
        )
        self.rgbd_sync.registerCallback(self.rgbd_cb)

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            camera_info_topic,
            self.camera_info_cb,
            qos_profile_sensor_data,
        )

        # Dynamic Publishers
        latest_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        
        self.state_pubs = {}
        self.raw_state_pubs = {}
        self.prediction_pubs = {}
        self.landmarks_2d_pubs = {}
        self.kf_diag_pubs = {}

        for side in self.active_sides:
            prefix = f"{side}_" if self.pose_side == "both" else ""

            s_topic = format_topic(str(config["state_topic"]), prefix)
            rs_topic = format_topic(str(config["raw_state_topic"]), prefix)
            p_topic = format_topic(str(config["prediction_topic"]), prefix)
            l2d_topic = format_topic(str(config["landmarks_2d_topic"]), prefix)
            diag_topic = f"/human/{prefix}kf_diagnostics"

            self.state_pubs[side] = self.create_publisher(HumanArmState, s_topic, latest_qos)
            self.raw_state_pubs[side] = self.create_publisher(HumanArmState, rs_topic, latest_qos)
            self.prediction_pubs[side] = self.create_publisher(HumanArmPrediction, p_topic, latest_qos)
            self.landmarks_2d_pubs[side] = self.create_publisher(PointCloud, l2d_topic, qos_profile_sensor_data)
            self.kf_diag_pubs[side] = self.create_publisher(KalmanDiagnostics, diag_topic, latest_qos)

        self.worker_thread = threading.Thread(
            target=self.processing_loop,
            name="human_tracker_worker",
            daemon=True,
        )
        self.worker_thread.start()

        self.get_logger().info(
            f"HumanTracker ready: side={self.pose_side}, "
            f"base_frame={self.base_frame}, model_complexity={model_complexity}, "
            f"inference_hz={self.inference_hz:.1f}"
        )

    # ------------------------------------------------------------------
    # Camera input
    # ------------------------------------------------------------------
    def camera_info_cb(self, msg):
        if self.fx is not None:
            return
        if msg.k[0] <= 0.0 or msg.k[4] <= 0.0:
            self.get_logger().warn("Invalid camera intrinsics, skipping.")
            return

        self.fx = float(msg.k[0])
        self.fy = float(msg.k[4])
        self.cx = float(msg.k[2])
        self.cy = float(msg.k[5])
        self.camera_frame = msg.header.frame_id

        self.get_logger().info(
            f"Camera intrinsics: fx={self.fx:.1f}, fy={self.fy:.1f}, "
            f"cx={self.cx:.1f}, cy={self.cy:.1f}"
        )

    def rgbd_cb(self, color_msg, depth_msg):
        """Store only the newest synchronized RGB-D pair."""
        with self.frame_lock:
            self.pending_rgbd = (color_msg, depth_msg)

    def processing_loop(self):
        """Process the newest available pair at a limited inference rate."""
        period = 1.0 / self.inference_hz
        next_run = time.monotonic()

        while not self.stop_event.is_set():
            wait_s = max(0.0, next_run - time.monotonic())
            if self.stop_event.wait(wait_s):
                break

            with self.frame_lock:
                rgbd = self.pending_rgbd
                self.pending_rgbd = None

            if rgbd is not None:
                try:
                    self.process_rgbd(*rgbd)
                except Exception:
                    if not rclpy.ok():
                        break
                    raise

            next_run = max(next_run + period, time.monotonic())

    def process_rgbd(self, color_msg, depth_msg):
        """Convert and process one synchronized RGB-D pair."""
        try:
            self.last_image = self.bridge.imgmsg_to_cv2(
                color_msg, desired_encoding="bgr8")
            self.last_depth = self.bridge.imgmsg_to_cv2(
                depth_msg, desired_encoding="passthrough")
        except Exception as exc:
            self.get_logger().warn(
                f"Image conversion failed: {exc}", throttle_duration_sec=2.0)
            return

        # Aligned depth must have exactly the same image size as RGB
        if self.last_image.shape[:2] != self.last_depth.shape[:2]:
            self.get_logger().error(
                f"RGB and aligned depth have different resolutions: "
                f"RGB={self.last_image.shape[1]}x{self.last_image.shape[0]}, "
                f"depth={self.last_depth.shape[1]}x{self.last_depth.shape[0]}",
                throttle_duration_sec=2.0,
            )
            return

        self.image_header = color_msg.header
        self.current_image_time = rclpy.time.Time.from_msg(
            color_msg.header.stamp
        )
        self.update_state()

    # ------------------------------------------------------------------
    # Static TF
    # ------------------------------------------------------------------
    def get_camera_to_base_transform(self):
        if self.R_camera_to_base is not None:
            return self.R_camera_to_base, self.t_camera_to_base
        if self.camera_frame is None:
            return None

        try:
            tf_msg = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.camera_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.1),
            )
        except Exception as exc:
            self.get_logger().warn(
                f"Camera-to-base TF unavailable: {exc}",
                throttle_duration_sec=2.0,
            )
            return None

        q = tf_msg.transform.rotation
        self.R_camera_to_base = quaternion_to_rotation(
            q.x, q.y, q.z, q.w
        )
        self.t_camera_to_base = np.array(
            [
                tf_msg.transform.translation.x,
                tf_msg.transform.translation.y,
                tf_msg.transform.translation.z,
            ],
            dtype=float,
        )
        self.get_logger().info(
            f"Cached TF: {self.camera_frame} -> {self.base_frame}"
        )
        return self.R_camera_to_base, self.t_camera_to_base

    def compute_dt(self, current_time):
        if self.last_update_time is None:
            return list(self.kfs.values())[0].dt

        dt = current_time - self.last_update_time
        if dt <= 0.0:
            for side in self.active_sides:
                self.kfs[side].reset()
                self.last_valid_time[side][:] = np.nan
            return list(self.kfs.values())[0].dt
        return float(np.clip(dt, 1e-3, 0.2))

    # ------------------------------------------------------------------
    # Kalman update and publications
    # ------------------------------------------------------------------
    def update_state(self):
        if self.fx is None:
            return

        # Run MediaPipe pose estimation on the latest RGB image and extract 2D keypoints
        image_rgb = cv2.cvtColor(self.last_image, cv2.COLOR_BGR2RGB)
        result = self.pose.process(image_rgb)

        # Compute current time for dt and engage logic
        current_time = self.current_image_time.nanoseconds * 1e-9

        # Landmarks and Visibility
        current_visibilities = {}
        extracted_landmarks = {}

        for side in self.active_sides:
            landmarks = extract_arm_landmarks(
                result.pose_landmarks, self.last_image.shape, side, self.KEYPOINT_NAMES
            )
            extracted_landmarks[side] = landmarks
            
            vis = np.zeros(4, dtype=float)
            if landmarks is not None:
                for i, name in enumerate(self.KEYPOINT_NAMES):
                    vis[i] = landmarks[name]["visibility"]
            current_visibilities[side] = vis

            # Published for every processed frame (also when not engaged)
            self.landmarks_2d_pubs[side].publish(
                build_2d_landmarks_msg(landmarks, self.KEYPOINT_NAMES, self.image_header)
            )

        # Engage Logic
        if not self.is_engaged:
            if check_engagement_start(self.active_sides, self.visibility_threshold, current_visibilities):
                if getattr(self, 'first_visible_time', None) is None:
                    self.first_visible_time = current_time
                elif (current_time - self.first_visible_time) >= getattr(self, 'engage_stability_s', 0.3):
                    self.is_engaged = True
                    self.first_visible_time = None
                    self.get_logger().info(
                        "Human ENGAGED: at least one arm is stably visible. Start tracking.", 
                        throttle_duration_sec=1.0
                    )
            else:
                self.first_visible_time = None

            if not self.is_engaged:
                return

        # Compute the time delta since the last update
        dt = self.compute_dt(current_time)

        # Compute 3D positions of the keypoints in the robot base frame
        camera_tf = self.get_camera_to_base_transform()
        speed_logs = []
        current_validities = {}

        # Iterate on each active side (left/right) and process the keypoints
        for side in self.active_sides:
            landmarks = extracted_landmarks[side]
            visibilities = current_visibilities[side]

            log_prefix = f"[{side.upper()}] " if len(self.active_sides) > 1 else ""
            positions = np.full((4, 3), np.nan, dtype=float)
            depths = np.zeros(4, dtype=float)

            if landmarks is not None and camera_tf is not None:
                rotation, translation = camera_tf
                valid_depths = []
                landmark_pixels = {}

                for i, name in enumerate(self.KEYPOINT_NAMES):
                    # Skip keypoints that are not visible enough or have no valid depth
                    visibility = visibilities[i]
                    if visibility < self.visibility_threshold:
                        continue

                    landmark = landmarks[name]
                    u = int(round(landmark["x_px"]))
                    v = int(round(landmark["y_px"]))
                    landmark_pixels[i] = (u, v)

                    # Get the median depth in a small patch around the keypoint, ignoring invalid pixels
                    depth_m = depth_patch_median(
                        self.last_depth, u, v, self.depth_patch_radius,
                        self.min_depth_m, self.max_depth_m,
                    )
                    if depth_m is not None:
                        depths[i] = depth_m
                        valid_depths.append(depth_m)

                fallback_depth = None
                if len(valid_depths) >= 2:
                    ref_median = float(np.median(valid_depths))
                    consistent = [d for d in valid_depths if abs(d - ref_median) <= 0.20]
                    if len(consistent) >= 2:
                        fallback_depth = float(np.median(consistent))

                for i in range(4):
                    if i not in landmark_pixels:
                        continue
                    d = depths[i] if depths[i] > 0.0 else fallback_depth
                    if d is None:
                        continue
                    u, v = landmark_pixels[i]

                    # Deproject the 2D pixel to 3D in the camera frame and transform to the robot base frame
                    point_camera = deproject(u, v, d, self.fx, self.fy, self.cx, self.cy)
                    point_base = rotation @ point_camera + translation
                    if np.all(np.isfinite(point_base)):
                        positions[i] = point_base

            # Publish raw data for KF post-comparison
            raw_msg = build_arm_state_msg(
                positions=positions, velocities=np.zeros((4, 3)), visibilities=visibilities, 
                measured=np.ones(4, dtype=bool), keypoint_valid=np.ones(4, dtype=bool), 
                age=np.zeros(4), header=self.image_header, base_frame=self.base_frame
            )
            self.raw_state_pubs[side].publish(raw_msg)

            # --- Kalman Filter Update ---
            filtered_pos, filtered_vel, measured = self.kfs[side].step(
                positions=positions, visibilities=visibilities, depths=depths, dt=dt
            )

            # Update the last valid time for each keypoint
            self.last_valid_time[side][measured] = current_time
            age = measurement_age(self.last_valid_time[side], current_time)

            # Reset any keypoint whose latest valid measurement is too old
            for i in range(4):
                if self.kfs[side].initialized[i] and age[i] > self.reset_after_s:
                    self.kfs[side].reset(i)
                    self.last_valid_time[side][i] = np.nan
                    filtered_pos[i] = np.nan
                    filtered_vel[i] = np.nan
                    age[i] = -1.0

            # Determine which keypoints are valid for publication
            keypoint_valid = (
                self.kfs[side].initialized & np.all(np.isfinite(filtered_pos), axis=1)
                & (age >= 0.0) & (age <= self.max_state_age_s)
            )
            
            # Save validity for the disengage logic
            current_validities[side] = keypoint_valid

            # Publish KF Diagnostics
            innovations, p_traces = self.kfs[side].get_diagnostics()
            diag_msg = KalmanDiagnostics()
            diag_msg.header = self.image_header
            for i in range(4):
                vec = Vector3()
                vec.x, vec.y, vec.z = float(innovations[i, 0]), float(innovations[i, 1]), float(innovations[i, 2])
                diag_msg.innovations.append(vec)
                diag_msg.p_traces.append(float(p_traces[i]))
            self.kf_diag_pubs[side].publish(diag_msg)

            # Sanity Check: discard any keypoint whose speed exceeds a reasonable threshold
            speed = np.linalg.norm(filtered_vel, axis=1)
            for i in range(4):
                if keypoint_valid[i] and speed[i] > self.max_speed_m_s:
                    self.get_logger().warn(
                        f"{log_prefix}Anomalous velocity for {self.KEYPOINT_NAMES[i]}: {speed[i]:.2f} m/s. Discard data."
                    )
                    keypoint_valid[i] = False
                    self.kfs[side].reset(i)

            # Publish Filtered State
            state_msg = build_arm_state_msg(
                positions=filtered_pos, velocities=filtered_vel, visibilities=visibilities, measured=measured, 
                keypoint_valid=keypoint_valid, age=age, header=self.image_header, base_frame=self.base_frame
            )
            self.state_pubs[side].publish(state_msg)

            # Publish Constant-Velocity Prediction if enabled
            if self.publish_prediction_enabled:
                pred_msg = build_prediction_msg(
                    filtered_pos, filtered_vel, keypoint_valid, age, self.prediction_dt, 
                    self.prediction_steps, self.image_header, self.base_frame
                )
                self.prediction_pubs[side].publish(pred_msg)

            # Log the KF speed for each keypoint
            values = [speed[i] if keypoint_valid[i] else np.nan for i in range(4)]
            speed_logs.append(
                f"{log_prefix}SH={values[0]:.3f}, EL={values[1]:.3f}, "
                f"WR={values[2]:.3f}, HA={values[3]:.3f}"
            )

        self.last_update_time = current_time

        if speed_logs:
            self.get_logger().info(
                " | ".join(speed_logs),
                throttle_duration_sec=1.0,
            )

        # Disengage Logic
        if self.is_engaged:
            if check_engagement_loss(self.active_sides, current_validities):
                if getattr(self, 'first_lost_time', None) is None:
                    self.first_lost_time = current_time
                elif (current_time - self.first_lost_time) >= getattr(self, 'loss_stability_s', 0.5):
                    self.is_engaged = False
                    self.first_lost_time = None
                    self.get_logger().warn(
                        "Human LOST: all keypoints are occluded or expired. DISENGAGE.", 
                        throttle_duration_sec=1.0
                    )
                    
                    # Total reset of filters
                    for side in self.active_sides:
                        self.kfs[side].reset()
                        self.last_valid_time[side][:] = np.nan
            else:
                self.first_lost_time = None

    def stop_worker(self):
        self.stop_event.set()
        self.worker_thread.join()


def main(args=None):
    rclpy.init(args=args)
    node = HumanTracker()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.stop_worker()
        node.pose.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()