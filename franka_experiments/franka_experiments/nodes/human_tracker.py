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
from geometry_msgs.msg import Point, Point32, Vector3
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, ChannelFloat32, Image, PointCloud
from tf2_ros import Buffer, TransformListener

from franka_experiments.utils.arm_kf import ArmKalmanFilter
from franka_experiments.utils.distance_utils import load_robot_config
from franka_experiments.utils.human_utils import (
    deproject,
    depth_patch_median,
    extract_arm_landmarks,
    measurement_age,
    quaternion_to_rotation,
    to_point,
    to_vector,
)
from franka_msgs.msg import HumanArmPrediction, HumanArmState


class HumanTracker(Node):
    KEYPOINT_NAMES = ("shoulder", "elbow", "wrist", "index")

    def __init__(self):
        config_path = os.path.join(
            get_package_share_directory("franka_experiments"),
            "config",
            "human_params.yaml",
        )
        full_config = load_robot_config(config_path)
        config = full_config["human_tracker"]
        use_sim_time = bool(
            full_config.get("common", {}).get("use_sim_time", True)
        )

        super().__init__(
            "human_tracker",
            parameter_overrides=[
                Parameter(
                    "use_sim_time",
                    Parameter.Type.BOOL,
                    use_sim_time,
                )
            ],
            automatically_declare_parameters_from_overrides=True,
        )

        self.base_frame = str(config["base_frame"])
        self.pose_side = str(config["pose_side"]).lower()
        if self.pose_side not in ("left", "right"):
            raise ValueError("pose_side must be 'left' or 'right'.")

        self.inference_hz = max(1.0, float(config["inference_hz"]))
        self.inference_scale = float(config["inference_scale"])
        self.inference_scale = float(np.clip(self.inference_scale, 0.1, 1.0))
        self.visibility_threshold = float(config["visibility_threshold"])
        self.depth_patch_radius = int(config["depth_patch_radius"])
        self.min_depth_m = float(config["min_depth_m"])
        self.max_depth_m = float(config["max_depth_m"])
        self.max_state_age_s = float(config["max_state_age_s"])
        self.reset_after_s = max(
            self.max_state_age_s,
            float(config["reset_after_s"]),
        )

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
        self.latest_arm_landmarks = None
        self.last_update_time = None

        # Keep only the newest synchronized RGB-D pair. MediaPipe runs in a
        # separate worker so old camera frames never accumulate
        self.frame_lock = threading.Lock()
        self.pending_rgbd = None
        self.stop_event = threading.Event()

        # Static camera -> robot-base transform, cached after the first lookup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.R_camera_to_base = None
        self.t_camera_to_base = None

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

        self.arm_kf = ArmKalmanFilter(
            dt=float(config["kf_nominal_dt"]),
            process_accel_std=float(config["kf_process_accel_std"]),
            measurement_std=float(config["kf_measurement_std"]),
            visibility_threshold=self.visibility_threshold,
        )
        self.last_valid_time = np.full(4, np.nan, dtype=float)

        color_topic = str(config["color_topic"])
        depth_topic = str(config["depth_topic"])
        camera_info_topic = str(config["camera_info_topic"])

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

        latest_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.state_pub = self.create_publisher(
            HumanArmState,
            str(config["state_topic"]),
            latest_qos,
        )
        self.prediction_pub = self.create_publisher(
            HumanArmPrediction,
            str(config["prediction_topic"]),
            latest_qos,
        )
        self.landmarks_2d_pub = self.create_publisher(
            PointCloud,
            str(config["landmarks_2d_topic"]),
            qos_profile_sensor_data,
        )

        self.worker_thread = threading.Thread(
            target=self.processing_loop,
            name="human_tracker_worker",
            daemon=True,
        )
        self.worker_thread.start()

        self.get_logger().info(
            f"HumanTracker ready: side={self.pose_side}, "
            f"base_frame={self.base_frame}, model_complexity={model_complexity}, "
            f"inference_hz={self.inference_hz:.1f}, "
            f"inference_scale={self.inference_scale:.2f}"
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
                self.process_rgbd(*rgbd)

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
    # MediaPipe and depth
    # ------------------------------------------------------------------
    def extract_keypoints(self):
        inference_image = self.last_image
        if self.inference_scale < 1.0:
            inference_image = cv2.resize(
                self.last_image,
                dsize=None,
                fx=self.inference_scale,
                fy=self.inference_scale,
                interpolation=cv2.INTER_AREA,
            )

        image_rgb = cv2.cvtColor(inference_image, cv2.COLOR_BGR2RGB)
        result = self.pose.process(image_rgb)
        self.latest_arm_landmarks = extract_arm_landmarks(
            result.pose_landmarks,
            self.last_image.shape,
            self.pose_side,
            self.KEYPOINT_NAMES,
        )

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

    # ------------------------------------------------------------------
    # Kalman update and publications
    # ------------------------------------------------------------------
    def update_state(self):
        if self.fx is None:
            return

        self.extract_keypoints()
        self.publish_2d_landmarks()

        current_time = self.current_image_time.nanoseconds * 1e-9
        dt = self.compute_dt(current_time)

        positions = np.full((4, 3), np.nan, dtype=float)
        visibilities = np.zeros(4, dtype=float)
        depths = np.zeros(4, dtype=float)
        camera_tf = self.get_camera_to_base_transform()

        if self.latest_arm_landmarks is not None and camera_tf is not None:
            rotation, translation = camera_tf

            for i, name in enumerate(self.KEYPOINT_NAMES):
                landmark = self.latest_arm_landmarks[name]
                visibility = landmark["visibility"]
                visibilities[i] = visibility
                if visibility < self.visibility_threshold:
                    continue

                u = int(round(landmark["x_px"]))
                v = int(round(landmark["y_px"]))

                depth_m = depth_patch_median(
                    self.last_depth,
                    u,
                    v,
                    self.depth_patch_radius,
                    self.min_depth_m,
                    self.max_depth_m,
                )
                if depth_m is None:
                    continue

                point_camera = deproject(
                    u, v, depth_m, self.fx, self.fy, self.cx, self.cy
                )
                point_base = rotation @ point_camera + translation
                if np.all(np.isfinite(point_base)):
                    positions[i] = point_base
                    depths[i] = depth_m

        filtered_pos, filtered_vel, measured = self.arm_kf.step(
            positions=positions,
            visibilities=visibilities,
            depths=depths,
            dt=dt,
        )

        self.last_valid_time[measured] = current_time
        age = measurement_age(self.last_valid_time, current_time)

        for i in range(4):
            if self.arm_kf.initialized[i] and age[i] > self.reset_after_s:
                self.arm_kf.reset(i)
                self.last_valid_time[i] = np.nan
                filtered_pos[i] = np.nan
                filtered_vel[i] = np.nan
                age[i] = -1.0

        keypoint_valid = (
            self.arm_kf.initialized
            & np.all(np.isfinite(filtered_pos), axis=1)
            & (age >= 0.0)
            & (age <= self.max_state_age_s)
        )

        self.publish_state(
            filtered_pos,
            filtered_vel,
            visibilities,
            measured,
            keypoint_valid,
            age,
        )

        if self.publish_prediction_enabled:
            self.publish_prediction(
                filtered_pos,
                filtered_vel,
                keypoint_valid,
                age,
            )

        self.last_update_time = current_time

        speed = np.linalg.norm(filtered_vel, axis=1)
        values = [speed[i] if keypoint_valid[i] else np.nan for i in range(4)]
        self.get_logger().info(
            "KF speed [m/s]: "
            f"shoulder={values[0]:.3f}, elbow={values[1]:.3f}, "
            f"wrist={values[2]:.3f}, hand={values[3]:.3f}",
            throttle_duration_sec=1.0,
        )

    def compute_dt(self, current_time):
        if self.last_update_time is None:
            return self.arm_kf.dt

        dt = current_time - self.last_update_time
        if dt <= 0.0:
            self.arm_kf.reset()
            self.last_valid_time[:] = np.nan
            return self.arm_kf.dt
        return float(np.clip(dt, 1e-3, 0.2))

    def publish_state(
        self,
        positions,
        velocities,
        visibilities,
        measured,
        keypoint_valid,
        age,
    ):
        msg = HumanArmState()
        msg.header = self.image_header
        msg.header.frame_id = self.base_frame

        points = [Point() for _ in range(4)]
        vectors = [Vector3() for _ in range(4)]
        for i in range(4):
            if keypoint_valid[i]:
                points[i] = to_point(positions[i])
                vectors[i] = to_vector(velocities[i])

        msg.shoulder, msg.elbow, msg.wrist, msg.hand = points
        (
            msg.shoulder_vel,
            msg.elbow_vel,
            msg.wrist_vel,
            msg.hand_vel,
        ) = vectors

        msg.visibility = visibilities.astype(float).tolist()
        msg.measured = measured.astype(bool).tolist()
        msg.keypoint_valid = keypoint_valid.astype(bool).tolist()
        msg.measurement_age = age.astype(float).tolist()
        msg.confidence = float(np.mean(visibilities))
        msg.valid = bool(np.all(keypoint_valid))
        msg.occluded = bool(not np.any(measured))
        self.state_pub.publish(msg)

    def publish_prediction(self, positions, velocities, valid, age):
        msg = HumanArmPrediction()
        msg.header = self.image_header
        msg.header.frame_id = self.base_frame
        msg.step_dt = float(self.prediction_dt)
        msg.num_steps = int(self.prediction_steps)
        msg.keypoint_valid = valid.astype(bool).tolist()
        msg.measurement_age = age.astype(float).tolist()

        for step in range(1, self.prediction_steps + 1):
            future_time = step * self.prediction_dt
            points = [Point() for _ in range(4)]

            for i in range(4):
                if valid[i]:
                    points[i] = to_point(
                        positions[i] + future_time * velocities[i]
                    )

            msg.shoulder.append(points[0])
            msg.elbow.append(points[1])
            msg.wrist.append(points[2])
            msg.hand.append(points[3])

        self.prediction_pub.publish(msg)

    def publish_2d_landmarks(self):
        msg = PointCloud()
        msg.header = self.image_header
        visibility = ChannelFloat32()
        visibility.name = "visibility"

        if self.latest_arm_landmarks is not None:
            for name in self.KEYPOINT_NAMES:
                landmark = self.latest_arm_landmarks[name]
                point = Point32()
                point.x = landmark["x_px"]
                point.y = landmark["y_px"]
                msg.points.append(point)
                visibility.values.append(landmark["visibility"])

        msg.channels.append(visibility)
        self.landmarks_2d_pub.publish(msg)

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