#!/usr/bin/env python3

import time
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Vector3

from franka_msgs.msg import (
    HandTrackingFiltered,
    HandState,
)


class HandStateEstimator(Node):

    def __init__(self):
        super().__init__('hand_state_estimator')

        self.declare_parameter(
            'min_pair_distance',
            0.002,
        )
        self.declare_parameter(
            'max_pair_distance',
            0.20,
        )
        self.declare_parameter(
            'min_axis_sin',
            0.05,
        )
        self.declare_parameter(
            'reacquire_frames',
            3,
        )
        self.declare_parameter(
            'confidence_sigma',
            0.05,
        )
        self.declare_parameter(
            'stability_speed',
            0.20,
        )
        self.declare_parameter(
            'lost_timeout',
            0.5,
        )

        self.min_pair_distance = float(
            self.get_parameter(
                'min_pair_distance'
            ).value
        )

        self.max_pair_distance = float(
            self.get_parameter(
                'max_pair_distance'
            ).value
        )

        self.min_axis_sin = float(
            self.get_parameter(
                'min_axis_sin'
            ).value
        )

        self.reacquire_frames = int(
            self.get_parameter(
                'reacquire_frames'
            ).value
        )

        self.confidence_sigma = float(
            self.get_parameter(
                'confidence_sigma'
            ).value
        )

        self.stability_speed = float(
            self.get_parameter(
                'stability_speed'
            ).value
        )

        self.lost_timeout = float(
            self.get_parameter(
                'lost_timeout'
            ).value
        )

        self.good_frames = 0
        self.ready = False
        self.prev_normal = None

        # C5 causal regression.
        self.palm_history = deque(
            maxlen=5
        )

        self.publisher = self.create_publisher(
            HandState,
            '/handover/hand_state',
            10,
        )

        self.subscription = self.create_subscription(
            HandTrackingFiltered,
            '/handover/hand_tracking_filtered',
            self.callback,
            10,
        )

        self.get_logger().info(
            'Hand state estimator node started'
        )

    @staticmethod
    def point_array(point):
        return np.array(
            [
                point.x,
                point.y,
                point.z,
            ],
            dtype=float,
        )

    def estimate_palm_velocity(
        self,
        stamp,
        position,
    ):
        """
        Causal linear regression on up to
        the latest five palm positions.
        """

        t = (
            stamp.sec
            + stamp.nanosec * 1e-9
        )

        self.palm_history.append(
            (
                t,
                position.copy(),
            )
        )

        if len(self.palm_history) < 3:
            return np.zeros(3)

        times = np.array([
            item[0]
            for item in self.palm_history
        ])

        positions = np.array([
            item[1]
            for item in self.palm_history
        ])

        tc = (
            times
            - np.mean(times)
        )

        denom = np.dot(
            tc,
            tc,
        )

        if denom <= 0.0:
            return np.zeros(3)

        return np.sum(
            tc[:, None]
            * (
                positions
                - np.mean(
                    positions,
                    axis=0,
                )
            ),
            axis=0,
        ) / denom

    def hand_frame(self, points):
        """
        Return longitudinal axis and
        palm normal, or None.
        """

        for i in range(4):
            for j in range(
                i + 1,
                4,
            ):
                distance = np.linalg.norm(
                    points[i]
                    - points[j]
                )

                if (
                    distance
                    < self.min_pair_distance
                    or
                    distance
                    > self.max_pair_distance
                ):
                    return None

        # Fixed order [0, 5, 9, 17]
        p0, p5, p9, p17 = points

        ex = p5 - p17
        ey_raw = p9 - p0

        ex_norm = np.linalg.norm(ex)

        ey_raw_norm = np.linalg.norm(
            ey_raw
        )

        if (
            ex_norm <= 0.0
            or
            ey_raw_norm <= 0.0
        ):
            return None

        ex /= ex_norm
        ey_raw /= ey_raw_norm

        # Gram-Schmidt.
        ey = (
            ey_raw
            - ex
            * np.dot(
                ex,
                ey_raw,
            )
        )

        ey_norm = np.linalg.norm(ey)

        if ey_norm < self.min_axis_sin:
            return None

        ey /= ey_norm

        normal = np.cross(
            ex,
            ey,
        )

        normal_norm = np.linalg.norm(
            normal
        )

        if normal_norm <= 0.0:
            return None

        normal /= normal_norm

        return ey, normal

    def tracking_confidence(
        self,
        msg,
    ):
        scores = []

        for i, landmark_state in enumerate(
            msg.landmark_state
        ):
            if (
                landmark_state
                == HandTrackingFiltered.TRACKING
            ):
                score = 1.0

            elif (
                landmark_state
                == HandTrackingFiltered.PREDICT_ONLY
            ):
                age_factor = max(
                    0.0,
                    1.0
                    - float(
                        msg.age_s[i]
                    )
                    / self.lost_timeout,
                )

                score = (
                    0.6
                    * age_factor
                )

            else:
                score = 0.0

            if (
                msg.measurement_type[i]
                == HandTrackingFiltered.ESTIMATED
            ):
                score *= 0.7

            scores.append(score)

        variances = []

        for variance in msg.position_variance:
            variances.extend([
                variance.x,
                variance.y,
                variance.z,
            ])

        mean_sigma = np.sqrt(
            max(
                0.0,
                float(
                    np.mean(
                        variances
                    )
                ),
            )
        )

        uncertainty = np.exp(
            -mean_sigma
            / self.confidence_sigma
        )

        return float(
            np.clip(
                np.mean(scores)
                * uncertainty,
                0.0,
                1.0,
            )
        )

    def set_invalid(
        self,
        out,
        reset_normal,
    ):
        self.good_frames = 0
        self.ready = False
        self.palm_history.clear()

        if reset_normal:
            self.prev_normal = None

        self.publisher.publish(
            out
        )

    def callback(self, msg):
        start = time.perf_counter()

        out = HandState()

        out.header = msg.header

        out.filter_state = int(
            msg.filter_state
        )

        usable = all(
            state in (
                HandTrackingFiltered.TRACKING,
                HandTrackingFiltered.PREDICT_ONLY,
            )
            for state
            in msg.landmark_state
        )

        if not usable:
            out.processing_latency_ms = float(
                (
                    time.perf_counter()
                    - start
                )
                * 1000.0
            )

            self.set_invalid(
                out,
                reset_normal=True,
            )

            return

        has_tracking = any(
            state
            == HandTrackingFiltered.TRACKING
            for state
            in msg.landmark_state
        )

        if not has_tracking:
            out.processing_latency_ms = float(
                (
                    time.perf_counter()
                    - start
                )
                * 1000.0
            )

            self.set_invalid(
                out,
                reset_normal=False,
            )

            return

        points = np.array([
            self.point_array(point)
            for point in msg.positions
        ])

        frame = self.hand_frame(
            points
        )

        out.geometry_ok = (
            frame is not None
        )

        if frame is None:
            out.processing_latency_ms = float(
                (
                    time.perf_counter()
                    - start
                )
                * 1000.0
            )

            self.set_invalid(
                out,
                reset_normal=False,
            )

            return

        _, normal = frame
        
        if (
            self.prev_normal
            is not None
            and
            np.dot(
                normal,
                self.prev_normal,
            ) < 0.0
        ):
            normal = -normal

        self.prev_normal = (
            normal.copy()
        )

        # Palm center from MCP 5, 9, 17.
        palm_position = np.mean(
            points[
                [1, 2, 3]
            ],
            axis=0,
        )

        # C5 causal palm velocity.
        palm_velocity = (
            self.estimate_palm_velocity(
                msg.header.stamp,
                palm_position,
            )
        )

        if not self.ready:
            self.good_frames += 1

            self.ready = (
                self.good_frames
                >= self.reacquire_frames
            )

        speed = float(
            np.linalg.norm(
                palm_velocity
            )
        )

        stability = float(
            np.clip(
                1.0
                - speed
                / self.stability_speed,
                0.0,
                1.0,
            )
        )

        confidence = (
            self.tracking_confidence(
                msg
            )
        )

        if not self.ready:
            confidence = 0.0
            stability = 0.0

        out.valid = bool(
            self.ready
        )

        out.palm_position = Point(
            x=float(
                palm_position[0]
            ),
            y=float(
                palm_position[1]
            ),
            z=float(
                palm_position[2]
            ),
        )

        out.palm_velocity = Vector3(
            x=float(
                palm_velocity[0]
            ),
            y=float(
                palm_velocity[1]
            ),
            z=float(
                palm_velocity[2]
            ),
        )

        out.palm_normal = Vector3(
            x=float(
                normal[0]
            ),
            y=float(
                normal[1]
            ),
            z=float(
                normal[2]
            ),
        )

        out.palm_speed = speed

        out.tracking_confidence = (
            confidence
        )

        out.motion_stability = (
            stability
        )

        out.processing_latency_ms = float(
            (
                time.perf_counter()
                - start
            )
            * 1000.0
        )

        self.publisher.publish(
            out
        )


def main(args=None):
    rclpy.init(args=args)

    node = HandStateEstimator()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()