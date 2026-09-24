#!/usr/bin/env python3

import os
from collections import deque

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, Vector3
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

from franka_msgs.msg import EndEffectorState, HandState, HandoverDistance
from franka_experiments.utils.distance_utils import load_robot_config
from franka_experiments.utils.params import (
    load_hand_tracking_defaults,
    parameter_value,
)

_DISTANCE_DEFAULTS = load_hand_tracking_defaults(
    'distance_handover_estimator'
)


class HandoverDistanceEstimator(Node):

    def __init__(self):
        super().__init__('distance_handover_estimator')

        default_config = os.path.join(
            get_package_share_directory('franka_experiments'),
            'config',
            'fr3_complete.yaml',
        )

        self.declare_parameter(
            'robot_config_path',
            default_config,
        )
        self.declare_parameter(
            'max_ee_state_age_s',
            0.10,
        )

        config = load_robot_config(
            self.get_parameter(
                'robot_config_path'
            ).value
        )

        self.base_frame = config['robot']['base_frame']

        self.max_ee_state_age_s = float(
            self.get_parameter(
                'max_ee_state_age_s'
            ).value
        )

        self.ttc_min_closing_speed = float(
            parameter_value(
                self,
                _DISTANCE_DEFAULTS,
                'ttc_min_closing_speed',
            )
        )

        # TF remains only for hand -> base transformation.
        # Robot EE state comes from EndEffectorState.
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self,
        )

        # Short EE history only for timestamp alignment.
        # No differentiation and no C5 are performed here.
        self.ee_states = deque(maxlen=100)

        # Existing scalar-distance C5 fallback.
        # Intentionally unchanged.
        self.distance_history = deque(maxlen=5)

        self.publisher = self.create_publisher(
            HandoverDistance,
            '/handover/distance',
            10,
        )

        self.create_subscription(
            EndEffectorState,
            '/handover/end_effector_state',
            self.on_ee_state,
            10,
        )

        self.create_subscription(
            HandState,
            '/handover/hand_state',
            self.callback,
            10,
        )

        self.get_logger().info(
            'Handover distance estimator started '
            f'(EndEffectorState -> palm, '
            f'frame={self.base_frame}, '
            f'TTC min closing='
            f'{self.ttc_min_closing_speed:.3f} m/s)'
        )

    @staticmethod
    def point_array(p):
        return np.array(
            [p.x, p.y, p.z],
            dtype=np.float64,
        )

    @staticmethod
    def vector_array(v):
        return np.array(
            [v.x, v.y, v.z],
            dtype=np.float64,
        )

    @staticmethod
    def stamp_to_seconds(stamp):
        return (
            stamp.sec
            + stamp.nanosec * 1e-9
        )

    @staticmethod
    def causal_linear_rate(history):
        if len(history) < 3:
            return False, None

        times = np.asarray(
            [item[0] for item in history],
            dtype=np.float64,
        )

        values = np.asarray(
            [item[1] for item in history],
            dtype=np.float64,
        )

        tc = times - np.mean(times)

        denom = float(
            np.dot(
                tc,
                tc,
            )
        )

        if denom <= 1e-12:
            return False, None

        rate = float(
            np.dot(
                tc,
                values - np.mean(values),
            )
            / denom
        )

        if not np.isfinite(rate):
            return False, None

        return True, rate

    def estimate_distance_rate(
        self,
        stamp,
        distance,
    ):
        t = self.stamp_to_seconds(
            stamp
        )

        if (
            self.distance_history
            and
            t <= self.distance_history[-1][0]
        ):
            self.distance_history.clear()

        self.distance_history.append(
            (
                t,
                float(distance),
            )
        )

        valid, rate = (
            self.causal_linear_rate(
                self.distance_history
            )
        )

        return (
            valid,
            float(rate)
            if valid
            else 0.0,
        )

    def on_ee_state(
        self,
        msg: EndEffectorState,
    ):
        if not msg.valid:
            return

        if msg.header.frame_id != self.base_frame:
            self.get_logger().warn(
                'Ignoring EndEffectorState '
                f'frame={msg.header.frame_id}; '
                f'expected {self.base_frame}',
                throttle_duration_sec=2.0,
            )
            return

        t = self.stamp_to_seconds(
            msg.header.stamp
        )

        if (
            self.ee_states
            and
            t <= self.ee_states[-1][0]
        ):
            self.ee_states.clear()

        self.ee_states.append(
            (
                t,
                msg,
            )
        )

    def get_ee_state(
        self,
        stamp,
    ):
        if not self.ee_states:
            return (
                None,
                float('nan'),
            )

        t_hand = self.stamp_to_seconds(
            stamp
        )

        t_ee, state = min(
            self.ee_states,
            key=lambda item: abs(
                item[0] - t_hand
            ),
        )

        age = abs(
            t_hand - t_ee
        )

        if age > self.max_ee_state_age_s:
            return (
                None,
                age,
            )

        return (
            state,
            age,
        )

    def publish_invalid(
        self,
        out,
    ):
        self.distance_history.clear()
        self.publisher.publish(out)

    @staticmethod
    def quaternion_matrix(q):
        x = q.x
        y = q.y
        z = q.z
        w = q.w

        return np.array([
            [
                1 - 2*(y*y + z*z),
                2*(x*y - z*w),
                2*(x*z + y*w),
            ],
            [
                2*(x*y + z*w),
                1 - 2*(x*x + z*z),
                2*(y*z - x*w),
            ],
            [
                2*(x*z - y*w),
                2*(y*z + x*w),
                1 - 2*(x*x + y*y),
            ],
        ])

    def transform_hand(
        self,
        msg,
    ):
        p = self.point_array(
            msg.palm_position
        )

        covariance = np.diag([
            max(
                0.0,
                msg.palm_position_variance.x,
            ),
            max(
                0.0,
                msg.palm_position_variance.y,
            ),
            max(
                0.0,
                msg.palm_position_variance.z,
            ),
        ])

        if msg.header.frame_id == self.base_frame:
            return (
                p,
                covariance,
                np.eye(3),
            )

        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame,
                msg.header.frame_id,
                Time.from_msg(
                    msg.header.stamp
                ),
            )

        except Exception as exc:
            self.get_logger().warn(
                'Cannot transform hand '
                f'{msg.header.frame_id} -> '
                f'{self.base_frame}: {exc}',
                throttle_duration_sec=2.0,
            )

            return (
                None,
                None,
                None,
            )

        t = tf.transform.translation

        R = self.quaternion_matrix(
            tf.transform.rotation
        )

        translation = np.array(
            [
                t.x,
                t.y,
                t.z,
            ],
            dtype=np.float64,
        )

        return (
            R @ p + translation,
            R @ covariance @ R.T,
            R,
        )

    @staticmethod
    def set_vector3(
        field,
        value,
    ):
        field.x = float(value[0])
        field.y = float(value[1])
        field.z = float(value[2])

    def callback(
        self,
        msg: HandState,
    ):
        out = HandoverDistance()

        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.base_frame

        out.rate_source = (
            HandoverDistance.RATE_SOURCE_NONE
        )

        out.rate_age_s = float('nan')
        out.hand_velocity_age_s = float('nan')
        out.ee_velocity_age_s = float('nan')
        out.rate_consistency_error = float('nan')

        # Distance requires hand position only.
        if not msg.position_valid:
            self.publish_invalid(out)
            return

        (
            p_hand,
            covariance,
            hand_rotation,
        ) = self.transform_hand(msg)

        if p_hand is None:
            self.publish_invalid(out)
            return

        # -------------------------------------------------------------
        # ROBOT STATE
        #
        # No TF reconstruction.
        # No EE position differentiation.
        # No EE C5.
        #
        # p_ee and v_ee come directly from EndEffectorState:
        #
        #   p_ee = FK(q)
        #   v_ee = J_ee(q) qdot
        # -------------------------------------------------------------

        ee_state, ee_age = (
            self.get_ee_state(
                msg.header.stamp
            )
        )

        if ee_state is None:
            self.publish_invalid(out)
            return

        p_ee = self.point_array(
            ee_state.position
        )

        ee_velocity = self.vector_array(
            ee_state.velocity
        )

        if not np.all(
            np.isfinite(p_ee)
        ):
            self.publish_invalid(out)
            return

        ee_velocity_valid = bool(
            ee_state.valid
            and
            np.all(
                np.isfinite(
                    ee_velocity
                )
            )
        )

        delta = (
            p_hand - p_ee
        )

        distance = float(
            np.linalg.norm(
                delta
            )
        )

        if (
            not np.isfinite(distance)
            or
            distance <= 1e-9
        ):
            self.publish_invalid(out)
            return

        direction = (
            delta / distance
        )

        distance_variance = float(
            direction.T
            @ covariance
            @ direction
        )

        position_predicted = bool(
            int(msg.position_source)
            ==
            int(
                HandState.POSITION_SOURCE_PREDICTED_W75
            )
        )

        # -------------------------------------------------------------
        # LEGACY SCALAR DISTANCE C5
        #
        # This is intentionally preserved.
        # It is the existing fallback for distance-rate estimation and
        # is independent from the removed EE TF-C5 velocity estimator.
        # -------------------------------------------------------------

        if position_predicted:

            self.distance_history.clear()

            legacy_rate_valid = False
            legacy_distance_rate = 0.0

        else:

            (
                legacy_rate_valid,
                legacy_distance_rate,
            ) = self.estimate_distance_rate(
                msg.header.stamp,
                distance,
            )

        # HAND VELOCITY

        hand_velocity_valid = bool(
            msg.velocity_valid
        )

        hand_velocity = np.zeros(
            3,
            dtype=np.float64,
        )

        if hand_velocity_valid:

            hand_velocity = (
                hand_rotation
                @
                self.vector_array(
                    msg.palm_velocity
                )
            )

            if not np.all(
                np.isfinite(
                    hand_velocity
                )
            ):
                hand_velocity_valid = False
                hand_velocity[:] = 0.0

        # RELATIVE RADIAL RATE

        relative_velocity = np.zeros(
            3,
            dtype=np.float64,
        )

        relative_rate_valid = False
        relative_rate = 0.0

        if (
            not position_predicted
            and
            hand_velocity_valid
            and
            ee_velocity_valid
        ):
            relative_velocity = (
                hand_velocity
                -
                ee_velocity
            )

            relative_rate = float(
                np.dot(
                    direction,
                    relative_velocity,
                )
            )

            relative_rate_valid = bool(
                np.isfinite(
                    relative_rate
                )
            )

        primary_source = (
            HandoverDistance.RATE_SOURCE_NONE
        )

        if relative_rate_valid:

            if (
                int(msg.velocity_source)
                ==
                HandState.VELOCITY_SOURCE_UPDATED
            ):
                primary_source = (
                    HandoverDistance
                    .RATE_SOURCE_RELATIVE_UPDATED
                )

            elif (
                int(msg.velocity_source)
                ==
                HandState.VELOCITY_SOURCE_HOLD
            ):
                primary_source = (
                    HandoverDistance
                    .RATE_SOURCE_RELATIVE_HOLD
                )

            else:
                relative_rate_valid = False

        # SELECT RATE

        if relative_rate_valid:

            rate_valid = True
            distance_rate = relative_rate
            rate_source = primary_source

            rate_degraded = bool(
                rate_source
                ==
                HandoverDistance
                .RATE_SOURCE_RELATIVE_HOLD
            )

            hand_age = float(
                msg.velocity_age_s
            )

            rate_age_s = (
                max(
                    hand_age,
                    0.0,
                )
                if np.isfinite(hand_age)
                else 0.0
            )

        elif legacy_rate_valid:

            rate_valid = True

            distance_rate = float(
                legacy_distance_rate
            )

            rate_source = (
                HandoverDistance
                .RATE_SOURCE_DISTANCE_C5
            )

            rate_degraded = True
            rate_age_s = 0.0

        else:

            rate_valid = False
            distance_rate = 0.0

            rate_source = (
                HandoverDistance
                .RATE_SOURCE_NONE
            )

            rate_degraded = bool(
                position_predicted
            )

            rate_age_s = (
                float(
                    msg.position_age_s
                )
                if (
                    position_predicted
                    and
                    np.isfinite(
                        float(
                            msg.position_age_s
                        )
                    )
                )
                else float('nan')
            )

        closing_velocity = (
            -distance_rate
            if rate_valid
            else 0.0
        )

        # ROBUST TTC

        ttc_valid = bool(
            rate_valid
            and
            closing_velocity
            >
            self.ttc_min_closing_speed
        )

        ttc = (
            distance
            /
            closing_velocity
            if ttc_valid
            else 0.0
        )

        # Existing scalar-distance diagnostics.

        legacy_closing = (
            -legacy_distance_rate
            if legacy_rate_valid
            else 0.0
        )

        legacy_ttc_valid = bool(
            legacy_rate_valid
            and
            legacy_closing > 1e-6
        )

        legacy_ttc = (
            distance
            /
            legacy_closing
            if legacy_ttc_valid
            else 0.0
        )

        consistency_error = float('nan')

        if (
            relative_rate_valid
            and
            legacy_rate_valid
        ):
            consistency_error = abs(
                float(relative_rate)
                -
                float(
                    legacy_distance_rate
                )
            )

        # OUTPUT

        out.valid = True

        out.palm_position = Point(
            x=float(p_hand[0]),
            y=float(p_hand[1]),
            z=float(p_hand[2]),
        )

        out.ee_control_point = Point(
            x=float(p_ee[0]),
            y=float(p_ee[1]),
            z=float(p_ee[2]),
        )

        out.ee_to_palm = Vector3(
            x=float(delta[0]),
            y=float(delta[1]),
            z=float(delta[2]),
        )

        out.distance = float(
            distance
        )

        out.distance_sigma = float(
            np.sqrt(
                max(
                    distance_variance,
                    0.0,
                )
            )
        )

        out.tracking_confidence = float(
            msg.tracking_confidence
        )

        out.motion_stability = float(
            msg.motion_stability
        )

        out.hand_velocity_valid = bool(
            hand_velocity_valid
        )

        out.hand_velocity_source = int(
            msg.velocity_source
        )

        out.hand_velocity_estimator = int(
            msg.velocity_estimator
        )

        out.hand_velocity_age_s = (
            float(
                msg.velocity_age_s
            )
            if hand_velocity_valid
            else float('nan')
        )

        self.set_vector3(
            out.hand_velocity,
            hand_velocity,
        )

        out.ee_velocity_valid = bool(
            ee_velocity_valid
        )

        out.ee_velocity_age_s = (
            float(ee_age)
            if ee_velocity_valid
            else float('nan')
        )

        self.set_vector3(
            out.ee_velocity,
            ee_velocity,
        )

        self.set_vector3(
            out.relative_velocity,
            relative_velocity,
        )

        out.legacy_rate_valid = bool(
            legacy_rate_valid
        )

        out.legacy_distance_rate = float(
            legacy_distance_rate
        )

        out.rate_consistency_error = float(
            consistency_error
        )

        out.rate_valid = bool(
            rate_valid
        )

        out.rate_source = int(
            rate_source
        )

        out.rate_degraded = bool(
            rate_degraded
        )

        out.rate_age_s = float(
            rate_age_s
        )

        out.distance_rate = float(
            distance_rate
        )

        out.closing_velocity = float(
            closing_velocity
        )

        out.ttc_valid = bool(
            ttc_valid
        )

        out.ttc = float(
            ttc
        )

        out.legacy_ttc_valid = bool(
            legacy_ttc_valid
        )

        out.legacy_ttc = float(
            legacy_ttc
        )

        self.publisher.publish(out)


def main(args=None):
    rclpy.init(args=args)

    node = HandoverDistanceEstimator()

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
