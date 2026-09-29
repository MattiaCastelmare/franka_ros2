#!/usr/bin/env python3
"""Expose the existing EE–palm estimate without recomputing FK, J or rates."""

import rclpy
from rclpy.node import Node
from franka_msgs.msg import HandoverDistance, ProximityState
from franka_experiments.utils.node_runtime import teardown


class ProximityEstimator(Node):
    def __init__(self):
        super().__init__('proximity_estimator')
        self.publisher = self.create_publisher(
            ProximityState, '/handover/proximity', 10)
        self.subscription = self.create_subscription(
            HandoverDistance, '/handover/distance', self.callback, 10)

    def callback(self, msg):
        out = ProximityState()
        out.header = msg.header
        out.robot_point_id = 'ee_control_point'
        out.target_point_id = 'active_palm'
        out.valid = msg.valid
        if msg.valid:
            out.robot_position = msg.ee_control_point
            out.target_position = msg.palm_position
            out.distance = msg.distance
            out.distance_sigma = msg.distance_sigma
            out.rate_valid = msg.rate_valid
            out.rate_source = msg.rate_source
            out.rate_degraded = msg.rate_degraded
            out.rate_age_s = msg.rate_age_s
            if msg.rate_valid:
                out.distance_rate = msg.distance_rate
                out.closing_velocity = msg.closing_velocity
                out.relative_velocity_valid = bool(
                    msg.hand_velocity_valid and msg.ee_velocity_valid
                    and msg.rate_source in (
                        HandoverDistance.RATE_SOURCE_RELATIVE_UPDATED,
                        HandoverDistance.RATE_SOURCE_RELATIVE_HOLD))
                if out.relative_velocity_valid:
                    out.relative_velocity = msg.relative_velocity
                out.ttc_valid = msg.ttc_valid
                out.ttc = msg.ttc if msg.ttc_valid else 0.0
        self.publisher.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ProximityEstimator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        teardown(node)
