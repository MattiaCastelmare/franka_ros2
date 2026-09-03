#!/usr/bin/env python3

import csv
from datetime import datetime
from pathlib import Path
from zoneinfo import ZoneInfo

import rclpy
from rclpy.node import Node

from franka_msgs.msg import (
    HandTrackingRaw,
    HandTrackingFiltered,
    HandState,
)


RAW_STATES = {
    HandTrackingRaw.NO_HAND: 'NO_HAND',
    HandTrackingRaw.TRACKING_PARTIAL: 'TRACKING_PARTIAL',
    HandTrackingRaw.TRACKING_FULL: 'TRACKING_FULL',
    HandTrackingRaw.TRACKING_ESTIMATED: 'TRACKING_ESTIMATED',
    HandTrackingRaw.INVALID_DEPTH: 'INVALID_DEPTH',
}

FILTER_STATES = {
    HandTrackingFiltered.UNINITIALIZED: 'UNINITIALIZED',
    HandTrackingFiltered.TRACKING: 'TRACKING',
    HandTrackingFiltered.PREDICT_ONLY: 'PREDICT_ONLY',
    HandTrackingFiltered.LOST: 'LOST',
}

MEASUREMENT_TYPES = {
    HandTrackingRaw.DIRECT: 'DIRECT',
    HandTrackingRaw.ESTIMATED: 'ESTIMATED',
    HandTrackingRaw.INVALID: 'INVALID',
}


class HandTrackingCsvLogger(Node):

    def __init__(self):
        super().__init__('hand_tracking_csv_logger')

        self.declare_parameter(
            'output_root',
            '/ros2_ws/src/franka_experiments/results',
        )

        timestamp = datetime.now(
            ZoneInfo('Europe/Rome')
        ).strftime('%Y%m%d_%H%M%S')

        run_dir = Path(self.get_parameter('output_root').value) / timestamp
        run_dir.mkdir(parents=True, exist_ok=True)

        self.raw_path = run_dir / 'hand_tracking_raw.csv'
        self.filtered_path = run_dir / 'hand_tracking_filtered.csv'
        self.state_path = run_dir / 'hand_state.csv'

        self.raw_file = self.raw_path.open(
            'w', newline='', encoding='utf-8'
        )
        self.filtered_file = self.filtered_path.open(
            'w', newline='', encoding='utf-8'
        )
        self.state_file = self.state_path.open(
            'w', newline='', encoding='utf-8'
        )

        self.raw_writer = csv.writer(self.raw_file)
        self.filtered_writer = csv.writer(self.filtered_file)
        self.state_writer = csv.writer(self.state_file)

        self.raw_first_timestamp = None
        self.filtered_first_timestamp = None
        self.state_first_timestamp = None

        self.raw_writer.writerow(self.raw_header())
        self.filtered_writer.writerow(self.filtered_header())
        self.state_writer.writerow(self.state_header())

        self.raw_file.flush()
        self.filtered_file.flush()
        self.state_file.flush()

        self.create_subscription(
            HandTrackingRaw,
            '/handover/hand_tracking_raw',
            self.raw_callback,
            10,
        )

        self.create_subscription(
            HandTrackingFiltered,
            '/handover/hand_tracking_filtered',
            self.filtered_callback,
            10,
        )

        self.create_subscription(
            HandState,
            '/handover/hand_state',
            self.state_callback,
            10,
        )

        self.get_logger().info(f'CSV raw: {self.raw_path}')
        self.get_logger().info(f'CSV filtered: {self.filtered_path}')
        self.get_logger().info(f'CSV state: {self.state_path}')

    @staticmethod
    def timestamp_s(msg):
        return (
            float(msg.header.stamp.sec)
            + 1e-9 * float(msg.header.stamp.nanosec)
        )

    @staticmethod
    def raw_header():
        header = [
            'timestamp_s',
            'elapsed_s',
            'frame_id',
            'tracking_state',
            'tracking_state_name',
            'processing_latency_ms',
        ]

        for i in range(4):
            header.extend([
                f'landmark_{i}_id',
                f'landmark_{i}_x',
                f'landmark_{i}_y',
                f'landmark_{i}_z',
                f'landmark_{i}_valid',
                f'landmark_{i}_measurement_type',
                f'landmark_{i}_measurement_name',
            ])

        return header

    @staticmethod
    def filtered_header():
        header = [
            'timestamp_s',
            'elapsed_s',
            'frame_id',
            'filter_state',
            'filter_state_name',
            'processing_latency_ms',
        ]

        for i in range(4):
            header.extend([
                f'landmark_{i}_id',
                f'landmark_{i}_state',
                f'landmark_{i}_state_name',
                f'landmark_{i}_measurement_used',
                f'landmark_{i}_measurement_type',
                f'landmark_{i}_measurement_name',
                f'landmark_{i}_mahalanobis_sq',
                f'landmark_{i}_x',
                f'landmark_{i}_y',
                f'landmark_{i}_z',
                f'landmark_{i}_vx',
                f'landmark_{i}_vy',
                f'landmark_{i}_vz',
                f'landmark_{i}_position_var_x',
                f'landmark_{i}_position_var_y',
                f'landmark_{i}_position_var_z',
                f'landmark_{i}_velocity_var_x',
                f'landmark_{i}_velocity_var_y',
                f'landmark_{i}_velocity_var_z',
                f'landmark_{i}_age_s',
                f'landmark_{i}_missed_updates',
            ])

        return header

    @staticmethod
    def state_header():
        return [
            'timestamp_s',
            'elapsed_s',
            'frame_id',
            'valid',
            'geometry_ok',
            'filter_state',
            'filter_state_name',
            'palm_x',
            'palm_y',
            'palm_z',
            'palm_vx',
            'palm_vy',
            'palm_vz',
            'normal_x',
            'normal_y',
            'normal_z',
            'palm_speed',
            'tracking_confidence',
            'motion_stability',
            'processing_latency_ms',
        ]

    def raw_callback(self, msg):
        timestamp = self.timestamp_s(msg)

        if self.raw_first_timestamp is None:
            self.raw_first_timestamp = timestamp

        state = int(msg.tracking_state)

        row = [
            timestamp,
            timestamp - self.raw_first_timestamp,
            msg.header.frame_id,
            state,
            RAW_STATES.get(state, 'UNKNOWN'),
            float(msg.processing_latency_ms),
        ]

        for i in range(4):
            point = msg.positions[i]
            measurement_type = int(msg.measurement_type[i])

            row.extend([
                int(msg.landmark_ids[i]),
                float(point.x),
                float(point.y),
                float(point.z),
                int(msg.valid[i]),
                measurement_type,
                MEASUREMENT_TYPES.get(
                    measurement_type,
                    'UNKNOWN',
                ),
            ])

        self.raw_writer.writerow(row)
        self.raw_file.flush()

    def filtered_callback(self, msg):
        timestamp = self.timestamp_s(msg)

        if self.filtered_first_timestamp is None:
            self.filtered_first_timestamp = timestamp

        state = int(msg.filter_state)

        row = [
            timestamp,
            timestamp - self.filtered_first_timestamp,
            msg.header.frame_id,
            state,
            FILTER_STATES.get(state, 'UNKNOWN'),
            float(msg.processing_latency_ms),
        ]

        for i in range(4):
            point = msg.positions[i]
            velocity = msg.velocities[i]
            position_var = msg.position_variance[i]
            velocity_var = msg.velocity_variance[i]

            landmark_state = int(msg.landmark_state[i])
            measurement_type = int(msg.measurement_type[i])

            row.extend([
                int(msg.landmark_ids[i]),
                landmark_state,
                FILTER_STATES.get(
                    landmark_state,
                    'UNKNOWN',
                ),
                int(msg.measurement_used[i]),
                measurement_type,
                MEASUREMENT_TYPES.get(
                    measurement_type,
                    'UNKNOWN',
                ),
                float(msg.mahalanobis_sq[i]),
                float(point.x),
                float(point.y),
                float(point.z),
                float(velocity.x),
                float(velocity.y),
                float(velocity.z),
                float(position_var.x),
                float(position_var.y),
                float(position_var.z),
                float(velocity_var.x),
                float(velocity_var.y),
                float(velocity_var.z),
                float(msg.age_s[i]),
                int(msg.missed_updates[i]),
            ])

        self.filtered_writer.writerow(row)
        self.filtered_file.flush()

    def state_callback(self, msg):
        timestamp = self.timestamp_s(msg)

        if self.state_first_timestamp is None:
            self.state_first_timestamp = timestamp

        state = int(msg.filter_state)

        row = [
            timestamp,
            timestamp - self.state_first_timestamp,
            msg.header.frame_id,
            int(msg.valid),
            int(msg.geometry_ok),
            state,
            FILTER_STATES.get(state, 'UNKNOWN'),
            float(msg.palm_position.x),
            float(msg.palm_position.y),
            float(msg.palm_position.z),
            float(msg.palm_velocity.x),
            float(msg.palm_velocity.y),
            float(msg.palm_velocity.z),
            float(msg.palm_normal.x),
            float(msg.palm_normal.y),
            float(msg.palm_normal.z),
            float(msg.palm_speed),
            float(msg.tracking_confidence),
            float(msg.motion_stability),
            float(msg.processing_latency_ms),
        ]

        self.state_writer.writerow(row)
        self.state_file.flush()

    def destroy_node(self):
        if not self.raw_file.closed:
            self.raw_file.close()

        if not self.filtered_file.closed:
            self.filtered_file.close()

        if not self.state_file.closed:
            self.state_file.close()

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = HandTrackingCsvLogger()

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
