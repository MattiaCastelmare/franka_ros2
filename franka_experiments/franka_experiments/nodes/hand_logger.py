#!/usr/bin/env python3

import csv
from datetime import datetime
from pathlib import Path

import rclpy
from franka_msgs.msg import HandTrackingRaw
from rclpy.node import Node


TRACKING_STATES = {
    HandTrackingRaw.NO_HAND: 'NO_HAND',
    HandTrackingRaw.TRACKING_PARTIAL: 'TRACKING_PARTIAL',
    HandTrackingRaw.TRACKING_FULL: 'TRACKING_FULL',
    HandTrackingRaw.TRACKING_ESTIMATED: 'TRACKING_ESTIMATED',
    HandTrackingRaw.INVALID_DEPTH: 'INVALID_DEPTH',
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

        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')

        run_dir = (
            Path(self.get_parameter('output_root').value)
            / timestamp
        )
        run_dir.mkdir(parents=True, exist_ok=True)

        self.output_path = run_dir / 'hand_tracking.csv'
        self.file = self.output_path.open(
            'w',
            newline='',
            encoding='utf-8',
        )
        self.writer = csv.writer(self.file)
        self.first_timestamp = None

        header = [
            'timestamp_s',
            'elapsed_s',
            'frame_id',
            'tracking_state',
            'tracking_state_name',
            'processing_latency_ms',
        ]

        for index in range(4):
            header.extend([
                f'landmark_{index}_id',
                f'landmark_{index}_x',
                f'landmark_{index}_y',
                f'landmark_{index}_z',
                f'landmark_{index}_valid',
                f'landmark_{index}_measurement_type',
                f'landmark_{index}_measurement_name',
            ])

        self.writer.writerow(header)
        self.file.flush()

        self.subscription = self.create_subscription(
            HandTrackingRaw,
            '/handover/hand_tracking_raw',
            self.tracking_callback,
            10,
        )

        self.get_logger().info(
            f'CSV automatico: {self.output_path}'
        )

    def tracking_callback(self, msg):
        timestamp = (
            float(msg.header.stamp.sec)
            + 1e-9 * float(msg.header.stamp.nanosec)
        )

        if self.first_timestamp is None:
            self.first_timestamp = timestamp

        state = int(msg.tracking_state)

        row = [
            timestamp,
            timestamp - self.first_timestamp,
            msg.header.frame_id,
            state,
            TRACKING_STATES.get(state, 'UNKNOWN'),
            float(msg.processing_latency_ms),
        ]

        for index in range(4):
            point = msg.positions[index]
            measurement_type = int(
                msg.measurement_type[index]
            )

            row.extend([
                int(msg.landmark_ids[index]),
                float(point.x),
                float(point.y),
                float(point.z),
                int(msg.valid[index]),
                measurement_type,
                MEASUREMENT_TYPES.get(
                    measurement_type,
                    'UNKNOWN',
                ),
            ])

        self.writer.writerow(row)
        self.file.flush()

    def destroy_node(self):
        if not self.file.closed:
            self.file.close()

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