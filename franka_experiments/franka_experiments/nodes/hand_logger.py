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
    HandoverDistance,
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

HANDEDNESS_NAMES = {
    HandTrackingRaw.HAND_UNKNOWN: 'UNKNOWN',
    HandTrackingRaw.HAND_LEFT: 'LEFT',
    HandTrackingRaw.HAND_RIGHT: 'RIGHT',
}

VELOCITY_SOURCES = {
    HandState.VELOCITY_SOURCE_NONE: 'NONE',
    HandState.VELOCITY_SOURCE_UPDATED: 'UPDATED',
    HandState.VELOCITY_SOURCE_HOLD: 'HOLD',
}

VELOCITY_ESTIMATORS = {
    HandState.VELOCITY_ESTIMATOR_C5: 'C5',
    HandState.VELOCITY_ESTIMATOR_W75: 'W75',
}

from franka_experiments.utils.params import (
    load_hand_tracking_defaults,
    parameter_value,
)

_LOGGER_DEFAULTS = load_hand_tracking_defaults("hand_tracking_csv_logger")

class HandTrackingCsvLogger(Node):

    def __init__(self):
        super().__init__('hand_tracking_csv_logger')
        timestamp = datetime.now(
            ZoneInfo('Europe/Rome')
        ).strftime('%Y%m%d_%H%M%S')
        run_dir = Path(parameter_value(self, _LOGGER_DEFAULTS, "output_root")) / timestamp
        run_dir.mkdir(parents=True, exist_ok=True)
        self.raw_path = run_dir / 'hand_tracking_raw.csv'
        self.filtered_path = run_dir / 'hand_tracking_filtered.csv'
        self.state_path = run_dir / 'hand_state.csv'
        self.distance_path = run_dir / 'handover_distance.csv'
        self.raw_file = self.raw_path.open(
            'w', newline='', encoding='utf-8'
        )
        self.filtered_file = self.filtered_path.open(
            'w', newline='', encoding='utf-8'
        )
        self.state_file = self.state_path.open(
            'w', newline='', encoding='utf-8'
        )
        self.distance_file = self.distance_path.open(
            'w', newline='', encoding='utf-8'
        )
        self.raw_writer = csv.writer(self.raw_file)
        self.filtered_writer = csv.writer(self.filtered_file)
        self.state_writer = csv.writer(self.state_file)
        self.distance_writer = csv.writer(self.distance_file)
        self.raw_first_timestamp = None
        self.filtered_first_timestamp = None
        self.state_first_timestamp = None
        self.distance_first_timestamp = None
        self.raw_writer.writerow(self.raw_header())
        self.filtered_writer.writerow(self.filtered_header())
        self.state_writer.writerow(
            self.state_header()
            + self.prediction_header()
        )
        self.distance_writer.writerow(self.distance_header())
        self.raw_file.flush()
        self.filtered_file.flush()
        self.state_file.flush()
        self.distance_file.flush()
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
        self.create_subscription(
            HandoverDistance,
            '/handover/distance',
            self.distance_callback,
            10,
        )
        self.get_logger().info(f'CSV raw: {self.raw_path}')
        self.get_logger().info(f'CSV filtered: {self.filtered_path}')
        self.get_logger().info(f'CSV state: {self.state_path}')
        self.get_logger().info(f'CSV distance: {self.distance_path}')

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
            'handedness',
            'handedness_name',
            'handedness_score',
            'palm_plane_valid',
            'palm_plane_normal_x',
            'palm_plane_normal_y',
            'palm_plane_normal_z',
            'palm_anchor_cross_x',
            'palm_anchor_cross_y',
            'palm_anchor_cross_z',
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
            'handedness',
            'handedness_name',
            'handedness_score',
            'palm_plane_valid',
            'palm_plane_normal_x',
            'palm_plane_normal_y',
            'palm_plane_normal_z',
            'palm_anchor_cross_x',
            'palm_anchor_cross_y',
            'palm_anchor_cross_z',
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
    def distance_header():
        return [
            'timestamp_s',
            'elapsed_s',
            'frame_id',
            'valid',
            'palm_x',
            'palm_y',
            'palm_z',
            'ee_x',
            'ee_y',
            'ee_z',
            'ee_to_palm_x',
            'ee_to_palm_y',
            'ee_to_palm_z',
            'distance_m',
            'distance_sigma_m',
            'rate_valid',
            'rate_source',
            'rate_degraded',
            'rate_age_s',
            'distance_rate_m_s',
            'closing_velocity_m_s',
            'hand_velocity_valid',
            'hand_velocity_source',
            'hand_velocity_estimator',
            'hand_velocity_age_s',
            'hand_vx',
            'hand_vy',
            'hand_vz',
            'ee_velocity_valid',
            'ee_velocity_age_s',
            'ee_vx',
            'ee_vy',
            'ee_vz',
            'relative_vx',
            'relative_vy',
            'relative_vz',
            'legacy_rate_valid',
            'legacy_distance_rate_m_s',
            'rate_consistency_error_m_s',
            'ttc_valid',
            'ttc_s',
            'legacy_ttc_valid',
            'legacy_ttc_s',
            'tracking_confidence',
            'motion_stability',
        ]

    @staticmethod
    def state_header():
        return [
            'timestamp_s',
            'elapsed_s',
            'frame_id',
            'valid',
            'position_valid',
            'position_fresh',
            'position_age_s',
            'runtime_bridge_inferred',
            'position_source_name',
            'velocity_valid',
            'velocity_source',
            'velocity_source_name',
            'velocity_estimator',
            'velocity_estimator_name',
            'velocity_age_s',
            'geometry_ok',
            'filter_state',
            'filter_state_name',
            'palm_x',
            'palm_y',
            'palm_z',
            'palm_vx',
            'palm_vy',
            'palm_vz',
            'longitudinal_x',
            'longitudinal_y',
            'longitudinal_z',
            'normal_x',
            'normal_y',
            'normal_z',
            'palm_var_x',
            'palm_var_y',
            'palm_var_z',
            'palm_speed',
            'tracking_confidence',
            'motion_stability',
            'processing_latency_ms',
        ]

    @staticmethod
    def prediction_header():
        """
        Prediction is logged in hand_state.csv itself.

        We only store the future palm because the four future
        landmarks are a rigid W75 translation of the already
        logged filtered landmarks and would duplicate data.
        """
        return [
            'prediction_valid',
            'prediction_source_name',
            'pred_h1_s',
            'pred_h1_palm_x',
            'pred_h1_palm_y',
            'pred_h1_palm_z',
            'pred_h2_s',
            'pred_h2_palm_x',
            'pred_h2_palm_y',
            'pred_h2_palm_z',
            'pred_h3_s',
            'pred_h3_palm_x',
            'pred_h3_palm_y',
            'pred_h3_palm_z',
        ]

    @staticmethod
    def prediction_row(msg):
        nan = float('nan')
        updated_source = int(
            getattr(
                HandState,
                'VELOCITY_SOURCE_UPDATED',
                1,
            )
        )
        position_valid = bool(
            getattr(
                msg,
                'position_valid',
                msg.valid,
            )
        )
        position_fresh = bool(
            getattr(
                msg,
                'position_fresh',
                position_valid,
            )
        )
        velocity_valid = bool(
            getattr(
                msg,
                'velocity_valid',
                msg.valid,
            )
        )
        velocity_source = int(
            getattr(
                msg,
                'velocity_source',
                updated_source,
            )
        )
        velocity_age_s = float(
            getattr(
                msg,
                'velocity_age_s',
                0.0,
            )
        )
        filter_tracking = (
            int(msg.filter_state)
            == int(
                HandTrackingFiltered.TRACKING
            )
        )
        prediction_valid = (
            position_valid
            and position_fresh
            and velocity_valid
            and velocity_source
                == updated_source
            and velocity_age_s
                <= 0.10 + 1e-9
            and filter_tracking
        )
        if not prediction_valid:
            return [
                0,
                'NONE',
                0.10,
                nan,
                nan,
                nan,
                0.20,
                nan,
                nan,
                nan,
                0.30,
                nan,
                nan,
                nan,
            ]
        px = float(msg.palm_position.x)
        py = float(msg.palm_position.y)
        pz = float(msg.palm_position.z)
        vx = float(msg.palm_velocity.x)
        vy = float(msg.palm_velocity.y)
        vz = float(msg.palm_velocity.z)
        h1 = 0.10
        h2 = 0.20
        h3 = 0.30
        return [
            1,
            'W75',
            h1,
            px + vx * h1,
            py + vy * h1,
            pz + vz * h1,
            h2,
            px + vx * h2,
            py + vy * h2,
            pz + vz * h2,
            h3,
            px + vx * h3,
            py + vy * h3,
            pz + vz * h3,
        ]

    def raw_callback(self, msg):
        timestamp = self.timestamp_s(msg)
        if self.raw_first_timestamp is None:
            self.raw_first_timestamp = timestamp
        state = int(msg.tracking_state)
        handedness = int(
            msg.handedness
        )
        row = [
            timestamp,
            timestamp - self.raw_first_timestamp,
            msg.header.frame_id,
            state,
            RAW_STATES.get(state, 'UNKNOWN'),
            float(msg.processing_latency_ms),
            handedness,
            HANDEDNESS_NAMES.get(
                handedness,
                'UNKNOWN',
            ),
            float(
                msg.handedness_score
            ),
            int(
                bool(
                    msg.palm_plane_valid
                )), float(msg.palm_plane_normal.x), float(msg.palm_plane_normal.y
            ), float(msg.palm_plane_normal.z), float(msg.palm_anchor_cross.x
            ), float(msg.palm_anchor_cross.y), float(msg.palm_anchor_cross.z),]
        for i in range(4):
            point = msg.positions[i]
            measurement_type = int(msg.measurement_type[i])
            row.extend([int(msg.landmark_ids[i]),
                float(point.x), float(point.y), float(point.z), int(msg.valid[i]),
                measurement_type, MEASUREMENT_TYPES.get(measurement_type, 'UNKNOWN',),])
        self.raw_writer.writerow(row)
        self.raw_file.flush()

    def filtered_callback(self, msg):
        timestamp = self.timestamp_s(msg)
        if self.filtered_first_timestamp is None:
            self.filtered_first_timestamp = timestamp
        state = int(msg.filter_state)
        handedness = int(msg.handedness)
        row = [timestamp, timestamp - self.filtered_first_timestamp,
            msg.header.frame_id, state, FILTER_STATES.get(state, 'UNKNOWN'),
            float(msg.processing_latency_ms), handedness, HANDEDNESS_NAMES.get(
                handedness, 'UNKNOWN',), float(msg.handedness_score), int(bool(msg.palm_plane_valid
                )), float(msg.palm_plane_normal.x), float(msg.palm_plane_normal.y
            ), float(msg.palm_plane_normal.z), float(msg.palm_anchor_cross.x
            ), float(msg.palm_anchor_cross.y), float(msg.palm_anchor_cross.z),]
        for i in range(4):
            point = msg.positions[i]
            velocity = msg.velocities[i]
            position_var = msg.position_variance[i]
            velocity_var = msg.velocity_variance[i]
            landmark_state = int(msg.landmark_state[i])
            measurement_type = int(msg.measurement_type[i])
            row.extend([int(msg.landmark_ids[i]), landmark_state, FILTER_STATES.get(
                    landmark_state, 'UNKNOWN',), int(msg.measurement_used[i]), measurement_type,
                MEASUREMENT_TYPES.get(measurement_type, 'UNKNOWN',), float(msg.mahalanobis_sq[i]),
                float(point.x), float(point.y), float(point.z), float(velocity.x),
                float(velocity.y), float(velocity.z), float(position_var.x), float(position_var.y),
                float(position_var.z), float(velocity_var.x), float(velocity_var.y),
                float(velocity_var.z), float(msg.age_s[i]), int(msg.missed_updates[i]),])
        self.filtered_writer.writerow(row)
        self.filtered_file.flush()

    def distance_callback(self, msg):
        timestamp = self.timestamp_s(msg)
        if self.distance_first_timestamp is None:
            self.distance_first_timestamp = timestamp
        row = [timestamp, timestamp - self.distance_first_timestamp,
            msg.header.frame_id, int(msg.valid), float(msg.palm_position.x),
            float(msg.palm_position.y), float(msg.palm_position.z), float(msg.ee_control_point.x),
            float(msg.ee_control_point.y), float(msg.ee_control_point.z),
            float(msg.ee_to_palm.x), float(msg.ee_to_palm.y), float(msg.ee_to_palm.z),
            float(msg.distance), float(msg.distance_sigma), int(msg.rate_valid),
            int(msg.rate_source), int(msg.rate_degraded), float(msg.rate_age_s),
            float(msg.distance_rate), float(msg.closing_velocity), int(msg.hand_velocity_valid),
            int(msg.hand_velocity_source), int(msg.hand_velocity_estimator),
            float(msg.hand_velocity_age_s), float(msg.hand_velocity.x), float(msg.hand_velocity.y),
            float(msg.hand_velocity.z), int(msg.ee_velocity_valid), float(msg.ee_velocity_age_s),
            float(msg.ee_velocity.x), float(msg.ee_velocity.y), float(msg.ee_velocity.z),
            float(msg.relative_velocity.x), float(msg.relative_velocity.y),
            float(msg.relative_velocity.z), int(msg.legacy_rate_valid),
            float(msg.legacy_distance_rate), float(msg.rate_consistency_error),
            int(msg.ttc_valid), float(msg.ttc), int(msg.legacy_ttc_valid),
            float(msg.legacy_ttc), float(msg.tracking_confidence), float(msg.motion_stability),]
        self.distance_writer.writerow(row)
        self.distance_file.flush()

    def state_callback(self, msg):
        timestamp = self.timestamp_s(msg)
        if self.state_first_timestamp is None:
            self.state_first_timestamp = timestamp
        state = int(msg.filter_state)
        velocity_source = int(msg.velocity_source)
        velocity_estimator = int(msg.velocity_estimator)
        position_valid = bool(msg.position_valid)
        position_fresh = bool(msg.position_fresh)
        hold_source = int(HandState.VELOCITY_SOURCE_HOLD)
        runtime_bridge_inferred = bool(
            position_valid and not position_fresh and int(msg.filter_state)
                == int(HandTrackingFiltered.PREDICT_ONLY) and bool(msg.velocity_valid
            ) and velocity_source == hold_source and not bool(msg.geometry_ok))
        if not position_valid:
            position_source_name = ('INVALID')
        elif position_fresh:
            position_source_name = ('MEASURED')
        elif runtime_bridge_inferred:
            position_source_name = ('PREDICTED_W75')
        else:
            position_source_name = ('DEGRADED_FILTERED')
        row = [
            timestamp, timestamp - self.state_first_timestamp, msg.header.frame_id, int(msg.valid),
            int(msg.position_valid), int(msg.position_fresh), float(msg.position_age_s),
            int(runtime_bridge_inferred), position_source_name, int(msg.velocity_valid),
            velocity_source, VELOCITY_SOURCES.get(velocity_source, 'UNKNOWN',), velocity_estimator,
            VELOCITY_ESTIMATORS.get(velocity_estimator, 'UNKNOWN',), float(msg.velocity_age_s),
            int(msg.geometry_ok), state, FILTER_STATES.get(state, 'UNKNOWN'),
            float(msg.palm_position.x), float(msg.palm_position.y), float(msg.palm_position.z),
            float(msg.palm_velocity.x), float(msg.palm_velocity.y), float(msg.palm_velocity.z),
            float(msg.palm_longitudinal.x), float(msg.palm_longitudinal.y),
            float(msg.palm_longitudinal.z), float(msg.palm_normal.x),
            float(msg.palm_normal.y), float(msg.palm_normal.z), float(msg.palm_position_variance.x),
            float(msg.palm_position_variance.y), float(msg.palm_position_variance.z),
            float(msg.palm_speed), float(msg.tracking_confidence),
            float(msg.motion_stability), float(msg.processing_latency_ms),]
        row.extend(self.prediction_row(msg))
        self.state_writer.writerow(row)
        self.state_file.flush()

    def destroy_node(self):
        if not self.raw_file.closed:
            self.raw_file.close()
        if not self.filtered_file.closed:
            self.filtered_file.close()
        if not self.state_file.closed:
            self.state_file.close()
        if not self.distance_file.closed:
            self.distance_file.close()
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
