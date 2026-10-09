#!/usr/bin/env python3
"""CSV logger: one folder per run, results/<date_time>/ (output_root), five files.

  hand_tracking_raw.csv       HandTrackingRaw        hand_tracking_filtered.csv  HandTrackingFiltered
  hand_state.csv              HandState (+ W75 constant-velocity palm at 0.1 / 0.2 / 0.3 s)
  handover_distance.csv       HandoverDistance       hand_object.csv             HandObjectState
Every file starts with timestamp_s, elapsed_s (from its first row), frame_id; the other columns are
listed below as (name, value of the message). The column names are read by the evaluation scripts.
"""

import csv
from datetime import datetime
from pathlib import Path
from zoneinfo import ZoneInfo

import rclpy
from franka_msgs.msg import HandObjectState, HandoverDistance, HandState, HandTrackingFiltered, HandTrackingRaw
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

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

POSITION_SOURCES = {
    HandState.POSITION_SOURCE_NONE: 'INVALID',
    HandState.POSITION_SOURCE_MEASURED: 'MEASURED',
    HandState.POSITION_SOURCE_DEGRADED_FILTERED: 'DEGRADED_FILTERED',
    HandState.POSITION_SOURCE_PREDICTED_W75: 'PREDICTED_W75',
}


def xyz(name, get):
    return [(f'{name}{a}', lambda m, a=a: float(getattr(get(m), a))) for a in 'xyz']


def pair(name, get, names):
    """Integer code and its name."""
    return [(name, lambda m: int(get(m))), (f'{name}_name', lambda m: names.get(int(get(m)), 'UNKNOWN'))]


def hand(state_name, get_state, states):
    return (pair(state_name, get_state, states) + [('processing_latency_ms', lambda m: float(m.processing_latency_ms))]
            + pair('handedness', lambda m: m.handedness, HANDEDNESS_NAMES)
            + [('handedness_score', lambda m: float(m.handedness_score)),
               ('palm_plane_valid', lambda m: int(bool(m.palm_plane_valid)))]
            + xyz('palm_plane_normal_', lambda m: m.palm_plane_normal) + xyz('palm_anchor_cross_', lambda m: m.palm_anchor_cross))


def landmarks(fields):
    """fields(i) -> columns of landmark i, for the four landmarks."""
    return [column for i in range(4) for column in fields(i)]


RAW_COLUMNS = hand('tracking_state', lambda m: m.tracking_state, RAW_STATES) + landmarks(lambda i: (
    [(f'landmark_{i}_id', lambda m: int(m.landmark_ids[i]))] + xyz(f'landmark_{i}_', lambda m: m.positions[i])
    + [(f'landmark_{i}_valid', lambda m: int(m.valid[i]))]
    + pair(f'landmark_{i}_measurement_type', lambda m: m.measurement_type[i], MEASUREMENT_TYPES)))
RAW_COLUMNS = [(n.replace('measurement_type_name', 'measurement_name'), g) for n, g in RAW_COLUMNS]

FILTERED_COLUMNS = hand('filter_state', lambda m: m.filter_state, FILTER_STATES) + landmarks(lambda i: (
    [(f'landmark_{i}_id', lambda m: int(m.landmark_ids[i]))]
    + pair(f'landmark_{i}_state', lambda m: m.landmark_state[i], FILTER_STATES)
    + [(f'landmark_{i}_measurement_used', lambda m: int(m.measurement_used[i]))]
    + pair(f'landmark_{i}_measurement_type', lambda m: m.measurement_type[i], MEASUREMENT_TYPES)
    + [(f'landmark_{i}_mahalanobis_sq', lambda m: float(m.mahalanobis_sq[i]))]
    + xyz(f'landmark_{i}_', lambda m: m.positions[i]) + xyz(f'landmark_{i}_v', lambda m: m.velocities[i])
    + xyz(f'landmark_{i}_position_var_', lambda m: m.position_variance[i])
    + xyz(f'landmark_{i}_velocity_var_', lambda m: m.velocity_variance[i])
    + [(f'landmark_{i}_age_s', lambda m: float(m.age_s[i])), (f'landmark_{i}_missed_updates', lambda m: int(m.missed_updates[i]))]))
FILTERED_COLUMNS = [(n.replace('measurement_type_name', 'measurement_name'), g) for n, g in FILTERED_COLUMNS]

f, b = (lambda field: lambda m: float(getattr(m, field))), (lambda field: lambda m: int(getattr(m, field)))
DISTANCE_COLUMNS = (
    [('valid', b('valid'))] + xyz('palm_', lambda m: m.palm_position) + xyz('ee_', lambda m: m.ee_control_point)
    + xyz('ee_to_palm_', lambda m: m.ee_to_palm)
    + [('distance_m', f('distance')), ('distance_sigma_m', f('distance_sigma')), ('rate_valid', b('rate_valid')),
       ('rate_source', b('rate_source')), ('rate_degraded', b('rate_degraded')), ('rate_age_s', f('rate_age_s')),
       ('distance_rate_m_s', f('distance_rate')), ('closing_velocity_m_s', f('closing_velocity')),
       ('hand_velocity_valid', b('hand_velocity_valid')), ('hand_velocity_source', b('hand_velocity_source')),
       ('hand_velocity_estimator', b('hand_velocity_estimator')), ('hand_velocity_age_s', f('hand_velocity_age_s'))]
    + xyz('hand_v', lambda m: m.hand_velocity)
    + [('ee_velocity_valid', b('ee_velocity_valid')), ('ee_velocity_age_s', f('ee_velocity_age_s'))]
    + xyz('ee_v', lambda m: m.ee_velocity) + xyz('relative_v', lambda m: m.relative_velocity)
    + [('legacy_rate_valid', b('legacy_rate_valid')), ('legacy_distance_rate_m_s', f('legacy_distance_rate')),
       ('rate_consistency_error_m_s', f('rate_consistency_error')), ('ttc_valid', b('ttc_valid')), ('ttc_s', f('ttc')),
       ('legacy_ttc_valid', b('legacy_ttc_valid')), ('legacy_ttc_s', f('legacy_ttc')),
       ('tracking_confidence', f('tracking_confidence')), ('motion_stability', f('motion_stability'))])

OBJECT_COLUMNS = (
    [('valid', b('valid')), ('physical_hand', b('physical_hand')), ('object_present', b('object_present')),
     ('object_confidence', f('object_confidence')), ('object_age_s', f('object_age'))]
    + xyz('centroid_', lambda m: m.object_centroid_3d)
    + [(name, lambda m, k=k: float(m.bbox_px[k])) for k, name in enumerate(('bbox_u_min', 'bbox_v_min', 'bbox_u_max', 'bbox_v_max'))]
    + [(f'dim_{k}_m', lambda m, a=a: float(getattr(m.dimensions, a))) for k, a in ((1, 'x'), (2, 'y'), (3, 'z'))]
    + [('contour_points', lambda m: len(m.contour_px) // 2)])

STATE_COLUMNS = (
    [('valid', b('valid')), ('position_valid', b('position_valid')), ('position_fresh', b('position_fresh')),
     ('position_age_s', f('position_age_s')),
     ('runtime_bridge_inferred', lambda m: int(m.position_source == HandState.POSITION_SOURCE_PREDICTED_W75)),
     ('position_source_name', lambda m: POSITION_SOURCES.get(int(m.position_source), 'UNKNOWN')),
     ('velocity_valid', b('velocity_valid'))]
    + pair('velocity_source', lambda m: m.velocity_source, VELOCITY_SOURCES)
    + pair('velocity_estimator', lambda m: m.velocity_estimator, VELOCITY_ESTIMATORS)
    + [('velocity_age_s', f('velocity_age_s')), ('geometry_ok', b('geometry_ok'))]
    + pair('filter_state', lambda m: m.filter_state, FILTER_STATES)
    + xyz('palm_', lambda m: m.palm_position) + xyz('palm_v', lambda m: m.palm_velocity)
    + xyz('longitudinal_', lambda m: m.palm_longitudinal) + xyz('normal_', lambda m: m.palm_normal)
    + xyz('palm_var_', lambda m: m.palm_position_variance)
    + [('palm_speed', f('palm_speed')), ('tracking_confidence', f('tracking_confidence')),
       ('motion_stability', f('motion_stability')), ('processing_latency_ms', f('processing_latency_ms'))])
HORIZONS_S = (0.10, 0.20, 0.30)
PREDICTION_COLUMNS = ['prediction_valid', 'prediction_source_name'] + [
    name for k in (1, 2, 3) for name in (f'pred_h{k}_s', f'pred_h{k}_palm_x', f'pred_h{k}_palm_y', f'pred_h{k}_palm_z')]


def prediction_row(msg):
    """W75 constant-velocity palm at 0.1 / 0.2 / 0.3 s, from a fresh measured palm and an updated velocity."""
    valid = (msg.position_valid and msg.position_fresh and msg.velocity_valid
             and int(msg.velocity_source) == int(HandState.VELOCITY_SOURCE_UPDATED)
             and float(msg.velocity_age_s) <= 0.10 + 1e-9 and int(msg.filter_state) == int(HandTrackingFiltered.TRACKING))
    nan = float('nan')
    if not valid:
        return [0, 'NONE'] + [v for h in HORIZONS_S for v in (h, nan, nan, nan)]
    p, v = msg.palm_position, msg.palm_velocity
    px, py, pz, vx, vy, vz = float(p.x), float(p.y), float(p.z), float(v.x), float(v.y), float(v.z)
    return [1, 'W75'] + [c for h in HORIZONS_S for c in (h, px + vx * h, py + vy * h, pz + vz * h)]


class HandTrackingCsvLogger(Node):

    FILES = (('raw', 'hand_tracking_raw', HandTrackingRaw, '/handover/hand_tracking_raw', RAW_COLUMNS, None),
             ('filtered', 'hand_tracking_filtered', HandTrackingFiltered, '/handover/hand_tracking_filtered', FILTERED_COLUMNS, None),
             ('state', 'hand_state', HandState, '/handover/hand_state', STATE_COLUMNS, (PREDICTION_COLUMNS, prediction_row)),
             ('distance', 'handover_distance', HandoverDistance, '/handover/distance', DISTANCE_COLUMNS, None),
             ('object', 'hand_object', HandObjectState, '/handover/hand_object', OBJECT_COLUMNS, None))

    def __init__(self):
        super().__init__('hand_tracking_csv_logger')
        output_root = self.declare_parameter('output_root', descriptor=ParameterDescriptor(dynamic_typing=True)).value
        run_dir = Path(output_root) / datetime.now(ZoneInfo('Europe/Rome')).strftime('%Y%m%d_%H%M%S')
        run_dir.mkdir(parents=True, exist_ok=True)
        self.files = []
        for key, name, msg_type, topic, columns, extra in self.FILES:
            path = run_dir / f'{name}.csv'
            stream = path.open('w', newline='', encoding='utf-8')
            writer = csv.writer(stream)
            writer.writerow(['timestamp_s', 'elapsed_s', 'frame_id'] + [c for c, _ in columns] + (extra[0] if extra else []))
            stream.flush()
            log = {'stream': stream, 'writer': writer, 'columns': columns, 'extra': extra and extra[1], 'first': None}
            self.files.append(stream)
            self.create_subscription(msg_type, topic, lambda msg, log=log: self.write(log, msg), 10)
            self.get_logger().info(f'CSV {key}: {path}')

    @staticmethod
    def write(log, msg):
        t = float(msg.header.stamp.sec) + 1e-9 * float(msg.header.stamp.nanosec)
        if log['first'] is None:
            log['first'] = t
        row = [t, t - log['first'], msg.header.frame_id] + [get(msg) for _, get in log['columns']]
        if log['extra']:
            row.extend(log['extra'](msg))
        log['writer'].writerow(row)
        log['stream'].flush()

    def destroy_node(self):
        for stream in self.files:
            if not stream.closed:
                stream.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = HandTrackingCsvLogger()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):  # Ctrl+C / launch shutdown
        pass
    except Exception:
        if rclpy.ok():  # otherwise a shutdown race (context already gone)
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
