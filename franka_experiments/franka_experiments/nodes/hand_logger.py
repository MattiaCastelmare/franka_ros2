#!/usr/bin/env python3
"""CSV logger: one folder per run, results/<date_time>/ (output_root), six files and abort.md.

  hand_tracking_raw.csv       HandTrackingRaw        hand_tracking_filtered.csv  HandTrackingFiltered
  hand_state.csv              HandState (+ W75 constant-velocity palm at 0.1 / 0.2 / 0.3 s)
  handover_distance.csv       HandoverDistance (+ HandoverObserver of the same stamp, written when it arrives)
  hand_object.csv             HandObjectState (+ last GraspPose: new from the backend or moved with the palm)
  end_effector_state.csv      EndEffectorState: robot joints and gripper tip (base frame), 100 Hz, from
                              the joint states of the real robot, of the bag or of the simulated robot
                              (+ grasp executor phase, /handover/grasp_phase)
  abort.md                    grasp executor aborts: time, phase left, reason
Every file starts with timestamp_s, elapsed_s (from its first row), frame_id; the other columns are
listed below as (name, value of the message). The column names are read by the evaluation scripts.
"""

import csv
import math
from collections import deque
from datetime import datetime
from pathlib import Path
from zoneinfo import ZoneInfo

import rclpy
from franka_msgs.msg import (EndEffectorState, GraspPose, HandObjectState, HandoverDistance, HandoverObserver,
                             HandState, HandTrackingFiltered, HandTrackingRaw)
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

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

OBSERVER_STATES = {
    HandoverObserver.LOST: 'LOST',
    HandoverObserver.WARMUP: 'WARMUP',
    HandoverObserver.HOLD: 'HOLD',
    HandoverObserver.APPROACHING: 'APPROACHING',
    HandoverObserver.RETREATING: 'RETREATING',
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
    return (pair(state_name, get_state, states)
            + [('processing_latency_ms', lambda m: float(m.processing_latency_ms))]
            + pair('handedness', lambda m: m.handedness, HANDEDNESS_NAMES)
            + [('handedness_score', lambda m: float(m.handedness_score)),
               ('palm_plane_valid', lambda m: int(bool(m.palm_plane_valid)))]
            + xyz('palm_plane_normal_', lambda m: m.palm_plane_normal)
            + xyz('palm_anchor_cross_', lambda m: m.palm_anchor_cross))


def landmarks(fields):
    """fields(i) -> columns of landmark i, for the four landmarks."""
    return [column for i in range(4) for column in fields(i)]


RAW_COLUMNS = hand('tracking_state', lambda m: m.tracking_state, RAW_STATES) + landmarks(lambda i: (
    [(f'landmark_{i}_id', lambda m: int(m.landmark_ids[i]))]
    + xyz(f'landmark_{i}_', lambda m: m.positions[i])
    + [(f'landmark_{i}_valid', lambda m: int(m.valid[i]))]
    + pair(f'landmark_{i}_measurement_type', lambda m: m.measurement_type[i], MEASUREMENT_TYPES)))
RAW_COLUMNS = [(n.replace('measurement_type_name', 'measurement_name'), g) for n, g in RAW_COLUMNS]

FILTERED_COLUMNS = hand('filter_state', lambda m: m.filter_state, FILTER_STATES) + landmarks(lambda i: (
    [(f'landmark_{i}_id', lambda m: int(m.landmark_ids[i]))]
    + pair(f'landmark_{i}_state', lambda m: m.landmark_state[i], FILTER_STATES)
    + [(f'landmark_{i}_measurement_used', lambda m: int(m.measurement_used[i]))]
    + pair(f'landmark_{i}_measurement_type', lambda m: m.measurement_type[i], MEASUREMENT_TYPES)
    + [(f'landmark_{i}_mahalanobis_sq', lambda m: float(m.mahalanobis_sq[i]))]
    + xyz(f'landmark_{i}_', lambda m: m.positions[i])
    + xyz(f'landmark_{i}_v', lambda m: m.velocities[i])
    + xyz(f'landmark_{i}_position_var_', lambda m: m.position_variance[i])
    + xyz(f'landmark_{i}_velocity_var_', lambda m: m.velocity_variance[i])
    + [(f'landmark_{i}_age_s', lambda m: float(m.age_s[i])),
       (f'landmark_{i}_missed_updates', lambda m: int(m.missed_updates[i]))]))
FILTERED_COLUMNS = [(n.replace('measurement_type_name', 'measurement_name'), g) for n, g in FILTERED_COLUMNS]

f = (lambda field: lambda m: float(getattr(m, field)))  # float column of a message field
b = (lambda field: lambda m: int(getattr(m, field)))    # int / bool column of a message field

DISTANCE_COLUMNS = (
    [('valid', b('valid'))]
    + xyz('palm_', lambda m: m.palm_position)
    + xyz('ee_', lambda m: m.ee_control_point)
    + xyz('ee_to_palm_', lambda m: m.ee_to_palm)
    + [('distance_m', f('distance')), ('distance_sigma_m', f('distance_sigma')),
       ('rate_valid', b('rate_valid')), ('rate_source', b('rate_source')),
       ('rate_degraded', b('rate_degraded')), ('rate_age_s', f('rate_age_s')),
       ('distance_rate_m_s', f('distance_rate')), ('closing_velocity_m_s', f('closing_velocity')),
       ('hand_velocity_valid', b('hand_velocity_valid')), ('hand_velocity_source', b('hand_velocity_source')),
       ('hand_velocity_estimator', b('hand_velocity_estimator')),
       ('hand_velocity_age_s', f('hand_velocity_age_s'))]
    + xyz('hand_v', lambda m: m.hand_velocity)
    + [('ee_velocity_valid', b('ee_velocity_valid')), ('ee_velocity_age_s', f('ee_velocity_age_s'))]
    + xyz('ee_v', lambda m: m.ee_velocity)
    + xyz('relative_v', lambda m: m.relative_velocity)
    + [('legacy_rate_valid', b('legacy_rate_valid')), ('legacy_distance_rate_m_s', f('legacy_distance_rate')),
       ('rate_consistency_error_m_s', f('rate_consistency_error')),
       ('ttc_valid', b('ttc_valid')), ('ttc_s', f('ttc')),
       ('legacy_ttc_valid', b('legacy_ttc_valid')), ('legacy_ttc_s', f('legacy_ttc')),
       ('tracking_confidence', f('tracking_confidence')), ('motion_stability', f('motion_stability'))])

OBJECT_COLUMNS = (
    [('valid', b('valid')), ('physical_hand', b('physical_hand')), ('object_present', b('object_present')),
     ('object_confidence', f('object_confidence')), ('object_age_s', f('object_age'))]
    + xyz('centroid_', lambda m: m.object_centroid_3d)
    + [(name, lambda m, k=k: float(m.bbox_px[k]))
       for k, name in enumerate(('bbox_u_min', 'bbox_v_min', 'bbox_u_max', 'bbox_v_max'))]
    + [(f'dim_{k}_m', lambda m, a=a: float(getattr(m.dimensions, a)))
       for k, a in ((1, 'x'), (2, 'y'), (3, 'z'))]
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
    + xyz('palm_', lambda m: m.palm_position)
    + xyz('palm_v', lambda m: m.palm_velocity)
    + xyz('longitudinal_', lambda m: m.palm_longitudinal)
    + xyz('normal_', lambda m: m.palm_normal)
    + xyz('palm_var_', lambda m: m.palm_position_variance)
    + [('palm_speed', f('palm_speed')), ('tracking_confidence', f('tracking_confidence')),
       ('motion_stability', f('motion_stability')), ('processing_latency_ms', f('processing_latency_ms'))])

ROBOT_COLUMNS = (
    [('valid', b('valid'))]
    + [(f'q{i + 1}', lambda m, i=i: float(m.q[i])) for i in range(7)]
    + [(f'qdot{i + 1}', lambda m, i=i: float(m.qdot[i])) for i in range(7)]
    + xyz('tip_', lambda m: m.position)
    + xyz('tip_v', lambda m: m.velocity)
    + [('tip_speed', lambda m: math.hypot(m.velocity.x, m.velocity.y, m.velocity.z))])

OBSERVER_COLUMNS = ['observer_state', 'observer_state_name', 'observer_candidate', 'observer_closing_m_s']
GRASP_COLUMNS = (['grasp_valid', 'grasp_new', 'grasp_age_s', 'grasp_x', 'grasp_y', 'grasp_z',
                  'grasp_qx', 'grasp_qy', 'grasp_qz', 'grasp_qw', 'grasp_width_m', 'grasp_score', 'grasp_confidence'])


def seconds(stamp):
    return float(stamp.sec) + 1e-9 * float(stamp.nanosec)


HORIZONS_S = (0.10, 0.20, 0.30)
PREDICTION_COLUMNS = ['prediction_valid', 'prediction_source_name'] + [
    name for k in (1, 2, 3)
    for name in (f'pred_h{k}_s', f'pred_h{k}_palm_x', f'pred_h{k}_palm_y', f'pred_h{k}_palm_z')]


def prediction_row(msg):
    """W75 constant-velocity palm at 0.1 / 0.2 / 0.3 s, from a fresh measured palm and an updated velocity."""
    valid = (msg.position_valid and msg.position_fresh and msg.velocity_valid
             and int(msg.velocity_source) == int(HandState.VELOCITY_SOURCE_UPDATED)
             and float(msg.velocity_age_s) <= 0.10 + 1e-9
             and int(msg.filter_state) == int(HandTrackingFiltered.TRACKING))
    nan = float('nan')
    if not valid:
        return [0, 'NONE'] + [v for h in HORIZONS_S for v in (h, nan, nan, nan)]

    p, v = msg.palm_position, msg.palm_velocity
    px, py, pz, vx, vy, vz = float(p.x), float(p.y), float(p.z), float(v.x), float(v.y), float(v.z)
    return [1, 'W75'] + [c for h in HORIZONS_S for c in (h, px + vx * h, py + vy * h, pz + vz * h)]


class HandTrackingCsvLogger(Node):

    # (log key, file name, message type, topic, columns, extra (columns, row function or method name))
    FILES = (
        ('raw', 'hand_tracking_raw', HandTrackingRaw, '/handover/hand_tracking_raw', RAW_COLUMNS, None),
        ('filtered', 'hand_tracking_filtered', HandTrackingFiltered, '/handover/hand_tracking_filtered',
         FILTERED_COLUMNS, None),
        ('state', 'hand_state', HandState, '/handover/hand_state', STATE_COLUMNS,
         (PREDICTION_COLUMNS, prediction_row)),
        ('distance', 'handover_distance', HandoverDistance, '/handover/distance', DISTANCE_COLUMNS,
         (OBSERVER_COLUMNS, 'observer_row')),
        ('object', 'hand_object', HandObjectState, '/handover/hand_object', OBJECT_COLUMNS,
         (GRASP_COLUMNS, 'grasp_row')),
        ('robot', 'end_effector_state', EndEffectorState, '/handover/end_effector_state', ROBOT_COLUMNS,
         (['grasp_phase'], 'phase_row')))

    def __init__(self):
        super().__init__('hand_tracking_csv_logger')
        output_root = self.declare_parameter(
            'output_root', descriptor=ParameterDescriptor(dynamic_typing=True)).value
        run_dir = Path(output_root) / datetime.now(ZoneInfo('Europe/Rome')).strftime('%Y%m%d_%H%M%S')
        run_dir.mkdir(parents=True, exist_ok=True)

        self.files, self.logs = [], {}
        self.observer = self.grasp = None
        self.phase = 'NONE'
        self.distances = deque(maxlen=60)  # waiting for the observer of the same stamp
        for key, name, msg_type, topic, columns, extra in self.FILES:
            path = run_dir / f'{name}.csv'
            stream = path.open('w', newline='', encoding='utf-8')
            writer = csv.writer(stream)
            writer.writerow(['timestamp_s', 'elapsed_s', 'frame_id'] + [c for c, _ in columns]
                            + (extra[0] if extra else []))
            stream.flush()

            row = extra and (getattr(self, extra[1]) if isinstance(extra[1], str) else extra[1])
            log = {'stream': stream, 'writer': writer, 'columns': columns, 'extra': row, 'first': None}
            self.files.append(stream)
            self.logs[key] = log
            callback = self.on_distance if key == 'distance' else lambda msg, log=log: self.write(log, msg)
            self.create_subscription(msg_type, topic, callback, 10)
            self.get_logger().info(f'CSV {key}: {path}')

        self.create_subscription(HandoverObserver, '/handover/observer', self.on_observer, 10)
        self.create_subscription(GraspPose, '/handover/grasp_pose', lambda msg: setattr(self, 'grasp', msg), 10)
        self.create_subscription(String, '/handover/grasp_phase', self.on_phase, 10)
        self.aborts = (run_dir / 'abort.md').open('w', encoding='utf-8')
        self.aborts.write('# Abort della presa\n\n| t [s] | fase | motivo |\n|---|---|---|\n')
        self.aborts.flush()
        self.files.append(self.aborts)

    # ------------------------------------------------------------ joined columns
    def on_distance(self, msg):
        if len(self.distances) == self.distances.maxlen:  # no observer for 2 s: do not lose the row
            self.observer = None
            self.write(self.logs['distance'], self.distances.popleft())
        self.distances.append(msg)

    def on_observer(self, msg):
        """Distance rows up to this stamp: the one with the same stamp gets the observer, older ones none."""
        t = seconds(msg.header.stamp)
        while self.distances and seconds(self.distances[0].header.stamp) <= t:
            distance = self.distances.popleft()
            self.observer = msg if seconds(distance.header.stamp) == t else None
            self.write(self.logs['distance'], distance)

    def observer_row(self, msg):
        o = self.observer
        if o is None:
            return [-1, 'NONE', -1, float('nan')]
        return [int(o.state), OBSERVER_STATES.get(int(o.state), 'UNKNOWN'), int(o.candidate_state),
                float(o.closing_velocity)]

    def grasp_row(self, msg):
        g, t = self.grasp, seconds(msg.header.stamp)
        if g is None:
            return [0, 0] + [float('nan')] * (len(GRASP_COLUMNS) - 2)
        p, q = g.pose.position, g.pose.orientation
        new = seconds(g.source_stamp) in (0.0, seconds(g.header.stamp))  # 0: publisher without source_stamp
        return [int(t <= seconds(g.valid_until)), int(new), t - seconds(g.header.stamp),
                p.x, p.y, p.z, q.x, q.y, q.z, q.w, g.width, g.score, g.confidence]

    def phase_row(self, msg):
        return [self.phase]

    def on_phase(self, msg):
        """'PHASE|reason' from the grasp executor; a reason marks an abort."""
        phase, _, reason = msg.data.partition('|')
        if reason:
            t = self.get_clock().now().nanoseconds * 1e-9
            first = self.logs['robot']['first']
            self.aborts.write(f'| {t - first if first else t:.2f} | {self.phase} | {reason} |\n')
            self.aborts.flush()
        self.phase = phase

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
        while self.distances:  # observer never came
            self.observer = None
            self.write(self.logs['distance'], self.distances.popleft())
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
