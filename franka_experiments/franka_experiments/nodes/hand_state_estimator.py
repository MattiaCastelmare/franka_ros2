#!/usr/bin/env python3
"""Hand state: HandTrackingFiltered (Kalman landmarks 0, 5, 9, 17) -> HandState.

  position     palm = mean of MCP 5/9/17; valid if all three are TRACKING or PREDICT_ONLY, >= 1
               TRACKING and <= max_position_age_s old; fresh if all three are TRACKING
  velocity     W75 (utils/palm_velocity.py), fed with fresh palms only
  prediction   all three MCP lost: constant-velocity palm from the last fresh one, up to
               max_prediction_age_s, variance = Kalman + calibrated q97.5 error
  orientation  palm normal (anatomical sign, then temporal continuity) -> GUARD60 (a jump > 60 deg
               needs a second coherent frame) -> One-Euro; longitudinal axis = palm - wrist in the plane
A change of physical hand (LEFT <-> RIGHT) resets velocity, prediction and orientation.
"""

import time

import numpy as np
import rclpy
from franka_msgs.msg import HandState, HandTrackingFiltered
from geometry_msgs.msg import Point, Vector3
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from franka_experiments.utils.palm_velocity import (
    PREDICTION_CHI3_Q975_RADIUS, SOURCE_HOLD, W75VelocityEstimator, prediction_q975_error_m)

F = HandTrackingFiltered
SIDES = (F.HAND_LEFT, F.HAND_RIGHT)


def unit(vector):
    vector = np.asarray(vector, dtype=float)
    if not np.isfinite(vector).all():
        return None
    norm = float(np.linalg.norm(vector))
    return None if norm <= 1e-12 else vector / norm


def angle_deg(a, b):
    a, b = unit(a), unit(b)
    if a is None or b is None:
        return np.nan
    return float(np.degrees(np.arccos(float(np.clip(np.dot(a, b), -1.0, 1.0)))))


def filter_alpha(cutoff, dt):
    tau = 1.0 / (2.0 * np.pi * max(float(cutoff), 1e-6))
    return 1.0 / (1.0 + tau / dt)


def xyz(point):
    return np.array([point.x, point.y, point.z], dtype=float)


def vector3(v):
    return Vector3(x=float(v[0]), y=float(v[1]), z=float(v[2]))


class HandStateEstimator(Node):

    PALM_INDICES = (1, 2, 3)  # MCP 5, 9, 17 in HandTrackingFiltered
    NORMAL_INNOVATION_GATE_DEG = 60.0  # GUARD60
    NORMAL_CONFIRM_ANGLE_DEG = 35.0
    NORMAL_RESET_GAP_S = 0.30
    NORMAL_MIN_CUTOFF, NORMAL_BETA, NORMAL_D_CUTOFF = 1.0, 1.0, 1.0  # One-Euro

    def __init__(self):
        super().__init__('hand_state_estimator')
        param = lambda name: self.declare_parameter(  # config/hand_tracking.yaml, passed by the launch
            name, descriptor=ParameterDescriptor(dynamic_typing=True)).value
        self.reacquire_frames = int(param('reacquire_frames'))
        self.confidence_sigma = float(param('confidence_sigma'))
        self.stability_speed = float(param('stability_speed'))
        self.lost_timeout = float(param('lost_timeout'))
        self.max_position_age_s = float(param('max_position_age_s'))
        self.enable_prediction_bridge = bool(param('enable_prediction_bridge'))
        self.max_prediction_age_s = float(param('max_prediction_age_s'))
        if self.max_prediction_age_s <= 0.0:
            raise ValueError('max_prediction_age_s must be > 0')
        self.w75 = W75VelocityEstimator()
        self.last_timestamp_s = None
        self.reset_temporal_state()
        self.publisher = self.create_publisher(HandState, '/handover/hand_state', 10)
        self.create_subscription(HandTrackingFiltered, '/handover/hand_tracking_filtered', self.callback, 10)
        self.get_logger().info(
            f'Hand state estimator: W75, max_position_age_s={self.max_position_age_s:.3f}, '
            f'prediction_bridge={self.enable_prediction_bridge}, max_prediction_age_s={self.max_prediction_age_s:.3f}')

    def reset_temporal_state(self):
        self.w75.reset()
        self.good_frames, self.ready = 0, False
        self.anchor = None  # (t, palm, velocity) of the last fresh measured palm
        self.prev_normal = self.prev_longitudinal = None
        self.stable_handedness = F.HAND_UNKNOWN
        self.normal_state = None    # One-Euro state of the palm normal
        self.normal_pending = None  # first frame of a > 60 deg jump, waiting for confirmation

    # ------------------------------------------------------------ position
    def position_state(self, msg):
        states = [int(msg.landmark_state[i]) for i in self.PALM_INDICES]
        ages = [float(msg.age_s[i]) for i in self.PALM_INDICES]
        tracking = sum(s == F.TRACKING for s in states)
        ages_finite = all(np.isfinite(a) for a in ages)
        age = max(ages) if ages_finite else np.nan
        valid = (all(s in (F.TRACKING, F.PREDICT_ONLY) for s in states) and tracking >= 1 and ages_finite
                 and age <= self.max_position_age_s)
        fresh = valid and tracking == 3
        palm, variance = np.zeros(3, dtype=float), np.zeros(3, dtype=float)
        if valid:
            points = np.asarray([xyz(msg.positions[i]) for i in self.PALM_INDICES], dtype=float)
            if not np.isfinite(points).all():
                valid = fresh = False
            else:
                palm = np.mean(points, axis=0)
                variances = np.asarray([xyz(msg.position_variance[i]) for i in self.PALM_INDICES], dtype=float)
                variance = np.sum(variances, axis=0) / 9.0  # variance of the mean of 3 independent points
        return {'valid': bool(valid), 'fresh': bool(fresh), 'age_s': float(age), 'position': palm, 'variance': variance}

    # ------------------------------------------------------------ prediction bridge
    def update_prediction_anchor(self, now, position, velocity):
        """Only a fresh measured palm can become the anchor (never a predicted one)."""
        if not self.enable_prediction_bridge or not position['fresh'] or not velocity['available']:
            return
        p, v = np.asarray(position['position'], dtype=float), np.asarray(velocity['velocity'], dtype=float)
        if np.isfinite(p).all() and np.isfinite(v).all():
            self.anchor = (float(now), p.copy(), v.copy())

    def prediction_bridge_state(self, msg, now):
        """Constant-velocity palm while all MCP 5/9/17 are PREDICT_ONLY, within max_prediction_age_s."""
        if not self.enable_prediction_bridge or self.anchor is None:
            return None
        if not all(int(msg.landmark_state[i]) == F.PREDICT_ONLY for i in self.PALM_INDICES):
            return None
        ages = [float(msg.age_s[i]) for i in self.PALM_INDICES]
        if not all(np.isfinite(a) for a in ages):
            return None
        t0, p0, v0 = self.anchor
        age = float(now - t0)
        limit = self.max_prediction_age_s + 1e-9
        if age < -1e-9 or age > limit or max(ages) > limit:
            return None
        q975 = prediction_q975_error_m(age, float(np.linalg.norm(v0)))
        if not np.isfinite(q975):
            return None
        predicted = p0 + v0 * age
        if not np.isfinite(predicted).all():
            return None
        # the Kalman covariance already grows during PREDICT_ONLY: add the calibrated W75 error
        variances = np.asarray([xyz(msg.position_variance[i]) for i in self.PALM_INDICES], dtype=float)
        kalman = np.zeros(3, dtype=float) if not np.isfinite(variances).all() else np.sum(np.maximum(variances, 0.0), axis=0) / 9.0
        return {'valid': True, 'fresh': False, 'predicted': True, 'age_s': age, 'position': predicted,
                'variance': kalman + (q975 / PREDICTION_CHI3_Q975_RADIUS) ** 2, 'velocity': v0.copy()}

    # ------------------------------------------------------------ orientation
    def _reset_normal_filter(self, now, normal, handedness):
        normal = unit(normal)
        if normal is None:
            return None
        self.normal_state = {'timestamp': float(now), 'raw': normal.copy(), 'filtered': normal.copy(),
                             'derivative': np.zeros(3, dtype=float), 'handedness': int(handedness)}
        self.normal_pending = None
        return normal.copy()

    def _one_euro(self, now, normal):
        """One-Euro on the normal, keeping its sign (no flip towards the previous one)."""
        normal = unit(normal)
        if normal is None:
            return None
        state = self.normal_state
        if state is None:
            return normal.copy()
        dt = float(now) - float(state['timestamp'])
        if not np.isfinite(dt) or dt <= 0.0 or dt > self.NORMAL_RESET_GAP_S:
            return normal.copy()
        alpha_d = filter_alpha(self.NORMAL_D_CUTOFF, dt)
        derivative = alpha_d * ((normal - state['raw']) / dt) + (1.0 - alpha_d) * state['derivative']
        output = np.zeros(3, dtype=float)
        for axis in range(3):
            alpha = filter_alpha(self.NORMAL_MIN_CUTOFF + self.NORMAL_BETA * abs(derivative[axis]), dt)
            output[axis] = alpha * normal[axis] + (1.0 - alpha) * state['filtered'][axis]
        output = unit(output)
        if output is None:
            output = normal.copy()
        self.normal_state = {'timestamp': float(now), 'raw': normal.copy(), 'filtered': output.copy(),
                             'derivative': derivative.copy(), 'handedness': int(state['handedness'])}
        return output.copy()

    def _update_stable_handedness(self, msg, fresh):
        """Physical side from the tracker (body-aware); a confirmed change of hand resets everything."""
        if int(msg.filter_state) == int(F.LOST):
            self.stable_handedness = F.HAND_UNKNOWN
            self.normal_state = self.normal_pending = None
            return self.stable_handedness
        side, score = int(msg.handedness), float(msg.handedness_score)
        if (side not in SIDES or side == self.stable_handedness or int(msg.filter_state) != int(F.TRACKING)
                or not fresh or not np.isfinite(score) or score < 0.99):
            return self.stable_handedness
        if self.stable_handedness in SIDES:
            self.reset_temporal_state()
        self.stable_handedness = side
        self.normal_state = self.normal_pending = None
        return self.stable_handedness

    def _raw_normal(self, msg, hand, now):
        """Palm-plane normal with its sign: the anatomical anchor (RIGHT +, LEFT -) seeds the sign of a
        new episode, then the previous raw normal keeps it continuous (a real UP -> SIDE -> DOWN
        rotation is continuous). A normal already signed by the tracker (anchor = +-normal) is kept."""
        if not bool(msg.palm_plane_valid) or hand not in SIDES:
            return None
        plane, anchor = unit(xyz(msg.palm_plane_normal)), unit(xyz(msg.palm_anchor_cross))
        if plane is None or anchor is None:
            return None
        state = None if abs(float(np.dot(plane, anchor))) > 0.9999 else self.normal_state
        reference = None
        if state is not None:
            dt = float(now) - float(state['timestamp'])
            if (int(state['handedness']) == int(hand) and np.isfinite(dt)
                    and 0.0 < dt <= self.NORMAL_RESET_GAP_S):
                reference = unit(state['raw'])
        if reference is None:
            reference = anchor if hand == F.HAND_RIGHT else -anchor
        return -plane if np.dot(plane, reference) < 0.0 else plane

    def _longitudinal(self, msg, normal):
        """(palm - wrist) of the Kalman landmarks, projected in the palm plane."""
        try:
            wrist = xyz(msg.positions[0])
            palm = np.mean(np.asarray([xyz(msg.positions[i]) for i in self.PALM_INDICES], dtype=float), axis=0)
        except Exception:
            return None
        if not (np.isfinite(wrist).all() and np.isfinite(palm).all()):
            return None
        longitudinal = palm - wrist
        return unit(longitudinal - normal * np.dot(longitudinal, normal))

    def update_geometry(self, msg, position_valid, now, fresh):
        """-> (geometry_ok, longitudinal, normal). A jump > 60 deg is held one frame and accepted when
        the next frame confirms it (within 35 deg); HOLD is not geometry_ok."""
        hand = self._update_stable_handedness(msg, fresh)
        raw = self._raw_normal(msg, hand, now)
        action, filtered = 'NONE', None
        if position_valid and raw is not None:
            state = self.normal_state
            reset = state is None
            if not reset:
                dt = float(now) - float(state['timestamp'])
                reset = (not np.isfinite(dt) or dt <= 0.0 or dt > self.NORMAL_RESET_GAP_S
                         or int(state['handedness']) != int(hand))
            if reset:
                filtered, action = self._reset_normal_filter(now, raw, hand), 'RESET'
            elif (innovation := angle_deg(raw, state['filtered'])) > self.NORMAL_INNOVATION_GATE_DEG and np.isfinite(innovation):
                pending = self.normal_pending
                confirmed = False
                if pending is not None:
                    candidate = angle_deg(raw, pending['normal'])
                    confirmed = (int(pending['handedness']) == int(hand)
                                 and float(now) - float(pending['timestamp']) <= self.NORMAL_RESET_GAP_S
                                 and np.isfinite(candidate) and candidate <= self.NORMAL_CONFIRM_ANGLE_DEG)
                if confirmed:
                    filtered, action = self._one_euro(now, raw), 'CONFIRM'
                    self.normal_pending = None
                else:
                    self.normal_pending = {'timestamp': float(now), 'normal': raw.copy(), 'handedness': int(hand)}
                    filtered, action = state['filtered'].copy(), 'HOLD'
            else:
                self.normal_pending = None
                filtered, action = self._one_euro(now, raw), 'UPDATE'
        if filtered is None:  # no new geometry: previous orientation, for continuity only
            filtered = (self.normal_state['filtered'].copy() if self.normal_state is not None
                        else self.prev_normal.copy() if self.prev_normal is not None else np.zeros(3, dtype=float))
        longitudinal = self._longitudinal(msg, filtered) if np.linalg.norm(filtered) > 1e-12 else None
        if longitudinal is None:
            longitudinal = self.prev_longitudinal.copy() if self.prev_longitudinal is not None else np.zeros(3, dtype=float)
        geometry_ok = (action in ('RESET', 'UPDATE', 'CONFIRM') and np.linalg.norm(filtered) > 1e-12
                       and np.linalg.norm(longitudinal) > 1e-12)
        if geometry_ok:
            self.prev_normal, self.prev_longitudinal = filtered.copy(), longitudinal.copy()
            if not self.ready:
                self.good_frames += 1
                self.ready = self.good_frames >= self.reacquire_frames
        else:
            if not self.ready:
                self.good_frames = 0
            if int(msg.filter_state) == int(F.LOST):
                self.good_frames, self.ready = 0, False
        return bool(geometry_ok), longitudinal, filtered

    # ------------------------------------------------------------ quality
    def tracking_confidence(self, msg):
        """Mean landmark score (TRACKING 1, PREDICT_ONLY 0.6 fading with age, x0.7 if ESTIMATED)
        times exp(-mean sigma / confidence_sigma)."""
        scores = []
        for i, state in enumerate(msg.landmark_state):
            score = (1.0 if state == F.TRACKING else
                     0.6 * max(0.0, 1.0 - float(msg.age_s[i]) / self.lost_timeout) if state == F.PREDICT_ONLY else 0.0)
            if msg.measurement_type[i] == F.ESTIMATED:
                score *= 0.7
            scores.append(score)
        variances = [c for v in msg.position_variance for c in (v.x, v.y, v.z)]
        mean_sigma = np.sqrt(max(0.0, float(np.mean(variances))))
        return float(np.clip(np.mean(scores) * np.exp(-mean_sigma / self.confidence_sigma), 0.0, 1.0))

    # ------------------------------------------------------------ main
    def callback(self, msg):
        start = time.perf_counter()
        out = HandState()
        out.header = msg.header
        side = int(msg.handedness)
        out.physical_hand = side if side in SIDES else HandState.HAND_UNKNOWN
        out.position_source = HandState.POSITION_SOURCE_NONE
        out.filter_state = int(msg.filter_state)
        out.velocity_estimator = HandState.VELOCITY_ESTIMATOR_W75
        out.velocity_source = HandState.VELOCITY_SOURCE_NONE
        out.position_age_s = out.velocity_age_s = float('nan')
        if min(len(msg.positions), len(msg.landmark_state), len(msg.age_s), len(msg.position_variance),
               len(msg.measurement_type)) < 4:
            self.reset_temporal_state()
            out.processing_latency_ms = (time.perf_counter() - start) * 1000.0
            self.publisher.publish(out)
            return
        now = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        if self.last_timestamp_s is not None and now <= self.last_timestamp_s:
            self.reset_temporal_state()
        self.last_timestamp_s = now

        # measured palm; a confirmed change of physical hand resets W75 before its first sample
        measured = self.position_state(msg)
        score = float(msg.handedness_score)
        if (side in SIDES and self.stable_handedness in SIDES and side != self.stable_handedness
                and int(msg.filter_state) == int(F.TRACKING) and measured['fresh'] and np.isfinite(score) and score >= 0.99):
            previous = int(self.stable_handedness)
            self.reset_temporal_state()
            self.stable_handedness = side
            self.get_logger().info(f'HandState physical-hand reset: {previous} -> {side}')
        velocity = self.w75.update(now, measured['position'], measured['fresh'], measured['valid'])
        self.update_prediction_anchor(now, measured, velocity)
        position = dict(measured, predicted=False)
        velocity = dict(velocity)
        if not measured['valid'] and (predicted := self.prediction_bridge_state(msg, now)) is not None:
            # the velocity that generated the prediction, as an aged HOLD estimate
            position = predicted
            velocity = {'available': True, 'velocity': predicted['velocity'].copy(), 'source': SOURCE_HOLD,
                        'age_s': float(predicted['age_s'])}

        out.position_valid, out.position_fresh = bool(position['valid']), bool(position['fresh'])
        out.position_age_s = float(position['age_s'])
        out.position_source = (HandState.POSITION_SOURCE_NONE if not position['valid'] else
                               HandState.POSITION_SOURCE_PREDICTED_W75 if position['predicted'] else
                               HandState.POSITION_SOURCE_MEASURED if position['fresh'] else
                               HandState.POSITION_SOURCE_DEGRADED_FILTERED)
        if position['valid']:
            p = position['position']
            out.palm_position = Point(x=float(p[0]), y=float(p[1]), z=float(p[2]))
            out.palm_position_variance = vector3(position['variance'])
        out.velocity_valid = bool(velocity['available'])
        out.velocity_source = int(velocity['source'])
        out.velocity_age_s = float(velocity['age_s'])
        if velocity['available']:
            v = np.asarray(velocity['velocity'], dtype=float)
            speed = float(np.linalg.norm(v))
            out.palm_velocity, out.palm_speed = vector3(v), speed
            out.motion_stability = float(np.clip(1.0 - speed / self.stability_speed, 0.0, 1.0))

        if position['predicted']:  # orientation is not predicted: previous axes, geometry not ok
            geometry_ok = False
            longitudinal = self.prev_longitudinal.copy() if self.prev_longitudinal is not None else np.zeros(3, dtype=float)
            normal = self.prev_normal.copy() if self.prev_normal is not None else np.zeros(3, dtype=float)
        else:
            geometry_ok, longitudinal, normal = self.update_geometry(msg, position['valid'], now, measured['fresh'])
        out.geometry_ok = bool(geometry_ok)
        out.palm_longitudinal, out.palm_normal = vector3(longitudinal), vector3(normal)
        out.tracking_confidence = self.tracking_confidence(msg) if position['valid'] else 0.0
        out.valid = bool(out.position_valid and out.velocity_valid and out.geometry_ok and self.ready)
        if not out.velocity_valid:
            out.palm_speed = out.motion_stability = 0.0
        out.processing_latency_ms = (time.perf_counter() - start) * 1000.0
        self.publisher.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = HandStateEstimator()
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
