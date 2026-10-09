#!/usr/bin/env python3
"""Landmark Kalman: HandTrackingRaw -> HandTrackingFiltered.

One constant-velocity Kalman filter per landmark (0, 5, 9, 17) with a Mahalanobis gate; a landmark
without accepted measurements for lost_timeout becomes LOST and restarts on the next one. A change of
physical hand restarts all filters. Palm shape recovery: see _palm_shape_recovery.
"""

import time

import numpy as np
import rclpy
from franka_msgs.msg import HandTrackingFiltered, HandTrackingRaw
from geometry_msgs.msg import Point, Vector3
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class KalmanFilter6D:
    """Constant-velocity Kalman filter: [px, py, pz, vx, vy, vz]."""

    def __init__(self, sigma_accel=1.0, velocity_sigma=0.5):
        self.x = np.zeros(6)
        self.P = np.eye(6)
        self.sigma_accel = sigma_accel
        self.velocity_sigma = velocity_sigma
        self.initialized = False

    def initialize(self, z, position_sigma):
        self.x[:] = 0.0
        self.x[:3] = z
        self.P = np.diag([
            position_sigma**2,
            position_sigma**2,
            position_sigma**2,
            self.velocity_sigma**2,
            self.velocity_sigma**2,
            self.velocity_sigma**2,
        ])
        self.initialized = True

    def predict(self, dt):
        if not self.initialized or dt <= 0.0:
            return
        I3 = np.eye(3)
        F = np.block([
            [I3, dt * I3],
            [np.zeros((3, 3)), I3],
        ])
        G = np.vstack([
            0.5 * dt**2 * I3,
            dt * I3,
        ])
        Q = (self.sigma_accel**2) * (G @ G.T)
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + Q

    def update(self, z, sigma_measurement, threshold):
        H = np.hstack([np.eye(3), np.zeros((3, 3))])
        R = (sigma_measurement**2) * np.eye(3)
        y = z - H @ self.x
        S = H @ self.P @ H.T + R
        d2 = float(y.T @ np.linalg.solve(S, y))
        if d2 > threshold:
            return False, d2
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        # Joseph covariance update.
        I = np.eye(6)
        A = I - K @ H
        self.P = A @ self.P @ A.T + K @ R @ K.T
        return True, d2

def rigid_fit(A, B):
    """Least-squares rotation R and translation t with B ~ A R^T + t (Kabsch)."""
    ca, cb = A.mean(axis=0), B.mean(axis=0)
    U, _, Vt = np.linalg.svd((A - ca).T @ (B - cb))
    D = np.diag([1.0, 1.0, np.sign(np.linalg.det(Vt.T @ U.T))])
    R = Vt.T @ D @ U.T
    return R, cb - R @ ca


class KalmanHand(Node):

    LANDMARK_IDS = [0, 5, 9, 17]

    def __init__(self):
        super().__init__('kalman_hand')
        param = lambda name: self.declare_parameter(  # config/hand_tracking.yaml, passed by the launch
            name, descriptor=ParameterDescriptor(dynamic_typing=True)).value
        self.direct_sigma = float(param('direct_sigma'))
        self.estimated_sigma = float(param('estimated_sigma'))
        self.lost_timeout = float(param('lost_timeout'))
        self.mahalanobis_threshold = float(
            param('mahalanobis_threshold')
        )
        sigma_accel = float(param('sigma_accel'))
        self.filters = [KalmanFilter6D(sigma_accel=sigma_accel) for _ in range(4)]
        self.shape_recovery = bool(param('shape_recovery'))
        self.shape_tolerance = float(param('shape_tolerance_m'))
        self.reacquire_max_speed = float(param('reacquire_max_speed_m_s'))
        self.rigid_fill_sigma = float(param('rigid_fill_sigma'))
        self.current_hand_side = HandTrackingRaw.HAND_UNKNOWN
        self._reset()
        self.publisher = self.create_publisher(HandTrackingFiltered, '/handover/hand_tracking_filtered', 10)
        self.subscription = self.create_subscription(HandTrackingRaw, '/handover/hand_tracking_raw', self.callback, 10)
        self.get_logger().info('Kalman hand node started')

    def _reset(self):
        for kf in self.filters:
            kf.initialized = False
        self.last_msg_time = None
        self.last_update_time = [None] * 4
        self.last_source = [HandTrackingFiltered.INVALID] * 4
        self.missed_updates = [0] * 4
        self.is_lost = [False] * 4
        self.palm_template = None        # palm shape: last frame with all four measurements accepted
        self.last_accepted_z = [None] * 4

    def _not_updated(self, i, now, states):
        """No accepted measurement: PREDICT_ONLY, LOST after lost_timeout."""
        self.missed_updates[i] += 1
        age = now - self.last_update_time[i]
        states[i] = HandTrackingFiltered.LOST if age > self.lost_timeout else HandTrackingFiltered.PREDICT_ONLY
        self.is_lost[i] = age > self.lost_timeout

    @staticmethod
    def stamp_to_seconds(stamp):
        return stamp.sec + stamp.nanosec * 1e-9

    def callback(self, raw):
        t0 = time.perf_counter()
        now = self.stamp_to_seconds(raw.header.stamp)
        # a different physical hand is a new trajectory: never gate it against the old one
        raw_side = int(raw.handedness)
        sides = (HandTrackingRaw.HAND_LEFT, HandTrackingRaw.HAND_RIGHT)
        if raw_side in sides:
            if self.current_hand_side in sides and raw_side != self.current_hand_side:
                self.get_logger().info(f'Kalman physical-hand reset: {int(self.current_hand_side)} -> {raw_side}')
                self._reset()
            self.current_hand_side = raw_side
        dt = 0.0 if self.last_msg_time is None else max(
            0.0, now - self.last_msg_time
        )
        self.last_msg_time = now
        states = [HandTrackingFiltered.UNINITIALIZED] * 4
        measurement_used = [False] * 4
        mahalanobis_sq = [-1.0] * 4
        measured = [None] * 4
        for i, kf in enumerate(self.filters):
            if kf.initialized:
                kf.predict(dt)
            valid_measurement = (
                raw.valid[i]
                and raw.measurement_type[i] in (HandTrackingRaw.DIRECT, HandTrackingRaw.ESTIMATED,))
            if valid_measurement:
                z = np.array([raw.positions[i].x, raw.positions[i].y, raw.positions[i].z,])
                measured[i] = z
                sigma = (self.direct_sigma
                    if raw.measurement_type[i] == HandTrackingRaw.DIRECT else self.estimated_sigma)
                # First observation or recovery after LOST.
                if not kf.initialized or self.is_lost[i]:
                    kf.initialize(z, sigma)
                    accepted = True
                    self.is_lost[i] = False
                else:
                    accepted, d2 = kf.update(z, sigma, self.mahalanobis_threshold,)
                    mahalanobis_sq[i] = d2
                if accepted:
                    self.last_update_time[i] = now
                    self.last_source[i] = raw.measurement_type[i]
                    self.missed_updates[i] = 0
                    measurement_used[i] = True
                    states[i] = HandTrackingFiltered.TRACKING
                else:
                    self._not_updated(i, now, states)
            elif kf.initialized:
                self._not_updated(i, now, states)
        if self.shape_recovery:
            self._palm_shape_recovery(now, measured, raw.measurement_type, states, measurement_used)
        msg = HandTrackingFiltered()
        msg.header = raw.header
        # palm plane: passed through unchanged
        msg.handedness = int(raw.handedness)
        msg.handedness_score = float(raw.handedness_score)
        msg.palm_plane_valid = bool(raw.palm_plane_valid)
        msg.palm_plane_normal = (raw.palm_plane_normal)
        msg.palm_anchor_cross = (raw.palm_anchor_cross)
        msg.landmark_ids = [int(v) for v in self.LANDMARK_IDS]
        msg.landmark_state = [int(v) for v in states]
        msg.measurement_used = [bool(v) for v in measurement_used]
        msg.measurement_type = [int(v) for v in self.last_source]
        msg.mahalanobis_sq = [float(v) for v in mahalanobis_sq]
        msg.missed_updates = [int(v) for v in self.missed_updates]
        ages = []
        for i, kf in enumerate(self.filters):
            if not kf.initialized:
                msg.positions[i] = Point()
                msg.velocities[i] = Vector3()
                msg.position_variance[i] = Vector3()
                msg.velocity_variance[i] = Vector3()
                ages.append(0.0)
                continue
            msg.positions[i] = Point(x=float(kf.x[0]), y=float(kf.x[1]), z=float(kf.x[2]),)
            msg.velocities[i] = Vector3(x=float(kf.x[3]), y=float(kf.x[4]), z=float(kf.x[5]),)
            msg.position_variance[i] = Vector3(
                x=float(kf.P[0, 0]), y=float(kf.P[1, 1]), z=float(kf.P[2, 2]),)
            msg.velocity_variance[i] = Vector3(
                x=float(kf.P[3, 3]), y=float(kf.P[4, 4]), z=float(kf.P[5, 5]),)
            ages.append(float(now - self.last_update_time[i]))
        msg.age_s = [float(v) for v in ages]
        if all(s == HandTrackingFiltered.UNINITIALIZED for s in states):
            msg.filter_state = HandTrackingFiltered.UNINITIALIZED
        elif all(s == HandTrackingFiltered.TRACKING for s in states):
            msg.filter_state = HandTrackingFiltered.TRACKING
        elif any(s in (
                HandTrackingFiltered.TRACKING, HandTrackingFiltered.PREDICT_ONLY,) for s in states):
            msg.filter_state = HandTrackingFiltered.PREDICT_ONLY
        else:
            msg.filter_state = HandTrackingFiltered.LOST
        msg.processing_latency_ms = float((time.perf_counter() - t0) * 1000.0)
        self.publisher.publish(msg)

    def _palm_shape_recovery(self, now, measured, types, states, used):
        """Wrist and MCP 5/9/17 sit on the metacarpals, which move almost as one rigid body
        (rigid-body gap filling of motion capture). Template = last frame with all four
        measurements accepted.

        - A measurement rejected by the gate that keeps the palm's shape with the others
          (rigid fit within shape_tolerance_m) and is physically reachable from its last
          accepted position (reacquire_max_speed_m_s) is right: the filter lagged a fast
          motion and, left alone, keeps extrapolating away until lost_timeout. Re-acquire it
          on the measurement, with the velocity of the accepted landmarks of the same hand.
        - With three good landmarks, a missing / inconsistent one is rebuilt from the
          template and fed as a pseudo-measurement (rigid_fill_sigma, normal gate).
        """
        template = self.palm_template
        if template is not None:
            keys = [i for i in range(4) if measured[i] is not None]
            while len(keys) >= 3:  # drop the worst point until the rest is one rigid palm
                A = template[keys]; B = np.array([measured[i] for i in keys])
                R, t = rigid_fit(A, B)
                residual = np.linalg.norm(A @ R.T + t - B, axis=1)
                if residual.max() <= self.shape_tolerance:
                    break
                keys.pop(int(np.argmax(residual)))
            if len(keys) >= 3:
                velocities = [self.filters[i].x[3:].copy() for i in range(4) if used[i]]
                hand_velocity = np.mean(velocities, axis=0) if velocities else None
                for i in keys:
                    kf = self.filters[i]
                    if used[i] or not kf.initialized or self.last_accepted_z[i] is None:
                        continue
                    reach = 0.05 + self.reacquire_max_speed * (now - self.last_update_time[i])
                    if np.linalg.norm(measured[i] - self.last_accepted_z[i]) > reach:
                        continue
                    kf.initialize(measured[i], self.direct_sigma)
                    if hand_velocity is not None:
                        kf.x[3:] = hand_velocity
                    self.last_update_time[i] = now
                    self.last_source[i] = types[i]
                    self.missed_updates[i] = 0
                    self.is_lost[i] = False
                    used[i] = True
                    states[i] = HandTrackingFiltered.TRACKING
            good = [i for i in range(4) if used[i]]
            if len(good) == 3:
                A = template[good]; B = np.array([measured[i] for i in good])
                R, t = rigid_fit(A, B)
                if np.linalg.norm(A @ R.T + t - B, axis=1).max() <= self.shape_tolerance:
                    i = next(j for j in range(4) if not used[j])
                    kf = self.filters[i]
                    if kf.initialized and not self.is_lost[i]:
                        accepted, _ = kf.update(R @ template[i] + t, self.rigid_fill_sigma, self.mahalanobis_threshold)
                        if accepted:
                            self.last_update_time[i] = now
                            self.last_source[i] = HandTrackingRaw.ESTIMATED
                            self.missed_updates[i] = 0
                            states[i] = HandTrackingFiltered.TRACKING
        if all(used):
            self.palm_template = np.array(measured)
        for i in range(4):
            if used[i]:
                self.last_accepted_z[i] = measured[i]


def main(args=None):
    rclpy.init(args=args)
    node = KalmanHand()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):  # Ctrl+C / launch shutdown
        pass
    except Exception:
        if rclpy.ok():  # otherwise a shutdown race (context already gone)
            raise
    node.destroy_node()
    if rclpy.ok():  # Ctrl+C from launch already shut the context down
        rclpy.shutdown()


if __name__ == '__main__':
    main()
