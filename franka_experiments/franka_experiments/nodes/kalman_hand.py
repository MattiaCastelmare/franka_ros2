#!/usr/bin/env python3

import time
import numpy as np

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point, Vector3
from franka_msgs.msg import HandTrackingRaw, HandTrackingFiltered


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


class KalmanHand(Node):

    LANDMARK_IDS = [0, 5, 9, 17]

    def __init__(self):
        super().__init__('kalman_hand')
        
        self.declare_parameter('direct_sigma', 0.003)
        self.declare_parameter('estimated_sigma', 0.03)
        self.declare_parameter('sigma_accel', 8.0)
        self.declare_parameter('lost_timeout', 0.5)
        self.declare_parameter('mahalanobis_threshold', 11.345)

        self.direct_sigma = float(self.get_parameter('direct_sigma').value)
        self.estimated_sigma = float(self.get_parameter('estimated_sigma').value)
        self.lost_timeout = float(self.get_parameter('lost_timeout').value)
        self.mahalanobis_threshold = float(
            self.get_parameter('mahalanobis_threshold').value
        )

        sigma_accel = float(self.get_parameter('sigma_accel').value)

        self.filters = [
            KalmanFilter6D(sigma_accel=sigma_accel)
            for _ in range(4)
        ]

        self.last_msg_time = None
        self.last_update_time = [None] * 4
        self.last_source = [HandTrackingFiltered.INVALID] * 4
        self.missed_updates = [0] * 4
        self.is_lost = [False] * 4

        self.publisher = self.create_publisher(
            HandTrackingFiltered,
            '/handover/hand_tracking_filtered',
            10,
        )

        self.subscription = self.create_subscription(
            HandTrackingRaw,
            '/handover/hand_tracking_raw',
            self.callback,
            10,
        )

        self.get_logger().info('Kalman hand node started')

    @staticmethod
    def stamp_to_seconds(stamp):
        return stamp.sec + stamp.nanosec * 1e-9

    def callback(self, raw):
        t0 = time.perf_counter()
        now = self.stamp_to_seconds(raw.header.stamp)

        dt = 0.0 if self.last_msg_time is None else max(
            0.0, now - self.last_msg_time
        )
        self.last_msg_time = now

        states = [HandTrackingFiltered.UNINITIALIZED] * 4
        measurement_used = [False] * 4
        mahalanobis_sq = [-1.0] * 4

        for i, kf in enumerate(self.filters):

            if kf.initialized:
                kf.predict(dt)

            valid_measurement = (
                raw.valid[i]
                and raw.measurement_type[i] in (
                    HandTrackingRaw.DIRECT,
                    HandTrackingRaw.ESTIMATED,
                )
            )

            if valid_measurement:
                z = np.array([
                    raw.positions[i].x,
                    raw.positions[i].y,
                    raw.positions[i].z,
                ])

                sigma = (
                    self.direct_sigma
                    if raw.measurement_type[i] == HandTrackingRaw.DIRECT
                    else self.estimated_sigma
                )

                # First observation or recovery after LOST.
                if not kf.initialized or self.is_lost[i]:
                    kf.initialize(z, sigma)
                    accepted = True
                    self.is_lost[i] = False
                else:
                    accepted, d2 = kf.update(
                        z,
                        sigma,
                        self.mahalanobis_threshold,
                    )
                    mahalanobis_sq[i] = d2

                if accepted:
                    self.last_update_time[i] = now
                    self.last_source[i] = raw.measurement_type[i]
                    self.missed_updates[i] = 0
                    measurement_used[i] = True
                    states[i] = HandTrackingFiltered.TRACKING
                else:
                    self.missed_updates[i] += 1
                    age = now - self.last_update_time[i]
                    states[i] = (
                        HandTrackingFiltered.LOST
                        if age > self.lost_timeout
                        else HandTrackingFiltered.PREDICT_ONLY
                    )
                    self.is_lost[i] = age > self.lost_timeout

            elif kf.initialized:
                self.missed_updates[i] += 1
                age = now - self.last_update_time[i]
                states[i] = (
                    HandTrackingFiltered.LOST
                    if age > self.lost_timeout
                    else HandTrackingFiltered.PREDICT_ONLY
                )
                self.is_lost[i] = age > self.lost_timeout

        msg = HandTrackingFiltered()
        msg.header = raw.header
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

            msg.positions[i] = Point(
                x=float(kf.x[0]),
                y=float(kf.x[1]),
                z=float(kf.x[2]),
            )

            msg.velocities[i] = Vector3(
                x=float(kf.x[3]),
                y=float(kf.x[4]),
                z=float(kf.x[5]),
            )

            msg.position_variance[i] = Vector3(
                x=float(kf.P[0, 0]),
                y=float(kf.P[1, 1]),
                z=float(kf.P[2, 2]),
            )

            msg.velocity_variance[i] = Vector3(
                x=float(kf.P[3, 3]),
                y=float(kf.P[4, 4]),
                z=float(kf.P[5, 5]),
            )

            ages.append(float(now - self.last_update_time[i]))

        msg.age_s = [float(v) for v in ages]

        if all(s == HandTrackingFiltered.UNINITIALIZED for s in states):
            msg.filter_state = HandTrackingFiltered.UNINITIALIZED
        elif all(s == HandTrackingFiltered.TRACKING for s in states):
            msg.filter_state = HandTrackingFiltered.TRACKING
        elif any(
            s in (
                HandTrackingFiltered.TRACKING,
                HandTrackingFiltered.PREDICT_ONLY,
            )
            for s in states
        ):
            msg.filter_state = HandTrackingFiltered.PREDICT_ONLY
        else:
            msg.filter_state = HandTrackingFiltered.LOST

        msg.processing_latency_ms = float(
            (time.perf_counter() - t0) * 1000.0
        )

        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = KalmanHand()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()