#!/usr/bin/env python3
"""Joint-state logger and plotter.

Subscribes to /joint_states and records position, velocity, effort for all 7
FR3 joints.  On shutdown (Ctrl-C):
    1. Saves a CSV to /tmp/trajectory_log_<timestamp>.csv
    2. Shows a matplotlib plot (set PLOT=False below to skip)

Usage alongside any trajectory node — just launch it in a separate terminal:
    ros2 run franka_simulation trajectory_plotter
"""

import csv
import datetime
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import JointState

PLOT = True   # set False to skip matplotlib at shutdown

_JOINTS = [
    'fr3_joint1', 'fr3_joint2', 'fr3_joint3', 'fr3_joint4',
    'fr3_joint5', 'fr3_joint6', 'fr3_joint7',
]


class TrajectoryPlotter(Node):
    def __init__(self):
        super().__init__('trajectory_plotter')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._sub = self.create_subscription(JointState, '/joint_states', self._cb, qos)

        self._rows: list[list] = []   # [t, q1..q7, dq1..dq7, tau1..tau7]
        self._t0: float | None = None
        self._idx: list[int] | None = None

        self.get_logger().info('trajectory_plotter recording /joint_states — Ctrl-C to save+plot')

    def _cb(self, msg: JointState):
        if self._idx is None:
            try:
                self._idx = [msg.name.index(n) for n in _JOINTS]
            except ValueError as e:
                self.get_logger().warn(f'missing joint in /joint_states: {e}', throttle_duration_sec=5.0)
                return

        now = time.time()
        if self._t0 is None:
            self._t0 = now
        t = now - self._t0

        def _get(field, idx):
            if len(field) > max(idx):
                return [field[i] for i in idx]
            return [0.0] * len(idx)

        q   = _get(msg.position, self._idx)
        dq  = _get(msg.velocity, self._idx)
        tau = _get(msg.effort,   self._idx)
        self._rows.append([t] + q + dq + tau)

    def save_csv(self) -> str:
        stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        path = f'/tmp/trajectory_log_{stamp}.csv'
        header = (
            ['time_s']
            + [f'q{i+1}' for i in range(7)]
            + [f'dq{i+1}' for i in range(7)]
            + [f'tau{i+1}' for i in range(7)]
        )
        with open(path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(header)
            w.writerows(self._rows)
        self.get_logger().info(f'saved {len(self._rows)} samples → {path}')
        return path

    def plot(self, csv_path: str):
        try:
            import matplotlib.pyplot as plt
            import csv as _csv
        except ImportError:
            self.get_logger().warn('matplotlib not installed — skipping plot')
            return

        with open(csv_path) as f:
            reader = _csv.DictReader(f)
            data = list(reader)

        if not data:
            return

        t = [float(r['time_s']) for r in data]

        fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
        for i in range(7):
            axes[0].plot(t, [float(r[f'q{i+1}'])   for r in data], label=f'j{i+1}')
            axes[1].plot(t, [float(r[f'dq{i+1}'])  for r in data], label=f'j{i+1}')
            axes[2].plot(t, [float(r[f'tau{i+1}']) for r in data], label=f'j{i+1}')

        axes[0].set_ylabel('position [rad]')
        axes[1].set_ylabel('velocity [rad/s]')
        axes[2].set_ylabel('effort [Nm]')
        axes[2].set_xlabel('time [s]')
        for ax in axes:
            ax.legend(loc='upper right', fontsize=7)
            ax.grid(True, alpha=0.3)
        fig.suptitle(f'FR3 joint states — {os.path.basename(csv_path)}')
        plt.tight_layout()
        plt.show()


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPlotter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node._rows:
            path = node.save_csv()
            if PLOT:
                node.plot(path)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
