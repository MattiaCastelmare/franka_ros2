#!/usr/bin/env python3
"""Live readout of the tracked obstacle velocity, one line per control point.

WHAT IT IS FOR
--------------
Answering "is the tracker actually predicting the obstacle's velocity?" while
you wave something at the robot — before any of it is wired into the QP.
``ros2 topic echo`` on MultiLinkDistance is unreadable (11 entries × 10 fields
at 30 Hz); this prints the four numbers that matter and nothing else.

THE COLUMN THAT MATTERS IS ``v_obs``
------------------------------------
``v_obs = n̂ᵀ v_track``, with n̂ the message's ``direction`` (obstacle → control
point). That is the SAME scalar ``cbf_state_rows`` builds today as the residual
``aᵀq̇ − ḋ``, so it is directly comparable with what the filter already uses,
and it is the only component of the 3D velocity a barrier ever consumes.

Sign: POSITIVE = the obstacle is CLOSING on that control point. Negative =
receding, and step 7 will clamp it to zero — a receding obstacle may never
loosen a barrier.

``sigma`` is ``sqrt(n̂ᵀ P_vv n̂)``, the filter's own admitted uncertainty about
that same scalar. It is what step 8's margin is built from, and it is the thing
an EMA'd scalar cannot report at all. Watch it collapse as a track accumulates
frames, and inflate while a track coasts through an occlusion.

USAGE
-----
    python3 scripts/watch_obstacle_tracks.py
    python3 scripts/watch_obstacle_tracks.py --ros-args \\
        -p topic:=/cbf/per_link_distances_tracked \\
        -p rate_hz:=2.0 -p only_tracked:=true

Read-only: it subscribes and prints. It publishes nothing and cannot influence
the robot.
"""
from __future__ import annotations

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from franka_msgs.msg import MultiLinkDistance
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy


class WatchObstacleTracks(Node):

    def __init__(self):
        super().__init__('watch_obstacle_tracks')
        self.declare_parameter('topic', '/cbf/per_link_distances_tracked')
        self.declare_parameter('rate_hz', 2.0)
        self.declare_parameter('only_tracked', False)
        self.declare_parameter('gap_max_m', 10.0)

        topic = self.get_parameter('topic').value
        self._only_tracked = bool(self.get_parameter('only_tracked').value)
        self._gap_max = float(self.get_parameter('gap_max_m').value)

        self._msg = None
        self._n = 0
        self._n_prev = 0
        self._t_prev = None
        self._hz = 0.0

        self.create_subscription(
            MultiLinkDistance, topic, self._on_msg,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_timer(1.0 / max(0.1, float(self.get_parameter('rate_hz').value)),
                          self._print)
        self.get_logger().info(f'watching {topic}')

    def _on_msg(self, msg):
        self._msg = msg
        self._n += 1

    def _rate(self):
        """Messages per second, COUNTED over the print interval.

        Not an EMA on inter-arrival times: the subscription and the timer share
        one thread, so a slow print lets messages buffer and then be delivered
        back to back microseconds apart. An inter-arrival estimator reads that
        burst as kilohertz — it printed 1280 Hz on a 15 Hz bag replay — and a
        rate readout that lies is worse than none, because it is the first
        thing you look at to decide whether perception is keeping up.
        """
        now = self.get_clock().now().nanoseconds * 1e-9
        if self._t_prev is not None and now - self._t_prev > 1e-3:
            self._hz = (self._n - self._n_prev) / (now - self._t_prev)
        self._t_prev, self._n_prev = now, self._n
        return self._hz

    def _print(self):
        msg = self._msg
        if msg is None:
            self._rate()
            print('waiting for a message ...', flush=True)
            return
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        lines = [
            '',
            f'── {len(msg.links)} control points · {self._rate():.1f} Hz · '
            f'{self._n} msgs · stamp {stamp:.2f}',
            f'{"link":<14}{"gap m":>8}{"trk":>5}{"seen":>6}'
            f'{"v_obs":>9}{"sigma":>8}   {"|v| m/s":>8}  v_xyz (base)',
        ]
        n_trk = 0
        for ld in msg.links:
            if ld.track_id:
                n_trk += 1
            if self._only_tracked and not ld.track_id:
                continue
            if ld.distance > self._gap_max:
                continue
            n_hat = np.array([ld.direction.x, ld.direction.y, ld.direction.z])
            v = np.array([ld.obstacle_velocity.x, ld.obstacle_velocity.y,
                          ld.obstacle_velocity.z])
            P = np.asarray(ld.velocity_covariance, dtype=np.float64).reshape(3, 3)
            # The exact scalar the barrier would consume, and its uncertainty.
            v_obs = float(n_hat @ v)
            sigma = float(np.sqrt(max(n_hat @ P @ n_hat, 0.0)))
            flag = ' ' if ld.track_id else '·'
            lines.append(
                f'{ld.robot_link_name:<14}{ld.distance:>8.3f}'
                f'{ld.track_id:>5}{ld.frames_seen:>6}'
                f'{v_obs:>+9.3f}{sigma:>8.3f} {flag} {np.linalg.norm(v):>8.3f}  '
                f'[{v[0]:+.2f} {v[1]:+.2f} {v[2]:+.2f}]')
        lines.append(f'{n_trk}/{len(msg.links)} control points matched to a track'
                     '   (v_obs > 0 = CLOSING)')
        print('\n'.join(lines), flush=True)


def main(args=None):
    # ExternalShutdownException is what rclpy raises when the process is asked
    # to stop from outside (Ctrl-C forwarded as SIGINT/SIGTERM, `ros2 launch`
    # shutting the stack down, a bag replay ending). It is a normal exit, and
    # letting it escape prints a traceback that reads like a crash — which is
    # exactly the wrong thing to show someone who is watching this node to
    # decide whether the pipeline is trustworthy.
    rclpy.init(args=args)
    node = WatchObstacleTracks()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
