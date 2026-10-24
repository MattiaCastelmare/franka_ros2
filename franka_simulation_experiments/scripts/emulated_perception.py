#!/usr/bin/env python3
"""emulated_perception: cbf_scenarios' perception emulation on the Gazebo plant (Phase 5, column 2).

SIM only; ``perception:=emulated`` replaces the simulated D455, the depth
adapter and real_time_distance with this node. Every ``1/fps`` of sim time it
publishes the frame CAPTURED ``t_cam`` earlier: the TRUE joint state and the
TRUE obstacle poses (ScenarioObstacles, 1 kHz) at the capture instant, taken
from a short history buffer, one ControlPointResult per control point
(simexp.emulation), through ``perception_msgs.build_cp_messages`` on
/cbf/per_link_distances (BEST_EFFORT, like real_time_distance), stamped with
the capture time. One timer at the frame rate: a 1 kHz publish timer cost the
QP loop CPU it does not cost on hardware. With no control point within max_thresh it
publishes the empty heartbeat real_time_distance publishes. Obstacles that are
still parked (before the episode start) are not in the frame.
"""
import json
import os
import sys
from collections import deque

import numpy as np
import rclpy
import yaml
from franka_msgs.msg import MultiLinkDistance
from geometry_msgs.msg import PoseArray
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import JointState

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from simexp import emulation as E  # noqa: E402
from simexp.truth_geometry import TruthGeometry  # noqa: E402

ARM = [f'fr3_joint{i}' for i in range(1, 8)]
PARKED_Z = -10.0       # ScenarioObstacles parks at z = -20 m


class EmulatedPerception(Node):
    def __init__(self):
        super().__init__('emulated_perception')
        dp = self.declare_parameter
        with open(dp('perception_config', '').value) as f:
            self.cfg = yaml.safe_load(f)
        with open(dp('urdf_path', '').value) as f:
            self.geo = TruthGeometry(f.read(), self.cfg)
        obstacles = json.loads(dp('obstacles_json', '[]').value)
        self.radii = np.array([o['radius'] for o in obstacles], float)
        self.fps = float(dp('fps', 90.0).value)
        self.t_cam = float(dp('t_cam', E.T_CAM).value)
        self.tracks = E.TrackEmulator(len(obstacles), 1.0 / self.fps,
                                      cam_noise=bool(dp('cam_noise', False).value),
                                      seed=int(dp('seed', 0).value))
        self.thresholds = self.cfg['distance']['thresholds']
        self.fallback = float(self.cfg['distance'].get('fallback_distance', 2.0))
        self.js = deque(maxlen=200)          # (t, q), 1 kHz: 200 ms of history
        self.poses = deque(maxlen=200)       # (t, P)
        self.vels = deque(maxlen=200)        # (t, V)
        self.n_frames = 0
        self.pub = self.create_publisher(
            MultiLinkDistance, '/cbf/per_link_distances',
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(JointState, '/NS_1/franka/joint_states', self._js, 10)
        self.create_subscription(PoseArray, '/sim/truth/obstacle_poses', self._poses, 10)
        self.create_subscription(PoseArray, '/sim/truth/obstacle_poses_vel', self._vels, 10)
        self.create_timer(1.0 / self.fps, self._tick)
        self.get_logger().info(f'SIM emulated perception: {len(obstacles)} obstacle(s), '
                               f'{self.fps:g} fps, t_cam {1e3 * self.t_cam:.0f} ms')

    @staticmethod
    def _t(msg):
        return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def _js(self, msg):
        d = {n: i for i, n in enumerate(msg.name)}
        if all(n in d for n in ARM):
            self.js.append((self._t(msg), np.array([msg.position[d[n]] for n in ARM])))

    @staticmethod
    def _arr(msg):
        return np.array([[p.position.x, p.position.y, p.position.z] for p in msg.poses])

    def _poses(self, msg):
        self.poses.append((self._t(msg), self._arr(msg)))

    def _vels(self, msg):
        self.vels.append((self._t(msg), self._arr(msg)))

    @staticmethod
    def _at(buf, t):
        """The newest sample stamped at or before t (else the oldest one)."""
        best = None
        for ts, x in buf:
            if ts <= t + 1e-9:
                best = x
            else:
                break
        return best if best is not None else (buf[0][1] if buf else None)

    def _tick(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if not self.js or now <= self.t_cam:
            return
        self.pub.publish(self._frame(now - self.t_cam))

    def _frame(self, t_cap):
        from franka_experiments.utils.perception_msgs import build_cp_messages
        stamp = Time(nanoseconds=int(round(t_cap * 1e9))).to_msg()
        q, pos, vel = self._at(self.js, t_cap), self._at(self.poses, t_cap), self._at(self.vels, t_cap)
        live = (np.flatnonzero(pos[:, 2] > PARKED_Z)
                if pos is not None and vel is not None
                and len(pos) == len(self.radii) == len(vel) else np.zeros(0, int))
        empty = MultiLinkDistance()
        empty.header.stamp = stamp
        empty.header.frame_id = self.cfg['robot']['base_frame']
        if not live.size:
            return empty
        cps = self.geo.control_points(q)
        results, ks, gaps = E.frame_results(cps, pos[live], self.radii[live])
        if not (gaps <= self.thresholds['max_thresh']).any():
            return empty
        _, mld = build_cp_messages(
            cp_results=results, n_pts=500, stamp=stamp,
            frame_id=self.cfg['robot']['base_frame'],
            segment_links=self.cfg['robot'].get('segment_links', []),
            thresholds=self.thresholds, fallback=self.fallback,
            zones=self.cfg.get('zones', {}))
        # obstacle k of the live subset -> its index in the full list (frames_seen)
        E.fill_track_fields(mld, results, [int(live[k]) for k in ks], vel,
                            self.tracks, t_cap)
        self.n_frames += 1
        return mld


def main():
    rclpy.init()
    node = EmulatedPerception()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
