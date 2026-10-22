#!/usr/bin/env python3
"""ground_truth_node: true obstacle states and true per-control-point surface gaps (spec Phase 4).

SIM only; passive. Inputs: the obstacle poses and law velocities that the
ScenarioObstacles system publishes every physics step (bridged as PoseArray),
and the TRUE joint state (/NS_1/franka/joint_states, exact in sim).

On every joint-state sample (1 kHz) it computes the control points of
real_time_distance (``simexp.truth_geometry``: same function, same config)
and the signed gap of each one to each obstacle. Outputs:

  /sim/truth/obstacles  Float64MultiArray [t, n, (x, y, z, vx, vy, vz, r) * n]
  /sim/truth/cp_gaps    Float64MultiArray [t, n_cp, gap_0..gap_{n_cp-1}, min_gap, h_min]
                        (gap = min over obstacles; h = gap - d_safe), at 100 Hz
  <run_dir>/truth.npz   every 1 kHz sample: t, q, flange, cp positions, gaps
                        (n_cp, n_obs), obstacle p / v; labels and radii. Written
                        every 5 s and at exit.
"""
import json
import os
import sys

import numpy as np
import rclpy
from geometry_msgs.msg import PoseArray
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from simexp.truth_geometry import TruthGeometry, cp_label, load_perception_config, surface_gaps  # noqa: E402

ARM = [f'fr3_joint{i}' for i in range(1, 8)]


class GroundTruth(Node):
    def __init__(self):
        super().__init__('ground_truth')
        dp = self.declare_parameter
        self.run_dir = dp('run_dir', '').value
        urdf_path = dp('urdf_path', '').value
        self.obstacles = json.loads(dp('obstacles_json', '[]').value)
        self.d_safe = float(dp('d_safe', 0.15).value)
        pc = load_perception_config(dp('perception_config', '').value or None)
        with open(urdf_path) as f:
            self.geo = TruthGeometry(f.read(), pc)
        self.radii = np.array([o['radius'] for o in self.obstacles], float)
        self.n = len(self.obstacles)
        self.pos = self.vel = None
        self.t_pose = -1.0
        self.rows = {k: [] for k in ('t', 'q', 'flange', 'cp', 'gap', 'p', 'v', 't_pose')}
        self.labels = None
        self.cp_r = None
        self.pub_obs = self.create_publisher(Float64MultiArray, '/sim/truth/obstacles', 10)
        self.pub_gap = self.create_publisher(Float64MultiArray, '/sim/truth/cp_gaps', 10)
        self.create_subscription(PoseArray, '/sim/truth/obstacle_poses', self._poses, 50)
        self.create_subscription(PoseArray, '/sim/truth/obstacle_poses_vel', self._vels, 50)
        self.create_subscription(JointState, '/NS_1/franka/joint_states', self._js, 100)
        self.create_timer(0.01, self._publish)
        self.create_timer(5.0, self.save)
        self._last = None
        self.get_logger().info(f'SIM ground truth: {self.n} obstacle(s) -> {self.run_dir}/truth.npz')

    @staticmethod
    def _arr(msg):
        return np.array([[p.position.x, p.position.y, p.position.z] for p in msg.poses])

    def _poses(self, msg):
        self.pos = self._arr(msg)
        self.t_pose = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def _vels(self, msg):
        self.vel = self._arr(msg)

    def _js(self, msg):
        d = {n: i for i, n in enumerate(msg.name)}
        if not all(n in d for n in ARM):
            return
        q = np.array([msg.position[d[n]] for n in ARM])
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        cps = self.geo.control_points(q)
        if self.labels is None:
            self.labels = [cp_label(c) for c in cps]
            self.cp_r = np.array([c['radius'] for c in cps])
        P = np.array([c['point'] for c in cps])
        if self.n and self.pos is not None and len(self.pos) == self.n:
            gaps = surface_gaps(cps, self.pos, self.radii)
            p, v = self.pos.copy(), (self.vel.copy() if self.vel is not None and
                                     len(self.vel) == self.n else np.full((self.n, 3), np.nan))
        else:
            gaps = np.full((len(cps), max(self.n, 1)), np.inf)
            p = v = np.full((max(self.n, 1), 3), np.nan)
        flange = self.geo.flange(q)
        for k, val in (('t', t), ('q', q), ('flange', flange), ('cp', P), ('gap', gaps),
                       ('p', p), ('v', v), ('t_pose', self.t_pose)):
            self.rows[k].append(val)
        self._last = (t, gaps, p, v)

    def _publish(self):
        if self._last is None:
            return
        t, gaps, p, v = self._last
        gmin = gaps.min(axis=1)
        self.pub_gap.publish(Float64MultiArray(data=[
            t, float(len(gmin)), *[float(x) for x in gmin], float(gmin.min()),
            float(gmin.min() - self.d_safe)]))
        body = []
        for k in range(self.n):
            body += [*map(float, p[k]), *map(float, v[k]), float(self.radii[k])]
        self.pub_obs.publish(Float64MultiArray(data=[t, float(self.n), *body]))

    def save(self):
        if not self.rows['t'] or not self.run_dir:
            return
        tmp = os.path.join(self.run_dir, '.truth.tmp.npz')
        np.savez_compressed(
            tmp, **{k: np.asarray(v) for k, v in self.rows.items()},
            labels=np.asarray(self.labels or []), cp_radius=np.asarray(self.cp_r),
            obstacle_radius=self.radii, obstacle_names=np.asarray([o['name'] for o in self.obstacles]),
            d_safe=self.d_safe)
        os.replace(tmp, os.path.join(self.run_dir, 'truth.npz'))


def main():
    rclpy.init()
    node = GroundTruth()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.save()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
