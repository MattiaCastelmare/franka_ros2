#!/usr/bin/env python3
"""reflex_monitor: log every tick at which the real FR3 would have tripped (plan H5), and the RTF.

SIM only; passive (publishes diagnostics, commands nothing). For every sample
of /NS_1/franka/joint_states (1 kHz) it runs ``simexp.reflex_checks``:
velocity against libfranka's envelope, position limits, |tau| against the
effort limits (the abort reflexes: any one is a FAIL), and torque rate against
1000 N·m/s (on this robot libfranka's limiter CLIPS that, so it is counted
as ``torque_rate_clipped``, not as a trip). tau_total is the JSB effort
field (= tau_cmd + g_KDL(q), measured in Phase 1); tau_user = tau_total − g(q),
with g(q) from Pinocchio on the run's URDF at g = 9.8, the plugin's value.

Writes into ``run_dir``:
  reflex_trips.csv      one row per event (tick, joint, kind: trips and clips)
  reflex_summary.json   counts, worst ratios, RTF; rewritten every 2 s and at exit
Publishes /sim/reflex (Float64MultiArray, 10 Hz):
  [n_trips, n_velocity, n_position, n_torque, n_torque_rate_clipped,
   worst_vel_ratio, worst_tau_ratio, worst_rate, rtf_last_window]
"""
import csv
import json
import os
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from simexp.reflex_checks import KINDS, ReflexChecker, RtfMeter, robot_limits  # noqa: E402

ARM = [f'fr3_joint{i}' for i in range(1, 8)]


class ReflexMonitor(Node):
    def __init__(self):
        super().__init__('reflex_monitor')
        self.run_dir = self.declare_parameter('run_dir', '').value
        urdf_path = self.declare_parameter('urdf_path', '').value
        self.gravity = float(self.declare_parameter('gravity', 9.8).value)
        if not self.run_dir or not urdf_path:
            raise RuntimeError('reflex_monitor needs run_dir and urdf_path')
        import pinocchio as pin
        with open(urdf_path) as f:
            self.model = pin.buildModelFromXML(f.read())
        self.model.gravity.linear[:] = [0.0, 0.0, -self.gravity]
        self.data = self.model.createData()
        self.pin = pin
        self.qidx = [self.model.joints[self.model.getJointId(j)].idx_q for j in ARM]
        L = robot_limits()
        self.chk = ReflexChecker(L['q_min'], L['q_max'], L['tau_max'])
        self.rtf = RtfMeter(1.0)
        self._csv = open(os.path.join(self.run_dir, 'reflex_trips.csv'), 'w', newline='')
        self._w = csv.writer(self._csv)
        self._w.writerow(['t_sim', 'kind', 'joint', 'value', 'limit'])
        self._first_t = None
        self._last_t = None
        self.pub = self.create_publisher(Float64MultiArray, '/sim/reflex', 10)
        self.create_subscription(JointState, '/NS_1/franka/joint_states', self._js, 100)
        self.create_subscription(Clock, '/clock', self._on_clock, 10)
        self.create_timer(0.1, self._publish)
        self.create_timer(2.0, self.write_summary)
        self.get_logger().info(f'SIM reflex monitor: limits tau_max={L["tau_max"].tolist()} '
                               f'rate_max={self.chk.rate_max} N·m/s -> {self.run_dir}')

    def _on_clock(self, msg):
        self.rtf.push(msg.clock.sec + msg.clock.nanosec * 1e-9, time.monotonic())

    def _js(self, msg):
        d = {n: i for i, n in enumerate(msg.name)}
        if not all(n in d for n in ARM) or not msg.effort:
            return
        idx = [d[n] for n in ARM]
        q = np.array([msg.position[i] for i in idx])
        qd = np.array([msg.velocity[i] for i in idx])
        tau = np.array([msg.effort[i] for i in idx])
        qp = self.pin.neutral(self.model)
        qp[self.qidx] = q
        g = self.pin.computeGeneralizedGravity(self.model, self.data, qp)[self.qidx]
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self._first_t is None:
            self._first_t = t
        self._last_t = t
        for kind, j, val, lim in self.chk.check(t, q, qd, tau, tau - g):
            self._w.writerow([f'{t:.4f}', kind, ARM[j], f'{val:.6g}', f'{lim:.6g}'])

    def _publish(self):
        c, w = self.chk.counts, self.chk.worst
        last = self.rtf.windows[-1] if self.rtf.windows else float('nan')
        self.pub.publish(Float64MultiArray(data=[
            float(self.chk.n_trips), *[float(c[k]) for k in KINDS],
            w['vel_ratio'], w['tau_ratio'], w['rate_nms'], last]))

    def write_summary(self):
        self._csv.flush()
        # A monitor that saw (almost) nothing must not report PASS: it would
        # read as "no trip" when it means "no data". 1 kHz for >= 90 % of the
        # sim time it ran is the bar.
        sim_span = (self._last_t - self._first_t) if self._first_t is not None else 0.0
        enough = sim_span > 0.5 and self.chk.n_samples >= 0.9 * 1000.0 * sim_span
        verdict = ('FAIL' if self.chk.n_trips else ('PASS' if enough else 'NO_DATA'))
        out = {'SIM': True, 'plant': 'gazebo', 'n_samples': self.chk.n_samples,
               'sim_span_s': sim_span, 'n_trips': self.chk.n_trips, 'counts': self.chk.counts,
               'worst': self.chk.worst, 'verdict': verdict,
               'rtf': self.rtf.summary(), 'rtf_windows': self.rtf.windows[-600:],
               'limits': {'tau_max': self.chk.tau_max.tolist(), 'rate_max': self.chk.rate_max,
                          'vel_tol': self.chk.vel_tol, 'velocity': 'fr3_velocity_envelope(q, 1.0)'}}
        tmp = os.path.join(self.run_dir, '.reflex_summary.json.tmp')
        with open(tmp, 'w') as f:
            json.dump(out, f, indent=1, default=float)
        os.replace(tmp, os.path.join(self.run_dir, 'reflex_summary.json'))
        return out

    def finalise_manifest(self, out: dict) -> None:
        """Put the FINAL verdict, counts and RTF into run_manifest_sim.json.

        The episode supervisor writes the manifest when the episode ends, from
        the summary this node last wrote (up to 2 s old), and its exit is what
        shuts the launch down: this node's last summary always comes after it.
        """
        p = os.path.join(self.run_dir, 'run_manifest_sim.json')
        try:
            with open(p) as f:
                man = json.load(f)
        except (OSError, ValueError):
            return
        man.update({'rtf': out['rtf'], 'reflex_verdict': out['verdict'],
                    'reflex_counts': out['counts'], 'reflex_final': True})
        tmp = p + '.tmp'
        with open(tmp, 'w') as f:
            json.dump(man, f, indent=2, sort_keys=True, default=str)
        os.replace(tmp, p)


def main():
    rclpy.init()
    node = ReflexMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.finalise_manifest(node.write_summary())
        node._csv.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
