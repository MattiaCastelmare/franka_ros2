"""rclpy helpers for the slow Gazebo tests: record topics, publish commands, wait on sim time."""
from __future__ import annotations

import threading
import time

import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState

ARM = [f'fr3_joint{i}' for i in range(1, 8)]


class Probe(Node):
    """Records /NS_1/franka/joint_states (stamp, q, qdot, effort) in joint order."""

    def __init__(self, name='sim_test_probe', use_sim_time=True):
        super().__init__(name, parameter_overrides=[
            Parameter('use_sim_time', Parameter.Type.BOOL, use_sim_time)])
        self.lock = threading.Lock()
        self.js = []
        self.record = True
        self.create_subscription(JointState, '/NS_1/franka/joint_states', self._js, 200)

    def _js(self, msg):
        if not self.record:
            return
        d = {n: i for i, n in enumerate(msg.name)}
        if not all(n in d for n in ARM):
            return
        idx = [d[n] for n in ARM]
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        with self.lock:
            self.js.append((t, np.array([msg.position[i] for i in idx]),
                            np.array([msg.velocity[i] for i in idx]),
                            np.array([msg.effort[i] for i in idx]) if msg.effort else
                            np.full(7, np.nan)))

    def snapshot(self, t_from=-np.inf, t_to=np.inf):
        with self.lock:
            rows = [r for r in self.js if t_from <= r[0] <= t_to]
        if not rows:
            return np.zeros(0), np.zeros((0, 7)), np.zeros((0, 7)), np.zeros((0, 7))
        t = np.array([r[0] for r in rows])
        return t, np.stack([r[1] for r in rows]), np.stack([r[2] for r in rows]), \
            np.stack([r[3] for r in rows])

    def sim_now(self) -> float:
        with self.lock:
            return self.js[-1][0] if self.js else -1.0


class Spinner:
    """Spin nodes in a background executor for the duration of a ``with`` block."""

    def __init__(self, *nodes):
        self.ex = MultiThreadedExecutor(num_threads=4)
        for n in nodes:
            self.ex.add_node(n)
        self.th = threading.Thread(target=self.ex.spin, daemon=True)

    def __enter__(self):
        self.th.start()
        return self

    def __exit__(self, *exc):
        self.ex.shutdown(timeout_sec=2.0)
        return False


def wait_until(pred, timeout_s, period=0.1):
    t_end = time.time() + timeout_s
    while time.time() < t_end:
        if pred():
            return True
        time.sleep(period)
    return False


def wait_sim(probe: Probe, dt: float, timeout_s: float = 120.0) -> float:
    """Block until sim time advanced by dt; return the new sim time."""
    t0 = probe.sim_now()
    ok = wait_until(lambda: probe.sim_now() >= t0 + dt, timeout_s)
    if not ok:
        raise TimeoutError(f'sim time did not advance {dt} s in {timeout_s} s wall '
                           f'(stuck at {probe.sim_now()})')
    return probe.sim_now()


def init_ros(domain_id: int = 77):
    import os
    os.environ['ROS_DOMAIN_ID'] = str(domain_id)
    if not rclpy.ok():
        rclpy.init()
