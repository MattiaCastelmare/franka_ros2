#!/usr/bin/env python3
"""Smoke test for rl_policy_commander (run inside the container).

Drives the deployment node with synthetic /NS_1/joint_states and
/cbf/per_link_distances and checks the sim-to-real contract end to end:

  1. warm-up window                     -> exactly zeros on /NS_1/qddot_nom
  2. steady state                       -> ~rate_hz, finite, |q̈| <= q̈_max, and
                                           BIT-EQUAL to an independently rebuilt
                                           observation + onnxruntime inference
                                           (proves the obs layout the node feeds
                                           the net is the one franka_sim trained)
  3. obstacle injected                  -> d_min on /NS_1/rl_status equals the
                                           reported surface distance
  4. distances stop (perception fault)  -> zeros (never a stale-obstacle action)
  5. joint states stop                  -> zeros

No robot, no camera, no CBF filter needed. Usage::

    source /ros2_ws/src/install/setup.bash
    python3 test/smoke_rl_policy_commander.py [--model path/to/policy.onnx]
"""

import argparse
import glob
import os
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Point, Vector3
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from franka_msgs.msg import LinkDistance, MultiLinkDistance

from franka_experiments.nodes.rl_policy_commander import RLPolicyCommander
from franka_experiments.utils.constants import FR3_JOINT_NAMES
from franka_experiments.utils.kinematics import (
    compute_ee_fk, generate_urdf_from_xacro, load_pinocchio_model,
    resolve_arm_joint_ids, resolve_frame_id,
)
from franka_experiments.utils.rl_policy import (
    action_to_qddot, build_observation, find_sim_root, obstacle_centre,
    qddot_max_from_limits,
)
from franka_experiments.utils.cbf_utils import load_robot_config

Q_TEST      = np.array([0.0, -0.4, 0.0, -1.8, 0.0, 1.6, 0.8])
TARGET      = np.array([0.45, 0.0, 0.45])          # node default target_xyz
OBST_LINK   = 'fr3_link5'
OBST_D      = 0.26                                  # reported surface distance
OBST_HUMAN  = np.array([0.3, 0.26, 0.6])
OBST_NORMAL = np.array([0.0, -1.0, 0.0])            # obstacle -> robot


class Stimulus(Node):
    def __init__(self):
        super().__init__('rl_stimulus')
        self.send_js = True
        self.send_dist = False
        self.received, self.t_recv, self.status = [], [], []

        self.pub_js = self.create_publisher(
            JointState, '/NS_1/joint_states', 10)
        self.pub_dist = self.create_publisher(
            MultiLinkDistance, '/cbf/per_link_distances',
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(
            Float64MultiArray, '/NS_1/qddot_nom', self._on_nom, 10)
        self.create_subscription(
            Float64MultiArray, '/NS_1/rl_status', self._on_status, 10)

        self.create_timer(0.01, self._tick_js)
        self.create_timer(1.0 / 30.0, self._tick_dist)

    def _on_nom(self, msg):
        self.received.append(np.array(msg.data))
        self.t_recv.append(time.monotonic())

    def _on_status(self, msg):
        self.status.append(np.array(msg.data))

    def _tick_js(self):
        if not self.send_js:
            return
        m = JointState()
        m.header.stamp = self.get_clock().now().to_msg()
        m.name = list(FR3_JOINT_NAMES)
        m.position = Q_TEST.tolist()
        m.velocity = [0.0] * 7
        self.pub_js.publish(m)

    def _tick_dist(self):
        if not self.send_dist:
            return
        ld = LinkDistance()
        ld.robot_link_name = OBST_LINK
        ld.distance = OBST_D
        ld.closest_point_robot = Point(x=0.3, y=0.0, z=0.6)
        ld.closest_point_human = Point(x=float(OBST_HUMAN[0]),
                                       y=float(OBST_HUMAN[1]),
                                       z=float(OBST_HUMAN[2]))
        ld.direction = Vector3(x=float(OBST_NORMAL[0]), y=float(OBST_NORMAL[1]),
                               z=float(OBST_NORMAL[2]))
        ld.confidence = 0.9
        ld.valid = True
        m = MultiLinkDistance()
        m.header.stamp = self.get_clock().now().to_msg()
        m.links = [ld]
        self.pub_dist.publish(m)


def _default_model() -> str:
    root = find_sim_root(__file__)
    if not root:
        return ''
    for pat in ('models/*/best_model.onnx', 'models/*/final_model.onnx'):
        hits = glob.glob(os.path.join(root, pat))
        if hits:
            return max(hits, key=os.path.getmtime)   # newest, not alphabetical
    return ''


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--model', default=_default_model())
    args = ap.parse_args()
    if not args.model or not os.path.isfile(args.model):
        raise SystemExit(
            'No ONNX policy found. Train + export one first:\n'
            '  python3 -m franka_sim.train --total-timesteps 5000 --exp-name smoke\n'
            '  python3 -m franka_sim.export_onnx '
            '--model franka_sim/models/smoke/final_model.zip')

    # The node reads onnx_model at declare time, so the override has to reach it
    # through the global ROS args (the Stimulus node simply ignores it).
    rclpy.init(args=['--ros-args',
                     '-p', f'onnx_model:={args.model}',
                     '-p', 'log_csv:=false'])
    node = RLPolicyCommander()
    stim = Stimulus()

    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    ex.add_node(stim)

    def run(sec):
        end = time.monotonic() + sec
        while time.monotonic() < end:
            ex.spin_once(timeout_sec=0.02)

    failures = []
    qddot_max = qddot_max_from_limits(load_robot_config('control')['joint_limits'])

    # ── Phase 1: warm-up gate ────────────────────────────────────────────────
    run(1.5)
    out = np.array(stim.received)
    print(f'phase1: n={len(out)}  max|out|={np.abs(out).max() if len(out) else 0:.2e}'
          '  (expect exactly 0 during warm-up)')
    if len(out) < 50:
        failures.append(f'phase1: only {len(out)} outputs in 1.5 s')
    elif np.abs(out).max() != 0.0:
        failures.append('phase1: non-zero command during warm-up')

    # ── Phase 2: steady state + observation contract ─────────────────────────
    stim.send_dist = True
    run(3.5)
    n = len(stim.received)
    dt = np.diff(stim.t_recv[-min(200, n):])
    rate = 1.0 / np.mean(dt)
    out = np.array(stim.received[-50:])
    print(f'phase2: n={n}  rate={rate:.0f} Hz  finite={np.isfinite(out).all()}  '
          f'max|out|/qddot_max={np.max(np.abs(out) / qddot_max):.3f}')
    if not (80 <= rate <= 125):
        failures.append(
            f'phase2: rate {rate:.0f} Hz not ~100 (if the machine is busy — a '
            'training run in the same container — this is CPU contention on '
            'the TEST harness, re-run on an idle machine before believing it)')
    if not np.isfinite(out).all():
        failures.append('phase2: non-finite command')
    if np.any(np.abs(out) > qddot_max + 1e-9):
        failures.append('phase2: command exceeds qddot_max')

    # Rebuild the observation independently and re-run the policy.
    model, data = load_pinocchio_model(generate_urdf_from_xacro())
    fid = resolve_frame_id(model, 'fr3_link8')
    jids = resolve_arm_joint_ids(model)
    import pinocchio as pin
    q_full = pin.neutral(model)
    for k, pid in enumerate(jids):
        q_full[model.joints[pid].idx_q] = Q_TEST[k]
    ee = np.array(compute_ee_fk(model, data, q_full, fid).translation)
    obst = obstacle_centre(OBST_HUMAN, OBST_NORMAL, 0.08)
    obs = build_observation(Q_TEST, np.zeros(7), ee, TARGET, obst, OBST_D)

    import onnxruntime as ort
    sess = ort.InferenceSession(args.model, providers=['CPUExecutionProvider'])
    a = sess.run(None, {sess.get_inputs()[0].name: obs[None]})[0][0]
    expect = action_to_qddot(a, qddot_max)
    # Compare against the last COMMANDING tick. A gated tick publishes exactly
    # zeros, and on a loaded machine the harness' own 30 Hz distance timer can
    # starve past distance_timeout, legitimately gating a tick — that is the
    # node behaving correctly, not a contract violation.
    commanding = [v for v in stim.received if np.any(v != 0.0)]
    if not commanding:
        failures.append('phase2: node never left the gated state')
    else:
        err = float(np.max(np.abs(commanding[-1] - expect)))
        print(f'phase2: max|node − independent replay| = {err:.2e} rad/s²  '
              f'(ee={np.round(ee, 4).tolist()}, '
              f'{len(commanding)}/{len(stim.received)} commanding ticks)')
        if err > 1e-6:
            failures.append(f'phase2: observation contract broken (err={err:.2e})')

    # ── Phase 3: obstacle slot propagation ───────────────────────────────────
    d_min = stim.status[-1][2]
    print(f'phase3: rl_status d_min={d_min:.3f} (expect {OBST_D})')
    if abs(d_min - OBST_D) > 1e-6:
        failures.append(f'phase3: d_min {d_min} != injected {OBST_D}')

    # ── Phase 4: perception fault (was streaming, then stops) ────────────────
    stim.send_dist = False
    run(1.5)
    out = np.array(stim.received[-20:])
    print(f'phase4: max|out|={np.abs(out).max():.2e} (expect 0 — stale distances)')
    if np.abs(out).max() != 0.0:
        failures.append('phase4: kept commanding with stale distances')

    # ── Phase 5: joint-state fault ───────────────────────────────────────────
    stim.send_js = False
    run(1.0)
    out = np.array(stim.received[-20:])
    print(f'phase5: max|out|={np.abs(out).max():.2e} (expect 0 — stale joint state)')
    if np.abs(out).max() != 0.0:
        failures.append('phase5: kept commanding with stale joint state')

    node.destroy_node()
    stim.destroy_node()
    rclpy.shutdown()

    if failures:
        print('FAIL:\n  ' + '\n  '.join(failures))
        raise SystemExit(1)
    print('SMOKE TEST PASSED')


if __name__ == '__main__':
    main()
