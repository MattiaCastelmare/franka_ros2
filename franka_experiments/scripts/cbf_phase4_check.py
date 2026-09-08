#!/usr/bin/env python3
"""Phase 4 acceptance checker for the surface-gap / multi-CP CBF changes.

Subscribes to the live pipeline (a rosbag replay is just live topics, so the
same script covers both the offline replay and the supervised hardware run) and
reports, against the Phase 4 checklist:

  1. no collision            min surface gap over the run stays > 0
  2. edge distance > 0       time spent at a clamped gap of exactly 0.0
  3. num_active_cps > 1      simultaneous multi-CP activation was observed
  4. log consistency         the CBFDIAG d_min the filter SHOULD now print

It also measures the three things this change put at risk:

  * the min_thresh publish hole (fr3_complete.yaml): empty MultiLinkDistance
    frames, and specifically empty frames that arrive right after the gap was
    seen inside min_thresh — that is the hole firing, not "no obstacle"
  * OSQP setup() churn: how often n_c changes, since a changed row count forces
    a fresh setup() instead of a warm-started update()
  * shared-slack coupling: one slack relaxes every row, so slack is reported
    against the number of simultaneously violated CPs

Nothing here writes to the robot; it is read-only telemetry.

Usage
-----
    # terminal 1 — the stack under test (or: ros2 bag play <bag>)
    ros2 launch franka_experiments torque_control_stack.launch.py

    # terminal 2
    python3 <ws>/src/franka_experiments/scripts/cbf_phase4_check.py --duration 120

Exits 0 if every criterion passes, 1 otherwise, so it can gate a replay run.
"""

import argparse
import sys
import time
from collections import Counter

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64MultiArray

from franka_msgs.msg import MultiLinkDistance

# cbf_status layout — see fr3_control.yaml "cbf_status contract".
I_NC, I_SLACK, I_FAULT, I_NACT = 0, 1, 2, 3


class Phase4Check(Node):

    def __init__(self, args):
        super().__init__('cbf_phase4_check')
        self._d_safe     = args.d_safe
        self._min_thresh = args.min_thresh
        self._t0         = time.monotonic()

        # ── per-link-distance stream ────────────────────────────────────────
        self.n_msgs        = 0
        self.n_empty       = 0
        self.n_empty_after_close = 0   # the min_thresh publish hole firing
        self.min_gap       = np.inf
        self.min_gap_t     = None
        self.n_zero_gap    = 0         # frames whose closest CP was clamped to 0
        self.entries_hist  = Counter()  # entries per message → count
        self.links_seen    = Counter()
        self.max_below_dsafe = 0       # most CPs simultaneously inside d_safe
        # Largest (centre distance - published gap) seen: the bias the barrier
        # used to carry. Reported so the CBFDIAG comparison has a scale.
        self.max_axis_bias = 0.0
        self._last_close   = False     # previous frame's closest CP < min_thresh

        # ── cbf_status stream ───────────────────────────────────────────────
        self.n_status      = 0
        self.nc_hist       = Counter()
        self.nc_changes    = 0
        self._prev_nc      = None
        self.max_nact      = 0
        self.nact_hist     = Counter()
        self.max_slack     = 0.0
        self.slack_at_max_nact = 0.0
        self.n_fault       = 0
        self.status_seen_4 = False     # publisher carries the new data[3]

        self.create_subscription(
            MultiLinkDistance, args.distance_topic, self._on_dist,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(
            Float64MultiArray, args.status_topic, self._on_status,
            QoSProfile(depth=1))

        self.get_logger().info(
            f'watching {args.distance_topic} + {args.status_topic}  '
            f'(d_safe={self._d_safe}  min_thresh={self._min_thresh})')

    # ── callbacks ───────────────────────────────────────────────────────────

    def _on_dist(self, msg: MultiLinkDistance):
        self.n_msgs += 1
        valid = [ld for ld in msg.links if ld.valid]

        if not valid:
            self.n_empty += 1
            # An empty frame right after the closest CP was inside min_thresh is
            # the publish gate dropping a contact-range frame, NOT "no obstacle
            # in range". Distinguishing the two is the whole point of counting.
            if self._last_close:
                self.n_empty_after_close += 1
            return

        gaps = np.array([ld.distance for ld in valid])
        gmin = float(gaps.min())
        self.entries_hist[len(valid)] += 1
        for ld in valid:
            self.links_seen[ld.robot_link_name] += 1

        if gmin < self.min_gap:
            self.min_gap   = gmin
            self.min_gap_t = time.monotonic() - self._t0
        if gmin <= 0.0:
            self.n_zero_gap += 1
        self._last_close = gmin < self._min_thresh

        n_below = int(np.count_nonzero(gaps < self._d_safe))
        self.max_below_dsafe = max(self.max_below_dsafe, n_below)

        # Centre distance vs published gap — the quantity the old barrier used
        # instead of ld.distance. Its size is what the Phase 1 fix recovered.
        for ld in valid:
            pr = ld.closest_point_robot
            ph = ld.closest_point_human
            centre = float(np.linalg.norm(
                [pr.x - ph.x, pr.y - ph.y, pr.z - ph.z]))
            if centre > 0.0:
                self.max_axis_bias = max(self.max_axis_bias,
                                         centre - ld.distance)

    def _on_status(self, msg: Float64MultiArray):
        d = msg.data
        if len(d) < 3:
            return
        self.n_status += 1
        nc = int(d[I_NC])
        self.nc_hist[nc] += 1
        if self._prev_nc is not None and nc != self._prev_nc:
            self.nc_changes += 1          # forces a fresh OSQP setup()
        self._prev_nc = nc

        self.max_slack = max(self.max_slack, float(d[I_SLACK]))
        if float(d[I_FAULT]) > 0.5:
            self.n_fault += 1

        if len(d) >= 4:
            self.status_seen_4 = True
            nact = int(d[I_NACT])
            self.nact_hist[nact] += 1
            if nact > self.max_nact:
                self.max_nact = nact
                self.slack_at_max_nact = float(d[I_SLACK])

    # ── report ──────────────────────────────────────────────────────────────

    def report(self) -> bool:
        el = time.monotonic() - self._t0
        p  = print
        p('\n' + '=' * 72)
        p(f'Phase 4 acceptance — {el:.1f} s observed')
        p('=' * 72)

        if self.n_msgs == 0:
            p('NO /cbf/per_link_distances RECEIVED — is real_time_distance up, '
              'and is the depth topic remapped for this bag?')
            return False

        p(f'\nper-link-distance stream   {self.n_msgs} msgs '
          f'({self.n_msgs / max(el, 1e-9):.1f} Hz)')
        p(f'  empty (heartbeat)        {self.n_empty}'
          f'  ({100.0 * self.n_empty / self.n_msgs:.1f}%)')
        p(f'  empty right after close  {self.n_empty_after_close}'
          '   <- min_thresh publish hole firing')
        p(f'  entries per msg          '
          f'{dict(sorted(self.entries_hist.items()))}')
        p(f'  distinct links           {len(self.links_seen)}  '
          f'{dict(sorted(self.links_seen.items()))}')
        p(f'  min surface gap          {self.min_gap:.4f} m'
          + (f'  at t={self.min_gap_t:.1f} s' if self.min_gap_t is not None else ''))
        p(f'  frames at gap == 0       {self.n_zero_gap}')
        p(f'  max CPs inside d_safe    {self.max_below_dsafe}')
        p(f'  max axis-vs-surface bias {self.max_axis_bias:.4f} m'
          '   <- what the old barrier added back')

        if self.n_status:
            p(f'\ncbf_status stream          {self.n_status} msgs '
              f'({self.n_status / max(el, 1e-9):.1f} Hz)')
            p(f'  n_c histogram            {dict(sorted(self.nc_hist.items()))}')
            p(f'  n_c changes              {self.nc_changes}'
              f'  ({100.0 * self.nc_changes / self.n_status:.1f}% of ticks '
              'force an OSQP setup())')
            if self.status_seen_4:
                p(f'  n_active_cps histogram   '
                  f'{dict(sorted(self.nact_hist.items()))}')
                p(f'  max n_active_cps         {self.max_nact}'
                  f'   (slack there: {self.slack_at_max_nact:.4f})')
            else:
                p('  n_active_cps             ABSENT — publisher still sends '
                  '3 elements; rebuild franka_experiments')
            p(f'  max slack                {self.max_slack:.4f}')
            p(f'  fault-braking ticks      {self.n_fault}')
        else:
            p('\ncbf_status stream          NONE — cbf_safety_filter not running')

        # ── checklist ───────────────────────────────────────────────────────
        p('\n' + '-' * 72)
        checks = []
        checks.append((
            'no collision (min gap > 0 throughout)',
            self.min_gap > 0.0,
            f'min gap {self.min_gap:.4f} m'
            + ('' if self.min_gap > 0 else
               f'; clamped to 0 on {self.n_zero_gap} frames')))
        checks.append((
            'multi-CP activation observed (n_active_cps > 1)',
            self.max_nact > 1,
            f'max n_active_cps = {self.max_nact}'
            + ('' if self.status_seen_4 else ' (data[3] missing — rebuild)')))
        checks.append((
            'per-CP publishing (more entries than links)',
            max(self.entries_hist or [0]) > len(self.links_seen),
            f'max entries {max(self.entries_hist or [0])} '
            f'vs {len(self.links_seen)} distinct links'))
        checks.append((
            'no safety-chain faults',
            self.n_fault == 0,
            f'{self.n_fault} fault-braking ticks'))
        checks.append((
            'min_thresh hole did not fire',
            self.n_empty_after_close == 0,
            f'{self.n_empty_after_close} contact-range frames dropped'))

        ok = True
        for name, passed, detail in checks:
            p(f'  [{"PASS" if passed else "FAIL"}]  {name:48s} {detail}')
            ok &= passed

        if self.min_gap < np.inf:
            p(f'\nCBFDIAG cross-check: at the closest approach the filter should '
              f'print\n  d_min={self.min_gap:.3f}  (the SURFACE gap). If it '
              f'prints ~{self.min_gap + self.max_axis_bias:.3f} instead, the '
              'barrier is\n  still being rebuilt from the segment axis and the '
              'Phase 1 fix is not live.')
        p('=' * 72)
        return ok


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--duration', type=float, default=60.0,
                    help='seconds to observe (default 60)')
    ap.add_argument('--d-safe', type=float, default=0.20,
                    help='d_safe from fr3_control.yaml')
    ap.add_argument('--min-thresh', type=float, default=0.08,
                    help='distance.thresholds.min_thresh from fr3_complete.yaml')
    ap.add_argument('--distance-topic', default='/cbf/per_link_distances')
    ap.add_argument('--status-topic',   default='/NS_1/cbf_status')
    args = ap.parse_args()

    rclpy.init()
    node = Phase4Check(args)
    deadline = time.monotonic() + args.duration
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    ok = node.report()
    node.destroy_node()
    rclpy.shutdown()
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
