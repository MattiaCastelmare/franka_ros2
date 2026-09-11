#!/usr/bin/env python3
"""Live rate/jitter monitor for the whole control pipeline — bottleneck finder.

One process that subscribes to every stage of the stack at once and prints a
table of ACTUAL publish rate vs the NOMINAL rate each node is configured for,
so the stage that is starving the ones downstream shows up immediately:

    stage            topic                     node                 Hz   nom    %   p50    p99    max
    [perception]     /camera/.../image_rect_raw  camera             29.9   30   99  33.4   35.1   41.2
    [distance]       /cbf/per_link_distances     real_time_distance  8.1   30   27 121.0  260.3  310.7   <-- bottleneck
    [motion]         /NS_1/qddot_nom             pentagon_...       99.8  100  100  10.0   11.2   14.9
    [cbf]            /NS_1/qddot_safe            cbf_safety_filter  99.7  100  100  10.0   12.7   28.4
    [dynamics]       /NS_1/torque_cmd            qddot_to_torque    99.7  100  100  10.0   12.9   28.9
    [execution]      /NS_1/joint_states          joint_state_bro... 999.4 1000 100   1.0    1.3    4.1

Columns are measured at the SUBSCRIBER, i.e. what the next node in the chain
actually sees: p50/p99/max are inter-arrival periods in ms, so a healthy mean Hz
with a fat p99 means jitter (a stall), not a slow loop.

Subscriptions are raw (no deserialisation) and BEST_EFFORT/VOLATILE, which is
QoS-compatible with every publisher in this repo, so the monitor itself costs
almost nothing and never blocks a publisher.

Usage
-----
    # terminal 1 — the stack under test (or: ros2 bag play <bag>)
    ros2 launch franka_experiments torque_control_stack.launch.py

    # terminal 2 — live table, torque pipeline defaults
    python3 <ws>/src/franka_experiments/scripts/pipeline_rate_monitor.py

    # every topic in the graph, ranked by how far below nominal it runs
    python3 .../pipeline_rate_monitor.py --all

    # log the whole run and plot Hz-vs-time at the end
    python3 .../pipeline_rate_monitor.py --duration 120 \
        --csv ~/franka_logs/rates.csv --plot

    # add per-process CPU% (needs psutil) to separate "slow loop" from "CPU bound"
    python3 .../pipeline_rate_monitor.py --cpu

Read-only telemetry: it never publishes anything to the robot.

WARNING — running this during a HARDWARE run is not free.  The container sets
FASTRTPS_DEFAULT_PROFILES_FILE=fastdds_no_shm.xml, so shared memory is OFF and
every subscription travels the UDP loopback — the same kernel network path the
1 kHz FCI loop uses.  The two 1 kHz rows below therefore add ~2000 packets/s of
kernel work next to a loop whose deadline is 1 ms, and libfranka answers missed
deadlines with a communication_constraints_violation reflex.  On hardware:

    python3 pipeline_rate_monitor.py --exclude '(franka/joint_states|robot_state)'

which keeps every 100 Hz row and drops only the two kilohertz ones.  Their
health is visible anyway: if the RT loop stops, the whole table stops.
"""

from __future__ import annotations

import argparse
import csv
import os
import re
import sys
import time
from collections import deque
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rosidl_runtime_py.utilities import get_message


# ── Default watch list: one row per pipeline stage ───────────────────────────
# (stage label, topic, nominal Hz).  Nominal values mirror the node defaults:
# rate_hz=100 for the commander, control_rate_hz=100 for the CBF filters, the
# camera driver's configured fps for depth, and the 1 kHz controller_manager
# update rate for joint_states.  0 means "no nominal — just report".
DEFAULT_TOPICS: List[Tuple[str, str, float]] = [
    ('perception', '/camera/camera/depth/image_rect_raw', 30.0),
    # CAVEAT: this topic also carries the empty "no CP in range" heartbeat
    # (real_time_distance.py:467), and a raw subscriber cannot tell the two
    # apart.  30 Hz here proves perception is ALIVE, not that distances are
    # flowing — confirm content with scripts/cbf_phase4_check.py.
    ('distance',   '/cbf/per_link_distances',             30.0),
    # Fallback path only (_publish_fallback, real_time_distance.py:451): 0 Hz
    # here is the HEALTHY case, so no nominal — it must not read as a stall.
    ('distance',   '/human_robot/distance',                0.0),
    ('motion',     '/NS_1/qddot_nom',                    100.0),
    ('motion',     '/NS_1/q_des_state',                  100.0),
    ('cbf',        '/NS_1/qddot_safe',                   100.0),
    ('cbf',        '/NS_1/cbf_status',                   100.0),
    ('dynamics',   '/NS_1/torque_cmd',                   100.0),
    # The RT feed is franka/joint_states (joint_state_broadcaster, 1 kHz).
    # /NS_1/joint_states is the Python joint_state_publisher merger at
    # joint_state_rate = 30 Hz (franka.launch.py) — 30 Hz there is CORRECT,
    # not a bottleneck.  See config/fr3_control.yaml:6-27.
    ('execution',  '/NS_1/franka/joint_states',         1000.0),
    ('execution',  '/NS_1/joint_states',                  30.0),
    ('execution',  '/NS_1/franka_robot_state_broadcaster/robot_state', 1000.0),
    # velocity pipeline (only present when that stack is the one running)
    ('motion',     '/NS_1/tracking_qdot',                100.0),
    ('cbf',        '/NS_1/qdot_cmd',                     100.0),
]

# Topics that are pure noise in a rate table (latched / event-driven).
ALL_MODE_SKIP = re.compile(
    r'(/parameter_events|/rosout|_static$|/description$|/robot_description)')

# Processes worth showing in the CPU panel.
PROC_HINT = re.compile(
    r'(--ros-args|ros2 (run|launch)|controller_manager|ros2_control_node|'
    r'realsense2_camera|move_group|robot_state_publisher|rviz)')


class RateProbe:
    """Inter-arrival statistics for one topic."""

    def __init__(self, stage: str, topic: str, nominal: float, window: int):
        self.stage    = stage
        self.topic    = topic
        self.nominal  = nominal
        self.node     = '?'
        self.stamps: deque = deque(maxlen=window)
        self.total    = 0
        self.first_t: Optional[float] = None
        self.last_t:  Optional[float] = None

    def tick(self, now: float) -> None:
        self.stamps.append(now)
        self.total += 1
        if self.first_t is None:
            self.first_t = now
        self.last_t = now

    def stats(self, now: float) -> Dict[str, float]:
        """Windowed Hz + period percentiles [ms].  Zeros when never received."""
        if len(self.stamps) < 2:
            return dict(hz=0.0, p50=0.0, p99=0.0, mx=0.0, age=(
                now - self.last_t if self.last_t else float('inf')))
        s = list(self.stamps)
        d = sorted((b - a) for a, b in zip(s, s[1:]))
        n = len(d)
        span = s[-1] - s[0]
        return dict(
            hz  = (n / span) if span > 0 else 0.0,
            p50 = d[n // 2] * 1e3,
            p99 = d[min(n - 1, int(0.99 * n))] * 1e3,
            mx  = d[-1] * 1e3,
            age = now - s[-1],
        )


class PipelineRateMonitor(Node):

    def __init__(self, args):
        super().__init__('pipeline_rate_monitor')
        self.args   = args
        self.probes: Dict[str, RateProbe] = {}
        self.subs:   Dict[str, object] = {}
        # BEST_EFFORT + VOLATILE subscribes to every publisher QoS used here
        # (a reliable/transient-local publisher is compatible with it, not the
        # other way round), so no per-topic QoS negotiation is needed.
        self.qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
        )
        if not args.all:
            for stage, topic, nom in DEFAULT_TOPICS:
                self.probes[topic] = RateProbe(stage, topic, nom, args.window)
        for topic in args.topic or []:
            name, _, nom = topic.partition('=')
            self.probes[name] = RateProbe('user', name, float(nom or 0.0),
                                          args.window)

    # ── graph discovery ──────────────────────────────────────────────────────
    def rescan(self) -> None:
        """(Re)subscribe to anything now in the graph.

        Called every refresh, not once at start: the stack brings its nodes up
        on staggered TimerActions, so half the pipeline does not exist yet when
        the monitor starts.
        """
        for topic, types in self.get_topic_names_and_types():
            if topic in self.subs or not types:
                continue
            probe = self.probes.get(topic)
            if probe is None:
                if not self.args.all or ALL_MODE_SKIP.search(topic):
                    continue
                if self.args.include and not re.search(self.args.include, topic):
                    continue
                probe = RateProbe('discovered', topic, 0.0, self.args.window)
                self.probes[topic] = probe
            if self.args.exclude and re.search(self.args.exclude, topic):
                continue
            try:
                msg_type = get_message(types[0])
            except (ImportError, AttributeError, ValueError):
                continue
            self.subs[topic] = self.create_subscription(
                msg_type, topic,
                lambda _m, p=probe: p.tick(time.perf_counter()),
                self.qos, raw=True)      # raw: count frames, never deserialise

        for topic, probe in self.probes.items():
            if probe.node == '?':
                info = self.get_publishers_info_by_topic(topic)
                if info:
                    probe.node = info[0].node_name


# ── CPU panel ────────────────────────────────────────────────────────────────
class CpuSampler:
    """Per-process CPU% for ROS processes, or a no-op when psutil is missing."""

    def __init__(self, top: int = 8):
        self.top = top
        try:
            import psutil                                   # noqa: PLC0415
            self.psutil = psutil
        except ImportError:
            self.psutil = None
        self.procs: Dict[int, object] = {}

    def sample(self) -> List[Tuple[float, str]]:
        if self.psutil is None:
            return []
        for p in self.psutil.process_iter(['pid', 'cmdline']):
            if p.pid in self.procs:
                continue
            cmd = ' '.join(p.info.get('cmdline') or [])
            if PROC_HINT.search(cmd):
                self.procs[p.pid] = p
                try:
                    p.cpu_percent(None)          # prime the delta
                except Exception:                # noqa: BLE001 — process died
                    self.procs.pop(p.pid, None)
        out = []
        for pid, p in list(self.procs.items()):
            try:
                cmd = ' '.join(p.cmdline())
                out.append((p.cpu_percent(None), _short_cmd(cmd)))
            except Exception:                    # noqa: BLE001 — process died
                self.procs.pop(pid, None)
        return sorted(out, reverse=True)[:self.top]


def _short_cmd(cmd: str) -> str:
    """Best-effort node name out of a ROS process command line."""
    m = re.search(r'__node:=(\S+)', cmd)
    if m:
        return m.group(1)
    parts = [os.path.basename(c) for c in cmd.split()
             if not c.startswith('-') and '=' not in c]
    parts = [p for p in parts if p not in ('python3', 'python', 'ros2')]
    return parts[0] if parts else cmd[:28]


# ── rendering ────────────────────────────────────────────────────────────────
def _fit(text: str, width: int) -> str:
    """Keep the tail (the distinguishing half of a ROS name), marked elided."""
    return text if len(text) <= width else '…' + text[-(width - 1):]


HDR = (f'{"stage":<11} {"topic":<44} {"node":<24} '
       f'{"Hz":>7} {"nom":>6} {"%":>5} {"p50":>7} {"p99":>7} {"max":>7} {"n":>8}')


def render(mon: PipelineRateMonitor, cpu: CpuSampler, t0: float) -> str:
    now  = time.perf_counter()
    rows = []
    for topic, pr in mon.probes.items():
        st = pr.stats(now)
        pct = (100.0 * st['hz'] / pr.nominal) if pr.nominal > 0 else float('nan')
        rows.append((pct, pr, st))
    # worst offenders first; topics with no nominal, then topics never seen,
    # sink to the bottom (a nan pct compares false against everything)
    rows.sort(key=lambda r: (r[1].total == 0,
                             r[0] if r[0] == r[0] else 1e9,
                             -r[2]['hz']))

    out = [f'  pipeline rates — t = {now - t0:6.1f} s   '
           f'(window {mon.args.window} samples)', '', '  ' + HDR,
           '  ' + '-' * len(HDR)]
    for pct, pr, st in rows:
        if pr.total == 0:
            flag, pct_s = '  (no data)', '    -'
        else:
            pct_s = f'{pct:5.0f}' if pct == pct else '    -'
            flag = ''
            if pr.nominal > 0 and pct < 90:
                flag = '   <-- below nominal'
            if st['age'] > 1.0:
                flag = f'   <-- stalled {st["age"]:.1f}s'
        out.append(
            f'  {pr.stage:<11} {_fit(pr.topic, 44):<44} {_fit(pr.node, 24):<24} '
            f'{st["hz"]:7.1f} {pr.nominal:6.0f} {pct_s} '
            f'{st["p50"]:7.1f} {st["p99"]:7.1f} {st["mx"]:7.1f} '
            f'{pr.total:8d}{flag}')

    load = cpu.sample()
    if load:
        out += ['', '  CPU%  ' + '   '.join(f'{n}:{c:.0f}' for c, n in load)]
    elif cpu.psutil is None and cpu.top:
        out += ['', '  (pip install psutil for the per-process CPU panel)']
    return '\n'.join(out)


def write_plot(csv_path: str, png_path: str) -> None:
    import matplotlib                                        # noqa: PLC0415
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt                          # noqa: PLC0415

    series: Dict[str, Tuple[List[float], List[float]]] = {}
    with open(csv_path) as fh:
        for r in csv.DictReader(fh):
            t, h = series.setdefault(r['topic'], ([], []))
            t.append(float(r['t']))
            h.append(float(r['hz']))
    if not series:
        return
    fig, ax = plt.subplots(figsize=(12, 1.1 * len(series) + 2),
                           nrows=len(series), sharex=True, squeeze=False)
    for axis, (topic, (t, h)) in zip(ax[:, 0], sorted(series.items())):
        axis.plot(t, h, lw=1.0)
        axis.set_ylabel(topic.split('/')[-1][:18], fontsize=7)
        axis.grid(alpha=0.3)
        axis.set_ylim(bottom=0)
    ax[-1, 0].set_xlabel('time [s]')
    fig.suptitle('pipeline rate vs time')
    fig.tight_layout()
    fig.savefig(png_path, dpi=130)
    print(f'[rate_monitor] plot → {png_path}')


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--topic', action='append',
                    help='extra topic, optionally NAME=NOMINAL_HZ (repeatable)')
    ap.add_argument('--all', action='store_true',
                    help='monitor every topic in the graph, not just the stack')
    ap.add_argument('--include', help='regex a topic must match in --all mode')
    ap.add_argument('--exclude', help='regex of topics to skip')
    ap.add_argument('--interval', type=float, default=2.0,
                    help='table refresh period [s] (default 2)')
    ap.add_argument('--window', type=int, default=200,
                    help='inter-arrival samples per topic (default 200)')
    ap.add_argument('--duration', type=float, default=0.0,
                    help='stop after N seconds (0 = until Ctrl-C)')
    ap.add_argument('--csv', help='append one row per topic per refresh')
    ap.add_argument('--plot', action='store_true',
                    help='write <csv>.png (Hz vs time) on exit')
    ap.add_argument('--cpu', action='store_true',
                    help='per-process CPU%% panel (needs psutil)')
    ap.add_argument('--no-clear', action='store_true',
                    help='scroll instead of redrawing in place')
    args = ap.parse_args()

    rclpy.init()
    mon = PipelineRateMonitor(args)
    cpu = CpuSampler(top=8 if args.cpu else 0)

    writer = fh = None
    if args.csv:
        os.makedirs(os.path.dirname(os.path.abspath(args.csv)), exist_ok=True)
        fh = open(args.csv, 'w', newline='')
        writer = csv.writer(fh)
        writer.writerow(['t', 'stage', 'topic', 'node', 'hz', 'nominal',
                         'p50_ms', 'p99_ms', 'max_ms', 'count'])

    t0 = time.perf_counter()
    try:
        while rclpy.ok():
            deadline = time.perf_counter() + args.interval
            while time.perf_counter() < deadline:
                rclpy.spin_once(mon, timeout_sec=0.02)
            mon.rescan()

            if not args.no_clear:
                sys.stdout.write('\033[2J\033[H')
            print(render(mon, cpu, t0), flush=True)

            if writer:
                now = time.perf_counter()
                for topic, pr in mon.probes.items():
                    s = pr.stats(now)
                    writer.writerow([f'{now - t0:.3f}', pr.stage, topic, pr.node,
                                     f'{s["hz"]:.3f}', pr.nominal,
                                     f'{s["p50"]:.3f}', f'{s["p99"]:.3f}',
                                     f'{s["mx"]:.3f}', pr.total])
                fh.flush()

            if args.duration and (time.perf_counter() - t0) >= args.duration:
                break
    except KeyboardInterrupt:
        pass
    finally:
        if fh:
            fh.close()
            print(f'[rate_monitor] csv → {args.csv}')
            if args.plot:
                write_plot(args.csv, os.path.splitext(args.csv)[0] + '.png')
        mon.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
