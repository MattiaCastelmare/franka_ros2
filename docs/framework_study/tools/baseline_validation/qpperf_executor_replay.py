#!/usr/bin/env python3
"""QP-performance study, phase 2 — the production node in its production executor, fed by a bag.

Isolated benchmark, read-only. No Gazebo, no launch file. Runs inside the `franka_ros2` container on
an isolated DDS domain (set ROS_DOMAIN_ID / ROS_LOCALHOST_ONLY in the environment).

  python3 -B qpperf_executor_replay.py <cell_dir> <out_json> <rate> <variant> <params_file>...

  variant: base       subscribe to /NS_1/franka/joint_states (1 kHz), as in S6B
           js_low     remap the filter's joint-state topic to /NS_1/joint_states (the aggregated,
                      low-rate topic recorded in the same bag) -> removes the 1 kHz stream
           rebuild25  base + cbf_update_rate_hz:=25.0
           rebuild0   base, but `_update_constraints` returns immediately after the first rebuild
                      in the episode (the QP keeps using one frozen snapshot) -> removes the
                      rebuild COMPUTE while keeping its timer and executor traffic

The node is the real `CBFSafetyFilter` with the S6B parameters (`use_sim_time`, the launch param
files) spun by `MultiThreadedExecutor(num_threads=3)` exactly as `cbf_safety_filter.main`.
`ros2 bag play` (a separate C++ process) replays /clock (1 kHz, recorded), the joint states,
/NS_1/qddot_nom and /cbf/per_link_distances from the S6B bag at <rate> x real time.

Instrumentation (class-level wrappers installed BEFORE construction; no file is modified):
per call of each callback and of `_solve`: sim time, wall duration, thread CPU, kernel TID.
Executor: count and CPU of `wait_for_ready_callbacks`. TimeSource: /clock callbacks.
Per-thread CPU from /proc/self/task at the episode's start and end (sim 10.55 / 40.56 s).
Env QPPERF_EXECUTOR=mt3|single replaces the executor in THIS harness (variant only).
Env QPPERF_SWITCH_S=<s> sets sys.setswitchinterval in THIS harness process (variant only).
Raw per-call rows (sim t, wall start, wall dur, thread CPU, TID) go to <out_json>_raw.npz.
"""
import json
import os
import subprocess
import sys
import threading
import time

import numpy as np

sys.dont_write_bytecode = True
CD, OUTJ, RATE, VARIANT, PFILES = sys.argv[1], sys.argv[2], float(sys.argv[3]), sys.argv[4], sys.argv[5:]

import rclpy  # noqa: E402
import rclpy.executors as EX  # noqa: E402
import rclpy.time_source as TS  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402

import franka_experiments.nodes.cbf_safety_filter as M  # noqa: E402

LOG = {k: [] for k in ('js', 'nom', 'dist', 'rebuild', 'qp', 'solve')}
NODE = {}
man = json.load(open(os.path.join(CD, 'run_manifest_sim.json')))
T0, T1 = float(man['episode']['t_start_sim']), float(man['episode']['t_end_sim'])


def wrap(cls, name, key, freeze=False):
    orig = getattr(cls, name)
    state = {'frozen': False}

    def w(self, *a, **k):
        if freeze and state['frozen']:
            return None
        w0, c0 = time.perf_counter(), time.thread_time()
        r = orig(self, *a, **k)
        t = self._now()
        LOG[key].append((t, w0, time.perf_counter() - w0, time.thread_time() - c0, threading.get_native_id()))
        if freeze and T0 <= t and self._con is not None:
            state['frozen'] = True
        return r
    setattr(cls, name, w)


C = M.CBFSafetyFilter
wrap(C, '_on_joint_state', 'js')
wrap(C, '_on_qddot_nom', 'nom')
wrap(C, '_on_distances', 'dist')
wrap(C, '_update_constraints', 'rebuild', freeze=(VARIANT == 'rebuild0'))
wrap(C, '_qp_tick', 'qp')
wrap(C, '_solve', 'solve')

EXW = {'n': 0, 'cpu': 0.0, 'wall': 0.0}
_orig_wfrc = EX.Executor.wait_for_ready_callbacks


def _wfrc(self, *a, **k):
    c0, w0 = time.thread_time(), time.perf_counter()
    r = _orig_wfrc(self, *a, **k)
    EXW['n'] += 1
    EXW['cpu'] += time.thread_time() - c0
    EXW['wall'] += time.perf_counter() - w0
    return r


EX.Executor.wait_for_ready_callbacks = _wfrc
CLK = {'n': 0, 'cpu': 0.0}
_orig_cc = TS.TimeSource.clock_callback


def _cc(self, msg):
    c0 = time.thread_time()
    _orig_cc(self, msg)
    CLK['n'] += 1
    CLK['cpu'] += time.thread_time() - c0


TS.TimeSource.clock_callback = _cc

args = ['qpperf', '--ros-args', '-r', '__node:=cbf_safety_filter', '-p', 'use_sim_time:=True',
        '-p', 'truth_topic:=/sim/truth/cp_gaps', '-p', 'truth_n_cp:=11']
for p in PFILES:
    args += ['--params-file', p]
js_topic = '/NS_1/franka/joint_states'
if VARIANT == 'js_low':
    args += ['-r', '/NS_1/franka/joint_states:=/NS_1/joint_states']
    js_topic = '/NS_1/joint_states'
if VARIANT == 'rebuild25':
    args += ['-p', 'cbf_update_rate_hz:=25.0']
SWITCH = os.environ.get('QPPERF_SWITCH_S')
if SWITCH:                     # harness-only variant: interpreter GIL switch interval
    sys.setswitchinterval(float(SWITCH))
rclpy.init(args=args)
node = C()
NODE['n'] = node
EXEC = os.environ.get('QPPERF_EXECUTOR', 'mt3')   # harness-only variant: mt3 | single
# (rclpy Humble has no Python StaticSingleThreadedExecutor; a run asking for it failed at import.)
if EXEC == 'single':
    ex = EX.SingleThreadedExecutor()
else:
    ex = MultiThreadedExecutor(num_threads=3)
ex.add_node(node)


def task_cpu():
    out = {}
    for tid in os.listdir('/proc/self/task'):
        try:
            s = open(f'/proc/self/task/{tid}/stat').read()
        except OSError:
            continue
        r = s.rfind(')')
        f = s[r + 2:].split()
        out[int(tid)] = (s[s.find('(') + 1:r], int(f[11]) + int(f[12]))
    return out


SNAP = {}


def monitor():
    while not SNAP.get('stop'):
        t = node._now()
        if 'a' not in SNAP and t >= T0:
            SNAP['a'] = (time.perf_counter(), t, task_cpu(), dict(EXW), dict(CLK))
        if 'b' not in SNAP and t >= T1:
            SNAP['b'] = (time.perf_counter(), t, task_cpu(), dict(EXW), dict(CLK))
        time.sleep(0.05)


spin = threading.Thread(target=ex.spin, daemon=True)
spin.start()
mon = threading.Thread(target=monitor, daemon=True)
mon.start()
time.sleep(3.0)
topics = ['/clock', js_topic, '/NS_1/qddot_nom', '/cbf/per_link_distances']
player = subprocess.run(['ros2', 'bag', 'play', os.path.join(CD, 'bag'), '--rate', str(RATE),
                         '--topics', *topics], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
time.sleep(1.0)
SNAP['stop'] = True
names = {t.native_id: t.name for t in threading.enumerate()}
names[threading.main_thread().native_id] = 'main(monitor-host)'
names[spin.native_id] = 'executor-spin(main loop)'
names[mon.native_id] = 'monitor'
R = {'variant': VARIANT, 'rate': RATE, 'executor': EXEC, 'switchinterval_s': sys.getswitchinterval(), 'episode_sim': [T0, T1], 'player_rc': player.returncode,
     'player_tail': player.stdout[-400:]}


def st(x):
    x = np.asarray(x, float)
    if not x.size:
        return None
    return dict(n=int(x.size), p50=float(np.percentile(x, 50)), p95=float(np.percentile(x, 95)),
                max=float(x.max()), mean=float(x.mean()))


span = T1 - T0
for k, v in LOG.items():
    a = np.array([x for x in v if T0 <= x[0] <= T1])
    if not a.size:
        R[k] = None
        continue
    R[k] = dict(calls_per_sim_s=len(a) / span, wall_ms=st(1e3 * a[:, 2]), cpu_ms=st(1e3 * a[:, 3]),
                cpu_s_total=float(a[:, 3].sum()), wait_ms=st(1e3 * (a[:, 2] - a[:, 3])),
                threads=sorted({int(x) for x in a[:, 4]}))
    if k == 'qp':
        R[k]['sim_gap_ms'] = st(1e3 * np.diff(a[:, 0]))
        R[k]['n_gap_ge_15ms'] = int((np.diff(a[:, 0]) >= 0.015).sum())
if 'a' in SNAP and 'b' in SNAP:
    (wa, sa, ca, ea, ka), (wb, sb, cb, eb, kb) = SNAP['a'], SNAP['b']
    wall = wb - wa
    R['window'] = dict(wall_s=wall, sim_s=sb - sa, rtf=(sb - sa) / wall)
    thr = {}
    for tid, (nm, j) in cb.items():
        if tid in ca:
            thr[f'{names.get(tid, nm)}[{tid}]'] = round(100.0 * (j - ca[tid][1]) / 100.0 / wall, 1)
    R['thread_cpu_pct_of_core'] = dict(sorted(thr.items(), key=lambda x: -x[1]))
    R['process_cpu_pct_of_core'] = round(sum(thr.values()), 1)
    R['executor_wait_calls_per_wall_s'] = (eb['n'] - ea['n']) / wall
    R['executor_wait_cpu_pct'] = 100.0 * (eb['cpu'] - ea['cpu']) / wall
    R['clock_callbacks_per_wall_s'] = (kb['n'] - ka['n']) / wall
    R['clock_callback_cpu_pct'] = 100.0 * (kb['cpu'] - ka['cpu']) / wall
    R['callback_body_cpu_pct'] = 100.0 * sum(R[k]['cpu_s_total'] for k in ('js', 'nom', 'dist', 'rebuild', 'qp')
                                             if R[k]) / wall
R['qp_ticks_per_sim_s'] = R['qp']['calls_per_sim_s'] if R['qp'] else None
json.dump(R, open(OUTJ, 'w'), indent=1)
np.savez_compressed(OUTJ.replace('.json', '_raw.npz'),
                    **{k: np.array(v, float).reshape(-1, 5) for k, v in LOG.items()})
print(json.dumps(R, indent=1))
ex.shutdown(timeout_sec=2.0)
node.destroy_node()
rclpy.shutdown()
os._exit(0)
