#!/usr/bin/env python3
"""R1 — A/B of executor_mode on the S6B bag: the production node, its production executor choice.

Isolated benchmark, read-only. Same method as `qpperf_executor_replay.py` (phase 2), with two
differences: the executor is built by the PRODUCTION `cbf_safety_filter.make_executor` from the
node's own `executor_mode` parameter (set with `-p executor_mode:=<mode>`), exactly as `main()`;
and every tick records what inputs it used, so staleness and outputs can be compared.

  python3 -B r1_executor_replay.py <cell_dir> <out_prefix> <rate> <mode> <params_file>...

`ros2 bag play` (separate process) replays /clock (recorded 1 kHz), /NS_1/franka/joint_states,
/NS_1/qddot_nom and /cbf/per_link_distances at <rate> x real time. No production file is modified:
methods are wrapped at class level before construction.

Outputs <out_prefix>.json (summary) and <out_prefix>_raw.npz:
  js, nom, dist, rebuild, qp, solve : (node time at exit, wall start, wall dur, thread CPU, TID,
                                       node time at entry)
  tick_in  : per tick at ENTRY: wall, node now, js header stamp of the last processed joint state,
             js receipt stamp, nom receipt stamp, obs receipt stamp, obs capture stamp,
             wall and node-time at which the constraint snapshot in use was built
  pub      : per published command: wall, node now, qddot_safe[7]
  clock    : per /clock callback: wall, message sim time (to reconstruct the true playback time)
"""
import json
import os
import subprocess
import sys
import threading
import time

import numpy as np

sys.dont_write_bytecode = True
CD, OUTP, RATE, MODE, PFILES = sys.argv[1], sys.argv[2], float(sys.argv[3]), sys.argv[4], sys.argv[5:]

import rclpy  # noqa: E402
import rclpy.executors as EX  # noqa: E402
import rclpy.time_source as TS  # noqa: E402

import franka_experiments.nodes.cbf_safety_filter as M  # noqa: E402

man = json.load(open(os.path.join(CD, 'run_manifest_sim.json')))
T0, T1 = float(man['episode']['t_start_sim']), float(man['episode']['t_end_sim'])
LOG = {k: [] for k in ('js', 'nom', 'dist', 'rebuild', 'qp', 'solve')}
TICK_IN, PUB, CLOCK = [], [], []
NAN = float('nan')


def wrap(cls, name, key):
    orig = getattr(cls, name)

    def w(self, *a, **k):
        w0, c0 = time.perf_counter(), time.thread_time()
        t_in = self._now()
        if key == 'qp':
            js, nom, obs = self._js, self._nom, self._obs
            meta = getattr(self, '_r1_con_meta', (NAN, NAN))
            TICK_IN.append((w0, t_in,
                            NAN if self._diag_t_prev is None else self._diag_t_prev,
                            NAN if js is None else js.stamp, NAN if nom is None else nom.stamp,
                            NAN if obs is None else obs.stamp, NAN if obs is None else obs.t_cap,
                            meta[0], meta[1]))
        con_before = self._con if key == 'rebuild' else None
        r = orig(self, *a, **k)
        w1 = time.perf_counter()
        t = self._now()
        if key == 'rebuild' and self._con is not None and self._con is not con_before:
            self._r1_con_meta = (w1, t)          # harness-only attribute: when this snapshot was built
        LOG[key].append((t, w0, w1 - w0, time.thread_time() - c0, threading.get_native_id(), t_in))
        return r
    setattr(cls, name, w)


C = M.CBFSafetyFilter
for name, key in (('_on_joint_state', 'js'), ('_on_qddot_nom', 'nom'), ('_on_distances', 'dist'),
                  ('_update_constraints', 'rebuild'), ('_qp_tick', 'qp'), ('_solve', 'solve')):
    wrap(C, name, key)
_orig_pub = C._publish


def _pub(self, q):
    PUB.append((time.perf_counter(), self._now(), *np.asarray(q, float)))
    return _orig_pub(self, q)


C._publish = _pub
EXW = {'n': 0, 'cpu': 0.0}
_orig_wfrc = EX.Executor.wait_for_ready_callbacks


def _wfrc(self, *a, **k):
    c0 = time.thread_time()
    r = _orig_wfrc(self, *a, **k)
    EXW['n'] += 1
    EXW['cpu'] += time.thread_time() - c0
    return r


EX.Executor.wait_for_ready_callbacks = _wfrc
_orig_cc = TS.TimeSource.clock_callback


def _cc(self, msg):
    CLOCK.append((time.perf_counter(), msg.clock.sec + 1e-9 * msg.clock.nanosec))
    _orig_cc(self, msg)


TS.TimeSource.clock_callback = _cc

args = ['r1replay', '--ros-args', '-r', '__node:=cbf_safety_filter', '-p', 'use_sim_time:=True',
        '-p', 'truth_topic:=/sim/truth/cp_gaps', '-p', 'truth_n_cp:=11']
for p in PFILES:
    args += ['--params-file', p]
args += ['-p', f'executor_mode:={MODE}']
rclpy.init(args=args)
node = C()
ex = M.make_executor(node.P.executor_mode, node.P.qp_thread_fifo_prio)   # as main()
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
            SNAP['a'] = (time.perf_counter(), t, task_cpu(), dict(EXW))
        if 'b' not in SNAP and t >= T1:
            SNAP['b'] = (time.perf_counter(), t, task_cpu(), dict(EXW))
        time.sleep(0.05)


spin = threading.Thread(target=ex.spin, daemon=True)
spin.start()
mon = threading.Thread(target=monitor, daemon=True)
mon.start()
time.sleep(3.0)
topics = ['/clock', '/NS_1/franka/joint_states', '/NS_1/qddot_nom', '/cbf/per_link_distances']
player = subprocess.run(['ros2', 'bag', 'play', os.path.join(CD, 'bag'), '--rate', str(RATE),
                         '--topics', *topics], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
time.sleep(1.0)
SNAP['stop'] = True
names = {t.native_id: t.name for t in threading.enumerate()}
names[spin.native_id] = 'executor-spin'
names[mon.native_id] = 'monitor'
R = {'mode': MODE, 'executor': type(ex).__name__, 'rate': RATE, 'episode_sim': [T0, T1],
     'player_rc': player.returncode, 'affinity': sorted(os.sched_getaffinity(0))}
if 'a' in SNAP and 'b' in SNAP:
    (wa, sa, ca, ea), (wb, sb, cb, eb) = SNAP['a'], SNAP['b']
    wall = wb - wa
    thr = {f'{names.get(t, n)}[{t}]': round((j - ca[t][1]) / wall, 1) for t, (n, j) in cb.items() if t in ca}
    R['window'] = dict(wall_s=wall, sim_s=sb - sa, rtf=(sb - sa) / wall)
    R['thread_cpu_pct_of_core'] = {k: v for k, v in sorted(thr.items(), key=lambda x: -x[1]) if v > 0}
    R['process_cpu_pct_of_core'] = round(sum(thr.values()), 1)
    R['executor_loop_cpu_pct'] = 100.0 * (eb['cpu'] - ea['cpu']) / wall
json.dump(R, open(OUTP + '.json', 'w'), indent=1)
np.savez_compressed(OUTP + '_raw.npz',
                    **{k: np.array(v, float).reshape(-1, 6) for k, v in LOG.items()},
                    tick_in=np.array(TICK_IN, float).reshape(-1, 9), pub=np.array(PUB, float).reshape(-1, 9),
                    clock=np.array(CLOCK, float).reshape(-1, 2))
print(json.dumps(R))
ex.shutdown(timeout_sec=2.0)
node.destroy_node()
rclpy.shutdown()
os._exit(0)
