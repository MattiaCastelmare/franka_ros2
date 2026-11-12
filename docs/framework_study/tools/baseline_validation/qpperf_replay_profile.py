#!/usr/bin/env python3
"""QP-performance study, phase 1 — SEQUENTIAL replay cost profile of the production CBF filter.

Isolated benchmark, read-only. No Gazebo, no launch, no executor, no spinning. Runs inside the
`franka_ros2` container against the INSTALLED franka_experiments (== source at the baseline).

  ROS_DOMAIN_ID=231 ROS_LOCALHOST_ONLY=1 python3 -B qpperf_replay_profile.py <cell_dir> <out_dir> \
      <params_file> [<params_file> ...]

What it does
  * Instantiates the real `CBFSafetyFilter` node in-process with the S6B command-line parameters
    (`use_sim_time`, the three launch param files) on an isolated DDS domain. Its publishers
    publish into that empty domain; nothing subscribes.
  * Replays the S6B bag (/NS_1/franka/joint_states, /NS_1/qddot_nom, /cbf/per_link_distances) in
    bag-time order, setting the node's ROS clock to each message's recorded sim time, and calls the
    node's OWN callbacks; `_update_constraints` every 20 ms and `_qp_tick` every 10 ms of sim time,
    i.e. exactly the configured 50 / 100 Hz with no contention at all.
  * Times every call (perf_counter wall and thread CPU), times `_solve` separately (wrapped at the
    class level BEFORE construction; no file is modified), and cProfiles one replay pass of
    `build` and of `_qp_tick` for a function-level breakdown.
Output: <out_dir>/phase1_costs.json, phase1_profile_build.txt, phase1_profile_qptick.txt
"""
import cProfile
import io
import json
import os
import pstats
import sys
import time

import numpy as np

sys.dont_write_bytecode = True
CD, OUT, PFILES = sys.argv[1], sys.argv[2], sys.argv[3:]
os.makedirs(OUT, exist_ok=True)

import rclpy  # noqa: E402
import rosbag2_py  # noqa: E402
from rclpy.serialization import deserialize_message  # noqa: E402
from rclpy.time import Time  # noqa: E402
from rosidl_runtime_py.utilities import get_message  # noqa: E402

import franka_experiments.nodes.cbf_safety_filter as M  # noqa: E402

T_SOLVE = []


def _wrap_solve(orig):
    def w(self, *a, **k):
        t0 = time.perf_counter()
        c0 = time.thread_time()
        r = orig(self, *a, **k)
        T_SOLVE.append((self._now(), time.perf_counter() - t0, time.thread_time() - c0))
        return r
    return w


M.CBFSafetyFilter._solve = _wrap_solve(M.CBFSafetyFilter._solve)

args = ['qpperf', '--ros-args', '-r', '__node:=cbf_safety_filter', '-p', 'use_sim_time:=True',
        '-p', 'truth_topic:=/sim/truth/cp_gaps', '-p', 'truth_n_cp:=11']
for p in PFILES:
    args += ['--params-file', p]
rclpy.init(args=args)


def make_node():
    n = M.CBFSafetyFilter()
    n.get_clock()._set_ros_time_is_active(True)
    return n


# ------------------------------------------------------------------ bag
rd = rosbag2_py.SequentialReader()
rd.open(rosbag2_py.StorageOptions(uri=os.path.join(CD, 'bag'), storage_id='sqlite3'),
        rosbag2_py.ConverterOptions('cdr', 'cdr'))
types = {t.name: t.type for t in rd.get_all_topics_and_types()}
want = {'/NS_1/franka/joint_states': 'js', '/NS_1/qddot_nom': 'nom', '/cbf/per_link_distances': 'dist'}
ev = []
while rd.has_next():
    topic, raw, t_ns = rd.read_next()
    if topic in want:
        ev.append((t_ns * 1e-9, want[topic], deserialize_message(raw, get_message(types[topic]))))
ev.sort(key=lambda e: e[0])
t_end = ev[-1][0]
t_start = ev[0][0]
for k, dt in (('rebuild', 0.02), ('qp', 0.01)):
    ev += [(t, k, None) for t in np.arange(np.ceil(t_start / dt) * dt, t_end, dt)]
ev.sort(key=lambda e: (e[0], {'js': 0, 'nom': 1, 'dist': 2, 'rebuild': 3, 'qp': 4}[e[1]]))

man = json.load(open(os.path.join(CD, 'run_manifest_sim.json')))
T0, T1 = float(man['episode']['t_start_sim']), float(man['episode']['t_end_sim'])
KINDS = ('js', 'nom', 'dist', 'rebuild', 'qp')


def replay(node, prof_kind=None):
    call = {'js': node._on_joint_state, 'nom': node._on_qddot_nom, 'dist': node._on_distances,
            'rebuild': node._update_constraints, 'qp': node._qp_tick}
    clk = node.get_clock()
    T = {k: [] for k in call}
    n_rows = []
    pr = cProfile.Profile() if prof_kind else None
    for t, k, m in ev:
        clk.set_ros_time_override(Time(nanoseconds=int(round(t * 1e9))))
        f = call[k]
        if pr is not None and k == prof_kind and T0 <= t <= T1:
            pr.enable()
            f() if m is None else f(m)
            pr.disable()
            continue
        w0, c0 = time.perf_counter(), time.thread_time()
        f() if m is None else f(m)
        if T0 <= t <= T1:
            T[k].append((time.perf_counter() - w0, time.thread_time() - c0))
            if k == 'rebuild' and node._con is not None:
                n_rows.append(len(node._con.h_bar))
    return T, n_rows, pr


def st(x):
    x = 1e3 * np.asarray(x)
    return dict(n=int(x.size), p50=float(np.percentile(x, 50)), p95=float(np.percentile(x, 95)),
                p99=float(np.percentile(x, 99)), max=float(x.max()), mean=float(x.mean()))


node = make_node()
T2, n_rows, _ = replay(node)
solve = [s for s in T_SOLVE if T0 <= s[0] <= T1]
node.destroy_node()
R = {'episode_sim': [T0, T1], 'params_files': PFILES, 'note': 'one sequential pass; ms'}
span = T1 - T0
for k, v in T2.items():
    a = np.array(v)
    R[k] = dict(wall_ms=st(a[:, 0]), cpu_ms=st(a[:, 1]), calls_per_sim_s=len(v) / span,
                cpu_ms_per_sim_s=float(1e3 * a[:, 1].sum() / span))
s = np.array(solve)
R['solve'] = dict(wall_ms=st(s[:, 1]), cpu_ms=st(s[:, 2]), n=int(len(s)),
                  cpu_ms_per_sim_s=float(1e3 * s[:, 2].sum() / span))
R['rows_at_rebuild'] = dict(p50=float(np.percentile(n_rows, 50)), max=int(max(n_rows)))
R['budget_cpu_ms_per_sim_s_sum'] = float(sum(R[k]['cpu_ms_per_sim_s'] for k in KINDS))

for kind in ('rebuild', 'qp', 'js'):
    nd = make_node()
    _, _, pr = replay(nd, kind)
    nd.destroy_node()
    sio = io.StringIO()
    pstats.Stats(pr, stream=sio).sort_stats('cumulative').print_stats(45)
    sio2 = io.StringIO()
    pstats.Stats(pr, stream=sio2).sort_stats('tottime').print_stats(40)
    open(os.path.join(OUT, f'phase1_profile_{kind}.txt'), 'w').write(
        '### sorted by cumulative\n' + sio.getvalue() + '\n### sorted by tottime\n' + sio2.getvalue())

json.dump(R, open(os.path.join(OUT, 'phase1_costs.json'), 'w'), indent=1)
print(json.dumps(R, indent=1))
rclpy.shutdown()
