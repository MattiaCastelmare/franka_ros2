#!/usr/bin/env python3
"""R1 — deterministic per-tick outputs of the CBF filter on the S6B bag (no executor).

Isolated benchmark, read-only. Runs inside the `franka_ros2` container on an isolated DDS domain
against the INSTALLED franka_experiments. Same method as `qpperf_replay_profile.py` (phase 1):
the real `CBFSafetyFilter` is constructed with the S6B parameters, and its OWN callbacks are called
in recorded sim-time order (joint states, qddot_nom, distances from the bag; `_update_constraints`
every 20 ms and `_qp_tick` every 10 ms of sim time), with the ROS clock set to each event. There is
no executor and no thread, so the sequence of inputs to every tick is fixed by the bag alone.

  ROS_DOMAIN_ID=231 ROS_LOCALHOST_ONLY=1 python3 -B r1_sequential_outputs.py <cell_dir> <out.npz> \
      [--ros-arg -p --ros-arg executor_mode:=single ...] <params_file> [...]

Arguments after the cell dir and output path: `--ros-arg X` appends X to the node's --ros-args
(used after R1 to set executor_mode); anything else is a launch params file.

Records, for the whole replay (not only the episode):
  pub   : (t, qddot_safe[7])           every `_publish` call (the command that goes out)
  solve : (t, qddot[7], slack, level, n_iter, status_val, obj_val)   every `_solve` return
  tick  : (t, n_rows_of_current_snapshot)
and writes P (the node's full parameter namespace) to <out>.params.json.
Purpose: show that R1 changes no control equation: the baseline install and the R1 install, in
either executor mode, must give BIT-IDENTICAL arrays here.
"""
import json
import os
import sys

import numpy as np

sys.dont_write_bytecode = True
CD, OUT = sys.argv[1], sys.argv[2]
EXTRA, PFILES = [], []
_a = sys.argv[3:]
_i = 0
while _i < len(_a):
    if _a[_i] == '--ros-arg':
        EXTRA.append(_a[_i + 1])
        _i += 2
    else:
        PFILES.append(_a[_i])
        _i += 1

import rclpy  # noqa: E402
import rosbag2_py  # noqa: E402
from rclpy.serialization import deserialize_message  # noqa: E402
from rclpy.time import Time  # noqa: E402
from rosidl_runtime_py.utilities import get_message  # noqa: E402

import franka_experiments.nodes.cbf_safety_filter as M  # noqa: E402

PUB, SOLVE, TICK = [], [], []
C = M.CBFSafetyFilter
_orig_pub, _orig_solve, _orig_tick = C._publish, C._solve, C._qp_tick


def _pub(self, qddot_safe):
    PUB.append(np.r_[self._now(), np.asarray(qddot_safe, float)])
    return _orig_pub(self, qddot_safe)


def _solve(self, *a, **k):
    r = _orig_solve(self, *a, **k)
    qdd, slack, _ms, res, level = r
    info = getattr(res, 'info', None)
    SOLVE.append(np.r_[self._now(), np.asarray(qdd, float), float(np.sum(slack)) if slack is not None else np.nan,
                       float({'qp': 0, 'box_only': 1, 'brake': 2}.get(level, 3)) if isinstance(level, str) else float(level),
                       float(getattr(info, 'iter', np.nan)), float(getattr(info, 'status_val', np.nan)),
                       float(getattr(info, 'obj_val', np.nan))])
    return r


def _tick(self, *a, **k):
    r = _orig_tick(self, *a, **k)
    TICK.append((self._now(), -1.0 if self._con is None else float(len(self._con.h_bar))))
    return r


C._publish, C._solve, C._qp_tick = _pub, _solve, _tick

args = ['r1seq', '--ros-args', '-r', '__node:=cbf_safety_filter', '-p', 'use_sim_time:=True',
        '-p', 'truth_topic:=/sim/truth/cp_gaps', '-p', 'truth_n_cp:=11']
for p in PFILES:
    args += ['--params-file', p]
args += EXTRA
rclpy.init(args=args)
node = C()
node.get_clock()._set_ros_time_is_active(True)

rd = rosbag2_py.SequentialReader()
rd.open(rosbag2_py.StorageOptions(uri=os.path.join(CD, 'bag'), storage_id='sqlite3'),
        rosbag2_py.ConverterOptions('cdr', 'cdr'))
types = {t.name: t.type for t in rd.get_all_topics_and_types()}
want = {'/NS_1/franka/joint_states': 'js', '/NS_1/qddot_nom': 'nom', '/cbf/per_link_distances': 'dist'}
ev = []
while rd.has_next():
    topic, raw, t_ns = rd.read_next()
    if topic in want:
        ev.append((t_ns, want[topic], deserialize_message(raw, get_message(types[topic]))))
t0n, t1n = min(e[0] for e in ev), max(e[0] for e in ev)
for k, dt in (('rebuild', 20_000_000), ('qp', 10_000_000)):       # integer ns grid
    ev += [(t, k, None) for t in range(-(-t0n // dt) * dt, t1n, dt)]
ORDER = {'js': 0, 'nom': 1, 'dist': 2, 'rebuild': 3, 'qp': 4}
ev.sort(key=lambda e: (e[0], ORDER[e[1]]))
call = {'js': node._on_joint_state, 'nom': node._on_qddot_nom, 'dist': node._on_distances,
        'rebuild': node._update_constraints, 'qp': node._qp_tick}
clk = node.get_clock()
for t, k, m in ev:
    clk.set_ros_time_override(Time(nanoseconds=int(t)))
    call[k]() if m is None else call[k](m)

P = {k: (v if isinstance(v, (int, float, str, bool, type(None))) else repr(v))
     for k, v in sorted(vars(node.P).items())}
json.dump(P, open(OUT.replace('.npz', '') + '.params.json', 'w'), indent=1, sort_keys=True)
np.savez_compressed(OUT, pub=np.array(PUB), solve=np.array(SOLVE), tick=np.array(TICK))
print(f'events {len(ev)}  pub {len(PUB)}  solve {len(SOLVE)}  tick {len(TICK)}  P keys {len(P)}  '
      f'executor_mode={P.get("executor_mode", "<absent>")}')
node.destroy_node()
rclpy.shutdown()
