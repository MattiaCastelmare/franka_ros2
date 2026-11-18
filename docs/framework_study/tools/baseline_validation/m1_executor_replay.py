#!/usr/bin/env python3
"""M1: executor-thread replay. A thin harness wrapper around ``r1_executor_replay.py``.

Approved by the user on 2026-11-18 (experiment M1 of
``refactoring/R1_E3_MECHANISM_AND_VALIDATION_PLAN.md`` §4). The pre-registered protocol is
``refactor_r1_m1/protocol/M1_PROTOCOL.md``. Isolated benchmark: no Gazebo, and no production
file is modified.

  python3 -B m1_executor_replay.py <arm> <cell_dir> <out_prefix> <rate> <params_file>...

Arms:
  * ``single``: ``-p executor_mode:=single``; production ``make_executor`` gives a
    SingleThreadedExecutor.
  * ``mt3``: ``-p executor_mode:=multi``; production ``make_executor`` gives
    MultiThreadedExecutor(num_threads=3), the R1 default, unchanged.
  * ``mt1``: ``-p executor_mode:=multi`` with the module constant
    ``cbf_safety_filter.MULTI_EXECUTOR_THREADS`` set to 1 **in this process only**, before the
    node is built. Production ``make_executor`` then gives MultiThreadedExecutor(num_threads=1).

Everything else is ``r1_executor_replay.py``, run unmodified with ``runpy``:
  * the same node class and callbacks, QP and OSQP path;
  * the same parameter files, the same ``ros2 bag play`` of /clock, joint states, qddot_nom and
    distances;
  * the same per-callback instrumentation.

This wrapper adds only recording, which is identical in every arm:
  * each ``/cbf_status`` row the node builds (``_publish_status``, recorded after the call:
    ``n_rows``, ``solve_wall_ms``, ``constraint_build_wall_ms``, …);
  * the executor class and worker count actually built;
  * the node's subscribed topics;
  * a dump of every resolved filter parameter (``node.P``).

The extras go to ``<out_prefix>_m1.json`` / ``<out_prefix>_m1.npz``, written just before the
replay's own ``os._exit``.

``M1_DRY=1`` stops right after the executor is built: no spin and no playback. It writes only
``_m1.json``, which the setup validation uses to check the arms against each other.
"""
import json
import os
import runpy
import sys
import time

import numpy as np

sys.dont_write_bytecode = True
ARMS = {'single': ('single', None), 'mt1': ('multi', 1), 'mt3': ('multi', None)}
ARM = sys.argv[1]
if ARM not in ARMS:
    sys.exit(f'arm {ARM!r}: expected one of {list(ARMS)}')
CD, OUTP, RATE, PFILES = sys.argv[2], sys.argv[3], sys.argv[4], sys.argv[5:]
MODE, NTHREADS = ARMS[ARM]
DRY = os.environ.get('M1_DRY') == '1'
HERE = os.path.dirname(os.path.abspath(__file__))
REPLAY = os.path.join(HERE, 'r1_executor_replay.py')

import franka_experiments.nodes.cbf_safety_filter as M  # noqa: E402

if NTHREADS is not None:
    M.MULTI_EXECUTOR_THREADS = NTHREADS     # read by make_executor at call time

C = M.CBFSafetyFilter
STATUS = []
INFO = {'arm': ARM, 'executor_mode_param': MODE, 'rate': float(RATE), 'cell_dir': CD,
        'params_files': PFILES, 'replay_script': REPLAY,
        'multi_executor_threads_constant': M.MULTI_EXECUTOR_THREADS, 'dry': DRY}
NODE = {}

_orig_init = C.__init__


def _init(self, *a, **k):
    _orig_init(self, *a, **k)
    NODE['n'] = self


C.__init__ = _init
_orig_ps = C._publish_status


def _ps(self, *a, **k):
    r = _orig_ps(self, *a, **k)
    STATUS.append((time.perf_counter(), self._now(), *self._status_msg.data))
    return r


C._publish_status = _ps


def _jsonable(v):
    try:
        json.dumps(v)
        return v
    except TypeError:
        if isinstance(v, np.ndarray):
            return v.tolist()
        if isinstance(v, (list, tuple)):
            return [_jsonable(x) for x in v]
        if isinstance(v, dict):
            return {str(a): _jsonable(b) for a, b in v.items()}
        return repr(v)


def _write_info():
    json.dump(INFO, open(OUTP + '_m1.json', 'w'), indent=1, sort_keys=True)


_orig_mk = M.make_executor


def _mk(mode, prio=0):
    ex = _orig_mk(mode, prio)
    node = NODE.get('n')
    INFO['executor_class'] = type(ex).__name__
    pool = getattr(ex, '_executor', None)
    INFO['worker_threads'] = getattr(pool, '_max_workers', None)
    INFO['make_executor_args'] = [mode, prio]
    if node is not None:
        INFO['node_executor_mode'] = node.P.executor_mode
        INFO['subscriptions'] = sorted((s.topic_name, s.msg_type.__name__) for s in node.subscriptions)
        INFO['timers_period_s'] = sorted(t.timer_period_ns * 1e-9 for t in node.timers)
        try:
            items = vars(node.P).items()
        except TypeError:                       # a slotted / dataclass parameter object
            import dataclasses
            items = dataclasses.asdict(node.P).items()
        INFO['params'] = {k: _jsonable(v) for k, v in sorted(items)}
    if DRY:
        _write_info()
        print(json.dumps({k: INFO.get(k) for k in ('arm', 'executor_class', 'worker_threads',
                                                    'node_executor_mode', 'timers_period_s')}))
        sys.stdout.flush()
        os._exit(0)
    return ex


M.make_executor = _mk
_real_exit = os._exit


def _exit(code):
    try:
        INFO['status_rows'] = len(STATUS)
        _write_info()
        np.savez_compressed(OUTP + '_m1.npz', status=np.array(STATUS, float).reshape(len(STATUS), -1))
    finally:
        sys.stdout.flush()
        _real_exit(code)


os._exit = _exit
sys.argv = [REPLAY, CD, OUTP, RATE, MODE, *PFILES]
runpy.run_path(REPLAY, run_name='__main__')
