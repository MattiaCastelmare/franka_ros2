#!/usr/bin/env python3
"""Smoke test of the filter on the executor it SHIPS with (R2; run inside the container).

``smoke_cbf_safety_filter.py`` spins the filter and the stimulus together on its
own ``MultiThreadedExecutor(num_threads=4)``, an executor no launch uses, so it
never exercised the shipped one (F-8-54). This variant keeps that script's
stimulus and its three checks, and changes only the scheduling:

* the filter is spun ALONE on ``make_executor(P.executor_mode,
  P.qp_thread_fifo_prio)``, exactly what ``cbf_safety_filter.main`` builds from
  the installed ``fr3_control.yaml`` (``SingleThreadedExecutor`` since R2);
* the stimulus has its own ``SingleThreadedExecutor`` on another thread, so it
  does not share, or add work to, the filter's executor.

Each executor is driven by ``spin_once`` in a loop until a stop event: that is
what ``Executor.spin`` does (``while ok and not shutdown: spin_once()``), and it
lets the teardown stop and JOIN both threads before anything is destroyed.
Shutting an executor down while its thread still waits on the wait set aborts
the process ("terminate called without an active exception"; the first version
of this script, 3 of 6 runs, after every check had passed).

Checks (the same thresholds as the original script):

  0. the executor is the one the configuration asks for (``--expect`` sets the
     expected mode; default ``single``, the shipped default);
  1. qddot_safe published at ~ qp_rate_hz and all-finite, CBF active;
  2. distance stream stopped -> BRAKING fallback, not passthrough;
  3. qddot_nom stopped -> braking fallback (output ~0 with qdot = 0).

Usage::

    python3 test/smoke_cbf_safety_filter_shipped_executor.py
    python3 test/smoke_cbf_safety_filter_shipped_executor.py --expect multi \\
        --ros-args -p executor_mode:=multi
"""

import sys
import threading
import time

import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor

from franka_experiments.nodes.cbf_safety_filter import CBFSafetyFilter, make_executor
from smoke_cbf_safety_filter import Stimulus

EXPECTED_TYPE = {'single': SingleThreadedExecutor, 'multi': MultiThreadedExecutor}


def main(argv):
    expect = 'single'
    if len(argv) >= 2 and argv[0] == '--expect':
        expect, argv = argv[1], argv[2:]
    rclpy.init(args=argv)
    flt = CBFSafetyFilter()
    stim = Stimulus()
    failures = []

    ex = make_executor(flt.P.executor_mode, flt.P.qp_thread_fifo_prio)
    print(f'phase0: executor_mode={flt.P.executor_mode} ({type(ex).__name__}) '
          f'config={flt.P.config_path}')
    if flt.P.executor_mode != expect or type(ex) is not EXPECTED_TYPE[expect]:
        failures.append(f'phase0: executor {flt.P.executor_mode}/{type(ex).__name__}, '
                        f'expected {expect}/{EXPECTED_TYPE[expect].__name__}')
    ex.add_node(flt)
    ex_stim = SingleThreadedExecutor()
    ex_stim.add_node(stim)
    stop = threading.Event()

    def spin_until_stopped(e):
        while not stop.is_set() and rclpy.ok():
            e.spin_once(timeout_sec=0.1)

    threads = [threading.Thread(target=spin_until_stopped, args=(e,), daemon=True)
               for e in (ex, ex_stim)]
    for th in threads:
        th.start()

    # Phase 1: CBF active
    time.sleep(3.0)
    n1 = len(stim.received)
    if n1 < 10:
        failures.append(f'phase1: only {n1} outputs')
    else:
        dt = np.diff(stim.t_recv[-min(200, n1):])
        rate = 1.0 / np.mean(dt)
        out = np.array(stim.received[-50:])
        dev = np.abs(out - stim.qddot_nom).max()
        print(f'phase1: n={n1}  rate={rate:.0f} Hz  finite={np.isfinite(out).all()}  '
              f'max|out-nom|={dev:.3f}')
        want = float(flt.P.qp_rate_hz)
        if not (0.75 * want <= rate <= 1.3 * want):
            failures.append(f'phase1: rate {rate:.0f} Hz not ~{want:.0f}')
        if not np.isfinite(out).all():
            failures.append('phase1: non-finite output')
        if dev < 1e-3:
            failures.append('phase1: CBF constraint did not bind (expected clipping)')

    # Phase 2: stop distances -> BRAKING after distance_timeout
    stim.send_dist = False
    time.sleep(1.5)
    if not stim.received:
        failures.append('phase2: no qddot_safe at all')
    else:
        out = np.array(stim.received[-20:])
        print(f'phase2: max|out|={np.abs(out).max():.2e} '
              f'(expect ~0: braking with qdot=0, NOT passthrough)')
        if np.abs(out).max() > 1e-4:
            failures.append('phase2: no braking fallback after stale distances '
                            f'(max|out|={np.abs(out).max():.2e})')

        # Phase 3: stop nominal -> braking fallback (qdot=0 -> output ~0)
        stim.send_nom = False
        time.sleep(1.5)
        out = np.array(stim.received[-20:])
        print(f'phase3: max|out|={np.abs(out).max():.2e} (expect ~0, braking with qdot=0)')
        if np.abs(out).max() > 1e-4:
            failures.append('phase3: braking fallback not applied')

    stop.set()
    for th in threads:
        th.join(timeout=5.0)
    if any(th.is_alive() for th in threads):
        failures.append('teardown: a spin thread did not stop within 5 s')
    ex.shutdown(timeout_sec=2.0)
    ex_stim.shutdown(timeout_sec=2.0)
    flt.destroy_node()
    stim.destroy_node()
    rclpy.shutdown()

    if failures:
        print('FAIL:\n  ' + '\n  '.join(failures))
        raise SystemExit(1)
    print('SMOKE TEST (shipped executor) PASSED')


if __name__ == '__main__':
    main(sys.argv[1:])
