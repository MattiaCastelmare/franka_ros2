#!/usr/bin/env python3
"""M3: host sampler. Records per-core CPU jiffies, core frequency and the package temperature.

  python3 -I m3_host_sampler.py <out.npz> <stop_file> [period_s=0.5] [temp_abort_mC=95000] [n_consecutive=4]

* Every period, it records:
  * ``time.monotonic()``: the same clock as the replay's ``time.perf_counter()``
    (CLOCK_MONOTONIC, shared with the container);
  * ``time.time()``, to place the node's epoch-stamped log lines;
  * the ``/proc/stat`` per-cpu jiffies (user nice system idle iowait irq softirq steal);
  * every ``scaling_cur_freq`` (kHz);
  * ``x86_pkg_temp`` (m°C).
* It stops when ``stop_file`` exists, on SIGTERM / SIGINT, or after 900 s.
* **Thermal guard** (Addendum 01: a sustained reading, not a single sample). If the package
  temperature is at or above ``temp_abort_mC`` in ``n_consecutive`` consecutive samples
  (4 × 0.5 s = 2 s), the guard:
  * kills every M3 load worker at once (command-line marker ``M3_LOAD_WORKER``);
  * writes ``<out>.thermal_abort``, so that ``m3_run.sh`` aborts the batch.

  The original single-sample rule fired on a 0.5 s start-up spike (98 °C) of an idle host
  (ABORT_01).
* Read-only on the system: it only reads ``/proc`` and ``/sys`` and never changes a CPU setting.
"""
import glob
import os
import signal
import subprocess
import sys
import time

import numpy as np


def guard_fires(history, threshold, n):
    """True when the last ``n`` temperature samples are all at or above ``threshold``."""
    return len(history) >= n and all(k >= threshold for k in history[-n:])


def read_stat(ncpu):
    out = np.zeros((ncpu, 8), np.int64)
    with open('/proc/stat') as f:
        for line in f:
            if line.startswith('cpu') and line[3].isdigit():
                p = line.split()
                out[int(p[0][3:])] = [int(x) for x in p[1:9]]
    return out


def read_int(p):
    try:
        with open(p) as f:
            return int(f.read())
    except (OSError, ValueError):
        return -1


def main(argv):
    out, stop = argv[1], argv[2]
    period = float(argv[3]) if len(argv) > 3 else 0.5
    tabort = int(argv[4]) if len(argv) > 4 else 95000
    ncons = int(argv[5]) if len(argv) > 5 else 4
    ncpu = os.cpu_count()
    freq = [f'/sys/devices/system/cpu/cpu{i}/cpufreq/scaling_cur_freq' for i in range(ncpu)]
    tz = [z for z in glob.glob('/sys/class/thermal/thermal_zone*')
          if open(os.path.join(z, 'type')).read().strip() == 'x86_pkg_temp']
    tz = os.path.join(tz[0], 'temp') if tz else None
    run = [True]

    def _stop(*_):
        run[0] = False
    signal.signal(signal.SIGTERM, _stop)
    signal.signal(signal.SIGINT, _stop)
    T, E, S, F, K = [], [], [], [], []
    aborted = False
    t_end = time.monotonic() + 900.0
    while run[0] and not os.path.exists(stop) and time.monotonic() < t_end:
        T.append(time.monotonic())
        E.append(time.time())
        S.append(read_stat(ncpu))
        F.append([read_int(p) for p in freq])
        K.append(read_int(tz) if tz else -1)
        if not aborted and guard_fires(K, tabort, ncons):
            aborted = True
            subprocess.run(['pkill', '-KILL', '-f', 'M3_LOAD_WORKER'], check=False)
            with open(out + '.thermal_abort', 'w') as f:
                f.write(f'x86_pkg_temp >= {tabort} m°C in {ncons} consecutive samples {K[-ncons:]} '
                        f'at monotonic {T[-1]:.3f}; load workers killed\n')
        time.sleep(period)
    np.savez_compressed(out, t=np.array(T), epoch=np.array(E), stat=np.array(S), freq_khz=np.array(F),
                        pkg_temp_mC=np.array(K), period_s=period, temp_abort_mC=tabort, n_consecutive=ncons,
                        thermal_aborted=aborted)


if __name__ == '__main__':
    main(sys.argv)
