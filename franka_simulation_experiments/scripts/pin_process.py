#!/usr/bin/env python3
"""pin_process: move one process of THIS sim run, every thread, onto a CPU (gz_cbf_stack filter_cpu:=N).

SIM only. The sim counterpart of the stack's pin_rt_thread.sh: on the robot
the RT loop gets an isolated core (rt_pin_cpu); in Gazebo the loop whose rate
the contract checks is cbf_safety_filter's QP, and on a shared host it loses
ticks to other tenants. Measured with the same running stack on this host
(load ~20): 245 Hz of sim time unpinned, 300 Hz pinned to an idle isolated core.

Polls for a process whose command line contains --pattern AND whose
environment carries this launch's IGN_PARTITION (so a second sim on the host
is never touched), then runs ``taskset -a -cp <cpu> <pid>``. Exits 0 when
pinned, 1 on timeout.
"""
import argparse
import os
import subprocess
import sys
import time


def _environ(pid: str) -> str:
    try:
        with open(f'/proc/{pid}/environ', 'rb') as f:
            return f.read().decode(errors='ignore')
    except OSError:
        return ''


def _cmdline(pid: str) -> str:
    try:
        with open(f'/proc/{pid}/cmdline', 'rb') as f:
            return f.read().replace(b'\0', b' ').decode(errors='ignore')
    except OSError:
        return ''


def find(pattern: str, partition: str) -> list[str]:
    tag = f'IGN_PARTITION={partition}'
    return [p for p in os.listdir('/proc') if p.isdigit() and p != str(os.getpid())
            and pattern in _cmdline(p) and tag in _environ(p).split('\0')]


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--pattern', default='cbf_safety_filter --ros-args')
    ap.add_argument('--cpu', required=True)
    ap.add_argument('--timeout', type=float, default=120.0)
    a = ap.parse_args()
    partition = os.environ.get('IGN_PARTITION', '')
    if not partition:
        print('[pin_process] no IGN_PARTITION in the environment: refusing to guess', flush=True)
        sys.exit(1)
    t_end = time.time() + a.timeout
    while time.time() < t_end:
        pids = find(a.pattern, partition)
        if pids:
            for pid in pids:
                out = subprocess.run(['taskset', '-a', '-cp', a.cpu, pid],
                                     capture_output=True, text=True)
                print(f'[pin_process] {a.pattern!r} pid {pid} -> CPU {a.cpu} '
                      f'(rc {out.returncode})', flush=True)
            sys.exit(0)
        time.sleep(0.5)
    print(f'[pin_process] {a.pattern!r} not found in partition {partition} '
          f'within {a.timeout:.0f} s', flush=True)
    sys.exit(1)


if __name__ == '__main__':
    main()
