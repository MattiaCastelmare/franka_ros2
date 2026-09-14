#!/usr/bin/env python3
"""One-screen digest of an experiment run, for the eye and for a message.

``experiment_logger`` writes ~180 columns at 100 Hz. That is the right amount of
data to KEEP and the wrong amount to READ: after a run you want to know whether
the arm moved, how close it got, how hard the barrier fought the task, and
whether anything saturated or faulted — before deciding if the run is worth
looking at in detail.

So this prints that, grouped, with peaks and their timestamps, and it names
every channel that was not recording rather than reporting its absence as zero.

    python3 scripts/experiment_summary.py <run_dir> [--csv]

``<run_dir>`` is one directory written by ``experiment_logger``
(``experiment_log.csv`` + ``run_manifest.json``). ``--csv`` prints a single
machine-readable line instead, for collecting many runs into one table.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import os
import sys

NUM_JOINTS = 7


def load(run_dir):
    if os.path.isfile(run_dir):
        csv_p, run_dir = run_dir, os.path.dirname(run_dir)
    else:
        csv_p = os.path.join(run_dir, 'experiment_log.csv')
    if not os.path.exists(csv_p):
        sys.exit(f'no experiment_log.csv in {run_dir}')
    with open(csv_p) as fh:
        rows = list(csv.DictReader(fh))
    man = {}
    man_p = os.path.join(run_dir, 'run_manifest.json')
    if os.path.exists(man_p):
        with open(man_p) as fh:
            man = json.load(fh)
    return man, rows, csv_p


def col(rows, name):
    out = []
    for r in rows:
        try:
            out.append(float(r.get(name, '')))
        except (TypeError, ValueError):
            out.append(float('nan'))
    return out


def fin(xs):
    return [x for x in xs if not math.isnan(x)]


def peak(rows, name, lo=False):
    """(value, t) of the extreme finite sample, or (nan, nan)."""
    vals = col(rows, name)
    best, bt = float('nan'), float('nan')
    for r, v in zip(rows, vals):
        if math.isnan(v):
            continue
        if math.isnan(best) or (v < best if lo else v > best):
            best, bt = v, float(r.get('t', 'nan') or 'nan')
    return best, bt


def vec_peak(rows, prefix):
    """Largest |value| over the 7 joints of a q/qdot/tau block, and where."""
    best, bt, bj = float('nan'), float('nan'), 0
    for r in rows:
        for j in range(1, NUM_JOINTS + 1):
            try:
                v = abs(float(r.get(f'{prefix}_{j}', '')))
            except (TypeError, ValueError):
                continue
            if math.isnan(best) or v > best:
                best, bt, bj = v, float(r.get('t', 'nan') or 'nan'), j
    return best, bt, bj


def covered(rows, name):
    return len(fin(col(rows, name))) / len(rows) if rows else 0.0


def sat_count(rows):
    """Saturated joints per sample, from the per-joint tau_sat_1..7 flags.

    Derived rather than logged as its own column: the per-joint flags already
    carry strictly more information, and a redundant total is one more thing
    that can disagree with the data it summarises.

    NaN when the flags are absent — /torque_saturation not running is a
    different statement from "nothing saturated".
    """
    out = []
    for r in rows:
        n, seen = 0, False
        for j in range(1, NUM_JOINTS + 1):
            try:
                v = float(r.get(f'tau_sat_{j}', ''))
            except (TypeError, ValueError):
                continue
            if math.isnan(v):
                continue
            seen = True
            n += 1 if v else 0
        out.append(float(n) if seen else float('nan'))
    return out


def fmt(v, unit='', nd=3):
    return 'not recorded' if math.isnan(v) else f'{v:.{nd}f}{unit}'


def line(label, v, t=None, unit='', nd=3, extra=''):
    # A missing value gets NO timestamp and NO annotation: "not recorded
    # @ t = 15.42 s joint 7" reads as a measurement and is not one.
    if math.isnan(v):
        return f'  {label:<26} {"not recorded":>14}'
    at = '' if (t is None or math.isnan(t)) else f'   @ t = {t:6.2f} s'
    return f'  {label:<26} {fmt(v, unit, nd):>14}{at}{extra}'


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('run_dir')
    ap.add_argument('--csv', action='store_true',
                    help='one machine-readable line instead of the report')
    args = ap.parse_args()

    man, rows, csv_p = load(args.run_dir)
    if not rows:
        sys.exit('the CSV has no rows')
    ts = fin(col(rows, 't'))
    dur = (ts[-1] - ts[0]) if len(ts) > 1 else 0.0
    rate = (len(ts) - 1) / dur if dur > 0 else 0.0

    v_tcp, t_tcp = peak(rows, 'tcp_speed')
    w_tcp, t_w = peak(rows, 'tcp_omega')
    qd, t_qd, j_qd = vec_peak(rows, 'qdot')
    qdd, t_qdd, j_qdd = vec_peak(rows, 'qddot')
    tau, t_tau, j_tau = vec_peak(rows, 'tau_cmd')
    tau_m, t_taum, j_taum = vec_peak(rows, 'tau_effort')
    d_min, t_d = peak(rows, 'cp_min_distance', lo=True)
    d_leg, t_dleg = peak(rows, 'min_distance', lo=True)
    bend, t_bend = peak(rows, 'qddot_delta_norm')
    nom, _ = peak(rows, 'qddot_nom_norm')
    v_obs, t_vobs = peak(rows, 'cp_v_obs_max')
    slack, t_slack = peak(rows, 'cbf_slack')
    n_viol = [v for v in fin(col(rows, 'cbf_n_violated')) if v > 0]
    n_fault = [v for v in fin(col(rows, 'cbf_fault')) if v >= 1.0]
    sat = sat_count(rows)
    n_sat = [v for v in fin(sat) if v > 0]
    n_stop = [v for v in fin(col(rows, 'iso_stop_latched')) if v >= 1.0]
    comm, t_comm = peak(rows, 'comm_success_min', lo=True)
    dt = 1.0 / rate if rate > 0 else 0.0

    if args.csv:
        print(','.join(str(x) for x in [
            os.path.basename(args.run_dir.rstrip('/')), f'{dur:.2f}', f'{rate:.1f}',
            f'{v_tcp:.4f}', f'{qd:.4f}', f'{qdd:.3f}', f'{tau:.2f}',
            f'{d_min:.4f}', f'{bend:.3f}', len(n_viol), len(n_fault),
            len(n_sat), len(n_stop)]))
        return 0

    git = man.get('git', {}) or {}
    print('=' * 74)
    print(f'  EXPERIMENT SUMMARY   {os.path.basename(args.run_dir.rstrip("/"))}')
    print('=' * 74)
    print(f'  recorded    {man.get("created", "?")}')
    print(f'  git         {str(git.get("sha") or "?")[:12]}'
          f'{" (DIRTY)" if git.get("dirty") else ""}  {git.get("branch", "?")}')
    print(f'  duration    {dur:.2f} s at {rate:.1f} Hz  ({len(rows)} samples)')
    print(f'  d_safe      {man.get("d_safe", "?")} m      TCP frame: '
          f'{man.get("tcp_link", "?")}')
    print()

    print('  MOTION')
    print(line('peak TCP speed', v_tcp, t_tcp, ' m/s'))
    print(line('peak TCP angular speed', w_tcp, t_w, ' rad/s'))
    print(line('peak |q̇|', qd, t_qd, ' rad/s', extra=f'   joint {j_qd}'))
    print(line('peak |q̈|', qdd, t_qdd, ' rad/s²', extra=f'   joint {j_qdd}'))
    print()

    print('  TORQUE')
    print(line('peak |τ| commanded', tau, t_tau, ' N·m', nd=2,
               extra=f'   joint {j_tau}'))
    print(line('peak |τ| measured', tau_m, t_taum, ' N·m', nd=2,
               extra=f'   joint {j_taum}'))
    print(f'  {"saturated samples":<26} {len(n_sat):>14d}'
          + (f'   ({len(n_sat) * dt:.2f} s)' if n_sat else '')
          + ('' if len(fin(sat)) > 0.1 * max(len(rows), 1)
             else '   (/torque_saturation not recorded)'))
    print()

    print('  AVOIDANCE')
    print(line('closest approach', d_min, t_d, ' m', nd=4))
    if math.isnan(d_min) and not math.isnan(d_leg):
        print(line('  (legacy MultiDistance)', d_leg, t_dleg, ' m', nd=4))
    print(line('fastest tracked obstacle', v_obs, t_vobs, ' m/s'))
    print(line('peak ‖q̈_safe − q̈_nom‖', bend, t_bend, ' rad/s²',
               extra='   how hard the barrier fought the task'))
    print(line('peak ‖q̈_nom‖', nom, None, ' rad/s²'))
    print(line('peak QP slack', slack, t_slack, '', nd=4))
    print(f'  {"samples with a violated row":<26} {len(n_viol):>14d}'
          + (f'   ({len(n_viol) * dt:.2f} s)' if n_viol else ''))
    print()

    print('  HEALTH')
    print(f'  {"safety-chain fault":<26} {len(n_fault):>14d}'
          + (f'   ({len(n_fault) * dt:.2f} s)' if n_fault else '   samples'))
    print(f'  {"ISO stop latched":<26} {len(n_stop):>14d}'
          + (f'   ({len(n_stop) * dt:.2f} s)' if n_stop else '   samples'))
    print(line('worst FCI success rate', comm, t_comm, '', nd=3))
    print()

    # A channel that was not recording must be named, not reported as zero.
    watched = {
        'tcp_speed': 'Cartesian block (no Pinocchio model?)',
        'qddot_nom_norm': 'qddot_nom (is the commander running?)',
        'qddot_safe_norm': 'qddot_safe (is cbf_safety_filter running?)',
        'cp_min_distance': 'per_link_distances (is real_time_distance running?)',
        'cbf_slack': 'cbf_status',
        'tau_cmd_1': 'torque_cmd (is qddot_to_torque running?)',
        'tau_sat_1': '/torque_saturation (qddot_to_torque without the ISO layer?)',
        'comm_success_rate': 'franka_robot_state (fake hardware?)',
    }
    missing = [why for c, why in watched.items() if covered(rows, c) < 0.01]
    if missing:
        print('  NOT RECORDED IN THIS RUN')
        for why in missing:
            print(f'    - {why}')
        print('    Those columns are NaN, not zero: absent and measured-as-zero')
        print('    are different statements and the CSV keeps them apart.')
        print()
    print(f'  full data: {csv_p}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
