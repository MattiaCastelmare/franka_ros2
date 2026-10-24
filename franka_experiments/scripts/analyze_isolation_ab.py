#!/usr/bin/env python3
"""Analyze a pentagon_qddot_commander isolation_test run's experiment_log.csv,
and (with --a/--b) compare two runs (Condition A vs Condition B of the
rt_torque_controller Kp/Kd A/B test).

WHY q_cmd = qddot_safe, NOT qddot_nom
--------------------------------------
qddot_nom_j is what pentagon_qddot_commander PUBLISHES (feedforward + the
isolation-only joint-space PD, clamped) — but cbf_safety_filter sits between
that and the actuator, and this script's own validation run (fake hardware,
2026-09-22) measured up to 7 rad/s^2 of divergence between qddot_nom and
qddot_safe DURING the isolation window itself (joint-limit/box rows still
apply even with no camera / no tracked obstacle). qddot_safe is what
qddot_to_torque -> rt_torque_controller actually receives, so it is the
correct "commanded" signal for an ACTUATION-gap measurement. Using
qddot_nom instead would silently fold CBF reshaping into what looks like an
actuation-gap number. This script reports both, and flags large divergence
as a confound rather than hiding it.

SEGMENTATION
------------
Isolation start is detected as the first row where ANY qddot_nom_j is
non-zero, then segmented by the FIXED, KNOWN cadence (--seg-s, default the
node's isolation_seg_s=2.0s) rather than by re-detecting "engagement" every
row from a magnitude threshold — the latter fragments each joint's window at
the raised-cosine bump's acceleration zero-crossings (confirmed on a fake-
hardware validation run: a naive threshold split single 2.0s windows into
2-3 spurious sub-segments). Fixed-cadence slicing does not have this
problem and matches the commander's own `seg = floor(tau / iso_seg_s)`
exactly.

PENTAGON_RUN CSV (--a-pentagon/--b-pentagon), AND WHY IT MATTERS HERE
----------------------------------------------------------------------
experiment_log.csv (written by experiment_logger, one per-run subdirectory)
has qddot_safe (the true, post-CBF commanded acceleration) but NOT the
desired POSITION/VELOCITY trajectory. pentagon_run_<timestamp>.csv (written
directly by pentagon_qddot_commander, always-on, a flat file in franka_logs/
independent of experiment_logger) has q_des/dq_des alongside measured q/dq —
exactly what is needed to confirm rt_torque_controller's gains were really
zero in a "Condition B" run, per its own control law: with p_gains/d_gains=0
it is a pure feedforward pass-through with NO position/velocity correction
at its own 1 kHz loop, so held-joint POSITION TRACKING ERROR should be
visibly larger than with the compiled default gains active — this is a
confirmation, not an assumption; see --a-pentagon/--b-pentagon.

USAGE
-----
    # Single run, acceleration ratio only
    python3 scripts/analyze_isolation_ab.py --a path/to/experiment_log.csv

    # Single run, with tracking-error confirmation
    python3 scripts/analyze_isolation_ab.py \\
        --a path/to/experiment_log.csv --a-pentagon path/to/pentagon_run_*.csv

    # Full A/B comparison
    python3 scripts/analyze_isolation_ab.py \\
        --a franka_logs/<condA_run>/experiment_log.csv \\
        --a-pentagon franka_logs/pentagon_run_<condA_stamp>.csv \\
        --b franka_logs/<condB_run>/experiment_log.csv \\
        --b-pentagon franka_logs/pentagon_run_<condB_stamp>.csv

Pure numpy/csv; no ROS needed.
"""
from __future__ import annotations

import argparse
import csv

import numpy as np

NUM_JOINTS = 7


def load_csv(path):
    with open(path) as f:
        r = csv.reader(f)
        header = next(r)
        idx = {name: i for i, name in enumerate(header)}
        rows = [row for row in r]

    def col(name):
        out = []
        for row in rows:
            v = row[idx[name]]
            out.append(np.nan if v in ('', 'nan', 'NaN') else float(v))
        return np.array(out)

    return dict(
        t=col('t'),
        qddot_nom=np.stack([col(f'qddot_nom_{j}') for j in range(1, 8)], axis=1),
        qddot_safe=np.stack([col(f'qddot_safe_{j}') for j in range(1, 8)], axis=1),
        qddot_meas=np.stack([col(f'qddot_{j}') for j in range(1, 8)], axis=1),
        qdot_meas=np.stack([col(f'qdot_{j}') for j in range(1, 8)], axis=1),
    )


def load_pentagon_csv(path):
    """pentagon_qddot_commander's OWN CSV (pentagon_run_<timestamp>.csv) —
    a different schema from experiment_log.csv, see module docstring."""
    with open(path) as f:
        r = csv.reader(f)
        header = next(r)
        idx = {name: i for i, name in enumerate(header)}
        rows = [row for row in r]

    def col(name):
        out = []
        for row in rows:
            v = row[idx[name]]
            out.append(np.nan if v in ('', 'nan', 'NaN') else float(v))
        return np.array(out)

    return dict(
        t=col('time'),
        q=np.stack([col(f'q{j}') for j in range(1, 8)], axis=1),
        q_des=np.stack([col(f'q_des_{j}') for j in range(1, 8)], axis=1),
        dq=np.stack([col(f'dq{j}') for j in range(1, 8)], axis=1),
        dq_des=np.stack([col(f'dq_des_{j}') for j in range(1, 8)], axis=1),
        qddot_des=np.stack([col(f'qddot_des_{j}') for j in range(1, 8)], axis=1),
    )


def pentagon_tracking_stats(dp, seg_s):
    """Per-joint position/velocity tracking error during its own isolation
    window, from pentagon_run_*.csv. Large error here (vs another run's) is
    the behavioural confirmation that rt_torque_controller's own p_gains/
    d_gains were NOT correcting — it has no other feedback loop."""
    any_nonzero = np.any(np.abs(dp['qddot_des']) > 1e-9, axis=1)
    if not any_nonzero.any():
        raise SystemExit('pentagon_run CSV: qddot_des is all-zero — isolation '
                         'never engaged in this file either')
    i_start = int(np.argmax(any_nonzero))
    t0 = dp['t'][i_start]
    seg = np.clip(np.floor((dp['t'] - t0) / seg_s).astype(int), -1, 7)
    out = {}
    for j in range(NUM_JOINTS):
        m = seg == j
        if m.sum() == 0:
            out[j + 1] = dict(n_rows=0)
            continue
        e_q = dp['q_des'][m, j] - dp['q'][m, j]
        e_dq = dp['dq_des'][m, j] - dp['dq'][m, j]
        out[j + 1] = dict(
            n_rows=int(m.sum()),
            rms_pos_err=float(np.sqrt(np.mean(e_q**2))),
            peak_pos_err=float(np.max(np.abs(e_q))),
            rms_vel_err=float(np.sqrt(np.mean(e_dq**2))),
        )
    return out


def segment(d, seg_s):
    """Return (t_iso0, seg) — seg[i] in {-1..7}, 0..6 = joint1..7's window,
    -1 before isolation starts, 7 after the sequence finished."""
    any_nonzero = np.any(np.abs(d['qddot_nom']) > 1e-9, axis=1)
    if not any_nonzero.any():
        raise SystemExit(
            'no row has a non-zero qddot_nom anywhere in this CSV — '
            'isolation_test never engaged (wrong CSV, or isolation_test:=true '
            'was not actually passed to this run)')
    i_start = int(np.argmax(any_nonzero))
    t_iso0 = d['t'][i_start]
    seg = np.floor((d['t'] - t_iso0) / seg_s).astype(int)
    return t_iso0, np.clip(seg, -1, 7)


def per_joint_stats(d, seg_s):
    """One dict per joint (1-indexed key): rms_cmd, rms_real, peak_cmd,
    peak_real, ratio, n_rows, cbf_reshape_max/mean (|qddot_safe-qddot_nom|
    within that joint's own window), and a coarse near-zero-velocity vs
    moving residual split (stiction vs viscous signature)."""
    t_iso0, seg = segment(d, seg_s)
    out = {}
    for j in range(NUM_JOINTS):
        m = seg == j
        n = int(m.sum())
        if n == 0:
            out[j + 1] = dict(n_rows=0)
            continue
        cmd = d['qddot_safe'][m, j]      # what actually reached the actuator
        nom = d['qddot_nom'][m, j]       # pentagon's own isolation-PD intent
        real = d['qddot_meas'][m, j]
        qdot = d['qdot_meas'][m, j]
        residual = cmd - real
        rms_cmd, rms_real = float(np.sqrt(np.mean(cmd**2))), float(np.sqrt(np.mean(real**2)))
        reshape = np.abs(cmd - nom)

        # Near-zero-velocity vs moving: bottom/top tercile of |qdot| in this
        # joint's OWN window (relative, since the isolation amplitude differs
        # per joint). Not a claim, just a characterization signal.
        aq = np.abs(qdot)
        if np.ptp(aq) > 1e-9:
            lo_thr, hi_thr = np.percentile(aq, [33, 67])
            near_zero = aq <= lo_thr
            moving = aq >= hi_thr
            resid_near_zero = float(np.mean(np.abs(residual[near_zero]))) if near_zero.any() else float('nan')
            resid_moving = float(np.mean(np.abs(residual[moving]))) if moving.any() else float('nan')
        else:
            resid_near_zero = resid_moving = float('nan')

        out[j + 1] = dict(
            n_rows=n, t0=float(d['t'][m].min()), t1=float(d['t'][m].max()),
            rms_cmd=rms_cmd, rms_real=rms_real,
            ratio=(rms_real / rms_cmd if rms_cmd > 1e-9 else float('nan')),
            peak_cmd=float(np.max(np.abs(cmd))), peak_real=float(np.max(np.abs(real))),
            cbf_reshape_max=float(reshape.max()), cbf_reshape_mean=float(reshape.mean()),
            resid_near_zero_vel=resid_near_zero, resid_moving=resid_moving,
        )
    return out


def _print_single(label, stats, ptrack=None):
    print(f'\n=== {label} ===')
    print(f'{"joint":>7s} {"n":>5s} {"rms_cmd":>9s} {"rms_real":>9s} {"ratio":>7s} '
          f'{"cbf_dmax":>9s} {"resid@v~0":>10s} {"resid@moving":>12s}')
    for j in range(1, NUM_JOINTS + 1):
        s = stats[j]
        if s.get('n_rows', 0) == 0:
            print(f'{j:>7d}   (no rows found for this joint)')
            continue
        print(f'{j:>7d} {s["n_rows"]:>5d} {s["rms_cmd"]:>9.4f} {s["rms_real"]:>9.4f} '
              f'{s["ratio"]:>7.4f} {s["cbf_reshape_max"]:>9.4f} '
              f'{s["resid_near_zero_vel"]:>10.4f} {s["resid_moving"]:>12.4f}')
    if ptrack is not None:
        print(f'  -- pentagon_run tracking error (q_des - q_meas), own isolation window --')
        print(f'{"joint":>7s} {"n":>5s} {"rms_pos_err":>12s} {"peak_pos_err":>13s} {"rms_vel_err":>12s}')
        for j in range(1, NUM_JOINTS + 1):
            p = ptrack.get(j, {})
            if not p.get('n_rows'):
                print(f'{j:>7d}   (no rows)')
                continue
            print(f'{j:>7d} {p["n_rows"]:>5d} {p["rms_pos_err"]:>12.5f} '
                  f'{p["peak_pos_err"]:>13.5f} {p["rms_vel_err"]:>12.5f}')


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--a', required=True, help='Condition A experiment_log.csv (or the only run)')
    ap.add_argument('--b', default=None, help='Condition B experiment_log.csv (optional)')
    ap.add_argument('--a-pentagon', default=None,
                     help='Condition A pentagon_run_<stamp>.csv (tracking-error confirmation)')
    ap.add_argument('--b-pentagon', default=None,
                     help='Condition B pentagon_run_<stamp>.csv (tracking-error confirmation)')
    ap.add_argument('--seg-s', type=float, default=2.0,
                     help='isolation_seg_s used for the run (must match what was launched with)')
    args = ap.parse_args()

    da = load_csv(args.a)
    stats_a = per_joint_stats(da, args.seg_s)
    ptrack_a = pentagon_tracking_stats(load_pentagon_csv(args.a_pentagon), args.seg_s) if args.a_pentagon else None
    _print_single('Condition A' if args.b else 'This run', stats_a, ptrack_a)

    max_reshape_a = max((s['cbf_reshape_max'] for s in stats_a.values() if s.get('n_rows')), default=0.0)
    if max_reshape_a > 1.0:
        print(f'\nNOTE: qddot_safe diverged from qddot_nom by up to {max_reshape_a:.2f} '
              f'rad/s^2 during isolation in this run — the CBF is reshaping the '
              f'isolation command on at least one joint. That is expected to be a '
              f'SMALL, joint-limit-only effect this close to a neutral pose; if it '
              f'is large, check the arm was not near a joint/velocity limit at test '
              f'start, and treat this joint\'s ratio with caution.')

    if not args.b:
        return

    db = load_csv(args.b)
    stats_b = per_joint_stats(db, args.seg_s)
    ptrack_b = pentagon_tracking_stats(load_pentagon_csv(args.b_pentagon), args.seg_s) if args.b_pentagon else None
    _print_single('Condition B', stats_b, ptrack_b)

    print('\n=== A vs B: the demonstrable effect of rt_torque_controller\'s Kp/Kd ===')
    print(f'{"joint":>7s} {"ratio_A":>9s} {"ratio_B":>9s} {"delta(B-A)":>11s}')
    for j in range(1, NUM_JOINTS + 1):
        sa, sb = stats_a[j], stats_b[j]
        if not sa.get('n_rows') or not sb.get('n_rows'):
            print(f'{j:>7d}   (missing data in A or B)')
            continue
        delta = sb['ratio'] - sa['ratio']
        print(f'{j:>7d} {sa["ratio"]:>9.4f} {sb["ratio"]:>9.4f} {delta:>11.4f}')

    if ptrack_a is not None and ptrack_b is not None:
        print('\n=== A vs B: pentagon_run position tracking error (rms, rad) ===')
        print(f'{"joint":>7s} {"posErr_A":>9s} {"posErr_B":>9s} {"ratio B/A":>10s}')
        for j in range(1, NUM_JOINTS + 1):
            pa, pb = ptrack_a.get(j, {}), ptrack_b.get(j, {})
            if not pa.get('n_rows') or not pb.get('n_rows'):
                print(f'{j:>7d}   (missing data in A or B)')
                continue
            ra = pa['rms_pos_err']
            rb = pb['rms_pos_err']
            ratio = rb / ra if ra > 1e-9 else float('nan')
            print(f'{j:>7d} {ra:>9.5f} {rb:>9.5f} {ratio:>10.2f}')
        print('\nIf B\'s tracking error is NOT visibly larger than A\'s on most '
              'joints, the two runs may not actually differ in '
              'rt_torque_controller\'s gains as assumed -- do not trust the '
              'ratio_A/ratio_B table above as a gains A/B result in that case.')


if __name__ == '__main__':
    main()
