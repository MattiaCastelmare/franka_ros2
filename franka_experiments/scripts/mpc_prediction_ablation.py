#!/usr/bin/env python3
"""MPC human rows, CV forecast vs frozen arm: the metrics of human_logging runs side by side.

Each run is one experiment_logs/<mpc|mpc_cbf>/<run_name> folder written by human_logging
(easy_torque commander:=mpc human:=true, mpc_human_mode:=prediction | static; the driver
scripts/mpc_prediction_ablation.sh launches the pairs). Per run:

  exposure    time and approaches with the arm closer than d_activate (the runs are not
              paired: the replayed person meets the robot at other task phases, so compare
              the metrics below against how much interaction each run had)
  safety      human-robot distance (human_distance, robot at its real q): min, p5, time
              below d_safe and below d_safe + δ
  reaction    per approach, the first binding human row of the MPC: the real distance at
              that moment and how long before the closest point (both larger = earlier)
  slack       cycles with a human-row slack > 1 mm, the largest slack
  shield      mpc_cbf only: ‖q̈_safe − q̈_nom‖ (q̈_nom held between MPC cycles), the time
              the CBF changes the MPC's command (all, and with the arm near), its violated rows
  tracking    TCP error to the task preview (pick_place, reference_only)
  solver      solve time p50 / p95 and fallbacks

Every series is cut to [first MPC solve, end]; the time base is recv_time (sim time).

    python3 scripts/mpc_prediction_ablation.py RUN_DIR [RUN_DIR ...] [--csv out.csv]
"""
import argparse
import csv
import math
import os
import sys

import numpy as np

# Defaults of config/mpc.yaml (safety, human)
D_SAFE, DELTA, D_ACTIVATE = 0.15, 0.05, 0.40
SLACK_USED_M = 1e-3          # a row "used its slack" above this
SHIELD_ACTIVE = 0.5          # [rad/s²] ‖q̈_safe − q̈_nom‖ above this = the CBF changed the command
                             # (far from people the change is ~0.1: solver tolerance, self-collision rows)
EPISODE_GAP_S = 0.5          # approaches closer than this in time are one approach
REACTION_WINDOW_S = 1.0      # a binding row up to this long before the approach counts
MAX_SAMPLE_GAP_S = 0.2       # longer holes in a series are not counted as time spent


def read_columns(path, names):
    """{name: float array} of the columns names of a human_logging CSV ({} if the file is missing)."""
    if not os.path.exists(path):
        return {}
    with open(path) as fh:
        reader = csv.reader(fh)
        header = next(reader, [])
        idx = [header.index(name) for name in names if name in header]
        rows = [[row[i] for i in idx] for row in reader if len(row) == len(header)]
    out = {}
    for k, name in enumerate(name for name in names if name in header):
        values = []
        for row in rows:
            try:
                values.append(float(row[k]))
            except ValueError:
                values.append(math.nan)
        out[name] = np.asarray(values)
    return out


def time_where(t, mask):
    """Seconds spent where mask holds (sample-and-hold, holes longer than MAX_SAMPLE_GAP_S skipped)."""
    if len(t) < 2:
        return 0.0
    dt = np.diff(t)
    return float(np.sum(np.where((dt <= MAX_SAMPLE_GAP_S) & mask[:-1], dt, 0.0)))


def episodes(t, d, threshold):
    """[(t_start, t_end, t_min, d_min)] of the stretches with d < threshold."""
    close = np.flatnonzero(d < threshold)
    if close.size == 0:
        return []
    groups, start = [], close[0]
    for prev, cur in zip(close[:-1], close[1:]):
        if t[cur] - t[prev] > EPISODE_GAP_S:
            groups.append((start, prev))
            start = cur
    groups.append((start, close[-1]))
    out = []
    for i, j in groups:
        k = i + int(np.argmin(d[i:j + 1]))
        out.append((t[i], t[j], t[k], d[k]))
    return out


def run_metrics(run_dir):
    path = lambda name: os.path.join(run_dir, f'experiment_{name}.csv')
    diag = read_columns(path('mpc_diagnostics'), ['recv_time', 'solved', 'fallback', 'solve_time_ms',
                                                 'human_rows', 'human_binding', 'max_slack'])
    if not diag or len(diag['recv_time']) == 0:
        sys.exit(f'{run_dir}: no experiment_mpc_diagnostics.csv (not an MPC run?)')
    t0, t_end = diag['recv_time'][0], diag['recv_time'][-1]
    cut = lambda cols: {k: v[(cols['recv_time'] >= t0) & (cols['recv_time'] <= t_end)] for k, v in cols.items()}
    m = {'run': os.path.basename(os.path.normpath(run_dir)),
         'controller': os.path.basename(os.path.dirname(os.path.normpath(run_dir))),
         'duration_s': t_end - t0}

    # ── Exposure and safety (human_distance with the robot at its real q)
    safety = read_columns(path('safety_distance'), ['recv_time', 'min_distance'])
    safety = cut(safety) if safety else {'recv_time': np.array([]), 'min_distance': np.array([])}
    t, d = safety['recv_time'], safety['min_distance']
    ok = np.isfinite(d)
    t, d = t[ok], d[ok]
    approaches = episodes(t, d, D_ACTIVATE)
    m['time_near_s'] = time_where(t, d < D_ACTIVATE)
    m['approaches'] = len(approaches)
    m['d_min_m'] = float(d.min()) if d.size else math.nan
    m['d_p5_m'] = float(np.percentile(d[d < D_ACTIVATE], 5)) if np.any(d < D_ACTIVATE) else math.nan
    m['time_below_dsafe_s'] = time_where(t, d < D_SAFE)
    m['time_below_dsafe_delta_s'] = time_where(t, d < D_SAFE + DELTA)
    m['approaches_below_dsafe_delta'] = sum(a[3] < D_SAFE + DELTA for a in approaches)
    m['approach_dmin_median_m'] = float(np.median([a[3] for a in approaches])) if approaches else math.nan

    # ── Reaction: the first binding human row before each approach's closest point
    diag = cut(diag)
    t_bind = diag['recv_time'][diag['human_binding'] > 0]
    leads, d_react = [], []
    for t_start, _, t_min, _ in approaches:
        before = t_bind[(t_bind >= t_start - REACTION_WINDOW_S) & (t_bind <= t_min)]
        leads.append(t_min - before[0] if before.size else math.nan)
        d_react.append(np.interp(before[0], t, d) if before.size else math.nan)
    react = np.isfinite(leads)
    m['approaches_with_reaction'] = int(np.sum(react))
    m['reaction_distance_median_m'] = float(np.median(np.asarray(d_react)[react])) if react.any() else math.nan
    m['reaction_lead_median_s'] = float(np.median(np.asarray(leads)[react])) if react.any() else math.nan

    # ── Slack and solver
    m['cycles'] = len(diag['recv_time'])
    m['cycles_binding'] = int(np.sum(diag['human_binding'] > 0))
    m['cycles_slack_used'] = int(np.sum(diag['max_slack'] > SLACK_USED_M))
    m['max_slack_m'] = float(np.nanmax(diag['max_slack']))
    m['fallbacks'] = int(np.nansum(diag['fallback']))
    m['solve_ms_p50'] = float(np.nanmedian(diag['solve_time_ms']))
    m['solve_ms_p95'] = float(np.nanpercentile(diag['solve_time_ms'], 95))

    # ── Shield (mpc_cbf): how much the CBF changes the MPC's q̈
    joints = [f'qddot_nom_{j}' for j in range(1, 8)]
    nom = read_columns(path('qddot_nom'), ['recv_time'] + joints)
    safe = read_columns(path('qddot_safe'), ['recv_time'] + [f'qddot_safe_{j}' for j in range(1, 8)])
    if nom and safe and len(nom['recv_time']) and len(safe['recv_time']):
        nom, safe = cut(nom), cut(safe)
        q_nom = np.column_stack([nom[name] for name in joints])
        q_safe = np.column_stack([safe[f'qddot_safe_{j}'] for j in range(1, 8)])
        held = np.clip(np.searchsorted(nom['recv_time'], safe['recv_time'], side='right') - 1, 0, None)
        valid = safe['recv_time'] >= nom['recv_time'][0]
        change = np.linalg.norm(q_safe - q_nom[held], axis=1)
        t_safe = safe['recv_time']
        m['shield_time_active_s'] = time_where(t_safe[valid], change[valid] > SHIELD_ACTIVE)
        near = np.interp(t_safe, t, d) < D_ACTIVATE if d.size else np.zeros_like(valid)
        m['shield_time_active_near_s'] = time_where(t_safe[valid], (change > SHIELD_ACTIVE)[valid] & near[valid])
        m['shield_change_p95'] = float(np.percentile(change[valid], 95)) if np.any(valid) else math.nan
        m['shield_change_max'] = float(change[valid].max()) if np.any(valid) else math.nan
        status = cut(read_columns(path('cbf_status'), ['recv_time', 'n_violated', 'slack']))
        if status:
            m['shield_time_violated_s'] = time_where(status['recv_time'], status['n_violated'] > 0)

    # ── Tracking
    task = read_columns(path('task'), ['recv_time', 'ee_err'])
    if task:
        err = cut(task)['ee_err']
        err = err[np.isfinite(err)]
        if err.size:
            m['ee_err_mean_mm'] = 1e3 * float(err.mean())
            m['ee_err_p95_mm'] = 1e3 * float(np.percentile(err, 95))
            m['ee_err_max_mm'] = 1e3 * float(err.max())
    return m


ROWS = [  # (key, label, format)
    ('controller', 'controller', '{}'),
    ('duration_s', 'duration [s]', '{:.0f}'),
    ('time_near_s', f'time arm < {D_ACTIVATE:.2f} m [s]', '{:.1f}'),
    ('approaches', 'approaches', '{}'),
    ('d_min_m', 'distance min [m]', '{:.3f}'),
    ('d_p5_m', 'distance p5 while near [m]', '{:.3f}'),
    ('approach_dmin_median_m', 'closest point per approach, median [m]', '{:.3f}'),
    ('time_below_dsafe_s', f'time < d_safe {D_SAFE:.2f} m [s]', '{:.2f}'),
    ('time_below_dsafe_delta_s', f'time < d_safe+δ {D_SAFE + DELTA:.2f} m [s]', '{:.2f}'),
    ('approaches_below_dsafe_delta', 'approaches below d_safe+δ', '{}'),
    ('approaches_with_reaction', 'approaches with a binding row first', '{}'),
    ('reaction_distance_median_m', 'distance at the first binding row, median [m]', '{:.3f}'),
    ('reaction_lead_median_s', 'first binding row before the closest point, median [s]', '{:.2f}'),
    ('cycles_binding', 'MPC cycles with binding human rows', '{}'),
    ('cycles_slack_used', 'MPC cycles with slack > 1 mm', '{}'),
    ('max_slack_m', 'largest slack [m]', '{:.3f}'),
    ('shield_time_active_s', f'CBF changes q̈ (> {SHIELD_ACTIVE} rad/s²) [s]', '{:.2f}'),
    ('shield_time_active_near_s', f'  of which with the arm < {D_ACTIVATE:.2f} m [s]', '{:.2f}'),
    ('shield_change_p95', '‖q̈_safe − q̈_nom‖ p95 [rad/s²]', '{:.2f}'),
    ('shield_change_max', '‖q̈_safe − q̈_nom‖ max [rad/s²]', '{:.2f}'),
    ('shield_time_violated_s', 'CBF rows violated [s]', '{:.2f}'),
    ('ee_err_mean_mm', 'TCP error mean [mm]', '{:.1f}'),
    ('ee_err_p95_mm', 'TCP error p95 [mm]', '{:.1f}'),
    ('ee_err_max_mm', 'TCP error max [mm]', '{:.1f}'),
    ('solve_ms_p50', 'solve time p50 [ms]', '{:.2f}'),
    ('solve_ms_p95', 'solve time p95 [ms]', '{:.2f}'),
    ('fallbacks', 'failed solves', '{}'),
]


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('runs', nargs='+', help='experiment_logs/<mpc|mpc_cbf>/<run_name> folders')
    parser.add_argument('--csv', help='also write one row per run here')
    args = parser.parse_args()

    metrics = [run_metrics(run) for run in args.runs]
    fmt = lambda spec, v: '–' if v is None or (isinstance(v, float) and math.isnan(v)) else spec.format(v)
    label_w = max(len(label) for _, label, _ in ROWS)
    col_w = max(12, *(len(m['run']) for m in metrics))
    print(' ' * label_w + '  ' + '  '.join(m['run'].rjust(col_w) for m in metrics))
    for key, label, spec in ROWS:
        print(label.ljust(label_w) + '  ' + '  '.join(fmt(spec, m.get(key)).rjust(col_w) for m in metrics))

    if args.csv:
        keys = ['run'] + [key for key, _, _ in ROWS]
        with open(args.csv, 'w', newline='') as fh:
            writer = csv.DictWriter(fh, fieldnames=keys, extrasaction='ignore')
            writer.writeheader()
            writer.writerows(metrics)
        print(f'\nwrote {args.csv}')


if __name__ == '__main__':
    main()
