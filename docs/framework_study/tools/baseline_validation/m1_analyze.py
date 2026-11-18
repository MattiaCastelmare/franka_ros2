#!/usr/bin/env python3
"""M1: per-run metrics, validity checks and arm comparison for the executor-thread replays.

Pre-registered in ``refactor_r1_m1/protocol/M1_PROTOCOL.md`` (§6–§8). Read-only on its inputs.

  python3 -I m1_analyze.py <runs_dir> <e3_05_dir> <out.json> [<run_id> ...]

* ``runs_dir``: per run, ``<id>.json`` + ``<id>_raw.npz`` (``r1_executor_replay.py``) and
  ``<id>_m1.json`` + ``<id>_m1.npz`` (``m1_executor_replay.py``).
* ``e3_05_dir``: the sealed E3-05 run directory. It supplies the reference ``n_rows``, build and
  solve series (``analysis/s6b_series.npz``), t0 (``analysis/h2_metrics.json``) and the recorded
  bag for the input counts (opened read-only and immutable).
* Run ids default to every ``<id>_m1.json`` in ``runs_dir``.

Window: node (sim) time [t0 + 2, t0 + 30] s, the same window as the E3 H2 analysis.
Tick times are the node time at ``_qp_tick`` entry.
* "Late" means the sim gap to the previous tick is > 15 ms.
* A "missed slot" is a 10 ms slot of the window with no tick entry.
* A "new snapshot" tick is one whose constraint snapshot differs from the previous tick's.
"""
import glob
import json
import os
import sqlite3
import sys

import numpy as np

RUNS, E3, OUT = sys.argv[1], sys.argv[2], sys.argv[3]
IDS = sys.argv[4:] or sorted(os.path.basename(p)[:-8] for p in glob.glob(os.path.join(RUNS, '*_m1.json')))
T0 = float(json.load(open(os.path.join(E3, 'analysis/h2_metrics.json')))['placement']['t0'])
W0, W1 = T0 + 2.0, T0 + 30.0
SPAN = W1 - W0
F_ROWS, F_DOBS, F_NVIOL, F_FAULT, F_FALLBACK, F_SOLVE, F_JSAGE, F_DCAP, F_SNAP, F_BUILD = 0, 4, 3, 2, 11, 13, 15, 16, 18, 37
ROW_EDGES = (24.0, 28.0)          # fixed n_rows tercile edges (E3-05 terciles, C-073)
GATE = dict(rtf=(0.63, 0.77), rows_p50_tol=3.0, rows_mean_tol=3.0)


def missed_slots(slots):
    """Due 10 ms grid points in [W0, W1) with no tick (a tick counts for slot floor((t + 0.5 ms) / 10 ms)).

    Addendum 03, defect D-M1-2: this was ``round(SPAN / 0.01) - len(unique(slots))``, which assumes
    exactly 2800 slots and goes negative when ticks sit on both edges of the inclusive window.
    """
    due = np.arange(int(np.ceil(W0 / 0.01 - 1e-9)), int(np.ceil(W1 / 0.01 - 1e-9)))
    return int(np.setdiff1d(due, np.unique(slots)).size)


def st(x, scale=1.0):
    x = np.asarray(x, float) * scale
    x = x[np.isfinite(x)]
    if not x.size:
        return None
    return dict(n=int(x.size), p5=float(np.percentile(x, 5)), p50=float(np.percentile(x, 50)),
                p95=float(np.percentile(x, 95)), p99=float(np.percentile(x, 99)),
                max=float(x.max()), mean=float(x.mean()))


def bag_counts():
    db = glob.glob(os.path.join(E3, 'run/batch/cells/*/bag/bag_0.db3'))[0]
    con = sqlite3.connect(f'file:{db}?mode=ro&immutable=1', uri=True)
    out = {}
    for tid, name in con.execute('select id, name from topics'):
        n = con.execute('select count(*) from messages where topic_id=? and timestamp>=? and timestamp<?',
                        (tid, int(W0 * 1e9), int(W1 * 1e9))).fetchone()[0]
        out[name] = n
    con.close()
    return out


def e3_reference():
    z = np.load(os.path.join(E3, 'analysis/s6b_series.npz'))
    tS, S = z['tS'], z['S']
    e = (tS >= W0) & (tS <= W1)
    b = S[:, F_BUILD]
    newb = np.r_[np.isfinite(b[0]), (b[1:] != b[:-1]) & np.isfinite(b[1:])]
    return dict(t=tS[e], rows=S[e, F_ROWS],
                summary=dict(n_rows=st(S[e, F_ROWS]), build_wall_ms=st(b[e & newb]),
                             solve_wall_ms=st(S[e, F_SOLVE]), snapshot_age_ms=st(S[e, F_SNAP], 1e3),
                             obstacle_in_range_frac=float(np.isfinite(S[e, F_DOBS]).mean()),
                             ticks_per_sim_s=float(e.sum() / SPAN)))


def analyse(rid, ref, counts):
    S = json.load(open(os.path.join(RUNS, rid + '.json')))
    X = json.load(open(os.path.join(RUNS, rid + '_m1.json')))
    Z = np.load(os.path.join(RUNS, rid + '_raw.npz'))
    st_rows = np.load(os.path.join(RUNS, rid + '_m1.npz'))['status']
    clk = Z['clock']                                       # (wall, sim) per /clock callback
    R = {'id': rid, 'arm': X['arm'], 'executor_class': X.get('executor_class'),
         'worker_threads': X.get('worker_threads'), 'node_executor_mode': X.get('node_executor_mode'),
         'player_rc': S.get('player_rc'), 'replay_window': S.get('window')}
    # true playback rate in the window: slope of message sim time over callback wall time
    k = (clk[:, 1] >= W0) & (clk[:, 1] <= W1)
    slope = np.polyfit(clk[k, 0], clk[k, 1], 1)[0] if k.sum() > 10 else float('nan')
    a = np.max(clk[:, 1] - S['rate'] * clk[:, 0])
    T = lambda w: a + S['rate'] * w  # noqa: E731
    wall0, wall1 = np.interp([W0, W1], clk[k, 1], clk[k, 0]) if k.sum() > 10 else (np.nan, np.nan)
    R['sim_time_rate'] = dict(effective_rtf=float(slope), window_wall_s=float(wall1 - wall0))
    # ticks (entry node time)
    q = Z['qp']
    q = q[(q[:, 5] >= W0) & (q[:, 5] <= W1)]
    t_in = np.sort(q[:, 5])
    gaps = np.diff(t_in)
    slots = np.floor((t_in + 5e-4) / 0.01).astype(int)
    R['sim'] = dict(ticks=int(len(t_in)), ticks_per_sim_s=len(t_in) / SPAN,
                    missed_slots=missed_slots(slots),
                    tick_gap_ms=st(gaps, 1e3), late_gt15ms=int((gaps > 0.015).sum()),
                    late_frac=float((gaps > 0.015).mean()), gaps_ge50ms=int((gaps >= 0.05).sum()),
                    gaps_ge100ms=int((gaps >= 0.10).sum()))
    R['wall'] = dict(ticks_per_wall_s=len(t_in) / (wall1 - wall0),
                     tick_wall_ms=st(q[:, 2], 1e3), tick_cpu_ms=st(q[:, 3], 1e3),
                     tick_wait_ms=st(q[:, 2] - q[:, 3], 1e3))
    for key in ('rebuild', 'solve', 'js', 'nom', 'dist'):
        x = Z[key]
        x = x[(x[:, 5] >= W0) & (x[:, 5] <= W1)]
        R['wall'][key] = dict(per_sim_s=len(x) / SPAN, wall_ms=st(x[:, 2], 1e3), cpu_ms=st(x[:, 3], 1e3),
                              wait_ms=st(x[:, 2] - x[:, 3], 1e3))
    # input delivery vs the bag
    nclk = int(((clk[:, 1] >= W0) & (clk[:, 1] < W1)).sum())
    inp = {'clock': (nclk, counts.get('/clock')),
           'joint_states': (R['wall']['js']['per_sim_s'] * SPAN, counts.get('/NS_1/franka/joint_states')),
           'qddot_nom': (R['wall']['nom']['per_sim_s'] * SPAN, counts.get('/NS_1/qddot_nom')),
           'distances': (R['wall']['dist']['per_sim_s'] * SPAN, counts.get('/cbf/per_link_distances'))}
    R['inputs'] = {n: dict(processed=int(round(p)), in_bag=int(b), processed_frac=float(p / b) if b else None)
                   for n, (p, b) in inp.items()}
    # per tick: snapshot identity and ages (tick_in rows are in entry order)
    ti = Z['tick_in']
    ti = ti[(ti[:, 1] >= W0) & (ti[:, 1] <= W1)]
    ti = ti[np.argsort(ti[:, 1])]
    snap = ti[:, 7]
    new = np.r_[np.isfinite(snap[0]), (snap[1:] != snap[:-1]) & np.isfinite(snap[1:])]
    late = np.r_[False, np.diff(ti[:, 1]) > 0.015]
    R['late_vs_rebuild'] = dict(p_late_new_snapshot=float(late[new].mean()) if new.any() else None,
                                p_late_otherwise=float(late[~new].mean()) if (~new).any() else None,
                                n_new=int(new.sum()), n_other=int((~new).sum()))
    Tt = T(ti[:, 0])
    R['age_ms'] = dict(snapshot_node=st(ti[:, 1] - ti[:, 8], 1e3), snapshot_true=st(Tt - T(ti[:, 7]), 1e3),
                       clock_lag_true=st(Tt - ti[:, 1], 1e3), joint_state_true=st(Tt - ti[:, 2], 1e3),
                       distances_capture_true=st(Tt - ti[:, 6], 1e3))
    # /cbf_status rows built by the node (same fields as E3's cbf_status)
    sr = st_rows[(st_rows[:, 1] >= W0) & (st_rows[:, 1] <= W1)]
    D = sr[:, 2:]
    rows = D[:, F_ROWS]
    b = D[:, F_BUILD]
    newb = np.r_[np.isfinite(b[0]), (b[1:] != b[:-1]) & np.isfinite(b[1:])] if len(b) else np.zeros(0, bool)
    R['status'] = dict(rows=int(len(D)), n_rows=st(rows), build_wall_ms=st(b[newb]), solve_wall_ms=st(D[:, F_SOLVE]),
                       snapshot_age_ms=st(D[:, F_SNAP], 1e3), obstacle_in_range_frac=float(np.isfinite(D[:, F_DOBS]).mean()),
                       n_violated_max=float(np.nanmax(D[:, F_NVIOL])), fault_ticks=int((D[:, F_FAULT] > 0).sum()),
                       fallback_levels={str(int(u)): int(c) for u, c in zip(*np.unique(D[:, F_FALLBACK], return_counts=True))},
                       build_rows_le30_p50=st(b[newb & (rows <= 30)]), build_rows_gt30_p50=st(b[newb & (rows > 30)]))
    # late fraction by n_rows tercile (status row i is published by tick i, same order)
    srt = sr[:, 1]
    sgap = np.r_[np.nan, np.diff(srt)]
    lt = sgap > 0.015
    terc = {}
    for lab, lo, hi in (('low', -1, ROW_EDGES[0]), ('mid', ROW_EDGES[0], ROW_EDGES[1]), ('high', ROW_EDGES[1], 1e9)):
        s = (rows > lo) & (rows <= hi) & np.isfinite(sgap)
        terc[lab] = dict(n=int(s.sum()), late_frac=float(lt[s].mean()) if s.any() else None)
    R['late_by_rows'] = terc
    # workload equivalence with E3-05 at matched sim times
    j = np.clip(np.searchsorted(ref['t'], srt), 0, len(ref['t']) - 1)
    dr = np.abs(rows - ref['rows'][j])
    R['workload_vs_e3_05'] = dict(rows_p50_diff=float(np.median(rows) - np.median(ref['rows'])),
                                  rows_mean_diff=float(rows.mean() - ref['rows'].mean()),
                                  matched_abs_diff=st(dr), matched_within_2=float((dr <= 2).mean()))
    R['cpu'] = dict(process_pct_of_core=S.get('process_cpu_pct_of_core'),
                    executor_loop_pct=S.get('executor_loop_cpu_pct'), threads=S.get('thread_cpu_pct_of_core'),
                    note='CPU window = the manifest episode [t_start_sim, t_end_sim], not the analysis window')
    # validity (protocol §7)
    V = {}
    V['player_rc_0'] = S.get('player_rc') == 0
    V['window_snapshots'] = 'window' in S
    want = {'single': ('SingleThreadedExecutor', None), 'mt1': ('MultiThreadedExecutor', 1),
            'mt3': ('MultiThreadedExecutor', 3)}[X['arm']]
    V['executor_as_intended'] = (X.get('executor_class') == want[0]
                                 and (want[1] is None or X.get('worker_threads') == want[1]))
    V['rtf_in_range'] = GATE['rtf'][0] <= slope <= GATE['rtf'][1]
    V['status_rows_present'] = len(D) > 0.5 * SPAN * 50
    V['rebuilds_present'] = R['wall']['rebuild']['per_sim_s'] > 10
    V['obstacle_rows_present'] = R['status']['obstacle_in_range_frac'] > 0.5
    V['workload_rows_p50'] = abs(R['workload_vs_e3_05']['rows_p50_diff']) <= GATE['rows_p50_tol']
    V['workload_rows_mean'] = abs(R['workload_vs_e3_05']['rows_mean_diff']) <= GATE['rows_mean_tol']
    R['validity'] = {k: bool(v) for k, v in V.items()}      # numpy bools are not JSON-serialisable
    R['valid'] = bool(all(V.values()))
    return R


def summarise(res):
    arms = ('single', 'mt1', 'mt3')
    M = {'ticks_per_sim_s': ('sim', 'ticks_per_sim_s'), 'ticks_per_wall_s': ('wall', 'ticks_per_wall_s'),
         'missed_slots': ('sim', 'missed_slots'), 'late_gt15ms': ('sim', 'late_gt15ms'),
         'gap_p50_ms': ('sim', 'tick_gap_ms', 'p50'), 'gap_p95_ms': ('sim', 'tick_gap_ms', 'p95'),
         'gap_p99_ms': ('sim', 'tick_gap_ms', 'p99'), 'gap_max_ms': ('sim', 'tick_gap_ms', 'max'),
         'gaps_ge50ms': ('sim', 'gaps_ge50ms'),
         'rebuild_per_sim_s': ('wall', 'rebuild', 'per_sim_s'), 'rebuild_wall_p50': ('wall', 'rebuild', 'wall_ms', 'p50'),
         'rebuild_wall_p95': ('wall', 'rebuild', 'wall_ms', 'p95'), 'rebuild_cpu_p50': ('wall', 'rebuild', 'cpu_ms', 'p50'),
         'rebuild_wait_p95': ('wall', 'rebuild', 'wait_ms', 'p95'),
         'solve_wall_p50': ('wall', 'solve', 'wall_ms', 'p50'), 'solve_wall_p95': ('wall', 'solve', 'wall_ms', 'p95'),
         'solve_cpu_p50': ('wall', 'solve', 'cpu_ms', 'p50'),
         'tick_wall_p50': ('wall', 'tick_wall_ms', 'p50'), 'tick_cpu_p50': ('wall', 'tick_cpu_ms', 'p50'),
         'tick_wait_p95': ('wall', 'tick_wait_ms', 'p95'),
         'p_late_new_snapshot': ('late_vs_rebuild', 'p_late_new_snapshot'),
         'p_late_otherwise': ('late_vs_rebuild', 'p_late_otherwise'),
         'snapshot_age_p50_ms': ('status', 'snapshot_age_ms', 'p50'), 'snapshot_age_p95_ms': ('status', 'snapshot_age_ms', 'p95'),
         'n_rows_p50': ('status', 'n_rows', 'p50'), 'status_build_p50': ('status', 'build_wall_ms', 'p50'),
         'status_solve_p50': ('status', 'solve_wall_ms', 'p50'),
         'js_processed_frac': ('inputs', 'joint_states', 'processed_frac'),
         'clock_processed_frac': ('inputs', 'clock', 'processed_frac'),
         'js_age_p95_ms': ('age_ms', 'joint_state_true', 'p95'), 'clock_lag_p95_ms': ('age_ms', 'clock_lag_true', 'p95'),
         'process_cpu_pct': ('cpu', 'process_pct_of_core'), 'executor_loop_cpu_pct': ('cpu', 'executor_loop_pct'),
         'effective_rtf': ('sim_time_rate', 'effective_rtf')}

    def get(r, path):
        v = r
        for p in path:
            v = v.get(p) if isinstance(v, dict) else None
        return v
    per_arm = {}
    for arm in arms:
        rr = [r for r in res if r['arm'] == arm]
        per_arm[arm] = {}
        for m, path in M.items():
            vals = [get(r, path) for r in rr]
            vals = [float(v) for v in vals if v is not None]
            per_arm[arm][m] = dict(values=vals, mean=float(np.mean(vals)) if vals else None,
                                   sd=float(np.std(vals, ddof=1)) if len(vals) > 1 else None,
                                   min=min(vals) if vals else None, max=max(vals) if vals else None)
    # pre-registered position of mt1 between single (0) and mt3 (1), with a separation check
    pos = {}
    for m in ('late_gt15ms', 'missed_slots', 'ticks_per_sim_s', 'gap_p99_ms', 'rebuild_wall_p50',
              'solve_wall_p50', 'tick_wait_p95', 'p_late_otherwise', 'snapshot_age_p50_ms'):
        s, o, t = per_arm['single'][m], per_arm['mt1'][m], per_arm['mt3'][m]
        if None in (s['mean'], o['mean'], t['mean']):
            continue
        d = t['mean'] - s['mean']
        sds = [x['sd'] or 0.0 for x in (s, t)]
        separated = (abs(d) > 2 * max(sds)) and (s['max'] < t['min'] or t['max'] < s['min'])
        f = (o['mean'] - s['mean']) / d if d else None
        label = None
        if separated and f is not None:
            label = 'like_single' if f <= 1 / 3 else ('like_mt3' if f >= 2 / 3 else 'intermediate')
        pos[m] = dict(single=s['mean'], mt1=o['mean'], mt3=t['mean'], mt3_minus_single=d,
                      separated=bool(separated), f_mt1=f, label=label if separated else 'not_separated')
    pairs = {}
    for a, b in (('single', 'mt1'), ('mt1', 'mt3'), ('single', 'mt3')):
        pairs[f'{b}_minus_{a}'] = {m: (per_arm[b][m]['mean'] - per_arm[a][m]['mean'])
                                   if None not in (per_arm[a][m]['mean'], per_arm[b][m]['mean']) else None for m in M}
    return per_arm, pos, pairs


counts = bag_counts()
ref = e3_reference()
res = [analyse(i, ref, counts) for i in IDS]
out = dict(window_sim=[W0, W1], t0=T0, bag_counts_window=counts, e3_05_reference=ref['summary'], runs=res)
if {r['arm'] for r in res} >= {'single', 'mt1', 'mt3'}:
    out['per_arm'], out['mt1_position'], out['pair_differences'] = summarise(res)
json.dump(out, open(OUT, 'w'), indent=1)
for r in res:
    s, w = r['sim'], r['wall']
    print(f"{r['id']:14s} {r['arm']:6s} {r['executor_class']}/{r['worker_threads']} valid={r['valid']} "
          f"ticks/sim s {s['ticks_per_sim_s']:.1f} wall/s {w['ticks_per_wall_s']:.1f} missed {s['missed_slots']} "
          f"late {s['late_gt15ms']} gap p99/max {s['tick_gap_ms']['p99']:.1f}/{s['tick_gap_ms']['max']:.1f} ms "
          f"rebuild p50 {w['rebuild']['wall_ms']['p50']:.2f} solve p50 {w['solve']['wall_ms']['p50']:.2f} "
          f"rows p50 {r['status']['n_rows']['p50']:.0f} (E3-05 diff {r['workload_vs_e3_05']['rows_p50_diff']:+.1f}) "
          f"rtf {r['sim_time_rate']['effective_rtf']:.3f} cpu {r['cpu']['process_pct_of_core']}")
    bad = [k for k, v in r['validity'].items() if not v]
    if bad:
        print('   INVALID:', bad)
if 'mt1_position' in out:
    for m, v in out['mt1_position'].items():
        print(f"  {m:22s} single {v['single']:.3g}  mt1 {v['mt1']:.3g}  mt3 {v['mt3']:.3g}  -> {v['label']}"
              + (f" (f={v['f_mt1']:.2f})" if v['f_mt1'] is not None else ''))
