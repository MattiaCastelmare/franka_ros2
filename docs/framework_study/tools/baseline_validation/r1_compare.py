#!/usr/bin/env python3
"""R1 — analyse the executor A/B replays (`r1_executor_replay.py`) against each other and against
the deterministic, schedule-ideal reference (`r1_sequential_outputs.py`).

  python3 -I r1_compare.py <ab_dir> <seq_reference.npz> <out.json> <condition> [<condition> ...]

A condition is a run-name prefix; runs are <ab_dir>/<condition>_<mode>_r<k>.json/_raw.npz.

Per run (episode window from the S6B manifest, sim [10.55, 40.56]):
  * ticks per sim s; late ticks (sim gap >= 15 ms); missed 10 ms slots; max sim gap;
  * wall / thread-CPU p50, p95 of every callback and of `_solve`; process CPU (% of one core);
  * rebuild timing: node-time gap between rebuild starts (>= 30 ms = a missed 20 ms period) and
    the delay of each rebuild start (true time) after its due 20 ms grid point;
  * input age at each tick, in TRUE playback time. The true sim time at wall w is
    T(w) = a + rate * w, with a = max_i (clock_i - rate * w_i) over the /clock callbacks
    (a callback can be late, never early). The node's own clock lags T while callbacks queue.
    ages: clock lag = T - node_now; joint state = T - header stamp of the last processed joint
    state; qddot_nom and distances = T - (receipt stamp + 0) [receipt stamps are node time];
    constraint snapshot = T(tick) - T(build end);
    command latency = T(publish) - joint-state header stamp used.
  * outputs: each published command is matched to its 10 ms slot k = floor((now + 0.5 ms)/10 ms)
    (first per slot kept); the deviation from the sequential reference in the same slot is the
    effect of scheduling, since the reference runs the identical code with every input delivered
    on time. Cross-run deviations (same mode vs different mode) give the noise floor.
"""
import itertools
import json
import os
import sys

import numpy as np

AB, REF, OUT = sys.argv[1], sys.argv[2], sys.argv[3]
CONDS = sys.argv[4:]
T0, T1 = 10.55, 40.56
SPAN = T1 - T0


def pct(x, q):
    x = np.asarray(x, float)
    x = x[np.isfinite(x)]
    return float(np.percentile(x, q)) if x.size else float('nan')


def st(x, scale=1e3):
    x = np.asarray(x, float) * scale
    x = x[np.isfinite(x)]
    if not x.size:
        return None
    return dict(n=int(x.size), p50=float(np.percentile(x, 50)), p95=float(np.percentile(x, 95)),
                p99=float(np.percentile(x, 99)), max=float(x.max()))


def slots(pub):
    """{slot index: qddot_safe[7]} for commands published in the episode (first per slot)."""
    out = {}
    for row in pub:
        t = row[1]
        if T0 <= t <= T1:
            k = int(np.floor((t + 5e-4) / 0.01))
            out.setdefault(k, row[2:9])
    return out


ref = np.load(REF)['pub']                      # (t, q[7]) from the sequential replay
ref_slots = {}
for row in ref:
    if T0 <= row[0] <= T1:
        ref_slots.setdefault(int(np.floor((row[0] + 5e-4) / 0.01)), row[1:8])


def analyse(prefix):
    S = json.load(open(prefix + '.json'))
    Z = np.load(prefix + '_raw.npz')
    rate = S['rate']
    clk = Z['clock']
    a = np.max(clk[:, 1] - rate * clk[:, 0])
    T = lambda w: a + rate * w  # noqa: E731
    R = {'mode': S['mode'], 'executor': S['executor'], 'rate': rate,
         'process_cpu_pct': S.get('process_cpu_pct_of_core'), 'executor_loop_cpu_pct': S.get('executor_loop_cpu_pct'),
         'threads_pct': S.get('thread_cpu_pct_of_core'), 'rtf': S.get('window', {}).get('rtf')}
    for k in ('js', 'nom', 'dist', 'rebuild', 'qp', 'solve'):
        x = Z[k]
        x = x[(x[:, 0] >= T0) & (x[:, 0] <= T1)]
        R[k] = dict(per_sim_s=len(x) / SPAN, wall_ms=st(x[:, 2]), cpu_ms=st(x[:, 3]),
                    wait_ms=st(x[:, 2] - x[:, 3]), cpu_s=float(x[:, 3].sum()))
    q = Z['qp']
    q = q[(q[:, 0] >= T0) & (q[:, 0] <= T1)]
    gaps = np.diff(q[:, 0])
    k = np.floor((q[:, 0] + 5e-4) / 0.01).astype(int)
    R['ticks_per_sim_s'] = len(q) / SPAN
    R['late_ticks_ge15ms'] = int((gaps >= 0.015).sum())
    R['max_gap_ms'] = float(1e3 * gaps.max())
    R['missed_slots'] = int((k.max() - k.min() + 1) - len(np.unique(k)))
    b = Z['rebuild']
    b = b[(b[:, 0] >= T0) & (b[:, 0] <= T1)]
    # delay of each rebuild START after its due time: due = the 20 ms grid point at or just
    # before the node time at entry (the timer fired because node time reached it)
    due = np.floor((b[:, 5] + 5e-4) / 0.02) * 0.02
    R['rebuild_gap_ge30ms'] = int((np.diff(b[:, 5]) >= 0.030).sum())
    R['rebuild_start_delay_ms'] = st(T(b[:, 1]) - due)
    ti = Z['tick_in']
    ti = ti[(ti[:, 1] >= T0) & (ti[:, 1] <= T1)]
    Tt = T(ti[:, 0])
    lag = Tt - ti[:, 1]
    R['age_ms'] = dict(clock_lag=st(lag), joint_state=st(Tt - ti[:, 2]),
                       qddot_nom=st(Tt - ti[:, 4]), distances_receipt=st(Tt - ti[:, 5]),
                       distances_capture=st(Tt - ti[:, 6]), constraint_snapshot=st(Tt - T(ti[:, 7])),
                       tick_start_after_slot=st(Tt - np.floor((ti[:, 1] + 5e-4) / 0.01) * 0.01))
    pub = Z['pub']
    pe = pub[(pub[:, 1] >= T0) & (pub[:, 1] <= T1)]
    # command latency: publish time vs the joint-state header used by the tick that published it
    # (the latest tick that started before the publish, by wall time)
    ta = Z['tick_in']
    idx = np.searchsorted(ta[:, 0], pe[:, 0], side='right') - 1
    ok = idx >= 0
    R['age_ms']['command_latency'] = st(T(pe[ok, 0]) - ta[idx[ok], 2])
    R['ticks_without_command'] = int(len(ta) - len(np.unique(np.searchsorted(ta[:, 0], pub[:, 0], side='right') - 1)))
    s = slots(pub)
    common = sorted(set(s) & set(ref_slots))
    d = np.array([np.max(np.abs(s[i] - ref_slots[i])) for i in common])
    R['dev_from_reference'] = dict(n_slots=len(common), exact=int((d == 0).sum()),
                                   p50=pct(d, 50), p95=pct(d, 95), p99=pct(d, 99), max=float(d.max()),
                                   frac_gt_0p05=float((d > 0.05).mean()))
    return R, s


res, slotmap = {}, {}
for c in CONDS:
    for f in sorted(os.listdir(AB)):
        if f.startswith(c + '_') and f.endswith('.json'):
            name = f[:-5]
            res[name], slotmap[name] = analyse(os.path.join(AB, name))


def pair_dev(x, y):
    common = sorted(set(slotmap[x]) & set(slotmap[y]))
    d = np.array([np.max(np.abs(slotmap[x][i] - slotmap[y][i])) for i in common])
    return d


cross = {}
for c in CONDS:
    names = [n for n in res if n.startswith(c + '_')]
    groups = {m: [n for n in names if res[n]['mode'] == m] for m in ('multi', 'single')}
    out = {}
    for label, pairs in (('multi_vs_multi', itertools.combinations(groups['multi'], 2)),
                         ('single_vs_single', itertools.combinations(groups['single'], 2)),
                         ('multi_vs_single', itertools.product(groups['multi'], groups['single']))):
        ds = [pair_dev(x, y) for x, y in pairs]
        if ds:
            allv = np.concatenate(ds)
            out[label] = dict(pairs=len(ds), p50=pct(allv, 50), p95=pct(allv, 95), p99=pct(allv, 99),
                              max=float(allv.max()), frac_gt_0p05=float((allv > 0.05).mean()))
    cross[c] = out


def agg(c, mode, path):
    vals = []
    for n, r in res.items():
        if n.startswith(c + '_') and r['mode'] == mode:
            v = r
            for p in path:
                v = v.get(p) if isinstance(v, dict) else None
            vals.append(v)
    vals = [v for v in vals if v is not None]
    return dict(mean=float(np.mean(vals)), min=float(np.min(vals)), max=float(np.max(vals)), n=len(vals)) if vals else None


summary = {}
METRICS = {'ticks_per_sim_s': ('ticks_per_sim_s',), 'late_ticks_ge15ms': ('late_ticks_ge15ms',),
           'missed_slots': ('missed_slots',), 'max_gap_ms': ('max_gap_ms',),
           'process_cpu_pct': ('process_cpu_pct',), 'executor_loop_cpu_pct': ('executor_loop_cpu_pct',),
           'rebuild_wall_p50': ('rebuild', 'wall_ms', 'p50'), 'rebuild_wall_p95': ('rebuild', 'wall_ms', 'p95'),
           'rebuild_cpu_p50': ('rebuild', 'cpu_ms', 'p50'), 'rebuild_wait_p95': ('rebuild', 'wait_ms', 'p95'),
           'rebuild_gap_ge30ms': ('rebuild_gap_ge30ms',),
           'rebuild_start_delay_p95': ('rebuild_start_delay_ms', 'p95'),
           'tick_wall_p50': ('qp', 'wall_ms', 'p50'), 'tick_wall_p95': ('qp', 'wall_ms', 'p95'),
           'tick_cpu_p50': ('qp', 'cpu_ms', 'p50'), 'tick_wait_p95': ('qp', 'wait_ms', 'p95'),
           'solve_wall_p50': ('solve', 'wall_ms', 'p50'), 'solve_cpu_p50': ('solve', 'cpu_ms', 'p50'),
           'js_wall_p95': ('js', 'wall_ms', 'p95'), 'js_per_sim_s': ('js', 'per_sim_s'),
           'dist_wall_p95': ('dist', 'wall_ms', 'p95'),
           'age_clock_lag_p95': ('age_ms', 'clock_lag', 'p95'), 'age_clock_lag_max': ('age_ms', 'clock_lag', 'max'),
           'age_js_p50': ('age_ms', 'joint_state', 'p50'), 'age_js_p95': ('age_ms', 'joint_state', 'p95'),
           'age_js_max': ('age_ms', 'joint_state', 'max'),
           'age_nom_p95': ('age_ms', 'qddot_nom', 'p95'), 'age_dist_capture_p95': ('age_ms', 'distances_capture', 'p95'),
           'age_constraint_p50': ('age_ms', 'constraint_snapshot', 'p50'),
           'age_constraint_p95': ('age_ms', 'constraint_snapshot', 'p95'),
           'age_constraint_max': ('age_ms', 'constraint_snapshot', 'max'),
           'tick_start_after_slot_p95': ('age_ms', 'tick_start_after_slot', 'p95'),
           'command_latency_p50': ('age_ms', 'command_latency', 'p50'),
           'command_latency_p95': ('age_ms', 'command_latency', 'p95'),
           'command_latency_max': ('age_ms', 'command_latency', 'max'),
           'dev_ref_p50': ('dev_from_reference', 'p50'), 'dev_ref_p95': ('dev_from_reference', 'p95'),
           'dev_ref_max': ('dev_from_reference', 'max'), 'dev_ref_exact_slots': ('dev_from_reference', 'exact'),
           'dev_ref_n_slots': ('dev_from_reference', 'n_slots')}
for c in CONDS:
    summary[c] = {m: {k: agg(c, m, p) for k, p in METRICS.items()} for m in ('multi', 'single')}
json.dump(dict(runs=res, cross_run_output_deviation=cross, summary=summary), open(OUT, 'w'), indent=1)
for c in CONDS:
    print(f'===== {c}')
    for k in METRICS:
        a, b = summary[c]['multi'][k], summary[c]['single'][k]
        f = lambda v: '-' if v is None else f"{v['mean']:.3g} [{v['min']:.3g}..{v['max']:.3g}] n={v['n']}"  # noqa: E731
        print(f'  {k:28s} multi {f(a):38s} single {f(b)}')
    for k, v in cross[c].items():
        print(f'  cross {k:18s} ' + ' '.join(f'{a}={b:.3g}' for a, b in v.items()))
