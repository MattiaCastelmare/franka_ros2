#!/usr/bin/env python3
"""M3b: SingleThreadedExecutor at playback rate 1.0, idle host (L0) vs E3-like host load (L1).

Protocol: ``refactor_r1_m3b/protocol/M3B_PROTOCOL.md``.

  python3 -I m3b_analyze.py <evidence_dir> <e3_05_dir> <m3_analysis.json> <out.json>
  python3 -I m3b_analyze.py --check <evidence_dir> <e3_05_dir> <run_id>

The second form is the per-run gate. It exits 0 if the run is valid and 10 if it is invalid. Any
other exit code (1 = an uncaught exception) is a harness failure, never an invalid run.

* **Replay metrics** come from ``m1_analyze.py`` (unchanged), run through ``m3_analyze.run_m1``.
* **Host layer:** the load gates, stale events, CPU budget and headroom are ``m3_analyze``'s
  functions (unchanged), applied at M3b's rate.
* **What M3b changes is the validity rule only (§7).**
  * The M1 rate gate ``rtf_in_range`` (0.63–0.77, i.e. 0.70 ± 10 %) is replaced by
    ``rtf_in_m3b_range`` (1.0 ± 10 %).
  * Every other M1 check is kept as recorded.
  * Two host checks are added:
    * no suspend during the run (``boot_minus_mono`` unchanged);
    * the sleep inhibitor is held.
* **Added on top:** the pre-registered acceptance (§8) and the comparison with M3's single arm at
  rate 0.70 (from M3's sealed ``m3_analysis.json``).
* Read-only on its inputs. Importable: the rules are tested by ``test_m3b_tools.py``.
"""
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import m3_analyze as M3  # noqa: E402

RATE = 1.0
RTF_GATE = (0.90, 1.10)                  # §7: M1's relative width (±10 %) around the M3b rate
SUSPEND_TOL_S = 1.0                      # §7: CLOCK_BOOTTIME − CLOCK_MONOTONIC may not grow more than this
LOADS = ('L0', 'L1')
ACCEPT = dict(ticks_min_run=95.0,        # A1: every valid run ≥ 95 QP ticks per sim s (E3 gate R15) ...
              ticks_mean=99.0,           # ... and the cell mean ≥ 99
              missed_max=28,             # A2: ≤ 28 missed 10 ms slots (1 % of the 2800) ...
              gaps_ge50_max=0,           # ... and no tick gap ≥ 50 ms
              gap_p99_max_ms=15.0,       # A3: tick-gap p99 ≤ 15 ms
              snap_age_p95_max_ms=20.0)  # A4: snapshot age p95 ≤ one rebuild period (20 ms)
MARGIN_MIN = 1.10                        # A5 (margin, Derived): projected saturation rate ≥ 1.10 × real time
SAT_PY_CORES = 0.95                      # §9 saturation indicator: Python callback threads ≥ 0.95 core
MIN_VALID = 2                            # §8: a cell verdict needs at least two valid runs
INVALID_RC = 10                          # --check exit code of an invalid run (1 is an exception)

PATHS = {
    'ticks_per_sim_s': ('replay', 'sim', 'ticks_per_sim_s'), 'ticks_per_wall_s': ('replay', 'wall', 'ticks_per_wall_s'),
    'missed_slots': ('replay', 'sim', 'missed_slots'), 'late_gt15ms': ('replay', 'sim', 'late_gt15ms'),
    'gap_p50_ms': ('replay', 'sim', 'tick_gap_ms', 'p50'), 'gap_p95_ms': ('replay', 'sim', 'tick_gap_ms', 'p95'),
    'gap_p99_ms': ('replay', 'sim', 'tick_gap_ms', 'p99'), 'gap_max_ms': ('replay', 'sim', 'tick_gap_ms', 'max'),
    'gaps_ge50ms': ('replay', 'sim', 'gaps_ge50ms'),
    'rebuild_per_sim_s': ('replay', 'wall', 'rebuild', 'per_sim_s'),
    'rebuild_wall_p50': ('replay', 'wall', 'rebuild', 'wall_ms', 'p50'),
    'rebuild_wall_p95': ('replay', 'wall', 'rebuild', 'wall_ms', 'p95'),
    'rebuild_cpu_p50': ('replay', 'wall', 'rebuild', 'cpu_ms', 'p50'),
    'solve_wall_p50': ('replay', 'wall', 'solve', 'wall_ms', 'p50'),
    'solve_wall_p95': ('replay', 'wall', 'solve', 'wall_ms', 'p95'),
    'solve_cpu_p50': ('replay', 'wall', 'solve', 'cpu_ms', 'p50'),
    'tick_wall_p50': ('replay', 'wall', 'tick_wall_ms', 'p50'), 'tick_cpu_p50': ('replay', 'wall', 'tick_cpu_ms', 'p50'),
    'tick_wait_p95': ('replay', 'wall', 'tick_wait_ms', 'p95'),
    'snapshot_age_p50_ms': ('replay', 'status', 'snapshot_age_ms', 'p50'),
    'snapshot_age_p95_ms': ('replay', 'status', 'snapshot_age_ms', 'p95'),
    'n_rows_p50': ('replay', 'status', 'n_rows', 'p50'),
    'rows_p50_diff_e3_05': ('replay', 'workload_vs_e3_05', 'rows_p50_diff'),
    'rows_matched_within_2': ('replay', 'workload_vs_e3_05', 'matched_within_2'),
    'js_processed_frac': ('replay', 'inputs', 'joint_states', 'processed_frac'),
    'clock_processed_frac': ('replay', 'inputs', 'clock', 'processed_frac'),
    'js_age_p95_ms': ('replay', 'age_ms', 'joint_state_true', 'p95'),
    'effective_rtf': ('replay', 'sim_time_rate', 'effective_rtf'),
    'process_cpu_pct': ('replay', 'cpu', 'process_pct_of_core'),
    'executor_loop_cpu_pct': ('replay', 'cpu', 'executor_loop_pct'),
    'python_thread_cores': ('headroom', 'python_thread_cores'),
    'projected_saturation_rate': ('headroom', 'projected_saturation_rate'),
    'busy_cores': ('host', 'busy_cores'), 'freq_p_p50': ('host', 'freq_mhz_p_cores', 'p50'),
    'freq_e_p50': ('host', 'freq_mhz_e_cores', 'p50'), 'temp_max_c': ('host', 'pkg_temp_c_window_max'),
    'stale_in_window': ('stale', 'n_in_window'),
    'budget_process': ('cpu_budget', 'process'), 'budget_callbacks': ('cpu_budget', 'callbacks'),
    'budget_loop': ('cpu_budget', 'executor_loop'), 'budget_remainder': ('cpu_budget', 'remainder'),
}


def get(d, path):
    for p in path:
        d = d.get(p) if isinstance(d, dict) else None
    return d


def regate(validity, rtf):
    """M1's validity checks with the rate gate re-centred on M3b's rate; every other check unchanged."""
    v = {k: bool(x) for k, x in validity.items() if k != 'rtf_in_range'}
    v['rtf_in_m3b_range'] = bool(rtf is not None and np.isfinite(rtf) and RTF_GATE[0] <= rtf <= RTF_GATE[1])
    return v


def host_checks(kv):
    """No suspend during the run, and the sleep inhibitor held at the start and at the end."""
    try:
        d = abs(float(kv['post_boot_minus_mono'][0]) - float(kv['boot_minus_mono'][0]))
    except (KeyError, IndexError, ValueError):
        d = None
    return {'no_suspend': d is not None and d <= SUSPEND_TOL_S,
            'inhibitor_held': kv.get('inhibitor_held', ['0'])[0] == '1'
            and kv.get('post_inhibitor_held', ['0'])[0] == '1'}


def run_record(ev, R, load, W0, W1):
    rid = R['id']
    Z = np.load(os.path.join(ev, 'runs', rid + '_raw.npz'))
    S = json.load(open(os.path.join(ev, 'runs', rid + '.json')))
    clk = Z['clock']
    k = (clk[:, 1] >= W0) & (clk[:, 1] <= W1)
    o = np.argsort(clk[k, 1])
    w0, w1 = np.interp([W0, W1], clk[k, 1][o], clk[k, 0][o])
    smp = dict(np.load(os.path.join(ev, 'host', rid + '_samples.npz')))
    H = M3.host_metrics(smp, w0, w1)
    ep = np.interp([w0, w1], smp['t'], smp['epoch'])
    kv = M3.host_txt(os.path.join(ev, 'host', rid + '.txt'))
    ws = int(kv.get('load_workers_at_replay_start', ['0'])[0]) if load == 'L1' else None
    we = int(kv.get('load_workers_at_replay_end', ['0'])[0]) if load == 'L1' else None
    LG = M3.load_gate(load, H.get('busy_cores'), ws, we)
    HC = host_checks(kv)
    st = M3.stale_events(open(os.path.join(ev, 'logs', rid + '.log'), errors='replace').read().splitlines(),
                         ep[0], ep[1])
    rtf = R['sim_time_rate']['effective_rtf']
    B = M3.cpu_budget(Z, W0, W1, rtf, S['process_cpu_pct_of_core'], S['executor_loop_cpu_pct'])
    HR = M3.headroom(RATE, M3.python_thread_cpu(S.get('thread_cpu_pct_of_core')))
    RV = regate(R['validity'], rtf)
    RV['executor_is_single'] = R.get('executor_class') == 'SingleThreadedExecutor'
    valid = bool(all(RV.values()) and all(LG.values()) and all(HC.values()) and H.get('covered'))
    return dict(id=rid, arm=R['arm'], load=load, valid=valid, replay_validity=RV, load_gate=LG, host_checks=HC,
                host=H, stale=st, cpu_budget=B, headroom=HR, window_monotonic=[float(w0), float(w1)],
                window_epoch=[float(ep[0]), float(ep[1])], replay=R)


def cell_stats(runs):
    out = {}
    for m, path in PATHS.items():
        v = [float(get(r, path)) for r in runs if get(r, path) is not None]
        out[m] = dict(values=v, mean=float(np.mean(v)) if v else None,
                      sd=float(np.std(v, ddof=1)) if len(v) > 1 else None,
                      min=min(v) if v else None, max=max(v) if v else None)
    return out


def run_acceptance(r):
    g = lambda m: get(r, PATHS[m])  # noqa: E731
    return {'A1_rate': g('ticks_per_sim_s') >= ACCEPT['ticks_min_run'],
            'A2_deadlines': g('missed_slots') <= ACCEPT['missed_max'] and g('gaps_ge50ms') <= ACCEPT['gaps_ge50_max'],
            'A3_jitter': g('gap_p99_ms') <= ACCEPT['gap_p99_max_ms'],
            'A4_freshness': g('snapshot_age_p95_ms') <= ACCEPT['snap_age_p95_max_ms'],
            'A5_margin': (g('projected_saturation_rate') or 0.0) >= MARGIN_MIN,
            'saturated': (g('python_thread_cores') or 0.0) >= SAT_PY_CORES}


def cell_verdict(valid_runs):
    """§8. PASS: A1 (every run ≥ 95 and mean ≥ 99) and A2–A4 in every run. MARGINAL: every run ≥ 95 but
    PASS fails. FAIL: a valid run < 95. INCONCLUSIVE: fewer than MIN_VALID valid runs. A5 is reported
    as 'with margin' or 'without margin' next to the verdict; it does not change it."""
    if len(valid_runs) < MIN_VALID:
        return dict(verdict='INCONCLUSIVE', n_valid=len(valid_runs))
    acc = [run_acceptance(r) for r in valid_runs]
    ticks = [get(r, PATHS['ticks_per_sim_s']) for r in valid_runs]
    every95 = all(a['A1_rate'] for a in acc)
    mean_ok = float(np.mean(ticks)) >= ACCEPT['ticks_mean']
    rest = all(a['A2_deadlines'] and a['A3_jitter'] and a['A4_freshness'] for a in acc)
    v = 'FAIL' if not every95 else ('PASS' if (mean_ok and rest) else 'MARGINAL')
    return dict(verdict=v, n_valid=len(valid_runs), ticks_mean=float(np.mean(ticks)), every_run_ge_95=every95,
                mean_ge_99=mean_ok, A2_A4_every_run=rest, margin=all(a['A5_margin'] for a in acc),
                saturation_evidence=any(a['saturated'] for a in acc), per_run=acc)


def m3_single_reference(m3_json):
    """M3's single arm at rate 0.70, per load, from M3's sealed analysis (means and ranges)."""
    J = json.load(open(m3_json))
    out = {}
    for ld in LOADS:
        pa = J['cells'][ld]['per_arm']['single']
        out[ld] = {m: dict(mean=pa[m]['mean'], min=pa[m]['min'], max=pa[m]['max']) for m in pa
                   if isinstance(pa[m], dict) and 'mean' in pa[m]}
    return out


def plan_of(ev):
    return {ln.split()[1]: ln.split()[3] for ln in open(os.path.join(ev, 'logs/batch.txt')) if ln.startswith('== ')}


def analyse(ev, e305, ids_loads, out_prefix):
    recs = []
    W = None
    for ld in LOADS:
        ids = [i for i, x in ids_loads.items() if x == ld and os.path.exists(os.path.join(ev, 'runs', i + '_m1.json'))]
        if not ids:
            continue
        J, _ = M3.run_m1(ev, e305, ids, f'{out_prefix}_m1_{ld}.json')
        W = (J['window_sim'][0], J['window_sim'][1])
        for R in J['runs']:
            recs.append(run_record(ev, R, ld, *W))
    return recs, W


def check(ev, e305, rid):
    """Per-run gate (§7): the load is read from the run's own host file (``run <id> arm <a> load <L> …``)."""
    first = open(os.path.join(ev, 'host', rid + '.txt')).readline().split()
    load = first[first.index('load') + 1]
    recs, _ = analyse(ev, e305, {rid: load}, os.path.join(ev, 'runs', rid + '_check'))
    r = recs[0]
    bad = [k for d in ('replay_validity', 'load_gate', 'host_checks') for k, v in r[d].items() if not v]
    if not r['host'].get('covered'):
        bad.append('host_samples_cover_window')
    a = run_acceptance(r)
    print(f"{rid} {r['arm']} {load} valid={r['valid']} ticks/sim s {get(r, PATHS['ticks_per_sim_s']):.2f} "
          f"wall/s {get(r, PATHS['ticks_per_wall_s']):.2f} missed {get(r, PATHS['missed_slots'])} "
          f"gap p95/p99 {get(r, PATHS['gap_p95_ms']):.1f}/{get(r, PATHS['gap_p99_ms']):.1f} ms "
          f"rebuild p50 {get(r, PATHS['rebuild_wall_p50']):.2f} solve p50 {get(r, PATHS['solve_wall_p50']):.2f} "
          f"snap p95 {get(r, PATHS['snapshot_age_p95_ms']):.1f} rtf {get(r, PATHS['effective_rtf']):.3f} "
          f"cpu {get(r, PATHS['process_cpu_pct'])} py {get(r, PATHS['python_thread_cores']):.2f} "
          f"busy {get(r, PATHS['busy_cores']):.2f} acc {''.join('1' if a[k] else '0' for k in sorted(a))}"
          + (f" INVALID {bad}" if bad else ''))
    json.dump(r, open(os.path.join(ev, 'runs', rid + '_m3b_check.json'), 'w'), indent=1, default=float)
    return 0 if r['valid'] else INVALID_RC


def main(ev, e305, m3_json, out):
    adir = os.path.join(ev, 'analysis')
    os.makedirs(adir, exist_ok=True)
    plan = plan_of(ev)
    recs, W = analyse(ev, e305, plan, os.path.join(adir, 'm3b'))
    cells = {}
    for ld in LOADS:
        v = [r for r in recs if r['load'] == ld and r['valid']]
        cells[ld] = dict(stats=cell_stats(v), acceptance=cell_verdict(v),
                         n_runs=sum(r['load'] == ld for r in recs), n_valid=len(v))
    res = dict(rate=RATE, window_sim=list(W) if W else None, plan=plan, accept=ACCEPT, margin_min=MARGIN_MIN,
               rtf_gate=RTF_GATE, runs=recs, cells=cells, m3_single_rate_0_70=m3_single_reference(m3_json),
               n_valid=sum(r['valid'] for r in recs), n_runs=len(recs))
    json.dump(res, open(out, 'w'), indent=1, default=float)
    return res


if __name__ == '__main__':
    if sys.argv[1] == '--check':
        sys.exit(check(*sys.argv[2:5]))
    r = main(*sys.argv[1:5])
    for ld in LOADS:
        a = r['cells'][ld]['acceptance']
        print(ld, a['verdict'], {k: a.get(k) for k in ('n_valid', 'ticks_mean', 'margin', 'saturation_evidence')})
    print(f"valid {r['n_valid']}/{r['n_runs']}")
