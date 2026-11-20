#!/usr/bin/env python3
"""R2 performance regression check: the SHIPPED default executor (arm ``default``) at rate 0.70 and 1.0.

Protocol: ``refactor_r2/04_perf/protocol/R2_PERF_PROTOCOL.md``. Approved by the user on 2026-11-20 (R2 STEP 6).

  python3 -I r2_analyze.py <evidence_dir> <e3_05_dir> <out.json>
  python3 -I r2_analyze.py --check <evidence_dir> <e3_05_dir> <run_id>

The second form is the per-run gate: exit 0 valid, 10 invalid; any other exit code (1 = an uncaught
exception) is a harness failure, never an invalid run.

* **Replay metrics:** ``m1_analyze.py`` (unchanged), through ``m3_analyze.run_m1``.
* **Host layer:** ``m3_analyze``'s load gates, stale events, CPU budget and headroom, and ``m3b_analyze``'s
  host checks (no suspend, inhibitor held), all unchanged; headroom at the run's own rate.
* **Rate gate:** the run's own rate ± 10 % (``rtf_in_r2_range``) replaces M1's ``rtf_in_range``. At 0.70 it is
  M1's gate (0.63–0.77); at 1.0 it is M3b's (0.90–1.10). Every other M1 check is kept as recorded.
* **R2 checks (added):** no executor override reached the node; the node resolved ``single`` and built a
  ``SingleThreadedExecutor``; the node's whole parameter namespace equals the sealed M3b reference run's.
* **Acceptance (§8, pre-registered in R2_READINESS_M3B_AND_TEST_TRIAGE.md §6.3 before any R2 run):** per
  condition, every valid run must meet the condition's thresholds (A_L0, A_L1, C_L1) or the cell must get
  M3b's PASS verdict (B_L0). REPRODUCED / NOT_REPRODUCED / INCONCLUSIVE (fewer than 2 valid runs).
* **Comparison (reported, not gated):** M1 single (0.70, idle), M3 single (0.70, L0 / L1), M3b (1.0, L0 / L1).

Read-only on its inputs. Importable: the rules are tested by ``test_r2_tools.py``.
"""
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import m3_analyze as M3  # noqa: E402
import m3b_analyze as B  # noqa: E402

EV_ROOT = os.path.expanduser('~/franka_baseline_evidence')
REF_PARAMS_RUN = os.path.join(EV_ROOT, 'refactor_r1_m3b/data/runs/m3b_01_m1.json')
REF_M1 = os.path.join(EV_ROOT, 'refactor_r1_m1/analysis/m1_analysis.json')
REF_M3 = os.path.join(EV_ROOT, 'refactor_r1_m3/attempt1/analysis/m3_analysis.json')
REF_M3B = os.path.join(EV_ROOT, 'refactor_r1_m3b/data/analysis/m3b_analysis.json')

RTF_REL = 0.10                            # §7: the M1 / M3b relative rate gate
CONDITIONS = {'A_L0': (0.70, 'L0'), 'A_L1': (0.70, 'L1'), 'B_L0': (1.0, 'L0'), 'C_L1': (1.0, 'L1')}
QUANT_TOL = 1e-6                          # §8: gap and age metrics are multiples of the 1 ms /clock step;
#                                           only float noise (~1e-12) is tolerated, never a whole step
A_THRESH = dict(ticks_min=99.0, missed_max=10, gap_p99_max_ms=12.0, snap_age_p95_max_ms=11.0)   # R1_M3 §7.3 item 7
ACCEPT = {
    'A_L0': A_THRESH,
    'A_L1': A_THRESH,
    'B_L0': 'M3B_PASS',                   # m3b_analyze.cell_verdict == PASS (A1–A4 of M3b)
    # M3b L1 one-sided equivalence band: the worst M3b L1 value moved by 2 SD in the unfavourable direction
    # (ticks 86.36 − 2 × 2.94; missed 405 + 2 × 83.2; gaps ≥ 50 ms 1 + 2 × 0.58; gap p99 24 + 2 × 1.0;
    # snapshot age p95 16 + 2 × 0.577 = 17.15). The first two are R2_READINESS §6.3 item 5 verbatim.
    'C_L1': dict(ticks_min=80.4, missed_max=571, gaps_ge50_max=2, gap_p99_max_ms=26.0, snap_age_p95_max_ms=17.15),
}
MIN_VALID = 2
INVALID_RC = 10
COMPARE = ('ticks_per_sim_s', 'ticks_per_wall_s', 'missed_slots', 'late_gt15ms', 'gap_p95_ms', 'gap_p99_ms',
           'gap_max_ms', 'gaps_ge50ms', 'snapshot_age_p50_ms', 'snapshot_age_p95_ms', 'js_processed_frac',
           'js_age_p95_ms', 'rebuild_wall_p50', 'solve_wall_p50', 'tick_wait_p95', 'n_rows_p50',
           'process_cpu_pct', 'python_thread_cores', 'effective_rtf')
get = B.get
PATHS = B.PATHS


def load_json(path):
    with open(path) as f:
        return json.load(f)


def rtf_gate(rate):
    return (round(rate * (1 - RTF_REL), 6), round(rate * (1 + RTF_REL), 6))


def regate(validity, rtf, rate):
    """M1's validity checks with the rate gate centred on the run's rate; every other check unchanged."""
    lo, hi = rtf_gate(rate)
    v = {k: bool(x) for k, x in validity.items() if k != 'rtf_in_range'}
    v['rtf_in_r2_range'] = bool(rtf is not None and np.isfinite(rtf) and lo <= rtf <= hi)
    return v


def r2_checks(R, r2rec, params, ref_params):
    """The default arm really ran the shipped default, and nothing but the executor source changed."""
    args = (r2rec or {}).get('rclpy_init_args') or []
    return {
        'no_executor_override': bool(r2rec) and r2rec.get('removed_pair') == ['-p', 'executor_mode:=single']
        and not any(a.startswith('executor_mode:=') for a in args) and bool(r2rec.get('params_files')),
        'node_resolved_single': R.get('node_executor_mode') == 'single',
        'executor_is_single': R.get('executor_class') == 'SingleThreadedExecutor',
        'params_equal_m3b_reference': params is not None and params == ref_params,
    }


def first_line(ev, rid):
    with open(os.path.join(ev, 'host', rid + '.txt')) as f:
        p = f.readline().split()
    return float(p[p.index('rate') + 1]), p[p.index('load') + 1]


def run_record(ev, R, rate, load, W0, W1, ref_params):
    """``m3b_analyze.run_record`` with the rate as an argument, plus the R2 checks."""
    rid = R['id']
    Z = np.load(os.path.join(ev, 'runs', rid + '_raw.npz'))
    S = load_json((os.path.join(ev, 'runs', rid + '.json')))
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
    HC = B.host_checks(kv)
    with open(os.path.join(ev, 'logs', rid + '.log'), errors='replace') as f:
        st = M3.stale_events(f.read().splitlines(), ep[0], ep[1])
    rtf = R['sim_time_rate']['effective_rtf']
    BU = M3.cpu_budget(Z, W0, W1, rtf, S['process_cpu_pct_of_core'], S['executor_loop_cpu_pct'])
    HR = M3.headroom(rate, M3.python_thread_cpu(S.get('thread_cpu_pct_of_core')))
    RV = regate(R['validity'], rtf, rate)
    p_r2 = os.path.join(ev, 'runs', rid + '_r2.json')
    r2rec = load_json((p_r2)) if os.path.exists(p_r2) else None
    params = load_json((os.path.join(ev, 'runs', rid + '_m1.json'))).get('params')
    RC = r2_checks(R, r2rec, params, ref_params)
    valid = bool(all(RV.values()) and all(LG.values()) and all(HC.values()) and all(RC.values()) and H.get('covered'))
    return dict(id=rid, arm='default', rate=rate, load=load, condition=condition_of(rate, load), valid=valid,
                replay_validity=RV, r2_checks=RC, load_gate=LG, host_checks=HC, host=H, stale=st, cpu_budget=BU,
                headroom=HR, r2_record=r2rec, window_monotonic=[float(w0), float(w1)],
                window_epoch=[float(ep[0]), float(ep[1])], replay=R)


def condition_of(rate, load):
    for c, (r, ld) in CONDITIONS.items():
        if abs(r - rate) < 1e-9 and ld == load:
            return c
    raise ValueError(f'no condition for rate {rate} load {load}')


def run_acceptance(r, cond):
    """Per-run acceptance for A_L0, A_L1 and C_L1 (B_L0 uses m3b_analyze.run_acceptance)."""
    t = ACCEPT[cond]
    g = lambda m: get(r, PATHS[m])  # noqa: E731
    out = {'ticks': g('ticks_per_sim_s') >= t['ticks_min'] - QUANT_TOL,
           'missed': g('missed_slots') <= t['missed_max'],
           'gap_p99': g('gap_p99_ms') <= t['gap_p99_max_ms'] + QUANT_TOL,
           'freshness': g('snapshot_age_p95_ms') <= t['snap_age_p95_max_ms'] + QUANT_TOL}
    if 'gaps_ge50_max' in t:
        out['gaps_ge50'] = g('gaps_ge50ms') <= t['gaps_ge50_max']
    return out


def condition_verdict(cond, valid_runs):
    if len(valid_runs) < MIN_VALID:
        return dict(verdict='INCONCLUSIVE', n_valid=len(valid_runs))
    if ACCEPT[cond] == 'M3B_PASS':
        v = B.cell_verdict(valid_runs)
        return dict(verdict='REPRODUCED' if v['verdict'] == 'PASS' else 'NOT_REPRODUCED', n_valid=len(valid_runs),
                    m3b_verdict=v)
    per = {r.get('id', str(i)): run_acceptance(r, cond) for i, r in enumerate(valid_runs)}
    ok = all(all(a.values()) for a in per.values())
    return dict(verdict='REPRODUCED' if ok else 'NOT_REPRODUCED', n_valid=len(valid_runs), per_run=per)


def _stat(vals):
    v = [float(x) for x in vals if x is not None]
    return dict(values=v, mean=float(np.mean(v)) if v else None, sd=float(np.std(v, ddof=1)) if len(v) > 1 else None,
                min=min(v) if v else None, max=max(v) if v else None)


def references():
    """Sealed single-executor references per condition (means and ranges), for the comparison table."""
    m1 = load_json((REF_M1))['per_arm']['single']
    m3 = load_json((REF_M3))['cells']
    m3b = load_json((REF_M3B))['cells']
    pick = lambda d: {m: dict(mean=d[m]['mean'], min=d[m].get('min'), max=d[m].get('max'))  # noqa: E731
                      for m in COMPARE if isinstance(d.get(m), dict) and 'mean' in d[m]}
    return {'A_L0': {'M1_single_idle': pick(m1), 'M3_single_L0': pick(m3['L0']['per_arm']['single'])},
            'A_L1': {'M3_single_L1': pick(m3['L1']['per_arm']['single'])},
            'B_L0': {'M3b_L0': pick(m3b['L0']['stats'])},
            'C_L1': {'M3b_L1': pick(m3b['L1']['stats'])}}


def analyse(ev, e305, ids_conds, out_prefix):
    ref_params = load_json((REF_PARAMS_RUN))['params']
    recs, W = [], None
    for rate in sorted({CONDITIONS[c][0] for c in ids_conds.values()}):
        ids = [i for i, c in ids_conds.items() if CONDITIONS[c][0] == rate
               and os.path.exists(os.path.join(ev, 'runs', i + '_m1.json'))]
        if not ids:
            continue
        J, _ = M3.run_m1(ev, e305, ids, f'{out_prefix}_m1_rate{rate:.2f}.json')
        W = (J['window_sim'][0], J['window_sim'][1])
        for R in J['runs']:
            recs.append(run_record(ev, R, rate, CONDITIONS[ids_conds[R['id']]][1], *W, ref_params))
    return recs, W


def check(ev, e305, rid):
    rate, load = first_line(ev, rid)
    cond = condition_of(rate, load)
    recs, _ = analyse(ev, e305, {rid: cond}, os.path.join(ev, 'runs', rid + '_check'))
    r = recs[0]
    bad = [k for d in ('replay_validity', 'r2_checks', 'load_gate', 'host_checks') for k, v in r[d].items() if not v]
    if not r['host'].get('covered'):
        bad.append('host_samples_cover_window')
    a = B.run_acceptance(r) if ACCEPT[cond] == 'M3B_PASS' else run_acceptance(r, cond)
    print(f"{rid} {cond} rate {rate} {load} valid={r['valid']} ticks/sim s {get(r, PATHS['ticks_per_sim_s']):.2f} "
          f"missed {get(r, PATHS['missed_slots'])} gap p95/p99 {get(r, PATHS['gap_p95_ms']):.1f}/"
          f"{get(r, PATHS['gap_p99_ms']):.1f} ms snap p95 {get(r, PATHS['snapshot_age_p95_ms']):.1f} "
          f"rtf {get(r, PATHS['effective_rtf']):.3f} cpu {get(r, PATHS['process_cpu_pct'])} "
          f"py {get(r, PATHS['python_thread_cores']):.2f} busy {get(r, PATHS['busy_cores']):.2f} "
          f"executor {r['replay'].get('executor_class')}/{r['replay'].get('node_executor_mode')} "
          f"acc {''.join('1' if a[k] else '0' for k in sorted(a))}" + (f" INVALID {bad}" if bad else ''))
    with open(os.path.join(ev, 'runs', rid + '_r2_check.json'), 'w') as f:
        json.dump(r, f, indent=1, default=float)
    return 0 if r['valid'] else INVALID_RC


def plan_of(ev):
    """``== <id> default <load> <rate> <timestamp>`` lines of the batch log."""
    out = {}
    with open(os.path.join(ev, 'logs/batch.txt')) as f:
        for ln in f:
            if ln.startswith('== '):
                p = ln.split()
                out[p[1]] = condition_of(float(p[4]), p[3])
    return out


def main(ev, e305, out):
    plan = plan_of(ev)
    recs, W = analyse(ev, e305, plan, os.path.join(os.path.dirname(out), 'r2'))
    refs = references()
    conds = {}
    for c in CONDITIONS:
        v = [r for r in recs if r['condition'] == c and r['valid']]
        conds[c] = dict(rate=CONDITIONS[c][0], load=CONDITIONS[c][1], n_runs=sum(r['condition'] == c for r in recs),
                        n_valid=len(v), acceptance=condition_verdict(c, v),
                        stats={m: _stat(get(r, PATHS[m]) for r in v) for m in COMPARE}, references=refs[c])
    overall = ('REPRODUCED' if all(conds[c]['acceptance']['verdict'] == 'REPRODUCED' for c in CONDITIONS) else
               'INCONCLUSIVE' if any(conds[c]['acceptance']['verdict'] == 'INCONCLUSIVE' for c in CONDITIONS)
               and not any(conds[c]['acceptance']['verdict'] == 'NOT_REPRODUCED' for c in CONDITIONS)
               else 'NOT_REPRODUCED')
    res = dict(window_sim=list(W) if W else None, plan=plan, accept=ACCEPT, conditions=conds, overall=overall,
               runs=recs, n_valid=sum(r['valid'] for r in recs), n_runs=len(recs))
    with open(out, 'w') as f:
        json.dump(res, f, indent=1, default=float)
    return res


if __name__ == '__main__':
    if sys.argv[1] == '--check':
        sys.exit(check(*sys.argv[2:5]))
    r = main(*sys.argv[1:4])
    for c in CONDITIONS:
        a = r['conditions'][c]['acceptance']
        print(c, a['verdict'], f"valid {r['conditions'][c]['n_valid']}/{r['conditions'][c]['n_runs']}")
    print('overall', r['overall'], f"valid {r['n_valid']}/{r['n_runs']}")
