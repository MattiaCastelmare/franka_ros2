#!/usr/bin/env python3
"""M3: executor arm x host load. Per-run host metrics, load gates, per-cell summaries and the
pre-registered hypothesis rules (``refactor_r1_m3/protocol/M3_PROTOCOL.md`` §7–§9).

  python3 -I m3_analyze.py <evidence_dir> <e3_05_dir> <e3_runs_dir> <out.json>

* The replay metrics come from ``m1_analyze.py`` (the M1 analysis of record, unchanged), run once per
  load condition: once on every run of that load (per-run records and replay validity) and once
  on the valid runs only (per-arm statistics).
* This file adds:
  * the host layer (``host/<id>_samples.npz`` from ``m3_host_sampler.py``, cut to the replay's
    analysis window);
  * the load gates;
  * the per-callback CPU budget;
  * the stale-data warnings inside the window;
  * the headroom estimate;
  * the E3 reference values;
  * the hypothesis rules.
* Read-only on its inputs. Importable: the functions are tested by ``test_m3_tools.py`` on
  synthetic data.
"""
import json
import os
import re
import subprocess
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
P_CORES, E_CORES = list(range(0, 16)), list(range(16, 28))   # i7-14700HX, as in E3's s6b_host.py
LOAD_GATE = {'L0': (0.0, 3.0), 'L1': (11.5, 15.0)}          # host busy cores in the window, §7
N_WORKERS = 12
ARMS = ('single', 'mt1', 'mt3')
LOADS = ('L0', 'L1')
R15_HZ = 95.0                                                # E3 gate R15: 100 Hz +-5 %


def busy_cores(stat_a, stat_b):
    """Sum over CPUs of 1 - d(idle + iowait) / d(total), as E3's s6b_host.py computes it."""
    a, b = np.asarray(stat_a, float), np.asarray(stat_b, float)
    dtot = (b - a).sum(1)
    idle = (b[:, 3] + b[:, 4]) - (a[:, 3] + a[:, 4])
    util = 1.0 - idle / np.maximum(dtot, 1.0)
    return float(util.sum()), util


def window_indices(t, w0, w1):
    """Last sample at or before w0 and first sample at or after w1; None if the samples do not cover."""
    t = np.asarray(t, float)
    if not len(t) or t[0] > w0 or t[-1] < w1:
        return None
    i0 = int(np.searchsorted(t, w0, side='right') - 1)
    i1 = int(np.searchsorted(t, w1, side='left'))
    return i0, i1


def pstats(x):
    x = np.asarray(x, float)
    x = x[np.isfinite(x) & (x > 0)]
    if not x.size:
        return None
    return dict(n=int(x.size), p5=float(np.percentile(x, 5)), p50=float(np.percentile(x, 50)),
                p95=float(np.percentile(x, 95)), mean=float(x.mean()))


def host_metrics(samples, w0, w1):
    t = samples['t']
    ij = window_indices(t, w0, w1)
    if ij is None:
        return dict(covered=False)
    i0, i1 = ij
    busy, util = busy_cores(samples['stat'][i0], samples['stat'][i1])
    f = np.asarray(samples['freq_khz'][i0:i1 + 1], float) / 1e3
    k = np.asarray(samples['pkg_temp_mC'], float)
    return dict(covered=True, span_s=float(t[i1] - t[i0]), n_samples=int(i1 - i0 + 1),
                busy_cores=busy, core_util_pct=[round(100 * u, 1) for u in util],
                freq_mhz_p_cores=pstats(f[:, P_CORES].ravel()), freq_mhz_e_cores=pstats(f[:, E_CORES].ravel()),
                pkg_temp_c_window_max=float(k[i0:i1 + 1].max() / 1e3), pkg_temp_c_run_max=float(k.max() / 1e3),
                thermal_aborted=bool(samples.get('thermal_aborted', False)))


def load_gate(load, busy, workers_start, workers_end):
    lo, hi = LOAD_GATE[load]
    ok = {'host_busy_in_range': busy is not None and lo <= busy <= hi}
    if load == 'L1':
        ok['workers_alive_start'] = workers_start == N_WORKERS
        ok['workers_alive_end'] = workers_end == N_WORKERS
    return ok


def separated(a, b):
    """The M1 rule: |mean difference| > 2 x the larger sd, and the ranges do not overlap."""
    a, b = [float(x) for x in a], [float(x) for x in b]
    if len(a) < 2 or len(b) < 2:
        return False
    d = np.mean(b) - np.mean(a)
    sd = max(np.std(a, ddof=1), np.std(b, ddof=1))
    return bool(abs(d) > 2 * sd and (max(a) < min(b) or max(b) < min(a)))


def diff_growth(s0, t0, s1, t1):
    """(t1 - s1) - (t0 - s0) and whether it exceeds 2 x its propagated standard error (§9 H3)."""
    def se(x):
        return np.std(x, ddof=1) / np.sqrt(len(x)) if len(x) > 1 else np.nan
    d0, d1 = np.mean(t0) - np.mean(s0), np.mean(t1) - np.mean(s1)
    e = np.sqrt(se(s0) ** 2 + se(t0) ** 2 + se(s1) ** 2 + se(t1) ** 2)
    g = d1 - d0
    return dict(diff_L0=float(d0), diff_L1=float(d1), growth=float(g), se=float(e),
                grows=bool(np.isfinite(e) and g > 2 * e))


STALE_RE = re.compile(r'^\[(WARN|ERROR)\] \[(\d+\.\d+)\] \[cbf_safety_filter\]: (.*)$')


def stale_events(log_lines, e0, e1):
    """Node WARN/ERROR lines mentioning stale / timeout / starved / brak*, inside [e0, e1] (epoch s)."""
    inside, outside = {}, {}
    for line in log_lines:
        m = STALE_RE.match(line.strip())
        if not m or not re.search(r'stale|timeout|starv|brak', m.group(3), re.I):
            continue
        key = re.sub(r'[-+]?\d+(\.\d+)?', '#', m.group(3))[:80]
        d = inside if e0 <= float(m.group(2)) <= e1 else outside
        d[key] = d.get(key, 0) + 1
    return dict(in_window=inside, outside_window=outside, n_in_window=int(sum(inside.values())))


def python_thread_cpu(threads):
    """Cores used by the threads that run Python callbacks (spin + pool workers), from the replay summary."""
    return sum(v for k, v in (threads or {}).items()
               if k.startswith('executor-spin') or k.startswith('ThreadPoolExecutor')) / 100.0


def headroom(rate, py_cores):
    """Derived (§9 H): if the Python callback threads cannot together exceed one core, the playback
    rate at which they saturate is rate / py_cores. Assumes the demand scales linearly with the rate."""
    return dict(python_thread_cores=py_cores, idle_fraction=1.0 - py_cores,
                projected_saturation_rate=(rate / py_cores) if py_cores > 0 else None)


def cpu_budget(Z, W0, W1, rtf, process_pct, loop_pct):
    """ms of CPU per sim second: callbacks (thread CPU inside the window), executor loop, remainder."""
    span = W1 - W0
    out = {}
    for k in ('rebuild', 'qp', 'solve', 'js', 'nom', 'dist'):
        x = Z[k]
        x = x[(x[:, 5] >= W0) & (x[:, 5] <= W1)]
        out[k] = float(1e3 * x[:, 3].sum() / span)
    cb = out['rebuild'] + out['qp'] + out['js'] + out['nom'] + out['dist']
    proc = process_pct * 10.0 / rtf
    loop = loop_pct * 10.0 / rtf
    out.update(callbacks=cb, executor_loop=loop, process=proc, remainder=proc - cb - loop)
    return out


def e3_reference(e3_runs):
    """E3 H2 runs 05-10 in their own windows [t0+2, t0+30]: rate, per-operation times, CPU, frequency."""
    out = {}
    for d in sorted(os.listdir(e3_runs)):
        a = os.path.join(e3_runs, d, 'analysis')
        if not os.path.exists(os.path.join(a, 'h2_metrics.json')):
            continue
        t0 = float(json.load(open(os.path.join(a, 'h2_metrics.json')))['placement']['t0'])
        z = np.load(os.path.join(a, 's6b_series.npz'))
        tS, S = z['tS'], z['S']
        e = (tS >= t0 + 2) & (tS <= t0 + 30)
        b = S[:, 37]
        nb = np.r_[np.isfinite(b[0]), (b[1:] != b[:-1]) & np.isfinite(b[1:])]
        h = json.load(open(os.path.join(a, 's6b_host.json')))['cpu']
        filt = [v for k, v in h['process_pct_of_one_core'].items() if 'cbf_safety_filter' in k]
        out[d] = dict(executor='single' if 'single' in d else 'multi', ticks_per_sim_s=float(e.sum() / 28.0),
                      build_wall_p50=float(np.median(b[e & nb])), solve_wall_p50=float(np.median(S[e, 13])),
                      snapshot_age_p50_ms=float(1e3 * np.median(S[e, 18])), n_rows_p50=float(np.median(S[e, 0])),
                      filter_cpu_pct=filt[0] if filt else None, host_busy_cores=h['host_busy_cores_equiv'],
                      freq_p_p50=h['freq_mhz_p_cores_0_15']['p50'], freq_e_p50=h['freq_mhz_e_cores_16_27']['p50'])
    return out


def run_m1(ev, e305, ids, out):
    r = subprocess.run([sys.executable, '-I', os.path.join(HERE, 'm1_analyze.py'), os.path.join(ev, 'runs'), e305,
                        out, *ids], capture_output=True, text=True)
    if r.returncode != 0:
        raise RuntimeError(f'm1_analyze failed: {r.stderr[-2000:]}')
    return json.load(open(out)), r.stdout


def host_txt(path):
    kv = {}
    for line in open(path):
        p = line.split()
        if len(p) >= 2:
            kv.setdefault(p[0] if p[0] != 'post' else 'post_' + p[1], p[1:] if p[0] != 'post' else p[2:])
    return kv


def main(ev, e305, e3_runs, out):
    order = [ln.split()[1:4] for ln in open(os.path.join(ev, 'logs/batch.txt')) if ln.startswith('== ')]
    plan = {i: (a, ld) for i, a, ld in order}
    adir = os.path.join(ev, 'analysis')
    os.makedirs(adir, exist_ok=True)
    m1 = {}
    for ld in LOADS:
        ids = [i for i, (a, x) in plan.items() if x == ld and os.path.exists(os.path.join(ev, 'runs', i + '_m1.json'))]
        if ids:
            m1[ld], _ = run_m1(ev, e305, ids, os.path.join(adir, f'm1_{ld}_all.json'))
    T0 = m1[LOADS[0]]['t0'] if LOADS[0] in m1 else m1[LOADS[1]]['t0']
    W0, W1 = T0 + 2.0, T0 + 30.0
    runs = []
    for ld in m1:
        for R in m1[ld]['runs']:
            rid = R['id']
            Z = np.load(os.path.join(ev, 'runs', rid + '_raw.npz'))
            S = json.load(open(os.path.join(ev, 'runs', rid + '.json')))
            clk = Z['clock']
            k = (clk[:, 1] >= W0) & (clk[:, 1] <= W1)
            o = np.argsort(clk[k, 1])
            w0, w1 = np.interp([W0, W1], clk[k, 1][o], clk[k, 0][o])
            smp = dict(np.load(os.path.join(ev, 'host', rid + '_samples.npz')))
            H = host_metrics(smp, w0, w1)
            ep = np.interp([w0, w1], smp['t'], smp['epoch'])
            kv = host_txt(os.path.join(ev, 'host', rid + '.txt'))
            ws = int(kv.get('load_workers_at_replay_start', ['0'])[0]) if ld == 'L1' else None
            we = int(kv.get('load_workers_at_replay_end', ['0'])[0]) if ld == 'L1' else None
            LG = load_gate(ld, H.get('busy_cores'), ws, we)
            st = stale_events(open(os.path.join(ev, 'logs', rid + '.log'), errors='replace').read().splitlines(),
                              ep[0], ep[1])
            rtf = R['sim_time_rate']['effective_rtf']
            B = cpu_budget(Z, W0, W1, rtf, S['process_cpu_pct_of_core'], S['executor_loop_cpu_pct'])
            HR = headroom(0.70, python_thread_cpu(S.get('thread_cpu_pct_of_core')))
            valid = bool(R['valid'] and all(LG.values()) and H.get('covered'))
            runs.append(dict(id=rid, arm=R['arm'], load=ld, valid=valid, replay_valid=R['valid'],
                             replay_validity=R['validity'], load_gate=LG, host=H, stale=st, cpu_budget=B, headroom=HR,
                             window_monotonic=[float(w0), float(w1)], window_epoch=[float(ep[0]), float(ep[1])],
                             replay=R))
    # per-arm statistics on the valid runs of each load (m1_analyze on those ids)
    cells = {}
    for ld in LOADS:
        ids = [r['id'] for r in runs if r['load'] == ld and r['valid']]
        if {r['arm'] for r in runs if r['id'] in ids} >= set(ARMS):
            J, txt = run_m1(ev, e305, ids, os.path.join(adir, f'm1_{ld}_valid.json'))
            cells[ld] = dict(per_arm=J['per_arm'], mt1_position=J['mt1_position'], pair_differences=J['pair_differences'])
    extra = {'busy_cores': ('host', 'busy_cores'), 'freq_p_p50': ('host', 'freq_mhz_p_cores', 'p50'),
             'freq_e_p50': ('host', 'freq_mhz_e_cores', 'p50'), 'temp_max_c': ('host', 'pkg_temp_c_window_max'),
             'python_thread_cores': ('headroom', 'python_thread_cores'),
             'projected_saturation_rate': ('headroom', 'projected_saturation_rate'),
             'stale_in_window': ('stale', 'n_in_window'),
             'budget_callbacks': ('cpu_budget', 'callbacks'), 'budget_loop': ('cpu_budget', 'executor_loop'),
             'budget_remainder': ('cpu_budget', 'remainder'), 'budget_process': ('cpu_budget', 'process'),
             'budget_rebuild': ('cpu_budget', 'rebuild'), 'budget_tick': ('cpu_budget', 'qp')}

    def get(r, path):
        v = r
        for p in path:
            v = v.get(p) if isinstance(v, dict) else None
        return v
    for ld in cells:
        for arm in ARMS:
            rr = [r for r in runs if r['load'] == ld and r['arm'] == arm and r['valid']]
            for m, path in extra.items():
                vals = [float(get(r, path)) for r in rr if get(r, path) is not None]
                cells[ld]['per_arm'][arm][m] = dict(values=vals, mean=float(np.mean(vals)) if vals else None,
                                                     sd=float(np.std(vals, ddof=1)) if len(vals) > 1 else None,
                                                     min=min(vals) if vals else None, max=max(vals) if vals else None)
    res = dict(window_sim=[W0, W1], plan=plan, runs=runs, cells=cells,
               e3_reference=e3_reference(e3_runs), n_valid=sum(r['valid'] for r in runs), n_runs=len(runs))
    if set(cells) == set(LOADS):
        res['hypotheses'] = hypotheses(cells)
    json.dump(res, open(out, 'w'), indent=1)
    return res


def vals(cells, ld, arm, m):
    return cells[ld]['per_arm'][arm][m]['values']


def hypotheses(cells):
    """The pre-registered rules of protocol §9 (computed only when both loads have all three arms)."""
    H = {}
    s1 = vals(cells, 'L1', 'single', 'ticks_per_sim_s')
    H['H1_single_holds_rate_under_load'] = dict(values=s1, rule='mean >= 99.0 and every run >= 95 (R15)',
                                                supported=bool(np.mean(s1) >= 99.0 and min(s1) >= R15_HZ))
    loss = {a: [100.0 - v for v in vals(cells, 'L1', a, 'ticks_per_sim_s')] for a in ARMS}
    H['H2_loss_order_under_load'] = dict(
        mean_loss={a: float(np.mean(v)) for a, v in loss.items()},
        mt3_loses_more_than_mt1=dict(by_mean=bool(np.mean(loss['mt3']) > np.mean(loss['mt1'])),
                                     separated=separated(loss['mt1'], loss['mt3'])),
        mt1_loses_more_than_single=dict(by_mean=bool(np.mean(loss['mt1']) > np.mean(loss['single'])),
                                        separated=separated(loss['single'], loss['mt1'])))
    g = {}
    for m in ('rebuild_wall_p50', 'solve_wall_p50', 'tick_wait_p95', 'process_cpu_pct', 'missed_slots',
              'late_gt15ms', 'snapshot_age_p50_ms', 'rebuild_cpu_p50'):
        g[m] = {f'{b}_vs_single': diff_growth(vals(cells, 'L0', 'single', m), vals(cells, 'L0', b, m),
                                              vals(cells, 'L1', 'single', m), vals(cells, 'L1', b, m))
                for b in ('mt1', 'mt3')}
    H['H3_gap_grows_under_load'] = g
    m3 = vals(cells, 'L1', 'mt3', 'ticks_per_sim_s')
    H['H4_e3_degradation_reproduced'] = dict(values=m3, rule='mt3 under L1: mean < 95 ticks/sim s (R15 fail)',
                                             reproduced=bool(np.mean(m3) < R15_HZ))
    H['H5_load_inflates_cost'] = {
        a: {m: dict(L0=float(np.mean(vals(cells, 'L0', a, m))), L1=float(np.mean(vals(cells, 'L1', a, m))),
                    ratio=float(np.mean(vals(cells, 'L1', a, m)) / np.mean(vals(cells, 'L0', a, m)))
                    if np.mean(vals(cells, 'L0', a, m)) else None,
                    separated=separated(vals(cells, 'L0', a, m), vals(cells, 'L1', a, m)))
            for m in ('rebuild_cpu_p50', 'rebuild_wall_p50', 'solve_cpu_p50', 'solve_wall_p50', 'tick_cpu_p50',
                      'process_cpu_pct', 'executor_loop_cpu_pct', 'ticks_per_sim_s', 'snapshot_age_p50_ms',
                      'js_processed_frac', 'clock_processed_frac')}
        for a in ARMS}
    return H


if __name__ == '__main__':
    r = main(*sys.argv[1:5])
    for x in r['runs']:
        rp = x['replay']
        print(f"{x['id']} {x['arm']:6s} {x['load']} valid={x['valid']} ticks/sim s {rp['sim']['ticks_per_sim_s']:.2f} "
              f"missed {rp['sim']['missed_slots']} rebuild p50 {rp['wall']['rebuild']['wall_ms']['p50']:.2f} "
              f"solve p50 {rp['wall']['solve']['wall_ms']['p50']:.2f} busy {x['host'].get('busy_cores', float('nan')):.2f} "
              f"P-MHz p50 {(x['host'].get('freq_mhz_p_cores') or {}).get('p50', float('nan')):.0f} "
              f"Tmax {x['host'].get('pkg_temp_c_window_max', float('nan')):.0f} stale {x['stale']['n_in_window']}"
              + ('' if x['valid'] else f"  INVALID replay={x['replay_validity']} load={x['load_gate']}"))
    print(f"valid {r['n_valid']}/{r['n_runs']}")
