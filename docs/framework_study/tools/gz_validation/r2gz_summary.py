#!/usr/bin/env python3
"""R2 Gazebo static validation (C-078): campaign summary and the pre-registered answers Q1-Q6.
Host, read-only on the runs; writes one JSON and one text file (refuses to overwrite).

Usage:
  python3 -I r2gz_summary.py <new_out_dir> --pair <seed> <run_dir_default> <run_dir_multi> <compare_json> <pair_check_json>
                                          [--pair ...] [--ref <seed> <e3_single_run_dir> <e3_multi_run_dir>]...

A missing run or file is reported as missing, never as a pass. Thresholds are the constants below,
fixed in protocol/R2GZ_PROTOCOL.md §7 before any run; they are not arguments.
"""
import json
import os
import sys

sys.dont_write_bytecode = True

# ---- pre-registered constants (protocol §7) ---------------------------------------------------
D_SAFE = 0.10                 # m, fr3_control.yaml d_safe (the gate's R14 checks the filter's start-up value)
CLEARANCE_TOL_M = 0.005       # Q1: D's reconstructed min clearance >= M's - 5 mm
QP_MIN_TICKS = 95.0           # Q2: D's QP ticks per sim s in the window after t0+2 s
SNAP_P95_MAX_MS = 20.0        # Q3: D's constraint snapshot age p95 (window) <= one 50 Hz rebuild period
OSC_REL = 1.15                # Q4: D's reversals/command, command-step p99, jerk p99 <= 1.15 x M's
TRACK_TOL_M = 0.010           # Q4: D's pentagon tracking p95 <= M's + 10 mm
SAFETY_RULES = ('R08', 'R10', 'R11', 'R12', 'R14')


def load(path):
    try:
        with open(path) as f:
            return json.load(f)
    except (OSError, ValueError, TypeError):
        return None


def read_text(path):
    try:
        with open(path) as f:
            return f.read().strip()
    except (OSError, TypeError):
        return None


def run_record(d):
    gate = load(os.path.join(d, 'analysis', 'gate.json')) if d else None
    ex = load(os.path.join(d, 'analysis', 'r2gz_extra.json')) if d else None
    st = read_text(os.path.join(d, 'ANALYSIS_STATUS')) if d else None
    rs = read_text(os.path.join(d, 'RUN_STATUS')) if d else None
    rules = {r['id']: r['verdict'] for r in (gate or {}).get('rules', [])}
    return dict(dir=d, run_status=rs, analysis_status=st, gate=(gate or {}).get('overall'), rules=rules,
                gate_mode=(gate or {}).get('mode'), extra=ex,
                valid=(st == 'GATE_CONTINUE'),
                safety_rules={k: rules.get(k) for k in SAFETY_RULES})


def m(r, *path):
    x = r.get('extra')
    for k in path:
        if not isinstance(x, dict) or k not in x:
            return None
        x = x[k]
    return x


def metrics(r):
    return dict(
        qp_ticks_per_sim_s=m(r, 'summary', 'qp_ticks_per_sim_s_window'),
        missed_slots_window=m(r, 'summary', 'missed_slots_window'),
        tick_gap_p95_ms=m(r, 'summary', 'tick_gap_ms_window', 'p95'),
        tick_gap_p99_ms=m(r, 'summary', 'tick_gap_ms_window', 'p99'),
        tick_gap_max_ms=m(r, 'summary', 'tick_gap_ms_window', 'max'),
        gaps_ge_50ms=m(r, 'summary', 'gaps_ge_50ms_window'),
        snapshot_age_p50_ms=m(r, 'summary', 'snapshot_age_ms_window', 'p50'),
        snapshot_age_p95_ms=m(r, 'summary', 'snapshot_age_ms_window', 'p95'),
        snapshot_age_max_ms=m(r, 'summary', 'snapshot_age_ms_window', 'max'),
        recon_min_gap_m=m(r, 'summary', 'recon_min_gap_m'),
        continuous_lower_bound_m=m(r, 'summary', 'continuous_lower_bound_m'),
        truth_min_gap_m=m(r, 'summary', 'truth_min_gap_m'),
        reversals_per_command=m(r, 'oscillation', 'safe', 'reversals_per_command'),
        reversals_per_command_nominal=m(r, 'oscillation', 'nominal', 'reversals_per_command'),
        hf_power_frac=m(r, 'oscillation', 'safe', 'hf_power_frac'),
        step_norm_p99=m(r, 'oscillation', 'safe', 'step_norm', 'p99'),
        jerk_p99=m(r, 'summary', 'jerk_p99'),
        smoothness_overall=(m(r, 'summary', 'smoothness_verdicts') or {}).get('overall'),
        pentagon_p95_m=m(r, 'summary', 'pentagon', 'p95_m'),
        tracking_err_p95_m=m(r, 'summary', 'tracking_err_m_window', 'p95'),
        rtf=m(r, 'summary', 'rtf_harness'),
        filter_cpu_pct=m(r, 'summary', 'filter_cpu_pct_one_core_window'),
        host_busy_cores=m(r, 'summary', 'host_busy_cores_equiv'),
        build_wall_p50_ms=m(r, 'summary', 'build_wall_ms_window', 'p50'),
        solve_wall_p50_ms=m(r, 'summary', 'solve_wall_ms_window', 'p50'),
        rebuilds_per_sim_s=m(r, 'summary', 'rebuilds_per_sim_s_window'),
        fallback_levels=m(r, 'summary', 'fallback_levels'), fault_ticks=m(r, 'summary', 'fault_ticks'),
        reflex=m(r, 'summary', 'reflex_verdict'), slack_max_any=m(r, 'summary', 'slack_max_any'),
        perception_capture_age_p95_ms=m(r, 'summary', 'perception_capture_age_ms_p95'),
        rows_latency_p95_ms=m(r, 'summary', 'rows_latency_ms_p95'),
    )


def le(a, b):
    return None if a is None or b is None else a <= b


def all_true(xs):
    xs = list(xs)
    if any(x is None for x in xs) or not xs:
        return None
    return all(xs)


args = sys.argv[1:]
if not args:
    sys.exit(__doc__)
OUT, args = args[0], args[1:]
if os.path.exists(OUT):
    sys.exit(f'r2gz_summary: {OUT} exists')
pairs, refs = [], []
while args:
    if args[0] == '--pair' and len(args) >= 6:
        pairs.append(dict(seed=int(args[1]), D=args[2], M=args[3], compare=args[4], pair_check=args[5]))
        args = args[6:]
    elif args[0] == '--ref' and len(args) >= 4:
        refs.append(dict(seed=int(args[1]), single=args[2], multi=args[3]))
        args = args[4:]
    else:
        sys.exit(f'r2gz_summary: bad arguments near {args[:3]}')

P = []
for p in pairs:
    D, M = run_record(p['D']), run_record(p['M'])
    cmp_ = load(p['compare'])
    pc = load(p['pair_check'])
    md, mm = metrics(D), metrics(M)
    pair_valid = (D['valid'] and M['valid'] and (cmp_ or {}).get('verdict') == 'COMPARABLE'
                  and (pc or {}).get('verdict') == 'PASS' and D['gate_mode'] == 'single' and M['gate_mode'] == 'multi')
    crit = dict(
        Q1_rules=all_true(D['safety_rules'][k] == 'PASS' for k in ('R12', 'R14')) if D['gate'] else None,
        Q1_vs_M=le((mm['recon_min_gap_m'] or 0) - CLEARANCE_TOL_M, md['recon_min_gap_m'])
        if mm['recon_min_gap_m'] is not None else None,
        Q1_lb_ge_dsafe=le(D_SAFE, md['continuous_lower_bound_m']),
        Q2_ticks=le(QP_MIN_TICKS, md['qp_ticks_per_sim_s']),
        Q2_gt_M=(None if md['qp_ticks_per_sim_s'] is None or mm['qp_ticks_per_sim_s'] is None
                 else md['qp_ticks_per_sim_s'] > mm['qp_ticks_per_sim_s']),
        Q2_no_50ms=None if md['gaps_ge_50ms'] is None else md['gaps_ge_50ms'] == 0,
        Q3_p95=le(md['snapshot_age_p95_ms'], SNAP_P95_MAX_MS),
        Q3_lt_M=(None if md['snapshot_age_p95_ms'] is None or mm['snapshot_age_p95_ms'] is None
                 else md['snapshot_age_p95_ms'] < mm['snapshot_age_p95_ms']),
        Q4_smooth=None if md['smoothness_overall'] is None else md['smoothness_overall'] == 'PASS',
        Q4_reversals=le(md['reversals_per_command'], None if mm['reversals_per_command'] is None
                        else OSC_REL * mm['reversals_per_command']),
        Q4_step=le(md['step_norm_p99'], None if mm['step_norm_p99'] is None else OSC_REL * mm['step_norm_p99']),
        Q4_jerk=le(md['jerk_p99'], None if mm['jerk_p99'] is None else OSC_REL * mm['jerk_p99']),
        Q4_tracking=le(md['pentagon_p95_m'], None if mm['pentagon_p95_m'] is None else mm['pentagon_p95_m'] + TRACK_TOL_M),
        Q5=all_true(v == 'PASS' for v in D['safety_rules'].values()) if D['gate'] else None,
    )
    P.append(dict(seed=p['seed'], pair_valid=pair_valid, compare=(cmp_ or {}).get('verdict'),
                  pair_check=(pc or {}).get('verdict'), D=dict(D, extra=None, metrics=md),
                  M=dict(M, extra=None, metrics=mm), criteria=crit,
                  diff_D_minus_M={k: (md[k] - mm[k]) if isinstance(md[k], (int, float)) and isinstance(mm[k], (int, float))
                                  and not isinstance(md[k], bool) else None for k in md}))


def answer(keys):
    vals = [p['criteria'][k] for p in P for k in keys]
    if not P or any(v is None for v in vals):
        return 'UNDETERMINED'
    return 'YES' if all(vals) else 'NO'


ANS = dict(
    Q1_clearance_preserved=answer(['Q1_rules', 'Q1_vs_M', 'Q1_lb_ge_dsafe']),
    Q2_qp_timing_improved=answer(['Q2_ticks', 'Q2_gt_M', 'Q2_no_50ms']),
    Q3_staleness_reduced=answer(['Q3_p95', 'Q3_lt_M']),
    Q4_no_oscillation_or_tracking_degradation=answer(['Q4_smooth', 'Q4_reversals', 'Q4_step', 'Q4_jerk', 'Q4_tracking']),
    Q5_safety_constraints_respected=answer(['Q5']),
)
complete = len(P) == 2 and all(p['pair_valid'] for p in P)
if ANS['Q1_clearance_preserved'] == 'NO' or ANS['Q5_safety_constraints_respected'] == 'NO':
    q6 = 'DOES_NOT_SUPPORT'
elif complete and all(v == 'YES' for v in ANS.values()):
    q6 = 'SUPPORTS'
else:
    q6 = 'INCONCLUSIVE'
ANS['Q6_supports_moving_obstacle_phase'] = q6

R = []
for r in refs:
    s, mu = run_record(r['single']), run_record(r['multi'])
    R.append(dict(seed=r['seed'], e3_single=dict(dir=r['single'], analysis_status=s['analysis_status'], metrics=metrics(s)),
                  e3_multi=dict(dir=r['multi'], analysis_status=mu['analysis_status'], metrics=metrics(mu))))

doc = dict(answers=ANS, campaign_complete=complete, pairs=P, e3_references=R,
           constants=dict(D_SAFE=D_SAFE, CLEARANCE_TOL_M=CLEARANCE_TOL_M, QP_MIN_TICKS=QP_MIN_TICKS,
                          SNAP_P95_MAX_MS=SNAP_P95_MAX_MS, OSC_REL=OSC_REL, TRACK_TOL_M=TRACK_TOL_M),
           note='Preliminary evidence (2 pairs, one static scenario). Not a proof of general safety.')
os.makedirs(OUT)
with open(os.path.join(OUT, 'r2gz_summary.json'), 'x') as f:
    json.dump(doc, f, indent=1, default=str)


def fx(v, nd=3):
    if v is None:
        return 'n/a'
    if isinstance(v, bool):
        return str(v)
    if isinstance(v, (int, float)):
        return f'{v:.{nd}f}'
    return str(v)


L = ['R2 GAZEBO STATIC VALIDATION: SUMMARY (preliminary evidence)', '']
L += [f'  {k:45s} {v}' for k, v in ANS.items()]
L += ['', f'  campaign complete (2 valid, comparable pairs): {complete}', '']
rows = ['qp_ticks_per_sim_s', 'missed_slots_window', 'tick_gap_p95_ms', 'tick_gap_p99_ms', 'tick_gap_max_ms',
        'gaps_ge_50ms', 'snapshot_age_p50_ms', 'snapshot_age_p95_ms', 'snapshot_age_max_ms', 'recon_min_gap_m',
        'continuous_lower_bound_m', 'truth_min_gap_m', 'reversals_per_command', 'reversals_per_command_nominal',
        'hf_power_frac', 'step_norm_p99', 'jerk_p99', 'smoothness_overall', 'pentagon_p95_m', 'tracking_err_p95_m',
        'rtf', 'filter_cpu_pct', 'host_busy_cores', 'build_wall_p50_ms', 'solve_wall_p50_ms', 'rebuilds_per_sim_s',
        'perception_capture_age_p95_ms', 'rows_latency_p95_ms', 'reflex', 'fault_ticks', 'fallback_levels']
for p in P:
    L.append(f"seed {p['seed']}: pair valid {p['pair_valid']} (compare {p['compare']}, pair check {p['pair_check']}); "
             f"D {os.path.basename(str(p['D']['dir']))} {p['D']['analysis_status']}  M {os.path.basename(str(p['M']['dir']))} "
             f"{p['M']['analysis_status']}")
    for k in rows:
        L.append(f"    {k:32s} D {fx(p['D']['metrics'][k]):>12s}   M {fx(p['M']['metrics'][k]):>12s}   "
                 f"D-M {fx(p['diff_D_minus_M'][k]):>10s}")
    L.append('    criteria: ' + ', '.join(f'{k}={v}' for k, v in p['criteria'].items()))
    L.append('')
for r in R:
    L.append(f"E3 reference seed {r['seed']} (explicit --set, R1 code): single {r['e3_single']['metrics']['qp_ticks_per_sim_s']} "
             f"ticks/sim s, multi {r['e3_multi']['metrics']['qp_ticks_per_sim_s']}")
L += ['', doc['note']]
with open(os.path.join(OUT, 'r2gz_summary.txt'), 'x') as f:
    f.write('\n'.join(L) + '\n')
print('\n'.join(L))
