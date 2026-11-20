#!/usr/bin/env python3
"""R2 Gazebo static validation (C-078): per-run extras. Host, read-only, recorded; NEVER part of the gate.

Usage:  python3 -I r2gz_extra.py <run_dir> <new_out_json>

Reads only files the unchanged H2 analysis already wrote (<run_dir>/analysis/s6b_series.npz,
s6b_analysis.json, h2_metrics.json, s6b_host.json, s6b1_*.json, digest_cell.json) and the cell's
cell_result.json. Writes one JSON (refuses to overwrite).

Window W = [t0 + 2 s, episode end] (sim time), the H2 matched window 'after_t0+2'.

Oscillation / smoothness metrics on the published command q̈_safe (/NS_1/qddot_safe, series tq/Q) and,
as the same-run reference, on the nominal command q̈_nom (/NS_1/qddot_nom, tn/Nn):
  * reversals_per_s: sign changes per sim second, per joint, of the command with a dead band
    (|q̈| < DEADBAND is "zero" and never counts as a sign), summed over the 7 joints;
  * hf_power_frac: share of the command's power (mean removed, 100 Hz uniform resampling by linear
    interpolation, all joints pooled) above HF_HZ;
  * step_norm: ||q̈(k) - q̈(k-1)|| between consecutive published commands, percentiles.
The ratios safe/nominal separate what the filter adds from what the task itself contains.
"""
import json
import os
import sys

import numpy as np

sys.dont_write_bytecode = True
DEADBAND = 0.05     # rad/s²: below it a command component counts as zero (no sign)
HF_HZ = 10.0        # Hz: "high-frequency" boundary (the task's own content is < 1 Hz: a 6 s circle)
FS = 100.0          # Hz: resampling rate (= the QP target rate of condition full)
W_LEAD = 2.0        # s after the birth: the H2 matched window 'after_t0+2'

RUN, OUT = sys.argv[1], sys.argv[2]
if os.path.exists(OUT):
    sys.exit(f'r2gz_extra: {OUT} exists')
A_DIR = os.path.join(RUN, 'analysis')


def load(name):
    with open(os.path.join(A_DIR, name)) as f:
        return json.load(f)


def pct(x):
    x = np.asarray(x, float)
    x = x[np.isfinite(x)]
    if not x.size:
        return None
    return dict(n=int(x.size), p50=float(np.percentile(x, 50)), p95=float(np.percentile(x, 95)),
                p99=float(np.percentile(x, 99)), max=float(x.max()), mean=float(x.mean()))


def reversals_per_s(t, X):
    if len(t) < 3:
        return None
    sgn = np.sign(np.where(np.abs(X) < DEADBAND, 0.0, X))
    n = 0
    for j in range(X.shape[1]):
        s = sgn[:, j][sgn[:, j] != 0]
        n += int((s[1:] != s[:-1]).sum())
    return n / (t[-1] - t[0])


def hf_power_frac(t, X):
    if len(t) < 64:
        return None
    g = np.arange(t[0], t[-1], 1.0 / FS)
    tot = hf = 0.0
    for j in range(X.shape[1]):
        y = np.interp(g, t, X[:, j])
        y = y - y.mean()
        P = np.abs(np.fft.rfft(y)) ** 2
        f = np.fft.rfftfreq(len(y), 1.0 / FS)
        tot += P[1:].sum()
        hf += P[f > HF_HZ].sum()
    return float(hf / tot) if tot > 0 else None


def cmd_metrics(t, X, lo, hi):
    m = (t >= lo) & (t <= hi)
    t, X = t[m], X[m]
    rps = reversals_per_s(t, X)
    rate = (len(t) - 1) / (t[-1] - t[0]) if len(t) > 1 else None
    return dict(n=int(m.sum()), reversals_per_s=rps,
                # rate-independent form: sign reversals per published command (all joints), since a filter
                # publishing more often has more opportunities to reverse per second
                reversals_per_command=None if rps is None or not rate else rps / rate,
                hf_power_frac=hf_power_frac(t, X),
                step_norm=pct(np.linalg.norm(np.diff(X, axis=0), axis=1)) if len(X) > 1 else None,
                norm=pct(np.linalg.norm(X, axis=1)))


z = np.load(os.path.join(A_DIR, 's6b_series.npz'))
SA = load('s6b_analysis.json')
H2 = load('h2_metrics.json')
HO = load('s6b_host.json')
RC = load('s6b1_reconstruction.json')
SU = load('s6b1_supplementary.json')
DG = load('digest_cell.json')
cells = [os.path.join(RUN, 'run', 'batch', 'cells', c) for c in os.listdir(os.path.join(RUN, 'run', 'batch', 'cells'))]
if len(cells) != 1:
    sys.exit('r2gz_extra: expected one cell')
with open(os.path.join(cells[0], 'cell_result.json')) as f:
    CL = json.load(f).get('closed_loop', {})

t0 = float(H2['placement']['t0'])
t_end = float(SA['episode_sim'][1])
lo, hi = t0 + W_LEAD, t_end
safe = cmd_metrics(z['tq'], z['Q'], lo, hi)
nom = cmd_metrics(z['tn'], z['Nn'], lo, hi)


def ratio(a, b):
    return None if a is None or b in (None, 0) else a / b


W = H2['windows']['after_t0+2']
cpu = HO['cpu']
doc = dict(
    run_dir=RUN, window_sim=[lo, hi], t0=t0, constants=dict(deadband=DEADBAND, hf_hz=HF_HZ, fs=FS, w_lead=W_LEAD),
    oscillation=dict(
        safe=safe, nominal=nom,
        reversals_ratio_safe_over_nominal=ratio(safe['reversals_per_s'], nom['reversals_per_s']),
        reversals_per_command_ratio_safe_over_nominal=ratio(safe['reversals_per_command'],
                                                            nom['reversals_per_command']),
        hf_ratio_safe_over_nominal=ratio(safe['hf_power_frac'], nom['hf_power_frac'])),
    summary=dict(
        # timing (window after t0+2, H2 metrics; episode values from s6b_analysis)
        qp_ticks_per_sim_s_window=W['qp_ticks_per_sim_s'],
        qp_ticks_per_sim_s_episode=SA['qp']['rate_per_sim_s'],
        missed_slots_episode=SA['qp']['missed'],
        missed_slots_window=float(W['sim_s'] * 100.0 - W['qp_ticks']),
        tick_gap_ms_window=W['tick_gap_ms'], gaps_ge_50ms_window=W['gaps_ge_50ms'],
        gaps_ge_50ms_episode=SA['qp']['gaps_ge_50ms'], gaps_ge_100ms_episode=SA['qp']['gaps_ge_100ms'],
        tick_gap_wall_ms_episode=SA['wall_map']['tick_gap_wall_ms'],
        snapshot_age_ms_window=W['snapshot_age_ms'],
        ages_ms_episode=SA['qp']['ages_ms'],
        rebuilds_per_sim_s_window=W['rebuilds_per_sim_s'], build_wall_ms_window=W['build_wall_ms'],
        solve_wall_ms_window=W['solve_wall_ms'], n_rows_window=W['n_rows'],
        p_late_tick_after_fresh_rebuild=SA['gap_vs_rebuild']['p_long_after_new'],
        p_late_tick_otherwise=SA['gap_vs_rebuild']['p_long_otherwise'],
        # safety
        recon_min_gap_m=RC['recon_clearance']['min_gap'], recon_cp_min=RC['recon_clearance'].get('cp_min'),
        recon_time_below_dsafe_s=RC['recon_clearance']['time_below_dsafe_s'],
        continuous_lower_bound_m=SU['sampling']['continuous_lower_bound_m'],
        truth_min_gap_m=CL.get('min_gap'), contact=CL.get('contact'),
        min_clearance_window_m=W['min_clearance_m'],
        fallback_levels=CL.get('fallback_levels'), fault_ticks=CL.get('fault_ticks'),
        reflex_verdict=CL.get('reflex_verdict'), reflex_trips=CL.get('reflex_trips'),
        slack_max=CL.get('slack_max'), slack_max_any=CL.get('slack_max_any'),
        obs_slack_ticks=SA['family']['obs']['slack']['ticks_pos'],
        # tracking and smoothness
        tracking_err_m_window=W['tracking_err_m'], ee_err_norm_episode=SA['ee']['err_norm'],
        ee_final_err_m=SA['ee']['final_err'], pentagon=CL.get('pentagon'),
        pentagon_tracking_digest=DG.get('pentagon_tracking'),
        jerk_p50=CL.get('jerk_p50'), jerk_p99=CL.get('jerk_p99'), jerk_max=CL.get('jerk_max'),
        smoothness_verdicts=DG.get('smoothness_verdicts'),
        # host and real-time factor
        rtf_harness=CL.get('rtf'), rtf_p10=CL.get('rtf_p10'), rtf_wall_bag=SA['wall_map']['episode_rtf'],
        rtf_window=W['rtf'], filter_cpu_pct_one_core_window=W['filter_cpu_pct_one_core'],
        filter_cpu_pct_episode={k: v for k, v in cpu['process_pct_of_one_core'].items()
                                if k.startswith('cbf_safety_filter[')},
        host_busy_cores_equiv=cpu['host_busy_cores_equiv'],
        process_cpu_pct_one_core=cpu['process_pct_of_one_core'],
        # perception and controller timing
        perception_capture_age_ms_p95=CL.get('capage_ms_p95'), rows_rate_hz=CL.get('rows_rate_hz'),
        rows_latency_ms_p50=CL.get('rows_latency_ms_p50'), rows_latency_ms_p95=CL.get('rows_latency_ms_p95'),
        topic_rates_sim=SA['rates_sim'], truth_hole_max_ms=SA['truth']['hole_max_ms'],
        latency=H2.get('latency'), validity=H2['validity'].get('verdict'),
        placement={k: H2['placement'].get(k) for k in ('t0', 'phase_at_birth', 'phase_error_cycles',
                                                       'initial_clearance_m', 'initial_clearance_cp',
                                                       'tracking_err_at_birth_m')},
    ),
)
with open(OUT, 'x') as f:
    json.dump(doc, f, indent=1, default=float)
o = doc['oscillation']
print(f"r2gz_extra: window [{lo:.2f}, {hi:.2f}] s; reversals/s safe {safe['reversals_per_s']:.2f} "
      f"nominal {nom['reversals_per_s']:.2f}; HF>{HF_HZ:.0f}Hz safe {safe['hf_power_frac']:.4f} "
      f"nominal {nom['hf_power_frac']:.4f}; QP {W['qp_ticks_per_sim_s']:.2f}/sim s")
