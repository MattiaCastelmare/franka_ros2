"""E3 H2 — host step. Placement record, pre-registered validity, latencies, matched windows.

Read-only on its inputs; writes ONE new file (refuses to overwrite).

  python3 -I e3_h2_metrics.py <run_dir> <h2_bag_dir> <out_json>
     <run_dir>    runs/<NN>_s<seed>_<mode> (cell under run/, analysis/ with s6b_series.npz,
                  s6b1_reconstruction.{npz,json}; monitor/proc_samples.jsonl)
     <h2_bag_dir> where e3_h2_bag.py wrote h2_bag.json / h2_bag.npz (normally <run_dir>/analysis)

VALIDITY (PREREGISTRATION.md) uses ONLY the state at, or before, the obstacle's birth t0. No
quantity measured after t0 enters it: latencies, clearance minima and every window metric are
recorded beside it, never used to accept or reject the run. An INVALID run is kept and reported
in full (the gate STOPs on it; nothing is discarded or relabelled).
"""
import glob
import json
import math
import os
import re
import sys

import numpy as np

REPO = os.path.expanduser('~/Git/franka_ros2')
sys.path.insert(0, os.path.join(REPO, 'franka_experiments'))
sys.path.insert(0, os.path.join(REPO, 'franka_simulation_experiments', 'scripts'))
from simexp import phase_start as ps  # noqa: E402
from simexp import obstacle_laws as OL  # noqa: E402   (F-8-41: the obstacle's own law, as the plugin runs it)
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import h3_moving as H3  # noqa: E402   (F-8-41 harness correction; reduces exactly to H2 for a static scene)

RUN, BAGDIR, OUT = sys.argv[1], sys.argv[2], sys.argv[3]

# ------------------------------------------------------------------ pre-registered constants
PREREG = dict(
    phase0=0.0,                # [cycles] birth phase (E3-05/06)
    phase_tol=0.01,            # [cycles] = 60 ms of a 6 s cycle
    track_err_max=0.02,        # [m] = commander soft_reset_thr
    hard_reset_thr=0.05,       # [m] = commander hard_reset_thr
    settle_cycles=1.0,
    rate_rel_tol=0.02,
    d_safe=0.10,               # [m] fr3_control.yaml (R14 checks the filter's own line)
    v_ref=0.35,                # [m/s] bound on control-point speed in unobstructed tracking
    tau_budget=0.25,           # [s] birth -> first obstacle row in the QP, budget
    g_min_reference_014=0.10 + 0.149 * 0.2618,   # the 0.14 m proposal, RECORDED only
    pld_min_msgs_1s=10,        # perception: messages in the last second before the birth
    pld_max_capture_age_s=0.2,
)
PREREG['g_min'] = PREREG['d_safe'] + PREREG['v_ref'] * PREREG['tau_budget']   # 0.1875 m


def pct(x, ps_=(50, 95, 99)):
    x = np.asarray(x, float)
    x = x[np.isfinite(x)]
    if not x.size:
        return None
    d = {f'p{p}': float(np.percentile(x, p)) for p in ps_}
    d.update(n=int(x.size), min=float(x.min()), max=float(x.max()), mean=float(x.mean()))
    return d


cells = sorted(glob.glob(os.path.join(RUN, 'run', 'batch', 'cells', '*')))
if len(cells) != 1:
    sys.exit(f'expected one cell, found {cells}')
CELL = cells[0]
man = json.load(open(os.path.join(CELL, 'run_manifest_sim.json')))
# F-8-41 (C1): g_min bounds the CLOSING speed, which for a moving ball is |v_cp| + |v_obs| (static: 0.1875 m)
PREREG['v_obs_max'] = H3.scenario_speed_bound(man['scenario_resolved'])
PREREG['g_min'] = H3.g_min(PREREG['d_safe'], PREREG['v_ref'], PREREG['tau_budget'], PREREG['v_obs_max'])
ep = man['episode']
start = ep.get('start') or {}
T0, T1 = ep['t_start_sim'], ep['t_end_sim']
R = dict(run_dir=RUN, cell=CELL, prereg=PREREG, start_mode=start.get('mode', 'legacy'),
         start_status=start.get('status'))
if T0 is None:
    R['error'] = f'no birth: start status {start.get("status")}: {start.get("fail_reason")}'
    R['validity'] = dict(verdict='INVALID', criteria=[], reason=R['error'])
    with open(OUT, 'x') as f:
        json.dump(R, f, indent=1, default=str)
    print(R['error'])
    sys.exit(0)
T0, T1 = float(T0), float(T1)

# commander curve: the supervisor's record (phase mode) or the start-up banner (legacy runs)
cp_par = start.get('commander_params')
if cp_par:
    R['commander_params_source'] = 'supervisor GetParameters'
else:
    log = open(os.path.join(CELL, 'launch.log'), errors='replace').read()
    m = re.search(r'center\s*:\s*\[([^\]]+)\]\s*r=([0-9.]+) m\s*plane=(\w+)', log)
    c = re.search(r'cycle_t\s*:\s*([0-9.]+) s', log)
    cp_par = dict(path_type='pentagon' if 'only C¹' in log else 'circle',
                  center_xyz=[float(v) for v in m.group(1).split(',')], radius=float(m.group(2)),
                  plane=m.group(3), cycle_time=float(c.group(1)))
    R['commander_params_source'] = 'launch.log banner (legacy run; path_type from the absence of the pentagon warning)'
R['commander_params'] = cp_par
T = float(cp_par['cycle_time'])
est = ps.PhaseEstimator(ps.build_cycle_path(cp_par))

Z = np.load(os.path.join(RUN, 'analysis', 's6b_series.npz'))
REC = np.load(os.path.join(RUN, 'analysis', 's6b1_reconstruction.npz'))
RJ = json.load(open(os.path.join(RUN, 'analysis', 's6b1_reconstruction.json')))
B = json.load(open(os.path.join(BAGDIR, 'h2_bag.json')))
PLD = np.load(os.path.join(BAGDIR, 'h2_bag.npz'))['pld']
from franka_experiments.utils.cbf_diagnostics import STATUS_FIELDS  # noqa: E402  (as s6b_analyze.py)
FI = {n: i for i, n in enumerate(STATUS_FIELDS)}

tS, S = Z['tS'], Z['S']
if S.shape[1] != len(STATUS_FIELDS):
    sys.exit(f'cbf_status has {S.shape[1]} fields, STATUS_FIELDS {len(STATUS_FIELDS)}: layout mismatch')
ted, Ed, tea, Ea = Z['ted'], Z['Ed'], Z['tea'], Z['Ea']
PW = Z['wall_pairs']
tr, gmin, cpos = REC['t'], REC['gmin'], REC['cp']
labels = [str(x) for x in REC['labels']]

# ee pairs: the commander stamps both with one clock read; the bag keeps receipt times, so pair
# each ee_desired with the ee_actual received within 2 ms
j = np.clip(np.searchsorted(tea, ted), 1, len(tea) - 1)
j = np.where(np.abs(tea[j - 1] - ted) < np.abs(tea[j] - ted), j - 1, j)
okp = np.abs(tea[j] - ted) < 0.002
tp, err = ted[okp], np.linalg.norm(Ed[okp] - Ea[j[okp]], axis=1)

# phase of every ee_desired sample, with its distance to the curve, unwrapped on the curve
phi = np.empty(len(ted))
dcurve = np.empty(len(ted))
for k in range(len(ted)):
    phi[k], dcurve[k] = est.locate(Ed[k])
uw = phi + np.concatenate([[0.0], np.cumsum(np.round(-np.diff(phi)).clip(0, None))])


def phase_at(t):
    """curve phase at sim time t from the bracketing ee_desired samples (linear in phase)."""
    i = int(np.searchsorted(ted, t))
    if i == 0 or i >= len(ted):
        return None, None
    f = (t - ted[i - 1]) / (ted[i] - ted[i - 1])
    p = (phi[i - 1] + f * ps.wrap(phi[i] - phi[i - 1])) % 1.0
    return p, max(dcurve[i - 1], dcurve[i])


def interp_rows(t, tt, xx):
    return np.array([np.interp(t, tt, xx[:, c]) for c in range(xx.shape[1])])


# ------------------------------------------------------------------ placement (t <= t0 only)
ph0, dc0 = phase_at(T0)
pre = (tea <= T0) & (tea >= T0 - 0.05)
ee_v = (np.polyfit(tea[pre] - T0, Ea[pre], 1)[0] if pre.sum() >= 3 else np.full(3, np.nan))
i0 = int(np.searchsorted(tr, T0))                  # first reconstruction sample at/after t0
# S6B.1 reconstructs control points from t0 on, so their speed AT the birth is a 20 ms FORWARD
# difference: informational, never a validity input (V1-V7 use data at or before t0 only)
k = 20
v_cp = np.linalg.norm(cpos[i0 + k] - cpos[i0], axis=1) / (tr[i0 + k] - tr[i0])
# F-8-41 (C1): the ball's position at each sample from its own law (law time = t - t0); static: p0
ball = np.array([OL.position(man['scenario_resolved']['obstacles'][0], float(x) - T0)
                 for x in tr[i0:i0 + k + 1]], float)
dist_b = np.linalg.norm(cpos[i0:i0 + k + 1] - ball[:, None, :], axis=2)
v_close = -(dist_b[k] - dist_b[0]) / (tr[i0 + k] - tr[i0])
cyc = (tp >= T0 - T) & (tp <= T0)
js_b = B.get('joint_state_before_birth')
sup_pl = (start.get('placement') or {})
R['placement'] = dict(
    t0=T0, phase_at_birth=ph0, phase_error_cycles=None if ph0 is None else ps.wrap(ph0 - PREREG['phase0']),
    ee_desired_dist_to_curve_m=dc0,
    ee_position=interp_rows(T0, tea, Ea).tolist(), ee_velocity=ee_v.tolist(),
    ee_speed=float(np.linalg.norm(ee_v)),
    q=js_b['q'] if js_b else None, qdot=js_b['qdot'] if js_b else None,
    joint_state_stamp=js_b['stamp'] if js_b else None,
    tracking_err_at_birth_m=float(np.interp(T0, tp, err)),
    tracking_err_max_last_cycle_m=float(err[cyc].max()) if cyc.any() else None,
    pairs_last_cycle=int(cyc.sum()),
    pairs_above_hard_thr_last_cycle=int((err[cyc] > PREREG['hard_reset_thr']).sum()),
    pairs_above_hard_thr_before_birth=int((err[tp <= T0] > PREREG['hard_reset_thr']).sum()),
    initial_clearance_m=float(gmin[i0]), initial_clearance_cp=labels[int(np.nanargmin(REC['gap'][i0]))],
    initial_clearance_t=float(tr[i0]),
    cp_speed_max_first_20ms=float(np.nanmax(v_cp)), closing_speed_max_first_20ms=float(np.nanmax(v_close)),
    time_to_dsafe_at_vref_s=float((gmin[i0] - PREREG['d_safe']) / (PREREG['v_ref'] + PREREG['v_obs_max'])),
    perception_pre_birth=B.get('pre_birth_1s'),
    controller_active_at_start=ep.get('controller_active_at_start'),
    supervisor=dict(placement=sup_pl or None, decision=(start.get('qualifier') or {}).get('decision'),
                    controller_history=start.get('controller_history'),
                    hard_reset_log_lines=start.get('hard_reset_log_lines')),
    mask_at_birth=(B.get('mask_replay', {}).get('probes') or [None])[0])

# ------------------------------------------------------------------ validity (pre-registered)
P = R['placement']
dec = P['supervisor']['decision'] or {}
crit = []


def C(cid, text, ok, value, threshold):
    ok = None if ok is None else bool(ok)
    crit.append(dict(id=cid, text=text, verdict='PASS' if ok is True else 'FAIL' if ok is False else 'MISSING',
                     value=value, threshold=threshold))


ok1 = (start.get('mode') == 'phase' and start.get('status') == 'started' and bool(dec)
       and dec.get('window_cycles', 0) >= PREREG['settle_cycles']
       and dec.get('window_max_err_m', 1) <= PREREG['track_err_max']
       and abs(dec.get('rate_over_nominal', 0) - 1) <= PREREG['rate_rel_tol'])
C('V1', 'phase-mode start: >= 1 clean cycle tracked (controller active, on the curve, |e| <= 0.02 m, '
  'nominal phase rate) before the announcement', ok1,
  dict(mode=start.get('mode'), status=start.get('status'), window_cycles=dec.get('window_cycles'),
       window_max_err_m=dec.get('window_max_err_m'), rate_over_nominal=dec.get('rate_over_nominal')),
  dict(settle_cycles=PREREG['settle_cycles'], track_err_max=PREREG['track_err_max'],
       rate_rel_tol=PREREG['rate_rel_tol']))
C('V2', 'birth phase (recomputed from the bag ee_desired) within phase_tol of phase0',
  None if ph0 is None else (abs(P['phase_error_cycles']) <= PREREG['phase_tol'] and dc0 <= 1e-3),
  dict(phase=ph0, error=P['phase_error_cycles'], dist_to_curve=dc0), PREREG['phase_tol'])
lv = (sup_pl or {}).get('lead_violations')
C('V3', 'tracking over the last cycle before the birth: max |e| <= 0.02 m, no |e| > 0.05 m (HARD), '
  'no violation during the announcement lead',
  None if P['tracking_err_max_last_cycle_m'] is None else
  (P['tracking_err_max_last_cycle_m'] <= PREREG['track_err_max'] and P['pairs_above_hard_thr_last_cycle'] == 0
   and P['pairs_last_cycle'] >= 0.9 * T * 100 and lv == {}),
  dict(max_err=P['tracking_err_max_last_cycle_m'], hard_pairs=P['pairs_above_hard_thr_last_cycle'],
       pairs=P['pairs_last_cycle'], lead_violations=lv), PREREG['track_err_max'])
C('V4', 'initial clearance g0 >= d_safe + (v_ref + v_obs_max) * tau_budget', P['initial_clearance_m'] >= PREREG['g_min'],
  P['initial_clearance_m'], PREREG['g_min'])
mk = P['mask_at_birth']
C('V5', 'obstacle not inside the robot depth self-mask at the birth (replay: 0 px deleted, 0 px hidden)',
  None if mk is None else (mk['deleted_by_self_mask'] == 0 and mk['hidden_by_arm'] == 0),
  mk, 0)
pp = P['perception_pre_birth'] or {}
C('V6', 'perception live before the birth: >= 10 per_link_distances in the last 1 s, last capture <= 0.2 s old',
  None if not pp else (pp['msgs'] >= PREREG['pld_min_msgs_1s']
                       and pp['last_capture_age_s'] is not None
                       and pp['last_capture_age_s'] <= PREREG['pld_max_capture_age_s']),
  pp, dict(msgs=PREREG['pld_min_msgs_1s'], age=PREREG['pld_max_capture_age_s']))
C('V7', 'controller active at the birth (supervisor)', P['controller_active_at_start'] is True,
  P['controller_active_at_start'], True)
R['validity'] = dict(verdict='VALID' if all(c['verdict'] == 'PASS' for c in crit) else 'INVALID',
                     criteria=crit,
                     note='uses only the state at or before t0; an INVALID run is kept and fully reported')

# ------------------------------------------------------------------ latencies (recorded, not validity)
fd = B.get('first_nonempty_after_birth')
obs_row = np.isfinite(S[:, FI['d_obstacle_min']])
after = tS >= T0
iq = np.nonzero(after & obs_row)[0]
R['latency'] = dict(
    birth_to_first_nonempty_frame_capture_s=fd['capture_minus_t0'] if fd else None,
    birth_to_first_nonempty_frame_receipt_s=fd['receipt_minus_t0'] if fd else None,
    empty_frames_after_birth=fd['empty_frames_before'] if fd else None,
    first_frame_d_min=fd['d_min'] if fd else None,
    birth_to_first_qp_obstacle_row_s=float(tS[iq[0]] - T0) if iq.size else None,
    qp_rows_before_after=[pct(S[(tS < T0) & (tS >= T0 - 1), FI['n_rows']]),
                          pct(S[(tS >= T0 + 2), FI['n_rows']])],
    mask_replay=[dict(dt=r['dt'], visible=r['visible'], deleted=r['deleted_by_self_mask'],
                      hidden=r['hidden_by_arm']) for r in B.get('mask_replay', {}).get('probes', [])],
    first_visible_by_replay_s=next((r['dt'] for r in B.get('mask_replay', {}).get('probes', [])
                                    if r['visible'] > 0), None))

# ------------------------------------------------------------------ windows
G = np.arange(T0, T1, 0.001)
g_g = np.interp(G, tr, np.nan_to_num(gmin, nan=np.inf))
ok_curve = dcurve <= 1e-3
uw_g = np.interp(G, ted, uw)
ph_g = uw_g % 1.0
oncurve_g = np.interp(G, ted, ok_curve.astype(float)) > 0.999
s2w = (lambda s: np.interp(s, PW[:, 0], PW[:, 1])) if len(PW) > 10 else None

# filter CPU from the 4 Hz host sampler: cumulative (utime + stime) of the cbf_safety_filter pid
cpu_fn = None
try:
    recs = [json.loads(x) for x in open(os.path.join(RUN, 'monitor', 'proc_samples.jsonl')) if x.strip()]
    tck = recs[0]['clk_tck']
    names = {r['new']: r['cmdline'] for r in recs if 'new' in r}
    fpid = [p for p, c in names.items() if 'cbf_safety_filter' in c and '--ros-args' in c]
    smp = [r for r in recs if 'procs' in r and fpid and fpid[0] in r['procs']]
    if smp:
        cw = np.array([r['wall'] for r in smp])
        cc = np.array([(r['procs'][fpid[0]][0] + r['procs'][fpid[0]][1]) / tck for r in smp])
        cpu_fn = lambda w: np.interp(w, cw, cc)  # noqa: E731
except (OSError, ValueError, KeyError, IndexError):
    cpu_fn = None


def segments(mask):
    d = np.diff(np.r_[0, mask.astype(int), 0])
    return list(zip(np.nonzero(d == 1)[0], np.nonzero(d == -1)[0]))


def window(mask):
    segs = segments(mask)
    dur = mask.sum() / 1000.0
    if dur <= 0:
        return None
    sel = np.zeros(len(tS), bool)
    for a, b in segs:
        sel |= (tS >= G[a]) & (tS < G[b - 1] + 0.001)
    bw = S[:, FI['constraint_build_wall_ms']]
    newb = np.r_[np.isfinite(bw[0]), (bw[1:] != bw[:-1]) & np.isfinite(bw[1:])]
    gap = S[:, FI['tick_gap_ros_s']]
    wall = cpu = None
    if s2w is not None:
        wall = sum(float(s2w(G[b - 1] + 0.001) - s2w(G[a])) for a, b in segs)
        if cpu_fn is not None and wall > 0:
            cpu = 100.0 * sum(float(cpu_fn(s2w(G[b - 1] + 0.001)) - cpu_fn(s2w(G[a]))) for a, b in segs) / wall
    psel = np.zeros(len(tp), bool)
    for a, b in segs:
        psel |= (tp >= G[a]) & (tp < G[b - 1] + 0.001)
    return dict(
        sim_s=dur, segments=len(segs), qp_ticks=int(sel.sum()), qp_ticks_per_sim_s=float(sel.sum() / dur),
        tick_gap_ms=pct(1e3 * gap[sel]), gaps_ge_50ms=int((gap[sel] >= 0.05).sum()),
        n_rows=pct(S[sel, FI['n_rows']]), obstacle_row_fraction=float(obs_row[sel].mean()) if sel.any() else None,
        rebuilds_per_sim_s=float((newb & sel).sum() / dur), build_wall_ms=pct(bw[newb & sel]),
        solve_wall_ms=pct(S[sel, FI['solve_wall_ms']]),
        snapshot_age_ms=pct(1e3 * S[sel, FI['constraint_snapshot_age_s']]),
        rtf=(dur / wall) if wall else None, filter_cpu_pct_one_core=cpu,
        min_clearance_m=float(g_g[mask].min()), tracking_err_m=pct(err[psel]))


W = {}
late = G >= T0 + 2.0
W['after_t0+2'] = window(late)
for k6 in range(6):
    m = late & oncurve_g & (ph_g >= k6 / 6) & (ph_g < (k6 + 1) / 6)
    W[f'phase_{k6}/6'] = window(m)
for name, lo, hi in (('gap>0.25', 0.25, np.inf), ('gap_0.20-0.25', 0.20, 0.25), ('gap<=0.20', -np.inf, 0.20)):
    W[name] = window(late & (g_g > lo) & (g_g <= hi))
W['transient_t0..t0+1.5 (not matched)'] = window((G >= T0) & (G < T0 + 1.5))
R['windows'] = W
R['episode_cycles_after_t0+2'] = float((uw_g[-1] - uw_g[np.searchsorted(G, T0 + 2)]))
with open(OUT, 'x') as f:
    json.dump(R, f, indent=1, default=str)
v = R['validity']
print(f'validity {v["verdict"]}: ' + ', '.join(f'{c["id"]} {c["verdict"]}' for c in v['criteria']))
print('placement: phase', P['phase_at_birth'], 'g0', round(P['initial_clearance_m'], 4), 'err', round(P['tracking_err_at_birth_m'], 4))
print('latency:', {k: R['latency'][k] for k in ('birth_to_first_nonempty_frame_capture_s',
                                                 'birth_to_first_qp_obstacle_row_s', 'first_visible_by_replay_s')})
