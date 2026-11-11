#!/usr/bin/env python3
"""Scoring: what each sim metric measures, and where it can mislead (chapter 12 §8–§9). Container.

Production code, called on synthetic inputs in a temporary directory OUTSIDE the repository:
`simexp.reflex_checks.ReflexChecker/RtfMeter`, `simexp.closed_loop.summarize_run/rate_verdict`,
`simexp.batch.cell_ok`, `simexp.emulation.nearest/frame_results`, `simexp.truth_geometry.surface_gaps`,
`utils.depth_conversion.metres_to_mm/range_sigma/FidelityStages`.

  1. ReflexChecker: velocity / position / torque events are trips; a torque-rate excess is a CLIP,
     not a trip; a sample gap > 2.5 ms is not differentiated                                     (NUM)
  2. the monitor's verdict needs >= 90 % of 1 kHz samples, otherwise NO_DATA; RtfMeter            (FACT/NUM)
  3. (12.12)-(12.14) summarize_run on a synthetic truth.npz: min_h, contact and neg_duration_s are
     exact on a uniform grid; with the recorded kind of hole (200 ms) inside the penetration the
     minimum and the duration are under-reported (F-6-5)                                          (NUM)
  4. batch.cell_ok = "valid data point": contact, a reflex FAIL and an UNKNOWN rate all pass it  (FACT)
  5. F-5-21 scope: no batch scorer reads experiment_log.csv; only runner.digest_cell runs
     experiment_summary.py on it, and digest_cell is called only by a slow test                (FACT)
  6. emulated perception: the gap is clamped at 0, so penetration is invisible to the filter,
     and it equals surface_gaps otherwise; one track id (1) for every obstacle                     (NUM/FACT)
  7. depth adapter: encoding, sigma_z(z) = z^2 sigma_d / (f b), delay keeps the capture stamp     (NUM)
  8. rescoring uses the CURRENT conditions.yaml and estimator code, and writes into the run dir;
     the truth node rewrites its whole history every 5 s in its only executor thread (F-6-5)       (FACT)
"""
import glob
import json
import os
import sys
import tempfile

sys.dont_write_bytecode = True
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _common6 as C6  # noqa: E402
import numpy as np  # noqa: E402
from simexp import batch, closed_loop, emulation, reflex_checks as RC, truth_geometry as TG  # noqa: E402
from utils import depth_conversion as DC  # noqa: E402

R = C6.C.Report('check_scoring (chapter 12 §8-§9)')

# 1 ReflexChecker
L = RC.robot_limits()
q = 0.5 * (L['q_min'] + L['q_max'])
up, lo = RC.envelope(q)
chk = RC.ReflexChecker(L['q_min'], L['q_max'], L['tau_max'])
z = np.zeros(7)
chk.check(0.000, q, z, z, z)
qd = z.copy(); qd[0] = up[0] + 2e-3
ev1 = chk.check(0.001, q, qd, z, z)
qbad = q.copy(); qbad[3] = L['q_max'][3] + 1e-3
ev2 = chk.check(0.002, qbad, z, z, z)
tau = z.copy(); tau[6] = L['tau_max'][6] + 0.1
ev3 = chk.check(0.003, q, z, tau, z)
u = z.copy(); u[2] = 2.0                         # 2 N·m in 1 ms = 2000 N·m/s
ev4 = chk.check(0.004, q, z, z, u)
ev5 = chk.check(0.010, q, z, z, z)               # 6 ms gap: not differentiated
R.fact(f'1. events: velocity {ev1}, position {ev2}, torque {ev3}, rate {ev4}, after a 6 ms gap {ev5}',
       [e[0] for e in ev1] == ['velocity'] and [e[0] for e in ev2] == ['position']
       and [e[0] for e in ev3] == ['torque'] and [e[0] for e in ev4] == ['torque_rate_clipped'] and ev5 == [])
R.close('1. n_trips counts velocity + position + torque, not the clip', chk.n_trips, 3, 0, kind='NUM')
R.fact('1. velocity limit = fr3_velocity_envelope(q, margin=1.0) + 1e-3 rad/s; rate limit 1000 - 1e-3 N·m/s',
       RC.VEL_TOL == 1e-3 and abs(RC.TORQUE_RATE_MAX - 999.999) < 1e-12)

# 2 verdict and RTF
mon = C6.src('franka_simulation_experiments/scripts/reflex_monitor.py', 'py')
R.fact("2. verdict: FAIL if any trip, else PASS only if span > 0.5 s and n_samples >= 0.9 * 1000 * span, "
       "else NO_DATA (a starved monitor is not a PASS)",
       "enough = sim_span > 0.5 and self.chk.n_samples >= 0.9 * 1000.0 * sim_span" in mon
       and "verdict = ('FAIL' if self.chk.n_trips else ('PASS' if enough else 'NO_DATA'))" in mon)
m = RC.RtfMeter(1.0)
for k in range(0, 101):
    m.push(0.5 * 0.1 * k, 0.1 * k)              # sim advances at half the wall rate
s = m.summary()
R.close('2. RtfMeter on sim = 0.5 wall: rtf_run', s['rtf_run'], 0.5, 1e-9, kind='NUM')
R.fact('2. RtfMeter flags a run below 0.9', s['flagged'] is True)

# 3 summarize_run on synthetic truth
def run_dir(t, gap, d_safe=0.15):
    d = tempfile.mkdtemp(prefix='fs6_')
    with open(os.path.join(d, 'run_manifest_sim.json'), 'w') as f:
        json.dump({'episode': {'t_start_sim': 1.0, 't_end_sim': 9.0},
                   'condition': {'cbf_params': {'qp_rate_hz': 100.0}}}, f)
    n = len(t)
    np.savez(os.path.join(d, 'truth.npz'), t=t, gap=gap.reshape(n, 1, 1), q=np.zeros((n, 7)),
             labels=np.array(['fr3_link7#0']), d_safe=d_safe)
    return d

def g_of(t):
    return 0.10 - 0.13 * np.exp(-((t - 5.0) / 0.08) ** 2)     # min gap -0.03 m at t = 5 s

t_u = np.arange(0.0, 10.0, 1e-3)
fine = np.arange(1.0, 9.0, 1e-6)
neg_true = float((g_of(fine) < 0).sum() * 1e-6)
out_u = closed_loop.summarize_run(run_dir(t_u, g_of(t_u)))
R.close('3. uniform 1 kHz grid: min_h = min gap - d_safe [m]', out_u['min_h'], -0.03 - 0.15, 1e-6, kind='NUM')
R.fact('3. uniform grid: contact = (gap < 0) anywhere in the window', out_u['contact'] is True)
R.close('3. uniform grid: neg_duration_s counts h < 0 (gap < d_safe), not contact [s]',
        out_u['neg_duration_s'], float(((g_of(fine) - 0.15) < 0).sum() * 1e-6), 2e-3, kind='NUM')
keep = ~((t_u > 4.95) & (t_u < 5.15))           # a 200 ms hole, as recorded (F-6-5)
out_h = closed_loop.summarize_run(run_dir(t_u[keep], g_of(t_u[keep])))
R.fact(f"3. F-6-5 with a 200 ms hole: min gap reported {out_h['min_gap']:+.4f} m vs true -0.0300; "
       f"contact {out_h['contact']}; h<0 duration {out_h['neg_duration_s']:.3f} s vs {out_u['neg_duration_s']:.3f} s",
       out_h['min_gap'] > -0.03 + 1e-3 and out_h['neg_duration_s'] < out_u['neg_duration_s'] - 0.1)
R.info(f'3. true penetration time (gap < 0) of the synthetic pass: {neg_true:.3f} s; the scorer has no field '
       'for it (neg_duration_s is the time inside d_safe)')

# 4 cell_ok
base = dict(timed_out=False, leftovers=[], closed_loop=dict(min_h=-0.2, qp_rate_ok=None,
                                                            contact=True, reflex_verdict='FAIL'))
R.fact('4. cell_ok is True for a cell with contact, a reflex FAIL and an UNKNOWN QP rate',
       batch.cell_ok(base) is True)
R.fact('4. cell_ok is False only for a timeout, leftovers, no min_h, or a KNOWN rate miss',
       not batch.cell_ok(dict(base, timed_out=True))
       and not batch.cell_ok(dict(base, closed_loop=dict(min_h=None)))
       and not batch.cell_ok(dict(base, closed_loop=dict(min_h=0.1, qp_rate_ok=False))))

# 5 F-5-21 scope
sx = os.path.join(C6.SIMEXP_SCRIPTS, 'simexp')
readers = sorted(os.path.basename(p) for p in glob.glob(os.path.join(sx, '*.py'))
                 if 'experiment_log.csv' in open(p).read())
callers = sorted(os.path.relpath(p, C6.C.REPO) for p in glob.glob(
    os.path.join(C6.C.REPO, 'franka_simulation_experiments', '**', '*.py'), recursive=True)
    if 'digest_cell(' in open(p, errors='ignore').read() and '/sim_runs/' not in p)
R.fact(f'5. simexp modules that name experiment_log.csv: {readers}; digest_cell( appears in {callers}',
       readers == ['analysis.py'] and callers == [
           'franka_simulation_experiments/scripts/simexp/runner.py',
           'franka_simulation_experiments/test/slow/test_phase2_closed_loop.py'])
R.fact('5. batch.score_cell = closed_loop + estimator_metrics + attribution (truth.npz, the bag, launch.log)',
       "out = {'closed_loop': closed_loop.summarize_run(rd)}" in C6.src(
           'franka_simulation_experiments/scripts/simexp/batch.py'))

# 6 emulation
rng = np.random.default_rng(3)
cps = [dict(point=rng.uniform(-0.5, 0.5, 3), radius=float(r), seg_idx=0, cp_idx=i, start_link='fr3_link6',
            end_link='fr3_link7') for i, r in enumerate(rng.uniform(0.03, 0.08, 8))]
cen, rad = rng.uniform(-0.5, 0.5, (3, 3)), rng.uniform(0.03, 0.1, 3)
G = TG.surface_gaps(cps, cen, rad)
res, ks, gaps = emulation.frame_results(cps, cen, rad)
R.below('6. emulation gap = min over obstacles of surface_gaps (12.11) [m]',
        float(np.max(np.abs(gaps - G.min(axis=1)))), 1e-12)
cps_in = [dict(cps[0], point=cen[0] + np.array([0.01, 0, 0]))]
r_in, _, g_in = emulation.frame_results(cps_in, cen[:1], rad[:1])
R.fact(f'6. a penetrating CP (true gap {g_in[0]:+.3f} m) is reported at distance {r_in[0].distance}',
       g_in[0] < 0 and r_in[0].distance == 0.0)
em = C6.src('franka_simulation_experiments/scripts/simexp/emulation.py', 'py')
R.fact('6. every emulated row carries track_id = 1, whatever the obstacle', 'ld.track_id = 1' in em)

# 7 depth adapter
mm = DC.metres_to_mm(np.array([0.5004, 0.5006, np.nan, np.inf, -np.inf, -0.1, 70.0], np.float32))
R.fact(f'7. metres_to_mm [0.5004, 0.5006, nan, inf, -inf, -0.1, 70] -> {mm.tolist()}',
       mm.tolist() == [500, 501, 0, 0, 0, 0, 0])
R.close('7. range_sigma(2 m, f=390 px, b=0.095 m, 0.2 px) = z^2 sigma_d / (f b) [m]',
        float(DC.range_sigma(np.array([2.0]), 390.0, 0.095, 0.2)[0]), 4.0 * 0.2 / (390.0 * 0.095), 1e-15, kind='NUM')
st = DC.FidelityStages(delay_s=0.03)
st.push(1.000, np.ones((2, 2), np.float32), tag='a')
early, due = st.pop_due(1.029), st.pop_due(1.030)
R.fact('7. delay 30 ms: released at capture + 30 ms with the capture stamp kept',
       early == [] and len(due) == 1 and due[0][0] == 1.000 and due[0][1] == 'a')
st = DC.FidelityStages(dropout_p=0.1, seed=0)
kept = sum(st.push(float(i), np.ones((1, 1), np.float32)) for i in range(20000))
R.close('7. dropout 0.1: share of frames kept', kept / 20000, 0.9, 0.01, kind='NUM')

# 8 rescoring and truth saving
est = C6.src('franka_simulation_experiments/scripts/simexp/estimator_metrics.py', 'py')
R.fact('8. the residual estimator of a rescore is built from conditions.resolve(condition) and the CURRENT '
       'shipped_params, not from the run manifest; score_run writes compare_vobs_sim.npz into the run dir',
       'over = offline.overrides(conditions.resolve(condition, sets), C.shipped_params())' in est
       and "np.savez_compressed(os.path.join(run_dir, 'compare_vobs_sim.npz')," in est)
gt = C6.src('franka_simulation_experiments/scripts/ground_truth_node.py', 'py')
R.fact('8. ground_truth: 1 kHz joint-state callback (queue 100) and a 5 s timer that np.savez_compresses the '
       'WHOLE history, in one single-threaded rclpy.spin',
       "self.create_timer(5.0, self.save)" in gt and 'np.savez_compressed(' in gt
       and "self._js, 100)" in gt and 'rclpy.spin(node)' in gt and 'MultiThreaded' not in gt)
sys.exit(R.finish())
