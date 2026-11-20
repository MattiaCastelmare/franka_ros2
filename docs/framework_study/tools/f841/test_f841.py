#!/usr/bin/env python3
"""F-8-41 regression tests for the moving-obstacle harness correction (scripts_h3). Host, read-only on
every sealed run (outputs go to a fresh temporary directory, deleted at the end).

  python3 -I test_f841.py [-v]

A. h3_moving unit and property tests (speed bounds, g_min, lower bound, pose-pairing tolerance).
B. Bug demonstrations: synthetic moving-ball cases where the SEALED static formulas are wrong
   (lower bound above the true minimum; R13 STOP on an exact geometry) and the corrected ones are not.
C. Static equivalence on every sealed H2 run (E3-05..10 and, when present, R2 static 01..04): the
   corrected s6b1_supplementary.py and e3_h2_metrics.py reproduce the sealed outputs exactly, and the
   corrected gate (h3_analyze.sh --gate-only) gives the sealed verdict for every rule it can evaluate.
D. Moving-scenario numbers: wander_close's speed bound and g_min, the V4 margin at a phi0 = 0 birth.
The container-side corrections (C4 e3_h2_bag.py, C5 s6b1_reconstruct_clearance.py) are checked by
test_f841_container.sh against the same sealed runs.
"""
import glob
import json
import math
import os
import shutil
import subprocess
import sys
import tempfile
import unittest

import numpy as np

sys.dont_write_bytecode = True
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import h3_moving as H3  # noqa: E402

EV = os.path.expanduser('~/franka_baseline_evidence')
E3_RUNS = [os.path.join(EV, 'refactor_r1_e3', 'runs', r) for r in
           ('05_s0_multi', '06_s0_single', '07_s1_multi', '08_s1_single', '09_s2_single', '10_s2_multi')]
R2_RUNS = sorted(glob.glob(os.path.join(EV, 'refactor_r2', '05_gz_static', 'runs', '0[1-4]_s[01]_*')))
SEALED_STATIC = [r for r in E3_RUNS + R2_RUNS if os.path.exists(os.path.join(r, 'analysis', 'gate.json'))]
WANDER_CLOSE = dict(p0=[0.44, 0.0, 0.45], radius=0.06, law='wander', box=[0.08, 0.22, 0.14], period=7.0,
                    phase_deg_xyz=[0.0, -90.0, 0.0])
TMP = tempfile.mkdtemp(prefix='f841_')


def law_velocity(spec, t):
    """Independent re-statement of the wander / oscillate / linear velocities (not h3_moving's code)."""
    if spec['law'] == 'wander':
        w = 2 * math.pi / spec['period'] * np.asarray(spec.get('ratios', H3.WANDER_RATIOS))
        ph = np.radians(spec.get('phase_deg_xyz', (0, 0, 0)))
        return np.asarray(spec['box']) * w * np.cos(np.outer(t, w) + ph)
    raise ValueError


class A_Units(unittest.TestCase):
    def test_static_is_zero_and_g_min_is_the_sealed_value_bit_for_bit(self):
        self.assertEqual(H3.speed_bound({'law': 'static', 'p0': [0, 0, 0]}), 0.0)
        sealed = 0.10 + 0.35 * 0.25          # e3_h2_metrics.PREREG['g_min'] in scripts_h2
        self.assertEqual(H3.g_min(0.10, 0.35, 0.25, 0.0), sealed)
        self.assertEqual(sealed, 0.1875)

    def test_linear_and_oscillate(self):
        self.assertAlmostEqual(H3.speed_bound({'law': 'linear', 'v0': [0.3, 0.4, 0.0]}), 0.5, places=12)
        self.assertAlmostEqual(H3.speed_bound({'law': 'oscillate', 'amplitude': 0.35, 'period': 12.0,
                                               'axis': [0, 1, 0]}), 0.35 * 2 * math.pi / 12.0, places=12)

    def test_wander_bound_is_the_supremum_never_exceeded(self):
        b = H3.speed_bound(WANDER_CLOSE)
        t = np.arange(0.0, 3600.0, 0.001)
        s = np.linalg.norm(law_velocity(WANDER_CLOSE, t), axis=1)
        self.assertLessEqual(s.max(), b + 1e-12)          # an upper bound
        self.assertGreater(s.max(), b - 1e-3)             # and tight (approached within 1 mm/s in 1 h)

    def test_unbounded_laws_are_refused(self):
        for spec in ({'law': 'ballistic', 'v0': [0, 0, 1]}, {'law': 'pursuit', 'speed': 0.2}, {'law': 'nope'}):
            with self.assertRaises(ValueError):
                H3.speed_bound(spec)

    def test_scenario_bound_is_the_max_over_obstacles(self):
        sc = {'obstacles': [{'law': 'static'}, WANDER_CLOSE, {'law': 'linear', 'v0': [0.1, 0, 0]}]}
        self.assertEqual(H3.scenario_speed_bound(sc), H3.speed_bound(WANDER_CLOSE))
        self.assertEqual(H3.scenario_speed_bound({'obstacles': []}), 0.0)

    def test_lower_bound_static_reduces_to_the_sealed_formula(self):
        rng = np.random.default_rng(1)
        g = rng.uniform(0.1, 0.5, 1000)
        g[::97] = np.nan
        age = rng.uniform(0, 0.05, 1000)
        lb, dip = H3.continuous_lower_bound(g, age, 0.002, 0.31, 0.0)
        self.assertEqual(lb, float(np.nanmin(g) - 0.31 * 0.002 / 2.0))
        self.assertEqual(dip, float(0.31 * 0.002 / 2.0))

    def test_lower_bound_refuses_a_negative_or_missing_pose_age_when_moving(self):
        with self.assertRaises(ValueError):
            H3.continuous_lower_bound([0.2, 0.3], [0.01, -0.001], 0.001, 0.3, 0.4)
        with self.assertRaises(ValueError):
            H3.continuous_lower_bound([0.2, 0.3], [0.01, np.nan], 0.001, 0.3, 0.4)

    def test_g_min_bounds_the_worst_head_on_closing(self):
        # property: for random cp and ball velocities within the bounds, a gap of g_min at t0 stays >= d_safe
        rng = np.random.default_rng(2)
        vref, vobs, tau, d = 0.35, H3.speed_bound(WANDER_CLOSE), 0.25, 0.10
        g0 = H3.g_min(d, vref, tau, vobs)
        for _ in range(2000):
            u = rng.normal(size=3); u /= np.linalg.norm(u)
            c0 = np.zeros(3); p0 = c0 + u * (g0 + 0.06 + 0.05)          # surface gap g0 (radii 0.06 + 0.05)
            vc = rng.normal(size=3); vc *= rng.uniform(0, vref) / np.linalg.norm(vc)
            vb = rng.normal(size=3); vb *= rng.uniform(0, vobs) / np.linalg.norm(vb)
            ts = np.linspace(0, tau, 51)
            gap = np.linalg.norm((p0 + np.outer(ts, vc)) - (c0 + np.outer(ts, vb)), axis=1) - 0.11
            self.assertGreaterEqual(gap.min(), d - 1e-12)


class B_BugDemonstrations(unittest.TestCase):
    """The sealed static formulas applied to a moving ball, and the corrected ones."""

    def _approach(self):
        # a control point at rest; a ball closing on it at |v| = 0.44 m/s; joints at 1 kHz; obstacle pose
        # published every 50 ms (F-8-13) and paired causally, as s6b1_reconstruct_clearance.py does
        v = 0.44
        t = np.arange(0.0, 0.5, 0.001)
        ctr = lambda x: np.c_[0.5 - v * x, np.zeros_like(x), np.zeros_like(x)]   # noqa: E731
        true_gap = np.linalg.norm(ctr(t), axis=1) - 0.11
        pose_t = np.floor(t / 0.05) * 0.05
        recon = np.linalg.norm(ctr(pose_t), axis=1) - 0.11
        return t, true_gap, recon, t - pose_t, v

    def test_sealed_static_lower_bound_overestimates_a_moving_ball(self):
        t, true_gap, recon, age, v = self._approach()
        old = float(np.nanmin(recon) - 0.0 * 0.001 / 2.0)               # s6b1_supplementary (sealed), v_cp = 0
        self.assertGreater(old, true_gap.min() + 0.02)                  # wrong by > 2 cm: not a bound
        new, _ = H3.continuous_lower_bound(recon, age, 0.001, 0.0, v)
        self.assertLessEqual(new, true_gap.min() + 1e-12)               # corrected: a bound
        self.assertLess(true_gap.min() - new, 0.0006)                   # and tight to within the dip term

    def test_sealed_r13_stops_on_an_exact_geometry_and_the_corrected_one_does_not(self):
        t, true_gap, recon, age, v = self._approach()
        truth_pose_t = np.floor(t / 0.001) * 0.001                       # the truth node's own (fresher) pose
        truth = np.linalg.norm(np.c_[0.5 - v * truth_pose_t, 0 * t, 0 * t], axis=1) - 0.11
        d = np.abs(recon - truth)
        dt_pose = np.abs((t - age) - truth_pose_t)
        self.assertGreater(np.sort(d)[-2], 1e-9)                         # sealed R13 (<= EXACT) would STOP
        ex = H3.pose_pairing_excess(d, dt_pose, v)
        self.assertLessEqual(np.sort(ex)[-2], 1e-9)                      # corrected R13 passes
        d_bad = d + 1e-3                                                 # a real 1 mm geometry error
        self.assertGreater(np.sort(H3.pose_pairing_excess(d_bad, dt_pose, v))[-2], 1e-9)   # is still caught

    def test_sealed_v4_threshold_accepts_a_birth_that_can_reach_d_safe(self):
        vref, tau, d = 0.35, 0.25, 0.10
        vobs = H3.speed_bound(WANDER_CLOSE)
        g0 = 0.19                                    # passes the sealed g_min 0.1875
        closing = vref + vobs                         # head-on, both at their bounds
        self.assertLess(g0 - closing * tau, d)        # yet can reach d_safe within tau_budget
        self.assertLess(g0, H3.g_min(d, vref, tau, vobs))   # the corrected V4 rejects it


def sh(cmd, **kw):
    return subprocess.run(cmd, capture_output=True, text=True, **kw)


def strip(d, drop):
    if isinstance(d, dict):
        return {k: strip(v, drop) for k, v in d.items() if k not in drop}
    if isinstance(d, list):
        return [strip(x, drop) for x in d]
    return d


@unittest.skipUnless(SEALED_STATIC, 'no sealed H2 runs found')
class C_StaticEquivalence(unittest.TestCase):
    NEW_SUP = {'v_obs_max_mps', 'episode_min_gap_excess_over_pose_bound', 'pose_age_ms_max'}

    def _cell(self, run):
        c = glob.glob(os.path.join(run, 'run', 'batch', 'cells', '*'))
        self.assertEqual(len(c), 1)
        return c[0]

    def test_supplementary_reproduces_every_sealed_run(self):
        for run in SEALED_STATIC:
            with self.subTest(run=os.path.basename(run)):
                out = os.path.join(TMP, os.path.basename(run) + '_sup_t.json')
                p = sh([sys.executable, '-I', '-B', os.path.join(HERE, 's6b1_supplementary.py'), self._cell(run),
                         os.path.join(run, 'analysis', 's6b1_reconstruction.npz'), out])
                self.assertEqual(p.returncode, 0, p.stderr[-800:])
                new = json.load(open(out))
                old = json.load(open(os.path.join(run, 'analysis', 's6b1_supplementary.json')))
                self.assertEqual(new['v_obs_max_mps'], 0.0)
                self.assertEqual(strip(new, self.NEW_SUP), old)                       # bit-identical
                self.assertEqual(new['episode_min_gap_excess_over_pose_bound']['max'],
                                 old['episode_min_gap_abs_diff_excluding_placement_instant']['max'])

    def test_metrics_reproduce_every_sealed_run(self):
        for run in SEALED_STATIC:
            with self.subTest(run=os.path.basename(run)):
                out = os.path.join(TMP, os.path.basename(run) + '_h2m_t.json')
                p = sh([sys.executable, '-I', '-B', os.path.join(HERE, 'e3_h2_metrics.py'), run,
                         os.path.join(run, 'analysis'), out])
                self.assertEqual(p.returncode, 0, p.stderr[-800:])
                new = json.load(open(out))
                old = json.load(open(os.path.join(run, 'analysis', 'h2_metrics.json')))
                self.assertEqual(new['prereg']['v_obs_max'], 0.0)
                self.assertEqual(new['prereg']['g_min'], old['prereg']['g_min'])
                self.assertEqual(strip(new['prereg'], {'v_obs_max'}), old['prereg'])
                for a, b in zip(new['validity']['criteria'], old['validity']['criteria']):
                    self.assertEqual((a['id'], a['verdict'], a.get('value'), a.get('threshold')),
                                     (b['id'], b['verdict'], b.get('value'), b.get('threshold')))
                self.assertEqual(new['validity']['verdict'], old['validity']['verdict'])
                self.assertEqual(strip(new, {'prereg', 'validity'}), strip(old, {'prereg', 'validity'}))

    def test_gate_gives_every_sealed_verdict(self):
        for run in SEALED_STATIC:
            with self.subTest(run=os.path.basename(run)):
                name = os.path.basename(run)
                sup = os.path.join(TMP, name + '_sup.json')
                h2m = os.path.join(TMP, name + '_h2m.json')
                if not (os.path.exists(sup) and os.path.exists(h2m)):
                    for script, args, outp in (('s6b1_supplementary.py', [self._cell(run), os.path.join(
                            run, 'analysis', 's6b1_reconstruction.npz')], sup),
                            ('e3_h2_metrics.py', [run, os.path.join(run, 'analysis')], h2m)):
                        if not os.path.exists(outp):
                            sh([sys.executable, '-I', '-B', os.path.join(HERE, script), *args, outp])
                adir = os.path.join(TMP, name + '_adir')
                os.makedirs(adir)
                for f in ('s6b_analysis.json', 's6b_host.json', 'digest_cell.json', 's6b1_reconstruction.json'):
                    shutil.copy(os.path.join(run, 'analysis', f), adir)
                shutil.copy(sup, os.path.join(adir, 's6b1_supplementary.json'))
                shutil.copy(h2m, os.path.join(adir, 'h2_metrics.json'))
                old = json.load(open(os.path.join(run, 'analysis', 'gate.json')))
                cell = self._cell(run)
                man = json.load(open(os.path.join(cell, 'run_manifest_sim.json')))
                env = dict(os.environ, H3_ROOT=TMP, E3_EXPECT_HEAD=man['git']['sha'])
                outd = os.path.join(TMP, name + '_gate')
                p = sh(['bash', os.path.join(HERE, 'h3_analyze.sh'), '--gate-only', cell, adir, adir, old['mode'],
                         outd], env=env)
                self.assertIn(p.returncode, (0, 3), p.stderr[-800:])
                new = json.load(open(os.path.join(outd, 'gate.json')))
                o = {r['id']: r['verdict'] for r in old['rules']}
                n = {r['id']: r['verdict'] for r in new['rules']}
                gate_only_missing = {'R01', 'R02', 'R18'}            # need the run dir: MISSING in --gate-only
                for rid in o:
                    if rid in gate_only_missing:
                        self.assertEqual(n[rid], 'MISSING')
                    else:
                        self.assertEqual(n[rid], o[rid], f'{rid}: {n[rid]} vs sealed {o[rid]}')
                dn = {r['id']: r['detail'] for r in new['rules']}
                do = {r['id']: r['detail'] for r in old['rules']}
                self.assertEqual(dn['R14'], do['R14'])                # same numbers in the clearance rule
                self.assertEqual(dn['R12'], do['R12'])
                self.assertEqual(n['R23'], 'PASS')                   # A1: static -> not needed

    def test_r23_law_check_stops_a_ball_off_its_law(self):
        # one sealed run's outputs, copied; the copies are edited to look like a moving run
        run = SEALED_STATIC[0]
        cell = self._cell(run)
        man = json.load(open(os.path.join(cell, 'run_manifest_sim.json')))
        old = json.load(open(os.path.join(run, 'analysis', 'gate.json')))
        for dev, want in ((0.005, 'STOP'), (0.001, 'PASS')):
            with self.subTest(dev=dev):
                adir = os.path.join(TMP, f'r23_{dev}')
                os.makedirs(adir)
                for f in ('s6b_analysis.json', 's6b_host.json', 'digest_cell.json', 'h2_metrics.json',
                          's6b1_supplementary.json', 's6b1_reconstruction.json'):
                    shutil.copyfile(os.path.join(run, 'analysis', f), os.path.join(adir, f))
                sp = os.path.join(adir, 's6b1_supplementary.json')
                sup = json.load(open(sp)); sup['v_obs_max_mps'] = 0.4464
                json.dump(sup, open(sp, 'w'))
                rp = os.path.join(adir, 's6b1_reconstruction.json')
                rec = json.load(open(rp)); rec['obstacle']['placed_max_dev_from_law_m'] = dev
                json.dump(rec, open(rp, 'w'))
                outd = os.path.join(TMP, f'r23_{dev}_gate')
                env = dict(os.environ, H3_ROOT=TMP, E3_EXPECT_HEAD=man['git']['sha'])
                p = sh(['bash', os.path.join(HERE, 'h3_analyze.sh'), '--gate-only', cell, adir, adir, old['mode'],
                        outd], env=env)
                self.assertIn(p.returncode, (0, 3), p.stderr[-800:])
                g = {r['id']: r['verdict'] for r in json.load(open(os.path.join(outd, 'gate.json')))['rules']}
                self.assertEqual(g['R23'], want)


class D_MovingScenarioNumbers(unittest.TestCase):
    def test_wander_close_bound_and_g_min(self):
        b = H3.speed_bound(WANDER_CLOSE)
        self.assertAlmostEqual(b, 0.4464, places=4)
        self.assertAlmostEqual(H3.g_min(0.10, 0.35, 0.25, b), 0.2991, places=4)
        # the reported 0.2975 used 0.44 = the max SAMPLED over one 40 s episode, below the supremum
        self.assertAlmostEqual(H3.g_min(0.10, 0.35, 0.25, 0.44), 0.2975, places=10)
        t = np.arange(0.0, 40.0, 0.001)
        self.assertAlmostEqual(float(np.linalg.norm(law_velocity(WANDER_CLOSE, t), axis=1).max()), 0.4400, places=4)

    def test_the_scenario_file_matches_the_spec_used_here(self):
        y = os.path.expanduser('~/Git/franka_ros2/franka_simulation_experiments/config/scenarios.yaml')
        txt = open(y).read()
        blk = txt[txt.index('pentagon_wander_close:'):]
        blk = blk[:blk.index('pentagon_multi_static:')]
        self.assertIn('obstacles: [{p0: [0.44, 0.0, 0.45], radius: 0.06, law: wander,', blk)
        self.assertIn('box: [0.08, 0.22, 0.14], period: 7.0,', blk)
        self.assertIn('phase_deg_xyz: [0.0, -90.0, 0.0]}]', blk)


def tearDownModule():
    shutil.rmtree(TMP, ignore_errors=True)


if __name__ == '__main__':
    unittest.main()
