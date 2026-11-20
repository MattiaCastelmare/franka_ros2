#!/usr/bin/env python3
"""Unit and integration tests of the M3b tools (``m3b_analyze.py``, ``m3b_run.sh``, ``m3b_batch.sh``).

  python3 -I -X dev test_m3b_tools.py -v

The integration test, ``TestCheckOnASealedM3Run``, copies one sealed M3 run (rate 0.70) to a
temporary directory and runs the M3b per-run gate on it. It checks two things:
* M3b's metrics reproduce M3's analysis of record for that run;
* the M3b rate gate rejects a 0.70 run (exit code 10, not 1).

It needs the sealed evidence under ``~/franka_baseline_evidence``.
"""
import json
import os
import shutil
import subprocess
import sys
import tempfile
import time
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import m3b_analyze as B  # noqa: E402


def read(name):
    with open(os.path.join(HERE, name)) as f:
        return f.read()

EV = os.path.expanduser('~/franka_baseline_evidence')
M3 = os.path.join(EV, 'refactor_r1_m3/attempt1')
E305 = os.path.join(EV, 'refactor_r1_e3/runs/05_s0_multi')


def fake_run(ticks=100.0, missed=0, ge50=0, p99=11.0, snap95=11.0, py=0.5, rate=1.0):
    r = {'replay': {'sim': {'ticks_per_sim_s': ticks, 'missed_slots': missed, 'gaps_ge50ms': ge50,
                            'tick_gap_ms': {'p99': p99}},
                    'status': {'snapshot_age_ms': {'p95': snap95}}}}
    r['headroom'] = {'python_thread_cores': py, 'projected_saturation_rate': rate / py}
    return r


class TestRegate(unittest.TestCase):
    def test_the_rate_gate_is_recentred_and_nothing_else_changes(self):
        v = {'player_rc_0': True, 'rtf_in_range': False, 'workload_rows_p50': True}
        g = B.regate(v, 0.998)
        self.assertNotIn('rtf_in_range', g)
        self.assertTrue(g['rtf_in_m3b_range'] and g['player_rc_0'] and g['workload_rows_p50'])

    def test_rate_gate_edges_and_a_failing_m1_check_survives(self):
        self.assertFalse(B.regate({}, 0.70)['rtf_in_m3b_range'])
        self.assertTrue(B.regate({}, 0.90)['rtf_in_m3b_range'])
        self.assertTrue(B.regate({}, 1.10)['rtf_in_m3b_range'])
        self.assertFalse(B.regate({}, 0.8999)['rtf_in_m3b_range'])
        self.assertFalse(B.regate({}, float('nan'))['rtf_in_m3b_range'])
        self.assertFalse(B.regate({'workload_rows_mean': False}, 1.0)['workload_rows_mean'])


class TestHostChecks(unittest.TestCase):
    def kv(self, pre, post, inh=('1', '1')):
        return {'boot_minus_mono': [pre], 'post_boot_minus_mono': [post],
                'inhibitor_held': [inh[0]], 'post_inhibitor_held': [inh[1]]}

    def test_no_suspend_and_inhibitor(self):
        self.assertEqual(B.host_checks(self.kv('120.000', '120.004')), {'no_suspend': True, 'inhibitor_held': True})

    def test_a_suspend_is_detected(self):
        # the 2026-11-18 suspend: 68 870 s between monotonic and boottime
        self.assertFalse(B.host_checks(self.kv('120.0', '68990.0'))['no_suspend'])

    def test_missing_or_lost_inhibitor_and_missing_fields(self):
        self.assertFalse(B.host_checks(self.kv('1', '1', ('1', '0')))['inhibitor_held'])
        self.assertFalse(B.host_checks(self.kv('1', '1', ('0', '1')))['inhibitor_held'])
        self.assertEqual(B.host_checks({}), {'no_suspend': False, 'inhibitor_held': False})

    def test_the_offset_grows_across_a_suspend_only(self):
        # Live: over 0.2 s of no suspend the offset is constant to well below the tolerance
        f = lambda: time.clock_gettime(time.CLOCK_BOOTTIME) - time.monotonic()  # noqa: E731
        a = f()
        time.sleep(0.2)
        self.assertLess(abs(f() - a), 0.01)


class TestAcceptance(unittest.TestCase):
    def test_pass_with_margin(self):
        v = B.cell_verdict([fake_run(), fake_run(99.9), fake_run(100.0)])
        self.assertEqual(v['verdict'], 'PASS')
        self.assertTrue(v['margin'])
        self.assertFalse(v['saturation_evidence'])

    def test_pass_without_margin_and_with_saturation_evidence(self):
        v = B.cell_verdict([fake_run(py=0.97)] * 3)
        self.assertEqual(v['verdict'], 'PASS')
        self.assertFalse(v['margin'])
        self.assertTrue(v['saturation_evidence'])

    def test_marginal_when_rate_holds_but_a_secondary_criterion_fails(self):
        for bad in (dict(missed=29), dict(ge50=1), dict(p99=15.1), dict(snap95=20.5), dict(ticks=96.0)):
            v = B.cell_verdict([fake_run(**bad), fake_run(), fake_run()])
            self.assertEqual(v['verdict'], 'MARGINAL', bad)

    def test_mean_below_99_with_every_run_above_95_is_marginal(self):
        self.assertEqual(B.cell_verdict([fake_run(97.0)] * 3)['verdict'], 'MARGINAL')

    def test_fail_when_one_run_is_below_95(self):
        self.assertEqual(B.cell_verdict([fake_run(94.9), fake_run(), fake_run()])['verdict'], 'FAIL')

    def test_inconclusive_with_fewer_than_two_valid_runs(self):
        self.assertEqual(B.cell_verdict([fake_run()])['verdict'], 'INCONCLUSIVE')
        self.assertEqual(B.cell_verdict([])['verdict'], 'INCONCLUSIVE')

    def test_thresholds_are_the_preregistered_ones(self):
        self.assertEqual(B.ACCEPT, dict(ticks_min_run=95.0, ticks_mean=99.0, missed_max=28, gaps_ge50_max=0,
                                        gap_p99_max_ms=15.0, snap_age_p95_max_ms=20.0))
        self.assertEqual((B.RATE, B.RTF_GATE, B.MARGIN_MIN, B.SAT_PY_CORES, B.MIN_VALID),
                         (1.0, (0.90, 1.10), 1.10, 0.95, 2))


class TestScripts(unittest.TestCase):
    def test_run_script_differs_from_m3_only_where_documented(self):
        a = read('m3_run.sh').splitlines()
        b = read('m3b_run.sh').splitlines()
        added = [ln for ln in b if ln not in a]
        removed = [ln for ln in a if ln not in b]
        code_added = [ln for ln in added if not ln.startswith('#')]
        self.assertIn('RATE=1.0', code_added)
        self.assertIn('W=/home/user/m3b_20261120', code_added)
        self.assertIn('EV=$1; ID=$2; LOAD=$3; ARM=single', code_added)
        self.assertTrue(any('m3b_analyze.py" --check' in ln for ln in code_added))
        self.assertTrue(any('boot_minus_mono' in ln for ln in code_added))
        self.assertTrue(any('inhibitor_held' in ln for ln in code_added))
        code_removed = [ln for ln in removed if not ln.startswith('#')]
        # nothing else of M3's run script is dropped: the load, sampler, pre-checks and cleanup are kept
        for ln in code_removed:
            self.assertTrue(any(k in ln for k in ('RATE=0.70', 'm3_20261118', 'ARM=$3', 'm1_analyze.py',
                                                  "['runs'][0]['valid']", 'replay validity gate', 'CAP=330',
                                                  'fi', 'echo "run $ID arm $ARM load $LOAD pre')), ln)

    def test_batch_runs_under_the_inhibitor_in_the_sealed_order(self):
        s = read('m3b_batch.sh')
        self.assertIn('exec systemd-inhibit --what=sleep:idle:handle-lid-switch --mode=block --who=M3B_BATCH', s)
        self.assertIn('ORDER="m3b_01:L0 m3b_02:L1 m3b_03:L1 m3b_04:L0 m3b_05:L0 m3b_06:L1"', s)
        self.assertNotIn('retry', s.split('ORDER=')[1].lower())

    def test_inhibitor_detection_function_sees_a_live_lock(self):
        fn = [ln for ln in read('m3b_run.sh').splitlines() if ln.startswith('inhibitor_held()')][0]
        probe = fn + '\ninhibitor_held\n'
        self.assertEqual(subprocess.run(['bash', '-c', probe], capture_output=True, text=True).stdout.strip(), '0')
        out = subprocess.run(['systemd-inhibit', '--what=sleep', '--mode=block', '--who=M3B_BATCH',
                              '--why=test_m3b_tools', 'bash', '-c', probe], capture_output=True, text=True)
        self.assertEqual(out.stdout.strip(), '1', out.stderr)


@unittest.skipUnless(os.path.isdir(M3) and os.path.isdir(E305), 'sealed M3 / E3-05 evidence not available')
class TestCheckOnASealedM3Run(unittest.TestCase):
    """m3_05 = single / L1 at rate 0.70 (M3 attempt 1)."""

    def setUp(self):
        self.d = tempfile.mkdtemp(prefix='m3b_test_')
        for sub in ('runs', 'host', 'logs'):
            os.makedirs(os.path.join(self.d, sub))
        for s in ('.json', '_raw.npz', '_m1.json', '_m1.npz'):
            shutil.copy(os.path.join(M3, 'runs', 'm3_05' + s), os.path.join(self.d, 'runs'))
        shutil.copy(os.path.join(M3, 'host', 'm3_05_samples.npz'), os.path.join(self.d, 'host'))
        shutil.copy(os.path.join(M3, 'logs', 'm3_05.log'), os.path.join(self.d, 'logs'))
        with open(os.path.join(M3, 'host', 'm3_05.txt')) as f:
            txt = f.read()
        txt += 'boot_minus_mono 10.000\ninhibitor_held 1\npost boot_minus_mono 10.001\npost inhibitor_held 1\n'
        with open(os.path.join(self.d, 'host', 'm3_05.txt'), 'w') as f:
            f.write(txt)
        for root, dirs, files in os.walk(self.d):
            for f in files:
                os.chmod(os.path.join(root, f), 0o644)

    def tearDown(self):
        shutil.rmtree(self.d)

    def test_metrics_match_m3_and_the_rate_gate_rejects_0_70(self):
        p = subprocess.run([sys.executable, '-I', os.path.join(HERE, 'm3b_analyze.py'), '--check', self.d, E305,
                            'm3_05'], capture_output=True, text=True)
        self.assertEqual(p.returncode, B.INVALID_RC, p.stdout + p.stderr)
        with open(os.path.join(self.d, 'runs', 'm3_05_m3b_check.json')) as f:
            r = json.load(f)
        self.assertEqual(r['load'], 'L1')
        self.assertFalse(r['replay_validity']['rtf_in_m3b_range'])
        others = {k: v for k, v in r['replay_validity'].items() if k != 'rtf_in_m3b_range'}
        self.assertTrue(all(others.values()), others)
        self.assertTrue(all(r['load_gate'].values()) and all(r['host_checks'].values()))
        with open(os.path.join(M3, 'analysis/m3_analysis.json')) as f:
            ref = [x for x in json.load(f)['runs'] if x['id'] == 'm3_05'][0]
        for m in ('ticks_per_sim_s', 'missed_slots', 'rebuild_wall_p50', 'solve_wall_p50', 'snapshot_age_p95_ms',
                  'busy_cores', 'python_thread_cores', 'process_cpu_pct'):
            self.assertAlmostEqual(B.get(r, B.PATHS[m]), B.get(ref, B.PATHS[m]), places=9, msg=m)
        # headroom is recomputed at M3b's rate (1.0) instead of 0.70
        self.assertAlmostEqual(r['headroom']['projected_saturation_rate'],
                               ref['headroom']['projected_saturation_rate'] / 0.70, places=9)


if __name__ == '__main__':
    unittest.main()
