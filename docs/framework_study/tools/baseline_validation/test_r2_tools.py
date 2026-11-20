#!/usr/bin/env python3
"""Unit and integration tests of the R2 performance tools (``r2_default_replay.py``, ``r2_analyze.py``,
``r2_run.sh``, ``r2_batch.sh``).

  python3 -I -X dev test_r2_tools.py -v

The integration tests copy one sealed M3b run (rate 1.0) and one sealed M3 run (rate 0.70) to a temporary
directory and run the R2 per-run gate on them. They check that R2's metrics reproduce the analyses of
record, that the rate gate follows the run's rate, and that the R2 checks reject a run without the
default-arm record. They need the sealed evidence under ``~/franka_baseline_evidence``.
"""
import json
import os
import shutil
import subprocess
import sys
import tempfile
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import m3b_analyze as B  # noqa: E402
import r2_analyze as R  # noqa: E402
import r2_default_replay as D  # noqa: E402

EV = os.path.expanduser('~/franka_baseline_evidence')
M3B = os.path.join(EV, 'refactor_r1_m3b/data')
M3 = os.path.join(EV, 'refactor_r1_m3/attempt1')
E305 = os.path.join(EV, 'refactor_r1_e3/runs/05_s0_multi')


def load(path):
    with open(path) as f:
        return json.load(f)


def read(name):
    with open(os.path.join(HERE, name)) as f:
        return f.read()


def fake_run(ticks=100.0, missed=0, ge50=0, p99=11.0, snap95=11.0, py=0.5, rate=1.0):
    r = {'replay': {'sim': {'ticks_per_sim_s': ticks, 'missed_slots': missed, 'gaps_ge50ms': ge50,
                            'tick_gap_ms': {'p99': p99}},
                    'status': {'snapshot_age_ms': {'p95': snap95}}}}
    r['headroom'] = {'python_thread_cores': py, 'projected_saturation_rate': rate / py}
    return r


GOOD_R2 = {'removed_pair': ['-p', 'executor_mode:=single'], 'params_files': {'a': 'h'},
           'rclpy_init_args': ['r1replay', '--ros-args', '-p', 'use_sim_time:=True', '--params-file', 'a']}


class TestStripOverride(unittest.TestCase):
    def test_removes_exactly_the_appended_pair(self):
        a = ['x', '--ros-args', '--params-file', 'p', '-p', 'executor_mode:=single']
        self.assertEqual(D.strip_override(a), ['x', '--ros-args', '--params-file', 'p'])

    def test_refuses_missing_duplicate_or_leftover_overrides(self):
        for a in (['x'], ['-p', 'executor_mode:=single'] * 2,
                  ['-p', 'executor_mode:=multi', '-p', 'executor_mode:=single'],
                  ['-p', 'executor_mode:=single', 'executor_mode:=multi']):
            with self.assertRaises(ValueError, msg=a):
                D.strip_override(a)

    def test_refuses_a_params_file_that_sets_the_executor(self):
        d = tempfile.mkdtemp(prefix='r2_test_')
        try:
            p = os.path.join(d, 'params')
            with open(p, 'w') as f:
                f.write('/**:\n  ros__parameters:\n    executor_mode: multi\n')
            r = subprocess.run([sys.executable, '-I', os.path.join(HERE, 'r2_default_replay.py'), 'cell',
                                os.path.join(d, 'out'), '1.0', p], capture_output=True, text=True)
            self.assertNotEqual(r.returncode, 0)
            self.assertIn('sets executor_mode', r.stderr)
            self.assertFalse(os.path.exists(os.path.join(d, 'out_r2.json')))
        finally:
            shutil.rmtree(d)

    def test_a_key_is_refused_a_path_mentioning_it_is_not(self):
        self.assertTrue(D.sets_executor(b'/**:\n  ros__parameters:\n    executor_mode: single\n'))
        self.assertTrue(D.sets_executor(b'a: 1\nexecutor_mode : multi\n'))
        self.assertFalse(D.sets_executor(b'    robot_config_path: /c/x__full-executor_mode=multi__gz__s0/f.yaml\n'))
        self.assertFalse(D.sets_executor(b'    qp_rate_hz: 100.0\n'))
        self.assertEqual(D.strip_override(['--params-file', '/w/x__full-executor_mode=multi__/p', '-p',
                                           'executor_mode:=single']), ['--params-file', '/w/x__full-executor_mode=multi__/p'])

    def test_the_wrapper_runs_m1_single_unchanged(self):
        s = read('r2_default_replay.py')
        self.assertIn("UNDERLYING_ARM = 'single'", s)
        self.assertIn("runpy.run_path(M1, run_name='__main__')", s)
        self.assertIn("args += ['-p', f'executor_mode:={MODE}']", read('r1_executor_replay.py'))


class TestRegate(unittest.TestCase):
    def test_gate_is_m1s_at_0_70_and_m3bs_at_1_0(self):
        self.assertIn('GATE = dict(rtf=(0.63, 0.77)', read('m1_analyze.py'))
        self.assertEqual(R.rtf_gate(0.70), (0.63, 0.77))
        self.assertEqual(R.rtf_gate(1.0), B.RTF_GATE)

    def test_only_the_rate_check_changes(self):
        v = {'player_rc_0': True, 'rtf_in_range': False, 'workload_rows_mean': False}
        g = R.regate(v, 0.70, 0.70)
        self.assertNotIn('rtf_in_range', g)
        self.assertTrue(g['rtf_in_r2_range'] and g['player_rc_0'])
        self.assertFalse(g['workload_rows_mean'])
        self.assertFalse(R.regate({}, 1.0, 0.70)['rtf_in_r2_range'])
        self.assertFalse(R.regate({}, 0.70, 1.0)['rtf_in_r2_range'])
        self.assertFalse(R.regate({}, float('nan'), 1.0)['rtf_in_r2_range'])


class TestR2Checks(unittest.TestCase):
    R_OK = {'node_executor_mode': 'single', 'executor_class': 'SingleThreadedExecutor'}

    def test_all_pass(self):
        self.assertTrue(all(R.r2_checks(self.R_OK, GOOD_R2, {'a': 1}, {'a': 1}).values()))

    def test_each_failure_is_caught(self):
        c = R.r2_checks(self.R_OK, None, {'a': 1}, {'a': 1})
        self.assertFalse(c['no_executor_override'])
        bad = dict(GOOD_R2, rclpy_init_args=GOOD_R2['rclpy_init_args'] + ['-p', 'executor_mode:=multi'])
        self.assertFalse(R.r2_checks(self.R_OK, bad, {'a': 1}, {'a': 1})['no_executor_override'])
        m = {'node_executor_mode': 'multi', 'executor_class': 'MultiThreadedExecutor'}
        c = R.r2_checks(m, GOOD_R2, {'a': 1}, {'a': 1})
        self.assertFalse(c['node_resolved_single'] or c['executor_is_single'])
        self.assertFalse(R.r2_checks(self.R_OK, GOOD_R2, {'a': 2}, {'a': 1})['params_equal_m3b_reference'])
        self.assertFalse(R.r2_checks(self.R_OK, GOOD_R2, None, {'a': 1})['params_equal_m3b_reference'])


class TestAcceptance(unittest.TestCase):
    def test_thresholds_are_the_preregistered_ones(self):
        a = dict(ticks_min=99.0, missed_max=10, gap_p99_max_ms=12.0, snap_age_p95_max_ms=11.0)
        self.assertEqual(R.ACCEPT['A_L0'], a)
        self.assertEqual(R.ACCEPT['A_L1'], a)
        self.assertEqual(R.ACCEPT['B_L0'], 'M3B_PASS')
        self.assertEqual(R.ACCEPT['C_L1'], dict(ticks_min=80.4, missed_max=571, gaps_ge50_max=2,
                                                gap_p99_max_ms=26.0, snap_age_p95_max_ms=17.15))
        self.assertEqual((R.MIN_VALID, R.RTF_REL, R.QUANT_TOL), (2, 0.10, 1e-6))
        self.assertEqual(R.CONDITIONS, {'A_L0': (0.70, 'L0'), 'A_L1': (0.70, 'L1'), 'B_L0': (1.0, 'L0'),
                                        'C_L1': (1.0, 'L1')})

    def test_c_band_is_derived_from_m3b_l1(self):
        s = load((os.path.join(M3B, 'analysis/m3b_analysis.json')))['cells']['L1']['stats']
        band = lambda m, sign: s[m]['min' if sign < 0 else 'max'] + sign * 2 * s[m]['sd']  # noqa: E731
        c = R.ACCEPT['C_L1']
        self.assertAlmostEqual(c['ticks_min'], band('ticks_per_sim_s', -1), delta=0.1)
        self.assertAlmostEqual(c['missed_max'], band('missed_slots', +1), delta=1.0)
        self.assertEqual(c['gaps_ge50_max'], int(band('gaps_ge50ms', +1)))
        self.assertAlmostEqual(c['gap_p99_max_ms'], band('gap_p99_ms', +1), delta=1e-6)
        self.assertAlmostEqual(c['snap_age_p95_max_ms'], band('snapshot_age_p95_ms', +1), delta=0.01)

    def test_float_noise_on_a_quantised_metric_passes_a_whole_step_fails(self):
        self.assertTrue(R.run_acceptance(fake_run(p99=12.000000000000455, snap95=11.000000000002785), 'A_L1')['gap_p99'])
        self.assertFalse(R.run_acceptance(fake_run(p99=13.0), 'A_L1')['gap_p99'])
        self.assertFalse(R.run_acceptance(fake_run(snap95=12.0), 'A_L0')['freshness'])
        self.assertTrue(R.run_acceptance(fake_run(ticks=99.0 - 1e-9), 'A_L0')['ticks'])
        self.assertFalse(R.run_acceptance(fake_run(ticks=98.99), 'A_L0')['ticks'])

    def test_condition_verdicts(self):
        good_c = fake_run(ticks=88.0, missed=320, ge50=1, p99=23.0, snap95=16.0)
        self.assertEqual(R.condition_verdict('C_L1', [good_c] * 3)['verdict'], 'REPRODUCED')
        for bad in (dict(ticks=80.3), dict(missed=572), dict(ge50=3), dict(p99=27.0), dict(snap95=18.0)):
            kw = dict(ticks=88.0, missed=320, ge50=1, p99=23.0, snap95=16.0)
            kw.update(bad)
            self.assertEqual(R.condition_verdict('C_L1', [good_c, fake_run(**kw)])['verdict'], 'NOT_REPRODUCED', bad)
        self.assertEqual(R.condition_verdict('A_L0', [fake_run()])['verdict'], 'INCONCLUSIVE')
        self.assertEqual(R.condition_verdict('A_L1', [fake_run(), fake_run(missed=11)])['verdict'], 'NOT_REPRODUCED')
        self.assertEqual(R.condition_verdict('B_L0', [fake_run()] * 3)['verdict'], 'REPRODUCED')
        self.assertEqual(R.condition_verdict('B_L0', [fake_run(ticks=97.0)] * 3)['verdict'], 'NOT_REPRODUCED')


class TestScripts(unittest.TestCase):
    def test_run_script_differs_from_m3b_only_where_documented(self):
        a = read('m3b_run.sh').splitlines()
        b = read('r2_run.sh').splitlines()
        added = [ln for ln in b if ln not in a and not ln.startswith('#')]
        removed = [ln for ln in a if ln not in b and not ln.startswith('#')]
        self.assertEqual(removed, [
            'EV=$1; ID=$2; LOAD=$3; ARM=single', 'W=/home/user/m3b_20261120', 'RATE=1.0',
            "inhibitor_held() { systemd-inhibit --list --no-pager 2>/dev/null | grep -q 'M3B_BATCH' && echo 1 || echo 0; }",
            "grep -q '^inhibitor_held 1$' \"$H\" || fail_pre 'M3B_BATCH sleep inhibitor not held'",
            a[[i for i, ln in enumerate(a) if 'm1_executor_replay.py' in ln][0]],
            'for s in .json _raw.npz _m1.json _m1.npz; do docker cp "franka_ros2:$W/runs/$ID$s" "$EV/runs/" 2>>"$EV/logs/${ID}.log"; done',
            'for s in .json _raw.npz _m1.json _m1.npz; do [ -s "$EV/runs/$ID$s" ] || { echo "HARNESS FAILURE $ID: missing $ID$s"; exit 5; }; done',
            'python3 -I "$HERE/m3b_analyze.py" --check "$EV" "$E305" "$ID" > "$EV/logs/${ID}_check.txt" 2>&1'])
        self.assertEqual(len(added), len(removed) - 1 + 1)        # RATE=1.0 dropped, rate case added
        self.assertIn('EV=$1; ID=$2; RATE=$3; LOAD=$4; ARM=default', added)
        self.assertTrue(any('r2_default_replay.py' in ln and 'launch_params_xa00zx9n_noexec' in ln for ln in added))
        self.assertTrue(any('r2_analyze.py" --check' in ln for ln in added))

    def test_batch_order_and_inhibitor(self):
        s = read('r2_batch.sh')
        self.assertIn('exec systemd-inhibit --what=sleep:idle:handle-lid-switch --mode=block --who=R2_BATCH', s)
        order = s.split('ORDER="')[1].split('"')[0].split()
        conds = [R.condition_of(float(x.split(':')[1]), x.split(':')[2]) for x in order]
        self.assertEqual([x.split(':')[0] for x in order], [f'r2_{i:02d}' for i in range(1, 13)])
        self.assertEqual({c: conds.count(c) for c in set(conds)}, {c: 3 for c in R.CONDITIONS})
        self.assertNotIn('retry', s.split('ORDER=')[1].lower())

    def test_inhibitor_detection_function_sees_a_live_lock(self):
        fn = [ln for ln in read('r2_run.sh').splitlines() if ln.startswith('inhibitor_held()')][0]
        probe = fn + '\ninhibitor_held\n'
        self.assertEqual(subprocess.run(['bash', '-c', probe], capture_output=True, text=True).stdout.strip(), '0')
        out = subprocess.run(['systemd-inhibit', '--what=sleep', '--mode=block', '--who=R2_BATCH',
                              '--why=test_r2_tools', 'bash', '-c', probe], capture_output=True, text=True)
        self.assertEqual(out.stdout.strip(), '1', out.stderr)


def _copy_run(src, rid, d, host_extra='', first_line_fix=None):
    for sub in ('runs', 'host', 'logs'):
        os.makedirs(os.path.join(d, sub), exist_ok=True)
    for s in ('.json', '_raw.npz', '_m1.json', '_m1.npz'):
        shutil.copy(os.path.join(src, 'runs', rid + s), os.path.join(d, 'runs'))
    shutil.copy(os.path.join(src, 'host', rid + '_samples.npz'), os.path.join(d, 'host'))
    shutil.copy(os.path.join(src, 'logs', rid + '.log'), os.path.join(d, 'logs'))
    with open(os.path.join(src, 'host', rid + '.txt')) as f:
        txt = f.read()
    if first_line_fix:
        txt = first_line_fix(txt)
    with open(os.path.join(d, 'host', rid + '.txt'), 'w') as f:
        f.write(txt + host_extra)
    for root, _, files in os.walk(d):
        for f in files:
            os.chmod(os.path.join(root, f), 0o644)


def _check(d, rid):
    return subprocess.run([sys.executable, '-I', os.path.join(HERE, 'r2_analyze.py'), '--check', d, E305, rid],
                          capture_output=True, text=True)


@unittest.skipUnless(os.path.isdir(M3B) and os.path.isdir(M3) and os.path.isdir(E305), 'sealed evidence not available')
class TestCheckOnSealedRuns(unittest.TestCase):
    def setUp(self):
        self.d = tempfile.mkdtemp(prefix='r2_test_')

    def tearDown(self):
        shutil.rmtree(self.d)

    def test_m3b_run_reproduces_m3b_metrics_and_needs_the_default_arm_record(self):
        _copy_run(M3B, 'm3b_01', self.d)
        p = _check(self.d, 'm3b_01')
        self.assertEqual(p.returncode, R.INVALID_RC, p.stdout + p.stderr)       # no _r2.json: not a default-arm run
        r = load((os.path.join(self.d, 'runs', 'm3b_01_r2_check.json')))
        self.assertFalse(r['r2_checks']['no_executor_override'])
        self.assertTrue(r['r2_checks']['params_equal_m3b_reference'] and r['r2_checks']['executor_is_single'])
        ref = [x for x in load((os.path.join(M3B, 'analysis/m3b_analysis.json')))['runs'] if x['id'] == 'm3b_01'][0]
        for m in ('ticks_per_sim_s', 'missed_slots', 'gap_p99_ms', 'snapshot_age_p95_ms', 'busy_cores',
                  'python_thread_cores', 'projected_saturation_rate', 'process_cpu_pct'):
            self.assertAlmostEqual(R.get(r, R.PATHS[m]), R.get(ref, R.PATHS[m]), places=9, msg=m)
        with open(os.path.join(self.d, 'runs', 'm3b_01_r2.json'), 'w') as f:
            json.dump(GOOD_R2, f)
        p = _check(self.d, 'm3b_01')
        self.assertEqual(p.returncode, 0, p.stdout + p.stderr)
        self.assertIn('B_L0', p.stdout)

    def test_m3_run_at_0_70_is_gated_at_0_70(self):
        fix = lambda t: t.replace(' load L1 pre', ' load L1 rate 0.70 pre', 1)  # noqa: E731
        extra = 'boot_minus_mono 10.000\ninhibitor_held 1\npost boot_minus_mono 10.001\npost inhibitor_held 1\n'
        _copy_run(M3, 'm3_05', self.d, extra, fix)                              # m3_05 = single / L1 at 0.70
        with open(os.path.join(self.d, 'runs', 'm3_05_r2.json'), 'w') as f:
            json.dump(GOOD_R2, f)
        p = _check(self.d, 'm3_05')
        r = load((os.path.join(self.d, 'runs', 'm3_05_r2_check.json')))
        self.assertEqual(r['condition'], 'A_L1')
        self.assertTrue(r['replay_validity']['rtf_in_r2_range'])
        ref = [x for x in load((os.path.join(M3, 'analysis/m3_analysis.json')))['runs'] if x['id'] == 'm3_05'][0]
        for m in ('ticks_per_sim_s', 'missed_slots', 'snapshot_age_p95_ms', 'busy_cores', 'python_thread_cores'):
            self.assertAlmostEqual(R.get(r, R.PATHS[m]), R.get(ref, R.PATHS[m]), places=9, msg=m)
        self.assertAlmostEqual(r['headroom']['projected_saturation_rate'], ref['headroom']['projected_saturation_rate'],
                               places=9)
        # M3's parameter namespace (R1 install, -p single) equals M3b's: the reference rule holds across both
        self.assertTrue(r['r2_checks']['params_equal_m3b_reference'], p.stdout)
        self.assertEqual(p.returncode, 0, p.stdout + p.stderr)


if __name__ == '__main__':
    unittest.main()
