#!/usr/bin/env python3
"""Tests for the M3 tools (m3_analyze.py, m3_host_sampler.py, m3_load.sh). Written before any M3 data.

  python3 -I docs/framework_study/tools/baseline_validation/test_m3_tools.py -v

Part 1 checks the analysis functions on synthetic data with known answers. Part 2 checks the load
workers (start, count, stop, timeout cap) and the sampler (fields, stop file, clock) in a few seconds,
with two workers. Standard library unittest + numpy; no ROS.
"""
import importlib.util
import os
import subprocess
import sys
import tempfile
import time
import unittest

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
spec = importlib.util.spec_from_file_location('m3_analyze', os.path.join(HERE, 'm3_analyze.py'))
A = importlib.util.module_from_spec(spec)
spec.loader.exec_module(A)


def stat_rows(busy_frac, ticks=1000, ncpu=28):
    """Synthetic /proc/stat deltas: each CPU busy for the given fraction of `ticks` jiffies."""
    busy_frac = np.broadcast_to(np.asarray(busy_frac, float), (ncpu,))
    s = np.zeros((ncpu, 8))
    s[:, 0] = busy_frac * ticks            # user
    s[:, 3] = (1 - busy_frac) * ticks      # idle
    return s


class TestHostMetrics(unittest.TestCase):
    def test_busy_cores_counts_idle_and_iowait_as_idle(self):
        a = np.zeros((4, 8))
        b = np.zeros((4, 8))
        b[:, 0] = [100, 50, 0, 0]
        b[:, 3] = [0, 50, 100, 50]
        b[:, 4] = [0, 0, 0, 50]            # iowait counts as idle
        busy, util = A.busy_cores(a, b)
        self.assertAlmostEqual(busy, 1.5)
        np.testing.assert_allclose(util, [1.0, 0.5, 0.0, 0.0])

    def test_twelve_busy_cores_of_28(self):
        frac = np.r_[np.ones(12), np.zeros(16)]
        busy, _ = A.busy_cores(np.zeros((28, 8)), stat_rows(frac))
        self.assertAlmostEqual(busy, 12.0)

    def test_window_indices_bracket_the_window(self):
        t = np.arange(0, 10, 0.5)
        self.assertEqual(A.window_indices(t, 2.2, 7.1), (4, 15))      # 2.0 .. 7.5
        self.assertEqual(A.window_indices(t, 2.0, 7.0), (4, 14))      # exact hits included
        self.assertIsNone(A.window_indices(t, -1.0, 5.0))             # not covered at the start
        self.assertIsNone(A.window_indices(t, 1.0, 12.0))             # not covered at the end

    def test_host_metrics_uses_only_window_samples_and_splits_p_and_e_cores(self):
        n = 21
        t = np.arange(n) * 0.5
        stat = np.cumsum(np.stack([stat_rows(np.r_[np.ones(6), np.zeros(22)], ticks=50)] * n), 0)
        f = np.zeros((n, 28))
        f[:, :16] = 2_100_000
        f[:, 16:] = 1_900_000
        f[:4] = 5_000_000                                              # outside the window: ignored
        k = np.full(n, 60000)
        k[-1] = 70000
        H = A.host_metrics(dict(t=t, stat=stat, freq_khz=f, pkg_temp_mC=k), 2.0, 8.0)
        self.assertTrue(H['covered'])
        self.assertAlmostEqual(H['busy_cores'], 6.0)
        self.assertAlmostEqual(H['freq_mhz_p_cores']['p50'], 2100.0)
        self.assertAlmostEqual(H['freq_mhz_e_cores']['p50'], 1900.0)
        self.assertEqual(H['pkg_temp_c_window_max'], 60.0)
        self.assertEqual(H['pkg_temp_c_run_max'], 70.0)

    def test_host_metrics_reports_missing_coverage(self):
        self.assertEqual(A.host_metrics(dict(t=np.array([5.0, 6.0])), 2.0, 8.0), {'covered': False})


class TestLoadGate(unittest.TestCase):
    def test_idle_gate(self):
        self.assertTrue(all(A.load_gate('L0', 1.2, None, None).values()))
        self.assertFalse(A.load_gate('L0', 3.5, None, None)['host_busy_in_range'])
        self.assertFalse(A.load_gate('L0', None, None, None)['host_busy_in_range'])

    def test_load_gate_needs_range_and_all_workers(self):
        self.assertTrue(all(A.load_gate('L1', 12.8, 12, 12).values()))
        self.assertFalse(A.load_gate('L1', 10.9, 12, 12)['host_busy_in_range'])
        self.assertFalse(A.load_gate('L1', 15.5, 12, 12)['host_busy_in_range'])
        self.assertFalse(A.load_gate('L1', 12.8, 11, 12)['workers_alive_start'])
        self.assertFalse(A.load_gate('L1', 12.8, 12, 0)['workers_alive_end'])


class TestStatistics(unittest.TestCase):
    def test_separated_needs_distance_and_no_overlap(self):
        self.assertTrue(A.separated([1.0, 1.1, 0.9], [2.0, 2.1, 1.9]))
        self.assertFalse(A.separated([1.0, 1.1, 0.9], [1.05, 1.15, 0.95]))      # overlapping ranges
        self.assertFalse(A.separated([1.0, 2.0, 3.0], [3.5, 4.0, 4.5]))         # |d| < 2 sd
        self.assertFalse(A.separated([1.0], [5.0]))                             # n < 2

    def test_diff_growth(self):
        g = A.diff_growth([1, 1.1, 0.9], [2, 2.1, 1.9], [1, 1.1, 0.9], [4, 4.1, 3.9])
        self.assertAlmostEqual(g['diff_L0'], 1.0)
        self.assertAlmostEqual(g['diff_L1'], 3.0)
        self.assertTrue(g['grows'])
        g = A.diff_growth([1, 1.1, 0.9], [2, 2.1, 1.9], [1, 1.1, 0.9], [2.05, 2.15, 1.95])
        self.assertFalse(g['grows'])

    def test_headroom(self):
        h = A.headroom(0.7, 0.5)
        self.assertAlmostEqual(h['projected_saturation_rate'], 1.4)
        self.assertAlmostEqual(h['idle_fraction'], 0.5)
        self.assertIsNone(A.headroom(0.7, 0.0)['projected_saturation_rate'])

    def test_python_thread_cpu_counts_spin_and_pool_only(self):
        th = {'executor-spin[1]': 27.0, 'ThreadPoolExecutor-0_0[2]': 13.0, 'ThreadPoolExecutor-0_1[3]': 13.0,
              'python3[4]': 3.0, 'monitor[5]': 0.2}
        self.assertAlmostEqual(A.python_thread_cpu(th), 0.53)
        self.assertEqual(A.python_thread_cpu(None), 0.0)

    def test_cpu_budget_arithmetic(self):
        W0, W1 = 10.0, 20.0
        Z = {}
        for k, cpu in (('rebuild', 0.002), ('qp', 0.001), ('solve', 0.0004), ('js', 0.0001), ('nom', 0.0), ('dist', 0.0)):
            x = np.zeros((200, 6))
            x[:, 5] = np.linspace(5, 25, 200)                  # half inside the window
            x[:, 3] = cpu
            Z[k] = x
        B = A.cpu_budget(Z, W0, W1, 0.7, 70.0, 28.0)
        inside = ((Z['rebuild'][:, 5] >= W0) & (Z['rebuild'][:, 5] <= W1)).sum()
        self.assertAlmostEqual(B['rebuild'], 1e3 * 0.002 * inside / 10.0)
        self.assertAlmostEqual(B['process'], 1000.0)          # 70 % of a core at RTF 0.7
        self.assertAlmostEqual(B['executor_loop'], 400.0)
        self.assertAlmostEqual(B['remainder'], B['process'] - B['callbacks'] - B['executor_loop'])


class TestStaleEvents(unittest.TestCase):
    LINES = ['[WARN] [100.5] [cbf_safety_filter]: qddot_nom stale → braking',
             '[WARN] [200.5] [cbf_safety_filter]: qddot_nom stale → braking',
             '[WARN] [201.0] [cbf_safety_filter]: joint state stale 12.5 ms → braking',
             '[INFO] [202.0] [cbf_safety_filter]: stale but INFO is ignored',
             '[WARN] [203.0] [other_node]: stale elsewhere is ignored',
             '[WARN] [204.0] [cbf_safety_filter]: distance rate re-derived 30.3 Hz']
    def test_counts_only_node_warnings_inside_the_window(self):
        s = A.stale_events(self.LINES, 150.0, 250.0)
        self.assertEqual(s['n_in_window'], 2)
        self.assertEqual(sum(s['outside_window'].values()), 1)
        self.assertIn('joint state stale # ms → braking', s['in_window'])


def cells_fixture(single_L1=(100, 100, 100), mt3_L1=(80, 85, 90)):
    def arm(vals_by_metric):
        return {m: dict(values=list(v), mean=float(np.mean(v)), sd=float(np.std(v, ddof=1)),
                        min=min(v), max=max(v)) for m, v in vals_by_metric.items()}
    base = {m: (1.0, 1.0, 1.0) for m in ('rebuild_wall_p50', 'solve_wall_p50', 'tick_wait_p95', 'process_cpu_pct',
                                         'missed_slots', 'late_gt15ms', 'snapshot_age_p50_ms', 'rebuild_cpu_p50',
                                         'solve_cpu_p50', 'tick_cpu_p50', 'executor_loop_cpu_pct',
                                         'js_processed_frac', 'clock_processed_frac')}
    C = {}
    for ld in ('L0', 'L1'):
        C[ld] = {'per_arm': {}}
        for a in ('single', 'mt1', 'mt3'):
            m = dict(base)
            m['ticks_per_sim_s'] = (100.0, 100.0, 100.0)
            if ld == 'L1':
                m['rebuild_wall_p50'] = {'single': (2, 2.1, 1.9), 'mt1': (4, 4.1, 3.9), 'mt3': (8, 8.1, 7.9)}[a]
                m['ticks_per_sim_s'] = {'single': single_L1, 'mt1': (95, 96, 97), 'mt3': mt3_L1}[a]
            C[ld]['per_arm'][a] = arm(m)
    return C


class TestHypothesisRules(unittest.TestCase):
    def test_rules_on_a_fixture_where_multi_degrades(self):
        H = A.hypotheses(cells_fixture())
        self.assertTrue(H['H1_single_holds_rate_under_load']['supported'])
        self.assertTrue(H['H2_loss_order_under_load']['mt3_loses_more_than_mt1']['separated'])
        self.assertTrue(H['H2_loss_order_under_load']['mt1_loses_more_than_single']['by_mean'])
        self.assertTrue(H['H3_gap_grows_under_load']['rebuild_wall_p50']['mt3_vs_single']['grows'])
        self.assertTrue(H['H4_e3_degradation_reproduced']['reproduced'])

    def test_rules_on_a_fixture_where_nothing_degrades(self):
        H = A.hypotheses(cells_fixture(mt3_L1=(100, 99.9, 100)))
        self.assertFalse(H['H4_e3_degradation_reproduced']['reproduced'])
        H = A.hypotheses(cells_fixture(single_L1=(99.5, 98.0, 94.0)))
        self.assertFalse(H['H1_single_holds_rate_under_load']['supported'])   # one run below R15


class TestThermalGuardRule(unittest.TestCase):
    """Addendum 01: the guard needs n consecutive samples at or above the threshold."""
    def setUp(self):
        spec = importlib.util.spec_from_file_location('m3_host_sampler', os.path.join(HERE, 'm3_host_sampler.py'))
        self.S = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(self.S)

    def test_the_abort_01_spike_does_not_fire(self):
        self.assertFalse(self.S.guard_fires([60000, 73000, 98000, 60000, 60000], 95000, 4))
        for i in range(1, 6):
            self.assertFalse(self.S.guard_fires([60000, 73000, 98000, 60000, 60000][:i], 95000, 4))

    def test_sustained_heat_fires(self):
        self.assertTrue(self.S.guard_fires([60000, 95000, 96000, 97000, 99000], 95000, 4))
        self.assertFalse(self.S.guard_fires([95000, 96000, 94999, 97000], 95000, 4))
        self.assertFalse(self.S.guard_fires([99000, 99000, 99000], 95000, 4))   # fewer than n samples


class TestLoadAndSampler(unittest.TestCase):
    LOAD = os.path.join(HERE, 'm3_load.sh')

    def count(self):
        return int(subprocess.run([self.LOAD, 'count'], capture_output=True, text=True).stdout.strip())

    def setUp(self):
        if self.count() != 0:
            self.skipTest('M3 load workers already running: refusing to interfere')

    def tearDown(self):
        subprocess.run([self.LOAD, 'stop'], capture_output=True)

    def test_start_count_stop(self):
        r = subprocess.run([self.LOAD, 'start', '2', '30'], capture_output=True, text=True)
        self.assertIn('started 2', r.stdout)
        self.assertEqual(self.count(), 2)
        r = subprocess.run([self.LOAD, 'stop'], capture_output=True, text=True)
        self.assertEqual(r.returncode, 0, r.stdout)
        self.assertEqual(self.count(), 0)
        left = subprocess.run(['pgrep', '-fc', 'M3_LOAD_WORKER'], capture_output=True, text=True).stdout.strip()
        self.assertEqual(left, '0')                       # timeout wrappers gone too

    def test_timeout_cap_kills_workers_without_stop(self):
        subprocess.run([self.LOAD, 'start', '2', '2'], capture_output=True, text=True)
        self.assertEqual(self.count(), 2)
        time.sleep(3.0)
        self.assertEqual(self.count(), 0)

    def test_sampler_records_fields_and_stops_on_the_stop_file(self):
        with tempfile.TemporaryDirectory() as d:
            out, stop = os.path.join(d, 's.npz'), os.path.join(d, 'stop')
            t_before = time.monotonic()
            p = subprocess.Popen([sys.executable, '-I', os.path.join(HERE, 'm3_host_sampler.py'), out, stop, '0.2'])
            time.sleep(1.2)
            open(stop, 'w').close()
            self.assertEqual(p.wait(timeout=5), 0)
            with np.load(out) as z:
                self.assertGreaterEqual(len(z['t']), 4)
                self.assertEqual(z['stat'].shape[1:], (os.cpu_count(), 8))
                self.assertEqual(z['freq_khz'].shape[1], os.cpu_count())
                self.assertTrue((z['pkg_temp_mC'] > 0).all())
                self.assertGreaterEqual(z['t'][0], t_before)               # monotonic clock
                self.assertLess(abs(z['epoch'][0] - time.time()), 10.0)    # epoch clock
                self.assertFalse(bool(z['thermal_aborted']))
            self.assertFalse(os.path.exists(out + '.thermal_abort'))

    def test_thermal_guard_kills_workers(self):
        subprocess.run([self.LOAD, 'start', '2', '30'], capture_output=True, text=True)
        with tempfile.TemporaryDirectory() as d:
            out, stop = os.path.join(d, 's.npz'), os.path.join(d, 'stop')
            p = subprocess.Popen([sys.executable, '-I', os.path.join(HERE, 'm3_host_sampler.py'), out, stop,
                                  '0.2', '1'])            # threshold 1 m°C: fires after 4 samples (0.8 s)
            time.sleep(2.0)
            open(stop, 'w').close()
            p.wait(timeout=5)
            self.assertTrue(os.path.exists(out + '.thermal_abort'))
            self.assertEqual(self.count(), 0)


if __name__ == '__main__':
    unittest.main()
