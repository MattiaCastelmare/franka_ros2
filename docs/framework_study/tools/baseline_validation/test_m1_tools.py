#!/usr/bin/env python3
"""Tests for the M1 analysis tooling (``m1_analyze.py``) and the M1 evidence.

Standard-library ``unittest`` with numpy; nothing from ROS is needed. Run from the repository root:

  python3 -I docs/framework_study/tools/baseline_validation/test_m1_tools.py -v

Environment variables:
* ``M1_ANALYZE``: the analysis script under test (default: the one beside this file).
* ``M1_EVIDENCE``: the M1 evidence root (default ``~/franka_baseline_evidence/refactor_r1_m1``).
* ``M1_E3_05``: the sealed E3-05 run (default ``~/franka_baseline_evidence/refactor_r1_e3/runs/05_s0_multi``).

**Part 1, synthetic: the metric semantics.** It runs ``m1_analyze.py`` as a black box on synthetic
runs whose expected values are computed independently here with plain loops, and it never imports
the script. The cases are:
- missed slots, late ticks, gaps;
- P(late | new snapshot);
- late fraction by row band;
- workload equivalence;
- the validity gates;
- JSON-serialisable output (the Addendum 01 defect);
- the window-boundary case of ``missed_slots`` (defect D-M1-2, Addendum 03);
- the pre-registered mt1-position rule.

**Part 2, evidence: the recorded experiment.** It checks:
- the checksums;
- the arms and executor configurations;
- identical parameters and inputs;
- the sealed run order;
- the pre-registered validity of every run;
- that the reported analysis is reproducible from the immutable data.

It is skipped, and reported as skipped, when the evidence is absent.
"""
import hashlib
import json
import os
import sqlite3
import subprocess
import sys
import tempfile
import unittest

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ANALYZE = os.environ.get('M1_ANALYZE', os.path.join(HERE, 'm1_analyze.py'))
EVID = os.environ.get('M1_EVIDENCE', os.path.expanduser('~/franka_baseline_evidence/refactor_r1_m1'))
E305 = os.environ.get('M1_E3_05', os.path.expanduser('~/franka_baseline_evidence/refactor_r1_e3/runs/05_s0_multi'))
ORDER = [('m1_01', 'single'), ('m1_02', 'mt1'), ('m1_03', 'mt3'), ('m1_04', 'mt1'), ('m1_05', 'mt3'),
         ('m1_06', 'single'), ('m1_07', 'mt3'), ('m1_08', 'single'), ('m1_09', 'mt1')]
NF = 41                                          # cbf_status fields (STATUS_FIELDS)
T0 = 10.0                                        # synthetic t0 -> window [12, 40]
W0, W1 = T0 + 2.0, T0 + 30.0
RATE = 0.7


def jdump(obj, path):
    with open(path, 'w') as f:
        json.dump(obj, f)


def jload(path):
    with open(path) as f:
        return json.load(f)


# ─────────────────────────────── synthetic fixtures ───────────────────────────────

def make_e3(root, ref_rows=26.0):
    """A minimal E3-05-shaped directory: t0, a reference cbf_status series, a bag with counts."""
    os.makedirs(os.path.join(root, 'analysis'))
    jdump({'placement': {'t0': T0}}, os.path.join(root, 'analysis/h2_metrics.json'))
    tS = np.round(np.arange(0.0, 45.0, 0.01), 9)
    S = np.full((len(tS), NF), np.nan)
    S[:, 0] = ref_rows
    S[:, 4] = 0.3
    S[:, 13] = 1.0
    S[:, 18] = 0.01
    S[:, 37] = np.repeat(np.arange(len(tS) // 2 + 1), 2)[:len(tS)] * 1e-3 + 5.0
    np.savez(os.path.join(root, 'analysis/s6b_series.npz'), tS=tS, S=S)
    bag = os.path.join(root, 'run/batch/cells/cell/bag')
    os.makedirs(bag)
    con = sqlite3.connect(os.path.join(bag, 'bag_0.db3'))
    con.execute('create table topics (id integer, name text)')
    con.execute('create table messages (topic_id integer, timestamp integer)')
    rates = {'/clock': 1000, '/NS_1/franka/joint_states': 1000, '/NS_1/qddot_nom': 100, '/cbf/per_link_distances': 30}
    for i, (name, hz) in enumerate(rates.items()):
        con.execute('insert into topics values (?, ?)', (i, name))
        ts = (np.arange(0, int(45 * hz)) * (1e9 / hz)).astype(np.int64)
        con.executemany('insert into messages values (?, ?)', [(i, int(t)) for t in ts])
    con.commit()
    con.close()
    return rates


def tick_indices(drop=()):
    """Tick k is due at W0 + 0.01 k, k = 0 … 2799 (the 2800 slots of the window), minus `drop`."""
    return [k for k in range(2800) if k not in set(drop)]


def make_run(d, rid, arm, ks, *, rows_of=lambda k: (20.0, 26.0, 35.0)[k % 3], snap_of=lambda k: k // 2,
             lag=0.0005, rtf=RATE, worker=None, rebuild_ms=2.0, solve_ms=0.4, tick_wait_ms=0.0,
             executor=None, t_of=None, js_n=None):
    """Write the four per-run files m1_analyze.py reads. Ticks enter `lag` after their due time."""
    t_in = np.array([t_of(k) if t_of else W0 + 0.01 * k + lag for k in ks])
    wall = t_in / rtf
    n = len(t_in)
    qp = np.c_[t_in + 1e-3, wall, np.full(n, 1e-3 + tick_wait_ms * 1e-3), np.full(n, 1e-3), np.zeros(n), t_in]
    reb_t = W0 + 0.02 * np.arange(1400) + 1e-4
    rebuild = np.c_[reb_t + 2e-3, reb_t / rtf, np.full(1400, rebuild_ms * 1e-3), np.full(1400, rebuild_ms * 1e-3),
                    np.zeros(1400), reb_t]
    solve = np.c_[t_in + 5e-4, wall, np.full(n, solve_ms * 1e-3), np.full(n, solve_ms * 1e-3), np.zeros(n), t_in]
    js_n = 28000 if js_n is None else js_n
    js_t = W0 + np.arange(js_n) * (28.0 / js_n)
    js = np.c_[js_t, js_t / rtf, np.full(js_n, 3e-5), np.full(js_n, 3e-5), np.zeros(js_n), js_t]
    nom_t = W0 + 0.01 * np.arange(2800)
    nom = np.c_[nom_t, nom_t / rtf, np.full(2800, 2e-5), np.full(2800, 2e-5), np.zeros(2800), nom_t]
    dist_t = W0 + np.arange(840) / 30.0
    dist = np.c_[dist_t, dist_t / rtf, np.full(840, 1e-4), np.full(840, 1e-4), np.zeros(840), dist_t]
    sid = np.array([snap_of(k) for k in ks], float)
    snap_node = W0 + 0.02 * sid
    tick_in = np.c_[wall, t_in, t_in - 2e-3, t_in - 2e-3, t_in - 5e-3, t_in - 0.02, t_in - 0.05,
                    snap_node / rtf, snap_node]
    clk_sim = np.arange(0, 45000) * 1e-3
    clock = np.c_[clk_sim / rtf, clk_sim]
    np.savez(os.path.join(d, rid + '_raw.npz'), js=js, nom=nom, dist=dist, rebuild=rebuild, qp=qp, solve=solve,
             tick_in=tick_in, pub=np.zeros((0, 9)), clock=clock)
    st = np.full((n, 2 + NF), np.nan)
    st[:, 0] = wall
    st[:, 1] = t_in + 1e-3
    D = st[:, 2:]
    D[:, 0] = [rows_of(k) for k in ks]
    D[:, 2] = 0.0
    D[:, 3] = 0.0
    D[:, 4] = 0.2
    D[:, 11] = 0.0
    D[:, 13] = solve_ms
    D[:, 18] = t_in - snap_node
    D[:, 37] = rebuild_ms + 1e-3 * sid
    np.savez(os.path.join(d, rid + '_m1.npz'), status=st)
    want = {'single': ('SingleThreadedExecutor', None), 'mt1': ('MultiThreadedExecutor', 1),
            'mt3': ('MultiThreadedExecutor', 3)}[arm]
    jdump({'arm': arm, 'executor_class': executor or want[0],
           'worker_threads': want[1] if worker is None else worker,
           'node_executor_mode': 'single' if arm == 'single' else 'multi'},
          os.path.join(d, rid + '_m1.json'))
    jdump({'rate': RATE, 'player_rc': 0, 'window': {'wall_s': 30 / rtf, 'sim_s': 30.0, 'rtf': rtf},
           'process_cpu_pct_of_core': 50.0, 'executor_loop_cpu_pct': 15.0,
           'thread_cpu_pct_of_core': {'executor-spin[1]': 45.0}}, os.path.join(d, rid + '.json'))
    return t_in


def run_analyze(runs, e3, ids, script=ANALYZE):
    out = os.path.join(runs, 'out.json')
    p = subprocess.run([sys.executable, '-I', script, runs, e3, out, *ids], capture_output=True, text=True)
    return p, (jload(out) if p.returncode == 0 and os.path.exists(out) else None)


# independent reference computations (plain loops, no shared code with m1_analyze.py)

def ref_gaps(t):
    return [b - a for a, b in zip(t[:-1], t[1:])]


def ref_missed(t):
    due = range(int(round(W0 * 100)), int(round(W1 * 100)))       # due points in [W0, W1)
    have = {int((x + 5e-4) // 0.01) for x in t}
    return sum(1 for k in due if k not in have)


class TestSyntheticMetrics(unittest.TestCase):
    """Part 1: the metric semantics on synthetic runs with known answers."""

    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory()
        cls.e3 = os.path.join(cls.tmp.name, 'e3')
        cls.rates = make_e3(cls.e3)
        cls.runs = os.path.join(cls.tmp.name, 'runs')
        os.makedirs(cls.runs)
        # drops: one slot (20 ms gap), two slots (30 ms), six slots (70 ms)
        cls.drop = (100, 500, 501, 1000, 1001, 1002, 1003, 1004, 1005)
        cls.ks = tick_indices(cls.drop)
        cls.t = make_run(cls.runs, 'syn', 'mt1', cls.ks)
        cls.p, cls.out = run_analyze(cls.runs, cls.e3, ['syn'])

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def test_00_runs_and_writes_json(self):
        self.assertEqual(self.p.returncode, 0, self.p.stderr[-2000:])
        self.assertIsNotNone(self.out)

    def test_validity_values_are_python_bools(self):
        """Addendum 01 regression: validity entries must be JSON booleans."""
        r = self.out['runs'][0]
        self.assertTrue(all(isinstance(v, bool) for v in r['validity'].values()))
        self.assertIsInstance(r['valid'], bool)

    def test_tick_counts_gaps_and_lateness(self):
        s = self.out['runs'][0]['sim']
        g = ref_gaps(list(self.t))
        self.assertEqual(s['ticks'], len(self.ks))
        self.assertAlmostEqual(s['ticks_per_sim_s'], len(self.ks) / 28.0, places=9)
        self.assertEqual(s['late_gt15ms'], sum(1 for x in g if x > 0.015))
        self.assertEqual(s['late_gt15ms'], 3)
        self.assertEqual(s['gaps_ge50ms'], sum(1 for x in g if x >= 0.05))
        self.assertEqual(s['gaps_ge50ms'], 1)
        self.assertAlmostEqual(s['tick_gap_ms']['max'], 70.0, places=6)
        self.assertAlmostEqual(s['tick_gap_ms']['p50'], 10.0, places=6)

    def test_missed_slots(self):
        self.assertEqual(ref_missed(list(self.t)), len(self.drop))
        self.assertEqual(self.out['runs'][0]['sim']['missed_slots'], len(self.drop))

    def test_late_given_new_snapshot(self):
        lv = self.out['runs'][0]['late_vs_rebuild']
        sid = [k // 2 for k in self.ks]
        new = [True] + [a != b for a, b in zip(sid[:-1], sid[1:])]
        late = [False] + [x > 0.015 for x in ref_gaps(list(self.t))]
        n_new = sum(new)
        self.assertEqual(lv['n_new'], n_new)
        self.assertEqual(lv['n_other'], len(self.ks) - n_new)
        self.assertAlmostEqual(lv['p_late_new_snapshot'], sum(l for l, n in zip(late, new) if n) / n_new, places=12)
        self.assertAlmostEqual(lv['p_late_otherwise'],
                               sum(l for l, n in zip(late, new) if not n) / (len(self.ks) - n_new), places=12)

    def test_late_by_rows_band(self):
        lb = self.out['runs'][0]['late_by_rows']
        rows = [(20.0, 26.0, 35.0)[k % 3] for k in self.ks]
        g = [float('nan')] + ref_gaps(list(self.t))
        for lab, lo, hi in (('low', -1, 24), ('mid', 24, 28), ('high', 28, 1e9)):
            sel = [i for i in range(1, len(rows)) if lo < rows[i] <= hi]
            self.assertEqual(lb[lab]['n'], len(sel), lab)
            self.assertAlmostEqual(lb[lab]['late_frac'], sum(g[i] > 0.015 for i in sel) / len(sel), places=12, msg=lab)

    def test_input_delivery_fractions(self):
        inp = self.out['runs'][0]['inputs']
        self.assertEqual(inp['distances']['in_bag'], 840)
        self.assertAlmostEqual(inp['distances']['processed_frac'], 1.0, places=9)
        self.assertEqual(inp['clock']['in_bag'], 28000)
        self.assertAlmostEqual(inp['clock']['processed_frac'], 1.0, places=9)

    def test_effective_rtf_and_validity(self):
        r = self.out['runs'][0]
        self.assertAlmostEqual(r['sim_time_rate']['effective_rtf'], RATE, places=6)
        self.assertTrue(r['valid'], r['validity'])

    def test_workload_diff(self):
        w = self.out['runs'][0]['workload_vs_e3_05']
        self.assertAlmostEqual(w['rows_p50_diff'], 0.0, places=9)      # median of {20, 26, 35} thirds = 26
        mean = float(np.mean([(20.0, 26.0, 35.0)[k % 3] for k in self.ks]))
        self.assertAlmostEqual(w['rows_mean_diff'], mean - 26.0, places=9)


class TestValidityGates(unittest.TestCase):
    """Part 1: each pre-registered gate rejects what it should (protocol §7)."""

    def _one(self, name, ref_rows=26.0, **kw):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        e3 = os.path.join(tmp.name, 'e3')
        make_e3(e3, ref_rows=ref_rows)
        runs = os.path.join(tmp.name, 'runs')
        os.makedirs(runs)
        make_run(runs, name, kw.pop('arm', 'mt3'), tick_indices(), **kw)
        p, out = run_analyze(runs, e3, [name])
        self.assertEqual(p.returncode, 0, p.stderr[-2000:])
        return out['runs'][0]

    def test_workload_gate_rejects_row_shift(self):
        r = self._one('w', ref_rows=21.0)                # replay median 26 -> +5 rows
        self.assertFalse(r['validity']['workload_rows_p50'])
        self.assertFalse(r['valid'])

    def test_executor_gate_rejects_wrong_worker_count(self):
        r = self._one('x', arm='mt1', worker=3)
        self.assertFalse(r['validity']['executor_as_intended'])
        self.assertFalse(r['valid'])

    def test_executor_gate_rejects_wrong_class(self):
        r = self._one('c', arm='single', executor='MultiThreadedExecutor')
        self.assertFalse(r['validity']['executor_as_intended'])

    def test_rtf_gate_rejects_slow_playback(self):
        r = self._one('r', rtf=0.5)
        self.assertAlmostEqual(r['sim_time_rate']['effective_rtf'], 0.5, places=6)
        self.assertFalse(r['validity']['rtf_in_range'])

    def test_all_gates_pass_on_nominal(self):
        r = self._one('ok')
        self.assertTrue(r['valid'], r['validity'])


class TestWindowBoundary(unittest.TestCase):
    """Defect D-M1-2 (Addendum 03): ``missed_slots`` assumed exactly 2800 slots in [W0, W1].

    With ticks exactly on the grid at both ends of the inclusive window there are 2801 tick slots,
    and ``2800 - unique`` went negative. The definition is "due 10 ms grid points in [W0, W1) with
    no tick", which can never be negative.
    """

    def test_ticks_on_both_window_edges(self):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        e3 = os.path.join(tmp.name, 'e3')
        make_e3(e3)
        runs = os.path.join(tmp.name, 'runs')
        os.makedirs(runs)
        ks = list(range(2801))                              # 12.00 … 40.00 inclusive
        t = make_run(runs, 'edge', 'single', ks, t_of=lambda k: round(W0 + k / 100.0, 9))
        self.assertEqual(t[0], W0)
        self.assertEqual(t[-1], W1)
        p, out = run_analyze(runs, e3, ['edge'])
        self.assertEqual(p.returncode, 0, p.stderr[-2000:])
        s = out['runs'][0]['sim']
        self.assertGreaterEqual(s['missed_slots'], 0)
        self.assertEqual(s['missed_slots'], ref_missed(list(t)))
        self.assertEqual(s['missed_slots'], 0)


class TestArmSummary(unittest.TestCase):
    """Part 1: the pre-registered mt1-position rule (protocol §8)."""

    def _arms(self, rebuild):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        e3 = os.path.join(tmp.name, 'e3')
        make_e3(e3)
        runs = os.path.join(tmp.name, 'runs')
        os.makedirs(runs)
        ids = []
        for arm, vals in rebuild.items():
            for j, v in enumerate(vals):
                rid = f'{arm}_{j}'
                make_run(runs, rid, arm, tick_indices(), rebuild_ms=v)
                ids.append(rid)
        p, out = run_analyze(runs, e3, ids)
        self.assertEqual(p.returncode, 0, p.stderr[-2000:])
        return out

    def test_like_single(self):
        out = self._arms({'single': [2.0, 2.1, 1.9], 'mt1': [2.4, 2.5, 2.3], 'mt3': [6.0, 6.1, 5.9]})
        m = out['mt1_position']['rebuild_wall_p50']
        self.assertTrue(m['separated'])
        self.assertAlmostEqual(m['f_mt1'], 0.1, places=6)
        self.assertEqual(m['label'], 'like_single')
        a = out['per_arm']['mt3']['rebuild_wall_p50']
        self.assertAlmostEqual(a['mean'], 6.0, places=6)
        self.assertAlmostEqual(a['sd'], 0.1, places=6)

    def test_like_mt3_and_intermediate(self):
        out = self._arms({'single': [2.0, 2.1, 1.9], 'mt1': [5.0, 5.1, 4.9], 'mt3': [6.0, 6.1, 5.9]})
        self.assertEqual(out['mt1_position']['rebuild_wall_p50']['label'], 'like_mt3')
        out = self._arms({'single': [2.0, 2.1, 1.9], 'mt1': [4.0, 4.1, 3.9], 'mt3': [6.0, 6.1, 5.9]})
        self.assertEqual(out['mt1_position']['rebuild_wall_p50']['label'], 'intermediate')

    def test_not_separated_when_ranges_overlap(self):
        out = self._arms({'single': [2.0, 3.0, 4.0], 'mt1': [3.0, 3.0, 3.0], 'mt3': [3.5, 4.5, 5.5]})
        m = out['mt1_position']['rebuild_wall_p50']
        self.assertFalse(m['separated'])
        self.assertEqual(m['label'], 'not_separated')


# ─────────────────────────────── recorded evidence ───────────────────────────────

def sha(p):
    h = hashlib.sha256()
    with open(p, 'rb') as f:
        for b in iter(lambda: f.read(1 << 20), b''):
            h.update(b)
    return h.hexdigest()


@unittest.skipUnless(os.path.isdir(os.path.join(EVID, 'runs')) and os.path.isdir(E305), 'M1 evidence not present')
class TestRecordedEvidence(unittest.TestCase):
    """Part 2: the recorded M1 experiment."""

    def _check_sums(self, base, sums):
        bad = []
        with open(os.path.join(base, sums)) as fh:
            lines = fh.read().splitlines()
        for line in lines:
            h, f = line.split('  ', 1)
            if sha(os.path.join(base, f)) != h:
                bad.append(f)
        return bad

    def test_protocol_scripts_and_raw_data_checksums(self):
        self.assertEqual(self._check_sums(os.path.join(EVID, 'protocol'), 'SHA256SUMS'), [])
        self.assertEqual(self._check_sums(os.path.join(EVID, 'scripts'), 'SHA256SUMS'), [])
        self.assertEqual(self._check_sums(os.path.join(EVID, 'scripts_a01'), 'SHA256SUMS'), [])
        self.assertEqual(self._check_sums(EVID, 'SHA256SUMS_raw_data'), [])

    def test_raw_data_is_write_protected(self):
        for d in ('runs', 'logs', 'host', 'setup', 'smoke'):
            for f in os.listdir(os.path.join(EVID, d)):
                p = os.path.join(EVID, d, f)
                if os.path.isfile(p):
                    self.assertFalse(os.access(p, os.W_OK), p)

    def test_unchanged_harness_between_scripts_and_a01(self):
        def sums(p):
            with open(p) as fh:
                return {l.split()[1]: l.split()[0] for l in fh.read().splitlines()}
        a = sums(os.path.join(EVID, 'scripts/SHA256SUMS'))
        b = sums(os.path.join(EVID, 'scripts_a01/SHA256SUMS'))
        for f in ('m1_executor_replay.py', 'r1_executor_replay.py', 'm1_run.sh', 'm1_batch.sh'):
            self.assertEqual(a[f], b[f], f)

    def test_all_nine_runs_present_with_all_files(self):
        for rid, _ in ORDER:
            for s in ('.json', '_raw.npz', '_m1.json', '_m1.npz', '_check.json'):
                self.assertTrue(os.path.isfile(os.path.join(EVID, 'runs', rid + s)), rid + s)

    def test_executor_configuration_per_arm(self):
        want = {'single': ('SingleThreadedExecutor', None, 'single'), 'mt1': ('MultiThreadedExecutor', 1, 'multi'),
                'mt3': ('MultiThreadedExecutor', 3, 'multi')}
        for rid, arm in ORDER:
            X = jload(os.path.join(EVID, 'runs', rid + '_m1.json'))
            S = jload(os.path.join(EVID, 'runs', rid + '.json'))
            w = want[arm]
            self.assertEqual((X['arm'], X['executor_class'], X['worker_threads'], X['node_executor_mode']),
                             (arm, w[0], w[1], w[2]), rid)
            self.assertEqual((S['executor'], S['mode']), (w[0], w[2]), rid)
            self.assertEqual(X['multi_executor_threads_constant'], 1 if arm == 'mt1' else 3, rid)

    def test_identical_parameters_and_inputs(self):
        ref = None
        for rid, arm in ORDER:
            X = jload(os.path.join(EVID, 'runs', rid + '_m1.json'))
            p = dict(X['params'])
            self.assertEqual(p.pop('executor_mode'), 'single' if arm == 'single' else 'multi')
            key = (json.dumps(p, sort_keys=True), X['cell_dir'], tuple(X['params_files']), X['rate'],
                   json.dumps(X['subscriptions']), json.dumps(X['timers_period_s']))
            ref = ref or key
            self.assertEqual(key, ref, rid)
        self.assertEqual(json.loads(ref[4]), [['/NS_1/franka/joint_states', 'JointState'],
                                              ['/NS_1/iso_safety', 'Float64MultiArray'],
                                              ['/NS_1/qddot_nom', 'Float64MultiArray'],
                                              ['/cbf/per_link_distances', 'MultiLinkDistance'],
                                              ['/clock', 'Clock']])

    def test_parameter_files_are_the_e3_05_files(self):
        src = os.path.join(E305, 'run/container_tmp')
        exp = {'quny0qie': '1ea074547a2b10034d7bfbafc7275585249463f49a9d906c0e42ed1e8efd2b73',
               'w4adpacz': '5e18b7bfe3f0f9aaca5f86ae1fd962b6b3e210f30adebe8f4ce3e2c63c9f4bbf',
               'xa00zx9n': '6162eadc95f0ccc58bd29abf248f59bcc16e4fcc70b91730e4b3b899e71f8862'}
        for k, h in exp.items():
            self.assertEqual(sha(os.path.join(src, f'tmp__launch_params_{k}')), h, k)

    def test_execution_order_matches_protocol(self):
        starts = []
        for rid, _ in ORDER:
            with open(os.path.join(EVID, 'host', rid + '.txt')) as fh:
                line = [l for l in fh.read().splitlines() if l.startswith('rc ')][0].split()
            self.assertEqual(line[1], '0', rid)
            starts.append((line[3], rid))
        self.assertEqual([r for _, r in sorted(starts)], [r for r, _ in ORDER])

    def test_pre_registered_validity_of_every_run(self):
        A = jload(os.path.join(EVID, 'analysis/m1_analysis.json'))
        self.assertEqual([r['id'] for r in A['runs']], [r for r, _ in ORDER])
        for r in A['runs']:
            self.assertTrue(r['valid'], (r['id'], r['validity']))

    def test_host_state_unchanged(self):
        for rid, _ in ORDER:
            with open(os.path.join(EVID, 'host', rid + '.txt')) as fh:
                txt = fh.read()
            for k in ('governor 28 powersave', 'epp 28 performance', 'post governor 28 powersave',
                      'post epp 28 performance', 'host_ros_gz 0', 'container_ros 0', 'post container_ros 0'):
                self.assertIn(k + '\n', txt, (rid, k))

    def test_analysis_reproducible_from_immutable_data(self):
        """The reported analysis equals a fresh run of the same script on the sealed data."""
        A = os.path.join(EVID, 'analysis/m1_analysis.json')
        script = os.environ.get('M1_ANALYZE_REPORTED', os.path.join(EVID, 'scripts_a01/m1_analyze.py'))
        with tempfile.TemporaryDirectory() as tmp:
            out = os.path.join(tmp, 'again.json')
            p = subprocess.run([sys.executable, '-I', script, os.path.join(EVID, 'runs'), E305, out,
                                *[r for r, _ in ORDER]], capture_output=True, text=True)
            self.assertEqual(p.returncode, 0, p.stderr[-2000:])
            self.assertEqual(jload(out), jload(A))

    def test_corrected_analysis_changes_no_reported_value(self):
        """Addendum 03: the D-M1-2 fix (the script under test) reproduces the reported analysis exactly."""
        A = os.path.join(EVID, 'analysis/m1_analysis.json')
        with tempfile.TemporaryDirectory() as tmp:
            out = os.path.join(tmp, 'fixed.json')
            p = subprocess.run([sys.executable, '-I', ANALYZE, os.path.join(EVID, 'runs'), E305, out,
                                *[r for r, _ in ORDER]], capture_output=True, text=True)
            self.assertEqual(p.returncode, 0, p.stderr[-2000:])
            self.assertEqual(jload(out), jload(A))

    def test_workload_matches_e3_05_at_matched_times(self):
        """Descriptive check beyond the gates: at matched sim times the replay's n_rows equals E3-05's
        within ±2 rows on at least 95 % of ticks, in every run. The 95 % bar is the smoke run's 97.8 %
        less a margin. It is not a pre-registered gate; a failure would be reported, not hidden."""
        A = jload(os.path.join(EVID, 'analysis/m1_analysis.json'))
        for r in A['runs']:
            self.assertGreaterEqual(r['workload_vs_e3_05']['matched_within_2'], 0.95, r['id'])
            self.assertEqual(r['status']['fault_ticks'], 0, r['id'])
            self.assertEqual(r['status']['fallback_levels'], {'0': r['status']['rows']}, r['id'])


if __name__ == '__main__':
    unittest.main()
