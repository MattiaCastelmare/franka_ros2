#!/usr/bin/env python3
"""F-8-44: does the state governor pair sigma_min and d_sc_min from different constraint builds?

Isolated, deterministic test on the **real** node and builder. It is not part of the package test
suite, and no production file is modified. It runs inside the container (ROS environment):

  source /ros2_ws/install/setup.bash
  F844_PARAMS="<p1> <p2> <p3>" F844_OUT=<out.json> python3 -B test_f844_governor_sigma_snapshot.py -v

* ``F844_PARAMS``: the E3-05 filter parameter files (the configuration of E3 and M1 / M3).
* No executor and no spin. The callbacks are called directly, in a chosen order, on one thread.
  The multi-threaded interleaving is reproduced exactly: one ``_qp_tick`` is called from inside
  ``ConstraintBuilder.build``, at the self-collision FK pass. That pass runs after ``diag_sigma``
  has been set (``cbf_state_rows.py`` L3350) and before the new snapshot is returned (L3795) and
  assigned to ``node._con``. This is the window in which a concurrent tick can run under
  ``MultiThreadedExecutor`` with ≥ 2 workers.

Tests:
  T1 (multi interleaving): the governor gets sigma from the in-progress build and d_sc from the
     previous snapshot. Reproduction: passes when the defect is present.
  T2 (sequential, as single / MT(1)): the governor's sigma is the one of the build that produced
     the snapshot.
  T3 (single, exception path): when the singularity row raises, ``diag_sigma`` stays at the older
     build while the snapshot is new. A mismatch is possible in ``single`` too.
  T4 (effect): the two sigma values give different governor weights and nominal commands.
  T5 (constraints untouched): the QP is solved with the snapshot's own rows in either case.
  T6 (spec, expected failure until P2): the governor's sigma belongs to the snapshot it uses.
  T7 (magnitude): the worst-case sigma change between two rebuilds at the FR3 velocity limits, and
     the governor-weight change that it can cause (Derived, printed and saved).
"""
import json
import os
import unittest

import numpy as np
import rclpy
from franka_msgs.msg import LinkDistance, MultiLinkDistance
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from franka_experiments.nodes.cbf_safety_filter import CBFSafetyFilter, FR3_JOINTS
from franka_experiments.utils.cbf_state_rows import G_SING
from franka_experiments.utils.state_governor import directional_fade, governor_from_params

PFILES = os.environ.get('F844_PARAMS', '').split()
OUT = os.environ.get('F844_OUT')
RESULTS = {}
Q0 = np.array([0.0, -0.4, 0.0, -1.8, 0.0, 1.6, 0.8])       # the smoke test's pose
QDOT_MAX = np.array([2.62, 2.62, 2.62, 2.62, 5.26, 4.18, 5.26])  # FR3 joint velocity limits [rad/s]
NODE = None


def setUpModule():
    global NODE
    args = ['f844', '--ros-args', '-r', '__node:=cbf_safety_filter']
    for p in PFILES:
        args += ['--params-file', p]
    rclpy.init(args=args)
    NODE = CBFSafetyFilter()


def tearDownModule():
    if OUT:
        with open(OUT, 'w') as f:
            json.dump(RESULTS, f, indent=1)
    NODE.destroy_node()
    rclpy.shutdown()


def feed(node, q, qdot, nom):
    """Fresh joint state, nominal and one far obstacle, through the node's own callbacks."""
    stamp = node.get_clock().now().to_msg()
    js = JointState()
    js.header.stamp = stamp
    js.name = list(FR3_JOINTS)
    js.position = [float(x) for x in q]
    js.velocity = [float(x) for x in qdot]
    node._on_joint_state(js)
    m = Float64MultiArray()
    m.data = [float(x) for x in nom]
    node._on_qddot_nom(m)
    ld = LinkDistance()
    ld.robot_link_name = 'fr3_link5'
    ld.distance = 0.80
    ld.closest_point_robot = Point(x=0.3, y=0.0, z=0.6)
    ld.closest_point_human = Point(x=0.3, y=0.8, z=0.6)
    ld.confidence = 0.9
    ld.valid = True
    d = MultiLinkDistance()
    d.header.stamp = stamp
    d.links = [ld]
    node._on_distances(d)


def sigma_of(node, q):
    return float(node._rows._sing_rows.sigma(np.asarray(q, float)))


def pose_with_sigma(node, target, j=3, lo=-1.8, hi=-0.46):
    """Bisect joint j (elbow, q4) between Q0 (sigma 0.143) and the extended elbow (sigma 0.0006) until
    sigma_min == target. A first version searched q6 on [0, 1.6], whose minimum is 0.0565, and skipped."""
    f = lambda x: sigma_of(node, np.r_[Q0[:j], x, Q0[j + 1:]]) - target  # noqa: E731
    a, b = lo, hi
    if f(a) * f(b) > 0:
        raise unittest.SkipTest(f'no pose with sigma={target} on q{j + 1} in [{lo}, {hi}]')
    for _ in range(60):
        m = 0.5 * (a + b)
        if f(a) * f(m) <= 0:
            b = m
        else:
            a = m
    return np.r_[Q0[:j], 0.5 * (a + b), Q0[j + 1:]]


class Recorder:
    """Wrap the governor's weight() and the node's _solve() to see what one tick used."""
    def __init__(self, node):
        self.node, self.calls, self.solves = node, [], []
        self._w, self._s = node._gov.weight, node._solve

    def __enter__(self):
        node = self.node

        def weight(**kw):
            self.calls.append(dict(sigma=kw.get('sigma'), d_sc=kw.get('d_sc'), con_id=id(node._con)))
            return self._w(**kw)

        def solve(*a, **k):
            self.solves.append(dict(con_id=id(node._con), n_rows=None if node._con is None else int(node._con.A.shape[0])))
            return self._s(*a, **k)
        node._gov.weight = weight
        node._solve = solve
        return self

    def __exit__(self, *exc):
        self.node._gov.weight = self._w
        self.node._solve = self._s


class F844(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        n = NODE
        if n._gov is None or n._rows._sing_rows is None or n._rows._sc_rows is None:
            raise unittest.SkipTest('governor, singularity rows or self-collision rows disabled by the parameters')
        P = n.P
        cls.floor, cls.band = float(P.singularity_sigma_floor), float(P.governor_sigma_band)
        # sigma inside the governor band: A at the band middle, B nearer its top (as if 20 ms later)
        cls.qA = pose_with_sigma(n, cls.floor + 0.5 * cls.band)
        cls.qB = pose_with_sigma(n, cls.floor + 0.9 * cls.band)
        cls.qd = np.zeros(7)
        cls.nom = np.array([0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0])    # drives q4 up (elbow extends) = sigma down
        RESULTS['poses'] = dict(qA=cls.qA.tolist(), qB=cls.qB.tolist(), sigmaA=sigma_of(n, cls.qA),
                                sigmaB=sigma_of(n, cls.qB), floor=cls.floor, band=cls.band,
                                dq4_rad=float(cls.qB[3] - cls.qA[3]),
                                qdot4_needed_for_20ms_rad_s=float((cls.qB[3] - cls.qA[3]) / 0.02))

    def setUp(self):
        NODE._gov.reset(1.0)

    def build_A(self):
        feed(NODE, self.qA, self.qd, self.nom)
        NODE._update_constraints()
        self.assertIsNotNone(NODE._con)
        return NODE._con, float(NODE._rows.diag_sigma)

    def interleaved_tick(self):
        """Build A, then build B with ONE tick placed inside it, right after diag_sigma is set."""
        conA, sA = self.build_A()
        feed(NODE, self.qB, self.qd, self.nom)
        kin = NODE._rows._sc_kin
        orig = kin.update
        state = {}

        def update(*a, **k):
            r = orig(*a, **k)
            if 'done' not in state:
                state['done'] = True
                state['sigma_builder_mid_build'] = float(NODE._rows.diag_sigma)
                state['con_is_A'] = NODE._con is conA
                with Recorder(NODE) as rec:
                    NODE._qp_tick()
                state['rec'] = rec
            return r
        kin.update = update
        try:
            NODE._update_constraints()
        finally:
            kin.update = orig
        return conA, sA, NODE._con, state

    def test_T1_multi_interleaving_pairs_two_builds(self):
        conA, sA, conB, st = self.interleaved_tick()
        call = st['rec'].calls[-1]
        self.assertTrue(st['con_is_A'])                                    # the tick still saw snapshot A
        self.assertAlmostEqual(st['sigma_builder_mid_build'], sigma_of(NODE, self.qB), places=9)
        self.assertAlmostEqual(call['sigma'], sigma_of(NODE, self.qB), places=9)   # sigma of build B
        self.assertEqual(call['d_sc'], conA.d_sc_min)                      # d_sc of snapshot A
        self.assertNotAlmostEqual(call['sigma'], sA, places=6)             # not snapshot A's sigma
        self.assertIsNot(conB, conA)
        RESULTS['T1'] = dict(sigma_used=call['sigma'], sigma_of_snapshot_used=sA, d_sc_used=call['d_sc'])

    def test_T2_sequential_ticks_are_consistent(self):
        conA, sA = self.build_A()
        with Recorder(NODE) as rec:
            NODE._qp_tick()
        call = rec.calls[-1]
        self.assertEqual(call['con_id'], id(conA))
        self.assertAlmostEqual(call['sigma'], sA, places=12)
        self.assertEqual(call['d_sc'], conA.d_sc_min)

    def test_T3_single_exception_path_keeps_an_older_sigma(self):
        conA, sA = self.build_A()
        feed(NODE, self.qB, self.qd, self.nom)
        sr = NODE._rows._sing_rows
        orig = sr.build

        def boom(*a, **k):
            raise RuntimeError('F-8-44 test: singularity row failure')
        sr.build = boom
        try:
            NODE._update_constraints()
        finally:
            sr.build = orig
        conB = NODE._con
        self.assertIsNot(conB, conA)
        self.assertFalse(np.any(conB.group == G_SING))                     # the new snapshot has no sing row
        with Recorder(NODE) as rec:
            NODE._qp_tick()
        call = rec.calls[-1]
        self.assertAlmostEqual(call['sigma'], sA, places=12)               # sigma of build A
        self.assertEqual(call['d_sc'], conB.d_sc_min)                      # d_sc of snapshot B
        RESULTS['T3'] = dict(sigma_used=call['sigma'], sigma_true_at_B=sigma_of(NODE, self.qB))

    def test_T4_the_mismatch_changes_the_governor_command(self):
        conA, sA = self.build_A()
        sB = sigma_of(NODE, self.qB)
        out = {}
        for lab, s in (('consistent_sigmaA', sA), ('mismatched_sigmaB', sB)):
            g = governor_from_params(NODE.P)
            st = g.weight(q=self.qB, qdot=self.qd, sigma=s, d_sc=conA.d_sc_min, dt=NODE._dt_qp)
            idx = np.flatnonzero(conA.group == G_SING)
            if st.w < 1.0 and st.binding == 'sing' and idx.size and NODE.P.state_governor_directional:
                cmd = directional_fade(self.nom, self.qd, conA.A[idx], 1.0 - st.w, NODE.P.k_brake)
                path = 'directional'
            elif st.w < 1.0:
                cmd = st.w * self.nom + (1.0 - st.w) * (-NODE.P.k_brake * self.qd)
                path = 'uniform'
            else:
                cmd, path = self.nom.copy(), 'none'
            out[lab] = dict(w=st.w, binding=st.binding, path=path, cmd=cmd.tolist())
        self.assertNotAlmostEqual(out['consistent_sigmaA']['w'], out['mismatched_sigmaB']['w'], places=3)
        dcmd = np.subtract(out['mismatched_sigmaB']['cmd'], out['consistent_sigmaA']['cmd'])
        self.assertGreater(float(np.abs(dcmd).max()), 1e-3)
        out['max_abs_cmd_diff_rad_s2'] = float(np.abs(dcmd).max())
        RESULTS['T4'] = out

    def test_T5_constraints_come_from_the_snapshot_in_either_case(self):
        conA, sA, conB, st = self.interleaved_tick()
        s = st['rec'].solves
        self.assertTrue(s, 'the interleaved tick did not reach _solve')
        self.assertEqual(s[-1]['con_id'], id(conA))
        self.assertEqual(s[-1]['n_rows'], int(conA.A.shape[0]))

    @unittest.expectedFailure
    def test_T6_spec_governor_sigma_belongs_to_the_snapshot_it_uses(self):
        """Fails today (F-8-44). P2 (sigma carried in ConstraintSnap) should make it pass."""
        conA, sA, conB, st = self.interleaved_tick()
        self.assertAlmostEqual(st['rec'].calls[-1]['sigma'], sA, places=12)

    def test_T7_magnitude_bound(self):
        """Derived. |d sigma| over one rebuild period (20 ms) at the FR3 velocity limits, near the band."""
        sr, g = NODE._rows._sing_rows, governor_from_params(NODE.P)
        rows = []
        for q in (self.qA, self.qB):
            grad = np.asarray(sr.grad_sigma(np.asarray(q, float)), float)
            for lab, v in (('fr3_limits', QDOT_MAX), ('half_rad_s_each', np.full(7, 0.5))):
                ds = float(np.abs(grad) @ v * 0.02)
                rows.append(dict(sigma=sigma_of(NODE, q), velocity=lab, dsigma_max=ds,
                                 dw_max=min(1.0, 1.5 / g.sigma_band * ds)))   # smoothstep slope <= 1.5 / band
        for r in rows:
            self.assertTrue(np.isfinite(r['dsigma_max']))
        RESULTS['T7'] = rows
        print('\nT7', json.dumps(rows, indent=1))


if __name__ == '__main__':
    unittest.main()
