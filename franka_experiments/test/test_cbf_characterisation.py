"""Characterisation tests: pin the CURRENT behaviour of the CBF safety filter before the
``cbf_state_rows.py`` / ``cbf_safety_filter.py`` refactoring.

These tests describe what the code DOES at the refactoring baseline (B-2, ``humble-mattia``
``34d8daa``), not what it should do. A pure structural extraction must keep every one of them green
without touching the goldens. A failing test after a production change means the behaviour changed;
the goldens are regenerated only when a behaviour change has been approved on its own
(``gen_cbf_characterisation_goldens.py``). Tests that pin a KNOWN DEFECT say so in their name and
docstring (F-8-44, F-8-67); fixing the defect is a separate, approved step that updates them.

Equality is EXACT everywhere (``np.array_equal``, NaN == NaN): the harness is deterministic (fake
clock, sequential callbacks, OSQP ρ-interval fixed; see ``_cbf_characterisation.py``) and a pure
extraction does not change floating-point operations or their order. No tolerance is used.

Harness and goldens: ``_cbf_characterisation.py``, ``data/cbf_characterisation/``. Analysis and the
extraction plan: ``docs/framework_study/refactoring/CBF_MONOLITHS_ANALYSIS.md``.
"""
import inspect
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _cbf_characterisation as H  # noqa: E402

import franka_experiments.nodes.cbf_safety_filter as csf  # noqa: E402
import franka_experiments.utils.cbf_state_rows as csr  # noqa: E402
from franka_experiments.utils.cbf_qp_assembly import (  # noqa: E402
    OSQP_LEVEL_BOX, OSQP_LEVEL_BRAKE, OSQP_LEVEL_FULL)
from franka_experiments.utils.cbf_state_rows import (  # noqa: E402
    G_CAP, G_OBS, G_QLIM, G_SC, G_SING, G_SPD, GROUP_NAMES)

SCENARIOS = ['free_space', 'obstacle_approach', 'obstacle_static_close', 'joint_limit',
             'singularity', 'self_collision', 'faults', 'faults_box_only', 'optional_families',
             'iso_speed_rows', 'input_edge_cases']

#: What each scenario must exercise (checked on the golden), so a golden cannot silently stop
#: covering the family it was written for. Groups are ConstraintSnap.group values.
EXPECT = {
    'free_space': dict(groups={G_SING}, levels={OSQP_LEVEL_FULL}),
    'obstacle_approach': dict(groups={G_OBS, G_CAP, G_SPD, G_SING}, levels={OSQP_LEVEL_FULL}),
    'obstacle_static_close': dict(groups={G_OBS, G_CAP, G_SPD}, levels={OSQP_LEVEL_FULL}),
    'joint_limit': dict(groups={G_QLIM}, levels={OSQP_LEVEL_FULL}),
    'singularity': dict(groups={G_SING}, levels={OSQP_LEVEL_FULL}, governor=True),
    'self_collision': dict(groups={G_SC}, levels={OSQP_LEVEL_FULL}),
    'faults': dict(groups={G_OBS}, levels={OSQP_LEVEL_FULL, OSQP_LEVEL_BRAKE}, fault=True),
    'faults_box_only': dict(groups={G_OBS}, levels={OSQP_LEVEL_FULL, OSQP_LEVEL_BOX}, fault=True),
    'optional_families': dict(groups={G_OBS}, levels={OSQP_LEVEL_FULL}, memory=True),
    'iso_speed_rows': dict(groups={G_OBS, G_SPD}, levels={OSQP_LEVEL_FULL}),
    'input_edge_cases': dict(groups={G_OBS}, levels={OSQP_LEVEL_FULL}),
}


@pytest.fixture(scope='module')
def scenario_map():
    stale = H.installed_vs_source()
    assert not stale, (f'the imported franka_experiments differs from the source tree in {stale}: '
                       f'rebuild (colcon build) before running the characterisation tests')
    node, _ = H.make_node()
    try:
        kin = H.harness_kinematics(node)
    finally:
        H.close_node(node)
    m = {sc.name: sc for sc in H.scenarios(kin)}
    assert sorted(m) == sorted(SCENARIOS)
    return m


def _golden(name):
    num, meta = H.load(name)
    if meta['environment'] != H.fingerprint():
        pytest.skip(f'goldens are bit-exact for {meta["environment"]}, this is '
                    f'{H.fingerprint()}: regenerate them at the baseline commit in this environment')
    return num, meta


def _values(num, key):
    if key in num:
        return num[key].ravel()
    return num[key + '::flat']


# ── 1-7, 9: golden closed-loop runs ──────────────────────────────────────────

@pytest.mark.parametrize('name', SCENARIOS)
def test_golden_closed_loop(scenario_map, name):
    """Rows of every family, QP matrices and bounds, nominal and safe commands, /cbf_status,
    fallback rungs and edge-case inputs, tick by tick, bit-exact against the baseline."""
    num_g, meta_g = _golden(name)
    num_n, meta_n = H.record(scenario_map[name])
    assert meta_n['config_sha256'] == meta_g['config_sha256'], (
        'fr3_control.yaml differs from the one the goldens were recorded with: a configuration '
        'change is a behaviour change and needs its own approval and a deliberate re-baseline')
    diffs = H.compare(num_g, meta_g, num_n, meta_n)
    assert not diffs, f'{name}: behaviour changed:\n  ' + '\n  '.join(diffs[:25])


@pytest.mark.parametrize('name', SCENARIOS)
def test_golden_exercises_its_family(name):
    num, meta = _golden(name)
    e = EXPECT[name]
    groups = {int(g) for g in _values(num, 'rebuild/snap_group')}
    assert e['groups'] <= groups, f'{name}: groups {sorted(GROUP_NAMES[g] for g in groups)}'
    levels = {int(x) for x in num['tick/level']}
    assert levels == e['levels'], f'{name}: fallback levels {levels}'
    # every tick with rows went through OSQP, and its matrices and bounds are in the golden
    assert any(k.startswith('tick/osqp') and k.endswith(('_l', '_l::flat')) for k in num)
    fault = num['tick/status'][:, meta['status_fields'].index('fault_braking')]
    assert bool(np.any(fault > 0)) == e.get('fault', False)
    if e.get('governor'):
        assert np.nanmin(num['tick/gov_w']) < 1.0
    if e.get('memory'):
        assert np.any(_values(num, 'rebuild/rows_diag_mem_rows') > 0)


def test_harness_is_deterministic_in_process(scenario_map):
    """The precondition of exact goldens: two runs in one process are bit-identical."""
    sc = scenario_map['obstacle_static_close']
    a = H.record(sc)
    b = H.record(sc)
    assert H.compare(*a, *b) == []


# ── 4: workspace ─────────────────────────────────────────────────────────────

def test_no_workspace_family_in_the_shipped_filter():
    """The shipped filter has NO Cartesian workspace rows: the row families are exactly these
    seven, and ``cbf_hard_limits.workspace_face_rows`` (TODO[LEGACY], reverted in 4d4d450) is not
    used by the node or the builder. A refactor must not (re)introduce one silently."""
    assert GROUP_NAMES == ('obs', 'sc', 'qlim', 'sing', 'cap', 'spd', 'ssb')
    for mod in (csf, csr):
        assert 'workspace_face_rows' not in inspect.getsource(mod)


# ── helpers for the focused tests ────────────────────────────────────────────

def _feed(node, clock, kin, q, ball, frames):
    node._on_joint_state(H.joint_state_msg(q, np.zeros(H.NV), clock.t))
    node._on_qddot_nom(H.nominal_msg(np.zeros(H.NV)))
    node._on_distances(H.distance_msg(kin, q, np.zeros(H.NV), clock.t - 0.02, ball,
                                      frames_seen=frames))


def _near_ball(kin):
    ee0 = H._ee_point(kin, H.Q0)
    return (ee0 + np.array([0.12, 0.0, 0.0]), np.zeros(3), 0.05)


# ── 8: snapshot consistency and F-8-44 ───────────────────────────────────────

def test_tick_solves_with_the_snapshot_it_read_first():
    """A rebuild that lands DURING a tick (MultiThreadedExecutor, ≥2 workers) must not change the
    rows that tick solves: ``_qp_tick`` reads ``self._con`` once. The swap is made from inside the
    governor call, i.e. after the read and before the solve."""
    node, clock = H.make_node()
    try:
        kin = H.harness_kinematics(node)
        log = H.QPLog()
        with H.deterministic_osqp(log):
            ball = _near_ball(kin)
            _feed(node, clock, kin, H.Q0, ball, 5)
            node._update_constraints()
            con_a = node._con
            clock.t += 0.02
            q_b = H.Q0 + np.array([0.0, 0.1, 0.0, 0.3, 0.0, 0.0, 0.0])
            _feed(node, clock, kin, q_b, ball, 6)
            node._update_constraints()
            con_b = node._con
            assert con_b is not con_a
            assert con_b.A.shape != con_a.A.shape or not np.array_equal(con_b.A, con_a.A)
            node._con = con_a
            seen = {}
            w0, s0 = node._gov.weight, node._solve

            def weight(**kw):
                node._con = con_b                     # the concurrent rebuild lands here
                return w0(**kw)

            def solve(G, h_qp, n_c, qdot):
                seen.update(G=G, n_c=n_c)
                return s0(G, h_qp, n_c, qdot)
            node._gov.weight, node._solve = weight, solve
            node._qp_tick()
        assert seen['G'] is con_a.G
        assert seen['n_c'] == con_a.A.shape[0]
        assert node._status_pub.msgs[-1][0] == con_a.A.shape[0]
    finally:
        H.close_node(node)


def test_f844_known_defect_governor_sigma_not_from_the_snapshot():
    """CHARACTERISES THE KNOWN DEFECT F-8-44 (current behaviour; update deliberately when the fix,
    P2, is approved). The governor's sigma is read from the BUILDER attribute ``diag_sigma``, its
    d_sc from the SNAPSHOT. When a rebuild's singularity row raises (contained by its own
    try/except), the new snapshot is installed but ``diag_sigma`` keeps the previous build's value:
    one tick then pairs sigma(q_a) with d_sc(q_b). Reproducible in a single thread."""
    node, clock = H.make_node()
    try:
        kin = H.harness_kinematics(node)
        log = H.QPLog()
        with H.deterministic_osqp(log):
            ball = _near_ball(kin)
            _feed(node, clock, kin, H.Q0, ball, 5)
            node._update_constraints()
            sigma_a = node._rows.diag_sigma
            clock.t += 0.02
            q_b = H.Q0.copy()
            q_b[3] = -1.0                             # elbow stretched: a different sigma
            _feed(node, clock, kin, q_b, ball, 6)
            sing = node._rows._sing_rows

            def boom(*a, **k):
                raise RuntimeError('injected singularity-row failure')
            sing.build = boom
            node._update_constraints()                # contained: no exception here
            del sing.build
            con_b = node._con
            assert con_b is not None and G_SING not in set(con_b.group.tolist())
            sigma_b = float(sing.sigma(q_b))
            assert sigma_b != sigma_a
            seen = {}
            w0 = node._gov.weight

            def weight(**kw):
                seen.update(kw)
                return w0(**kw)
            node._gov.weight = weight
            node._qp_tick()
        assert seen['sigma'] == sigma_a               # F-8-44: the previous build's sigma ...
        assert seen['d_sc'] == con_b.d_sc_min         # ... paired with the new snapshot's d_sc
    finally:
        H.close_node(node)


# ── 9: exceptional cases ─────────────────────────────────────────────────────

def test_f867_known_behaviour_unguarded_build_exception_propagates():
    """CHARACTERISES F-8-67 (current behaviour). An exception in ``ConstraintBuilder.build``
    outside the three guarded families (singularity, self-collision, SSB) is not caught by
    ``build`` or by ``_update_constraints``: it reaches the executor, which re-raises it from
    ``spin()`` (rclpy Humble), and ``main`` destroys the node. The previous snapshot stays."""
    node, clock = H.make_node()
    try:
        kin = H.harness_kinematics(node)
        ball = _near_ball(kin)
        _feed(node, clock, kin, H.Q0, ball, 5)
        node._update_constraints()
        con_a = node._con
        clock.t += 0.02
        _feed(node, clock, kin, H.Q0, ball, 6)

        def boom(*a, **k):
            raise RuntimeError('injected obstacle-row failure')
        node._rows._control_point_jacobian = boom
        with pytest.raises(RuntimeError, match='injected obstacle-row failure'):
            node._update_constraints()
        assert node._con is con_a
    finally:
        H.close_node(node)


def test_self_collision_failure_is_contained():
    """The self-collision family's own try/except: rows skipped, snapshot still built."""
    node, clock = H.make_node()
    try:
        kin = H.harness_kinematics(node)
        ball = _near_ball(kin)
        q_sc = np.array([0.0, 0.6, 0.0, -2.5, 0.0, 1.0, 0.8])
        _feed(node, clock, kin, q_sc, ball, 5)

        def boom(*a, **k):
            raise RuntimeError('injected self-collision failure')
        node._rows._sc_rows.build = boom
        node._update_constraints()
        assert node._con is not None
        assert G_SC not in set(node._con.group.tolist())
    finally:
        H.close_node(node)


def test_iso_enabled_is_refused_at_the_shipped_intrusion_distance():
    """With iso_enabled and the shipped C = 0.85 m, the node refuses to start (Annex L): there is
    no bypass. A refactor of __init__ must keep this check before the node runs."""
    with pytest.raises(ValueError, match='Annex L'):
        H.make_node({'iso_enabled': True})
