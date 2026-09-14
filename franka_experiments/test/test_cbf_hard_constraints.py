"""Unit tests for the hard-constraint math of the acceleration-level CBF.

Covers the pure-numpy helpers (no ROS2 runtime, no Pinocchio):

* self_collision:  URDF capsule parsing, pair selection, segment/segment
  closest-point distance
* cbf_hard_limits: joint position/velocity accel box, slew limiting,
  workspace face rows

Run with pytest, or directly:  python3 test_cbf_hard_constraints.py
"""

import numpy as np

from franka_experiments.utils.self_collision import (
    Capsule,
    build_capsule_pairs,
    parse_sc_capsules,
    segment_segment_closest,
)
from franka_experiments.utils.cbf_hard_limits import (
    apply_slew_limit,
    hard_accel_box,
    position_velocity_accel_box,
    velocity_accel_box,
    workspace_face_rows,
)


# ── segment/segment distance ─────────────────────────────────────────────────

def test_segseg_crossing():
    # Perpendicular skew segments, min distance 1 at their midpoints.
    pa, pb, d = segment_segment_closest(
        np.array([-1.0, 0.0, 0.0]), np.array([1.0, 0.0, 0.0]),
        np.array([0.0, -1.0, 1.0]), np.array([0.0, 1.0, 1.0]))
    assert abs(d - 1.0) < 1e-12
    assert np.allclose(pa, [0, 0, 0]) and np.allclose(pb, [0, 0, 1])


def test_segseg_parallel():
    # Parallel, laterally offset by 2, overlapping in x.
    _, _, d = segment_segment_closest(
        np.array([0.0, 0.0, 0.0]), np.array([1.0, 0.0, 0.0]),
        np.array([0.5, 2.0, 0.0]), np.array([1.5, 2.0, 0.0]))
    assert abs(d - 2.0) < 1e-12


def test_segseg_endpoint_clamp():
    # Disjoint collinear segments → distance between nearest endpoints.
    _, _, d = segment_segment_closest(
        np.array([0.0, 0.0, 0.0]), np.array([1.0, 0.0, 0.0]),
        np.array([3.0, 0.0, 0.0]), np.array([4.0, 0.0, 0.0]))
    assert abs(d - 2.0) < 1e-12


def test_segseg_degenerate_points():
    # Both segments degenerate to points.
    pa, pb, d = segment_segment_closest(
        np.array([0.0, 0.0, 0.0]), np.array([0.0, 0.0, 0.0]),
        np.array([0.0, 3.0, 4.0]), np.array([0.0, 3.0, 4.0]))
    assert abs(d - 5.0) < 1e-12
    assert np.allclose(pa, 0.0) and np.allclose(pb, [0, 3, 4])


def test_segseg_symmetry():
    rng = np.random.default_rng(7)
    for _ in range(50):
        p1, q1, p2, q2 = rng.normal(size=(4, 3))
        _, _, d12 = segment_segment_closest(p1, q1, p2, q2)
        _, _, d21 = segment_segment_closest(p2, q2, p1, q1)
        assert abs(d12 - d21) < 1e-9


# ── URDF capsule parsing ─────────────────────────────────────────────────────

# Hand-expanded output of the franka_description collision_capsule macro:
# one z-capsule on link1_sc, one x-capsule (rpy p=π/2) on link0_sc, plus a
# sphere pair (must be ignored) and a plain mesh link (must be skipped).
_URDF_SNIPPET = """<?xml version="1.0"?>
<robot name="t">
  <link name="fr3_link0_sc">
    <collision>
      <origin xyz="-0.075 0 0.06" rpy="0 1.5707963267948966 0"/>
      <geometry><cylinder radius="0.09" length="0.03"/></geometry>
    </collision>
    <collision>
      <origin xyz="-0.06 0 0.06"/>
      <geometry><sphere radius="0.09"/></geometry>
    </collision>
    <collision>
      <origin xyz="-0.09 0 0.06"/>
      <geometry><sphere radius="0.09"/></geometry>
    </collision>
  </link>
  <link name="fr3_link1_sc">
    <collision>
      <origin xyz="0 0 -0.1915"/>
      <geometry><cylinder radius="0.09" length="0.283"/></geometry>
    </collision>
  </link>
  <link name="fr3_link2">
    <collision><geometry>
      <mesh filename="package://x/link2.stl"/>
    </geometry></collision>
  </link>
</robot>
"""


def test_parse_capsules():
    caps = parse_sc_capsules(_URDF_SNIPPET)
    assert len(caps) == 2                      # spheres and mesh link ignored
    by_frame = {c.frame: c for c in caps}

    c0 = by_frame['fr3_link0_sc']
    assert c0.body == 'fr3_link0'
    assert abs(c0.radius - 0.09) < 1e-12
    # x-direction capsule (pitch π/2 maps local z → world x): endpoints at
    # xyz ± (L/2)·x̂
    assert np.allclose(sorted([c0.p1[0], c0.p2[0]]), [-0.09, -0.06], atol=1e-9)
    assert np.allclose(c0.p1[1:], [0.0, 0.06], atol=1e-9)

    c1 = by_frame['fr3_link1_sc']
    assert np.allclose(sorted([c1.p1[2], c1.p2[2]]),
                       [-0.1915 - 0.1415, -0.1915 + 0.1415], atol=1e-9)


def _mk(body):
    return Capsule(frame=f'{body}_sc', body=body,
                   p1=np.zeros(3), p2=np.ones(3), radius=0.05)


def test_pair_selection_adjacency():
    caps = [_mk(f'fr3_link{i}') for i in range(8)] + [_mk('fr3_hand')]
    pairs = build_capsule_pairs(caps)
    bodies = {frozenset((caps[i].body, caps[j].body)) for i, j in pairs}
    # Adjacent arm links never paired
    assert frozenset(('fr3_link3', 'fr3_link4')) not in bodies
    # Hand is adjacent to link7 AND link6 (bolted / joint7-rotation only)
    assert frozenset(('fr3_hand', 'fr3_link7')) not in bodies
    assert frozenset(('fr3_hand', 'fr3_link6')) not in bodies
    # Physically meaningful pairs present
    assert frozenset(('fr3_hand', 'fr3_link2')) in bodies
    assert frozenset(('fr3_link5', 'fr3_link7')) in bodies
    assert frozenset(('fr3_link0', 'fr3_link5')) in bodies


def test_pair_exclusion():
    caps = [_mk(f'fr3_link{i}') for i in range(8)]
    pairs = build_capsule_pairs(caps, exclude_bodies=['link0-link2'])
    bodies = {frozenset((caps[i].body, caps[j].body)) for i, j in pairs}
    assert frozenset(('fr3_link0', 'fr3_link2')) not in bodies
    assert frozenset(('fr3_link0', 'fr3_link3')) in bodies


# ── hard accel box (velocity + position limits) ──────────────────────────────

_N = 7
_ACC = np.full(_N, 10.0)
_VMAX = np.full(_N, 2.0)
_QMIN = np.full(_N, -2.5)
_QMAX = np.full(_N, 2.5)
_KW = dict(acc_lb=-_ACC, acc_ub=_ACC, qdot_max=_VMAX, v_margin=0.9,
           q_min=_QMIN, q_max=_QMAX, q_margin=0.05, brake_eta=0.7, dt=0.01)


def test_box_far_from_limits_is_static():
    lb, ub = hard_accel_box(np.zeros(_N), np.zeros(_N), **_KW)
    assert np.allclose(lb, -_ACC) and np.allclose(ub, _ACC)


def test_box_velocity_cap_bites():
    # At +v_cap the upper accel bound collapses to ~0, braking stays free.
    qdot = np.full(_N, 0.9 * 2.0)
    lb, ub = hard_accel_box(np.zeros(_N), qdot, **_KW)
    assert np.all(ub <= 1e-9)
    assert np.allclose(lb, -_ACC)


def test_box_position_limit_forces_stop():
    # Standing AT the margin-shrunk upper limit with zero velocity:
    # v_ub = 0 → no positive acceleration allowed; negative still allowed.
    q = _QMAX - 0.05
    lb, ub = hard_accel_box(q, np.zeros(_N), **_KW)
    assert np.all(ub <= 1e-9)
    assert np.all(lb < -1.0)


def test_box_braking_curve_respects_authority():
    # Approaching the upper limit ON the braking curve: demanded decel must
    # not exceed the static authority (that's what η < 1 guarantees).
    h = 0.2
    q = _QMAX - 0.05 - h
    v = np.sqrt(2 * 0.7 * 10.0 * h)          # exactly on the curve
    lb, ub = hard_accel_box(q, np.full(_N, v), **_KW)
    assert np.all(lb >= -10.0 - 1e-9)         # never beyond static decel
    assert np.all(ub <= 0.0 + 1e-9)           # must be braking
    assert np.all(ub >= -10.0)                # but not a hard-stop demand


def test_box_feasibility_guard():
    # Past the position limit at high positive velocity: bounds must still
    # satisfy lb ≤ ub (guard collapses the box instead of inverting it).
    q = _QMAX + 0.1
    lb, ub = hard_accel_box(q, np.full(_N, 1.5), **_KW)
    assert np.all(lb <= ub + 1e-12)


# ── slew limit ───────────────────────────────────────────────────────────────

def test_slew_intersection():
    lb = np.full(_N, -10.0)
    ub = np.full(_N, 10.0)
    prev = np.full(_N, 2.0)
    lo, hi = apply_slew_limit(lb, ub, prev, 5.0)
    assert np.allclose(lo, -3.0) and np.allclose(hi, 7.0)


def test_slew_pins_when_safety_above():
    # Safety box demands ≥ 9 but prev+Δ = 7 → pinned at 7 (max admissible rate)
    lo, hi = apply_slew_limit(np.full(_N, 9.0), np.full(_N, 10.0),
                              np.full(_N, 2.0), 5.0)
    assert np.allclose(lo, 7.0) and np.allclose(hi, 7.0)


def test_slew_pins_when_safety_below():
    lo, hi = apply_slew_limit(np.full(_N, -10.0), np.full(_N, -9.0),
                              np.full(_N, 2.0), 5.0)
    assert np.allclose(lo, -3.0) and np.allclose(hi, -3.0)


def test_slew_never_empty_random():
    rng = np.random.default_rng(3)
    for _ in range(200):
        lb = rng.uniform(-20, 20, _N)
        ub = lb + rng.uniform(0, 10, _N)
        prev = rng.uniform(-15, 15, _N)
        lo, hi = apply_slew_limit(lb, ub, prev, 2.0)
        assert np.all(lo <= hi + 1e-12)
        # And always inside the slew box
        assert np.all(lo >= prev - 2.0 - 1e-12)
        assert np.all(hi <= prev + 2.0 + 1e-12)


# ── workspace rows ───────────────────────────────────────────────────────────

def test_workspace_no_rows_when_deep_inside():
    p = np.array([0.4, 0.0, 0.5])
    Jp = np.zeros((3, _N)); Jp[:, :3] = np.eye(3)
    rows = workspace_face_rows(p, Jp, np.zeros(3),
                               np.array([-1.0, -1.0, 0.0]),
                               np.array([2.0, 1.0, 1.5]),
                               margin=0.05, horizon=0.3)
    assert rows == []


def test_workspace_row_signs():
    # Near the z+ face: barrier h = ws_max_z − margin − p_z, a = −Jp[2]
    p = np.array([0.4, 0.0, 0.9])
    Jp = np.zeros((3, _N)); Jp[:, :3] = np.eye(3)
    jdq = np.array([0.0, 0.0, 0.3])
    rows = workspace_face_rows(p, Jp, jdq,
                               np.array([-1.0, -1.0, 0.0]),
                               np.array([2.0, 1.0, 1.0]),
                               margin=0.05, horizon=0.3)
    assert len(rows) == 1
    a, h, jdq_r, label = rows[0]
    assert label == 'ws:z+'
    assert abs(h - (1.0 - 0.05 - 0.9)) < 1e-12
    assert np.allclose(a[:3], [0, 0, -1.0])       # pushes p_z DOWN
    assert abs(jdq_r + 0.3) < 1e-12               # sign flipped with the row


def test_workspace_row_near_lower_face():
    p = np.array([0.4, 0.0, 0.06])
    Jp = np.zeros((3, _N)); Jp[:, :3] = np.eye(3)
    rows = workspace_face_rows(p, Jp, np.zeros(3),
                               np.array([-1.0, -1.0, 0.0]),
                               np.array([2.0, 1.0, 1.0]),
                               margin=0.05, horizon=0.3)
    labels = [r[3] for r in rows]
    assert labels == ['ws:z-']
    a, h, _, _ = rows[0]
    assert abs(h - 0.01) < 1e-12
    assert np.allclose(a[:3], [0, 0, 1.0])        # pushes p_z UP


if __name__ == '__main__':
    import sys
    mod = sys.modules['__main__']
    fails = 0
    for name in sorted(dir(mod)):
        if name.startswith('test_'):
            try:
                getattr(mod, name)()
                print(f'PASS {name}')
            except AssertionError as exc:
                fails += 1
                print(f'FAIL {name}: {exc}')
    sys.exit(1 if fails else 0)


# ── position_velocity_accel_box: the wrapper cbf_safety_filter actually calls ──
#
# It must be hard_accel_box exactly — the point of the wrapper is the calling
# convention (in-place buffers + the (ratio, bite) diagnostics the CBFDIAG and
# VELHI log lines read), never different math.

# Real FR3 numbers from franka_description; prefixed so they cannot clobber
# the synthetic fixtures the hard_accel_box tests above rely on.
_FR3_ACC  = np.array([6.0, 2.585, 3.5, 4.0, 17.0, 5.5, 17.0])   # decel_max, official
_FR3_VEL   = np.array([2.62, 2.62, 2.62, 2.62, 5.26, 4.18, 5.26])
_FR3_QMIN = np.array([-2.9007, -1.8361, -2.9007, -3.0770, -2.8763, 0.4398, -3.0508])
_FR3_QMAX = np.array([2.9007, 1.8361, 2.9007, -0.1169, 2.8763, 4.6216, 3.0508])
_FR3_KW = dict(acc_lb=-_FR3_ACC, acc_ub=_FR3_ACC, qdot_max=_FR3_VEL, v_margin=0.9,
           q_min=_FR3_QMIN, q_max=_FR3_QMAX, q_margin=0.05, brake_eta=0.6, dt=0.01)


def _pv_box(q, qdot):
    lb, ub = np.zeros(7), np.zeros(7)
    ratio, bite = position_velocity_accel_box(q, qdot, out_lb=lb, out_ub=ub, **_FR3_KW)
    return lb, ub, ratio, bite


def test_pv_box_matches_hard_accel_box_exactly():
    rng = np.random.default_rng(0)
    for _ in range(3000):
        q    = _FR3_QMIN + rng.random(7) * (_FR3_QMAX - _FR3_QMIN)
        qdot = rng.uniform(-1.0, 1.0, 7) * _FR3_VEL
        ref_lb, ref_ub = hard_accel_box(q, qdot, **_FR3_KW)
        lb, ub, _, _ = _pv_box(q, qdot)
        assert np.array_equal(lb, ref_lb) and np.array_equal(ub, ref_ub)


def test_pv_box_never_returns_an_inverted_box():
    """QP feasibility depends on this: lb > ub would empty the box."""
    rng = np.random.default_rng(1)
    for _ in range(3000):
        # Deliberately include states already past a limit / over speed.
        q    = _FR3_QMIN - 0.2 + rng.random(7) * (_FR3_QMAX - _FR3_QMIN + 0.4)
        qdot = rng.uniform(-1.4, 1.4, 7) * _FR3_VEL
        lb, ub, _, _ = _pv_box(q, qdot)
        assert np.all(lb <= ub)


def test_pv_box_diagnostics_shape_and_meaning():
    q = 0.5 * (_FR3_QMIN + _FR3_QMAX)   # zeros(7) is OUTSIDE joint4/joint6 range
    qdot = 0.5 * _FR3_VEL
    _, _, ratio, bite = _pv_box(q, qdot)
    assert ratio.shape == (7,) and bite.shape == (7,) and bite.dtype == bool
    assert np.allclose(ratio, 0.5)
    assert not bite.any(), 'mid-range at half speed must not tighten the box'


def test_position_limit_caps_approach_velocity():
    """The behaviour velocity_accel_box could not provide at all."""
    J = 1                                   # joint2, q_max = 1.8361
    q, qdot = np.zeros(7), np.zeros(7)
    q[J], qdot[J] = 1.70, 0.5               # close to the limit, moving into it
    lb, ub, _, bite = _pv_box(q, qdot)
    assert ub[J] < _FR3_ACC[J], 'approaching a position limit must tighten ub'
    assert bite[J]
    assert lb[J] == -_FR3_ACC[J], 'braking authority away from the limit is untouched'

    # The velocity-only box has no idea the limit exists.
    old_lb, old_ub = -_FR3_ACC.copy(), _FR3_ACC.copy()
    velocity_accel_box(qdot, acc_lb=-_FR3_ACC, acc_ub=_FR3_ACC, qdot_max=_FR3_VEL,
                       v_margin=0.9, dt=0.01, out_lb=old_lb, out_ub=old_ub)
    assert old_ub[J] == _FR3_ACC[J]


def test_past_the_barrier_forbids_moving_further_in():
    J = 1
    q, qdot = np.zeros(7), np.zeros(7)
    q[J] = _FR3_QMAX[J] - 0.01                  # inside q_margin = 0.05
    lb, ub, _, _ = _pv_box(q, qdot)
    assert ub[J] <= 0.0, 'no positive accel allowed past the barrier'
    assert lb[J] == -_FR3_ACC[J]


def test_far_from_every_limit_the_box_is_static():
    q, qdot = np.zeros(7), np.zeros(7)
    q[:] = 0.5 * (_FR3_QMIN + _FR3_QMAX)            # mid-range
    lb, ub, _, bite = _pv_box(q, qdot)
    assert np.allclose(lb, -_FR3_ACC) and np.allclose(ub, _FR3_ACC)
    assert not bite.any()
