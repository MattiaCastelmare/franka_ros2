"""HOCBF rows for joint limits and self-collision, and what they fix.

The regression these guard against is concrete and was measured on hardware:
joint4 sat exactly on its position braking curve, the box collapsed to
``lb = 0``, the commander kept asking for -4 rad/s², and the QP dumped the whole
4 rad/s² correction on that one joint (``dnorm=10.989``, ``dq_ort=10.941``).
The arm could not track it (``trk_err=8.428``) and the firmware aborted with
``joint_velocity_violation``.

A row cannot prevent that on its own — the box is still the hard floor — but it
starts pushing long before the wall, and the QP distributes the correction
instead of saturating one joint. The last test here is the one that shows it.

Pure numpy + scipy. Run with pytest, or directly.
"""

import numpy as np
from scipy.optimize import linprog

from franka_experiments.utils.cbf_hard_limits import (
    apply_slew_limit,
    hard_accel_box,
)
from franka_experiments.utils.cbf_state_rows import joint_limit_rows
from franka_experiments.utils.self_collision import (
    Capsule,
    build_capsule_pairs,
    segment_segment_closest,
)

NV = 7
K0, K1 = 25.0, 10.5

# Official FR3, franka_description/robots/fr3/joint_limits.yaml
Q_MIN = np.array([-2.9007, -1.8361, -2.9007, -3.0770, -2.8763, 0.4398, -3.0508])
Q_MAX = np.array([2.9007, 1.8361, 2.9007, -0.1169, 2.8763, 4.6216, 3.0508])
QDD = np.array([6.0, 2.585, 3.5, 4.0, 17.0, 5.5, 17.0])
QD = np.array([2.62, 2.62, 2.62, 2.62, 5.26, 4.18, 5.26])


# ── joint-limit rows ─────────────────────────────────────────────────────────

def test_no_rows_in_the_middle_of_the_range():
    q = 0.5 * (Q_MIN + Q_MAX)
    assert joint_limit_rows(q, Q_MIN, Q_MAX, 0.10, 0.60, NV) == []


def test_one_row_per_approached_limit_with_correct_sign():
    q = 0.5 * (Q_MIN + Q_MAX)
    q[3] = Q_MIN[3] + 0.30                      # joint4 near its LOWER limit
    rows = joint_limit_rows(q, Q_MIN, Q_MAX, 0.10, 0.60, NV)
    assert len(rows) == 1
    a, h, jdq, lbl = rows[0]
    assert lbl == 'q4-'
    assert jdq == 0.0, 'hdd = +qdd carries no drift term'
    assert abs(h - (0.30 - 0.10)) < 1e-12
    # Lower barrier: h grows as q grows, so a = +e_4.
    expect = np.zeros(NV); expect[3] = 1.0
    assert np.array_equal(a, expect)


def test_upper_limit_row_has_the_opposite_sign():
    q = 0.5 * (Q_MIN + Q_MAX)
    q[3] = Q_MAX[3] - 0.30
    rows = joint_limit_rows(q, Q_MIN, Q_MAX, 0.10, 0.60, NV)
    assert len(rows) == 1
    a, h, _, lbl = rows[0]
    assert lbl == 'q4+'
    expect = np.zeros(NV); expect[3] = -1.0
    assert np.array_equal(a, expect)


def test_barrier_goes_negative_past_the_margin():
    """h < 0 inside the margin — the row then demands active retreat."""
    q = 0.5 * (Q_MIN + Q_MAX)
    q[3] = Q_MIN[3] + 0.04                      # inside margin 0.10
    (_, h, _, lbl), = joint_limit_rows(q, Q_MIN, Q_MAX, 0.10, 0.60, NV)
    assert lbl == 'q4-' and h < 0.0


def test_row_ordering_is_deterministic():
    """OSQP warm-starts on a fixed sparsity pattern; order must not wobble."""
    q = Q_MIN + 0.25
    a = [r[3] for r in joint_limit_rows(q, Q_MIN, Q_MAX, 0.10, 0.60, NV)]
    b = [r[3] for r in joint_limit_rows(q, Q_MIN, Q_MAX, 0.10, 0.60, NV)]
    assert a == b == sorted(a, key=lambda s: (int(s[1]), s[2]))


def test_horizon_caps_the_row_count():
    q = 0.5 * (Q_MIN + Q_MAX)
    wide = joint_limit_rows(q, Q_MIN, Q_MAX, 0.10, 10.0, NV)
    assert len(wide) == 2 * NV, 'a huge horizon emits both barriers per joint'
    assert joint_limit_rows(q, Q_MIN, Q_MAX, 0.10, 0.01, NV) == []


# ── self-collision row geometry ──────────────────────────────────────────────

def test_relative_jacobian_is_the_difference_not_one_side():
    """Both capsules move, so h-dot needs J_a - J_b.

    Built explicitly here rather than through Pinocchio: if both closest points
    translate together, the gap does not change and the row must be identically
    zero. Using only one link's Jacobian would produce a spurious non-zero row
    and steer the arm for no reason.
    """
    n = np.array([1.0, 0.0, 0.0])
    J_a = np.tile(np.array([[1.0], [0.0], [0.0]]), (1, NV))
    J_b = J_a.copy()                             # identical motion
    a = n @ (J_a - J_b)
    assert np.allclose(a, 0.0)

    J_b = np.zeros((3, NV))                      # only A moves
    a = n @ (J_a - J_b)
    assert np.allclose(a, 1.0)


def test_gap_and_normal_from_closest_points():
    """h = ||pa-pb|| - r_a - r_b, and n points from B to A."""
    p1, q1 = np.array([0.0, 0, 0]), np.array([1.0, 0, 0])
    p2, q2 = np.array([0.0, 0.5, 0]), np.array([1.0, 0.5, 0])
    pa, pb, d = segment_segment_closest(p1, q1, p2, q2)
    assert abs(d - 0.5) < 1e-12
    gap = d - 0.1 - 0.15
    assert abs(gap - 0.25) < 1e-12
    n = (pa - pb) / np.linalg.norm(pa - pb)
    assert np.allclose(n, [0.0, -1.0, 0.0])


def test_srdf_exclusion_removes_the_pair_that_stalled_the_arm():
    spec = [('fr3_link0', .06), ('fr3_link1', .06), ('fr3_link2', .06),
            ('fr3_link3', .06), ('fr3_link4', .06), ('fr3_link5', .06),
            ('fr3_link5', .025), ('fr3_link6', .05), ('fr3_link7', .04),
            ('fr3_link7', .03), ('fr3_hand', .06), ('fr3_hand', .04)]
    caps = [Capsule(f'{b}_sc', b, np.zeros(3), np.ones(3), r) for b, r in spec]
    srdf = ['link0-link2', 'link0-link3', 'link0-link4', 'link1-link3',
            'link1-link4', 'link2-link4', 'link2-link6', 'link3-link5',
            'link3-link6', 'link3-link7', 'link4-link6', 'link4-link7',
            'link5-link7', 'hand-link3', 'hand-link4', 'hand-link6']
    before = build_capsule_pairs(caps)
    after = build_capsule_pairs(caps, srdf)
    assert len(before) == 47 and len(after) == 24
    bodies = {(caps[i].body, caps[j].body) for i, j in after}
    assert not any({a, b} == {'fr3_link1', 'fr3_link3'} for a, b in bodies)
    # The genuinely reachable ones must survive.
    assert any({a, b} == {'fr3_link0', 'fr3_hand'} for a, b in bodies)


# ── the point of the change: a row lets the QP compromise ────────────────────

def _solve(qddot_nom, G=None, h_qp=None, box_lb=None, box_ub=None, rho=1000.0):
    """The node's QP, small and dense: min ½||qdd-nom||² + ½ρs² s.t. rows, box.

    Solved with SLSQP rather than OSQP so the test needs no solver install; the
    problem is 8 variables and a handful of rows, so accuracy is not a concern.
    """
    from scipy.optimize import minimize
    n = NV + 1
    lb = np.append(box_lb, 0.0)
    ub = np.append(box_ub, 1e6)
    tgt = np.asarray(qddot_nom, dtype=float)
    cons = []
    if G is not None and len(G):
        A_ub = np.asarray(G, dtype=float).reshape(-1, n)
        b_ub = np.asarray(h_qp, dtype=float).reshape(-1)
        cons.append({'type': 'ineq', 'fun': lambda x: b_ub - A_ub @ x})

    def f(x):
        d = x[:NV] - tgt
        return 0.5 * d @ d + 0.5 * rho * x[-1] ** 2

    x0 = np.append(np.clip(qddot_nom, box_lb, box_ub), 0.0)
    res = minimize(f, x0, bounds=list(zip(lb, ub)), constraints=cons,
                   method='SLSQP', options={'maxiter': 400, 'ftol': 1e-12})
    assert res.success, res.message
    return res.x[:NV]


def test_row_acts_while_the_box_is_still_wide_open():
    """The whole point of the change, in one assertion.

    The box is a wall: it does nothing until the joint is ON the braking curve,
    then clips that joint to zero in a single tick. On hardware that produced
    dnorm=10.989 with dq_ort=10.941 — the entire correction dumped on joint4 —
    and a command the arm could not track.

    The row engages far earlier, while the box is STILL FULLY OPEN, and asks for
    a proportionate reduction the QP can absorb smoothly.
    """
    J = 3
    q = 0.5 * (Q_MIN + Q_MAX)
    qdot = np.zeros(NV)
    q[J] = Q_MIN[J] + 0.05 + 0.55        # 0.55 rad from the BOX barrier
    qdot[J] = -1.0                        # closing on the lower limit
    qddot_nom = np.zeros(NV)
    qddot_nom[J] = -4.0                   # commander pushing into the limit

    box_lb, box_ub = hard_accel_box(
        q, qdot, acc_lb=-QDD, acc_ub=QDD, qdot_max=QD, v_margin=0.9,
        q_min=Q_MIN, q_max=Q_MAX, q_margin=0.05, brake_eta=0.6, dt=0.01)
    assert np.allclose(box_lb, -QDD) and np.allclose(box_ub, QDD), \
        'precondition: at this distance the box must still be fully open'

    # (a) box only: nothing stops the commander. qddot4 passes through at -4.
    qdd_box = _solve(qddot_nom, box_lb=box_lb, box_ub=box_ub)
    assert abs(qdd_box[J] - (-4.0)) < 1e-6

    # (b) with the row: h = 0.55 - 0.10 = 0.45 is inside the 0.60 horizon, so a
    #     row exists and pulls the command back — smoothly, not to zero.
    rows = joint_limit_rows(q, Q_MIN, Q_MAX, 0.10, 0.60, NV)
    assert len(rows) == 1 and rows[0][3] == 'q4-'
    a, h, jdq, _ = rows[0]
    g = np.zeros(NV + 1); g[:NV] = -a; g[-1] = -1.0
    h_qp = K1 * (a @ qdot) + K0 * h + jdq
    qdd_row = _solve(qddot_nom, G=[g], h_qp=[h_qp],
                     box_lb=box_lb, box_ub=box_ub)

    assert qdd_row[J] > qdd_box[J] + 0.5, \
        f'the row must brake the approach: {qdd_row[J]:.3f} vs {qdd_box[J]:.3f}'
    assert qdd_row[J] < 0.0, 'but not reverse the joint outright'
    # And it must not disturb any joint that is nowhere near a limit.
    others = [k for k in range(NV) if k != J]
    assert np.allclose(qdd_row[others], 0.0, atol=1e-6)


def test_row_demands_hard_braking_once_on_the_box_curve():
    """Deep in the approach the row and the box agree on the sign.

    Same state the hardware log captured (joint4 exactly on its curve). The box
    alone permits qddot4 = 0 — coasting into the limit at 1.07 rad/s. The row
    computes a strictly POSITIVE demand, i.e. actually retreat.
    """
    J = 3
    h_curve = 0.240
    q = 0.5 * (Q_MIN + Q_MAX)
    qdot = np.zeros(NV)
    q[J] = Q_MIN[J] + 0.05 + h_curve
    qdot[J] = -np.sqrt(2.0 * 0.6 * QDD[J] * h_curve)
    assert abs(qdot[J] + 1.074) < 1e-3, 'matches the logged -1.07 rad/s'

    box_lb, _ = hard_accel_box(
        q, qdot, acc_lb=-QDD, acc_ub=QDD, qdot_max=QD, v_margin=0.9,
        q_min=Q_MIN, q_max=Q_MAX, q_margin=0.05, brake_eta=0.6, dt=0.01)
    assert abs(box_lb[J]) < 1e-6, 'the box has collapsed to lb = 0: coast only'

    (a, h, jdq, lbl), = joint_limit_rows(q, Q_MIN, Q_MAX, 0.10, 0.60, NV)
    assert lbl == 'q4-'
    # Row is  a·qdd + s >= -h_qp ; the demanded acceleration at s = 0.
    demand = -(K1 * (a @ qdot) + K0 * h + jdq)
    assert demand > 0.0, (
        f'on the curve the row must demand positive (retreating) accel, '
        f'got {demand:.3f}')
    assert demand > box_lb[J], 'strictly stronger than what the box allows'


def test_joint_limit_row_is_feasible_across_the_whole_range():
    """Rows must never make the QP infeasible, including past the margin."""
    rng = np.random.default_rng(0)
    for _ in range(400):
        q = Q_MIN - 0.05 + rng.random(NV) * (Q_MAX - Q_MIN + 0.10)
        qdot = rng.uniform(-0.6, 0.6, NV) * QD
        rows = joint_limit_rows(q, Q_MIN, Q_MAX, 0.10, 0.60, NV)
        box_lb, box_ub = hard_accel_box(
            q, qdot, acc_lb=-QDD, acc_ub=QDD, qdot_max=QD, v_margin=0.9,
            q_min=Q_MIN, q_max=Q_MAX, q_margin=0.05, brake_eta=0.6, dt=0.01)
        n = NV + 1
        A = [np.concatenate([-a, [-1.0]]) for a, _, _, _ in rows]
        b = [K1 * (a @ qdot) + K0 * h + jdq for a, h, jdq, _ in rows]
        lo = np.append(box_lb, 0.0); hi = np.append(box_ub, 1e6)
        A_ub = np.array(A).reshape(-1, n)
        res = linprog(np.zeros(n), A_ub=A_ub if A_ub.size else None,
                      b_ub=np.array(b) if A_ub.size else None,
                      bounds=list(zip(lo, hi)), method='highs')
        assert res.status == 0, 'shared slack must keep this feasible'


if __name__ == '__main__':
    import pytest
    raise SystemExit(pytest.main([__file__, '-v']))


# ── Regressions from the two hardware aborts ─────────────────────────────────

def test_row_margin_must_not_steal_usable_range():
    """joint6 works at 4.51 rad; a 0.10 barrier parked the row on top of it.

    Reconstructed from the log: the q6+ row reported hdot=-0.060 and
    h_qp=-0.357, i.e. h = (h_qp - k1*hdot)/k0 = +0.011 rad. With margin 0.10
    that means q6 = 4.511 — 0.11 rad clear of its official 4.6216 limit and
    entirely legal. The row nevertheless demanded qddot6 <= -0.357 EVERY tick,
    which is what produced the sustained dnorm~10 and the lost tracking.
    """
    J = 5                                    # joint6
    q = 0.5 * (Q_MIN + Q_MAX)
    q[J] = 4.5107                            # the pose the trajectory needs
    true_clearance = Q_MAX[J] - q[J]
    assert abs(true_clearance - 0.1109) < 1e-3

    # Old default: barrier 0.10 inside → the joint is ON it, h ~ 0.
    (_, h_bad, _, lbl), = [r for r in joint_limit_rows(
        q, Q_MIN, Q_MAX, 0.10, 0.60, NV) if r[3] == 'q6+']
    assert lbl == 'q6+'
    assert abs(h_bad - 0.0109) < 1e-3, h_bad
    demand_bad = -(K1 * (-0.060) + K0 * h_bad)
    assert demand_bad > 0.3, 'the old default demanded active retreat'

    # New default: barrier AT the official limit. The row still exists (0.11 is
    # inside the 0.30 horizon) — that is fine and wanted, it is watching the
    # approach. What matters is that it is PERMISSIVE: h is the true clearance,
    # so the demand is negative, i.e. the QP is free to keep the joint there.
    (_, h_ok, _, _), = [r for r in joint_limit_rows(
        q, Q_MIN, Q_MAX, 0.0, 0.30, NV) if r[3] == 'q6+']
    assert abs(h_ok - true_clearance) < 1e-9, 'h must be the real clearance'
    demand_ok = -(K1 * (-0.060) + K0 * h_ok)
    assert demand_ok < 0.0, (
        f'a legal pose must demand no retreat, got {demand_ok:+.3f} '
        f'(the old default demanded {demand_bad:+.3f})')
    assert h_ok > h_bad


def test_zero_margin_never_removes_range_on_any_joint():
    """With margin 0 the barrier is the official limit, on every joint."""
    for j in range(NV):
        q = 0.5 * (Q_MIN + Q_MAX)
        q[j] = Q_MAX[j] - 1e-6              # a hair inside the real limit
        rows = [r for r in joint_limit_rows(q, Q_MIN, Q_MAX, 0.0, 10.0, NV)
                if r[3] == f'q{j + 1}+']
        assert len(rows) == 1 and rows[0][1] >= -1e-6, \
            f'joint{j + 1}: h must not be negative at a legal pose'


def test_velocity_box_engages_before_the_cap_with_relax():
    """joint5 ramped 39% -> 70% with vbite empty on every line, then aborted.

    The one-step box only acts in the final q̈_max·dt = 0.17 rad/s. The relax
    horizon makes it act proportionally on the whole approach.
    """
    J = 4                                    # joint5, qdot_max 5.26
    q = 0.5 * (Q_MIN + Q_MAX)
    qdot = np.zeros(NV)
    qdot[J] = 3.68                           # 70% — the value at the abort
    cap = 0.9 * QD[J]
    assert cap - qdot[J] > QDD[J] * 0.01, 'precondition: outside the one-step sliver'

    kw = dict(acc_lb=-QDD, acc_ub=QDD, qdot_max=QD, v_margin=0.9,
              q_min=Q_MIN, q_max=Q_MAX, q_margin=0.05, brake_eta=0.6, dt=0.01)

    # Legacy: one step. The box is wide open on joint5.
    _, ub_old = hard_accel_box(q, qdot, **kw)
    assert abs(ub_old[J] - QDD[J]) < 1e-9, 'the one-step box does not bite here'

    # With a 0.10 s approach horizon it does, and proportionally.
    lb_new, ub_new = hard_accel_box(q, qdot, relax_dt=0.10, **kw)
    assert ub_new[J] < QDD[J], 'the relaxed box must engage before the cap'
    assert abs(ub_new[J] - (cap - qdot[J]) / 0.10) < 1e-9
    assert ub_new[J] > 0.0, 'still allowed to accelerate, just less'
    assert lb_new[J] == -QDD[J], 'braking authority untouched'


def test_relax_never_softens_braking_past_a_cap():
    """Over the cap the one-step dt must come back, at full authority."""
    J = 4
    q = 0.5 * (Q_MIN + Q_MAX)
    qdot = np.zeros(NV)
    qdot[J] = 0.95 * QD[J]                   # past 0.9*qdot_max
    kw = dict(acc_lb=-QDD, acc_ub=QDD, qdot_max=QD, v_margin=0.9,
              q_min=Q_MIN, q_max=Q_MAX, q_margin=0.05, brake_eta=0.6, dt=0.01)
    _, ub_fast = hard_accel_box(q, qdot, relax_dt=0.10, **kw)
    _, ub_ref = hard_accel_box(q, qdot, **kw)
    assert ub_fast[J] == ub_ref[J], 'past the cap relax must not apply'
    assert ub_fast[J] < 0.0, 'and it must demand deceleration'


def test_relax_default_is_bit_identical_to_legacy():
    rng = np.random.default_rng(3)
    kw = dict(acc_lb=-QDD, acc_ub=QDD, qdot_max=QD, v_margin=0.9,
              q_min=Q_MIN, q_max=Q_MAX, q_margin=0.05, brake_eta=0.6, dt=0.01)
    for _ in range(500):
        q = Q_MIN + rng.random(NV) * (Q_MAX - Q_MIN)
        qdot = rng.uniform(-1, 1, NV) * QD
        a = hard_accel_box(q, qdot, **kw)
        b = hard_accel_box(q, qdot, relax_dt=None, **kw)
        assert np.array_equal(a[0], b[0]) and np.array_equal(a[1], b[1])


def test_row_count_stays_bounded_over_the_workspace():
    """n_c hit 18 against 7 DOF. The narrower horizon must keep it sane."""
    rng = np.random.default_rng(7)
    worst_old = worst_new = 0
    for _ in range(3000):
        q = Q_MIN + rng.random(NV) * (Q_MAX - Q_MIN)
        worst_old = max(worst_old, len(joint_limit_rows(q, Q_MIN, Q_MAX, 0.10, 0.60, NV)))
        worst_new = max(worst_new, len(joint_limit_rows(q, Q_MIN, Q_MAX, 0.0, 0.30, NV)))
    # Measured over the reachable range, not guessed: the point is the drop,
    # and that the count stays well under the 7 DOF the QP has to satisfy them.
    print(f'    worst-case joint-limit rows: {worst_old} (old) -> {worst_new} (new)')
    assert worst_new < worst_old, (worst_old, worst_new)
    assert worst_new <= 6, f'joint-limit rows must stay bounded, got {worst_new}'


# ── Slew box: the command-discontinuity failure ──────────────────────────────

DELTA = 5.0          # max_qddot_delta, rad/s² per tick at 100 Hz


def _boxes(q, qdot):
    return hard_accel_box(q, qdot, acc_lb=-QDD, acc_ub=QDD, qdot_max=QD,
                          v_margin=0.9, q_min=Q_MIN, q_max=Q_MAX,
                          q_margin=0.05, brake_eta=0.6, dt=0.01, relax_dt=0.10)


def test_slew_box_bounds_the_measured_jump():
    """dnorm went 5.278 -> 12.742 in one 60 ms window; the arm could not follow.

    Reproduced as a per-joint step of the same magnitude: the slew box caps how
    far the command may move from the previous one, per tick.
    """
    q = 0.5 * (Q_MIN + Q_MAX)
    qdot = np.zeros(NV)
    lb, ub = _boxes(q, qdot)

    prev = np.zeros(NV)
    lo, hi = apply_slew_limit(lb, ub, prev, DELTA)
    # The state box alone allows 17 on the wrist joints; the slew box caps the
    # per-tick move at DELTA regardless.
    assert np.all(hi <= prev + DELTA + 1e-9)
    assert np.all(lo >= prev - DELTA - 1e-9)
    assert hi[4] < QDD[4], 'joint5 (qddot_max 17) must be slew-limited from rest'


def test_slew_never_impedes_full_braking_authority():
    """A safety filter must not be slowed down when it needs to brake."""
    q = 0.5 * (Q_MIN + Q_MAX)
    qdot = np.zeros(NV)
    lb, ub = _boxes(q, qdot)
    prev = np.zeros(NV)
    lo, _ = apply_slew_limit(lb, ub, prev, DELTA)
    # joint4's full authority is 4.0 < DELTA, so it is reachable in ONE tick.
    assert lo[3] <= -QDD[3] + 1e-9, 'joint4 must reach full brake in one tick'
    # The wrist joints (17) take ceil(17/5) = 4 ticks; verify the walk gets there.
    v = 0.0
    for _ in range(4):
        p = np.zeros(NV); p[4] = v
        l, _h = apply_slew_limit(lb, ub, p, DELTA)
        v = l[4]
    assert abs(v + QDD[4]) < 1e-9, f'joint5 must reach -17 in 4 ticks, got {v}'


def test_slew_box_is_never_empty_even_when_disjoint():
    """The state box can move beyond reach in one tick; the QP must stay feasible."""
    q = 0.5 * (Q_MIN + Q_MAX)
    qdot = np.zeros(NV)
    lb, ub = _boxes(q, qdot)
    # Previous command far outside the current safety box on every joint.
    for prev_val in (-50.0, +50.0):
        prev = np.full(NV, prev_val)
        lo, hi = apply_slew_limit(lb, ub, prev, DELTA)
        assert np.all(lo <= hi + 1e-12), 'slew must never invert the box'


def test_slew_walks_toward_an_unreachable_bound_at_max_rate():
    """Disjoint case: approach the safety bound, do not jump to it."""
    lb = np.full(NV, 10.0)          # safety box demands >= +10
    ub = np.full(NV, 12.0)
    prev = np.zeros(NV)             # but we are at 0
    lo, hi = apply_slew_limit(lb, ub, prev, DELTA)
    assert np.allclose(lo, DELTA) and np.allclose(hi, DELTA), \
        'must pin to the slew edge nearest the safety box'
    # Next tick it advances another DELTA, and so on until it arrives.
    lo2, _ = apply_slew_limit(lb, ub, lo, DELTA)
    assert np.allclose(lo2, 2 * DELTA)
    lo3, _ = apply_slew_limit(lb, ub, lo2, DELTA)
    assert np.allclose(lo3, 10.0), 'arrives at the safety bound, not past it'


def test_slew_disabled_reproduces_the_plain_state_box():
    q = 0.5 * (Q_MIN + Q_MAX)
    rng = np.random.default_rng(11)
    for _ in range(200):
        qdot = rng.uniform(-0.5, 0.5, NV) * QD
        lb, ub = _boxes(q, qdot)
        prev = rng.uniform(-3, 3, NV)
        lo, hi = apply_slew_limit(lb, ub, prev, 1e6)   # effectively no limit
        assert np.allclose(lo, lb) and np.allclose(hi, ub)


# ── Anticipation-term filter ─────────────────────────────────────────────────
#
# hdot = a^T qdot is a derivative of a measured signal, multiplied by k1 = 10.5
# straight into h_qp. Measured on hardware, that one term covered 159% of
# h_qp's entire swing and the commanded acceleration flipped sign 9 times in 28
# intervals. These tests pin what the filter does and, just as importantly,
# what it must NOT touch.

def _ema(seq, alpha):
    out, y = [], 0.0
    for v in seq:
        y = alpha * y + (1.0 - alpha) * v if alpha > 0.0 else v
        out.append(y)
    return np.array(out)


# The hdot sequence read off the hardware log, in order.
HDOT_LOG = np.array([0.323, 0.308, 0.294, 0.259, 0.228, 0.231, 0.219, 0.095,
                     0.194, 0.105, 0.035, 0.169, 0.137, 0.141, 0.043, 0.121,
                     0.142, 0.184, 0.188, 0.193, 0.194, 0.192, 0.162, 0.204])


def test_filter_cuts_the_swing_that_drove_the_oscillation():
    raw_swing = float(HDOT_LOG.max() - HDOT_LOG.min())
    filt = _ema(HDOT_LOG, 0.6)[3:]          # skip the start-up transient
    filt_swing = float(filt.max() - filt.min())
    assert raw_swing > 0.28, raw_swing
    assert filt_swing < 0.6 * raw_swing, (
        f'the filter must materially reduce the swing: {raw_swing:.3f} -> '
        f'{filt_swing:.3f}')
    # And what that means at the constraint: k1 = 10.5 multiplies it.
    assert 10.5 * (raw_swing - filt_swing) > 1.0, 'worth more than 1.0 of h_qp'


def test_alpha_zero_is_exactly_the_legacy_behaviour():
    assert np.array_equal(_ema(HDOT_LOG, 0.0), HDOT_LOG)


def test_filter_is_a_lag_not_a_bias():
    """A constant approach speed must be tracked exactly, just later."""
    const = np.full(400, 0.25)
    out = _ema(const, 0.6)
    assert abs(out[-1] - 0.25) < 1e-9, 'no steady-state error'
    assert out[5] < 0.25, 'but it does lag on the way in'


def test_lag_is_small_against_d_safe():
    """Quantify the safety cost: how much extra closure the lag allows."""
    alpha, dt, approach = 0.6, 0.01, 1.0     # 1 m/s, the human-motion scale
    tau = alpha * dt / (1.0 - alpha)         # EMA time constant
    extra_closure = approach * tau
    assert extra_closure < 0.02, extra_closure
    assert extra_closure < 0.20 / 5.0, 'must stay well under d_safe = 0.20 m'


# ── Asymmetric barrier smoothing (obstacle rows only) ────────────────────────
#
# Obstacle h comes from the camera at ~30 Hz while the QP runs at 100 Hz, so it
# sits frozen for a tick or two and then jumps: measured, d_min repeated its
# exact value on 7 of 26 consecutive diagnostic lines and then moved 24 mm,
# which k0 = 25 turns into a 0.60 rad/s^2 step in the constraint.
#
# Closing is instant, recovery is rate-limited. The first attempt at this
# EXTRAPOLATED h forward with hdot = a^T qdot instead; that assumes a static
# obstacle, so when the human is the one moving it mispredicts and the
# correction at the next frame is bigger than the step it was removing. The
# logged sequence below is the one that exposed it.

H_LOG = [0.124, 0.112, 0.121, 0.110, 0.116, 0.121, 0.124, 0.125, 0.123]
FRAMES_PER_TICK = 3          # 30 Hz perception vs 100 Hz QP
DT_CBF = 1.0 / 50.0


def _smooth_run(h_updates, alpha, per_frame=FRAMES_PER_TICK):
    """Replay perception frames at rebuild rate through the asymmetric rule."""
    out, prev = [], None
    for h_raw in h_updates:
        for _ in range(per_frame):
            if prev is None or h_raw <= prev:
                h = h_raw                      # closer, or first sight
            else:
                h = alpha * prev + (1.0 - alpha) * h_raw
            prev = h
            out.append(h)
    return out


def _frozen_run(h_updates, per_frame=FRAMES_PER_TICK):
    return [h for h in h_updates for _ in range(per_frame)]


def _worst_step(seq):
    return max(abs(b - a) for a, b in zip(seq, seq[1:]))


def _worst_rise(seq):
    return max((b - a for a, b in zip(seq, seq[1:])), default=0.0)


def _worst_drop(seq):
    return max((a - b for a, b in zip(seq, seq[1:])), default=0.0)


def test_smoothing_shrinks_the_recovery_staircase_only():
    """It removes the moving-away half of the staircase, by design.

    Approach steps stay instant: a 12 mm drop in measured distance at 30 Hz is
    0.36 m/s of real closing motion, and reacting to it immediately is the whole
    point of the filter. Only the recovery side is held back, which is the half
    that costs nothing to smooth.
    """
    frozen = _frozen_run(H_LOG)
    smooth = _smooth_run(H_LOG, 0.8)
    assert _worst_rise(frozen) > 0.008, 'precondition: the raw data does rise'
    assert _worst_rise(smooth) < 0.5 * _worst_rise(frozen), (
        f'recovery steps must shrink: {_worst_rise(frozen):.4f} -> '
        f'{_worst_rise(smooth):.4f}')
    assert _worst_drop(smooth) == _worst_drop(frozen), (
        'approach steps must be untouched — that reaction is wanted')


def test_lower_k0_is_the_lever_for_the_remaining_approach_jerk():
    """What a perception step does to the constraint, as a function of k0.

    The approach half of the staircase is real motion, so it cannot be filtered
    away. It CAN be made to matter less: h_qp moves by k0 * delta_h, so k0 sets
    how violently a 12 mm perception step shakes the constraint — and lowering
    it also starts the ramp further out, which is the softer onset asked for.
    """
    step = 0.012                              # the logged worst approach step
    k1, d_safe, hdot = 10.5, 0.20, 0.20
    for k0 in (25.0, 15.0):
        jolt = k0 * step
        onset = d_safe + k1 * hdot / k0
        if k0 == 25.0:
            assert abs(jolt - 0.30) < 1e-9 and abs(onset - 0.284) < 1e-3
        else:
            assert jolt < 0.20, jolt          # 40% less shake per step
            assert onset > 0.33, onset        # and it starts much earlier
        assert k1 / (2.0 * np.sqrt(k0)) >= 1.0, 'must stay overdamped'  


def test_extrapolation_would_have_made_it_worse():
    """The rejected approach, on the sequence that exposed it.

    h -> 0.124, 0.112, 0.121: the gap closes then opens, so a static-obstacle
    prediction is wrong in both directions and the frame boundary gets a bigger
    correction than the frozen value it replaced.
    """
    hdot, dt, cap = -0.20, 0.01, 0.04
    extrap = []
    for h_raw in H_LOG:
        for k in range(FRAMES_PER_TICK):
            extrap.append(h_raw + hdot * min(k * dt, cap))
    assert _worst_step(extrap) > _worst_step(_frozen_run(H_LOG)), (
        'this is exactly why extrapolation was rejected: '
        f'{_worst_step(_frozen_run(H_LOG)):.4f} -> {_worst_step(extrap):.4f}')
    # And the chosen rule does not have that failure mode.
    assert _worst_step(_smooth_run(H_LOG, 0.8)) < _worst_step(extrap)


def test_smoothed_value_is_never_more_optimistic_than_the_measurement():
    """The safety property: h_eff <= h_raw at every instant, always."""
    rng = np.random.default_rng(0)
    for _ in range(200):
        seq = list(rng.uniform(0.0, 0.4, 40))
        smooth = _smooth_run(seq, 0.8)
        frozen = _frozen_run(seq)
        assert np.all(np.asarray(smooth) <= np.asarray(frozen) + 1e-12)


def test_closing_is_always_instant():
    """No lag in the direction that matters."""
    seq = [0.30, 0.05]                    # a sudden 25 cm approach
    out = _smooth_run(seq, 0.8)
    assert out[FRAMES_PER_TICK] == 0.05, 'the drop must land on the first tick'


def test_recovery_is_rate_limited_but_arrives():
    seq = [0.05] + [0.30] * 40
    out = _smooth_run(seq, 0.8)
    assert out[FRAMES_PER_TICK] < 0.10, 'must not jump back in one tick'
    assert abs(out[-1] - 0.30) < 1e-9, 'but must get there'
    rises = [b - a for a, b in zip(out, out[1:]) if b > a]
    assert max(rises) < 0.25 * (0.30 - 0.05), 'first step is a fraction of the jump'


def test_zero_alpha_restores_the_raw_staircase():
    assert _smooth_run(H_LOG, 0.0) == _frozen_run(H_LOG)


# ── The class-K ramp is already continuous ───────────────────────────────────

def test_linear_class_k_gives_a_continuous_ramp_not_a_step():
    """The demanded accel rises smoothly from zero, it does not switch on."""
    k0, k1, d_safe, hdot = 25.0, 10.5, 0.20, 0.20
    def demand(d):
        return max(k1 * hdot - k0 * (d - d_safe), 0.0)
    onset = d_safe + k1 * hdot / k0
    assert abs(demand(onset)) < 1e-12, 'zero exactly at onset — no jump'
    ds = np.linspace(onset, d_safe, 50)
    vals = np.array([demand(d) for d in ds])
    assert np.all(np.diff(vals) > 0), 'monotonically stronger as d shrinks'
    steps = np.diff(vals)
    assert steps.max() / steps.min() < 1.05, 'and evenly, with no kink'


def test_lowering_k0_widens_and_softens_the_onset():
    """The lever for "start earlier, push more gently", with damping intact."""
    k1, d_safe, hdot = 10.5, 0.20, 0.20
    onset = lambda k0: d_safe + k1 * hdot / k0
    slope = lambda k0: k0                    # d(demand)/d(-h)
    assert onset(15.0) > onset(25.0), 'lower k0 starts further out'
    assert slope(15.0) < slope(25.0), 'and ramps up more gently'
    # Damping must stay overdamped: zeta = k1 / (2*sqrt(k0)) >= 1.
    for k0 in (15.0, 25.0):
        assert k1 / (2.0 * np.sqrt(k0)) >= 1.0, k0
