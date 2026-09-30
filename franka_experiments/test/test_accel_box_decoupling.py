"""qddot_accel_limits widens the COMMAND box without touching the braking curve
toward the joint limits (brake_acc), which stays at Franka's deceleration_limit."""
import numpy as np

from franka_experiments.utils.cbf_hard_limits import hard_accel_box

DECEL = np.array([6, 2.585, 3.5, 4, 10, 5.5, 10.0])
WIDE = np.array([8, 5, 6, 6, 10, 7, 10.0])
KW = dict(qdot_max=np.full(7, 2.62), v_margin=0.9, q_min=np.full(7, -2.8), q_max=np.full(7, 2.8),
          q_margin=0.05, brake_eta=0.9, dt=0.01, relax_dt=0.1, clip_to_limits=True)


def _box(acc, q, qd, brake=None):
    return hard_accel_box(q, qd, acc_lb=-acc, acc_ub=acc, brake_acc=brake, **KW)


def test_brake_acc_equal_to_the_box_is_the_legacy_arithmetic():
    rng = np.random.default_rng(0)
    for _ in range(50):
        q, qd = rng.uniform(-2.5, 2.5, 7), rng.uniform(-2, 2, 7)
        a, b = _box(DECEL, q, qd), _box(DECEL, q, qd, brake=DECEL)
        np.testing.assert_array_equal(a[0], b[0]); np.testing.assert_array_equal(a[1], b[1])


# A valid FR3 pose (joint4 and joint6 have one-sided ranges; q = 0 is outside them).
Q0 = np.array([0.0, -0.3, 0.0, -2.0, 0.0, 1.9, 0.8])


def test_braking_toward_a_limit_is_unchanged_by_the_wide_box():
    """Joint 1 near its upper limit, moving toward it: the admitted acceleration
    comes from the braking curve and must be IDENTICAL with the wide box."""
    q = Q0.copy(); q[0] = 2.6
    qd = np.zeros(7); qd[0] = 0.6
    _, ub_old = _box(DECEL, q, qd)
    _, ub_new = _box(WIDE, q, qd, brake=DECEL)
    assert ub_old[0] < DECEL[0]                 # the braking curve is what binds
    assert ub_new[0] == ub_old[0]


def test_wide_box_gives_more_authority_in_free_space():
    lb, ub = _box(WIDE, Q0, np.zeros(7), brake=DECEL)
    np.testing.assert_allclose(ub, WIDE); np.testing.assert_allclose(lb, -WIDE)


def _penetration(box, brake, v0):
    """Joint 5 driven at full box toward its upper limit at entry speed v0; how far past the margin it gets."""
    kw = dict(KW, v_margin=0.7, relax_dt=0.5, brake_eta=0.6)   # the shipped values
    q = Q0.copy(); q[4] = 1.2
    qd = np.zeros(7); qd[4] = v0
    for _ in range(800):
        lb, ub = hard_accel_box(q, qd, acc_lb=np.full(7, -box), acc_ub=np.full(7, box),
                                brake_acc=np.full(7, brake), **kw)
        qd[4] += ub[4] * 0.01
        q[4] += qd[4] * 0.01
    return q[4] - (2.8 - KW['q_margin'])


def test_a_narrower_box_narrows_the_braking_curve_too():
    """qddot_accel_limits below the braking authority (wrists at 6 against 10): the curve must assume the
    joint can only do what the box lets it do. cbf_safety_filter passes brake_acc = min(decel, box)."""
    for v0 in (0.5, 1.0, 1.5, 1.8):
        legacy = _penetration(10.0, 10.0, v0)        # the arithmetic before the box was narrowed
        inconsistent = _penetration(6.0, 10.0, v0)   # narrow box, curve still sized for 10
        consistent = _penetration(6.0, 6.0, v0)      # what the node now does
        assert consistent <= legacy + 1e-9
        assert consistent <= inconsistent + 1e-9
        assert consistent < 0.005                    # rad past the 0.05 rad margin, i.e. it stops on the curve
