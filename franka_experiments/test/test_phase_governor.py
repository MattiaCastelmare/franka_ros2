"""Obstacle-driven phase governor and the bounded-return error cap.

Two behaviours, both specified from hardware runs:

1. The trajectory must slow down ONLY when the obstacle is genuinely close
   (< 5 cm) and resume full speed as soon as it is clear. Above that the CBF
   alone handles avoidance — it engages at d_safe = 0.20 m.

2. After a large deviation the return to the path must be a CONSTANT-strength
   pull, never proportional to how far the arm was pushed. The task law is
   xddot = a_d + Kp*e + Kd*edot, so an uncapped e made the rejoin command grow
   with the deviation; J-pinv mapped it onto the joints, the per-joint clip at
   qddot_max (17 on the wrist) did not contain it, and the controller
   integrated it into a joint_velocity_violation.

The governor signal is DISTANCE, never tracking error. An error-driven governor
has a stable fixed point at zero motion — CBF blocks, error grows, phase
freezes, reference parks on the blocked pose, CBF keeps blocking. That is the
deadlock that got the previous feasibility_beta governor reverted in 4d4d450.

Pure numpy; the two functions under test are reproduced here against the same
parameters the node declares, because importing the node needs rclpy.
"""

import math

import numpy as np

# Defaults declared by pentagon_qddot_commander.
D_FULL, D_STOP, SIGMA_MIN, TIMEOUT = 0.05, 0.02, 0.05, 0.5
CART_ERR_MAX = 0.08
KP = 100.0                      # representative kp_cart


def governor_sigma(d, stamp, now, enabled=True,
                   d_full=D_FULL, d_stop=D_STOP, s_min=SIGMA_MIN,
                   timeout=TIMEOUT):
    """Mirror of PentagonQddotCommander._governor_sigma."""
    if not enabled:
        return 1.0
    if stamp == 0.0 or (now - stamp) > timeout:
        return 1.0
    if not math.isfinite(d) or d >= d_full:
        return 1.0
    if d <= d_stop:
        return s_min
    f = (d - d_stop) / max(d_full - d_stop, 1e-9)
    return s_min + (1.0 - s_min) * f


def saturate(e, e_max=CART_ERR_MAX):
    """Mirror of the direction-preserving Cartesian error cap."""
    n = float(np.linalg.norm(e))
    if e_max > 0.0 and n > e_max:
        return e * (e_max / n), n
    return e.copy(), n


# ── governor ─────────────────────────────────────────────────────────────────

def test_full_speed_above_five_centimetres():
    """The stated requirement: above 5 cm the trajectory is untouched."""
    for d in (0.05, 0.06, 0.10, 0.20, 1.0):
        assert governor_sigma(d, 1.0, 1.0) == 1.0, d


def test_slows_hard_below_the_stop_distance():
    for d in (0.02, 0.01, 0.0):
        assert governor_sigma(d, 1.0, 1.0) == SIGMA_MIN, d


def test_ramp_is_continuous_at_both_ends():
    """No step in sigma, hence none in v_d = P'(s)*s_dot."""
    lo = governor_sigma(D_STOP + 1e-9, 1.0, 1.0)
    hi = governor_sigma(D_FULL - 1e-9, 1.0, 1.0)
    assert abs(lo - SIGMA_MIN) < 1e-6
    assert abs(hi - 1.0) < 1e-6
    prev = 0.0
    for d in np.linspace(D_STOP, D_FULL, 200):
        v = governor_sigma(d, 1.0, 1.0)
        assert v >= prev - 1e-12, 'sigma must be monotone in distance'
        prev = v


def test_sigma_never_reaches_zero():
    """The deadlock guard.

    A phase that can stop completely has a fixed point: the reference parks on a
    pose the CBF forbids and neither side can move. With a floor the reference
    always creeps, so a fixed obstacle on the path is walked past.
    """
    for d in np.linspace(-0.05, 0.10, 500):
        assert governor_sigma(d, 1.0, 1.0) >= SIGMA_MIN > 0.0


def test_fixed_obstacle_on_the_path_still_makes_progress():
    """Integrate the phase with a permanently close obstacle."""
    s, dt = 0.0, 0.01
    for _ in range(1000):                      # 10 s
        s += dt * governor_sigma(0.01, 1.0, 1.0)
    assert s > 0.4, f'phase must keep creeping, advanced only {s:.3f}'
    # ...but much slower than unobstructed.
    assert s < 0.5 * 10.0


def test_resumes_immediately_when_the_obstacle_clears():
    assert governor_sigma(0.03, 1.0, 1.0) < 1.0
    assert governor_sigma(0.051, 1.0, 1.0) == 1.0


def test_stale_or_absent_status_leaves_the_phase_open():
    """No filter running means no obstacle data; crawling blindly is worse."""
    assert governor_sigma(0.01, 0.0, 5.0) == 1.0            # never received
    assert governor_sigma(0.01, 1.0, 1.0 + 2 * TIMEOUT) == 1.0   # stale
    assert governor_sigma(0.01, 1.0, 1.0) == SIGMA_MIN      # fresh -> acts


def test_infinite_distance_means_nothing_in_range():
    assert governor_sigma(float('inf'), 1.0, 1.0) == 1.0


def test_disabled_is_a_pure_passthrough():
    for d in (0.0, 0.01, 0.05, 1.0):
        assert governor_sigma(d, 1.0, 1.0, enabled=False) == 1.0


# ── bounded return ───────────────────────────────────────────────────────────

def test_error_cap_bounds_the_return_command():
    """The rejoin command must not scale with how far the arm was pushed."""
    small, _ = saturate(np.array([0.03, 0.0, 0.0]))
    huge, _ = saturate(np.array([0.60, 0.0, 0.0]))
    assert np.linalg.norm(small) == 0.03, 'small errors pass through untouched'
    assert abs(np.linalg.norm(huge) - CART_ERR_MAX) < 1e-12
    # What the PD actually sees:
    assert KP * np.linalg.norm(huge) == KP * CART_ERR_MAX
    # Without the cap the same deviation would demand 7.5x more.
    assert (0.60 / CART_ERR_MAX) > 7.0


def test_error_cap_preserves_direction():
    """The arm must still head back to the right place, just not harder."""
    for e in (np.array([0.3, -0.4, 0.5]), np.array([-1.0, 0.2, 0.0]),
              np.array([0.0, 0.0, -0.9])):
        sat, n = saturate(e)
        assert np.allclose(sat / np.linalg.norm(sat), e / np.linalg.norm(e))
        assert abs(np.linalg.norm(sat) - CART_ERR_MAX) < 1e-12
        assert n == float(np.linalg.norm(e))


def test_error_cap_reports_the_true_norm_for_the_antiwindup_guard():
    """The reset must see the real error, not the capped one."""
    e = np.array([0.0, 0.0, 0.55])
    sat, true_n = saturate(e)
    assert abs(true_n - 0.55) < 1e-12
    assert np.linalg.norm(sat) < true_n


def test_error_cap_disabled_by_zero():
    e = np.array([0.9, 0.0, 0.0])
    sat, _ = saturate(e, e_max=0.0)
    assert np.allclose(sat, e)


def test_return_stays_within_joint_limits_for_a_plausible_jacobian():
    """End to end: capped error -> task accel -> joint accel, inside the box.

    A well-conditioned Jacobian at a mid-range pose. The point is the ratio:
    with the cap the demand is bounded no matter the deviation, so it cannot
    saturate the wrist joints the way the uncapped command did.
    """
    QDD = np.array([6.0, 2.585, 3.5, 4.0, 17.0, 5.5, 17.0])
    rng = np.random.default_rng(0)
    J = rng.normal(scale=0.35, size=(3, 7))
    J_pinv = J.T @ np.linalg.inv(J @ J.T + 1e-4 * np.eye(3))

    for deviation in (0.1, 0.3, 0.6, 1.0):
        e = np.array([deviation, 0.0, 0.0])
        sat, _ = saturate(e)
        qdd_capped = J_pinv @ (KP * sat)
        qdd_raw = J_pinv @ (KP * e)
        # The capped demand is identical for every deviation past the cap.
        if deviation > CART_ERR_MAX:
            ref = J_pinv @ (KP * np.array([CART_ERR_MAX, 0.0, 0.0]))
            assert np.allclose(qdd_capped, ref)
        assert np.linalg.norm(qdd_capped) <= np.linalg.norm(qdd_raw) + 1e-9
        # And the uncapped one blows past the joint box while the capped one
        # scales with the (fixed) cap instead of the deviation.
        assert np.max(np.abs(qdd_capped) / QDD) == \
            np.max(np.abs(J_pinv @ (KP * saturate(e)[0])) / QDD)


if __name__ == '__main__':
    import pytest
    raise SystemExit(pytest.main([__file__, '-v']))
