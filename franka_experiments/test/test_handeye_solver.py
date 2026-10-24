"""Eye-in-hand hand-eye solver: does it recover a known camera mount?

Every test builds a synthetic rig where the answer is known exactly: a camera
mount X (flange → optical frame) and a tag pose Y (base → tag), then derives
the two measurements the node would record — ``T_BE = T_BC · X⁻¹`` from FK and
``T_CT = T_BC⁻¹ · Y`` from the detector — and checks what comes back.

Pure numpy/scipy.
"""

import math

import numpy as np
import pytest

from franka_experiments.utils.handeye_solver import (
    angle_deg, bootstrap_ee_poses, from_rotvec, initial_guess, inv_T, look_at,
    loop_closure_errors, make_T, orbit_camera_poses, rotation_distance_deg,
    solve)

# A plausible D405 bracket: optical frame ~6 cm off the flange, rotated 90°
# about the flange z and pitched a little.
X_TRUE = make_T(from_rotvec([0.0, 0.0, math.pi / 2]) @ from_rotvec([0.15, 0.0, 0.0]),
                [0.05, -0.03, 0.06])
Y_TRUE = make_T(from_rotvec([0.0, 0.0, 0.4]), [0.50, 0.05, 0.0])
# Start: camera 0.30 m above the tag and slightly behind it, looking down.
_P_C0 = Y_TRUE[:3, 3] + [-0.08, 0.0, 0.29]
T_BC0 = make_T(look_at(_P_C0, Y_TRUE[:3, 3], [0.0, -1.0, 0.0]), _P_C0)


def _measure(T_BC_list, X=X_TRUE, Y=Y_TRUE, noise_t=0.0, noise_r_deg=0.0, seed=0):
    rng = np.random.default_rng(seed)
    T_BE, T_CT = [], []
    for T_BC in T_BC_list:
        T_BE.append(T_BC @ inv_T(X))
        ct = inv_T(T_BC) @ Y
        if noise_t or noise_r_deg:
            w = rng.normal(0.0, math.radians(noise_r_deg), 3)
            ct = ct @ make_T(from_rotvec(w), rng.normal(0.0, noise_t, 3))
        T_CT.append(ct)
    return T_BE, T_CT


def _orbit(n=30, seed=1):
    return orbit_camera_poses(Y_TRUE[:3, 3], T_BC0, n, rng=np.random.default_rng(seed))


def _err(X):
    return (1000.0 * float(np.linalg.norm(X[:3, 3] - X_TRUE[:3, 3])),
            rotation_distance_deg(X[:3, :3], X_TRUE[:3, :3]))


# ── Recovery ────────────────────────────────────────────────────────────────

def test_exact_data_recovers_the_mount():
    T_BE, T_CT = _measure(_orbit())
    res = solve(T_BE, T_CT)
    t_mm, r_deg = _err(res.X)
    assert t_mm < 1e-3 and r_deg < 1e-4
    assert res.train_t_mm < 1e-3 and res.test_t_mm < 1e-3


def test_initial_guess_alone_is_exact_on_clean_data():
    T_BE, T_CT = _measure(_orbit())
    X, Y = initial_guess(T_BE, T_CT)
    t_mm, r_deg = _err(X)
    assert t_mm < 1e-3 and r_deg < 1e-4
    assert float(np.linalg.norm(Y[:3, 3] - Y_TRUE[:3, 3])) < 1e-6


def test_realistic_detection_noise_stays_millimetric():
    """~1 mm / 0.3° per detection, the order of an AprilTag at 0.3 m."""
    T_BE, T_CT = _measure(_orbit(40), noise_t=0.001, noise_r_deg=0.3, seed=3)
    res = solve(T_BE, T_CT)
    t_mm, r_deg = _err(res.X)
    assert t_mm < 3.0 and r_deg < 0.5
    t_v, r_v, source = res.verdict_errors()
    assert source == 'test' and t_v < 5.0 and r_v < 1.0


def test_bad_detections_are_rejected_and_do_not_move_the_result():
    T_BE, T_CT = _measure(_orbit(30), noise_t=0.0005, noise_r_deg=0.1, seed=4)
    bad = [2, 11, 19]
    for i in bad:
        T_CT[i] = T_CT[i] @ make_T(from_rotvec([0.05, -0.04, 0.0]), [0.04, 0.0, -0.03])
    res = solve(T_BE, T_CT)
    assert not res.inliers[bad].any()
    assert res.inliers.sum() >= 25
    t_mm, r_deg = _err(res.X)
    assert t_mm < 2.0 and r_deg < 0.3


def test_mount_rotated_near_pi_is_not_a_singularity():
    X = make_T(from_rotvec([math.pi * 0.999, 0.0, 0.0]), [0.0, 0.07, 0.04])
    T_BE, T_CT = _measure(_orbit(), X=X, noise_t=0.0003, seed=5)
    res = solve(T_BE, T_CT)
    assert float(np.linalg.norm(res.X[:3, 3] - X[:3, 3])) < 0.002
    assert rotation_distance_deg(res.X[:3, :3], X[:3, :3]) < 0.3


# ── Observability ───────────────────────────────────────────────────────────

def test_rotation_about_a_single_axis_is_refused():
    """X's translation along that axis is unobservable: refuse, don't guess."""
    T_BE0 = T_BC0 @ inv_T(X_TRUE)
    T_BE = [T_BE0 @ make_T(from_rotvec([0.0, 0.0, a]), [0.0, 0.0, 0.0])
            for a in np.radians([0, 8, -8, 15, -15])]
    T_CT = [inv_T(T @ X_TRUE) @ Y_TRUE for T in T_BE]
    with pytest.raises(ValueError):
        initial_guess(T_BE, T_CT)


def test_too_few_samples_is_refused():
    T_BE, T_CT = _measure(_orbit(2))
    with pytest.raises(ValueError):
        initial_guess(T_BE, T_CT)


def test_orbit_spans_all_rotation_axes():
    T_BE, T_CT = _measure(_orbit(30))
    res = solve(T_BE, T_CT)
    assert res.axis_sigmas.min() > 0.2


# ── Bootstrap: the phase that has no prior on X ─────────────────────────────

def test_bootstrap_poses_shape():
    T_BE0 = T_BC0 @ inv_T(X_TRUE)
    poses = bootstrap_ee_poses(T_BE0, rot_deg=10.0, trans_m=0.03)
    assert len(poses) == 10
    for T in poses[:6]:
        assert np.allclose(T[:3, 3], T_BE0[:3, 3])
        assert abs(rotation_distance_deg(T[:3, :3], T_BE0[:3, :3]) - 10.0) < 1e-6
    for T in poses[6:]:
        assert np.allclose(T[:3, :3], T_BE0[:3, :3])
        assert abs(np.linalg.norm(T[:3, 3] - T_BE0[:3, 3]) - 0.03) < 1e-9


def test_bootstrap_keeps_the_tag_in_front_of_the_camera():
    T_BE0 = T_BC0 @ inv_T(X_TRUE)
    for T_BE in bootstrap_ee_poses(T_BE0):
        p_tag_cam = (inv_T(T_BE @ X_TRUE) @ Y_TRUE)[:3, 3]
        # Inside a ±35° cone: well within the D405's 87° × 58° field of view.
        assert p_tag_cam[2] > 0.2
        assert math.degrees(math.atan2(np.hypot(*p_tag_cam[:2]), p_tag_cam[2])) < 29.0


def test_bootstrap_alone_is_good_enough_to_aim_the_orbit():
    """The orbit is aimed with the bootstrap's X and Y: a centimetre-level
    error is enough to keep the tag near the image centre."""
    T_BE0 = T_BC0 @ inv_T(X_TRUE)
    T_BC = [T @ X_TRUE for T in [T_BE0] + bootstrap_ee_poses(T_BE0)]
    T_BE, T_CT = _measure(T_BC, noise_t=0.001, noise_r_deg=0.3, seed=6)
    res = solve(T_BE, T_CT, validation_ratio=0.0)
    t_mm, r_deg = _err(res.X)
    assert t_mm < 15.0 and r_deg < 1.5
    assert float(np.linalg.norm(res.Y[:3, 3] - Y_TRUE[:3, 3])) < 0.015


# ── Orbit geometry ──────────────────────────────────────────────────────────

def test_look_at_is_a_proper_rotation_pointing_at_the_target():
    R = look_at([0.3, 0.1, 0.5], [0.5, 0.0, 0.0], [0.0, -1.0, 0.0])
    assert np.allclose(R.T @ R, np.eye(3), atol=1e-12)
    assert np.isclose(np.linalg.det(R), 1.0)
    d = np.array([0.2, -0.1, -0.5])
    assert np.allclose(R[:, 2], d / np.linalg.norm(d))


def test_look_at_survives_a_hint_parallel_to_the_view():
    R = look_at([0.5, 0.0, 0.4], [0.5, 0.0, 0.0], [0.0, 0.0, -1.0])
    assert np.isclose(np.linalg.det(R), 1.0)


def test_orbit_poses_respect_the_cap_and_see_the_tag():
    p_tag = Y_TRUE[:3, 3]
    poses = orbit_camera_poses(p_tag, T_BC0, 25, radius_range=(0.22, 0.38),
                               max_tilt_deg=30.0, max_roll_deg=20.0,
                               rng=np.random.default_rng(2))
    assert len(poses) == 25
    for T in poses:
        v = T[:3, 3] - p_tag
        assert 0.22 - 1e-9 <= np.linalg.norm(v) <= 0.38 + 1e-9
        assert math.degrees(math.acos(v[2] / np.linalg.norm(v))) <= 30.0 + 1e-6
        p_tag_cam = (inv_T(T) @ make_T(np.eye(3), p_tag))[:3, 3]
        assert p_tag_cam[2] > 0.2 and np.hypot(*p_tag_cam[:2]) < 1e-9


def test_orbit_accept_filter_is_honoured():
    poses = orbit_camera_poses(Y_TRUE[:3, 3], T_BC0, 15,
                               accept=lambda T: T[1, 3] > Y_TRUE[1, 3],
                               rng=np.random.default_rng(7))
    assert poses and all(T[1, 3] > Y_TRUE[1, 3] for T in poses)
    assert orbit_camera_poses(Y_TRUE[:3, 3], T_BC0, 5, accept=lambda T: False) == []


def test_orbit_roll_stays_near_the_start_image_orientation():
    """Bounded roll = bounded joint-7 travel between samples."""
    poses = orbit_camera_poses(Y_TRUE[:3, 3], T_BC0, 20, max_roll_deg=25.0,
                               azimuth_half_range_deg=45.0,
                               rng=np.random.default_rng(8))
    for T in poses:
        assert angle_deg(T_BC0[:3, :3].T @ T[:3, :3]) < 90.0


def test_loop_closure_errors_are_zero_for_the_truth():
    T_BE, T_CT = _measure(_orbit(10))
    t, r = loop_closure_errors(X_TRUE, Y_TRUE, T_BE, T_CT)
    assert t.max() < 1e-9 and r.max() < 1e-6
