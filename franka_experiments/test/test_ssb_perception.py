"""SSB through the REAL DistanceEngine, on frames built from the gate's numbers.

``R_base = I`` and ``t_base = 0`` throughout, so base frame == camera frame and
a pixel's base-frame z IS its depth. That is what makes the assertions below
readable as metres in front of the arm.
"""
import numpy as np
import pytest

from franka_experiments.utils.distance_engine import DistanceEngine, _DEPTH_TO_M

H, W = 48, 48
ROBOT_Z = 1.50          # where the model puts the arm
BIAS, TOL, GUARD = 0.06, 0.10, 0.04
SURF = ROBOT_Z - BIAS   # 1.44 — the debiased model surface
GATE = SURF - TOL       # 1.34 — below this the gate keeps the pixel


def engine(**over):
    cfg = {'min_depth_m': 0.15, 'max_depth_m': 4.0, 'lpf_alpha': 0.0,
           'depth_gate_tol_m': TOL, 'depth_gate_bias_m': BIAS,
           'ssb_enabled': True, 'ssb_guard_m': GUARD,
           'ssb_model_clearance_m': 0.05, 'ssb_min_support_px': 6}
    cfg.update(over)
    return DistanceEngine(cfg)


def cp(point, radius=0.0):
    return [dict(point=np.asarray(point, float), radius=radius, seg_idx=0,
                 cp_idx=0, start_link='fr3_link6', end_link='fr3_link6')]


def model_points(fx_inv=1.0 / 400.0):
    """The arm's own mesh samples in CAMERA frame, over the masked square."""
    vv, uu = np.mgrid[10:38:2, 10:38:2]
    u = uu.ravel().astype(float); v = vv.ravel().astype(float)
    z = np.full(u.size, SURF)
    return np.stack([(u - W / 2) * z * fx_inv,
                     (v - H / 2) * z * fx_inv, z], axis=1)


def run(e, depth, excl, robot_depth, point=(0.0, 0.0, 1.45), radius=0.0,
        model=None, stamp=0.0):
    return e.compute(
        depth=depth, cx_f32=np.float32(W / 2), cy_f32=np.float32(H / 2),
        fx_inv_f32=np.float32(1.0 / 400.0), fy_inv_f32=np.float32(1.0 / 400.0),
        R_base_f32=np.eye(3, dtype=np.float32),
        t_base_f32=np.zeros(3, dtype=np.float32),
        control_points=cp(point, radius), x=np.array([0, W]),
        y=np.array([0, H]), step=1, search_exclusion_mask=excl,
        robot_depth=robot_depth, model_points_cam=model, frame_stamp=stamp)


def scene(obstacle_z, blob=8, model_everywhere=False):
    """The arm masked at ROBOT_Z, with a ``blob x blob`` patch at ``obstacle_z``.

    A CALIBRATION-CONSISTENT scene, and that matters. ``robot_depth`` is the
    MODEL (``ROBOT_Z``) while the arm MEASURES at ``SURF = ROBOT_Z - BIAS``,
    i.e. the residual ``measured - model`` is exactly ``-BIAS`` — which is what
    ``mask.depth_gate_bias_m`` is documented to be set to. Building the scene
    with a zero residual and a non-zero bias instead would put the arm's own
    pixels BEHIND the debiased surface and class them ``THROUGH``, which says
    something about an inconsistent configuration rather than about SSB.

    Everything outside the arm's silhouette is the far wall at 3 m, which the
    depth-range filter keeps and the distance pass is free to pick — exactly as
    a real room does.
    """
    depth = np.full((H, W), np.uint16(3.0 / _DEPTH_TO_M), dtype=np.uint16)
    excl = np.zeros((H, W), dtype=bool)
    robot_depth = np.full((H, W), ROBOT_Z if model_everywhere else np.inf,
                          dtype=np.float32)
    robot_depth[10:38, 10:38] = ROBOT_Z
    depth[10:38, 10:38] = np.uint16(SURF / _DEPTH_TO_M)
    excl[10:38, 10:38] = True
    if obstacle_z is not None:
        depth[20:20 + blob, 20:20 + blob] = np.uint16(obstacle_z / _DEPTH_TO_M)
    return depth, excl, robot_depth


# ── The default is inert ────────────────────────────────────────────────────

def test_with_the_flag_off_nothing_is_measured_and_no_field_is_written():
    e = engine(ssb_enabled=False)
    res, _ = run(e, *scene(GATE + 0.02), model=model_points())
    assert e.last_shadow_cloud is None and e.last_shadow_stats == {}
    assert np.isinf(res[0].shadow_distance)
    assert res[0].shadow_point is None and res[0].shadow_direction is None


def test_the_obstacle_path_is_bit_identical_with_ssb_on():
    args = scene(GATE + 0.02)
    off = run(engine(ssb_enabled=False), *args, model=model_points())
    on = run(engine(), *args, model=model_points())
    assert off[1] == on[1]
    for a, b in zip(off[0], on[0]):
        assert a.distance == b.distance
        assert a.range_m == b.range_m
        np.testing.assert_array_equal(a.direction, b.direction)
        np.testing.assert_array_equal(a.closest_obstacle_point,
                                      b.closest_obstacle_point)


# ── The mechanism ───────────────────────────────────────────────────────────

def test_a_return_the_gate_deleted_in_front_of_the_arm_becomes_a_shadow():
    obs_z = GATE + 0.02                       # 1.36: deleted, 8 cm in front
    e = engine()
    res, _ = run(e, *scene(obs_z), model=model_points())
    r = res[0]
    assert r.shadow_distance < np.inf, e.last_shadow_stats
    assert r.shadow_point is not None
    # The recovered point really is the obstacle, at its own depth.
    assert r.shadow_point[2] == pytest.approx(obs_z, abs=2e-3)
    # ... and it sits SURF - obs_z in front of the debiased model surface.
    assert r.shadow_front_m == pytest.approx(SURF - obs_z, abs=2e-3)
    # The gap is the control point's own distance to it.
    want = float(np.linalg.norm(np.array([0.0, 0.0, 1.45]) - r.shadow_point))
    assert r.shadow_distance == pytest.approx(want, abs=1e-5)


def test_the_ordinary_pipeline_really_is_blind_to_it():
    """The premise. Without this the mechanism would be solving nothing."""
    obs_z = GATE + 0.02
    res, _ = run(engine(), *scene(obs_z), model=model_points())
    # The engine's nearest OBSERVED obstacle is the 3 m wall, not the 1.36 m
    # patch: the patch was deleted.
    assert res[0].closest_obstacle_point[2] > 2.0
    assert res[0].distance > 0.5


def test_a_return_at_the_arms_own_surface_is_not_a_shadow():
    e = engine()
    res, _ = run(e, *scene(SURF), model=model_points())
    assert np.isinf(res[0].shadow_distance)
    assert e.last_shadow_stats['cls_self'] > 0
    assert e.last_shadow_stats['cls_candidate'] == 0


def test_a_return_the_gate_kept_is_the_obstacle_paths_and_not_a_shadow():
    e = engine()
    res, _ = run(e, *scene(GATE - 0.05), model=model_points())
    assert np.isinf(res[0].shadow_distance)
    assert res[0].closest_obstacle_point[2] == pytest.approx(GATE - 0.05, abs=2e-3)


def test_a_ray_that_sees_past_the_arm_is_free_space_not_a_shadow():
    # A hole in the model (the dilation halo): the ray reaches the 3 m wall.
    depth, excl, robot_depth = scene(None)
    depth[20:28, 20:28] = np.uint16(3.0 / _DEPTH_TO_M)
    e = engine()
    res, _ = run(e, depth, excl, robot_depth, model=model_points())
    assert np.isinf(res[0].shadow_distance)
    assert e.last_shadow_stats['cls_through'] == 64
    assert e.last_shadow_stats['cls_self'] == 28 * 28 - 64


def test_a_missing_return_is_counted_unknown_and_builds_no_row():
    depth, excl, robot_depth = scene(None)
    depth[20:28, 20:28] = 0                      # no return at all
    e = engine()
    res, _ = run(e, depth, excl, robot_depth, model=model_points())
    assert np.isinf(res[0].shadow_distance), 'VOID must not become a row'
    assert e.last_shadow_stats['cls_void'] == 64
    assert e.last_shadow_cloud.unknown_fraction() == pytest.approx(
        64 / (28 * 28))


# ── The filters ─────────────────────────────────────────────────────────────

def test_a_candidate_the_model_explains_is_dropped_by_the_3d_test():
    """A pixel whose z-buffer entry is wrong, not an obstacle.

    The arm is really at 1.36 here, the model says 1.50, so the per-ray depth
    test calls the arm's own surface a candidate. Only the 3-D test can tell:
    the point coincides with a mesh sample.
    """
    obs_z = GATE + 0.02
    depth, excl, robot_depth = scene(obs_z)
    # Mesh samples placed at the PATCH's real depth, i.e. the arm is there.
    vv, uu = np.mgrid[20:28, 20:28]
    u = uu.ravel().astype(float); v = vv.ravel().astype(float)
    z = np.full(u.size, obs_z)
    at_patch = np.stack([(u - W / 2) * z / 400.0,
                         (v - H / 2) * z / 400.0, z], axis=1)
    model = np.vstack([model_points(), at_patch])
    e = engine()
    res, _ = run(e, depth, excl, robot_depth, model=model)
    assert e.last_shadow_stats['n_band'] == 64
    assert e.last_shadow_stats['n_unexplained'] == 0
    assert np.isinf(res[0].shadow_distance)


def test_speckle_is_dropped_by_the_support_filter():
    obs_z = GATE + 0.02
    depth, excl, robot_depth = scene(None)
    for u, v in ((15, 15), (30, 30), (15, 30)):   # three isolated pixels
        depth[v, u] = np.uint16(obs_z / _DEPTH_TO_M)
    e = engine(ssb_min_support_px=6)
    res, _ = run(e, depth, excl, robot_depth, model=model_points())
    assert e.last_shadow_stats['n_unexplained'] == 3
    assert e.last_shadow_stats['n_supported'] == 0
    assert np.isinf(res[0].shadow_distance)
    # ... and is kept when the support requirement is removed.
    e2 = engine(ssb_min_support_px=1)
    res2, _ = run(e2, depth, excl, robot_depth, model=model_points())
    assert np.isfinite(res2[0].shadow_distance)


def test_the_guard_band_is_what_keeps_the_arm_out():
    """At guard = 0 the arm's own pixels become candidates by construction."""
    # A perfectly calibrated model, with the sensor's own 2 cm spread on top.
    jitter = np.random.default_rng(7).normal(0.0, 0.02, (28, 28))
    depth, excl, robot_depth = scene(None)
    depth[10:38, 10:38] = ((SURF + jitter) / _DEPTH_TO_M).astype(np.uint16)
    e_on, e_off = engine(ssb_guard_m=GUARD), engine(ssb_guard_m=0.0)
    run(e_on, depth, excl, robot_depth, model=None)
    run(e_off, depth, excl, robot_depth, model=None)
    n_on = e_on.last_shadow_stats['cls_candidate']
    n_off = e_off.last_shadow_stats['cls_candidate']
    # guard = 0 lets HALF the distribution through, by construction: bias is a
    # median. guard = 2 sigma leaves the tail, which the 3-D model test and the
    # support filter then have to carry -- which is the whole reason neither of
    # them is optional.
    assert n_off > 0.4 * 28 * 28
    assert n_on < 0.05 * 28 * 28
    assert n_on < n_off / 10


# ── Robustness of the measurement contract ──────────────────────────────────

def test_a_frame_with_no_obstacle_pixel_still_carries_the_shadow():
    """The failure SSB exists for: nothing publishable, something shadowed."""
    obs_z = GATE + 0.02
    depth, excl, robot_depth = scene(obs_z, model_everywhere=True)
    excl[:] = True                      # the whole frame is inside the mask
    e = engine()
    res, n_px = run(e, depth, excl, robot_depth, model=model_points())
    assert n_px == 0, 'the obstacle pass must really have nothing'
    assert not np.isfinite(res[0].distance)
    assert np.isfinite(res[0].shadow_distance)


def test_the_cloud_is_never_stale_when_the_gate_cannot_run():
    e = engine()
    run(e, *scene(GATE + 0.02), model=model_points(), stamp=0.0)
    assert e.last_shadow_cloud.n > 0
    # Second frame with no exclusion mask at all: the gate did not run, so
    # there is no measurement and the previous frame's must not stand.
    depth, _excl, robot_depth = scene(GATE + 0.02)
    e.compute(depth=depth, cx_f32=np.float32(W / 2), cy_f32=np.float32(H / 2),
              fx_inv_f32=np.float32(1 / 400.0), fy_inv_f32=np.float32(1 / 400.0),
              R_base_f32=np.eye(3, dtype=np.float32),
              t_base_f32=np.zeros(3, dtype=np.float32),
              control_points=cp((0, 0, 1.45)), x=np.array([0, W]),
              y=np.array([0, H]), step=1, search_exclusion_mask=None,
              robot_depth=robot_depth, frame_stamp=0.1)
    assert e.last_shadow_cloud.n == 0
    assert e.last_shadow_stats == {'reason': 'gate_not_run'}


def test_no_model_points_keeps_candidates_and_says_so():
    e = engine()
    run(e, *scene(GATE + 0.02), model=None)
    assert e.last_shadow_stats['n_model_pts'] == 0
    assert e.last_shadow_stats['n_unexplained'] == e.last_shadow_stats['n_band']


def test_the_shadow_gap_subtracts_the_capsule_radius_and_may_go_negative():
    obs_z = GATE + 0.02
    e = engine()
    res, _ = run(e, *scene(obs_z), point=(0.0, 0.0, obs_z + 0.02),
                 radius=0.05, model=model_points())
    assert res[0].shadow_distance < 0.0


def test_the_two_model_rejection_paths_agree_exactly():
    """The brute-force and cKDTree branches of `reject_model_explained` must match.

    The function picks between them on candidate count, because the tree's build
    is paid every frame and dominates at the 1-5 candidates the runs actually
    report. That is a performance switch on a SAFETY filter -- the thing that
    stops the arm being classified as its own shadow -- so the two paths are
    checked against each other on the same data, including candidates placed
    exactly at the clearance radius, where a `>` and a `>=` would disagree.
    """
    import numpy as np
    from scipy.spatial import cKDTree

    from franka_experiments.utils.self_shadow import reject_model_explained

    rng = np.random.default_rng(7)
    model = rng.uniform(-0.4, 0.4, (3000, 3))
    clearance = 0.05
    for n in (1, 2, 5, 8, 9, 20, 64):
        q = rng.uniform(-0.5, 0.5, (n, 3))
        # force a few exact-radius cases: sit on a sphere of radius `clearance`
        # around a model point, which is the boundary the comparison decides
        if n >= 5:
            d = rng.normal(size=(3, 3))
            d /= np.linalg.norm(d, axis=1)[:, None]
            q[:3] = model[:3] + d * clearance
        brute = reject_model_explained(q, model, clearance)
        tree = reject_model_explained(q, cKDTree(model), clearance)
        assert brute.dtype == bool and tree.dtype == bool
        np.testing.assert_array_equal(brute, tree, err_msg=f'n={n}')


def test_model_rejection_keeps_everything_without_a_model():
    """No model is not evidence that the arm is absent -- on BOTH paths.

    The docstring's promise (``model_pts`` of None or empty keeps everything)
    has to survive the algorithm switch, since an empty model set is exactly
    the case where the product-based rule sends work to the brute-force branch.
    """
    import numpy as np

    from franka_experiments.utils.self_shadow import reject_model_explained

    q = np.zeros((4, 3))
    for model in (None, np.zeros((0, 3))):
        keep = reject_model_explained(q, model, 0.05)
        assert keep.shape == (4,) and keep.all()
    # clearance of 0 disables the test, and must not reject either
    assert reject_model_explained(q, np.zeros((10, 3)), 0.0).all()
    # ... and an empty candidate set returns an empty mask, not a crash
    assert reject_model_explained(np.zeros((0, 3)), np.zeros((10, 3)), 0.05).shape == (0,)
