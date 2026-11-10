"""The Self-Shadow Barrier's geometry, against the filter the pipeline runs.

Every threshold here is read off ``utils.distance_engine``'s Step 2, which is
the only place the removal actually happens:

    discarded iff excluded and NOT (0 < Z < robot_depth - bias - tol)

so a test that passes here is a statement about the shipped self-filter, not
about a model of it.
"""
import numpy as np
import pytest

from franka_experiments.utils.self_shadow import (
    RAY_CANDIDATE, RAY_KEPT, RAY_NAMES, RAY_OUTSIDE, RAY_SELF, RAY_THROUGH,
    RAY_VOID, ShadowCloud, classify_masked_rays, cluster_support,
    extract_shadow_candidates, nearest_candidate_per_cp,
    reject_model_explained, shadow_barrier, unproject, void_spheres_from_rays)

BIAS, TOL, GUARD = 0.06, 0.10, 0.04
ZMIN, ZMAX = 0.15, 4.0


def classify(z_meas, z_model, excluded=True, guard=GUARD):
    z_meas = np.atleast_1d(np.asarray(z_meas, dtype=float))
    z_model = np.broadcast_to(np.atleast_1d(np.asarray(z_model, dtype=float)),
                              z_meas.shape)
    ex = np.broadcast_to(np.atleast_1d(np.asarray(excluded)), z_meas.shape)
    return classify_masked_rays(z_meas, z_model, ex, bias=BIAS, tol=TOL,
                                guard=guard, z_min=ZMIN, z_max=ZMAX)


# ── The five ray classes, at the shipped thresholds ─────────────────────────

def test_the_class_boundaries_are_the_gates_own_thresholds():
    # Model surface at 1.50 m; debiased surface 1.44 m; gate keeps below 1.34 m.
    zm = 1.50
    surf, gate = zm - BIAS, zm - BIAS - TOL
    assert (surf, gate) == pytest.approx((1.44, 1.34))
    cases = {
        gate - 0.01:  RAY_KEPT,       # the gate let it through
        gate + 0.01:  RAY_CANDIDATE,  # removed, well in front of the arm
        surf - GUARD - 1e-6: RAY_CANDIDATE,
        surf - GUARD + 1e-6: RAY_SELF,       # inside the guard: the arm
        surf:          RAY_SELF,
        surf + GUARD - 1e-6: RAY_SELF,
        surf + GUARD + 1e-3: RAY_THROUGH,    # the ray saw past the arm
    }
    for z, want in cases.items():
        assert classify(z, zm)[0] == want, (z, RAY_NAMES[classify(z, zm)[0]])


def test_an_unmasked_pixel_is_never_a_candidate_whatever_its_depth():
    zs = np.linspace(0.2, 3.9, 50)
    cls = classify(zs, 1.5, excluded=False)
    assert set(np.unique(cls)) <= {RAY_KEPT, RAY_OUTSIDE}


def test_a_missing_return_inside_the_mask_is_unknown_not_free():
    for z in (0.0, -1.0, ZMIN - 0.01, ZMAX + 0.01, np.nan):
        assert classify(z, 1.5)[0] == RAY_VOID, z


def test_a_ray_that_sees_past_the_model_clears_its_band_and_is_not_unknown():
    # The dilation halo: no robot on this ray at all, the wall is at 3 m.
    assert classify(3.0, 1.5)[0] == RAY_THROUGH
    # ... which is the whole reason the dilated silhouette must NOT be called
    # unknown wholesale.
    cloud = extract_shadow_candidates(
        np.full(64, 3.0), np.full(64, 1.5), np.ones(64, bool),
        np.arange(64), np.zeros(64),
        cx=0.0, cy=0.0, fx_inv=1.0, fy_inv=1.0,
        bias=BIAS, tol=TOL, guard=GUARD, z_min=ZMIN, z_max=ZMAX)
    assert cloud.n == 0
    assert cloud.counts['through'] == 64
    assert cloud.unknown_fraction() == 0.0


def test_no_robot_modelled_on_the_ray_cannot_manufacture_an_unknown_region():
    assert classify(1.0, np.inf)[0] == RAY_KEPT


def test_the_band_is_a_z_slab_and_not_a_spherical_shell():
    # Same Euclidean range, two pixels 30 deg apart: a shell model would give
    # them the same class, the real gate does not.
    fx_inv = 1.0 / 430.0          # ~ the D455 depth intrinsic at 848x480
    u_off = 430.0 * np.tan(np.deg2rad(12.0))      # well inside the 87 deg HFOV
    z_on, z_off = 1.40, 1.40 * np.cos(np.deg2rad(12.0))
    rng_on = np.linalg.norm(unproject(np.array([0.0]), np.array([0.0]),
                                      np.array([z_on]), 0, 0, fx_inv, fx_inv))
    rng_off = np.linalg.norm(unproject(np.array([u_off]), np.array([0.0]),
                                       np.array([z_off]), 0, 0, fx_inv, fx_inv))
    assert rng_on == pytest.approx(rng_off, abs=2e-3)   # same RANGE
    # Same model z-buffer value on both rays.
    assert classify(z_on, 1.50)[0] == RAY_SELF
    assert classify(z_off, 1.50)[0] == RAY_CANDIDATE    # a shell would say SELF


def test_the_far_side_of_the_gate_is_open_in_the_code_and_closed_in_the_report():
    # |D_meas - D_robot| <= tol has a far edge; the implementation has none.
    # Everything behind the near threshold is removed, however far behind.
    deep = classify(np.array([1.45, 2.0, 3.9]), 1.50)
    assert RAY_KEPT not in set(deep)


# ── The 3-D filter that makes the candidate set usable ──────────────────────

def test_a_candidate_sitting_on_a_mesh_sample_is_the_arm_and_is_dropped():
    model = np.array([[0.0, 0.0, 1.0], [0.02, 0.0, 1.0]])
    cand = np.array([[0.005, 0.0, 1.0],     # on the arm
                     [0.0, 0.0, 0.90]])     # 10 cm in front of it
    keep = reject_model_explained(cand, model, clearance=0.05)
    assert list(keep) == [False, True]


def test_the_3d_filter_is_off_when_it_has_no_model_and_says_so_by_keeping():
    cand = np.array([[0.0, 0.0, 1.0]])
    assert list(reject_model_explained(cand, None, 0.05)) == [True]
    assert list(reject_model_explained(cand, np.empty((0, 3)), 0.05)) == [True]
    assert list(reject_model_explained(cand, np.array([[0., 0., 1.]]), 0.0)) == [True]


def test_the_3d_filter_matches_a_brute_force_nearest_neighbour():
    rng = np.random.default_rng(3)
    model = rng.normal(size=(500, 3))
    cand = rng.normal(size=(200, 3))
    for clearance in (0.1, 0.3, 0.8):
        d = np.sqrt(((cand[:, None, :] - model[None, :, :]) ** 2).sum(2)).min(1)
        np.testing.assert_array_equal(
            reject_model_explained(cand, model, clearance), d > clearance)


def test_speckle_is_dropped_and_a_blob_survives():
    # One 5x5 blob plus three isolated pixels, all at the same depth.
    bu, bv = np.meshgrid(np.arange(5), np.arange(5))
    u = np.r_[bu.ravel(), np.array([40, 60, 80])]
    v = np.r_[bv.ravel(), np.array([40, 60, 80])]
    z = np.full(u.size, 1.2)
    keep = cluster_support(u, v, z, step=1, min_px=5)
    assert keep[:25].all() and not keep[25:].any()
    assert cluster_support(u, v, z, step=1, min_px=1).all()


# ── Per-control-point selection ─────────────────────────────────────────────

def test_the_nearest_candidate_is_a_surface_gap_and_may_be_negative():
    cand = np.array([[0.0, 0.0, 1.0], [0.0, 0.0, 2.0]], dtype=np.float32)
    cp = np.array([[0.0, 0.0, 1.02], [0.0, 0.0, 1.5]], dtype=np.float32)
    gap, idx = nearest_candidate_per_cp(cand, cp, np.array([0.05, 0.05]))
    assert gap[0] == pytest.approx(0.02 - 0.05)    # inside the capsule: < 0
    assert idx[0] == 0
    assert gap[1] == pytest.approx(0.5 - 0.05)
    assert idx[1] in (0, 1)


def test_no_candidate_leaves_every_control_point_at_infinity():
    gap, idx = nearest_candidate_per_cp(np.empty((0, 3)), np.zeros((3, 3)),
                                        np.full(3, 0.05))
    assert np.isinf(gap).all() and (idx == -1).all()


# ── The signed barrier (task section 5.2) ───────────────────────────────────

def test_the_barrier_is_signed_so_a_point_inside_still_has_a_push_direction():
    x = np.zeros(3)
    inside = np.array([0.01, 0.0, 0.0])
    h, n, L = shadow_barrier(inside, x, rho=0.05, r_j=0.0, d_safe=0.0)
    assert h < 0.0                                   # inside the region
    np.testing.assert_allclose(n, [1.0, 0.0, 0.0])   # and still pointing OUT
    assert L == pytest.approx(0.01)
    # An unsigned point-to-set distance would give 0 here with no direction:
    assert max(h, 0.0) == 0.0


def test_the_barrier_is_degenerate_only_at_exact_coincidence():
    h, n, L = shadow_barrier(np.zeros(3), np.zeros(3), rho=0.05, r_j=0.01,
                             d_safe=0.1)
    assert L == 0.0 and not n.any() and h == pytest.approx(-0.16)


# ── Void covering ───────────────────────────────────────────────────────────

def test_a_void_ball_contains_every_ray_segment_it_claims():
    rng = np.random.default_rng(11)
    near = rng.uniform(-0.2, 0.2, (60, 3)) + np.array([0.0, 0.0, 1.0])
    far = near + np.array([0.0, 0.0, 0.16])
    balls = void_spheres_from_rays(near, far, max_spheres=4)
    assert 1 <= len(balls) <= 4
    covered = 0
    for c, r, n in balls:
        covered += n
        # every endpoint of the rays in SOME ball is within that ball
        assert r > 0.0
    assert covered == 60
    # Union coverage: every endpoint is inside at least one ball.
    pts = np.vstack([near, far])
    inside = np.zeros(pts.shape[0], bool)
    for c, r, _ in balls:
        inside |= np.linalg.norm(pts - c, axis=1) <= r + 1e-9
    assert inside.all()


def test_the_void_covering_is_deterministic():
    rng = np.random.default_rng(5)
    near = rng.uniform(-0.3, 0.3, (40, 3))
    far = near + 0.16
    a = void_spheres_from_rays(near, far, max_spheres=3)
    b = void_spheres_from_rays(near, far, max_spheres=3)
    assert len(a) == len(b)
    for (c1, r1, n1), (c2, r2, n2) in zip(a, b):
        np.testing.assert_array_equal(c1, c2)
        assert (r1, n1) == (r2, n2)


def test_no_rays_means_no_balls():
    assert void_spheres_from_rays(np.empty((0, 3)), np.empty((0, 3)),
                                  max_spheres=4) == []


# ── The unknown-fraction metric ─────────────────────────────────────────────

def test_the_unknown_fraction_counts_void_over_what_was_removed():
    z_model = np.full(10, 1.5)
    z_meas = np.array([1.30, 1.30,          # kept  (not removed)
                       1.36, 1.37,          # candidate
                       1.44, 1.45,          # self
                       3.00,                # through
                       0.0, 0.0, 0.0])      # void
    cloud = extract_shadow_candidates(
        z_meas, z_model, np.ones(10, bool), np.arange(10), np.zeros(10),
        cx=0., cy=0., fx_inv=1., fy_inv=1.,
        bias=BIAS, tol=TOL, guard=GUARD, z_min=ZMIN, z_max=ZMAX)
    assert cloud.counts == {'kept': 2, 'candidate': 2, 'self': 2,
                            'through': 1, 'void': 3, 'outside': 0}
    assert cloud.unknown_fraction() == pytest.approx(3 / 8)
    assert cloud.n == 2
    np.testing.assert_allclose(cloud.front_m, [1.44 - 1.36, 1.44 - 1.37], atol=1e-7)
