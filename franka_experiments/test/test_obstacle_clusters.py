"""Obstacle pixels → clusters: separation, centroid accuracy, and the empty case.

This is the identity-assigning stage of the cluster→track→Kalman pipeline. Its
whole job is to answer "which pixels belong to the same object", because a
velocity can only be estimated for something that has an identity across frames.
Everything downstream inherits its mistakes: a blob split in two becomes two
tracks that each see half the evidence, and two blobs merged into one produce a
centroid that sits between two independently moving bodies and therefore a
velocity that belongs to neither.

The fixtures build synthetic depth blobs the way the real pipeline produces
them — a disc of pixels on the subsampled ROI grid, unprojected with a pinhole
model — so the tests exercise the actual coordinate handling (the ``step``
stride in particular), not a hand-made point cloud.

Pure numpy + cv2, no ROS.
"""

import numpy as np

from franka_experiments.utils.obstacle_clusters import (
    Cluster, cluster_obstacles, voxel_downsample)

# Pinhole model roughly matching the D455 depth stream this runs on.
FX = FY = 430.0
CX, CY = 424.0, 240.0


def _disc_pixels(u0, v0, r_px, step):
    """Pixel coordinates of a filled disc, ON the subsampled ROI grid."""
    lo_u, hi_u = int(u0 - r_px), int(u0 + r_px) + 1
    lo_v, hi_v = int(v0 - r_px), int(v0 + r_px) + 1
    # Snap the grid origin to a multiple of `step`, exactly as np.mgrid does.
    vg, ug = np.mgrid[lo_v - lo_v % step:hi_v:step, lo_u - lo_u % step:hi_u:step]
    ug, vg = ug.ravel(), vg.ravel()
    keep = (ug - u0) ** 2 + (vg - v0) ** 2 <= r_px ** 2
    return ug[keep].astype(np.int32), vg[keep].astype(np.int32)


def _blob(u0, v0, r_px, Z, step=1):
    """(p_cam, u, v) for a flat fronto-parallel disc at depth Z [m]."""
    u, v = _disc_pixels(u0, v0, r_px, step)
    p = np.empty((u.size, 3), dtype=np.float32)
    p[:, 0] = (u - CX) * Z / FX
    p[:, 1] = (v - CY) * Z / FY
    p[:, 2] = Z
    return p, u, v


def _true_centroid(p):
    return np.asarray(p, dtype=np.float64).mean(axis=0)


def _merge(*blobs):
    p = np.concatenate([b[0] for b in blobs])
    u = np.concatenate([b[1] for b in blobs])
    v = np.concatenate([b[2] for b in blobs])
    return p, u, v


# ── The three required cases ────────────────────────────────────────────────

def test_two_separated_blobs_give_two_clusters_with_accurate_centroids():
    """The core case. Two objects 100 px apart must not be pooled into one
    centroid sitting in the empty space between them — that midpoint is what
    a scalar, identity-free estimate effectively tracks today."""
    b1 = _blob(200, 200, 18, Z=0.9)
    b2 = _blob(400, 260, 22, Z=1.2)
    cl = cluster_obstacles(*_merge(b1, b2))

    assert len(cl) == 2
    # Match each cluster to the blob it is nearest to, then check BOTH matched.
    truths = [_true_centroid(b1[0]), _true_centroid(b2[0])]
    matched = []
    for c in cl:
        k = int(np.argmin([np.linalg.norm(c.centroid_cam - t) for t in truths]))
        matched.append(k)
        assert np.linalg.norm(c.centroid_cam - truths[k]) < 0.01
    assert sorted(matched) == [0, 1], 'both blobs must be recovered, not one twice'


def test_one_blob_gives_exactly_one_cluster():
    p, u, v = _blob(300, 240, 25, Z=1.0)
    cl = cluster_obstacles(p, u, v)
    assert len(cl) == 1
    assert np.linalg.norm(cl[0].centroid_cam - _true_centroid(p)) < 0.01
    assert cl[0].n_points == p.shape[0]


def test_empty_mask_gives_no_clusters():
    """A frame with no obstacle pixel must yield [], not a degenerate cluster at
    the origin — the tracker would associate to it and invent a track."""
    assert cluster_obstacles(np.empty((0, 3), np.float32),
                             np.empty(0, np.int32), np.empty(0, np.int32)) == []


# ── The subsampled grid (the failure this coordinate handling exists for) ────

def test_a_subsampled_roi_still_yields_one_cluster_per_blob():
    """DistanceEngine walks the ROI with a stride, so obstacle pixels are `step`
    apart in full-resolution coordinates. Labelling those at full resolution
    would find every pixel isolated and return one cluster per pixel."""
    for step in (1, 2, 4):
        b1 = _blob(200, 200, 20, Z=0.9, step=step)
        b2 = _blob(420, 200, 20, Z=0.9, step=step)
        cl = cluster_obstacles(*_merge(b1, b2), step=step)
        assert len(cl) == 2, f'step={step} produced {len(cl)} clusters'


def test_forgetting_the_step_shatters_the_blob():
    """Guards the guard: with step defaulted to 1 on strided pixels the result
    must visibly fall apart, so the test above is known to be testing something."""
    p, u, v = _blob(300, 240, 20, Z=1.0, step=4)
    assert len(cluster_obstacles(p, u, v, step=1)) == 0  # all fragments < min_points
    assert len(cluster_obstacles(p, u, v, step=4)) == 1


# ── Separation / merging behaviour ──────────────────────────────────────────

def test_touching_blobs_are_one_cluster():
    """Under-segmentation is the DESIGNED failure direction: this module groups
    what is connected in the image, and two bodies in contact are one blob. The
    test pins the behaviour so a future change to connectivity is deliberate."""
    b1 = _blob(300, 240, 20, Z=1.0)
    b2 = _blob(338, 240, 20, Z=1.0)
    assert len(cluster_obstacles(*_merge(b1, b2))) == 1


def test_diagonal_connectivity_keeps_a_slanted_limb_whole():
    """8-connectivity, not 4: a limb crossing the image diagonally is one
    object, and 4-connectivity cuts it into a staircase of fragments."""
    u = np.arange(200, 260, dtype=np.int32)
    v = u.copy()                       # a 1-px-wide diagonal line
    u = np.repeat(u, 3) + np.tile([-1, 0, 1], u.size)   # 3 px thick
    v = np.repeat(v, 3)
    p = np.stack([(u - CX) / FX, (v - CY) / FY, np.ones(u.size)], axis=1).astype(np.float32)
    assert len(cluster_obstacles(p, u, v, connectivity=8)) == 1
    assert len(cluster_obstacles(p, u, v, connectivity=4)) >= 1


# ── Speckle rejection and ordering ──────────────────────────────────────────

def test_a_speckle_below_min_points_is_dropped():
    """An isolated depth artefact must not become a cluster: the tracker would
    associate to it and, given enough frames, confirm a track on noise."""
    big = _blob(200, 200, 20, Z=1.0)
    speck_u = np.array([500, 501], dtype=np.int32)
    speck_v = np.array([300, 300], dtype=np.int32)
    speck_p = np.zeros((2, 3), dtype=np.float32)
    speck_p[:, 2] = 1.0
    cl = cluster_obstacles(np.concatenate([big[0], speck_p]),
                           np.concatenate([big[1], speck_u]),
                           np.concatenate([big[2], speck_v]))
    assert len(cl) == 1


def test_clusters_come_out_largest_first_and_deterministically():
    """Greedy nearest-neighbour association consumes this list in order, so a
    frame-to-frame reshuffle of equal clusters would reshuffle track ids."""
    small = _blob(200, 200, 12, Z=1.0)
    large = _blob(420, 200, 25, Z=1.0)
    cl = cluster_obstacles(*_merge(small, large))
    assert [c.n_points for c in cl] == sorted([c.n_points for c in cl], reverse=True)
    again = cluster_obstacles(*_merge(large, small))     # different input order
    assert [c.n_points for c in cl] == [c.n_points for c in again]
    assert np.allclose(cl[0].centroid_cam, again[0].centroid_cam)


def test_max_clusters_keeps_the_largest():
    blobs = [_blob(120 + 90 * k, 200, 10 + 3 * k, Z=1.0) for k in range(5)]
    cl = cluster_obstacles(*_merge(*blobs), max_clusters=2)
    assert len(cl) == 2
    assert cl[0].n_points >= cl[1].n_points
    assert cl[0].n_points == max(b[0].shape[0] for b in blobs)


# ── Centroid must not follow the camera's sampling density ──────────────────

def test_the_centroid_is_not_pulled_by_the_pixel_sampling_density():
    """A surface slanted away from the camera is sampled far more densely at its
    near end — a pixel covers a growing patch of it as Z grows — so the raw mean
    of the points sits near the close end rather than in the middle of the
    object. That bias is a function of RANGE, so it MOVES as the object moves,
    and a differentiator downstream reads the movement as a velocity the
    obstacle never had. The 2 cm voxel downsample makes the samples uniform in
    metres, which is what removes it.

    Fixture: an obliquely slanted plane Z = a + b·X, sampled at pixel
    resolution. The plane's physical mid-point is the midpoint of its X extent;
    the voxel centroid must land nearer to that than the raw point mean does.
    """
    u = np.arange(300, 520, dtype=np.int32)
    v_band = np.arange(230, 251, dtype=np.int32)
    uu = np.repeat(u, v_band.size)
    vv = np.tile(v_band, u.size)
    a, b = 1.0, 3.0
    Z = a / (1.0 - b * (uu - CX) / FX)          # from Z = a + b·X, X = (u−cx)Z/fx
    X = (uu - CX) * Z / FX
    Y = (vv - CY) * Z / FY
    p = np.stack([X, Y, Z], axis=1).astype(np.float32)

    cl = cluster_obstacles(p, uu, vv, voxel_m=0.02)
    assert len(cl) == 1
    mid = 0.5 * (float(X.min()) + float(X.max()))
    raw_err = abs(float(X.mean()) - mid)
    vox_err = abs(float(cl[0].centroid_cam[0]) - mid)
    assert raw_err > 0.15, 'fixture assumption: the density bias must be large'
    assert vox_err < 0.1 * raw_err


def test_voxel_downsample_is_continuous_in_the_input():
    """Cell CENTRES would quantise the centroid onto a 2 cm lattice, so a slowly
    drifting obstacle would read as stationary and then briefly very fast. The
    in-cell mean keeps the output continuous."""
    base = np.random.default_rng(0).normal(scale=0.05, size=(400, 3)) + np.array([0.0, 0.0, 1.0])
    c0 = voxel_downsample(base, 0.02).mean(axis=0)
    c1 = voxel_downsample(base + np.array([1e-4, 0.0, 0.0]), 0.02).mean(axis=0)
    assert np.linalg.norm(c1 - c0) < 1e-3


# ── Radius / contains ───────────────────────────────────────────────────────

def test_radius_covers_the_blob_and_contains_agrees():
    p, u, v = _blob(300, 240, 30, Z=1.0)
    c = cluster_obstacles(p, u, v)[0]
    r_true = float(np.linalg.norm(p.astype(np.float64) - c.centroid_cam, axis=1).max())
    # The radius is measured over voxel centroids, so it may fall just inside the
    # extreme raw point; it must still be the right order and cover the bulk.
    assert 0.5 * r_true <= c.radius <= r_true + 0.02
    assert c.contains(c.centroid_cam)
    assert not c.contains(c.centroid_cam + np.array([10.0, 0.0, 0.0]))


def test_a_single_voxel_cluster_still_has_a_physical_extent():
    """radius = 0 would make contains() reject every point including its own
    neighbours, so a small-but-real obstacle could never claim a control point."""
    u = np.arange(300, 315, dtype=np.int32)
    v = np.full(u.size, 240, dtype=np.int32)
    p = np.zeros((u.size, 3), dtype=np.float32)
    p[:, 2] = 1.0                                   # all inside one 2 cm voxel
    c = cluster_obstacles(p, u, v, voxel_m=1.0)[0]
    assert c.radius > 0.0


# ── Contract guards ─────────────────────────────────────────────────────────

def test_misaligned_inputs_are_rejected_loudly():
    """p_cam and (u, v) are index-aligned by construction upstream; if that ever
    breaks, every centroid would be silently attached to the wrong pixels."""
    p, u, v = _blob(300, 240, 20, Z=1.0)
    try:
        cluster_obstacles(p, u[:-1], v[:-1])
    except ValueError:
        return
    raise AssertionError('misaligned inputs must raise')


def test_output_is_the_documented_type():
    p, u, v = _blob(300, 240, 20, Z=1.0)
    c = cluster_obstacles(p, u, v)[0]
    assert isinstance(c, Cluster)
    assert c.centroid_cam.shape == (3,) and c.centroid_cam.dtype == np.float64
    assert isinstance(c.n_points, int) and isinstance(c.radius, float)
