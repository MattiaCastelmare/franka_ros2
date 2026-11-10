"""Self-Shadow Barrier (SSB): what the robot's own depth self-filter deleted.

WHY THIS EXISTS
---------------
``real_time_distance`` removes the robot from the depth image before it looks
for obstacles.  The removal is a two-part test, and this module is written
against the test the code ACTUALLY implements (``utils.distance_engine``
Step 2), not against the one a reader would guess:

    a pixel is DISCARDED  iff  search_exclusion_mask[v, u]
                               and NOT (0 < Z < z_model - bias - tol)

with

* ``Z``        the measured OPTICAL-AXIS depth (``p_cam[:, 2]``), in metres —
  NOT a Euclidean camera range;
* ``z_model``  ``MaskBuilder.robot_depth``, the per-pixel minimum of the
  projected mesh samples, min-filtered over the dilation kernel — again an
  optical-axis z, not a range;
* ``bias``     ``mask.depth_gate_bias_m`` (0.06 m shipped), the MEDIAN of the
  model-minus-measurement residual;
* ``tol``      ``mask.depth_gate_tol_m`` (0.10 m shipped), its SPREAD.

Three consequences that the obvious "spherical shell wedge" model gets wrong,
and that the geometry below gets right:

1. **It is a z-slab, not a spherical shell.**  ``|‖x−c‖ − D| ≤ tol`` and
   ``|z_cam(x) − z_model| ≤ tol`` differ by ``1/cos θ`` off-axis.  The D455's
   848×480 depth stream is ~87° horizontal, so at the edge of the frame that
   factor is ~1.4: a shell model misplaces the band by tens of centimetres on
   exactly the pixels where the arm is most often seen.
2. **The band is ONE-SIDED.**  The code keeps only what is clearly IN FRONT;
   everything at or behind ``z_model − bias − tol`` is discarded, with no far
   edge.  ``|D_measured − D_robot| ≤ tol`` has a far edge the code does not.
3. **The angular extent is the DILATED SILHOUETTE of the mesh**, not a cone of
   half-angle ``θ_i`` about a control point.  It is a constant PIXEL margin
   (12 px body / 24 px end effector at full resolution, from
   ``robot_mask_dilate_px`` 2, ``ee_mask_dilate_px`` 18,
   ``search_exclusion_extra_px`` 8 at ``mask_downsample`` 4), so its metric
   width grows linearly with depth, and its angular width varies across the
   frame because a pinhole's ``dθ/du`` is ``f/(f² + r²)``, not a constant.

WHAT IS UNKNOWN, AND WHAT IS ONLY HIDDEN
----------------------------------------
A depth camera returns at most ONE range per ray, so "was something deleted
here" is answered per ray, from the single measurement the ray carries:

``KEPT``     ``Z < z_model − bias − tol``.  The gate kept it.  The ordinary
             obstacle pipeline owns this pixel and this module ignores it.
``SELF``     ``Z`` within ``guard`` of ``z_model − bias``.  The return is where
             the model says the robot's own surface is, so the robot explains
             it.  Not evidence of an obstacle.
``CANDIDATE`` ``z_model − bias − tol ≤ Z < z_model − bias − guard``.  A real,
             valid return, clearly IN FRONT of the robot's own surface, that
             the tolerance swallowed.  **This is the self-shadow proper**: an
             observed surface of unknown identity.  It is what the barrier is
             built on.
``THROUGH``  ``Z > z_model − bias + guard``.  The ray saw PAST where the model
             puts the arm.  Either no robot is on this ray (a dilation-halo
             pixel) or the pose is wrong; either way the band was transparent,
             so the band along this ray is OBSERVED FREE.  Not unknown — this
             is the case that makes "treat the whole dilated silhouette as
             unknown" wrong.
``VOID``     inside the mask, no valid return at all (0, or outside
             ``[min_depth, max_depth]``).  No information whatsoever: the whole
             slab in front of the model surface along this ray is genuinely
             UNKNOWN.  A missing return is NOT evidence of free space.

The three states the task asks to distinguish map onto these as
observed-occupied = ``KEPT`` ∪ ``CANDIDATE`` ∪ ``SELF``, observed-free = the
space nearer than any valid return (so ``THROUGH`` clears the band),
unknown = ``VOID``.

WHY A CANDIDATE POINT IS NOT JUST "THE ARM WITH A BAD z_model"
---------------------------------------------------------------
``z_model`` is a sparse z-buffer with a hole-filling min-filter, and its
residual against the real arm is several centimetres with a several-centimetre
spread (``mask.depth_gate_bias_m``'s own note: −2.6 to −5.3 cm, MAD 3.8–5.7 cm
on one day of runs).  A per-ray depth comparison therefore cannot, on its own,
separate "an obstacle 8 cm in front of the arm" from "the arm, and the buffer
was 8 cm out here".

So the candidate set is filtered a SECOND time, in 3-D and against a
representation whose error is not the z-buffer's: the projected mesh samples
themselves (:func:`reject_model_explained`).  A candidate that is farther than
``clearance`` from EVERY mesh sample of the robot cannot be the robot, whatever
the z-buffer said.  That test is independent of the sampling-density bias,
because it is a distance to the samples rather than a comparison against their
per-pixel minimum.

A third filter asks for spatial support (:func:`cluster_support`): a handful of
isolated pixels is depth speckle, an object is a blob.  Same argument, and the
same ``label_points`` labelling, that ``tracking.cluster_min_points`` already
makes for the ordinary pipeline.

THE BARRIER
-----------
:func:`shadow_row` builds one acceleration-level HOCBF row from a protected
control point and one shadow point.  It covers both regimes with one formula:

* **SSB-S** (a ``CANDIDATE`` return).  The measurement is a statement about the
  WORLD — a surface was observed at this place — so the point is frozen in the
  base frame for the life of the frame and ``J_x = 0``.
* **SSB-V** (a ``VOID`` slab).  Nothing was observed; the unknown REGION is a
  property of where the robot is, so it is carried rigidly by the link that
  casts it and ``J_x`` is that link's point Jacobian.  This is the "the unknown
  region moves with q" term, and it is what makes ``ḣ`` right rather than
  merely conservative.

Both use a SIGNED, sphere-inflated distance

    h = ‖p_j − x‖ − ρ − r_j − d_safe

rather than the unsigned point-to-set distance ``dist(p, U)``.  ``dist(p, U)``
is identically 0 inside ``U`` and its subgradient there contains 0, so it gives
the QP no escape direction at exactly the configurations where one is needed.
``‖p_j − x‖ − ρ`` is smooth and strictly decreasing toward the centre on all of
``R³ \\ {x}``, so inside the region the row still asks for motion away from the
centre; it is singular only at ``p_j = x`` exactly, which the builders skip.

Everything here is pure numpy (+ an optional ``scipy.spatial.cKDTree``) and
imports no ROS, so the arithmetic and every gate are unit-testable directly.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional, Tuple

import numpy as np

# ── Ray classes ──────────────────────────────────────────────────────────────
#: The gate kept this pixel; the ordinary obstacle pipeline owns it.
RAY_KEPT = 0
#: Removed, and the robot's own modelled surface explains the return.
RAY_SELF = 1
#: Removed, valid return clearly IN FRONT of the modelled surface — a shadow
#: candidate: an observed surface whose identity the pipeline threw away.
RAY_CANDIDATE = 2
#: Removed, but the ray saw PAST the modelled surface, so the band was
#: transparent along it: observed free, not unknown.
RAY_THROUGH = 3
#: Inside the mask with no valid return: genuinely unknown.
RAY_VOID = 4
#: Not inside the exclusion mask at all (nothing was removed here).
RAY_OUTSIDE = 5

RAY_NAMES = ('kept', 'self', 'candidate', 'through', 'void', 'outside')


def classify_masked_rays(
    z_meas: np.ndarray,
    z_model: np.ndarray,
    excluded: np.ndarray,
    *,
    bias: float,
    tol: float,
    guard: float,
    z_min: float,
    z_max: float,
) -> np.ndarray:
    """Per-ray class of the self-filter's decision.  Vectorised, no allocation
    beyond the output and its masks.

    Args:
        z_meas: (N,) measured optical-axis depth [m].  Non-positive or outside
            ``[z_min, z_max]`` counts as "no return".
        z_model: (N,) ``MaskBuilder.robot_depth`` at the same pixels, ``+inf``
            where the robot is not.
        excluded: (N,) bool, ``search_exclusion_mask`` at the same pixels.
        bias, tol: ``mask.depth_gate_bias_m`` / ``mask.depth_gate_tol_m``.  The
            gate keeps a masked pixel iff ``0 < z_meas < z_model − bias − tol``.
        guard: [m] half-width of the band around the bias-corrected model
            surface in which a return is read as the ROBOT.  Sized by the
            SPREAD of the calibration residual, not by sensor noise: at
            ``guard = 0`` half the arm's own pixels become candidates by
            construction, because ``bias`` is a median.
        z_min, z_max: the engine's ``distance.min_depth_m`` / ``max_depth_m``.

    Returns:
        (N,) int8 of ``RAY_*``.

    ``z_model = +inf`` (no robot modelled on this ray) can only occur outside
    the exclusion mask in the shipped pipeline — the depth buffer is eroded
    with the same kernel the mask is dilated with — but it is handled anyway:
    the gate's own comparison is then vacuously true, the pixel is kept, and
    this reports ``RAY_KEPT`` rather than inventing an unknown region out of a
    missing model.
    """
    z_meas = np.asarray(z_meas, dtype=np.float64)
    z_model = np.asarray(z_model, dtype=np.float64)
    excluded = np.asarray(excluded, dtype=bool)
    out = np.full(z_meas.shape, RAY_OUTSIDE, dtype=np.int8)

    have = (z_meas > 0.0) & (z_meas >= z_min) & (z_meas <= z_max) & np.isfinite(z_meas)
    surf = z_model - float(bias)              # the model's front surface, debiased
    gate = surf - float(tol)                  # the gate's keep threshold

    # Outside the mask nothing was removed, and a masked pixel the gate kept is
    # the ordinary pipeline's. Both are RAY_KEPT for accounting purposes; only
    # `excluded` pixels can be anything else.
    out[~excluded & have] = RAY_KEPT
    masked = excluded

    kept = masked & have & (z_meas < gate)
    out[kept] = RAY_KEPT

    # No return inside the mask: nothing is known along this ray.
    out[masked & ~have] = RAY_VOID

    rest = masked & have & ~kept
    # EXHAUSTIVE by construction: SELF is the default of `rest` and the two
    # tails overwrite it. Writing the three as independent comparisons leaves a
    # hole exactly at |z - surf| == guard, where floating-point rounding makes
    # `<= guard` and `< -guard` / `> +guard` all false and the ray ends up in
    # no class at all. The default-plus-tails form cannot have one.
    out[rest] = RAY_SELF
    out[rest & (z_meas > surf + float(guard))] = RAY_THROUGH
    out[rest & (z_meas < surf - float(guard))] = RAY_CANDIDATE
    return out


@dataclass
class ShadowCloud:
    """The shadow candidates of ONE depth frame, in CAMERA frame.

    ``p_cam`` is index-aligned with ``u`` / ``v`` / ``z``.  ``counts`` is the
    per-class tally over every ray the engine's decimated grid visited, which
    is what the "fraction of the self-filtered region classified unknown"
    metric is computed from — a number that is meaningless without the
    denominator, hence ``n_grid``.
    """
    p_cam: np.ndarray                      # (N, 3) float32
    u: np.ndarray                          # (N,) int32
    v: np.ndarray                          # (N,) int32
    z: np.ndarray                          # (N,) float32, optical-axis depth
    counts: dict = field(default_factory=dict)
    n_grid: int = 0
    step: int = 1
    stamp: Optional[float] = None
    #: (N,) metres by which each candidate sits IN FRONT of the debiased model
    #: surface. Diagnostic: it is the quantity the whole mechanism turns on.
    front_m: Optional[np.ndarray] = None

    @property
    def n(self) -> int:
        return int(self.p_cam.shape[0])

    def unknown_fraction(self) -> float:
        """Share of the rays the self-filter removed that carry no information.

        ``VOID`` over everything the mask removed (``VOID`` + ``SELF`` +
        ``CANDIDATE`` + ``THROUGH``).  Returns 0.0 when nothing was removed, so
        an empty scene reads as "nothing unknown" rather than NaN.
        """
        removed = sum(self.counts.get(RAY_NAMES[k], 0)
                      for k in (RAY_SELF, RAY_CANDIDATE, RAY_THROUGH, RAY_VOID))
        if removed <= 0:
            return 0.0
        return float(self.counts.get(RAY_NAMES[RAY_VOID], 0)) / float(removed)


def empty_cloud(step: int = 1, stamp: Optional[float] = None) -> ShadowCloud:
    return ShadowCloud(p_cam=np.empty((0, 3), dtype=np.float32),
                       u=np.empty(0, dtype=np.int32),
                       v=np.empty(0, dtype=np.int32),
                       z=np.empty(0, dtype=np.float32),
                       counts={n: 0 for n in RAY_NAMES},
                       n_grid=0, step=int(step), stamp=stamp,
                       front_m=np.empty(0, dtype=np.float32))


def unproject(u: np.ndarray, v: np.ndarray, z: np.ndarray,
              cx: float, cy: float, fx_inv: float, fy_inv: float) -> np.ndarray:
    """(N,3) camera-frame points from pixel coordinates and optical-axis depth.

    The same pinhole the distance engine uses, written once so the shadow
    geometry cannot drift from the obstacle geometry.
    """
    p = np.empty((u.size, 3), dtype=np.float32)
    zf = np.asarray(z, dtype=np.float32)
    p[:, 0] = (np.asarray(u, dtype=np.float32) - np.float32(cx)) * (zf * np.float32(fx_inv))
    p[:, 1] = (np.asarray(v, dtype=np.float32) - np.float32(cy)) * (zf * np.float32(fy_inv))
    p[:, 2] = zf
    return p


def reject_model_explained(p_cam: np.ndarray, model_pts: Optional[np.ndarray],
                           clearance: float) -> np.ndarray:
    """Keep only candidates the robot's own geometry cannot account for.

    ``model_pts`` is the robot's projected mesh samples in the SAME (camera)
    frame — the points ``MaskBuilder`` already transforms every rebuild.  A
    candidate within ``clearance`` of any of them is the arm, whatever the
    z-buffer said at its pixel, and is dropped.

    This is the filter that makes the mechanism safe to switch on.  The
    per-pixel depth test cannot do it: ``robot_depth`` is a SPARSE buffer whose
    bias moves by centimetres with ``meshes.sample_points_per_link`` alone
    (utils/calibration_check: −0.9 cm at 300 samples/link, +6.0 cm at 20000, on
    one unchanged extrinsic), so a depth-only candidate set is dominated by the
    arm itself.  A distance to the samples has no such bias — it is wrong only
    by the sample SPACING, which is ~3 mm at the shipped 3000 samples/link and
    ~14 mm if the set is decimated 20×.  ``clearance`` must exceed that spacing.

    Returns a bool keep-mask.  ``model_pts`` of ``None`` or length 0 keeps
    everything: not having the model is not evidence that the arm is absent,
    and the caller's own gates (the depth band, the cluster support) still
    apply.  Callers that cannot tolerate that should check for themselves.
    """
    n = int(np.asarray(p_cam).shape[0])
    if n == 0:
        return np.zeros(0, dtype=bool)
    # `model_pts` may be a PREBUILT cKDTree, which has no len(): its size is
    # `.n`. Asking len() of one raises TypeError, so the size is taken through
    # this helper rather than inline -- the tree branch is documented and
    # supported, and was previously unreachable because of exactly that.
    n_model = getattr(model_pts, 'n', None)
    if n_model is None:
        n_model = 0 if model_pts is None else len(model_pts)
    if n_model == 0 or clearance <= 0.0:
        return np.ones(n, dtype=bool)

    def _brute() -> np.ndarray:
        q = np.asarray(p_cam, dtype=np.float64)
        m = np.asarray(model_pts, dtype=np.float64)
        c = float(clearance)
        # EXACT bounding-box prefilter, not an approximation: a model point
        # within `c` of ANY candidate must lie inside the candidates' bounding
        # box inflated by `c`, so everything outside can be discarded without
        # changing the answer. The candidates of one frame sit on one obstacle,
        # a few centimetres across, so this typically drops the 13500 model
        # samples to a few hundred -- and it is what makes the brute-force
        # branch cheaper than the tree instead of merely competitive.
        inside = np.all((m >= q.min(axis=0) - c) & (m <= q.max(axis=0) + c),
                        axis=1)
        m = m[inside]
        if m.shape[0] == 0:
            # nothing of the arm is near any candidate: all unexplained
            return np.ones(q.shape[0], dtype=bool)
        # per candidate, so the (n, N, 3) temporary is never allocated
        c2 = c * c
        out = np.empty(q.shape[0], dtype=bool)
        for i in range(q.shape[0]):
            d = m - q[i]
            out[i] = bool(np.einsum('ij,ij->i', d, d).min() > c2)
        return out

    try:
        from scipy.spatial import cKDTree
    except ImportError:                                      # pragma: no cover
        return _brute()

    # WHICH ALGORITHM. Building a cKDTree over the model samples costs
    # O(N log N) and is paid EVERY frame, because the samples move with q;
    # querying it is almost free. Measured on this machine at the shipped
    # N = 13500 (ssb_model_points_stride 8), per call:
    #
    #   n_cand      1       3       5      10      25     100     425
    #   brute   0.32    1.16    1.12    2.16    5.84   53.6   257.3   ms
    #   tree    5.29    2.66    1.89    2.07    1.82    4.96     4.94  ms
    #
    # so the crossover is ~8 candidates, and the tree's build alone is 4.84 ms
    # of its 5.09 ms. The Gazebo runs report 1-5 candidates per frame, i.e. the
    # regime where the tree is 2-16x the wrong choice -- and the geometry pass
    # it dominated was measured costing 10-15 % of the published distance-row
    # rate (34-39 Hz -> 29-33 Hz). Hence: brute force while the work is small,
    # the tree once it is not. Expressed as the product so that a decimated
    # model set widens the brute-force regime, which is the right scaling.
    if not isinstance(model_pts, cKDTree) and n * n_model <= 120_000:
        return _brute()
    tree = model_pts if isinstance(model_pts, cKDTree) else cKDTree(
        np.asarray(model_pts, dtype=np.float64))
    d, _ = tree.query(np.asarray(p_cam, dtype=np.float64), k=1,
                      distance_upper_bound=float(clearance))
    # cKDTree returns +inf for a point with no neighbour inside the bound,
    # which is exactly "farther than clearance from every sample".
    return ~np.isfinite(d)


def cluster_support(u: np.ndarray, v: np.ndarray, z: np.ndarray, *,
                    step: int, min_px: int, connectivity: int = 8,
                    depth_jump: float = 0.10,
                    out_max_blob: Optional[list] = None) -> np.ndarray:
    """Keep candidates belonging to a blob of at least ``min_px`` samples.

    Speckle rejection, and the same labelling the ordinary pipeline clusters
    with (``utils.obstacle_clusters.label_points``), so a shadow blob and an
    obstacle blob are the same kind of object.  ``min_px <= 1`` keeps
    everything and costs nothing.

    UNITS. ``min_px`` counts GRID SAMPLES, not raw pixels: the engine samples
    every ``distance.pixel_step``-th pixel (6 shipped), so one sample stands
    for step² = 36 pixels and the threshold is 36x coarser than its name
    suggests. The number that matters is therefore how many SAMPLES an obstacle
    covers: a sphere of radius r at range Z subtends r/Z radians, i.e.
    ``f*r/(Z*step)`` samples of radius — for r = 4 cm at 1.5 m with f = 431.5
    and step 6 that is 1.9, so about 11 samples for the whole disc. A threshold
    of 6 keeps roughly half a disc and rejects less.

    ``out_max_blob``, when given, is appended the largest blob size found, for
    the SSBDIAG line: without it a frame where every candidate was rejected for
    want of support is indistinguishable from one with no candidates at all.
    """
    n = int(np.asarray(u).size)
    if n == 0:
        if out_max_blob is not None:
            out_max_blob.append(0)
        return np.zeros(0, dtype=bool)
    if min_px <= 1:
        if out_max_blob is not None:
            out_max_blob.append(n)
        return np.ones(n, dtype=bool)
    from franka_experiments.utils.obstacle_clusters import label_points
    _, labels = label_points(np.asarray(u), np.asarray(v), np.asarray(z),
                             step=int(step), connectivity=int(connectivity),
                             depth_jump=float(depth_jump))
    labels = np.asarray(labels, dtype=np.int64)
    if labels.size == 0:
        if out_max_blob is not None:
            out_max_blob.append(0)
        return np.zeros(n, dtype=bool)
    sizes = np.bincount(labels)
    if out_max_blob is not None:
        out_max_blob.append(int(sizes.max()) if sizes.size else 0)
    return sizes[labels] >= int(min_px)


def extract_shadow_candidates(
    depth_m: np.ndarray,
    z_model: np.ndarray,
    excluded: np.ndarray,
    ug: np.ndarray,
    vg: np.ndarray,
    *,
    cx: float, cy: float, fx_inv: float, fy_inv: float,
    bias: float, tol: float, guard: float,
    z_min: float, z_max: float,
    step: int = 1,
    stamp: Optional[float] = None,
) -> ShadowCloud:
    """Classify one frame's grid and unproject the ``CANDIDATE`` rays.

    ``depth_m``, ``z_model`` and ``excluded`` are already sampled at
    ``(vg, ug)`` by the caller — passing the sampled vectors rather than the
    full images keeps this allocation-light and lets the distance engine reuse
    the arrays it has in hand.
    """
    cls = classify_masked_rays(depth_m, z_model, excluded, bias=bias, tol=tol,
                               guard=guard, z_min=z_min, z_max=z_max)
    counts = {RAY_NAMES[k]: int(np.count_nonzero(cls == k))
              for k in range(len(RAY_NAMES))}
    sel = cls == RAY_CANDIDATE
    if not np.any(sel):
        c = empty_cloud(step=step, stamp=stamp)
        c.counts = counts
        c.n_grid = int(cls.size)
        return c
    u = np.asarray(ug)[sel].astype(np.int32)
    v = np.asarray(vg)[sel].astype(np.int32)
    z = np.asarray(depth_m)[sel].astype(np.float32)
    front = (np.asarray(z_model)[sel] - float(bias) - z).astype(np.float32)
    return ShadowCloud(
        p_cam=unproject(u, v, z, cx, cy, fx_inv, fy_inv),
        u=u, v=v, z=z, counts=counts, n_grid=int(cls.size), step=int(step),
        stamp=stamp, front_m=front)


def nearest_candidate_per_cp(p_cam: np.ndarray, cp_cam: np.ndarray,
                             radii: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """Surface gap and winning index of the nearest shadow point per control point.

    Mirrors the distance engine's Step 6 exactly — direct vectorised
    subtraction, ``gap = ‖·‖ − r``, no dilation-margin subtraction.  The margin
    correction exists because the exclusion mask pushes the first OBSERVABLE
    obstacle pixel outward; these pixels are read from INSIDE the mask, so the
    correction does not apply to them and adding it would shorten the gap
    twice.

    Unlike the obstacle path the gap is NOT clamped at 0: a negative gap means
    the shadow surface is inside the control point's own capsule, and the sign
    is what tells the barrier whether it is being pushed out or held off.

    Returns ``(gap (n_cp,), idx (n_cp,))``; ``gap`` is ``+inf`` and ``idx`` is
    ``-1`` for every control point when there is no candidate at all.
    """
    n_cp = int(np.asarray(cp_cam).shape[0])
    if p_cam is None or len(p_cam) == 0 or n_cp == 0:
        return np.full(n_cp, np.inf), np.full(n_cp, -1, dtype=np.int64)
    P = np.asarray(p_cam, dtype=np.float32)
    gaps = np.empty(n_cp)
    idxs = np.empty(n_cp, dtype=np.int64)
    for i in range(n_cp):
        diff = P - np.asarray(cp_cam, dtype=np.float32)[i]
        d = np.sqrt((diff * diff).sum(axis=1)) - np.float32(radii[i])
        j = int(d.argmin())
        gaps[i] = float(d[j])
        idxs[i] = j
    return gaps, idxs


# ═════════════════════════════════════════════════════════════════════════════
#  The barrier
# ═════════════════════════════════════════════════════════════════════════════

class ShadowRow(Tuple):
    """Documentation alias; the builders return plain tuples."""


def shadow_barrier(p_j: np.ndarray, x: np.ndarray, *,
                   rho: float, r_j: float, d_safe: float) -> Tuple[float, np.ndarray, float]:
    """``h``, the unit normal ``n̂`` and the centre distance, for one pair.

        h(q) = ‖p_j(q) − x(q)‖ − ρ − r_j − d_safe,     n̂ = (p_j − x)/‖p_j − x‖

    SIGNED by construction, and that is the point (task §5.2).  The unsigned
    point-to-set distance ``dist(p, U)`` is 0 on all of ``U`` and its
    subdifferential there contains the zero vector, so a QP row built on it
    asks for nothing at exactly the configurations where the protected point is
    already inside the unknown region.  ``‖p_j − x‖ − ρ`` keeps a non-zero
    gradient everywhere except the single point ``p_j = x``: inside the region
    it is negative and still decreasing toward the centre, so the HOCBF row
    reads as "accelerate away from the centre", which is the only direction
    that can be right with no further information.

    ``n̂`` is undefined at ``p_j = x`` exactly; the caller gets ``n̂ = 0`` there
    and must skip the row (every builder in this module does).
    """
    d = np.asarray(p_j, dtype=np.float64) - np.asarray(x, dtype=np.float64)
    L = float(np.linalg.norm(d))
    if L <= 1e-12:
        return float(-rho - r_j - d_safe), np.zeros(3), 0.0
    return float(L - rho - r_j - d_safe), d / L, L


def shadow_row(p_j: np.ndarray, J_j: np.ndarray, Jd_j: Optional[np.ndarray],
               x: np.ndarray, qdot: np.ndarray, *,
               J_x: Optional[np.ndarray] = None,
               Jd_x: Optional[np.ndarray] = None,
               rho: float = 0.0, r_j: float = 0.0, d_safe: float = 0.0,
               with_kappa: bool = False):
    """One acceleration-level HOCBF row for a protected point and a shadow point.

    Returns ``(a, h, cdot, kappa, n_hat, L)`` or ``None`` when the geometry is
    degenerate (coincident points, or a non-finite entry).

    THE DERIVATIVES, IN FULL.  With ``v_rel = ṗ_j − ẋ = (J_j − J_x) q̇``:

        h    = ‖p_j − x‖ − ρ − r_j − d_safe
        ∂h/∂q = n̂ᵀ(J_j − J_x)                                      ( = a )
        ḣ    = a q̇
        ḧ    = a q̈  +  n̂ᵀ(J̇_j − J̇_x) q̇  +  ṅ̂ᵀ v_rel
             = a q̈  +  ċ  +  κ,        κ = ‖(I − n̂n̂ᵀ) v_rel‖² / L ≥ 0

    where ``L = ‖p_j − x‖`` and ``ṅ̂ = (I − n̂n̂ᵀ) v_rel / L``.  The HOCBF
    constraint ``ḧ + k₁ḣ + k₀h ≥ 0`` is therefore

        a q̈  ≥  −k₁ (a q̇) − k₀ h − ċ − κ

    which is exactly the ``build_row_rhs`` expression the rest of the filter
    already evaluates, with ``v_obs = 0`` (both ends of this row are carried by
    the robot's own kinematics, or frozen, so nothing external moves it).

    ``κ ≥ 0`` RELAXES the row, so omitting it is conservative, and
    ``with_kappa=False`` is the default for that reason: it is the same term
    ``enable_curvature_term`` gates on the obstacle family and the same
    argument applies — do not claim a relaxation without the measurement that
    supports it.  It is returned either way so a caller can log it.

    THE TWO REGIMES differ only in ``J_x``:

    * ``J_x = None`` → the shadow point is FROZEN in the base frame.  Correct
      for a ``CANDIDATE``: a depth return is a statement about the world, and
      the world does not move because the arm did.
    * ``J_x = J(link_i, x)`` → the point is carried rigidly by the link that
      casts the shadow.  Correct for a ``VOID`` slab, whose very existence is a
      property of where the arm is.  This is the "the unknown region moves with
      ``q``" term; dropping it would make ``ḣ`` wrong by ``−n̂ᵀJ_x q̇``, which
      for a slab 0.16 m in front of a link moving at 0.5 m/s is the same order
      as ``ḣ`` itself.
    """
    h, n_hat, L = shadow_barrier(p_j, x, rho=rho, r_j=r_j, d_safe=d_safe)
    if L <= 1e-9:
        return None
    J_j = np.asarray(J_j, dtype=np.float64)
    JJ = J_j if J_x is None else J_j - np.asarray(J_x, dtype=np.float64)
    a = n_hat @ JJ
    qdot = np.asarray(qdot, dtype=np.float64)
    v_rel = JJ @ qdot
    if Jd_j is None:
        cdot = 0.0
    else:
        DD = np.asarray(Jd_j, dtype=np.float64)
        if Jd_x is not None:
            DD = DD - np.asarray(Jd_x, dtype=np.float64)
        cdot = float(n_hat @ (DD @ qdot))
    v_perp = v_rel - float(n_hat @ v_rel) * n_hat
    kappa = float(v_perp @ v_perp) / L
    if not (np.all(np.isfinite(a)) and np.isfinite(h) and np.isfinite(cdot)
            and np.isfinite(kappa)):
        return None
    return (np.ascontiguousarray(a), float(h), float(cdot),
            float(kappa) if with_kappa else 0.0, n_hat, float(L))


# ── Void slabs: the unknown region that genuinely moves with the robot ───────

@dataclass
class VoidSphere:
    """One ball of unknown space, carried rigidly by the link that casts it.

    ``centre_local`` is in the owner link's frame, so the world centre is
    ``R_i(q) centre_local + t_i(q)`` and its Jacobian is the link's point
    Jacobian at that centre — which is what makes the barrier's ``ḣ`` account
    for the region's own motion rather than only the protected point's.

    A ball, not the exact slab-of-a-frustum, and deliberately: the slab is a
    non-convex, pixel-defined set whose exact distance is neither closed-form
    nor differentiable where the active face switches, while an OUTER ball
    around it is both, and being an OUTER bound it can only ever make the
    barrier more conservative — never less.  :func:`void_spheres_from_rays`
    documents how tight that bound is.
    """
    link: str
    centre_local: np.ndarray    # (3,) owner-link frame
    radius: float               # [m] outer radius of the covered slab
    n_rays: int = 0             # how many VOID rays it covers (diagnostic)


def void_spheres_from_rays(p_near: np.ndarray, p_far: np.ndarray,
                           *, max_spheres: int, min_rays: int = 1
                           ) -> list[Tuple[np.ndarray, float, int]]:
    """Cover a bundle of unknown ray-segments with a few outer balls.

    ``p_near`` / ``p_far`` are (N,3) endpoints of each unknown ray segment: the
    near face of the blind slab (``z_model − bias − tol``) and its far face
    (the modelled surface, behind which the arm is opaque and the space is
    occluded rather than unknown).

    Greedy, deterministic and allocation-bounded: the segment endpoints are
    split into ``max_spheres`` groups by a fixed-axis median split (the axis of
    largest spread, recursively), and each group is covered by the ball through
    its endpoint bounding sphere.  Determinism matters more than tightness here
    — a covering that reshuffles between frames is a constraint set that
    reshuffles between frames.

    Returns ``[(centre, radius, n_rays), …]`` in the SAME frame the endpoints
    were given in, largest group first.  Groups with fewer than ``min_rays``
    rays are dropped.
    """
    P = np.asarray(p_near, dtype=np.float64)
    Q = np.asarray(p_far, dtype=np.float64)
    if P.shape[0] == 0 or max_spheres <= 0:
        return []
    groups = [np.arange(P.shape[0])]
    while len(groups) < max_spheres:
        # Split the group whose endpoint cloud is widest, along its widest axis.
        k = max(range(len(groups)),
                key=lambda g: _spread(P[groups[g]], Q[groups[g]]))
        idx = groups[k]
        if idx.size < 2 * max(min_rays, 1):
            break
        pts = np.vstack([P[idx], Q[idx]])
        ax = int(np.argmax(pts.max(axis=0) - pts.min(axis=0)))
        key = 0.5 * (P[idx, ax] + Q[idx, ax])
        order = np.argsort(key, kind='stable')
        half = idx.size // 2
        groups[k] = idx[order[:half]]
        groups.append(idx[order[half:]])
    out = []
    for idx in groups:
        if idx.size < min_rays:
            continue
        pts = np.vstack([P[idx], Q[idx]])
        c = 0.5 * (pts.max(axis=0) + pts.min(axis=0))
        r = float(np.max(np.linalg.norm(pts - c, axis=1)))
        out.append((c, r, int(idx.size)))
    out.sort(key=lambda t: -t[2])
    return out


def _spread(p: np.ndarray, q: np.ndarray) -> float:
    if p.shape[0] == 0:
        return 0.0
    pts = np.vstack([p, q])
    return float(np.max(pts.max(axis=0) - pts.min(axis=0)))
