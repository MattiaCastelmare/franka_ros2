"""Obstacle depth pixels → a handful of discrete objects with a centroid each.

WHY THIS EXISTS
---------------
Everything downstream of perception in this filter is currently *anonymous*.
``DistanceEngine`` answers one question per control point — "how far is the
nearest obstacle pixel" — and the CBF turns that scalar into a barrier. Nothing
anywhere carries the notion of WHICH OBJECT the pixel belonged to.

That anonymity is exactly what caps the obstacle-velocity estimate. Today
``ConstraintBuilder._obstacle_speed`` recovers a closing speed as the residual
``v_obs = aᵀq̇ − ḋ``, i.e. by differencing a distance that is itself an argmin
over pixels. The engine's own docstring calls that argmin memoryless: it hops
between surface patches, and when it hops from one object to another the
difference is not a velocity at all, it is a discontinuity. The residual has no
way to tell the two apart, so it is EMA'd at α = 0.7 and clamped — which is
another way of saying the estimate is deliberately made slow because it cannot
be made trustworthy.

A velocity needs an IDENTITY to be attached to. This module produces the
identity: it groups the obstacle pixels the distance engine already selected
into connected objects and reduces each to a centroid. That centroid is a
measurement a Kalman filter can be fed (``obstacle_tracker``), and the filter's
state is a genuine 3D velocity with a covariance, not a scalar residual.

WHAT THIS MODULE IS NOT
-----------------------
It is not a scene segmentation and it is not a person detector. Two people
standing shoulder to shoulder are one cluster here, and one person seen through
a foreground occluder is two. That is acceptable because of how the result is
used: the cluster only has to be COHERENT ENOUGH that the points inside it share
a velocity. Over-segmentation costs a track; under-segmentation costs
resolution. Neither can loosen a barrier — the consumer (step 7) only ever uses
a track's velocity to TIGHTEN a row, and a control point with no matching
cluster falls back to zero, i.e. to the pre-existing behaviour.

REUSE, NOT RECOMPUTATION
------------------------
The inputs are precisely what ``DistanceEngine.compute`` already built in its
Step 3/4: the unprojected camera-frame points ``p_cam`` and the pixel
coordinates ``(u, v)`` that survived the exclusion mask and the depth-range
filter. Those index arrays ARE the obstacle boolean mask, held sparsely. This
module scatters them back into a dense grid because 2D connected components
needs adjacency, and for no other reason — no mask is rebuilt, no depth is
re-read, no pixel is re-tested.

THE PIXEL GRID IS SUBSAMPLED
----------------------------
``DistanceEngine`` walks the ROI with ``np.mgrid[y0:y1:step, x0:x1:step]``, so
consecutive obstacle pixels are ``step`` apart in full-resolution coordinates.
Running 8-connectivity on a full-resolution mask would therefore find every
pixel isolated and return N clusters of one point each. The mask is built in
GRID coordinates instead (``(u − u_min) // step``), where neighbours really are
adjacent. ``(u − u_min)`` is a multiple of ``step`` by construction, so the
mapping is exact and injective.

ORDER OF OPERATIONS
-------------------
Connected components runs on the FULL pixel set; the 2 cm voxel downsample is
applied afterwards, INSIDE each label. The two are independent — voxelisation
changes which points contribute to a centroid, labelling decides which points
belong together — so the order is free, and doing it this way keeps
connectivity intact. (Voxelising first and then labelling the surviving pixels
would puncture every blob into a lattice of isolated cells and fragment it.)

The downsample is not a speed trick. A depth camera samples a near surface far
more densely than a far one — pixel density falls as 1/Z² — so the raw mean of
a cluster's points is pulled toward whatever part of it is closest to the
camera, and that bias MOVES as the object moves. Voxel centroids are uniform in
metres, so the reported centroid is the centroid of the observed SURFACE rather
than of the sampling pattern, and it does not drift with range. A tracker
differentiates this quantity; a range-dependent bias in it would come out as a
fabricated velocity.

``cv2`` is used for the labelling only, exactly as ``mask_builder`` one layer
up does for its dilations — it is already a hard dependency of this perception
path. No ROS anywhere here, so the module is unit-testable headless.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Sequence

import cv2
import numpy as np


@dataclass(frozen=True)
class Cluster:
    """One connected obstacle blob, reduced to the three numbers a tracker needs.

    Attributes:
        centroid_cam: (3,) float64 centroid in CAMERA frame [m], computed over
            voxel centroids so it is invariant to the camera's 1/Z² sampling
            density (see the module docstring).
        n_points: how many RAW obstacle pixels the blob contained — not how many
            voxels survived. It is the cluster's evidence weight: association
            and track confirmation upstream should trust a 4000-pixel torso more
            than a 12-pixel speckle, and only the raw count says which is which.
        radius: [m] largest distance from ``centroid_cam`` to any voxel
            centroid, i.e. the radius of the smallest sphere about the centroid
            that covers the observed surface. Floored at half a voxel so a
            single-voxel cluster still has a physical extent rather than a
            degenerate zero.
    """

    centroid_cam: np.ndarray
    n_points: int
    radius: float

    def contains(self, p_cam: np.ndarray, *, tol: float = 0.0) -> bool:
        """Is ``p_cam`` inside this cluster's covering sphere?

        Used by the tracker node to decide which track a control point's
        ``closest_point_human`` belongs to. A SPHERE, deliberately, not the
        exact point set: the depth camera sees only the front surface of an
        object, so the true extent is unknown behind it and any tighter test
        would reject points that are genuinely on the same body. Over-inclusion
        merely attributes a control point to a slightly wrong track; the
        consumer clamps the resulting velocity to the tightening half anyway.

        Args:
            p_cam: (3,) point in CAMERA frame [m].
            tol: [m] extra slack added to the radius.
        """
        d = np.asarray(p_cam, dtype=np.float64) - self.centroid_cam
        return bool(float(np.sqrt(d @ d)) <= self.radius + float(tol))


def voxel_downsample(points: np.ndarray, voxel_m: float) -> np.ndarray:
    """(M, 3) voxel centroids of ``points``, one per occupied ``voxel_m`` cell.

    Returns the MEAN of the points inside each cell rather than the cell centre:
    the cell centre quantises the result onto a lattice, and a lattice-quantised
    centroid moves in 2 cm steps, which a downstream differentiator would read
    as an obstacle that is stationary and then briefly very fast. The in-cell
    mean keeps the output continuous in the input.

    Cells are keyed by ``floor(p / voxel_m)`` on the RAW camera-frame
    coordinates — no re-centring — so the lattice is fixed in space and does not
    shift with the cluster, which would otherwise re-partition the same points
    differently from frame to frame.
    """
    pts = np.asarray(points, dtype=np.float64)
    if pts.size == 0:
        return np.empty((0, 3), dtype=np.float64)
    keys = np.floor(pts / float(voxel_m)).astype(np.int64)
    _, inv = np.unique(keys, axis=0, return_inverse=True)
    inv = inv.ravel()
    counts = np.bincount(inv).astype(np.float64)
    cent = np.empty((counts.size, 3), dtype=np.float64)
    for k in range(3):
        cent[:, k] = np.bincount(inv, weights=pts[:, k]) / counts
    return cent


def cluster_obstacles(
    p_cam: np.ndarray,
    u: np.ndarray,
    v: np.ndarray,
    *,
    step: int = 1,
    voxel_m: float = 0.02,
    connectivity: int = 8,
    min_points: int = 10,
    max_clusters: int = 16,
    max_radius: Optional[float] = None,
) -> List[Cluster]:
    """Group obstacle pixels into objects and reduce each to a :class:`Cluster`.

    Args:
        p_cam: (N, 3) obstacle points in CAMERA frame [m], as unprojected by
            ``DistanceEngine.compute`` Step 4.
        u: (N,) full-resolution pixel COLUMN of each point.
        v: (N,) full-resolution pixel ROW of each point.
        step: the ROI subsampling stride the pixels were drawn with. Pixel
            coordinates are divided by it before the mask is built, so
            neighbouring samples are adjacent — see the module docstring.
        voxel_m: [m] downsample cell size. 2 cm is comfortably below the size of
            anything that has to be tracked (a forearm is ~8 cm across) and
            comfortably above the depth noise of a D405/D455 at working range,
            so it averages sensor noise without averaging away the object.
        connectivity: 4 or 8, passed to ``cv2.connectedComponents``. 8 by
            default: a limb crossing the image diagonally is one object, and
            4-connectivity would cut it into a staircase of fragments.
        min_points: blobs with fewer RAW pixels than this are dropped. This is
            the speckle filter — an isolated depth artefact is a handful of
            pixels, and promoting one to a cluster would give the tracker a
            measurement to associate and eventually a spurious velocity.
        max_clusters: keep at most this many, LARGEST FIRST. Bounds the
            per-frame cost of the association step, which is O(tracks ×
            clusters). Dropping the smallest is the right sacrifice: the largest
            blobs are the ones carrying a real body.
        max_radius: [m] drop clusters larger than this, or ``None`` for no
            limit. This is the SCENE guard, and it is the opposite end of the
            same axis as ``min_points``.

            A depth camera pointed at a room returns the room. With the shipped
            ``max_depth_m`` of 4 m, the far wall, the table and the floor are
            all "obstacle pixels", 8-connectivity links them into a single blob
            spanning the whole frustum, and its centroid is not the position of
            anything. Worse, such a blob MERGES AND SPLITS between frames as the
            robot mask carves different holes in it, and each merge steps the
            centroid by tens of centimetres — which a differentiator reads as
            metres per second. Measured on rosbag/arm_complex: one cluster of
            radius 2.2–2.6 m whose centroid alternates between z = 2.28 m and
            z = 2.57 m, giving the tracked estimate a 0.198 m/s noise floor on
            segments where the true obstacle speed is zero.

            A cluster wider than a person is not a person, and a velocity
            estimated for it is meaningless. Dropping it is also the SAFE
            failure: no cluster means no track, no track means the message's
            zero defaults, and zero defaults mean the barrier falls back to the
            behaviour it has today. ``None`` (the default) preserves the
            pre-guard behaviour exactly.

    Returns:
        Clusters ordered by descending ``n_points``, ties broken by centroid
        coordinates. The order is DETERMINISTIC: greedy nearest-neighbour
        association upstream consumes this list in order, so a frame-to-frame
        reshuffle of equal-sized clusters would be a reshuffle of track ids.
        Empty list when there is nothing to cluster.
    """
    p_cam = np.asarray(p_cam)
    u = np.asarray(u).ravel()
    v = np.asarray(v).ravel()
    if p_cam.size == 0 or u.size == 0:
        return []
    if p_cam.shape[0] != u.size or p_cam.shape[0] != v.size:
        raise ValueError(
            f'p_cam has {p_cam.shape[0]} points but got {u.size}/{v.size} '
            f'pixel coordinates — they must be index-aligned')

    pts = p_cam.astype(np.float64, copy=False)
    st = max(1, int(step))

    # ── Sparse obstacle pixels → dense GRID mask (see module docstring) ──────
    uu = (u.astype(np.int64) - int(u.min())) // st
    vv = (v.astype(np.int64) - int(v.min())) // st
    mask = np.zeros((int(vv.max()) + 1, int(uu.max()) + 1), dtype=np.uint8)
    mask[vv, uu] = 255

    n_labels, label_img = cv2.connectedComponents(mask, connectivity=int(connectivity))
    if n_labels <= 1:
        return []

    lab = label_img[vv, uu]                       # (N,) label per obstacle point

    clusters: List[Cluster] = []
    # bincount over labels first: one pass to find which labels are even worth
    # gathering, instead of a boolean scan of the whole point set per label.
    sizes = np.bincount(lab, minlength=n_labels)
    for lb in range(1, n_labels):
        if sizes[lb] < min_points:
            continue
        blob = pts[lab == lb]
        cent_v = voxel_downsample(blob, voxel_m)
        if cent_v.shape[0] == 0:
            continue
        centroid = cent_v.mean(axis=0)
        radial = np.sqrt(((cent_v - centroid) ** 2).sum(axis=1))
        radius = max(float(radial.max()), 0.5 * float(voxel_m))
        if max_radius is not None and radius > float(max_radius):
            continue
        clusters.append(Cluster(centroid_cam=centroid,
                                n_points=int(sizes[lb]),
                                radius=radius))

    clusters.sort(key=lambda c: (-c.n_points, c.centroid_cam[0],
                                 c.centroid_cam[1], c.centroid_cam[2]))
    return clusters[:int(max_clusters)]
