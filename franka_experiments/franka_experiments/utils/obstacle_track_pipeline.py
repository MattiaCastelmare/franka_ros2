"""Obstacle cloud → clusters → tracks → a velocity for one control point.

WHY THIS IS A SEPARATE MODULE FROM THE NODE
-------------------------------------------
Everything the obstacle tracker actually DECIDES lives here, so it can be run
and asserted on a synthetic sequence with no ROS, no camera and no launch file:
which pixels form an object, which object is the same object as last frame, and
which object a given control point is looking at. ``nodes/obstacle_tracker_node``
is left as a thin shell that owns subscriptions, TF and message construction —
the parts that cannot be unit-tested anyway.

THE FRAME QUESTION, AND WHY TRACKING HAPPENS IN THE BASE FRAME
--------------------------------------------------------------
Clusters come out of ``obstacle_clusters`` in CAMERA frame, and the message the
result is written into carries BASE-frame points. The pipeline converts the
centroids to base BEFORE feeding the filter, and tracks there.

That ordering is not cosmetic. A Kalman filter DIFFERENTIATES its input, so any
frame change applied after it would have to be applied to a velocity — and a
velocity only rotates cleanly between two frames if the transform between them
is constant. Converting first makes the filter's output a base-frame velocity by
construction, and if the extrinsic ever becomes time-varying (an eye-in-hand
camera), the camera's own motion is differenced out by the transform instead of
being reported as obstacle motion.

NOTE THE STANDING ASSUMPTION IT INHERITS: ``R_base``/``t_base`` come from
``camera_extrinsics.yaml``, which is a STATIC calibration. The whole distance
pipeline already assumes a fixed camera; this module does not make that
assumption any worse, but it does not fix it either.

MATCHING A CONTROL POINT TO A TRACK
-----------------------------------
The barrier is built per control point, and each control point reports the
single obstacle point nearest to it (``closest_point_human``). The velocity that
belongs to that control point is therefore the velocity of whatever OBJECT that
point sits on — so the lookup is point → cluster → track, never "nearest track
centre". The distinction matters for a large cluster: its far edge can easily be
closer to a different track's centre than to its own, and a nearest-centre
lookup would hand the control point the wrong obstacle's velocity.

Only CONFIRMED tracks are ever returned. A tentative track is one that has been
seen once or twice, and its velocity is dominated by its own initial
uncertainty; publishing it would put a number derived from noise onto the wire
at exactly the moment a consumer has no way to tell it apart from a real one.

Pure numpy. No ROS.
"""

from __future__ import annotations

from typing import List, Optional, Sequence, Tuple

import numpy as np

from franka_experiments.utils.obstacle_clusters import Cluster, cluster_obstacles
from franka_experiments.utils.obstacle_tracker import KalmanTrack, TrackManager


class ObstacleTrackPipeline:
    """Per-frame: cluster the obstacle cloud, advance the tracks, answer lookups.

    Args:
        voxel_m / min_cluster_points / max_clusters / connectivity /
        depth_jump / max_cluster_radius: forwarded to
            :func:`~franka_experiments.utils.obstacle_clusters.cluster_obstacles`.
        contains_tol: [m] slack added to a cluster's radius when testing whether
            a control point's nearest obstacle point belongs to it. Needed
            because the radius is measured over VOXEL CENTROIDS while the query
            is a RAW surface point, so a legitimate point on the object's rim
            sits up to half a voxel plus the voxel-averaging error outside the
            sphere. 0.05 m also absorbs the one-frame skew between the depth
            frame this pipeline clustered and the one the incoming distance
            message was computed from.
        default_dt: [s] used when a frame carries no usable timestamp, or when
            the stamp difference is implausible. Without it a missing stamp
            would silently freeze every filter (``predict`` ignores dt ≤ 0), and
            the tracks would report the velocity they had when the clock broke.
        tracker: an existing :class:`TrackManager` to use; otherwise one is
            built from ``**tracker_kw``.
    """

    def __init__(
        self,
        *,
        voxel_m: float = 0.02,
        min_cluster_points: int = 10,
        max_clusters: int = 16,
        max_cluster_radius: Optional[float] = None,
        connectivity: int = 8,
        depth_jump: float = 0.10,
        contains_tol: float = 0.05,
        default_dt: float = 1.0 / 30.0,
        tracker: Optional[TrackManager] = None,
        **tracker_kw,
    ) -> None:
        self.voxel_m = float(voxel_m)
        self.min_cluster_points = int(min_cluster_points)
        self.max_clusters = int(max_clusters)
        self.max_cluster_radius = (None if max_cluster_radius is None
                                   else float(max_cluster_radius))
        self.connectivity = int(connectivity)
        self.depth_jump = float(depth_jump)
        self.contains_tol = float(contains_tol)
        self.default_dt = float(default_dt)
        self.tracker = tracker if tracker is not None else TrackManager(**tracker_kw)

        self._prev_stamp: Optional[float] = None
        #: (centroid_base, radius, track_or_None) for the last frame, in the
        #: cluster order :func:`cluster_obstacles` returned.
        self._entries: List[Tuple[np.ndarray, float, Optional[KalmanTrack]]] = []

        # Diagnostics, read by the node's status line and by nothing else.
        self.last_clusters: List[Cluster] = []
        self.last_dt: float = 0.0
        self.last_n_points: int = 0

    # ── Per-frame update ────────────────────────────────────────────────────

    def update(
        self,
        cloud,
        R_base: np.ndarray,
        t_base: np.ndarray,
        *,
        stamp: Optional[float] = None,
    ) -> List[KalmanTrack]:
        """Cluster ``cloud``, advance every track, and return the confirmed ones.

        Args:
            cloud: an ``ObstacleCloud`` (or anything with ``p_cam``/``u``/``v``/
                ``step``) — the pixels the distance pass already selected. ``None``
                or an empty cloud is a valid frame meaning "perception saw no
                obstacle": every track then coasts, which is the correct
                response, rather than being silently held frozen.
            R_base: (3, 3) camera→base rotation of the static extrinsic.
            t_base: (3,) camera→base translation [m].
            stamp: [s] CAPTURE time of the frame. Receipt time carries transport
                and executor jitter, and dividing a position difference by a
                jittered dt puts that jitter straight into the velocity — the
                same lesson ``ObstacleSnap.t_cap`` records one layer down.
        """
        dt = self.default_dt
        if stamp is not None and self._prev_stamp is not None:
            d = float(stamp) - self._prev_stamp
            if 1e-4 < d < 1.0:
                dt = d
        if stamp is not None:
            self._prev_stamp = float(stamp)
        self.last_dt = dt

        clusters = self._cluster(cloud)
        self.last_clusters = clusters
        self.last_n_points = sum(c.n_points for c in clusters)

        # Camera → base BEFORE the filter sees the measurement, never after.
        # See the module docstring: the filter differentiates, and a velocity
        # only rotates cleanly between frames related by a CONSTANT transform.
        R = np.asarray(R_base, dtype=np.float64)
        t = np.asarray(t_base, dtype=np.float64).ravel()
        z_base = [R @ c.centroid_cam + t for c in clusters]

        self.tracker.step(z_base, dt)

        assoc = self.tracker.last_assoc
        self._entries = [
            (z_base[i], float(c.radius), assoc.get(i))
            for i, c in enumerate(clusters)
        ]
        return self.tracker.confirmed_tracks()

    def _cluster(self, cloud) -> List[Cluster]:
        if cloud is None:
            return []
        p_cam = getattr(cloud, 'p_cam', None)
        if p_cam is None or len(p_cam) == 0:
            return []
        return cluster_obstacles(
            p_cam, cloud.u, cloud.v, step=int(getattr(cloud, 'step', 1)),
            voxel_m=self.voxel_m, connectivity=self.connectivity,
            depth_jump=self.depth_jump,
            min_points=self.min_cluster_points, max_clusters=self.max_clusters,
            max_radius=self.max_cluster_radius)

    # ── Lookup ──────────────────────────────────────────────────────────────

    def track_for_point(self, p_base) -> Optional[KalmanTrack]:
        """The confirmed track whose cluster contains ``p_base``, or ``None``.

        ``p_base`` is a control point's ``closest_point_human`` in BASE frame.
        When several clusters cover it — they overlap freely, the spheres are
        covers and not partitions — the one whose CENTROID is nearest wins. That
        is the tightest available reading of "which object is this point on".

        ``None`` whenever the point belongs to nothing confirmed, which the
        caller must publish as the all-zero "no track" state. Returning a track
        on a guess is the one thing this must not do: the consumer treats a
        published velocity as evidence, and evidence attached to the wrong body
        is worse than no evidence at all.
        """
        if not self._entries:
            return None
        p = np.asarray(p_base, dtype=np.float64).ravel()
        best, best_d = None, np.inf
        for centroid, radius, trk in self._entries:
            if trk is None or not self.tracker.is_confirmed(trk):
                continue
            d = float(np.linalg.norm(p - centroid))
            if d <= radius + self.contains_tol and d < best_d:
                best, best_d = trk, d
        return best

    def velocity_for_point(self, p_base) -> Tuple[int, int, np.ndarray, np.ndarray]:
        """``(track_id, frames_seen, v_base (3,), P_vv_base (3, 3))`` for a point.

        The all-zero tuple ``(0, 0, zeros(3), zeros((3, 3)))`` when the point
        matches nothing — exactly the message's documented "no track" defaults,
        which the consumer already treats as "contribute nothing".

        No rotation is applied: the filter already runs in the base frame (see
        the module docstring), so its state and covariance are base-frame
        quantities as they stand.
        """
        trk = self.track_for_point(p_base)
        if trk is None:
            return 0, 0, np.zeros(3), np.zeros((3, 3))
        return (int(trk.track_id), int(trk.frames_seen),
                trk.velocity, trk.velocity_cov)

    # ── Lifecycle ───────────────────────────────────────────────────────────

    def reset(self) -> None:
        """Drop all tracks and clock state (camera restart, resolution change)."""
        self.tracker.reset()
        self._prev_stamp = None
        self._entries = []
        self.last_clusters = []
        self.last_n_points = 0

    def describe(self) -> str:
        return (f'clusters={len(self.last_clusters)} '
                f'tracks={len(self.tracker.tracks)} '
                f'confirmed={len(self.tracker.confirmed_tracks())} '
                f'dt={self.last_dt * 1e3:.1f}ms')
