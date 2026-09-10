"""Cluster centroids → tracked 3D obstacle velocity with a covariance.

WHY THIS EXISTS
---------------
``ConstraintBuilder._obstacle_speed`` estimates the obstacle's contribution to
the closing rate as a SCALAR RESIDUAL,

    v_obs = aᵀq̇ − ḋ_measured ,

then EMA-filters it at α = 0.7 and clamps it. Every property of that estimator
is a consequence of having no object identity:

* **it is scalar.** Only the component along the current n̂ is ever recovered,
  so the estimate has to be rebuilt from scratch the moment n̂ turns — which it
  does continuously as the arm moves. The obstacle's actual velocity vector is
  the same in every frame and would not need rebuilding.
* **it is per control point.** Eleven control points looking at ONE person
  produce eleven independent estimates of that person's motion, each from its
  own noisy distance channel, none of them pooling evidence.
* **it differences an argmin.** ``d`` is the distance to the closest pixel, and
  the closest pixel jumps between surface patches — and between OBJECTS. A jump
  from one object to another is a discontinuity, not a velocity, and nothing in
  the residual can tell the two apart.
* **it lags.** The α = 0.7 EMA at 30 Hz is a ~75 ms time constant, and it is
  there precisely because the raw residual cannot be trusted. 75 ms of lag on a
  1 m/s approach is 7.5 cm of barrier, against ``d_safe`` = 0.15 m.

A Kalman filter fixes the last two structurally rather than by tuning. It
smooths AND differentiates in one step, with a gain derived from the ratio of
process to measurement noise instead of a hand-set α, and it reports how
uncertain it is — which is what makes step 8's uncertainty-derived tightening
possible at all. An EMA cannot report that.

THE MOTION MODEL: CONSTANT ACCELERATION, WHITE-NOISE JERK
---------------------------------------------------------
State ``x = [p(3), v(3), a(3)]``, ordered as three blocks of three so that
``F = kron(F_1d, I₃)`` and every axis is independent.

Constant VELOCITY was the obvious alternative and is the wrong one here. The
obstacles this filter exists for are human limbs, which spend most of their
time accelerating: a reach starts and stops inside ~0.5 s. Under a CV model an
acceleration is unmodelled error, and the filter answers it the only way it
can — by lagging, then by inflating its own covariance until it catches up.
That lag is the exact quantity this whole pipeline is trying to remove.
Carrying ``a`` in the state means an accelerating approach is EXTRAPOLATED
rather than chased, which shows up as ``v`` leading a finite difference instead
of trailing it.

The price is one extra state per axis and slightly more noise on ``v`` when the
target really is moving at constant velocity. That is the right trade for a
safety filter: an over-eager ``v`` tightens a barrier, an under-eager one does
not tighten it in time. Same conservative asymmetry as the rest of the module.

Process noise is the standard WHITE-NOISE-JERK discretisation — jerk is modelled
as a continuous white process of PSD ``q_jerk``, so per axis

    Q = q_jerk · [[dt⁵/20, dt⁴/8, dt³/6],
                  [dt⁴/8,  dt³/3, dt²/2],
                  [dt³/6,  dt²/2, dt  ]]

which is the exact integral ``∫₀^dt F(τ) G Gᵀ F(τ)ᵀ dτ`` with ``G = [0,0,1]ᵀ``,
not the piecewise-constant approximation. It matters because the off-diagonal
``p``–``v`` and ``v``–``a`` correlations are what let a position measurement
correct the velocity at all, and the crude form gets their scale wrong at the
30 Hz dt this runs at.

Measurement is the cluster centroid: ``H`` selects ``p``, ``R = σ_meas²·I₃``.
``σ_meas`` is NOT the depth sensor's per-pixel noise. The measurement is a
centroid over hundreds of pixels, so pixel noise averages down to nothing; what
is left is the CENTROID's own wander — the observed surface changing shape as
the object rotates, an arm entering the blob, the exclusion mask clipping an
edge. That is centimetres, not millimetres, and setting σ_meas from the sensor
datasheet instead would make the filter trust a wandering centroid and manufacture
velocity out of it.

No ROS here; pure numpy, so it is unit-testable headless.
"""

from __future__ import annotations

from typing import Optional

import numpy as np

# State layout. Three blocks of three: [px py pz | vx vy vz | ax ay az].
NP = 3
NX = 9
IP = slice(0, 3)
IV = slice(3, 6)
IA = slice(6, 9)

#: Measurement matrix: the centroid observes POSITION only. Built once — it is
#: a constant, and rebuilding it per update was measurable at 30 Hz × N tracks.
H = np.zeros((NP, NX))
H[:, IP] = np.eye(3)


def transition(dt: float) -> np.ndarray:
    """(9, 9) constant-acceleration state transition over ``dt``."""
    f = np.array([[1.0, dt, 0.5 * dt * dt],
                  [0.0, 1.0, dt],
                  [0.0, 0.0, 1.0]])
    return np.kron(f, np.eye(3))


def process_noise(dt: float, q_jerk: float) -> np.ndarray:
    """(9, 9) white-noise-jerk process covariance over ``dt``.

    The exact ``∫₀^dt F(τ)GGᵀF(τ)ᵀ dτ`` for ``G = [0, 0, 1]ᵀ`` — see the module
    docstring for why the exact form rather than the piecewise-constant one.
    """
    dt2 = dt * dt
    dt3 = dt2 * dt
    dt4 = dt3 * dt
    dt5 = dt4 * dt
    qc = np.array([[dt5 / 20.0, dt4 / 8.0, dt3 / 6.0],
                   [dt4 / 8.0,  dt3 / 3.0, dt2 / 2.0],
                   [dt3 / 6.0,  dt2 / 2.0, dt]])
    return float(q_jerk) * np.kron(qc, np.eye(3))


class KalmanTrack:
    """One tracked obstacle: constant-acceleration 3D KF over a centroid.

    Args:
        p0: (3,) initial position [m] — the centroid that spawned the track.
        q_jerk: [m²/s⁵] jerk PSD. Together with ``sigma_meas`` this is THE
            tuning knob — only their RATIO matters, and it sets the filter's
            bandwidth. Measured on the synthetic reach in
            ``test_obstacle_kalman`` (rest → 1 m/s in 0.2 s, sampled at 30 Hz
            with 1 cm noise), sweeping q_jerk over three decades:

                q_jerk   spurious speed on a STATIC target   lag to 90 % of a
                         (mean / peak, m/s)                  reach onset
                  0.2         0.059 / 0.122                      153 ms
                  2.0         0.098 / 0.241                       87 ms
                 20.0         0.165 / 0.423                       53 ms

            2.0 is the chosen compromise, and it is deliberately on the
            RESPONSIVE side of the knee. The asymmetry is the reason: the
            consumer uses ``max(n̂ᵀv, 0)``, so noise can only ever TIGHTEN a
            barrier (and its mean contribution is only σ/√(2π) ≈ 0.4σ of the
            projected noise, because the negative half is discarded), whereas
            lag UNDER-estimates a real approach — 150 ms of lag at 1 m/s is
            15 cm of barrier against a d_safe of 0.15 m. Same conservative
            asymmetry as the rest of the filter.
        sigma_meas: [m] centroid measurement noise, one axis. See the module
            docstring: this is the CENTROID's wander, not the depth sensor's
            per-pixel noise. 1 cm.
        sigma_v0: [m/s] initial velocity uncertainty. Large on purpose — a new
            track knows nothing about its velocity, and a small value here would
            make the first few updates barely move ``v`` at all, which is the
            slow-start the residual estimator already suffers from.
        sigma_a0: [m/s²] initial acceleration uncertainty, same reasoning.
        track_id: stable integer identity, assigned by :class:`TrackManager`.
    """

    def __init__(
        self,
        p0: np.ndarray,
        *,
        q_jerk: float = 2.0,
        sigma_meas: float = 0.01,
        sigma_v0: float = 1.0,
        sigma_a0: float = 5.0,
        track_id: int = 0,
    ) -> None:
        self.track_id = int(track_id)
        self.q_jerk = float(q_jerk)
        self.sigma_meas = float(sigma_meas)
        self.R = (self.sigma_meas ** 2) * np.eye(3)

        self.x = np.zeros(NX)
        self.x[IP] = np.asarray(p0, dtype=np.float64).ravel()
        self.P = np.zeros((NX, NX))
        # The position block starts at the measurement noise, not at zero: the
        # centroid that spawned the track is itself a measurement.
        self.P[IP, IP] = (self.sigma_meas ** 2) * np.eye(3)
        self.P[IV, IV] = (float(sigma_v0) ** 2) * np.eye(3)
        self.P[IA, IA] = (float(sigma_a0) ** 2) * np.eye(3)

        #: Frames this track has been UPDATED on (not predicted). The consumer
        #: gates on it — a one-frame-old velocity estimate is noise.
        self.frames_seen = 1
        #: Consecutive predicts without an update. The manager kills on it.
        self.missed = 0
        #: Total predicts, for diagnostics.
        self.age = 1

    # ── Estimates ───────────────────────────────────────────────────────────

    @property
    def position(self) -> np.ndarray:
        return self.x[IP].copy()

    @property
    def velocity(self) -> np.ndarray:
        return self.x[IV].copy()

    @property
    def acceleration(self) -> np.ndarray:
        return self.x[IA].copy()

    @property
    def velocity_cov(self) -> np.ndarray:
        """(3, 3) ``P_vv``, the velocity block of the state covariance.

        This is what step 8 turns into a barrier margin: ``n̂ᵀP_vv n̂`` is the
        variance of the velocity component the barrier actually consumes, so
        the tightening is derived from the filter's own admitted uncertainty
        rather than from a fixed guess. Returned as a copy — the caller must not
        be able to reach into the filter's state.
        """
        return self.P[IV, IV].copy()

    @property
    def position_cov(self) -> np.ndarray:
        """(3, 3) ``P_pp``, the position block. What the latency compensation
        propagates forward: the obstacle is moved by ``v·t + ½a·t²`` and this,
        propagated the same way, says how far off that prediction can be."""
        return self.P[IP, IP].copy()

    @property
    def pos_vel_cov(self) -> np.ndarray:
        """(3, 3) ``P_pv``, the position-velocity cross block. Needed for the
        propagation ``P_pp(t) = P_pp + t(P_pv + P_pvᵀ) + t²P_vv`` — dropping
        the cross term would under-state the propagated uncertainty of a
        track whose position and velocity errors are correlated, which after a
        Kalman update they always are."""
        return self.P[IP, IV].copy()

    def speed_variance_along(self, n_hat: np.ndarray) -> float:
        """``n̂ᵀ P_vv n̂`` [m²/s²], the variance of the projected speed.

        Clamped at ≥ 0: ``P`` is symmetric positive semi-definite in exact
        arithmetic, but the Joseph-form update below can still leave a quadratic
        form at −1e−20, and a negative variance would produce a NaN margin the
        moment step 8 takes its square root.
        """
        n = np.asarray(n_hat, dtype=np.float64).ravel()
        return max(float(n @ self.P[IV, IV] @ n), 0.0)

    # ── Filter ──────────────────────────────────────────────────────────────

    def predict(self, dt: float) -> None:
        """Advance the state by ``dt`` seconds.

        A non-positive or implausibly large ``dt`` is IGNORED rather than
        applied. Perception timestamps arrive from a camera and are occasionally
        duplicated or reordered; a negative dt would run the model backwards and
        a 2 s gap would blow ``P`` up by ``dt⁵`` (a factor of 10⁷ against the
        nominal 33 ms), after which the next measurement is accepted
        unconditionally by a gate that has become infinitely wide.
        """
        dt = float(dt)
        if not (1e-6 < dt < 1.0):
            return
        F = transition(dt)
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + process_noise(dt, self.q_jerk)
        self.missed += 1
        self.age += 1

    def update(self, z: np.ndarray) -> None:
        """Correct with a measured centroid ``z`` (3,) in the filter's frame.

        A ROBUST (innovation-gated) update was tried here and removed: on the
        real centroid sequences of rosbag/arm_complex and
        rosbag/handratacker_object, down-weighting a measurement whose
        innovation exceeded 2, 3 or 5 standard deviations changed the
        fabricated velocity by less than a millimetre per second at every
        percentile. The wander this filter suffers from is not a few large
        outliers a gate can catch; it is the centroid moving a little, all the
        time. It is handled where it is consumed — see
        ``ConstraintBuilder._obstacle_speed_tracked``.
        """
        z = np.asarray(z, dtype=np.float64).ravel()
        S = self.innovation_cov()
        K = self.P @ H.T @ np.linalg.inv(S)
        y = z - self.x[IP]
        self.x = self.x + K @ y
        # Joseph form, not (I − KH)P. The short form is only valid for the exact
        # optimal gain and loses symmetry as soon as round-off creeps in; over
        # minutes of 30 Hz operation that drift is what turns P asymmetric and
        # then indefinite, at which point speed_variance_along() starts
        # returning negative numbers and step 8's sqrt returns NaN.
        IKH = np.eye(NX) - K @ H
        self.P = IKH @ self.P @ IKH.T + K @ self.R @ K.T
        self.P = 0.5 * (self.P + self.P.T)      # kill residual asymmetry
        self.missed = 0
        self.frames_seen += 1

    # ── Association support ─────────────────────────────────────────────────

    def predicted_measurement(self) -> np.ndarray:
        """(3,) where this track expects the next centroid to be."""
        return self.x[IP].copy()

    def innovation_cov(self) -> np.ndarray:
        """(3, 3) ``S = H P Hᵀ + R``, the covariance of the innovation."""
        return self.P[IP, IP] + self.R

    def mahalanobis(self, z: np.ndarray) -> float:
        """Mahalanobis distance from this track's prediction to ``z``.

        The gating metric, and it has to be Mahalanobis rather than Euclidean:
        a coasting track's ``P`` grows every frame it is not updated, so its
        gate widens on its own and a re-appearing obstacle is re-associated
        instead of spawning a duplicate id. A fixed Euclidean radius cannot do
        that — sized for the occlusion case it would swap ids between two close
        obstacles on every frame.
        """
        y = np.asarray(z, dtype=np.float64).ravel() - self.x[IP]
        try:
            return float(np.sqrt(max(y @ np.linalg.solve(self.innovation_cov(), y), 0.0)))
        except np.linalg.LinAlgError:
            return float('inf')

    def __repr__(self) -> str:      # pragma: no cover - diagnostic only
        return (f'KalmanTrack(id={self.track_id} seen={self.frames_seen} '
                f'missed={self.missed} p={np.round(self.x[IP], 3).tolist()} '
                f'v={np.round(self.x[IV], 3).tolist()})')


# ── Track management ─────────────────────────────────────────────────────────
#
# WHY A LIFECYCLE AT ALL
#
# A Kalman filter answers "given that these measurements are of the same thing,
# what is its velocity". Something has to decide which measurements are of the
# same thing, and that decision is where a tracker earns or loses its keep:
#
# * associate too eagerly and two obstacles passing each other SWAP identities.
#   The velocity attached to each then reverses in one frame, which is the
#   largest possible error the estimator can make — and it makes it exactly when
#   two people are close to the robot, i.e. at the worst moment.
# * associate too reluctantly and every occlusion — a hand passing behind the
#   arm, the exclusion mask briefly swallowing a blob — kills the track and
#   spawns a new one. The new track's velocity starts at zero and takes ~5
#   frames to recover, so a continuously approaching obstacle is reported as
#   stationary for 150 ms every time it is briefly hidden.
# * confirm too eagerly and a single-frame depth artefact becomes a track with
#   a velocity, which the consumer then uses to tighten a barrier.
#
# The three defences are, in order: a Mahalanobis GATE (not a Euclidean radius —
# a coasting track's covariance grows, so its gate opens by itself and the
# occlusion case is re-associated without also widening the gate of a
# well-observed track), M-of-N CONFIRMATION before a track is visible to any
# consumer, and COASTING for K frames before death.
#
# ASSOCIATION IS GREEDY NEAREST-NEIGHBOUR, GLOBALLY ORDERED
#
# Every (track, cluster) pair inside the gate is scored, the whole set is sorted
# by Mahalanobis distance, and pairs are consumed best-first with each track and
# cluster used at most once. That is not optimal in the Hungarian sense, but it
# is deterministic, O(T·C log(T·C)), dependency-free, and it differs from the
# optimal assignment only when two gates overlap AND the cost matrix is nearly
# degenerate — which is precisely the case where the prediction is what
# disambiguates, not the assignment algorithm. Scoring against the PREDICTED
# measurement is what makes two crossing obstacles keep their ids: at the moment
# they overlap the positions are identical but the predictions are not.


def _measurement(c) -> np.ndarray:
    """(3,) measurement from a Cluster, a centroid-carrying object, or a point."""
    p = getattr(c, 'centroid_cam', c)
    return np.asarray(p, dtype=np.float64).ravel()


class TrackManager:
    """Nearest-neighbour tracker over :class:`KalmanTrack` instances.

    Args:
        q_jerk / sigma_meas / sigma_v0 / sigma_a0: forwarded to every new
            :class:`KalmanTrack`.
        gate_mahalanobis: association gate. A Mahalanobis distance, so it is in
            units of the filter's OWN uncertainty: 3.0 admits ~97 % of true
            associations for a 3-DoF innovation (χ²₃ at 9.0). Raising it trades
            lost tracks for identity swaps.
        gate_max_m: [m] hard Euclidean ceiling applied on top of the gate. The
            Mahalanobis gate widens without bound while a track coasts, and
            after enough missed frames it would accept a cluster anywhere in
            the room. This is the backstop that keeps a resurrected track
            physically plausible — 0.5 m is what a human limb can cover in the
            ~0.17 s a 5-frame coast lasts.
        confirm_hits / confirm_window: M-of-N birth rule. A track becomes
            visible to consumers after ``confirm_hits`` updates within the last
            ``confirm_window`` frames of its life. M-of-N rather than M
            CONSECUTIVE: a real obstacle at the edge of the depth range flickers,
            and demanding consecutive hits would keep restarting a track that is
            genuinely there, while a one-frame artefact fails both rules anyway.
        max_missed: frames a track may coast before it is deleted.
        max_tracks: hard ceiling on live tracks. A degenerate frame (the
            exclusion mask failing, the whole scene reading as obstacle) can
            otherwise produce hundreds of clusters and hence hundreds of tracks,
            and the association cost is O(T·C).
    """

    def __init__(
        self,
        *,
        q_jerk: float = 2.0,
        sigma_meas: float = 0.01,
        sigma_v0: float = 1.0,
        sigma_a0: float = 5.0,
        gate_mahalanobis: float = 3.0,
        gate_max_m: float = 0.5,
        confirm_hits: int = 3,
        confirm_window: int = 5,
        max_missed: int = 5,
        max_tracks: int = 12,
    ) -> None:
        self.q_jerk = float(q_jerk)
        self.sigma_meas = float(sigma_meas)
        self.sigma_v0 = float(sigma_v0)
        self.sigma_a0 = float(sigma_a0)
        self.gate_mahalanobis = float(gate_mahalanobis)
        self.gate_max_m = float(gate_max_m)
        self.confirm_hits = int(confirm_hits)
        self.confirm_window = int(confirm_window)
        self.max_missed = int(max_missed)
        self.max_tracks = int(max_tracks)

        self.tracks: list = []
        self._next_id = 1        # ids start at 1: 0 is the message's "no track"
        self._hits: dict = {}    # track_id → recent hit/miss history (deque-ish)
        self._confirmed: set = set()
        #: cluster index (of the LAST :meth:`step` call) → the track it fed.
        #: Published so a consumer can go from a POINT to a track: it finds the
        #: cluster the point falls in, then this map names the track. Without it
        #: the only route back is "nearest track position", which is wrong
        #: exactly when it matters — one large cluster's far edge can be nearer
        #: to a different track's centre than to its own.
        self.last_assoc: dict = {}

    # ── Query ───────────────────────────────────────────────────────────────

    def confirmed_tracks(self) -> list:
        """Tracks a consumer may act on, newest evidence first.

        A TENTATIVE track is deliberately invisible here. It still runs — it has
        to, or it could never accumulate the hits to be confirmed — but nothing
        downstream sees a velocity from it, so a speckle that survives two frames
        cannot reach the barrier.
        """
        return [t for t in self.tracks if t.track_id in self._confirmed]

    def is_confirmed(self, track: KalmanTrack) -> bool:
        return track.track_id in self._confirmed

    # ── Step ────────────────────────────────────────────────────────────────

    def step(self, clusters, dt: float) -> list:
        """Advance every track by ``dt``, associate ``clusters``, return the
        CONFIRMED tracks.

        Args:
            clusters: this frame's measurements — a sequence of
                :class:`~franka_experiments.utils.obstacle_clusters.Cluster`,
                of anything exposing ``centroid_cam``, or of bare (3,) points.
                The bare-point form is what ``ObstacleTrackPipeline`` uses: it
                transforms the centroids into the BASE frame before handing them
                over, because a Kalman filter differentiates its input and a
                velocity only rotates cleanly between frames whose transform is
                constant. May be empty — every track then coasts, which is the
                correct response to a frame where perception saw nothing.
            dt: [s] since the previous frame, from the CAPTURE clock.

        Returns:
            The confirmed tracks, i.e. exactly :meth:`confirmed_tracks`.
        """
        for t in self.tracks:
            t.predict(dt)

        z = [_measurement(c) for c in clusters]
        pairs = self._associate(z)

        self.last_assoc = {}
        used_c = set()
        for ti, ci in pairs:
            self.tracks[ti].update(z[ci])
            self.last_assoc[ci] = self.tracks[ti]
            used_c.add(ci)
        hit_t = {ti for ti, _ in pairs}

        for i, t in enumerate(self.tracks):
            self._record(t.track_id, i in hit_t)

        # Births from every cluster that matched nothing. Done AFTER the update
        # pass so a newborn is never itself an association candidate this frame
        # — otherwise two clusters from one split blob would spawn a track and
        # then immediately feed it, confirming a fragment.
        for ci, zc in enumerate(z):
            if ci in used_c or len(self.tracks) >= self.max_tracks:
                continue
            born = KalmanTrack(
                zc, q_jerk=self.q_jerk, sigma_meas=self.sigma_meas,
                sigma_v0=self.sigma_v0, sigma_a0=self.sigma_a0,
                track_id=self._next_id)
            self.tracks.append(born)
            self.last_assoc[ci] = born
            self._record(self._next_id, True)
            self._next_id += 1

        self._reap()
        return self.confirmed_tracks()

    # ── Association ─────────────────────────────────────────────────────────

    def _associate(self, z) -> list:
        """[(track_index, cluster_index)], greedy best-first inside the gate."""
        if not self.tracks or not z:
            return []
        cand = []
        for ti, t in enumerate(self.tracks):
            p_pred = t.predicted_measurement()
            for ci, zc in enumerate(z):
                if float(np.linalg.norm(zc - p_pred)) > self.gate_max_m:
                    continue
                d = t.mahalanobis(zc)
                if d <= self.gate_mahalanobis:
                    cand.append((d, ti, ci))
        # Sorted by distance, then by (track, cluster) index: the tie-break is
        # what makes the result independent of the order Python happened to
        # build `cand` in, and hence reproducible frame to frame.
        cand.sort(key=lambda r: (r[0], r[1], r[2]))
        taken_t, taken_c, out = set(), set(), []
        for _, ti, ci in cand:
            if ti in taken_t or ci in taken_c:
                continue
            taken_t.add(ti)
            taken_c.add(ci)
            out.append((ti, ci))
        return out

    # ── Lifecycle ───────────────────────────────────────────────────────────

    def _record(self, track_id: int, hit: bool) -> None:
        h = self._hits.setdefault(track_id, [])
        h.append(bool(hit))
        if len(h) > self.confirm_window:
            del h[:-self.confirm_window]
        if track_id not in self._confirmed and sum(h) >= self.confirm_hits:
            self._confirmed.add(track_id)

    def _reap(self) -> None:
        keep = []
        for t in self.tracks:
            if t.missed > self.max_missed:
                self._hits.pop(t.track_id, None)
                self._confirmed.discard(t.track_id)
            else:
                keep.append(t)
        self.tracks = keep

    def reset(self) -> None:
        """Drop every track and its history (camera restart, resolution change).

        Ids are NOT rewound: a consumer holding an old id must see it disappear,
        never see it silently refer to a different obstacle.
        """
        self.tracks = []
        self._hits.clear()
        self._confirmed.clear()
        self.last_assoc = {}
