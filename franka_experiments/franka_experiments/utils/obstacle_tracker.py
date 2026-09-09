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
        """Correct with a measured centroid ``z`` (3,) in the filter's frame."""
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
