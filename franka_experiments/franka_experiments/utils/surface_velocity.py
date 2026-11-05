"""How fast the obstacle SURFACE is closing, estimated without a centroid.

WHY THIS EXISTS
---------------
``LinkDistance.obstacle_velocity`` is the velocity of a CLUSTER CENTROID from
the tracker's Kalman state. A centroid is not a measurement of a body when the
cluster that defines it keeps changing shape, and in a furnished cell it changes
shape constantly: the robot mask carves a moving hole out of every surface
behind the arm, so a desk MERGES with the wall behind it on one frame and SPLITS
from it on the next. Each merge steps the centroid, and a differentiator reads
that step as metres per second. Measured on the DIAG lab bag
(``datasets/experiments_irim2.zip``): ``obstacle_velocity`` reached **7.78 m/s**
on the wire, in a cell where the fastest thing present was a hand.

The static-scene hold and the age gate (``utils/static_scene.py``) withhold that
number once they can prove the track is the room. This module attacks it one
step earlier, and without needing an identity at all: it measures the quantity
the barrier actually consumes, directly, from geometry.

WHAT IT MEASURES
----------------
The HOCBF row only ever uses the NORMAL component of the obstacle's velocity,
``n̂ᵀv`` — that is the whole of the obstacle's contribution to ``ḣ``. And
``closest_point_human`` is a point ON THE OBSTACLE'S SURFACE, published every
frame, tracked or not. So

    v_n  =  n̂ᵀ (p_h(t) − p_h(t−Δ)) / Δ          [m/s, >0 closing]

is an estimate of exactly the consumed quantity, built from two surface points
and no centroid, no cluster membership and no track id.

The projection onto n̂ is what makes this work while the arm is moving. The
nearest surface point SLIDES as the control point moves — it is an argmin, not a
material point — but that sliding is TANGENTIAL to the surface, and n̂ is the
surface normal (for the nearest point of any smooth surface, the direction to
the control point IS the normal). So the sliding falls in the null space of the
projection to first order: sweeping the arm along a static wall gives v_n ≈ 0,
which is the right answer, while a wall advancing at 0.3 m/s gives 0.3.

WHAT IT REFUSES TO MEASURE
--------------------------
Two frames can only be differenced when they are the same patch of the same
body, so the estimate is withheld — ``None``, not zero — whenever they might
not be:

* the argmin HOPPED. ``‖Δp_h‖ > max(jump_m, obstacle_velocity_max · Δt)``: the
  nearest point cannot have travelled further than the fastest obstacle this
  filter admits. On the DIAG bag the nearest point moved more than 10 cm between
  updates on 14–35 % of them — reattachment to another surface, which a finite
  difference cannot tell from an approach. Same physical criterion, and the same
  relax-when-a-frame-is-dropped shape, as the identity guard in
  ``cbf_state_rows._obstacle_identity_changed``.
* the gap between frames was too long (``max_dt_s``): the anchor is stale and
  the point in between is unobserved.
* fewer than ``min_samples`` samples, or less than ``min_span_s`` of time, have
  survived since the last anchor, so a short or sparse window never produces an
  estimate on its own.

WHY A LEAST-SQUARES SLOPE AND NOT A DIFFERENCE
----------------------------------------------
The first version of this module took the MEDIAN OF CONSECUTIVE DIFFERENCES,
and it was measured wrong — the refutation is worth keeping, because the shape
of the estimator turned out to matter more than anything else in it. On
``sim_runs/live_p34`` (the furnished DIAG lab, one ball sweeping through the
arm's circle, ground truth from ``truth.npz``), over the rows whose obstacle
point was ON the ball while the ball was genuinely closing:

    estimator                        v_n p50   corr with truth   capped below truth
    tracker's own centroid (ref.)     0.148         +0.68              15.8 %
    median of 5 consecutive diffs     0.020         +0.47              69.5 %
    difference over a 5-frame base    0.130         +0.65              22.4 %
    least-squares over 12 frames      0.141         +0.86              12.7 %
    LEAST-SQUARES OVER 20 FRAMES      0.131         +0.94               3.4 %

against a truth of 0.152-0.156 m/s. The median of consecutive differences
attenuates the signal NINEFOLD: each single-frame difference is dominated by the
argmin's own per-frame sliding, so the distribution the median picks the middle
of is noise centred near zero, and the real translation is thrown away with the
outliers. A slope over a window instead CANCELS that sliding (it is a
telescoping difference) and keeps the translation, and least squares does it
without the newest frame's noise entering whole.

The last column is the number that matters, and it is why this is safe to use as
a cap: a cap built on the 20-frame slope withholds real anticipation on a
genuinely approaching obstacle LESS OFTEN THAN TRUSTING THE TRACKER DOES
(3.4 % against 15.8 %), because the tracker's own estimate under-reads too. The
same window also makes the place memory in ``utils/static_scene.py`` strictly
more discriminating: it now sees 0.13 m/s on a moving ball where it used to see
0.02, which is the difference between a cell being released and not.

The strict test is HARM: rows where the tracker's own value was adequate (at or
above the truth) and the cap pulled it below. Over three 30 s runs of the
furnished DIAG lab that was **0, 0 and 2 rows** (0.0 %, 0.0 %, 0.2 % of the
genuinely-closing rows on the ball), because on 56-100 % of those rows the
tracker was ALREADY below the truth and the cap changed nothing about that.

WHAT IT IS WORTH, AND WHERE THAT CAN BE SHOWN
---------------------------------------------
Not in this simulator. The phantom this exists for is a property of a DEPTH
pipeline whose clusters merge and split around the robot mask, and Gazebo's
obstacle is a clean sphere with an accurate tracker: measured on the same runs,
the room's tracked ``v_obs`` p95 is only 0.17-0.27 m/s and the cap moves it to
0.16-0.26 — nothing, against the 3.11 m/s of conditioned ``v_obs`` the same
quantity reached on the DIAG HARDWARE bag. So in simulation this feature is
safe and inert, and its benefit can only be demonstrated on hardware or on
``datasets/experiments_irim2.zip``. It ships OFF for exactly that reason. The
place memory it also feeds (``enable_static_scene_voxels``) is a different
story: that one reaches 27 % of the rows in the same runs.

``None`` IS THE SAFE ANSWER, and that is the design: the consumer uses this
estimate only to CAP a claim the tracker is making, so no estimate means no cap,
i.e. exactly today's behaviour. The failure mode of every guard above is
therefore "the tracker is believed", never "the obstacle is believed to be
still". A ball crossing at 4 m/s at 30 Hz moves 13 cm per frame and trips the
hop gate: it gets no cap, and its Kalman velocity reaches the QP untouched.

Pure logic: no ROS. The consumer must call :meth:`update` at most once per
perception frame per control point; being called faster (the QP rebuilds at
~100 Hz against a ~30–90 Hz camera) is handled by ``min_dt_s``, which returns
the cached estimate without differencing a sub-frame interval.

Unit-tested in ``test/test_cbf_surface_velocity.py``.
"""
from __future__ import annotations

from typing import Optional

import numpy as np

#: Forget a control point not updated for this long. Bounds the memory; the key
#: set is one entry per control point, so this only matters across reconfigures.
IDLE_DROP_S = 2.0
#: Hard cap on samples per window, so a stalled stream cannot grow it.
MAX_SAMPLES = 256


class _Win:
    __slots__ = ('ts', 'ps', 'v')

    def __init__(self, t: float, p: np.ndarray):
        self.ts: list[float] = [float(t)]
        self.ps: list[np.ndarray] = [p]
        #: Last reported estimate, so being called faster than the camera
        #: delivers costs nothing and returns the same answer.
        self.v: Optional[float] = None


class SurfaceNormalSpeed:
    """Per-control-point normal closing speed of the nearest surface point.

    Args:
        window_s: [s] span the slope is fitted over. 0.22 s is ~20 samples at
            the shipped 90 Hz profile, the window measured in the table above.
        min_samples: samples the window must hold before any estimate.
        min_span_s: [s] time the window must span before any estimate, so the
            slope is never fitted over an interval shorter than the noise.
        jump_m: [m] floor on the hop test, for short dt. Sized ABOVE the
            argmin's own patch hopping (~5 cm) and below the separation of two
            distinct bodies.
        max_dt_s: [s] longest gap between consecutive samples that may be kept
            in one window.
        min_dt_s: [s] shortest. Below it the caller is rebuilding faster than
            the camera delivers and the cached estimate is returned unchanged.
        v_max: [m/s] fastest obstacle the filter admits; sets the hop test's
            physical term. Pass ``obstacle_velocity_max``.
    """

    def __init__(self, *, window_s: float = 0.22, min_samples: int = 8,
                 min_span_s: float = 0.08, jump_m: float = 0.10,
                 max_dt_s: float = 0.2, min_dt_s: float = 0.004,
                 v_max: float = 2.0):
        self._window = max(float(window_s), 1e-3)
        self._min_n = max(int(min_samples), 2)
        self._min_span = max(float(min_span_s), 0.0)
        self._jump = max(float(jump_m), 0.0)
        self._max_dt = max(float(max_dt_s), 1e-3)
        self._min_dt = max(float(min_dt_s), 0.0)
        self._v_max = max(float(v_max), 0.0)
        self._st: dict = {}

    # ── bookkeeping ──────────────────────────────────────────────────────────
    def update(self, key, p_h, n_hat, t: float) -> Optional[float]:
        """Feed this control point's surface point and normal for frame ``t``.

        Returns the fitted normal speed [m/s, >0 closing], or ``None`` for "no
        estimate" — first sight, a hop, a gap, or too little history yet.
        """
        p = np.asarray(p_h, dtype=np.float64).ravel()
        n = np.asarray(n_hat, dtype=np.float64).ravel()
        if p.size != 3 or n.size != 3:
            return None
        if not (np.all(np.isfinite(p)) and np.all(np.isfinite(n))):
            return None
        t = float(t)
        st = self._st.get(key)
        if st is None:
            self._st[key] = _Win(t, p)
            return None
        dt = t - st.ts[-1]
        if dt <= self._min_dt:
            # Same perception frame (or a stamp that did not advance): the
            # window is unchanged, so the answer is too.
            return st.v
        if dt > self._max_dt or float(np.linalg.norm(p - st.ps[-1])) > max(
                self._jump, self._v_max * dt):
            # A gap, or the argmin reattached to another surface. Either way
            # the window no longer describes one patch of one body: start it
            # again and report NO ESTIMATE, which leaves the consumer at its
            # unguarded default.
            st.ts, st.ps, st.v = [t], [p], None
            return None
        st.ts.append(t)
        st.ps.append(p)
        t_min = t - self._window
        while len(st.ts) > 2 and st.ts[0] < t_min:
            st.ts.pop(0)
            st.ps.pop(0)
        while len(st.ts) > MAX_SAMPLES:
            st.ts.pop(0)
            st.ps.pop(0)
        st.v = self._slope(st, n)
        return st.v

    def _slope(self, st: _Win, n: np.ndarray) -> Optional[float]:
        """Least-squares d/dt of the normal coordinate over the window.

        Every stored position is projected on the CURRENT normal, not on the
        one it arrived with: n̂ rotates as the arm moves, and the quantity the
        barrier consumes is the obstacle's speed along the normal the row is
        being built with now.
        """
        if len(st.ts) < self._min_n:
            return None
        ts = np.asarray(st.ts)
        if ts[-1] - ts[0] < self._min_span:
            return None
        tc = ts - ts.mean()
        den = float(tc @ tc)
        if den <= 1e-12:
            return None
        y = np.asarray(st.ps) @ n
        return float(tc @ (y - y.mean()) / den)

    def prune(self, t: float, idle_s: float = IDLE_DROP_S) -> None:
        """Forget control points not updated for ``idle_s``."""
        dead = [k for k, st in self._st.items() if t - st.ts[-1] > idle_s]
        for k in dead:
            del self._st[k]

    # ── queries ──────────────────────────────────────────────────────────────
    def last(self, key) -> Optional[float]:
        """The current estimate for ``key`` without advancing anything."""
        st = self._st.get(key)
        return None if st is None else st.v

    @property
    def n_keys(self) -> int:
        return len(self._st)


def surface_speed_from_params(P, enabled: Optional[bool] = None
                              ) -> Optional[SurfaceNormalSpeed]:
    """Build the estimator from the parameter namespace, or None when off.

    None is what keeps the rows bit-identical: the consumer then never asks a
    question and no code path changes.
    """
    on = (bool(getattr(P, 'enable_surface_velocity_cap', False))
          if enabled is None else enabled)
    if not on:
        return None
    return SurfaceNormalSpeed(
        window_s=float(getattr(P, 'surface_velocity_window_s', 0.22)),
        min_samples=int(getattr(P, 'surface_velocity_min_samples', 8)),
        min_span_s=float(getattr(P, 'surface_velocity_min_span_s', 0.08)),
        jump_m=float(getattr(P, 'surface_velocity_jump_m', 0.10)),
        max_dt_s=float(getattr(P, 'surface_velocity_max_dt_s', 0.2)),
        min_dt_s=float(getattr(P, 'surface_velocity_min_dt_s', 0.004)),
        v_max=float(getattr(P, 'obstacle_velocity_max', 2.0)))
