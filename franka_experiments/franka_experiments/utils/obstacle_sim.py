"""A synthetic obstacle rendered into a depth image, for end-to-end runs.

WHY THIS IS IN THE PACKAGE AND NOT ONLY IN A TEST
-------------------------------------------------
The avoidance behaviour this repo is built around — an obstacle approaching
faster than the arm can brake, and the arm stepping aside — cannot be exercised
without an obstacle that moves. None of the recorded bags contains one (they
were recorded to exercise the ARM: their only obstacle pixels are the static
room at 2-4 m), and rehearsing it with a real person in front of a real 7-DoF
arm is exactly the thing to avoid doing first.

So the same sphere the offline comparison uses can be rendered into the live
depth stream, which gives a complete end-to-end run — real perception, real
clustering, real tracking, real QP, real controller — where only the obstacle's
EXISTENCE is synthetic. Everything downstream is the shipped code path: the
same exclusion mask, the same ROI stride, the same depth quantisation.

IT IS A SIMULATION AID AND IT ANNOUNCES ITSELF
----------------------------------------------
Rendering a fake obstacle into a SAFETY pipeline is exactly the kind of thing
that must never be on by accident: the arm will move away from something that
is not there, and on hardware that is a surprise motion. Hence the flag is off
by default, the node logs a WARNING every time it is on, and this module does
nothing unless it is explicitly constructed.

Pure numpy. No ROS.
"""

from __future__ import annotations

import numpy as np


# ── Synthetic obstacle injection ─────────────────────────────────────────────
#
# WHY THIS IS HERE, AND WHY IT IS NOT CHEATING
#
# The four bags in this repo were recorded to exercise the ARM, not the
# avoidance: their only obstacle pixels are the static room at 2-4 m. Replaying
# them compares two estimators on a scene where the true obstacle velocity is
# zero everywhere, which measures noise and nothing else — no approach, no lag,
# no ground truth.
#
# So a sphere with a KNOWN trajectory is rendered into each depth frame before
# the pipeline sees it. Everything downstream is untouched and real: the same
# exclusion mask, the same ROI stride, the same depth quantisation, the same
# clustering, the same association, the same Jacobians from the same recorded
# joint states. Only the obstacle's existence is synthetic — and because its
# velocity is known exactly, both estimators can be scored against TRUTH rather
# than merely against each other.
#
# The sphere's FRONT SURFACE is rendered, not a flat disc, because that is what
# a depth camera returns and because the surface centroid then translates at
# exactly the sphere's velocity (a flat disc's would too, but its apparent size
# would not shrink with range, and the cluster's radius feeds the association
# gate).


class InjectedSphere:
    """A sphere SHUTTLING along a line in CAMERA frame, rendered into depth frames.

    A shuttle (approach, then recede, then approach again) rather than a
    one-way pass, for two reasons that both matter to the measurement:

    * the true ``v_obs`` is then a SQUARE WAVE, with a genuine sign change twice
      per period. A one-way pass gives a nearly constant truth, against which
      any lag measurement is meaningless — a constant signal correlates equally
      well at every shift. The sign flips are the edges the lag is read from.
    * the sphere stays in the near field the whole time, so it remains the
      nearest obstacle for the control points in front of it instead of
      disappearing for most of each cycle.

    Args:
        c0: (3,) near end of the shuttle, camera frame [m].
        vel: (3,) velocity while APPROACHING, camera frame [m/s]. Its direction
            sets the travel axis and its magnitude the speed; the amplitude
            comes from ``amplitude``.
        radius: [m] sphere radius. 0.15 m is a forearm in a sleeve, and — at the
            shipped ``pixel_step`` of 10 — it is also the smallest object that
            still lands 30+ samples on the ROI grid at 1.8 m. A 0.08 m sphere
            gives 8 samples there, below ``min_cluster_points``, so it is
            invisible to the clusterer: a real limitation of the stride, not of
            the tracker.
        amplitude: [m] half-stroke.
        period: [s] full out-and-back cycle.
    """

    def __init__(self, c0, vel, radius=0.15, period=2.0, amplitude=0.5):
        self.c0 = np.asarray(c0, dtype=np.float64)
        self.vel = np.asarray(vel, dtype=np.float64)
        self.speed = float(np.linalg.norm(self.vel))
        self.dir = (self.vel / self.speed if self.speed > 0
                    else np.array([0.0, 0.0, -1.0]))
        self.radius = float(radius)
        self.period = float(period)
        self.amplitude = float(amplitude)
        self.t0 = None

    def _phase(self, t):
        if self.t0 is None:
            self.t0 = t
        return ((t - self.t0) % self.period) / self.period

    def centre(self, t):
        s = self._phase(t)
        tri = 2.0 * s if s < 0.5 else 2.0 * (1.0 - s)   # 0 -> 1 -> 0
        return self.c0 + self.dir * (self.amplitude * tri)

    def velocity(self, t):
        """(3,) camera-frame velocity now — the SIGNED square wave."""
        v = 2.0 * self.amplitude / self.period
        return self.dir * (v if self._phase(t) < 0.5 else -v)

    def render(self, depth, t, K):
        """Paint the sphere's front surface into ``depth`` (uint16, mm). In place.

        Written with ``np.minimum`` so the sphere OCCLUDES what is behind it and
        is itself occluded by anything in front — the same visibility rule the
        real sensor obeys, and the reason the robot arm correctly hides it when
        it passes between the two.
        """
        c = self.centre(t)
        if c[2] <= self.radius:
            return
        fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
        # Bounding box of the projected sphere, padded by one pixel.
        ang = self.radius / c[2]
        du, dv = int(abs(fx) * ang * 1.4) + 2, int(abs(fy) * ang * 1.4) + 2
        u0 = int(cx + fx * c[0] / c[2])
        v0 = int(cy + fy * c[1] / c[2])
        H, W = depth.shape
        us = np.arange(max(0, u0 - du), min(W, u0 + du + 1))
        vs = np.arange(max(0, v0 - dv), min(H, v0 + dv + 1))
        if us.size == 0 or vs.size == 0:
            return
        uu, vv = np.meshgrid(us, vs)
        d = np.stack([(uu - cx) / fx, (vv - cy) / fy, np.ones_like(uu, float)], -1)
        # |s*d - c|^2 = r^2  ->  (d.d)s^2 - 2(d.c)s + (c.c - r^2) = 0
        A = (d * d).sum(-1)
        B = -2.0 * (d @ c)
        C = float(c @ c) - self.radius ** 2
        disc = B * B - 4.0 * A * C
        hit = disc >= 0.0
        if not hit.any():
            return
        s = np.zeros_like(A)
        s[hit] = (-B[hit] - np.sqrt(disc[hit])) / (2.0 * A[hit])
        z_mm = (s * 1000.0).astype(depth.dtype)
        patch = depth[vs[0]:vs[-1] + 1, us[0]:us[-1] + 1]
        # depth==0 means "no return"; the sphere must overwrite those, and must
        # otherwise win only where it is nearer.
        take = hit & (s > 0) & ((patch == 0) | (z_mm < patch))
        patch[take] = z_mm[take]
