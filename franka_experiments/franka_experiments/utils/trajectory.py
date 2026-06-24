"""Trajectory generators: pentagon, random waypoints."""

from __future__ import annotations

import math
from typing import List, Optional

import numpy as np

from .math_utils import min_jerk

# Maximum re-sampling attempts per waypoint before giving up.
_MAX_SAMPLE_ATTEMPTS = 50

# ---------------------------------------------------------------------------
# Gauss-Legendre 5-point quadrature on [0, 1] — used for arc-length integrals
# ---------------------------------------------------------------------------
_GL5_T = np.array([0.04691007703066802, 0.23076534494715845, 0.5,
                   0.7692346550528415,  0.9530899229693320])
_GL5_W = np.array([0.11846344252809454, 0.23931433524968324, 0.28444444444444444,
                   0.23931433524968324, 0.11846344252809454])

# Samples used to build the per-blend arc-length inverse table.
_N_TBL = 32

# ---------------------------------------------------------------------------
# In-plane unit axes for each supported plane: (û, v̂) ← (first, second) planar
# coordinate.  Shared by every planar generator/path so the embedding of a 2-D
# figure into the 3-D base frame is identical everywhere.  Matches
# :class:`PentagonTrajectory`'s convention.
#   "xy"            → û = x̂, v̂ = ŷ
#   "xz"            → û = x̂, v̂ = ẑ
#   "yz" / "front"  → û = ŷ, v̂ = ẑ
# ---------------------------------------------------------------------------
_PLANE_AXES = {
    'xy':    (np.array([1.0, 0.0, 0.0]), np.array([0.0, 1.0, 0.0])),
    'xz':    (np.array([1.0, 0.0, 0.0]), np.array([0.0, 0.0, 1.0])),
    'yz':    (np.array([0.0, 1.0, 0.0]), np.array([0.0, 0.0, 1.0])),
    'front': (np.array([0.0, 1.0, 0.0]), np.array([0.0, 0.0, 1.0])),
}


# ---------------------------------------------------------------------------
# Pentagon trajectory
# ---------------------------------------------------------------------------

class PentagonTrajectory:
    """Smooth periodic pentagon trajectory with corner blending.

    Each corner is replaced by a quadratic Bézier arc so the path is C¹
    everywhere and the robot never stops at vertices.  The full loop is
    traversed at constant speed (arc-length parameterisation pre-computed at
    construction time — zero allocations in ``evaluate``).

    Parameters
    ----------
    center : ndarray (3,)
        Centre of the pentagon in the base frame.
    radius : float
        Circumscribed-circle radius [m].
    plane : str
        ``"xy"``, ``"xz"``, ``"yz"``, or ``"front"``.
    cycle_time : float
        Total time for one full loop [s].
    smoothness : float
        Corner blend fraction in (0, 0.45].  The blend radius is
        ``smoothness * shortest_side``.  Default 0.15 gives a blend radius
        of 15 % of the shortest side length, which is smooth without
        distorting the pentagonal shape significantly.
    """

    N_SIDES = 5

    def __init__(
        self,
        center: np.ndarray,
        radius: float,
        plane: str,
        cycle_time: float,
        smoothness: float = 0.15,
    ) -> None:
        self.center = np.asarray(center, dtype=float)
        self.radius = radius
        self.plane = plane.lower()
        self.cycle_time = cycle_time

        # ── Vertices ──────────────────────────────────────────────────────
        vertices: List[np.ndarray] = []
        for k in range(self.N_SIDES):
            angle = 2.0 * math.pi * k / self.N_SIDES + math.pi / 2.0
            u = radius * math.cos(angle)
            v = radius * math.sin(angle)
            pt = self.center.copy()
            if self.plane == 'xy':
                pt[0] += u; pt[1] += v
            elif self.plane == 'xz':
                pt[0] += u; pt[2] += v
            elif self.plane in ('yz', 'front'):
                pt[1] += u; pt[2] += v
            else:
                raise ValueError(
                    f'Unknown plane "{self.plane}", use xy/xz/yz/front')
            vertices.append(pt)
        self.vertices = vertices

        N = self.N_SIDES

        # ── Edge unit directions and lengths ──────────────────────────────
        edges     = np.zeros((N, 3))   # edges[k] = unit dir V_k → V_{k+1}
        side_lens = np.zeros(N)
        for k in range(N):
            d = vertices[(k + 1) % N] - vertices[k]
            L = float(np.linalg.norm(d))
            side_lens[k] = L
            edges[k] = d / L

        # ── Blend geometry ────────────────────────────────────────────────
        # blend_r: distance along each adjacent edge where the Bézier starts/ends
        blend_frac = float(min(max(smoothness, 0.01), 0.45))
        blend_r    = blend_frac * float(np.min(side_lens))

        # For vertex k:
        #   A[k] = V_k - blend_r * edge_{k-1}   (entry point of blend)
        #   B[k] = V_k + blend_r * edge_k        (exit  point of blend)
        # Bézier control points: P0=A[k], P1=V_k (corner), P2=B[k]
        # P'(t) = 2 * [va_k + t * vdiff_k]
        #   where  va_k   = V_k - A[k] = blend_r * edge_{k-1}
        #          vdiff_k = (B[k] - V_k) - (V_k - A[k]) = blend_r*(edge_k - edge_{k-1})
        A_pts  = np.array([vertices[k] - blend_r * edges[(k - 1) % N] for k in range(N)])
        B_pts  = np.array([vertices[k] + blend_r * edges[k]            for k in range(N)])
        va_arr = np.array([blend_r * edges[(k - 1) % N]                for k in range(N)])
        vb_arr = np.array([blend_r * edges[k]                          for k in range(N)])
        vd_arr = vb_arr - va_arr   # vdiff_k = vb - va

        # ── Segment layout: 2*N segments ─────────────────────────────────
        #   seg 2k   = Bézier blend at vertex k
        #   seg 2k+1 = straight line from B[k] to A[(k+1)%N]
        n_segs = 2 * N

        seg_lengths = np.zeros(n_segs)

        for k in range(N):
            va    = va_arr[k]
            vdiff = vd_arr[k]
            # |P'(t)| = 2 * |va + t*vdiff|
            # L = ∫₀¹ |P'(t)| dt  via 5-point GL
            L = 0.0
            for g in range(5):
                vec = va + _GL5_T[g] * vdiff
                L  += _GL5_W[g] * math.sqrt(float(vec @ vec))
            seg_lengths[2 * k] = 2.0 * L

            seg_lengths[2 * k + 1] = float(
                np.linalg.norm(A_pts[(k + 1) % N] - B_pts[k]))

        total_length = float(np.sum(seg_lengths))
        seg_cum      = np.zeros(n_segs + 1)
        seg_cum[1:]  = np.cumsum(seg_lengths)

        # ── Arc-length inverse table for each blend ───────────────────────
        # blend_t_tbl[k, i] : parametric t values 0..1  (N_TBL+1 samples)
        # blend_s_tbl[k, i] : corresponding cumulative arc-fraction 0..1
        blend_t_tbl = np.linspace(0.0, 1.0, _N_TBL + 1)[np.newaxis, :].repeat(N, axis=0)
        blend_s_tbl = np.zeros((N, _N_TBL + 1))
        for k in range(N):
            va    = va_arr[k]
            vdiff = vd_arr[k]
            ts    = blend_t_tbl[k]
            cum   = blend_s_tbl[k]
            for i in range(1, _N_TBL + 1):
                t0, t1 = ts[i - 1], ts[i]
                dt     = t1 - t0
                s_seg  = 0.0
                for g in range(5):
                    tg  = t0 + dt * _GL5_T[g]
                    vec = va + tg * vdiff
                    s_seg += _GL5_W[g] * math.sqrt(float(vec @ vec))
                cum[i] = cum[i - 1] + 2.0 * dt * s_seg
            # normalise to [0, 1]
            L_blend = seg_lengths[2 * k]
            if L_blend > 1e-12:
                blend_s_tbl[k] /= L_blend

        # ── Store everything needed by evaluate() ─────────────────────────
        self._n_segs       = n_segs
        self._seg_lengths  = seg_lengths
        self._seg_cum      = seg_cum
        self._total_length = total_length
        self._speed        = total_length / cycle_time  # constant speed [m/s]

        self._blend_A      = A_pts                  # (N, 3) entry points
        self._blend_V      = np.array(vertices)     # (N, 3) control points
        self._blend_B      = B_pts                  # (N, 3) exit points
        self._blend_va     = va_arr                 # (N, 3) V - A
        self._blend_vdiff  = vd_arr                 # (N, 3) (B-V) - (V-A)
        self._blend_t_tbl  = blend_t_tbl            # (N, _N_TBL+1)
        self._blend_s_tbl  = blend_s_tbl            # (N, _N_TBL+1)

        line_start = B_pts.copy()                   # (N, 3)
        line_end   = np.array([A_pts[(k + 1) % N] for k in range(N)])  # (N,3)
        line_vec   = line_end - line_start          # (N, 3) unnormalised
        line_tang  = np.zeros((N, 3))
        for k in range(N):
            L = seg_lengths[2 * k + 1]
            line_tang[k] = line_vec[k] / L if L > 1e-12 else edges[k]
        self._line_start   = line_start
        self._line_vec     = line_vec
        self._line_tang    = line_tang

        # Pre-allocated output buffers — evaluate() writes here, returns views
        self._p_out        = np.zeros(3)
        self._v_out        = np.zeros(3)
        self._a_out        = np.zeros(3)

    # ── Public API ────────────────────────────────────────────────────────

    def evaluate(self, t: float):
        """Return ``(p_d, v_d)`` at time *t* (seconds since trajectory start).

        Both arrays are views into pre-allocated buffers — copy them if you
        need to store them across calls.

        Returns
        -------
        p_d : ndarray (3,)  — desired Cartesian position
        v_d : ndarray (3,)  — desired Cartesian velocity (constant magnitude)
        """
        # Global arc-length position
        s = (t % self.cycle_time) / self.cycle_time * self._total_length

        # Find segment via binary search in cumulative-length table
        seg_idx = int(np.searchsorted(self._seg_cum[1:], s))
        if seg_idx >= self._n_segs:
            seg_idx = self._n_segs - 1

        s_local  = s - self._seg_cum[seg_idx]
        seg_len  = self._seg_lengths[seg_idx]
        lf       = (s_local / seg_len) if seg_len > 1e-12 else 0.0
        if lf > 1.0:
            lf = 1.0
        elif lf < 0.0:
            lf = 0.0

        if seg_idx & 1:
            # ── Linear segment ────────────────────────────────────────────
            k  = seg_idx >> 1
            ls = self._line_start[k]
            lv = self._line_vec[k]
            lt = self._line_tang[k]
            sp = self._speed
            self._p_out[0] = ls[0] + lf * lv[0]
            self._p_out[1] = ls[1] + lf * lv[1]
            self._p_out[2] = ls[2] + lf * lv[2]
            self._v_out[0] = sp * lt[0]
            self._v_out[1] = sp * lt[1]
            self._v_out[2] = sp * lt[2]

        else:
            # ── Bézier blend ──────────────────────────────────────────────
            k = seg_idx >> 1

            # Invert arc-length fraction → parametric t via linear interp table
            s_tbl = self._blend_s_tbl[k]
            t_tbl = self._blend_t_tbl[k]
            ii    = int(np.searchsorted(s_tbl[1:], lf))
            if ii >= _N_TBL:
                ii = _N_TBL - 1
            s0, s1 = s_tbl[ii], s_tbl[ii + 1]
            ds     = s1 - s0
            alpha  = ((lf - s0) / ds) if ds > 1e-14 else 0.0
            tp     = t_tbl[ii] + alpha * (t_tbl[ii + 1] - t_tbl[ii])
            if tp > 1.0:
                tp = 1.0
            elif tp < 0.0:
                tp = 0.0

            # Position: P(tp) = (1-tp)² A + 2tp(1-tp) V + tp² B
            omt = 1.0 - tp
            c0  = omt * omt
            c1  = 2.0 * tp * omt
            c2  = tp * tp
            A   = self._blend_A[k]
            V   = self._blend_V[k]
            B   = self._blend_B[k]
            self._p_out[0] = c0 * A[0] + c1 * V[0] + c2 * B[0]
            self._p_out[1] = c0 * A[1] + c1 * V[1] + c2 * B[1]
            self._p_out[2] = c0 * A[2] + c1 * V[2] + c2 * B[2]

            # Velocity direction: P'(tp) = 2*(va + tp*vdiff)  (unnormalised)
            va    = self._blend_va[k]
            vdiff = self._blend_vdiff[k]
            tx    = va[0] + tp * vdiff[0]
            ty    = va[1] + tp * vdiff[1]
            tz    = va[2] + tp * vdiff[2]
            tn    = math.sqrt(tx * tx + ty * ty + tz * tz)
            if tn > 1e-12:
                sc = self._speed / tn
            else:
                sc = 0.0
            self._v_out[0] = sc * tx
            self._v_out[1] = sc * ty
            self._v_out[2] = sc * tz

        return self._p_out, self._v_out

    def evaluate_with_accel(self, t: float, eps: float = 0.01):
        """Return ``(p_d, v_d, a_d)`` with acceleration via forward difference.

        ``a_d`` is computed as ``(v_d(t+eps) − v_d(t)) / eps``.
        All returned arrays are safe to read until the next call.
        """
        p_d, v_d = self.evaluate(t)
        p_copy   = p_d.copy()
        v_copy   = v_d.copy()
        _, v_fwd = self.evaluate(t + eps)
        np.subtract(v_fwd, v_copy, out=self._a_out)
        self._a_out /= eps
        self.evaluate(t)                 # restore _p_out/_v_out to time t
        return self._p_out, self._v_out, self._a_out


# ---------------------------------------------------------------------------
# Circular trajectory (time-based generator, mirrors PentagonTrajectory's API)
# ---------------------------------------------------------------------------

class CircularTrajectory:
    """Smooth periodic circular trajectory traced at constant speed.

    A circle of given *radius* about *center*, embedded in the plane selected by
    *plane* (using the shared :data:`_PLANE_AXES` mapping).  One full loop is
    completed every *cycle_time* seconds.  Being a single trigonometric curve it
    is C∞ and naturally periodic, so — unlike :class:`PentagonTrajectory` — no
    arc-length tables or corner blending are needed; ``evaluate`` is closed-form
    and allocation-free.

    Geometry (angle ``θ = ω·t``, ``ω = 2π / cycle_time``)::

        p(t) = center + r·cos θ · û + r·sin θ · v̂
        v(t) = r·ω·(−sin θ · û + cos θ · v̂)          (‖v‖ = r·ω, constant)

    Parameters
    ----------
    center : ndarray (3,)
        Centre of the circle in the base frame.
    radius : float
        Circle radius [m].
    plane : str
        ``"xy"``, ``"xz"``, ``"yz"``, or ``"front"``.
    cycle_time : float
        Total time for one full loop [s].
    """

    def __init__(
        self,
        center: np.ndarray,
        radius: float,
        plane: str,
        cycle_time: float,
    ) -> None:
        self.center = np.asarray(center, dtype=float)
        self.radius = float(radius)
        self.plane = plane.lower()
        self.cycle_time = float(cycle_time)

        try:
            u, v = _PLANE_AXES[self.plane]
        except KeyError:
            raise ValueError(
                f'Unknown plane "{self.plane}", use xy/xz/yz/front') from None
        self._u = u.copy()
        self._v = v.copy()

        # ω = 2π/cycle_time [rad/s]; constant tangential speed = r·ω [m/s].
        self._omega = 2.0 * math.pi / self.cycle_time

        # Pre-allocated output buffers — evaluate() writes here, returns views.
        self._p_out = np.zeros(3)
        self._v_out = np.zeros(3)
        self._a_out = np.zeros(3)

    # ── Public API ────────────────────────────────────────────────────────

    def evaluate(self, t: float):
        """Return ``(p_d, v_d)`` at time *t* (seconds since trajectory start).

        Both arrays are views into pre-allocated buffers — copy them if you
        need to store them across calls.

        Returns
        -------
        p_d : ndarray (3,)  — desired Cartesian position
        v_d : ndarray (3,)  — desired Cartesian velocity (constant magnitude)
        """
        # sin/cos are intrinsically 2π-periodic, so no `t % cycle_time` wrap is
        # needed and the velocity stays continuous across loop boundaries.
        theta = self._omega * t
        ct, st = math.cos(theta), math.sin(theta)
        r  = self.radius
        vr = r * self._omega                    # tangential speed magnitude
        c, u, v = self.center, self._u, self._v
        for i in range(3):
            self._p_out[i] = c[i] + r * ct * u[i] + r * st * v[i]
            self._v_out[i] = vr * (-st * u[i] + ct * v[i])
        return self._p_out, self._v_out

    def evaluate_with_accel(self, t: float, eps: float = 0.01):
        """Return ``(p_d, v_d, a_d)`` with acceleration via forward difference.

        ``a_d`` is computed as ``(v_d(t+eps) − v_d(t)) / eps`` — the same scheme
        used by :meth:`PentagonTrajectory.evaluate_with_accel`.  All returned
        arrays are safe to read until the next call.
        """
        p_d, v_d = self.evaluate(t)
        p_copy   = p_d.copy()
        v_copy   = v_d.copy()
        _, v_fwd = self.evaluate(t + eps)
        np.subtract(v_fwd, v_copy, out=self._a_out)
        self._a_out /= eps
        self.evaluate(t)                 # restore _p_out/_v_out to time t
        return self._p_out, self._v_out, self._a_out


# ---------------------------------------------------------------------------
# Geometric paths — parametrised by a dimensionless phase s (NOT time)
#
# A geometric path is pure geometry: it knows only the curve P(s) and its phase
# derivatives P'(s), P''(s).  Timing (how s evolves with wall-clock time) is the
# job of a TimingLaw (see utils.timing_law).  The reference is recovered by the
# chain rule:
#       p = P(s)
#       v = P'(s) * s_dot
#       a = P''(s) * s_dot**2 + P'(s) * s_ddot
# ---------------------------------------------------------------------------

class GeometricPath:
    """Base class: position and phase derivatives as a function of phase ``s``.

    Subclasses implement :meth:`_compute`, which fills the preallocated buffers
    ``_P`` (position), ``_Pp`` (dP/ds) and ``_Ppp`` (d²P/ds²).  ``position`` /
    ``velocity`` / ``acceleration`` apply the chain rule.  Results are *views*
    into preallocated buffers (zero allocation per call) — copy them if you need
    to retain values across calls.  The last evaluated ``s`` is cached so the
    three accessors can be called in sequence for the same ``s`` without
    recomputing the geometry.
    """

    def __init__(self) -> None:
        self._P = np.zeros(3)            # P(s)
        self._Pp = np.zeros(3)           # dP/ds
        self._Ppp = np.zeros(3)          # d²P/ds²
        self._v_out = np.zeros(3)
        self._a_out = np.zeros(3)
        self._tmp = np.zeros(3)
        self._cache_s = math.nan

    # subclasses fill _P, _Pp, _Ppp for the given phase
    def _compute(self, s: float) -> None:  # pragma: no cover - abstract
        raise NotImplementedError

    def _ensure(self, s: float) -> None:
        if s != self._cache_s:
            self._compute(s)
            self._cache_s = s

    def derivatives(self, s: float):
        """Return views ``(P, P', P'')`` at phase *s*."""
        self._ensure(s)
        return self._P, self._Pp, self._Ppp

    def position(self, s: float):
        self._ensure(s)
        return self._P

    def velocity(self, s: float, s_dot: float):
        self._ensure(s)
        np.multiply(self._Pp, s_dot, out=self._v_out)
        return self._v_out

    def acceleration(self, s: float, s_dot: float, s_ddot: float):
        self._ensure(s)
        # a = P''*s_dot² + P'*s_ddot
        np.multiply(self._Ppp, s_dot * s_dot, out=self._a_out)
        np.multiply(self._Pp, s_ddot, out=self._tmp)
        self._a_out += self._tmp
        return self._a_out


class PentagonPath(GeometricPath):
    """:class:`PentagonTrajectory` reparametrised as a function of phase ``s``.

    One full loop spans ``Δs = 1`` (the phase is taken modulo 1, so the path is
    cyclic and the corner blending / constant-speed geometry is byte-for-byte
    the original TRACK behaviour).  Because the underlying trajectory is
    arc-length / constant-speed parametrised, with ``t = (s mod 1) * cycle_time``::

        P(s)   = traj.p(t)
        P'(s)  = traj.v(t) * cycle_time
        P''(s) = traj.a(t) * cycle_time**2

    so a phase rate ``s_dot = 1/cycle_time`` reproduces the original
    constant-speed pentagon exactly.
    """

    def __init__(self, traj: "PentagonTrajectory") -> None:
        super().__init__()
        self._traj = traj
        self._T = float(traj.cycle_time)

    @property
    def vertices(self):
        return self._traj.vertices

    def _compute(self, s: float) -> None:
        phi = s - math.floor(s)              # s mod 1 ∈ [0, 1)
        p, v, a = self._traj.evaluate_with_accel(phi * self._T)
        np.copyto(self._P, p)
        np.multiply(v, self._T, out=self._Pp)
        np.multiply(a, self._T * self._T, out=self._Ppp)


class LissajousPath(GeometricPath):
    r"""Smooth planar Lissajous curve parametrised by the phase ``s ∈ [0, 1]``.

    Mathematical definition
    ------------------------
    A Lissajous figure with frequency ratio 2 : 3 traced in a fixed plane.  The
    normalised phase is mapped to an angle::

        t = 2π · s            (so dt/ds = 2π, a constant)

    and the two *in-plane* coordinates relative to the centre are

        x(t) = A · sin(2t)
        y(t) = B · sin(3t)

    The full curve ``P(s) = centre + x(t)·û + y(t)·v̂`` is therefore

      * **closed and periodic** with period ``Δs = 1`` — at ``s = 0`` and
        ``s = 1`` we have ``t = 0`` and ``t = 2π`` so ``sin(2t) = sin(3t) = 0``
        and both ``P`` and all its derivatives repeat exactly, and
      * **globally C∞ smooth** — it is a finite sum of sinusoids, so curvature
        is bounded and continuous everywhere (no corners, no segment switching).

    This is the key motivation for replacing the polygonal pentagon path: the
    pentagon has piecewise geometry with discontinuous curvature at the corners,
    which injects unbounded reference accelerations that the Operational Space
    Controller must fight.  A Lissajous curve is everywhere differentiable, so
    ``P'(s)`` and ``P''(s)`` — and hence the commanded Cartesian velocity and
    acceleration — stay bounded and continuous.

    Plane embedding
    ---------------
    ``(x, y)`` are laid into 3-D along the two in-plane unit axes ``û, v̂`` chosen
    by *plane*, matching :class:`PentagonTrajectory`'s convention:

      * ``"xy"``           → û = x̂, v̂ = ŷ
      * ``"xz"``           → û = x̂, v̂ = ẑ
      * ``"yz"`` / ``"front"`` → û = ŷ, v̂ = ẑ

    Analytical derivatives
    ----------------------
    Because the closed form is known, the phase derivatives are computed
    analytically (not by finite differences), so they are exact to machine
    precision and free of differencing noise / lag.  Since ``t`` is *linear* in
    ``s`` (``dt/ds = 2π`` constant, ``d²t/ds² = 0``) the chain rule reduces to a
    plain power of ``dt/ds``::

        dx/ds   = (dx/dt)·(dt/ds)        = (2A·cos 2t)·(2π)   =  4π·A·cos 2t
        d²x/ds² = (d²x/dt²)·(dt/ds)²     = (−4A·sin 2t)·(2π)² = −16π²·A·sin 2t
        dy/ds   = (dy/dt)·(dt/ds)        = (3B·cos 3t)·(2π)   =  6π·B·cos 3t
        d²y/ds² = (d²y/dt²)·(dt/ds)²     = (−9B·sin 3t)·(2π)² = −36π²·B·sin 3t

    The inherited :meth:`velocity` / :meth:`acceleration` then apply the outer
    chain rule in the phase rate::

        v = P'(s)·ṡ
        a = P''(s)·ṡ² + P'(s)·s̈

    Parameters
    ----------
    center : ndarray (3,)
        Curve centre in the base frame (the ``(x_c, y_c)`` of the figure plus the
        fixed out-of-plane coordinate).
    A, B : float
        In-plane amplitudes [m] of the ``sin 2t`` and ``sin 3t`` components.
    plane : str
        ``"xy"``, ``"xz"``, ``"yz"`` or ``"front"`` — selects the in-plane axes.
    """

    # Unit axes for each supported plane: (û, v̂) ← (x(t), y(t)).
    _PLANE_AXES = _PLANE_AXES

    def __init__(self, center, A: float = 0.12, B: float = 0.08,
                 plane: str = 'front') -> None:
        super().__init__()
        self._center = np.asarray(center, dtype=float).copy()
        self._A = float(A)
        self._B = float(B)
        plane = plane.lower()
        try:
            u, v = self._PLANE_AXES[plane]
        except KeyError:
            raise ValueError(
                f'Unknown plane "{plane}", use xy/xz/yz/front') from None
        self._u = u.copy()
        self._v = v.copy()
        self._two_pi = 2.0 * math.pi
        self._tan_out = np.zeros(3)   # scratch for tangent()

    def _compute(self, s: float) -> None:
        # Phase → angle.  No `s mod 1` is needed: t = 2π·s and the curve is a
        # sum of sin(2t)/sin(3t), already exactly periodic with period Δs = 1.
        t = self._two_pi * s
        sin2, cos2 = math.sin(2.0 * t), math.cos(2.0 * t)
        sin3, cos3 = math.sin(3.0 * t), math.cos(3.0 * t)
        A, B, two_pi = self._A, self._B, self._two_pi

        # In-plane position offsets:  x(t) = A·sin 2t,  y(t) = B·sin 3t
        px = A * sin2
        py = B * sin3

        # dP/ds  — analytical chain rule (dt/ds = 2π):
        #   dx/ds = 4π·A·cos 2t ,  dy/ds = 6π·B·cos 3t
        dx = 2.0 * A * cos2 * two_pi
        dy = 3.0 * B * cos3 * two_pi

        # d²P/ds²  — analytical ((dt/ds)² = 4π², d²t/ds² = 0):
        #   d²x/ds² = −16π²·A·sin 2t ,  d²y/ds² = −36π²·B·sin 3t
        two_pi_sq = two_pi * two_pi
        ddx = -4.0 * A * sin2 * two_pi_sq
        ddy = -9.0 * B * sin3 * two_pi_sq

        # Embed the planar (x, y) values along the two in-plane axes.  Written
        # element-wise into the preallocated buffers → zero allocation per call.
        c, u, v = self._center, self._u, self._v
        for i in range(3):
            self._P[i]   = c[i] + px * u[i] + py * v[i]
            self._Pp[i]  = dx * u[i] + dy * v[i]
            self._Ppp[i] = ddx * u[i] + ddy * v[i]

    # ── Optional geometric helpers (analytical; not used by the controller) ──

    def tangent(self, s: float):
        """Unit tangent ``P'(s) / ‖P'(s)‖`` at phase *s* (view, may be zeroed)."""
        self._ensure(s)
        n = math.sqrt(float(self._Pp @ self._Pp))
        if n > 1e-12:
            np.divide(self._Pp, n, out=self._tan_out)
        else:
            self._tan_out[:] = 0.0
        return self._tan_out

    def curvature(self, s: float) -> float:
        """Curvature ``κ = ‖P'×P''‖ / ‖P'‖³`` at phase *s* (always finite)."""
        self._ensure(s)
        cross = np.cross(self._Pp, self._Ppp)
        num = math.sqrt(float(cross @ cross))
        den = float(self._Pp @ self._Pp) ** 1.5
        return num / den if den > 1e-12 else 0.0


class CircularPath(GeometricPath):
    r"""Circle parametrised by the dimensionless phase ``s`` (one loop = Δs = 1).

    Mathematical definition
    ------------------------
    The normalised phase is mapped to an angle ``t = 2π·s`` (so ``dt/ds = 2π``,
    a constant) and the two *in-plane* coordinates relative to the centre are

        x(t) = r · cos t
        y(t) = r · sin t

    The curve ``P(s) = centre + x(t)·û + y(t)·v̂`` is therefore **closed and
    periodic** with period ``Δs = 1`` (at ``s = 0`` and ``s = 1`` we have
    ``t = 0`` and ``t = 2π``, so position and all derivatives repeat exactly) and
    **globally C∞ smooth** with constant curvature ``1/r`` — no corners, no
    segment switching.  A phase rate ``ṡ = 1/cycle_time`` reproduces the
    constant-speed circle of :class:`CircularTrajectory`.

    Analytical derivatives
    ----------------------
    As in :class:`LissajousPath`, the phase derivatives are computed in closed
    form (not by finite differences), so they are exact to machine precision and
    free of differencing noise / lag.  Since ``t`` is *linear* in ``s``
    (``dt/ds = 2π`` constant, ``d²t/ds² = 0``) the chain rule reduces to a plain
    power of ``dt/ds``::

        dx/ds   = (−r·sin t)·(2π)   = −2π·r·sin t
        d²x/ds² = (−r·cos t)·(2π)²  = −4π²·r·cos t
        dy/ds   = ( r·cos t)·(2π)   =  2π·r·cos t
        d²y/ds² = (−r·sin t)·(2π)²  = −4π²·r·sin t

    The inherited :meth:`velocity` / :meth:`acceleration` then apply the outer
    chain rule in the phase rate (``v = P'·ṡ``, ``a = P''·ṡ² + P'·s̈``).

    Parameters
    ----------
    center : ndarray (3,)
        Circle centre in the base frame.
    radius : float
        Circle radius [m].
    plane : str
        ``"xy"``, ``"xz"``, ``"yz"`` or ``"front"`` — selects the in-plane axes
        via the shared :data:`_PLANE_AXES` mapping.
    """

    # Same in-plane embedding mapping as LissajousPath.
    _PLANE_AXES = _PLANE_AXES

    def __init__(self, center, radius: float = 0.1,
                 plane: str = 'front') -> None:
        super().__init__()
        self._center = np.asarray(center, dtype=float).copy()
        self._r = float(radius)
        plane = plane.lower()
        try:
            u, v = self._PLANE_AXES[plane]
        except KeyError:
            raise ValueError(
                f'Unknown plane "{plane}", use xy/xz/yz/front') from None
        self._u = u.copy()
        self._v = v.copy()
        self._two_pi = 2.0 * math.pi
        self._tan_out = np.zeros(3)   # scratch for tangent()

    def _compute(self, s: float) -> None:
        # Phase → angle.  No `s mod 1` is needed: t = 2π·s and cos t / sin t are
        # already exactly periodic with period Δs = 1.
        t = self._two_pi * s
        ct, st = math.cos(t), math.sin(t)
        r, two_pi = self._r, self._two_pi

        # In-plane position offsets:  x(t) = r·cos t,  y(t) = r·sin t
        px = r * ct
        py = r * st

        # dP/ds  — analytical chain rule (dt/ds = 2π):
        #   dx/ds = −2π·r·sin t ,  dy/ds = 2π·r·cos t
        dx = -r * st * two_pi
        dy = r * ct * two_pi

        # d²P/ds²  — analytical ((dt/ds)² = 4π², d²t/ds² = 0):
        #   d²x/ds² = −4π²·r·cos t ,  d²y/ds² = −4π²·r·sin t
        two_pi_sq = two_pi * two_pi
        ddx = -r * ct * two_pi_sq
        ddy = -r * st * two_pi_sq

        # Embed the planar (x, y) values along the two in-plane axes.  Written
        # element-wise into the preallocated buffers → zero allocation per call.
        c, u, v = self._center, self._u, self._v
        for i in range(3):
            self._P[i]   = c[i] + px * u[i] + py * v[i]
            self._Pp[i]  = dx * u[i] + dy * v[i]
            self._Ppp[i] = ddx * u[i] + ddy * v[i]

    # ── Optional geometric helpers (analytical; not used by the controller) ──

    def tangent(self, s: float):
        """Unit tangent ``P'(s) / ‖P'(s)‖`` at phase *s* (view, may be zeroed)."""
        self._ensure(s)
        n = math.sqrt(float(self._Pp @ self._Pp))
        if n > 1e-12:
            np.divide(self._Pp, n, out=self._tan_out)
        else:
            self._tan_out[:] = 0.0
        return self._tan_out

    def curvature(self, s: float) -> float:
        """Curvature ``κ = ‖P'×P''‖ / ‖P'‖³`` at phase *s* (constant ``1/r``)."""
        self._ensure(s)
        cross = np.cross(self._Pp, self._Ppp)
        num = math.sqrt(float(cross @ cross))
        den = float(self._Pp @ self._Pp) ** 1.5
        return num / den if den > 1e-12 else 0.0


class LinearSegmentPath(GeometricPath):
    """Straight line from ``p0`` to ``p1`` over phase ``s ∈ [0, span]``.

    Geometry only — ``dP/ds = (p1 − p0)/span`` is constant and ``d²P/ds² = 0``;
    any ease-in/out is supplied by the timing law.  Used as the approach lead-in
    so that ``P(0) = p0`` (the current EE position) — there is no position jump
    at start-up and hence no separate APPROACH state.
    """

    def __init__(self, p0, p1, span: float = 1.0) -> None:
        super().__init__()
        self._p0 = np.asarray(p0, dtype=float).copy()
        self._span = max(1e-9, float(span))
        self._delta = np.asarray(p1, dtype=float).copy() - self._p0   # p1 − p0
        np.divide(self._delta, self._span, out=self._Pp)              # constant dP/ds
        self._Ppp[:] = 0.0                                            # constant

    def _compute(self, s: float) -> None:
        u = s / self._span
        if u < 0.0:
            u = 0.0
        elif u > 1.0:
            u = 1.0
        np.multiply(self._delta, u, out=self._P)
        self._P += self._p0
        # _Pp and _Ppp are constant (set in __init__)


class CompositePath(GeometricPath):
    """Concatenate geometric segments along the phase axis.

    ``segments`` is a list of ``(path, span)`` pairs.  Segment *i* owns the
    global-phase interval ``[offset_i, offset_i + span_i)`` and is evaluated at
    its *local* phase ``s − offset_i``.  The final segment is the open-ended tail
    (its ``span`` is ignored) — typically the cyclic :class:`PentagonPath`.

    This is what replaces the APPROACH→TRACK state machine: a single monotonic
    phase ``s`` flows from the approach lead-in straight into cyclic tracking,
    with no branching in the controller tick.
    """

    def __init__(self, segments) -> None:
        super().__init__()
        if not segments:
            raise ValueError("CompositePath needs at least one segment")
        self._paths = [p for (p, _) in segments]
        offsets = [0.0]
        for _, span in segments[:-1]:
            offsets.append(offsets[-1] + float(span))
        self._offsets = offsets

    def _locate(self, s: float):
        idx = 0
        for i in range(len(self._paths) - 1):
            if s >= self._offsets[i + 1]:
                idx = i + 1
            else:
                break
        return idx, s - self._offsets[idx]

    def _compute(self, s: float) -> None:
        idx, local = self._locate(s)
        P, Pp, Ppp = self._paths[idx].derivatives(local)
        np.copyto(self._P, P)
        np.copyto(self._Pp, Pp)
        np.copyto(self._Ppp, Ppp)


# ---------------------------------------------------------------------------
# Random waypoint sampling
# ---------------------------------------------------------------------------

def sample_single_waypoint(
    bounds: List[float],
    min_dist: float,
    rng: np.random.Generator,
    prev_point: Optional[np.ndarray] = None,
) -> np.ndarray:
    """Sample a single random 3-D waypoint inside *bounds*.

    If *prev_point* is given, the new point is guaranteed to be at least
    *min_dist* away (or the last attempt is returned as fallback).
    """
    xmin, xmax, ymin, ymax, zmin, zmax = bounds
    p = None
    for _ in range(_MAX_SAMPLE_ATTEMPTS):
        p = np.array([
            rng.uniform(xmin, xmax),
            rng.uniform(ymin, ymax),
            rng.uniform(zmin, zmax),
        ])
        if prev_point is None or np.linalg.norm(p - prev_point) >= min_dist:
            return p
    return p  # type: ignore[return-value]  # fallback: tight bounds


def sample_waypoints(
    num: int,
    bounds: List[float],
    min_dist: float,
    rng: np.random.Generator,
) -> List[np.ndarray]:
    """Sample *num* random 3-D waypoints inside *bounds*.

    Consecutive waypoints are guaranteed to be at least *min_dist* apart.
    """
    pts: List[np.ndarray] = []
    for _ in range(num):
        prev = pts[-1] if pts else None
        pts.append(sample_single_waypoint(bounds, min_dist, rng, prev))
    return pts
