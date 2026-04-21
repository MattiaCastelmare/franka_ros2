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
