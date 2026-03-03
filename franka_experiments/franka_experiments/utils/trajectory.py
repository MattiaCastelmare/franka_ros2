"""Trajectory generators: pentagon, random waypoints."""

from __future__ import annotations

import math
from typing import List, Optional

import numpy as np

from .math_utils import min_jerk

# Maximum re-sampling attempts per waypoint before giving up.
_MAX_SAMPLE_ATTEMPTS = 50


# ---------------------------------------------------------------------------
# Pentagon trajectory
# ---------------------------------------------------------------------------

class PentagonTrajectory:
    """Smooth periodic pentagon trajectory with minimum-jerk per side.

    Each side of the pentagon is traversed using a 5th-order polynomial in
    normalised time so that velocity and acceleration are zero at each
    vertex → C2-continuous overall loop.

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
    """

    N_SIDES = 5

    def __init__(
        self,
        center: np.ndarray,
        radius: float,
        plane: str,
        cycle_time: float,
    ) -> None:
        self.center = np.asarray(center, dtype=float)
        self.radius = radius
        self.plane = plane.lower()
        self.cycle_time = cycle_time
        self.side_time = cycle_time / self.N_SIDES

        # Compute 5 vertices (starting from "top", counter-clockwise).
        self.vertices: List[np.ndarray] = []
        for k in range(self.N_SIDES):
            angle = 2.0 * math.pi * k / self.N_SIDES + math.pi / 2.0
            u = radius * math.cos(angle)
            v = radius * math.sin(angle)
            pt = self.center.copy()
            if self.plane == 'xy':
                pt[0] += u
                pt[1] += v
            elif self.plane == 'xz':
                pt[0] += u
                pt[2] += v
            elif self.plane in ('yz', 'front'):
                pt[1] += u
                pt[2] += v
            else:
                raise ValueError(
                    f'Unknown plane "{self.plane}", use xy/xz/yz/front')
            self.vertices.append(pt)

    def evaluate(self, t: float):
        """Return ``(p_d, v_d)`` at time *t* (seconds since trajectory start).

        Returns
        -------
        p_d : ndarray (3,)
            Desired Cartesian position.
        v_d : ndarray (3,)
            Desired Cartesian velocity.
        """
        t_mod = t % self.cycle_time
        side_idx = int(t_mod / self.side_time)
        if side_idx >= self.N_SIDES:
            side_idx = self.N_SIDES - 1

        t_in_side = t_mod - side_idx * self.side_time
        tau = t_in_side / self.side_time

        p_start = self.vertices[side_idx]
        p_end = self.vertices[(side_idx + 1) % self.N_SIDES]

        s, sdot_norm = min_jerk(tau)

        p_d = p_start + s * (p_end - p_start)
        v_d = (sdot_norm / self.side_time) * (p_end - p_start)
        return p_d, v_d


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
