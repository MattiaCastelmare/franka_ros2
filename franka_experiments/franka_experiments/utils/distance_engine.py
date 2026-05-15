"""Distance engine: depth frame → per-control-point obstacle distances.

Depth-space approach (Flacco et al., "A Depth Space Approach to Human-Robot
Collision Avoidance"):
  Each CP is projected into the image plane.  Obstacle pixels are unprojected
  directly in camera frame (no base-frame transform per pixel).  Per-CP
  distances are computed by direct vectorised subtraction — no cKDTree, no
  cdist self-filter (the search_exclusion_mask handles robot-body exclusion
  in 2D, which is sufficient given the dilated robot mask).

Low-pass filter:
  Per-CP EMA across frames.  When a new measurement is CLOSER than the
  smoothed value, the raw measurement is used immediately (conservative /
  safe-by-default).  Smoothing only applies to the recovery direction
  (obstacle moving away), which prevents the CBF from acting on sensor noise
  spikes while never delaying detection of a newly approaching obstacle.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import numpy as np


@dataclass
class ControlPointResult:
    """Immutable result of distance computation for one control point."""
    point:    np.ndarray
    seg_idx:  int
    cp_idx:   int
    radius:   float
    start_link: str
    end_link:   str
    distance: float = float('inf')
    direction: Optional[np.ndarray] = None
    closest_obstacle_point: Optional[np.ndarray] = None
    closest_pixel: Optional[Tuple[int, int]] = None


class DistanceEngine:
    """Depth-space per-CP distance engine with conservative LPF output."""

    def __init__(self, distance_cfg: dict, logger=None):
        self._cfg = distance_cfg
        self._log = logger

        self.min_depth  = float(distance_cfg['min_depth_m'])
        self.max_depth  = float(distance_cfg['max_depth_m'])
        self._lpf_alpha = float(distance_cfg.get('lpf_alpha', 0.5))

        self._grid_roi: Optional[tuple] = None
        self._grid_ug:  Optional[np.ndarray] = None
        self._grid_vg:  Optional[np.ndarray] = None

        # LPF state: (seg_idx, cp_idx) → (d_smooth, dir_smooth)
        self._lpf: Dict[Tuple[int, int], Tuple[float, Optional[np.ndarray]]] = {}

    def compute(
        self,
        depth: np.ndarray,
        cx_f32: np.float32,
        cy_f32: np.float32,
        fx_inv_f32: np.float32,
        fy_inv_f32: np.float32,
        R_base_f32: np.ndarray,
        t_base_f32: np.ndarray,
        control_points: List[dict],
        x: np.ndarray,
        y: np.ndarray,
        step: int,
        search_exclusion_mask: Optional[np.ndarray],
        transforms: Optional[dict] = None,  # kept for API compatibility
    ) -> Tuple[Optional[List[ControlPointResult]], int]:
        """Depth-space CP distance pipeline with LPF smoothing.

        Returns (results, n_obstacle_pixels) or (None, 0).
        """
        if depth is None or not control_points:
            return None, 0

        cp_positions = np.array([cp['point']  for cp in control_points], dtype=np.float32)
        radii        = np.array([cp['radius'] for cp in control_points], dtype=np.float32)

        # ── Step 1: pixel grid (cached by ROI key) ────────────────────────
        roi_key = (int(x[0]), int(x[1]), int(y[0]), int(y[1]), int(step))
        if self._grid_roi != roi_key:
            _vg, _ug = np.mgrid[roi_key[2]:roi_key[3]:roi_key[4],
                                 roi_key[0]:roi_key[1]:roi_key[4]]
            self._grid_ug  = _ug.ravel().astype(np.int32)
            self._grid_vg  = _vg.ravel().astype(np.int32)
            self._grid_roi = roi_key
        ug = self._grid_ug
        vg = self._grid_vg

        # ── Step 2: search exclusion mask ────────────────────────────────
        if search_exclusion_mask is not None:
            keep   = ~search_exclusion_mask[vg, ug]
            ug, vg = ug[keep], vg[keep]

        # ── Step 3: depth filter ──────────────────────────────────────────
        Z     = depth[vg, ug].astype(np.float32) * np.float32(0.001)
        valid = (Z >= self.min_depth) & (Z <= self.max_depth)
        ug, vg, Z = ug[valid], vg[valid], Z[valid]

        if ug.size == 0:
            return self._lpf_pass(self._empty_results(control_points)), 0

        # ── Step 4: unproject obstacle pixels to CAMERA frame ─────────────
        # Working in camera frame avoids transforming every pixel to base frame.
        p_cam = np.empty((ug.size, 3), dtype=np.float32)
        p_cam[:, 0] = (ug.astype(np.float32) - cx_f32) * (Z * fx_inv_f32)
        p_cam[:, 1] = (vg.astype(np.float32) - cy_f32) * (Z * fy_inv_f32)
        p_cam[:, 2] = Z

        # ── Step 5: project CPs into camera frame ─────────────────────────
        # R_base maps base→camera (rotation part of extrinsic).
        # cp_cam[i] = R_base.T @ (cp_base[i] − t_base)
        R_c2b  = R_base_f32.T                                            # (3, 3)
        cp_cam = (R_c2b @ (cp_positions.T - t_base_f32[:, None])).T     # (N_CP, 3)

        # ── Step 6: per-CP nearest obstacle ───────────────────────────────
        # Direct vectorised subtraction per CP — no KDTree overhead.
        results: List[ControlPointResult] = []
        for i, cp in enumerate(control_points):
            diff    = p_cam - cp_cam[i]                          # (N_pix, 3)
            dist_3d = np.sqrt((diff * diff).sum(axis=1))         # (N_pix,)
            surface = np.maximum(dist_3d - radii[i], np.float32(0.0))

            best     = int(surface.argmin())
            min_dist = float(surface[best])

            if np.isfinite(min_dist):
                obs_pt_cam  = p_cam[best]
                obs_pt_base = R_base_f32 @ obs_pt_cam + t_base_f32
                vec  = cp['point'].astype(np.float64) - obs_pt_base.astype(np.float64)
                norm = float(np.linalg.norm(vec))
                direction = (vec / norm).astype(np.float32) if norm > 1e-9 \
                            else np.zeros(3, np.float32)
                obs_pix = (int(ug[best]), int(vg[best]))
            else:
                obs_pt_base = None
                direction   = None
                obs_pix     = None

            results.append(ControlPointResult(
                point=cp['point'],
                seg_idx=cp['seg_idx'],
                cp_idx=cp['cp_idx'],
                radius=cp['radius'],
                start_link=cp['start_link'],
                end_link=cp['end_link'],
                distance=min_dist,
                direction=direction,
                closest_obstacle_point=obs_pt_base,
                closest_pixel=obs_pix,
            ))

        # ── Step 7: apply conservative low-pass filter ────────────────────
        return self._lpf_pass(results), int(ug.size)

    # ── Low-pass filter ───────────────────────────────────────────────────

    def _lpf_pass(self, results: List[ControlPointResult]) -> List[ControlPointResult]:
        """Conservative EMA: instant response to closer obstacles, smooth recovery."""
        alpha = self._lpf_alpha
        if alpha <= 0.0:
            return results

        smoothed: List[ControlPointResult] = []
        for r in results:
            key  = (r.seg_idx, r.cp_idx)
            prev = self._lpf.get(key)

            if prev is None:
                if np.isfinite(r.distance):
                    self._lpf[key] = (r.distance, r.direction)
                smoothed.append(r)
                continue

            d_prev, dir_prev = prev

            if not np.isfinite(r.distance):
                # No valid measurement: hold last smoothed value
                smoothed.append(ControlPointResult(
                    point=r.point, seg_idx=r.seg_idx, cp_idx=r.cp_idx,
                    radius=r.radius, start_link=r.start_link, end_link=r.end_link,
                    distance=d_prev, direction=dir_prev,
                    closest_obstacle_point=None, closest_pixel=None,
                ))
                continue

            # Closer obstacle → use raw immediately (safe-by-default)
            # Obstacle moving away → smooth to avoid acting on sensor noise
            if r.distance < d_prev:
                d_new = r.distance
            else:
                d_new = alpha * d_prev + (1.0 - alpha) * r.distance

            # Direction: always EMA-smoothed to avoid flip discontinuities
            if dir_prev is not None and r.direction is not None:
                dir_raw = alpha * dir_prev + (1.0 - alpha) * r.direction
                n = float(np.linalg.norm(dir_raw))
                dir_new = (dir_raw / n).astype(np.float32) if n > 1e-9 else r.direction
            else:
                dir_new = r.direction

            self._lpf[key] = (d_new, dir_new)
            smoothed.append(ControlPointResult(
                point=r.point, seg_idx=r.seg_idx, cp_idx=r.cp_idx,
                radius=r.radius, start_link=r.start_link, end_link=r.end_link,
                distance=d_new, direction=dir_new,
                closest_obstacle_point=r.closest_obstacle_point,
                closest_pixel=r.closest_pixel,
            ))

        return smoothed

    def invalidate_grid_cache(self):
        """Force pixel-grid rebuild on the next compute() call."""
        self._grid_roi = None

    def reset_lpf(self):
        """Clear all LPF state (call on camera resolution change)."""
        self._lpf.clear()

    @staticmethod
    def _empty_results(control_points: List[dict]) -> List[ControlPointResult]:
        return [ControlPointResult(
            point=cp['point'],
            seg_idx=cp['seg_idx'],
            cp_idx=cp['cp_idx'],
            radius=cp['radius'],
            start_link=cp['start_link'],
            end_link=cp['end_link'],
        ) for cp in control_points]
