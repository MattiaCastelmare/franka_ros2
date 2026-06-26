"""Distance engine: depth frame → per-control-point obstacle distances.

Depth-space approach (Flacco et al., "A Depth Space Approach to Human-Robot
Collision Avoidance"):
  Each CP is projected into the image plane.  Obstacle pixels are unprojected
  directly in camera frame (no base-frame transform per pixel).  Per-CP
  distances are computed by direct vectorised subtraction — no cKDTree, no
  cdist self-filter (the search_exclusion_mask handles robot-body exclusion
  in 2D, which is sufficient given the dilated robot mask).

Dilation-margin compensation:
  The search_exclusion_mask is dilated in pixel-space (MaskBuilder) to reject
  depth noise and self-occlusion, which shifts the first observable obstacle
  pixel artificially beyond the true robot surface.  When ee_source_mask and
  dilation_margins_px are supplied, compute() converts that pixel margin back to
  metres at the local pixel depth (per-pixel EE vs body margin) and subtracts it
  BEFORE the nearest-pixel selection, so an obstacle touching the dilated border
  reports ≈0 instead of a hidden positive offset.  Omitting either argument
  restores the legacy output bit-for-bit.

Low-pass filter:
  Per-CP EMA across frames.  When a new measurement is CLOSER than the
  smoothed value, the raw measurement is used immediately (conservative /
  safe-by-default).  Smoothing only applies to the recovery direction
  (obstacle moving away), which prevents the CBF from acting on sensor noise
  spikes while never delaying detection of a newly approaching obstacle.

Approach-spike outlier rejection (rate-limit + 1-frame confirmation):
  The "closer → raw immediately" rule above is safe against slow noise but
  blind to a SINGLE-FRAME spike (a depth artefact, or the memoryless argmin
  jumping to a different surface patch) that implies a physically implausible
  approach speed.  Such a spike makes d_min collapse for one frame and drives
  a violent CBF reaction on bad data.  Fix: when a closer measurement implies
  an approach speed above v_max_approach (computed from the REAL inter-frame
  dt), the drop is rate-limited to v_max_approach for that one frame and the
  raw value is held "pending".  On the NEXT frame the persistence decides:
  if the obstacle stayed close (did not recover toward the pre-spike value)
  the jump was real → accept the raw value immediately (1-frame delay only);
  if it recovered to baseline it was an isolated outlier → discard it.  Sub-
  threshold approaches and the moving-away branch are untouched (bit-for-bit).
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

        # Approach-spike outlier rejection (rate-limit + 1-frame confirmation).
        # v_max_approach: implied approach speed [m/s] above which a closer
        #   measurement is treated as a suspicious jump (rate-limited, then
        #   confirmed/discarded on the next frame) — NOT rejected outright.
        # outlier_recover_{frac,abs}: on the confirmation frame, a pending CP is
        #   declared an outlier only if d_raw recovered to within
        #   max(frac*d_pre, abs) of the pre-spike value; everything else (still
        #   close, or even closer) is confirmed and accepted (safe-by-default).
        self._v_max_approach       = float(distance_cfg.get('lpf_v_max_approach', 2.0))
        self._outlier_recover_frac = float(distance_cfg.get('lpf_outlier_recover_frac', 0.10))
        self._outlier_recover_abs  = float(distance_cfg.get('lpf_outlier_recover_abs', 0.03))
        # Spatial-continuity gate (option c): when a suspicious approach jump ALSO
        # teleports the winning surface point by more than this in 3D [m], the
        # argmin likely switched surface REGION (a static-obstacle artefact, not a
        # real fast approach), so demand ONE EXTRA confirmation frame (2 instead of
        # 1) before accepting it. Default 1.0 m = deliberately permissive / nearly
        # disabled: with no real d3d distribution yet, this MUST NOT change today's
        # behaviour (the analysis estimates the eventual threshold at ~0.1-0.2 m,
        # to be confirmed from the logged d3d before tightening). The reported
        # distance is NEVER altered by this gate — only the acceptance latency.
        self._d3d_gate             = float(distance_cfg.get('lpf_d3d_gate_m', 1.0))
        # Sync-jump gate (complementary to the per-CP d3d gate): when a large
        # FRACTION of the control points with a valid measurement this frame ALL
        # trip the approach rate-limit simultaneously, treat it as a likely
        # SYSTEMIC artefact (TF jitter, robot-mask instability, timestamp
        # mismatch) that shifts the whole observed surface — not independent
        # per-pixel noise — and demand the same extra confirmation frame the d3d
        # gate uses. The d3d gate is per-CP and blind to this cross-CP
        # correlation. frac chosen conservatively high so it fires only on broad,
        # correlated jumps; min_cps avoids a meaningless ratio on a tiny sample.
        self._sync_jump_frac    = float(distance_cfg.get('lpf_sync_jump_frac', 0.5))
        self._sync_jump_min_cps = int(distance_cfg.get('lpf_sync_jump_min_cps', 3))

        self._grid_roi: Optional[tuple] = None
        self._grid_ug:  Optional[np.ndarray] = None
        self._grid_vg:  Optional[np.ndarray] = None

        # LPF state: (seg_idx, cp_idx) → (d_smooth, dir_smooth, pt_acc, px_acc),
        # where pt_acc/px_acc are the last ACCEPTED winning obstacle point (base
        # frame) / pixel — kept frozen through a pending window so d3d measures the
        # teleport vs the last STABLE winner. Used only for the d3d gate + RTDDIAG
        # logging; the reported distance/direction never read from here.
        self._lpf: Dict[Tuple[int, int],
                        Tuple[float, Optional[np.ndarray],
                              Optional[np.ndarray], Optional[Tuple[int, int]]]] = {}
        # Pending suspicious-jump state: (seg_idx, cp_idx) → (d_pre, d_susp,
        # frames_left). Presence of a key ⇒ awaiting confirmation; frames_left =
        # confirmation frames still required (1 normally, 2 when the d3d gate fires).
        self._pending: Dict[Tuple[int, int], Tuple[float, float, int]] = {}
        # Timestamp [s] of the previous processed frame, for the REAL dt.
        self._prev_stamp: Optional[float] = None

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
        ee_source_mask: Optional[np.ndarray] = None,            # bool (H, W)
        dilation_margins_px: Optional[Tuple[int, int]] = None,  # (margin_body_px, margin_ee_px)
        frame_stamp: Optional[float] = None,                    # frame time [s] for REAL dt
    ) -> Tuple[Optional[List[ControlPointResult]], int]:
        """Depth-space CP distance pipeline with LPF smoothing.

        Returns (results, n_obstacle_pixels) or (None, 0).

        frame_stamp (seconds) feeds the approach-spike rate-limit: the engine
        derives the REAL inter-frame dt from it.  When omitted (None), the
        rate-limit is disabled and the output is bit-for-bit the legacy LPF.
        """
        if depth is None or not control_points:
            return None, 0

        # REAL inter-frame dt from consecutive stamps (jitter-aware).  Updated
        # here so BOTH _lpf_pass call sites below see a consistent dt, and the
        # very first frame (or a missing stamp) yields dt=None → legacy path.
        dt: Optional[float] = None
        if frame_stamp is not None and self._prev_stamp is not None:
            dt = frame_stamp - self._prev_stamp
        self._prev_stamp = frame_stamp

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

        # Sample the EE-vs-body provenance with the SAME (vg, ug) indices used
        # for Z below, so it stays index-aligned with p_cam through Step 4.
        ee_src = ee_source_mask[vg, ug] if ee_source_mask is not None else None

        # ── Step 3: depth filter ──────────────────────────────────────────
        Z     = depth[vg, ug].astype(np.float32) * np.float32(0.001)
        valid = (Z >= self.min_depth) & (Z <= self.max_depth)
        ug, vg, Z = ug[valid], vg[valid], Z[valid]
        if ee_src is not None:
            ee_src = ee_src[valid]   # same boolean filter as Z/ug/vg

        if ug.size == 0:
            return self._lpf_pass(self._empty_results(control_points), dt), 0

        # ── Step 4: unproject obstacle pixels to CAMERA frame ─────────────
        # Working in camera frame avoids transforming every pixel to base frame.
        p_cam = np.empty((ug.size, 3), dtype=np.float32)
        p_cam[:, 0] = (ug.astype(np.float32) - cx_f32) * (Z * fx_inv_f32)
        p_cam[:, 1] = (vg.astype(np.float32) - cy_f32) * (Z * fy_inv_f32)
        p_cam[:, 2] = Z

        # ── Step 4b: dilation-margin → metres, per obstacle pixel ─────────
        # Convert the pixel-space exclusion-mask dilation back to metres at the
        # LOCAL depth of each obstacle pixel so Step 6 can subtract it.  ee_src
        # picks the EE margin (hand region) vs the body margin per pixel.
        # Gated on BOTH inputs present — if either is None the correction is
        # skipped entirely and the output is identical to the legacy path.
        if ee_src is not None and dilation_margins_px is not None:
            # margin_px: (N_pix,) — margin_ee_px where ee_src True, else margin_body_px
            margin_px = np.where(ee_src, dilation_margins_px[1], dilation_margins_px[0])
            # SIMPLIFICATION: use fx_inv only (not the fx/fy average).  fx≈fy for the
            # RealSense cameras used in this project, so the metric error is
            # negligible; revisit if a camera with anisotropic focals is adopted.
            margin_m = margin_px.astype(np.float32) * Z * fx_inv_f32
        else:
            margin_m = np.float32(0.0)   # scalar → natural broadcast, no-op

        # ── Step 5: project CPs into camera frame ─────────────────────────
        # R_base maps base→camera (rotation part of extrinsic).
        # cp_cam[i] = R_base.T @ (cp_base[i] − t_base)
        R_c2b  = R_base_f32.T                                            # (3, 3)
        cp_cam = (R_c2b @ (cp_positions.T - t_base_f32[:, None])).T     # (N_CP, 3)

        # ── Step 6: per-CP nearest obstacle ───────────────────────────────
        # Direct vectorised subtraction per CP — no KDTree overhead.
        # margin_m is either a scalar 0.0 (correction disabled) or a (N_pix,)
        # array index-aligned with p_cam/dist_3d (same Step-3 `valid` filter),
        # so the subtraction below broadcasts correctly with no shape mismatch.
        # Subtracting the margin BEFORE argmin lets it change which pixel wins,
        # and np.maximum(..., 0.0) stays the LAST op so surface never goes < 0.
        results: List[ControlPointResult] = []
        for i, cp in enumerate(control_points):
            diff    = p_cam - cp_cam[i]                          # (N_pix, 3)
            dist_3d = np.sqrt((diff * diff).sum(axis=1))         # (N_pix,)
            surface = np.maximum(dist_3d - radii[i] - margin_m, np.float32(0.0))

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
        return self._lpf_pass(results, dt), int(ug.size)

    # ── Low-pass filter ───────────────────────────────────────────────────

    def _lpf_pass(
        self,
        results: List[ControlPointResult],
        dt: Optional[float] = None,
    ) -> List[ControlPointResult]:
        """Conservative EMA: instant response to closer obstacles, smooth recovery.

        Adds approach-spike rejection: a closer measurement whose implied speed
        exceeds v_max_approach is rate-limited for one frame and held pending,
        then confirmed or discarded on the next frame (see module docstring).
        dt is the REAL inter-frame interval [s]; when None/≤0 the rate-limit is
        skipped and every branch is bit-for-bit the legacy behaviour.
        """
        alpha = self._lpf_alpha
        if alpha <= 0.0:
            return results

        dt_valid = dt is not None and dt > 1e-4   # guard div-by-0 / dup stamps

        # ── Pre-pass: detect a SYNCHRONOUS approach jump across CPs ───────────
        # Read-only count (touches neither self._lpf nor self._pending): how many
        # CPs with a valid measurement AND a prior state would trip the approach
        # rate-limit (v_impl > v_max_approach) THIS frame. If a large fraction do
        # it together, it is almost certainly a systemic artefact → flag the whole
        # frame so the main loop demands an extra confirmation frame on those CPs
        # (same mechanism as the d3d gate; the reported distance is unchanged).
        sync_jump_detected = False
        if dt_valid:
            n_valid = n_suspect = 0
            for r in results:
                if not np.isfinite(r.distance):
                    continue
                prev = self._lpf.get((r.seg_idx, r.cp_idx))
                if prev is None:
                    continue
                n_valid += 1
                d_prev = prev[0]
                if r.distance < d_prev and \
                        (d_prev - r.distance) / dt > self._v_max_approach:
                    n_suspect += 1
            sync_jump_detected = (
                n_valid >= self._sync_jump_min_cps
                and n_suspect / n_valid >= self._sync_jump_frac)

        smoothed: List[ControlPointResult] = []
        for r in results:
            key  = (r.seg_idx, r.cp_idx)
            prev = self._lpf.get(key)

            if prev is None:
                if np.isfinite(r.distance):
                    self._lpf[key] = (r.distance, r.direction,
                                      r.closest_obstacle_point, r.closest_pixel)
                smoothed.append(r)
                continue

            d_prev, dir_prev, pt_prev, px_prev = prev

            if not np.isfinite(r.distance):
                # No valid measurement: hold last smoothed value (pending left
                # intact — it resolves when a finite measurement next arrives).
                smoothed.append(ControlPointResult(
                    point=r.point, seg_idx=r.seg_idx, cp_idx=r.cp_idx,
                    radius=r.radius, start_link=r.start_link, end_link=r.end_link,
                    distance=d_prev, direction=dir_prev,
                    closest_obstacle_point=None, closest_pixel=None,
                ))
                continue

            # Direction: always EMA-smoothed to avoid flip discontinuities
            dir_new = self._smooth_direction(dir_prev, r.direction, alpha)

            pend = self._pending.get(key)
            if pend is not None:
                # ── Resolve a pending suspicious jump (multi-frame confirm) ──
                # d3d = teleport of the current winning point vs the last STABLE
                # accepted point (pt_prev stays frozen through the whole pending
                # window → true teleport magnitude at every frame).
                d_pre, d_susp, frames_left = pend
                d3d = self._pt_dist(r.closest_obstacle_point, pt_prev)
                recover_tol = max(self._outlier_recover_frac * d_pre,
                                  self._outlier_recover_abs)
                if r.distance >= d_pre - recover_tol:
                    # Recovered to baseline at ANY confirm frame → isolated
                    # OUTLIER: discard now, keep the old stable winning point.
                    del self._pending[key]
                    if r.distance < d_pre:
                        d_new = r.distance
                    else:
                        d_new = alpha * d_pre + (1.0 - alpha) * r.distance
                    self._log_jump('outlier-discarded', r, d_pre, r.distance,
                                   dt, d_new, d3d=d3d, sync=sync_jump_detected)
                    pt_acc, px_acc = pt_prev, px_prev
                else:
                    frames_left -= 1
                    if frames_left <= 0:
                        # Stayed close through ALL required frames → CONFIRMED:
                        # accept raw and adopt the current winner as the new stable.
                        del self._pending[key]
                        d_new = r.distance
                        self._log_jump('confirmed', r, d_pre, r.distance,
                                       dt, d_new, d3d=d3d, sync=sync_jump_detected)
                        pt_acc, px_acc = r.closest_obstacle_point, r.closest_pixel
                    else:
                        # Still pending one more frame: KEEP the conservative
                        # rate-limit (never the optimistic d_pre, never a freeze —
                        # it keeps approaching r.distance at v_max each frame) and
                        # keep the old stable winning point.
                        d_new = (max(d_prev - self._v_max_approach * dt, r.distance)
                                 if dt_valid else r.distance)
                        self._pending[key] = (d_pre, r.distance, frames_left)
                        self._log_jump('pending', r, d_prev, r.distance,
                                       dt, d_new, d3d=d3d, sync=sync_jump_detected)
                        pt_acc, px_acc = pt_prev, px_prev
                self._lpf[key] = (d_new, dir_new, pt_acc, px_acc)
                smoothed.append(self._mk_result(r, d_new, dir_new))
                continue

            # ── Normal path ───────────────────────────────────────────────────
            if r.distance < d_prev:
                # Approach branch: gate on implied speed to catch single-frame
                # spikes.  Below threshold (or no dt) → raw immediately, exactly
                # as before.  Above threshold → rate-limit + start pending.
                v_impl = (d_prev - r.distance) / dt if dt_valid else 0.0
                if dt_valid and v_impl > self._v_max_approach:
                    # d3d = teleport of the winning point vs the last accepted
                    # point. A large teleport on a suspicious jump ⇒ the argmin
                    # likely switched surface REGION (not a real fast approach) ⇒
                    # demand an EXTRA confirmation frame. Small/zero teleport ⇒
                    # frames_needed=1 = today's behaviour exactly (no regression).
                    d3d = self._pt_dist(r.closest_obstacle_point, pt_prev)
                    # Extra confirmation frame if EITHER the per-CP teleport gate
                    # (d3d) OR the cross-CP synchronous-jump gate fires.
                    frames_needed = 2 if (d3d > self._d3d_gate
                                          or sync_jump_detected) else 1
                    d_new = max(d_prev - self._v_max_approach * dt, r.distance)
                    self._pending[key] = (d_prev, r.distance, frames_needed)
                    self._log_jump('rate-limited', r, d_prev, r.distance, dt,
                                   d_new, v_impl=v_impl, d3d=d3d,
                                   sync=sync_jump_detected)
                    # Jump not accepted yet → KEEP the old stable winning point.
                    self._lpf[key] = (d_new, dir_new, pt_prev, px_prev)
                    smoothed.append(self._mk_result(r, d_new, dir_new))
                    continue
                d_new = r.distance
            else:
                # Obstacle moving away → smooth to avoid acting on sensor noise
                d_new = alpha * d_prev + (1.0 - alpha) * r.distance

            # Accepted a (non-suspicious) value → adopt the current winner point.
            self._lpf[key] = (d_new, dir_new,
                              r.closest_obstacle_point, r.closest_pixel)
            smoothed.append(self._mk_result(r, d_new, dir_new))

        return smoothed

    @staticmethod
    def _pt_dist(p_a, p_b) -> float:
        """3D teleport magnitude [m] between two closest-obstacle points.

        Returns 0.0 if either point is missing (no finite winner) → the d3d gate
        defaults to today's single-frame behaviour (safe: no extra confirmation
        demanded when we can't measure the teleport).
        """
        if p_a is None or p_b is None:
            return 0.0
        return float(np.linalg.norm(np.asarray(p_a) - np.asarray(p_b)))

    @staticmethod
    def _smooth_direction(dir_prev, dir_raw, alpha):
        """EMA-smooth + renormalise a unit direction (unchanged legacy math)."""
        if dir_prev is not None and dir_raw is not None:
            d = alpha * dir_prev + (1.0 - alpha) * dir_raw
            n = float(np.linalg.norm(d))
            return (d / n).astype(np.float32) if n > 1e-9 else dir_raw
        return dir_raw

    @staticmethod
    def _mk_result(r: ControlPointResult, distance: float, direction) -> ControlPointResult:
        return ControlPointResult(
            point=r.point, seg_idx=r.seg_idx, cp_idx=r.cp_idx,
            radius=r.radius, start_link=r.start_link, end_link=r.end_link,
            distance=distance, direction=direction,
            closest_obstacle_point=r.closest_obstacle_point,
            closest_pixel=r.closest_pixel,
        )

    def _log_jump(self, outcome, r, d_prev, d_raw, dt, d_new, v_impl=None,
                  d3d=None, sync=None):
        """Diagnostic for suspicious-approach events (RTDDIAG, analog to CBFDIAG).

        Fires only on the rate-limit / pending / confirm / discard branches
        (rare), so it is intentionally NOT throttled — every event is a data
        point for validating the fix on hardware. d3d (winning-point 3D teleport
        [m]) is logged on EVERY jump event so the real d3d distribution for
        outlier-vs-confirmed can be collected before tightening lpf_d3d_gate_m.
        """
        if self._log is None:
            return
        v_txt = f'{v_impl:.2f}' if v_impl is not None else '   —'
        d3d_txt = f'{d3d:.3f}' if d3d is not None else '  —'
        sync_txt = f'{sync}' if sync is not None else '—'
        dt_ms = f'{dt * 1e3:.1f}' if dt is not None else '—'
        self._log.warning(
            f'RTDDIAG approach-jump [{outcome}] CP=({r.seg_idx},{r.cp_idx}) '
            f'v_impl={v_txt} m/s dt={dt_ms} ms d3d={d3d_txt} m sync={sync_txt} '
            f'd_prev={d_prev:.3f} d_raw={d_raw:.3f} -> d_out={d_new:.3f} m')

    def invalidate_grid_cache(self):
        """Force pixel-grid rebuild on the next compute() call."""
        self._grid_roi = None

    def reset_lpf(self):
        """Clear all LPF state (call on camera resolution change)."""
        self._lpf.clear()
        self._pending.clear()
        self._prev_stamp = None

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
