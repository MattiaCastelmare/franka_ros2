"""Robot mask construction — rebuilt unconditionally every frame.

All dilation operations run on a downsampled image (factor ``mask_downsample``
from config, default 4) and are upsampled with INTER_NEAREST before export.
This reduces mask rebuild time from ~70-80 ms to < 2 ms at 4× downsampling.

Pre-allocated zero buffers for the downsampled masks eliminate per-frame numpy
allocation overhead. Buffers are reallocated only on depth-resolution change.

Outputs (all at full depth-image resolution):
  .robot_mask            — bool array (H, W)
  .search_exclusion_mask — bool array (H, W), dilated beyond robot_mask
  .contours              — list from cv2.findContours (pre-scaled to full res)
  .ee_source_mask        — bool array (H, W), True where the EE dilation dominates
                           the combined exclusion mask (vs. the body dilation)
  .dilation_margins_px   — (margin_body_px, margin_ee_px), effective full-res
                           dilation margins for downstream metric compensation
  .robot_depth           — float32 (H, W), the robot's own expected depth per
                           pixel (+inf where the robot is not there)

THE DEPTH BUFFER, AND WHY THE 2D MASK ALONE IS NOT ENOUGH
---------------------------------------------------------
The exclusion mask is a SILHOUETTE: a pixel inside it is discarded whatever its
depth. That makes the mask a compromise with two bad ends, and this cell has
been bitten by both.

* Dilate MORE and the blind halo grows. At the shipped radii it is 12 px around
  the body and 24 px around the end effector at full resolution — about 4 cm
  and 8 cm at 1.5 m. **A hand inside that halo is not seen at all, at ANY
  depth**, which is precisely the approach direction the camera is supposed to
  cover: a hand a metre in front of the arm is as invisible as one touching it.
* Dilate LESS and robot pixels leak past the edge and read as an obstacle at a
  gap of ~0. Measured: 23 samples at 0.9-4.7 cm on one control point
  (fr3_link6#0), 17 barrier rows violated at once and QP slack of 58, on a run
  where nothing was near the arm.

Both disappear once the mask knows HOW FAR AWAY the robot is at each pixel,
because the two cases differ in exactly that: a leaking robot pixel sits AT the
robot's depth, a hand in front sits well BEFORE it. The depth is already
computed here — ``p_cam[:, 2]`` for every projected mesh sample — and was being
thrown away when the sample was rasterised into a binary mask. Keeping the
minimum per pixel costs one ufunc and turns the silhouette into a z-buffer.

Holes matter: the mesh samples are sparse, so most pixels inside the silhouette
receive no sample. They are filled by a MIN-FILTER over the same structuring
element the mask is dilated with — dilating a mask and eroding a depth buffer
are the same operation — so a pixel without a sample inherits the nearest
robot surface in front of it. Raise ``meshes.sample_points_per_link`` if the
buffer is still sparse; that cost is paid once, at startup.
"""
from __future__ import annotations

from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np


class MaskBuilder:
    """Builds and caches robot mask, exclusion mask, and contours."""

    def __init__(
        self,
        link_mesh_samples: Dict[str, np.ndarray],
        R_base: np.ndarray,
        t_base: np.ndarray,
        ee_link: str,
        mask_cfg: dict,
        skip_max: int = 4,
        link_thresh_m: float = 0.01,
        logger=None,
    ):
        #: Finite stand-in for +inf while the depth buffer is min-filtered:
        #: cv2 will not erode an array containing inf. Far beyond any depth the
        #: camera reports (distance.max_depth_m is 4 m), so it can never be
        #: mistaken for a real surface, and it is restored to inf afterwards.
        self._depth_far_sentinel = 1.0e6
        #: (H, W) float32, the robot's expected depth per pixel; +inf where the
        #: robot is not. Empty until the first rebuild().
        self.robot_depth = None
        self._zbuf = None
        self._samples     = link_mesh_samples
        self._R_base      = R_base
        self._t_base      = t_base
        self._ee_link     = ee_link
        self._cfg         = mask_cfg
        self._skip_max    = skip_max
        self._link_thresh = link_thresh_m
        self._log         = logger
        self._ds          = max(1, int(mask_cfg.get('mask_downsample', 4)))

        self._K: Optional[np.ndarray] = None

        self._frame_skip   = 0
        self._last_t_vecs: Dict[str, np.ndarray] = {}

        # Pre-allocated downsampled mask buffers (re-used across rebuilds).
        self._buf_ds_shape: Optional[Tuple[int, int]] = None
        self._mask_normal_buf: Optional[np.ndarray] = None
        self._mask_ee_buf: Optional[np.ndarray] = None

        self.robot_mask: Optional[np.ndarray] = None
        self.search_exclusion_mask: Optional[np.ndarray] = None
        self.contours: Optional[list] = None
        self.ee_source_mask: Optional[np.ndarray] = None
        self.dilation_margins_px: Tuple[int, int] = (0, 0)

    def set_intrinsics(self, K: np.ndarray):
        self._K = K

    def needs_rebuild(self, transforms: dict) -> bool:
        self._frame_skip += 1
        if self._frame_skip >= self._skip_max:
            self._frame_skip = 0
            return True
        for name, (_, t_new) in transforms.items():
            t_old = self._last_t_vecs.get(name)
            if t_old is None or np.linalg.norm(t_new - t_old) > self._link_thresh:
                self._frame_skip = 0
                return True
        return False

    def rebuild(self, transforms: dict, depth_shape: Tuple[int, int]):
        """Recompute all mask outputs on a downsampled image, then upsample."""
        if self._K is None:
            return

        H, W = depth_shape
        ds   = self._ds
        Hds  = max(1, H // ds)
        Wds  = max(1, W // ds)

        # Scale dilation radii to downsampled space (min 1 to preserve connectivity)
        dilate_px    = max(1, int(self._cfg['robot_mask_dilate_px'])     // ds)
        ee_dilate_px = max(1, int(self._cfg['ee_mask_dilate_px'])         // ds)
        extra_px     =        int(self._cfg['search_exclusion_extra_px']) // ds

        ds_shape = (Hds, Wds)
        if self._buf_ds_shape != ds_shape:
            self._buf_ds_shape    = ds_shape
            self._mask_normal_buf = np.zeros(ds_shape, dtype=np.uint8)
            self._mask_ee_buf     = np.zeros(ds_shape, dtype=np.uint8)
            self._zbuf            = np.empty(ds_shape, dtype=np.float32)
        else:
            self._mask_normal_buf[:] = 0
            self._mask_ee_buf[:]     = 0
        mask_normal = self._mask_normal_buf
        mask_ee     = self._mask_ee_buf
        # +inf = "the robot is not at this pixel". np.minimum.at then writes the
        # nearest projected sample, which is what a z-buffer is.
        zbuf = self._zbuf
        zbuf[:] = np.inf

        for link_name, pts_local in self._samples.items():
            if link_name not in transforms:
                continue
            R, t = transforms[link_name]
            pts_base = (R @ pts_local.T).T + t

            p_cam    = (self._R_base.T @ (pts_base - self._t_base).T).T
            in_front = p_cam[:, 2] > 0
            p_cam    = p_cam[in_front]
            if p_cam.shape[0] == 0:
                continue

            uv = (self._K @ p_cam.T).T
            us = (uv[:, 0] / uv[:, 2]).astype(int)
            vs = (uv[:, 1] / uv[:, 2]).astype(int)

            # Boundary check at full resolution, then scale to downsampled coords
            ok = (us >= 0) & (us < W) & (vs >= 0) & (vs < H)
            us = us[ok] // ds
            vs = vs[ok] // ds

            if link_name == self._ee_link:
                mask_ee[vs, us] = 255
            else:
                mask_normal[vs, us] = 255

            # The same points, keeping the depth this loop used to discard.
            # `.at` is the unbuffered form: without it, repeated indices in
            # (vs, us) — which are the norm, many samples land on one
            # downsampled pixel — would keep an arbitrary one instead of the
            # nearest.
            np.minimum.at(zbuf, (vs, us), p_cam[ok, 2].astype(np.float32))

        mask_normal = _dilate(mask_normal, dilate_px)
        mask_ee     = _dilate(mask_ee,     ee_dilate_px)

        # ── EE-margin compensation data (consumed downstream by DistanceEngine) ──
        # search_exclusion_mask dilata il bordo del robot in pixel-space per evitare
        # falsi positivi da rumore di profondità/self-occlusion. Questo sposta
        # artificialmente il primo pixel-ostacolo osservabile oltre la vera superficie
        # del robot. DistanceEngine usa ee_source_mask e dilation_margins_px per
        # sottrarre questo margine in metri dalla distanza calcolata, così che un
        # ostacolo a contatto col bordo dilatato riporti distanza ≈ 0 invece di un
        # offset positivo nascosto.
        #
        # Provenienza per-pixel del massimo combinato: True dove la dilatazione EE
        # domina quella body. In caso di parità si attribuisce alla sorgente EE,
        # perché è il margine più grande/conservativo (sovrastimare il margine EE può
        # solo accorciare la distanza riportata, mai mascherare una violazione).
        # Calcolata sulle mask già dilatate, PRIMA del np.maximum, così resta coerente
        # col pixel che vincerà il massimo.
        ee_source_ds = (mask_ee >= mask_normal) & (mask_ee > 0)

        combined_u8 = np.maximum(mask_normal, mask_ee)   # (Hds, Wds) uint8

        # Contours computed on downsampled mask; coordinates scaled to full res
        contours_ds, _ = cv2.findContours(
            combined_u8, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        self.contours = [c * ds for c in contours_ds]

        # Upsample to full resolution with nearest-neighbour (preserves edges)
        combined_full      = cv2.resize(combined_u8, (W, H), interpolation=cv2.INTER_NEAREST)
        self.robot_mask    = combined_full > 0

        # Exclusion mask: apply extra dilation on downsampled image, then upsample
        excl_ds = combined_u8.copy()
        if extra_px > 0:
            excl_ds = _dilate(excl_ds, extra_px)
        excl_full = cv2.resize(excl_ds, (W, H), interpolation=cv2.INTER_NEAREST)
        self.search_exclusion_mask = excl_full > 0

        # Propaga la provenienza EE nella stessa fascia di dilatazione extra usata
        # per excl_ds (stesso kernel ellittico / raggio extra_px di _dilate()), poi
        # threshold > 0. Approssima "il pixel pre-extra-dilation più vicino era EE",
        # così la banda aggiunta eredita la sorgente corretta. Upsample con NEAREST
        # esattamente come search_exclusion_mask.
        ee_source_u8 = ee_source_ds.astype(np.uint8) * 255
        if extra_px > 0:
            ee_source_u8 = _dilate(ee_source_u8, extra_px)
        ee_source_full = cv2.resize(ee_source_u8, (W, H), interpolation=cv2.INTER_NEAREST)
        self.ee_source_mask = ee_source_full > 0

        # ── Depth buffer: fill the holes, then upsample ─────────────────────
        # The samples are sparse, so most pixels inside the silhouette carry no
        # depth. A MIN-FILTER over the same structuring element the mask was
        # dilated with gives each of them the nearest robot surface in its
        # neighbourhood — dilating a mask and eroding a depth buffer are the
        # same operation, so the buffer ends up covering exactly the region the
        # exclusion mask covers.
        #
        # Eroding +inf is a no-op wherever nothing is near, which is what keeps
        # "the robot is not here" distinguishable from "the robot is very far".
        # cv2 will not erode float32 with inf, so the filter runs on a finite
        # sentinel and the inf is restored after.
        far = float(self._depth_far_sentinel)
        z_fin = np.where(np.isfinite(zbuf), zbuf, far).astype(np.float32)
        r_fill = max(dilate_px, ee_dilate_px) + max(extra_px, 0)
        if r_fill > 0:
            z_fin = cv2.erode(z_fin, _kernel(r_fill))
        z_fin[z_fin >= far - 1e-3] = np.inf
        self.robot_depth = cv2.resize(z_fin, (W, H),
                                      interpolation=cv2.INTER_NEAREST)

        # Margini totali EFFETTIVI a piena risoluzione: il raggio di dilatazione è
        # applicato in spazio downsampled, quindi un raggio di N px downsampled vale
        # N·ds px a piena risoluzione. Sommiamo dilatazione base + extra per sorgente.
        self.dilation_margins_px = (
            dilate_px * ds + extra_px * ds,        # margin_body_px
            ee_dilate_px * ds + extra_px * ds,     # margin_ee_px
        )

        self._last_t_vecs = {n: t.copy() for n, (_, t) in transforms.items()}

    def invalidate(self):
        self._frame_skip  = self._skip_max
        self._last_t_vecs = {}


def _kernel(r: int):
    """The structuring element ``_dilate`` uses, exposed so the depth buffer can
    be eroded with exactly the same shape the mask was dilated with."""
    return cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * r + 1, 2 * r + 1))


def _dilate(mask: np.ndarray, r: int) -> np.ndarray:
    if r <= 0 or not np.any(mask):
        return mask
    k = 2 * r + 1
    return cv2.dilate(mask, _kernel(r))
