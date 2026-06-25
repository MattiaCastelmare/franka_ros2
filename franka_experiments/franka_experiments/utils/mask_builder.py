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
        else:
            self._mask_normal_buf[:] = 0
            self._mask_ee_buf[:]     = 0
        mask_normal = self._mask_normal_buf
        mask_ee     = self._mask_ee_buf

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


def _dilate(mask: np.ndarray, r: int) -> np.ndarray:
    if r <= 0 or not np.any(mask):
        return mask
    k = 2 * r + 1
    return cv2.dilate(mask, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k)))
