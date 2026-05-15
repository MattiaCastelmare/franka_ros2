"""Visualization helpers for the RealTimeDistance node.

VisFrame is an immutable snapshot of all data needed to render one overlay frame.
draw_overlay() is a pure function: it takes a VisFrame and static camera
parameters, and returns a BGR image — it touches no shared node state.

Thread safety: process_depth() builds a VisFrame atomically (single reference
assignment under GIL + Lock) and stores it in node._vis_frame.
visualize() reads the reference once under the Lock, then operates entirely on
the local copy with no further shared-state access.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import cv2
import numpy as np


@dataclass
class VisFrame:
    """Snapshot of everything needed to draw one visualization frame.

    All fields are written once by process_depth() and then read-only.
    NumPy arrays here are the same objects as inside the node (no copy) —
    because visualize() runs strictly after process_depth() hands off the
    reference, and neither mutates the arrays in-place.
    """
    depth:             np.ndarray
    robot_mask:        Optional[np.ndarray]
    contours:          Optional[list]
    robot_segments:    Optional[list]
    cp_results:        Optional[list]           # List[ControlPointResult] or None
    closest_robot_pt:  Optional[np.ndarray]
    closest_uv_obs:    Optional[Tuple[int, int]]
    min_dist:          float
    roi_bounds:        Optional[Tuple[int, int, int, int]]
    use_segment_mode:  bool
    stamp:             object                   # builtin_interfaces.msg.Time

    # Visualization flags — copied from config at snapshot time so the renderer
    # reads zero shared node state.
    visual_ROI:               bool = False
    visual_exclusion_mask:    bool = True
    visualize_only_raw_video: bool = False


def draw_overlay(
    frame: VisFrame,
    zones: dict,
    K: np.ndarray,
    R_base: np.ndarray,
    t_base: np.ndarray,
    depth_shape: Tuple[int, int],
) -> np.ndarray:
    """Render a BGR overlay image from a VisFrame snapshot.

    Pure function — no shared state, no side effects.
    """
    depth_vis = cv2.normalize(frame.depth, None, 0, 255, cv2.NORM_MINMAX)
    depth_vis = depth_vis.astype(np.uint8)
    depth_vis = cv2.cvtColor(depth_vis, cv2.COLOR_GRAY2BGR)

    if frame.visual_ROI and frame.roi_bounds is not None:
        x0, y0, x1, y1 = frame.roi_bounds
        cv2.rectangle(depth_vis, (x0, y0), (x1, y1), (255, 0, 255), 2)

    if frame.visualize_only_raw_video:
        return depth_vis

    # Robot mask (blackout + contour outline)
    if frame.robot_mask is not None and frame.visual_exclusion_mask:
        depth_vis[frame.robot_mask] = (0, 0, 0)
        if frame.contours:
            cv2.drawContours(depth_vis, frame.contours, -1, (0, 255, 0), 3)

    # Skeleton: one line per capsule segment
    if frame.robot_segments:
        for seg in frame.robot_segments:
            uv_a = _project(seg['p0'], K, R_base, t_base, depth_shape)
            uv_b = _project(seg['p1'], K, R_base, t_base, depth_shape)
            if uv_a and uv_b:
                cv2.line(depth_vis, uv_a, uv_b, (0, 255, 0), 1)
                cv2.circle(depth_vis, uv_a, 2, (0, 255, 0), -1)
                cv2.circle(depth_vis, uv_b, 2, (0, 255, 0), -1)

    # Per-CP distance lines (CP mode only)
    if not frame.use_segment_mode and frame.cp_results:
        for r in frame.cp_results:
            if not np.isfinite(r.distance) or r.closest_pixel is None:
                continue
            uv_cp = _project(r.point, K, R_base, t_base, depth_shape)
            if uv_cp:
                cv2.line(depth_vis, uv_cp, r.closest_pixel,
                         _zone_color(r.distance, zones), 2)

    # Closest obstacle pixel (red dot)
    u, v = 20, 20
    if frame.closest_uv_obs is not None:
        u, v = frame.closest_uv_obs
        cv2.circle(depth_vis, (u, v), 5, (0, 0, 255), -1)

    # Closest robot point (cyan) + connecting line
    if frame.closest_robot_pt is not None and frame.closest_uv_obs is not None:
        uv_robot = _project(frame.closest_robot_pt, K, R_base, t_base, depth_shape)
        if uv_robot:
            line_color = (
                _zone_color(frame.min_dist, zones)
                if np.isfinite(frame.min_dist) else (255, 255, 255)
            )
            cv2.circle(depth_vis, uv_robot, 5, (255, 255, 0), -1)
            cv2.line(depth_vis, frame.closest_uv_obs, uv_robot, line_color, 3)

    # CP markers (orange, CP mode only)
    if not frame.use_segment_mode and frame.cp_results:
        for r in frame.cp_results:
            uv = _project(r.point, K, R_base, t_base, depth_shape)
            if uv:
                cv2.circle(depth_vis, uv, 3, (0, 165, 255), -1)

    # Distance text
    if np.isfinite(frame.min_dist):
        cv2.putText(
            depth_vis, f'{frame.min_dist:.3f} m',
            (u + 10, v), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

    return depth_vis


def _project(
    p_base: np.ndarray,
    K: np.ndarray,
    R_base: np.ndarray,
    t_base: np.ndarray,
    depth_shape: Tuple[int, int],
) -> Optional[Tuple[int, int]]:
    p_cam = R_base.T @ (np.asarray(p_base, dtype=np.float64) - t_base)
    if p_cam[2] <= 0:
        return None
    uv  = K @ p_cam
    u, v = int(uv[0] / uv[2]), int(uv[1] / uv[2])
    H, W = depth_shape
    return (u, v) if 0 <= u < W and 0 <= v < H else None


def _zone_color(distance: float, zones: dict) -> Tuple[int, int, int]:
    if distance <= zones.get('critical', 0.1):
        return (0, 0, 255)
    if distance <= zones.get('danger', 0.2):
        return (0, 165, 255)
    if distance <= zones.get('warning', 0.3):
        return (0, 255, 255)
    return (0, 255, 0)
