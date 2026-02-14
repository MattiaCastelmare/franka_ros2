"""Core avoidance logic (ROS-agnostic).

This module contains the *algorithmic* parts of the online avoidance controller:
- build capsule segments in world
- compute nominal avoidance from external obstacles + ground
- compute nominal avoidance from self-collision

It deliberately does **not** depend on rclpy or ROS message types.
The controller node remains responsible for:
- subscriptions/publications
- timers
- timestamping
- logging

Design note
-----------
The ordering of loops is preserved to keep marker IDs and debug marker ordering
stable.
"""

from __future__ import annotations

from typing import Any, Dict, List, Optional, Tuple

import numpy as np

from .avoidance_math import (
    closest_points_on_segments,
    distance_capsule_to_collision_object_boxes,
    point_jacobian_world,
    smooth_alpha,
)


def iter_world_capsule_segments(
    *,
    capsules: Dict[str, List[dict]],
    frame_ids: Dict[str, int],
    data: Any,
    debug_capsule_index: int = -1,
) -> List[dict]:
    """Build capsule segments (p0/p1 in world) for joint-to-joint capsules.
    
    Now uses direct joint/frame positions from Pinocchio instead of local coordinates.
    Applies geometry modifications for capsules 2, 4, 5 based on exact link lengths.
    
    IMPORTANT: order is kept identical to maintain stable marker IDs.
    """
    segments: List[dict] = []
    prev_capsule_end: Optional[np.ndarray] = None  # For cap5 start point

    for idx, capsule_name in enumerate(sorted(capsules.keys())):
        capsule_list = capsules[capsule_name]
        
        for caps in capsule_list:
            # Extract positions based on type
            start_id = int(caps["start_id"])
            end_id = int(caps["end_id"])
            start_type = str(caps["start_type"])
            end_type = str(caps["end_type"])
            link_idx = int(caps.get("link_idx", 0))
            
            # Get p0 (start point)
            # Special case: cap5 uses the previous capsule's endpoint
            if caps.get("use_prev_capsule_end", False) and prev_capsule_end is not None:
                p0 = prev_capsule_end
            elif start_type == "joint":
                p0 = data.oMi[start_id].translation
            elif start_type == "frame":
                p0 = data.oMf[start_id].translation
            else:
                raise ValueError(f"Unknown start_type: {start_type}")
            
            # Get p1 (end point)
            if end_type == "joint":
                p1 = data.oMi[end_id].translation
            elif end_type == "frame":
                p1 = data.oMf[end_id].translation
            else:
                raise ValueError(f"Unknown end_type: {end_type}")
            
            # Convert to numpy arrays
            p0 = np.array(p0, dtype=float).reshape(3)
            p1 = np.array(p1, dtype=float).reshape(3)
            
            # Apply shortening if target_length is specified (cap2, cap4)
            if "target_length" in caps:
                target_len = float(caps["target_length"])
                direction = p1 - p0
                current_len = float(np.linalg.norm(direction))
                if current_len > 1e-6:  # avoid division by zero
                    direction_normalized = direction / current_len
                    p1 = p0 + direction_normalized * target_len
            
            # Store endpoint for next capsule (needed for cap5)
            prev_capsule_end = p1.copy()
            
            # Apply debug filter AFTER computing prev_capsule_end
            if debug_capsule_index >= 0 and idx != debug_capsule_index:
                continue
            
            segments.append(
                {
                    "parent": capsule_name,
                    "fid": start_id,  # kept for compatibility
                    "link_idx": link_idx,
                    "capsule_idx": idx,  # for logging
                    "p0": p0,
                    "p1": p1,
                    "radius": float(caps["radius"]),
                }
            )

    return segments


def scan_external(
    *,
    segments: List[dict],
    obstacles: List[Any],
    box_projection_iters: int,
    repulsion_spread_enable: bool,
    repulsion_spread_samples: int,
    repulsion_spread_half_length: float,
    influence_distance: float,
    debug_stats: Optional[dict] = None,
) -> Tuple[float, Dict[str, dict], List[dict], List[dict], List[dict]]:
    """Scan PlanningScene collision objects and compute distances to robot capsules.

    This function performs distance computation ONLY (no velocity computation).
    It identifies closest contacts per obstacle and per link within influence distance.

    Returns:
      - d_min: global minimum distance (external obstacles only)
      - external_best: per-obstacle closest capsule contact (dict[obs_id, candidate])
      - distances_data: list for RViz debug markers (order preserved)
      - active_candidates: per-link closest contacts within influence distance
      - tip_to_obstacle_distances: closest points for the tool tip (for visualization)
    """
    d_min = 999.0
    external_best: Dict[str, dict] = {}
    distances_data: List[dict] = []
    active_candidates: List[dict] = []

    if debug_stats is not None:
        debug_stats.setdefault("per_link_closest", {})


    tip_to_obstacle_distances = []  # For end-effector tip distance visualization

    for seg in segments:
        link_idx = int(seg.get("link_idx", 0))
        # Exclude the first link from repulsion calculations
        if link_idx == 1:
            continue
        # Exclude the first capsule (capsule_idx == 0) from external/ground collision checks
        capsule_idx = int(seg.get("capsule_idx", -1))
        if capsule_idx == 0:
            continue
        p0 = np.array(seg["p0"], dtype=float).reshape(3)
        p1 = np.array(seg["p1"], dtype=float).reshape(3)
        fid = int(seg["fid"])
        radius = float(seg["radius"])
        parent_link = str(seg.get("parent", ""))
        best_link_candidate: Optional[dict] = None


        # ===== External obstacles (PlanningScene boxes) =====
        for obs in obstacles:
            d, dir_vec, p_seg, p_box, samples = distance_capsule_to_collision_object_boxes(
                p0_world=p0,
                p1_world=p1,
                radius=radius,
                obs=obs,
                box_projection_iters=int(box_projection_iters),
                repulsion_spread_enable=bool(repulsion_spread_enable),
                repulsion_spread_samples=int(repulsion_spread_samples),
                repulsion_spread_half_length=float(repulsion_spread_half_length),
            )
            if dir_vec is None:
                continue

            # --- End-effector tip distance calculation ---
            # If this segment is the last link (fr3_link8), treat p1 as the tip
            if link_idx == 8:
                tip_dist, tip_dir_vec, tip_p_seg, tip_p_box, _ = distance_capsule_to_collision_object_boxes(
                    p0_world=p1,  # Use p1 as the tip
                    p1_world=p1,
                    radius=radius,
                    obs=obs,
                    box_projection_iters=int(box_projection_iters),
                    repulsion_spread_enable=False,
                    repulsion_spread_samples=0,
                    repulsion_spread_half_length=0.0,
                )
                tip_to_obstacle_distances.append({
                    "p_tip": tip_p_seg,
                    "p_obstacle": tip_p_box,
                    "distance": tip_dist,
                    "infl": float(influence_distance),
                })

            # Ensure direction points from box -> capsule (p_seg - p_box).
            try:
                v = np.array(p_seg, dtype=float).reshape(3) - np.array(p_box, dtype=float).reshape(3)
                dv = np.array(dir_vec, dtype=float).reshape(3)
                if float(dv @ v) < 0.0:
                    dir_vec = -dv

                # Fix sample directions too (if any).
                if isinstance(samples, list) and len(samples) > 0:
                    fixed_samples = []
                    for s in samples:
                        try:
                            ps = np.array(s.get("p_seg", p_seg), dtype=float).reshape(3)
                            pb = np.array(s.get("p_box", p_box), dtype=float).reshape(3)
                            ds = np.array(s.get("dir", dir_vec), dtype=float).reshape(3)
                            if float(ds @ (ps - pb)) < 0.0:
                                ds = -ds
                            s2 = dict(s)
                            s2["dir"] = ds
                            fixed_samples.append(s2)
                        except Exception:
                            fixed_samples.append(s)
                    samples = fixed_samples
            except Exception:
                pass

            # Record best (closest) capsule contact for this obstacle (used by CBF constraints)
            obs_id = str(getattr(obs, "id", ""))
            candidate = {
                "kind": "external",
                "hazard": f"external:{obs_id}@{parent_link}",
                "d": float(d),
                "fid": int(fid),
                "p": np.array(p_seg, dtype=float).reshape(3),
                "n": np.array(dir_vec, dtype=float).reshape(3),
                "link": parent_link,
                "link_idx": int(link_idx),
            }

            if obs_id not in external_best or float(d) < float(external_best[obs_id]["d"]):
                external_best[obs_id] = dict(candidate)

            if (best_link_candidate is None) or (float(d) < float(best_link_candidate["d"])):
                best_link_candidate = dict(candidate)

            # Distance markers (for RViz)
            distances_data.append(
                {
                    "p_capsule": p_seg,
                    "p_obstacle": p_box,
                    "distance": d,
                    "infl": float(influence_distance),
                    "capsule_idx": int(seg.get("capsule_idx", -1)),  # for logging
                    "obstacle_name": obs_id,  # for logging
                }
            )

            # Track minimum distance regardless of activation
            d_min = min(d_min, float(d))

        if best_link_candidate is not None:
            if debug_stats is not None:
                try:
                    plc = debug_stats.setdefault("per_link_closest", {})
                    plc[parent_link] = {
                        "distance": float(best_link_candidate.get("d", 1e9)),
                        "hazard": str(best_link_candidate.get("hazard", "")),
                        "link_idx": int(best_link_candidate.get("link_idx", -1)),
                    }
                except Exception:
                    pass

            if float(best_link_candidate["d"]) <= float(influence_distance):
                active_candidates.append(best_link_candidate)

    return float(d_min), external_best, distances_data, active_candidates, tip_to_obstacle_distances


# REMOVED: scan_self_collision function (not needed in distance-only mode)
