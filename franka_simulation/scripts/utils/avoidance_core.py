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
    stable_sign_from_id,
    tangential_dir,
)


def iter_world_capsule_segments(
    *,
    capsules: Dict[str, List[dict]],
    frame_ids: Dict[str, int],
    data: Any,
) -> List[dict]:
    """Build capsule segments (p0/p1 in world) used for ground/self-collision checks.

    IMPORTANT: order is kept identical to the controller's iteration order to avoid
    changing marker IDs and the ordering of debug distance markers.
    """
    segments: List[dict] = []
    link_to_index = {f"fr3_link{i}": i for i in range(1, 9)}

    for parent in capsules:
        fid = int(frame_ids[parent])
        link_idx = int(link_to_index.get(parent, 0))

        for caps in capsules[parent]:
            oMp = data.oMf[fid]
            p0 = oMp.translation + oMp.rotation @ caps["p0"]
            p1 = oMp.translation + oMp.rotation @ caps["p1"]
            segments.append(
                {
                    "parent": parent,
                    "fid": fid,
                    "link_idx": link_idx,
                    "p0": np.array(p0, dtype=float).reshape(3),
                    "p1": np.array(p1, dtype=float).reshape(3),
                    "radius": float(caps["radius"]),
                }
            )

    return segments


def scan_external_and_ground(
    *,
    segments: List[dict],
    obstacles: List[Any],
    model: Any,
    data: Any,
    q: np.ndarray,
    # distance model knobs
    box_projection_iters: int,
    repulsion_spread_enable: bool,
    repulsion_spread_samples: int,
    repulsion_spread_half_length: float,
    # external avoidance params
    d_infl: float,
    d_aggr: float,
    d_safe: float,
    k_aggr: float,
    k_null: float,
    k_tan: float,
    # ground params
    enable_ground: bool,
    ground_z: float,
    ground_infl: float,
    ground_safe: float,
    k_ground: float,
) -> Tuple[np.ndarray, float, Dict[str, dict], Optional[dict], List[dict]]:
    """Scan PlanningScene obstacles + ground plane and build the nominal avoidance command.

    Returns:
      - qdot_avoid: nominal avoidance joint velocity (potential-field based)
      - d_min: global minimum distance (external/ground) found so far
      - external_best: per-obstacle closest capsule contact (for CBF candidates)
      - ground_best: closest ground contact (for CBF candidate)
      - distances_data: list for RViz debug markers (order preserved)
    """
    qdot_avoid = np.zeros(7)
    d_min = 999.0
    external_best: Dict[str, dict] = {}
    ground_best: Optional[dict] = None
    distances_data: List[dict] = []

    for seg in segments:
        p0 = np.array(seg["p0"], dtype=float).reshape(3)
        p1 = np.array(seg["p1"], dtype=float).reshape(3)
        fid = int(seg["fid"])
        radius = float(seg["radius"])

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

            # Record best (closest) capsule contact for this obstacle (used by CBF constraints)
            obs_id = str(getattr(obs, "id", ""))
            if obs_id not in external_best or float(d) < float(external_best[obs_id]["d"]):
                external_best[obs_id] = {
                    "kind": "external",
                    "hazard": f"external:{obs_id}",
                    "d": float(d),
                    "fid": int(fid),
                    "p": np.array(p_seg, dtype=float).reshape(3),
                    "n": np.array(dir_vec, dtype=float).reshape(3),
                }

            # Distance markers (for RViz)
            distances_data.append({"p_capsule": p_seg, "p_obstacle": p_box, "distance": d})

            # Track minimum distance regardless of activation
            d_min = min(d_min, float(d))

            # Outside influence zone: do not contribute to nominal avoidance
            if d >= float(d_infl):
                continue

            sgn = stable_sign_from_id(str(getattr(obs, "id", "")))

            # If enabled, use multiple points around the closest point to create a "region" repulsion.
            rep_points = samples if (repulsion_spread_enable and len(samples) > 0) else [
                {"p_seg": p_seg, "dir": dir_vec, "distance": float(d), "weight": 1.0}
            ]

            # Combine region samples as a WEIGHTED AVERAGE in joint space.
            qdot_reg = np.zeros(7)
            w_sum = 0.0
            for s in rep_points:
                ds = float(s.get("distance", d))
                if ds >= float(d_infl):
                    continue

                # Base activation (0 at d_infl, 1 at d_aggr or closer)
                alpha_far = smooth_alpha(ds, float(d_infl), float(d_aggr))
                # Extra aggressive scaling inside the d_aggr zone down to d_safe
                alpha_close = smooth_alpha(ds, float(d_aggr), float(d_safe))
                gain_scale = 1.0 + float(k_aggr) * float(alpha_close)

                dir_s = np.array(s.get("dir", dir_vec), dtype=float).reshape(3)
                tan = tangential_dir(dir_s)
                w = float(s.get("weight", 1.0))
                if w <= 0.0:
                    continue

                xdot_avoid = (
                    (float(k_null) * alpha_far * gain_scale) * dir_s
                    + (float(k_tan) * alpha_far * gain_scale * float(sgn)) * tan
                )

                Jp = point_jacobian_world(
                    model,
                    data,
                    q,
                    fid,
                    np.array(s.get("p_seg", p_seg), dtype=float).reshape(3),
                )
                qdot_reg += w * (Jp.T @ xdot_avoid)
                w_sum += w

            if w_sum > 1e-9:
                qdot_avoid += (qdot_reg / w_sum)

        # ===== Ground (floor plane z = ground_z) =====
        if bool(enable_ground):
            p_low = p0 if p0[2] <= p1[2] else p1
            d_ground = float((p_low[2] - float(ground_z)) - radius)
            p_plane = np.array([p_low[0], p_low[1], float(ground_z)], dtype=float)

            distances_data.append({"p_capsule": p_low, "p_obstacle": p_plane, "distance": d_ground})
            d_min = min(d_min, float(d_ground))

            if (ground_best is None) or (float(d_ground) < float(ground_best["d"])):
                ground_best = {
                    "kind": "ground",
                    "hazard": "ground:plane",
                    "d": float(d_ground),
                    "fid": int(fid),
                    "p": np.array(p_low, dtype=float).reshape(3),
                    "n": np.array([0.0, 0.0, 1.0], dtype=float),
                }

            if d_ground < float(ground_infl):
                alpha_g = smooth_alpha(float(d_ground), float(ground_infl), float(ground_safe))
                dir_g = np.array([0.0, 0.0, 1.0], dtype=float)
                xdot_g = float(k_ground) * float(alpha_g) * dir_g
                Jp_g = point_jacobian_world(model, data, q, fid, p_low)
                qdot_avoid += Jp_g.T @ xdot_g

    return qdot_avoid, float(d_min), external_best, ground_best, distances_data


def scan_self_collision(
    *,
    segments: List[dict],
    model: Any,
    data: Any,
    q: np.ndarray,
    # params
    enable_self: bool,
    self_skip_adjacent_links: int,
    self_infl: float,
    self_safe: float,
    k_self: float,
    d_min_in: float,
) -> Tuple[np.ndarray, float, Optional[dict], List[dict]]:
    """Scan capsule-capsule self-collision and build the nominal self-repulsion term.

    Returns:
      - qdot_self: avoidance contribution from self-collision
      - d_min: possibly updated global minimum distance (including self)
      - self_best: closest self-collision pair (for CBF candidate)
      - distances_data: distance markers for RViz (order preserved)
    """
    qdot_self = np.zeros(7)
    d_min = float(d_min_in)
    self_best: Optional[dict] = None
    distances_data: List[dict] = []

    if not (bool(enable_self) and len(segments) >= 2):
        return qdot_self, d_min, self_best, distances_data

    for i in range(len(segments)):
        si = segments[i]
        for j in range(i + 1, len(segments)):
            sj = segments[j]

            if abs(int(si["link_idx"]) - int(sj["link_idx"])) <= int(self_skip_adjacent_links):
                continue

            cp_i, cp_j = closest_points_on_segments(si["p0"], si["p1"], sj["p0"], sj["p1"])
            diff = cp_i - cp_j
            dist = float(np.linalg.norm(diff) - (float(si["radius"]) + float(sj["radius"])))

            d_min = min(d_min, float(dist))

            if (self_best is None) or (float(dist) < float(self_best["d"])):
                n_self = diff / (np.linalg.norm(diff) + 1e-9)
                self_best = {
                    "kind": "self",
                    "hazard": f"self:{si['parent']}<->{sj['parent']}",
                    "d": float(dist),
                    "fid_i": int(si["fid"]),
                    "p_i": np.array(cp_i, dtype=float).reshape(3),
                    "fid_j": int(sj["fid"]),
                    "p_j": np.array(cp_j, dtype=float).reshape(3),
                    "n": np.array(n_self, dtype=float).reshape(3),
                }

            distances_data.append({"p_capsule": cp_i, "p_obstacle": cp_j, "distance": dist})

            if dist >= float(self_infl):
                continue

            n = diff / (np.linalg.norm(diff) + 1e-9)
            alpha_s = smooth_alpha(float(dist), float(self_infl), float(self_safe))
            xdot_s = float(k_self) * float(alpha_s) * n

            J_i = point_jacobian_world(model, data, q, int(si["fid"]), cp_i)
            J_j = point_jacobian_world(model, data, q, int(sj["fid"]), cp_j)
            J_rel = (J_i - J_j)
            qdot_self += J_rel.T @ xdot_s

    return qdot_self, float(d_min), self_best, distances_data
