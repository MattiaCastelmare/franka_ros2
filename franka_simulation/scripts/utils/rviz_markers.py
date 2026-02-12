"""RViz marker building helpers.

This module keeps RViz-specific marker construction out of the main controller.
It is still ROS-message-facing (via avoidance_math.make_*), but it does not depend
on rclpy.

Ordering of markers is preserved to keep RViz stable across refactors.
"""

from __future__ import annotations

from typing import Any, Dict, List

import numpy as np

from visualization_msgs.msg import MarkerArray  # type: ignore

from .avoidance_math import make_capsule_markers, make_distance_markers


def build_marker_array(
    *,
    capsules: Dict[str, List[dict]],
    frame_ids: Dict[str, int],
    data: Any,
    distances_data: List[dict],
    influence_distance: float,
    distance_inflation: float,
    stamp_msg: Any,
    logger: Any = None,
    debug_capsule_index: int = -1,
) -> MarkerArray:
    """Build a MarkerArray for robot capsules + debug distance segments."""
    marker_array = MarkerArray()
    marker_id = 0
    prev_capsule_end = None  # For cap5 start point

    # ================= RViz CAPSULE VISUALIZATION =================
    for idx, capsule_name in enumerate(sorted(capsules.keys())):
        capsule_list = capsules[capsule_name]

        for caps in capsule_list:
            # Extract positions based on type (joint-to-joint capsules)
            start_id = int(caps["start_id"])
            end_id = int(caps["end_id"])
            start_type = str(caps["start_type"])
            end_type = str(caps["end_type"])
            
            # Get p0 (start point)
            # Special case: cap5 uses the previous capsule's endpoint
            if caps.get("use_prev_capsule_end", False) and prev_capsule_end is not None:
                p0 = prev_capsule_end
            elif start_type == "joint":
                p0 = data.oMi[start_id].translation
            elif start_type == "frame":
                p0 = data.oMf[start_id].translation
            else:
                continue
            
            # Get p1 (end point)
            if end_type == "joint":
                p1 = data.oMi[end_id].translation
            elif end_type == "frame":
                p1 = data.oMf[end_id].translation
            else:
                continue
            
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

            markers = make_capsule_markers(
                p0,
                p1,
                float(caps["radius"]),
                int(marker_id),
                stamp_msg=stamp_msg,
            )
            marker_array.markers.extend(markers)
            marker_id += len(markers)

    # ================= RViz DISTANCE VISUALIZATION =================
    debug_count = 0
    infl_bias = max(0.0, float(distance_inflation))
    for dist_data in distances_data:
        p_cap = dist_data["p_capsule"]
        p_obs = dist_data["p_obstacle"]
        d = float(dist_data["distance"])
        d_eff = float(d) - float(infl_bias)
        infl = float(influence_distance)
        is_active = bool(d_eff <= float(infl))
        rgba = (1.0, 0.0, 0.0, 0.8) if is_active else (0.0, 0.0, 1.0, 0.8)

        # Log per debug (una volta ogni 50 marker per non spammare)
        if logger is not None and debug_count == 0:
            color_name = "RED" if is_active else "BLUE"
            try:
                logger.debug(
                    f"   Distance: {d:.4f}m (eff={float(d_eff):.4f}, infl={float(infl):.4f}) → {color_name}"
                )
            except Exception:
                pass
        debug_count = (debug_count + 1) % 50

        dm = make_distance_markers(
            p_cap,
            p_obs,
            float(d),
            int(marker_id),
            stamp_msg=stamp_msg,
            activation_distance=float(infl),
            rgba=rgba,
        )
        marker_array.markers.extend(dm)
        marker_id += len(dm)

    return marker_array
