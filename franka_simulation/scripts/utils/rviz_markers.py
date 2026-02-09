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
) -> MarkerArray:
    """Build a MarkerArray for robot capsules + debug distance segments."""
    marker_array = MarkerArray()
    marker_id = 0

    # ================= RViz CAPSULE VISUALIZATION =================
    for parent in capsules:
        fid = int(frame_ids[parent])

        for caps in capsules[parent]:
            oMp = data.oMf[fid]
            p0 = oMp.translation + oMp.rotation @ caps["p0"]
            p1 = oMp.translation + oMp.rotation @ caps["p1"]

            markers = make_capsule_markers(
                np.array(p0, dtype=float).reshape(3),
                np.array(p1, dtype=float).reshape(3),
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
