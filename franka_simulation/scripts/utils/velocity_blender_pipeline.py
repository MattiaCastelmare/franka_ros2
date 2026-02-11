"""High-level control pipeline for velocity_control_blender.

SIMPLIFIED BASE VERSION:
- Simple joint-space tracking to next waypoint
- Alpha-blending between tracking and avoidance based on obstacle distance
- No emergency gates, no CBF, no smoothing, just clean blending + clip
"""

from __future__ import annotations

from typing import Optional
import numpy as np

from .velocity_blender_state import BlenderRuntimeState


def step(
    *,
    rt: BlenderRuntimeState,
    params: object,
    now_wall: float,
    logger: object,
) -> Optional[np.ndarray]:
    """Run one control step (SIMPLIFIED BASE VERSION).

    Returns a joint-velocity command (np.ndarray), never None.

    Behavior:
    - If no trajectory: output avoidance only
    - If trajectory: blend tracking and avoidance based on obstacle distance
    - Simple waypoint advancement with threshold
    - No emergency gates, no CBF, no smoothing, just alpha-blending + clip
    """
    n = int(getattr(params, "n_dof"))

    # Extract current state
    q = np.array(rt.q, dtype=float).reshape(n)
    qdot_avoid = np.array(rt.qdot_avoid, dtype=float).reshape(n)
    pts = rt.trajectory_points

    # Parameters
    kp = float(getattr(params, "kp"))
    max_vel = float(getattr(params, "max_vel"))
    waypoint_threshold = float(getattr(params, "waypoint_threshold"))
    influence_distance = float(getattr(params, "influence_distance"))

    # =================================================================
    # CASE 1: No trajectory → pure avoidance
    # =================================================================
    if not rt.active or len(pts) == 0:
        qdot_cmd = qdot_avoid
        return np.clip(qdot_cmd, -max_vel, max_vel)

    # =================================================================
    # CASE 2: Trajectory following with avoidance blending
    # =================================================================

    # --- Select target waypoint ---
    current_index = int(rt.current_index)
    last_idx = len(pts) - 1

    # Clamp index to valid range
    if current_index < 0:
        current_index = 0
    if current_index > last_idx:
        current_index = last_idx

    q_target = np.array(pts[current_index], dtype=float).reshape(n)

    # Check if we reached current target and can advance
    error = q_target - q
    error_norm = float(np.linalg.norm(error))

    if error_norm < waypoint_threshold and current_index < last_idx:
        # Advance to next waypoint
        current_index += 1
        q_target = np.array(pts[current_index], dtype=float).reshape(n)
        error = q_target - q
        error_norm = float(np.linalg.norm(error))

    # Update runtime state
    rt.current_index = current_index

    # Check completion at final waypoint
    final_threshold = float(getattr(params, "final_threshold", waypoint_threshold))
    if current_index == last_idx and error_norm < final_threshold:
        logger.info(f"✅ Traiettoria completata! Errore finale: {error_norm:.4f} rad")
        rt.active = False
        return np.zeros(n, dtype=float)

    # --- Compute tracking command ---
    qdot_tracking = kp * error

    # --- Compute alpha based on closest obstacle distance ---
    d = float(rt.closest_d) if rt.closest_d is not None else float('inf')

    # Alpha blending function (piecewise linear):
    # d >= influence_distance → alpha = 1.0 (pure tracking)
    # d at 2/3 * influence_distance → alpha = 0.5
    # d at 1/3 * influence_distance → alpha = 0.2
    # d <= 0 → alpha = 0.0 (pure avoidance)

    d_far = influence_distance
    d_mid = (2.0 / 3.0) * influence_distance
    d_near = (1.0 / 3.0) * influence_distance

    if d >= d_far:
        alpha = 1.0
    elif d >= d_mid:
        # Linear from 1.0 at d_far to 0.5 at d_mid
        alpha = 0.5 + 0.5 * (d - d_mid) / (d_far - d_mid)
    elif d >= d_near:
        # Linear from 0.5 at d_mid to 0.2 at d_near
        alpha = 0.2 + 0.3 * (d - d_near) / (d_mid - d_near)
    else:
        # Linear from 0.2 at d_near to 0.0 at 0
        if d_near > 0:
            alpha = 0.2 * max(0.0, d) / d_near
        else:
            alpha = 0.0 if d <= 0 else 0.2

    # Clamp alpha to [0, 1] for safety
    alpha = max(0.0, min(1.0, alpha))

    # --- Blend tracking and avoidance ---
    qdot_cmd = alpha * qdot_tracking + (1.0 - alpha) * qdot_avoid

    # --- Final clip to max velocity ---
    qdot_cmd = np.clip(qdot_cmd, -max_vel, max_vel)

    return qdot_cmd
