"""Centralized logging helpers for distance-only controller.

These helpers keep node files short while preserving focused log content.
"""

from __future__ import annotations

from typing import Any


def log_capsule_distances_throttled(
    *,
    distances_data: list[dict],
    obstacles: list,
    logger: Any,
    last_wall: float,
    now_wall: float,
    throttle_s: float = 0.5,
) -> float:
    """Log capsule-obstacle distances in a compact format (throttled).
    
    Excludes capsule 0 (link0→joint1) as requested.
    Formats: capsule1: obstacleA=24cm obstacleB=34cm
    
    Args:
        distances_data: List of dicts with 'capsule_idx', 'obstacle_name', 'distance'.
        obstacles: List of obstacles (for context).
        logger: ROS logger.
        last_wall: Last wall clock time this was logged.
        now_wall: Current wall clock time.
        throttle_s: Throttle interval in seconds.
    
    Returns:
        Updated last_wall time (either now_wall or unchanged).
    """
    if (now_wall - float(last_wall)) < float(throttle_s):
        return last_wall
    
    if not distances_data or len(obstacles) == 0:
        logger.info("[CAPSULE-DISTANCES] no obstacles")
        return now_wall
    
    # Build map: {capsule_idx: {obstacle_name: min_distance}}
    distances_by_capsule = {}
    for dist_dict in distances_data:
        capsule_idx = int(dist_dict.get("capsule_idx", -1))
        obstacle_name = str(dist_dict.get("obstacle_name", "")).strip()
        distance = float(dist_dict.get("distance", 999.0))
        
        # Skip capsule 0 (link0→joint1)
        if capsule_idx == 0:
            continue
        
        # Skip invalid data
        if capsule_idx < 0 or not obstacle_name:
            continue
        
        if capsule_idx not in distances_by_capsule:
            distances_by_capsule[capsule_idx] = {}
        
        # Keep minimum distance for each capsule-obstacle pair
        obs_distances = distances_by_capsule[capsule_idx]
        if obstacle_name not in obs_distances or distance < obs_distances[obstacle_name]:
            obs_distances[obstacle_name] = distance
    
    if not distances_by_capsule:
        logger.info("[CAPSULE-DISTANCES] no data (all filtered)")
        return now_wall
    
    # Build compact log output
    log_lines = []
    for capsule_idx in sorted(distances_by_capsule.keys()):
        obs_dict = distances_by_capsule[capsule_idx]
        if not obs_dict:
            log_lines.append(f"  capsule{capsule_idx}: -")
            continue
        
        # Format: "obstacleA=24cm obstacleB=34cm"
        obs_parts = []
        for obs_name in sorted(obs_dict.keys()):
            distance_m = obs_dict[obs_name]
            distance_cm = distance_m * 100.0
            # Use integer if close to whole number, otherwise 1 decimal
            if abs(distance_cm - round(distance_cm)) < 0.05:
                obs_parts.append(f"{obs_name}={int(round(distance_cm))}cm")
            else:
                obs_parts.append(f"{obs_name}={distance_cm:.1f}cm")
        
        log_lines.append(f"  capsule{capsule_idx}: {' '.join(obs_parts)}")
    
    logger.info("[CAPSULE-DISTANCES]\n" + "\n".join(log_lines))
    return now_wall
