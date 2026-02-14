"""Utilities for closest constraint selection and formatting.

This module provides helper functions for selecting the closest hazard candidate
and formatting human-readable labels. Used by the distance-only controller.
"""

from __future__ import annotations

from typing import Optional, Sequence

import numpy as np


def select_closest_candidate(
    best_candidates: Sequence[dict],
) -> tuple[Optional[dict], float, float]:
    """Select the closest candidate with finite distance.
    
    Args:
        best_candidates: List of candidate dicts with 'd' field.
    
    Returns:
        (closest_candidate, closest_distance, d_min_raw)
        where d_min_raw is the raw minimum distance (999.0 if all invalid).
    """
    closest_candidate = None
    closest_distance = float("inf")
    d_min_raw = 999.0
    
    for candidate in best_candidates:
        d_val = float(candidate.get("d", float("inf")))
        if not np.isfinite(d_val):
            continue
        if d_val < closest_distance:
            closest_distance = d_val
            closest_candidate = candidate
    
    if closest_candidate is not None and np.isfinite(closest_distance):
        d_min_raw = float(closest_distance)
    
    return closest_candidate, closest_distance, d_min_raw


def format_closest_label(closest_candidate: Optional[dict]) -> str:
    """Format a human-readable label for the closest constraint.
    
    Args:
        closest_candidate: Candidate dict with 'hazard', 'kind', 'link', 'link_i', 'link_j' fields.
    
    Returns:
        Label string like 'hazard@link' or 'link_i↔link_j' for self-collision.
    """
    if closest_candidate is None:
        return "none"
    
    hazard = str(closest_candidate.get("hazard", "")).strip()
    kind = str(closest_candidate.get("kind", "")).strip()
    link_desc = ""
    
    if kind == "self":
        link_i = str(closest_candidate.get("link_i", "?"))
        link_j = str(closest_candidate.get("link_j", "?"))
        link_desc = f"{link_i}↔{link_j}"
    else:
        link_desc = str(closest_candidate.get("link", "")).strip()
    
    if hazard and link_desc:
        return hazard if "@" in hazard else f"{hazard}@{link_desc}"
    elif hazard:
        return hazard
    elif link_desc:
        return link_desc
    else:
        return kind or "hazard"
