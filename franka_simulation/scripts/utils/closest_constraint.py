"""Utilities for closest constraint selection and formatting.

This module provides helper functions for selecting the closest hazard candidate
and formatting human-readable labels. Used by the distance-only controller.
"""

from __future__ import annotations

from typing import Optional, Sequence, Tuple

import numpy as np

from .avoidance_math import capsule_risk_weight


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


def select_closest_candidate_risk_weighted(
    best_candidates: Sequence[dict],
    *,
    weight_last2: float = 2.0,
    weight_last3: float = 1.5,
    weight_default: float = 1.0,
    use_risk_scoring: bool = True,
) -> Tuple[Optional[dict], float, float, float]:
    """Select the candidate with minimum risk-weighted score.

    Risk score is defined as::

        score = d / w

    where *w* is the capsule risk weight (higher for EE capsules).
    A low score means high risk — either the obstacle is very close
    or the capsule is safety-critical.

    When ``use_risk_scoring`` is False the function selects by minimum
    geometric distance *d* (like :func:`select_closest_candidate`) but
    still returns the weight of the selected capsule for downstream use.

    Args:
        best_candidates: List of candidate dicts with ``'d'`` and
            ``'capsule_idx'`` fields.
        weight_last2: Multiplier for the last 2 capsules.
        weight_last3: Multiplier for the 3rd-from-last capsule.
        weight_default: Multiplier for all other capsules.
        use_risk_scoring: If True select by min d/w; if False by min d.

    Returns:
        ``(closest_candidate, d_min_raw, risk_score, cap_weight)``

        - *d_min_raw*: raw geometric distance of the selected candidate.
        - *risk_score*: ``d_min_raw / cap_weight``.
        - *cap_weight*: risk weight of the selected capsule.
    """
    if not best_candidates:
        return None, 999.0, 999.0, 1.0

    # Determine last capsule index dynamically from candidates
    last_idx = -1
    for c in best_candidates:
        idx = int(c.get("capsule_idx", -1))
        if idx > last_idx:
            last_idx = idx

    best: Optional[dict] = None
    best_metric = float("inf")
    best_d = 999.0
    best_w = 1.0

    for c in best_candidates:
        d_val = float(c.get("d", float("inf")))
        if not np.isfinite(d_val):
            continue

        cap_idx = int(c.get("capsule_idx", -1))
        w = capsule_risk_weight(
            cap_idx,
            last_idx,
            weight_last2=weight_last2,
            weight_last3=weight_last3,
            weight_default=weight_default,
        )

        metric = (d_val / w) if (use_risk_scoring and w > 1e-9) else d_val

        if metric < best_metric:
            best_metric = metric
            best = c
            best_d = d_val
            best_w = w

    if best is None:
        return None, 999.0, 999.0, 1.0

    risk_score = best_d / best_w if best_w > 1e-9 else best_d
    return best, best_d, risk_score, best_w
