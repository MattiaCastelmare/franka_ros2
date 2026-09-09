"""Avoidance-first motion shaping — pure-numpy helpers (no ROS, no Pinocchio).

Used by ``pentagon_qddot_commander`` to make the *nominal* command obstacle-
aware, so the CBF filter downstream is a safety certificate rather than the
de-facto motion planner.  Design intent (see the avoidance-first refactor):

* :func:`influence_weight` — smooth activation γ(d) ∈ [0, 1] of the avoidance
  terms as an obstacle enters the influence shell ``d_infl → d_close``.
  Smoothstep (C¹) so the shaped acceleration never jumps on entry/exit.

* :func:`tangential_redirect` — the core "steer, don't brake" operator.  The
  component of a commanded acceleration that points INTO an obstacle is not
  merely removed (that is what the CBF projection already does, and it is what
  makes the robot park in front of obstacles); it is *rotated* into the tangent
  plane of the obstacle normal, preserving command magnitude.  The tangent is
  chosen from a preference direction (typically the desired path velocity) with
  hysteresis on the previous tangent so the detour side cannot flip-flop
  between frames.

* :func:`feasibility_beta_target` — target for the phase-governor scale
  β ∈ [0, 1].  Deliberately driven by FEASIBILITY evidence only (QP slack,
  safety-chain fault, persistent tracking error, manipulability collapse) and
  NEVER by raw obstacle distance: distance must shape the *direction* of
  motion (above), while speed is reduced only when no admissible motion
  remains — the "slow down as last resort" requirement.

* :func:`rate_limited` — asymmetric first-order rate limiter for β (brake
  fast, resume gently) keeping the virtual-time scaling C⁰ with bounded slope,
  which the commander chain-rules into v_d/a_d.
"""

# TODO[LEGACY]: imported only by test/test_avoidance.py; the restored commander has no governor | confidence: medium | superseded-by: none (was pentagon_qddot_commander governor, reverted in 4d4d450) | flagged: 2026-09-01

from __future__ import annotations

import numpy as np


def influence_weight(d: float, d_close: float, d_infl: float) -> float:
    """Smoothstep activation γ(d): 0 for d ≥ d_infl, 1 for d ≤ d_close.

    ``d_close < d_infl`` is expected; degenerate configs fall back to a hard
    step at ``d_infl`` (still bounded in [0, 1], never NaN).
    """
    if not np.isfinite(d) or d >= d_infl:
        return 0.0
    if d <= d_close or d_infl - d_close <= 1e-9:
        return 1.0
    x = (d_infl - d) / (d_infl - d_close)          # 0 → 1 as d: d_infl → d_close
    return float(x * x * (3.0 - 2.0 * x))          # smoothstep, C¹ at both ends


def tangential_redirect(
    a: np.ndarray,           # (3,) commanded acceleration (any control frame)
    n_hat: np.ndarray,       # (3,) unit normal, obstacle → robot (same frame)
    gamma: float,            # [0, 1] activation from influence_weight()
    t_pref: np.ndarray,      # (3,) preferred travel direction (e.g. v_d)
    t_prev: np.ndarray | None = None,   # (3,) previous tangent (hysteresis)
    redirect_max: float = np.inf,       # cap on the re-injected magnitude
    eps: float = 1e-9,
) -> tuple[np.ndarray, np.ndarray | None]:
    """Rotate the into-obstacle component of ``a`` into the tangent plane.

    Returns ``(a_shaped, t_hat)`` where ``t_hat`` is the tangent actually used
    (None when the redirect was inactive).  Semantics:

    * approach = component of ``a`` along −n̂ (toward the obstacle).  When
      approach ≤ 0 the command already moves away → untouched.
    * a_shaped = a + γ·approach·n̂  (removes γ of the approach)
                   + γ·min(approach, redirect_max)·t̂  (re-injects it sideways)
    * t̂ = normalized tangential projection of ``t_pref``; if degenerate
      (head-on: t_pref ∥ n̂) fall back to ``t_prev``'s projection, then to an
      arbitrary axis ⊥ n̂ — deterministic, never a zero direction while γ > 0
      and approach > 0.

    Magnitude is (approximately) preserved instead of destroyed, which is what
    keeps the robot MOVING around the obstacle rather than braking in front of
    it.  Safety is not this operator's job — the CBF QP downstream still
    certifies the shaped command.
    """
    if gamma <= 0.0:
        return a, None
    approach = -float(a @ n_hat)                   # >0 ⇒ heading into obstacle
    if approach <= eps:
        return a, None

    # Tangent choice: preference direction, then hysteresis, then fixed fallback.
    t_hat = None
    for cand in (t_pref, t_prev):
        if cand is None:
            continue
        t_tan = cand - (cand @ n_hat) * n_hat
        norm = float(np.linalg.norm(t_tan))
        if norm > eps:
            t_hat = t_tan / norm
            break
    if t_hat is None:
        # Arbitrary but deterministic axis ⊥ n̂: cross with the world axis the
        # normal is least aligned with.
        k = int(np.argmin(np.abs(n_hat)))
        axis = np.zeros(3)
        axis[k] = 1.0
        t_tan = np.cross(n_hat, axis)
        t_hat = t_tan / float(np.linalg.norm(t_tan))

    a_shaped = a + (gamma * approach) * n_hat
    a_shaped = a_shaped + (gamma * min(approach, redirect_max)) * t_hat
    return a_shaped, t_hat


def feasibility_beta_target(
    slack: float,            # CBF QP slack (0 = constraint satisfiable)
    fault: bool,             # safety-chain fault braking (stale distance / QP fail)
    ee_err: float,           # [m] EE position tracking error
    manip: float,            # manipulability w = √det(JJᵀ)
    *,
    slack_engage: float,     # slack below this → no slowdown
    slack_full: float,       # slack at/above this → full stop demand
    err_lo: float,           # [m] error below this → no slowdown
    err_hi: float,           # [m] error at/above this → full stop demand
    manip_thr: float,        # w at/above this → no slowdown; w→0 → stop
) -> float:
    """β target ∈ [0, 1] from feasibility evidence only (never raw distance).

    β = min of independent factors, each 1 when healthy and → 0 when its
    evidence says no admissible motion remains:

    * fault      — safety chain degraded ⇒ 0 (stop the phase, robot is braking)
    * slack      — the CBF QP had to RELAX a barrier: geometrically no safe
                   direction had enough authority ⇒ scale down with slack
    * ee_err     — the robot physically cannot follow the (already avoidance-
                   shaped) reference ⇒ the path must wait
    * manip      — approaching a singular configuration ⇒ progressive slowdown
    """
    if fault:
        return 0.0
    f_slack = 1.0 - _unit_ramp(slack, slack_engage, slack_full)
    f_err   = 1.0 - _unit_ramp(ee_err, err_lo, err_hi)
    f_manip = _clamp01(manip / manip_thr) if manip_thr > 0.0 else 1.0
    return min(f_slack, f_err, f_manip)


def rate_limited(
    current: float,
    target: float,
    rate_up: float,          # [1/s] max increase rate (resume gently)
    rate_down: float,        # [1/s] max decrease rate (brake fast)
    dt: float,
) -> float:
    """One asymmetric rate-limiter step; returns the new value."""
    step = target - current
    step = min(step, rate_up * dt)
    step = max(step, -rate_down * dt)
    return current + step


def _unit_ramp(x: float, lo: float, hi: float) -> float:
    """0 for x ≤ lo, 1 for x ≥ hi, linear in between (degenerate → step)."""
    if hi - lo <= 1e-12:
        return 1.0 if x >= hi else 0.0
    return _clamp01((x - lo) / (hi - lo))


def _clamp01(x: float) -> float:
    return 0.0 if x < 0.0 else (1.0 if x > 1.0 else float(x))
