"""Escape direction when the obstacle cannot be outrun: retreat, or step aside.

WHY THIS EXISTS
---------------
Every obstacle row asks for the same thing — accelerate the control point along
n̂, away from the obstacle — and the retreat cap shapes how fast. That is the
right answer as long as the point CAN move along n̂ faster than the obstacle
closes. The joint velocity box says exactly how fast it can, in closed form
(:func:`cbf_state_rows.retreat_speed_available`):

    v_avail = max{ n̂ᵀJ_p q̇ : |q̇ᵢ| ≤ q̇max,ᵢ } = Σᵢ |(n̂ᵀJ_p)ᵢ|·q̇max,ᵢ

and on this arm it is 2-3 m/s at the flange but 0.1-0.2 m/s on the link4
control points along their weak normal (measured, Phase 0). An obstacle closing
at n̂ᵀv_obs > v_avail is NOT outrun, whatever the gains: the barrier row pays
slack and the arm backs off into a contact it cannot prevent.

"Cannot outrun" is not "cannot avoid". A person who cannot back away from a
moving object steps SIDEWAYS, out of its path — and sideways is usually far
cheaper for the arm as well. So:

    outrunnable  ⇔  n̂ᵀv_obs ≤ margin · v_avail
    outrunnable  →  escape = +n̂   (today's behaviour, and NO bias: the barrier's
                                   normal retreat is the whole response)
    otherwise    →  escape = the direction ⟂ v_obs the arm can move along
                    FASTEST, i.e. the unit e ⟂ v̂_obs maximising the achievable
                    speed Σᵢ max((eᵀJ_p)ᵢ·q̇min,ᵢ, (eᵀJ_p)ᵢ·q̇max,ᵢ)

blended with a smooth ramp on r = n̂ᵀv_obs / (margin·v_avail), never a switch:
a hard threshold on a 30 Hz vision quantity chatters, and what is switched here
is a bias on q̈_nom — the arm would jerk sideways and back at the noise rate.

HOW IT ENTERS THE QP
--------------------
As a bias on the objective, exactly like ``cbf_evasion``:

    q̈_nom += gain · J_p⁺ · (accel · m · w · ê_escape)

with ``m`` a ramp in the closing speed (0 at v_obs = 0, so the flag-off and
zero-velocity paths are bit-identical) and ``w`` the blend weight. Every safety
row is untouched, so this cannot loosen a barrier or make the QP infeasible; a
wrong direction costs tracking error and nothing else.

SIGN CONVENTION (this repository's): n̂ points OBSTACLE → CONTROL POINT and
``n̂ᵀJ_p q̇ > 0`` is separating, so the normal retreat is +n̂. A spec written
with n̂ pointing the other way calls the same direction −n; the vector is the
same.

Pure numpy, no ROS, no Pinocchio.
"""

from __future__ import annotations

from typing import Optional, Tuple

import numpy as np

from franka_experiments.utils.cbf_state_rows import retreat_speed_available


def outrun_ratio(v_close: float, v_avail: float, *, margin: float,
                 v_avail_floor: float = 0.0) -> float:
    """``r = n̂ᵀv_obs / (margin·max(v_avail, v_avail_floor))``: how far the
    closing speed exceeds what the point can outrun. ``r ≤ 1`` is outrunnable.

    ``v_close ≤ 0`` (receding or static) is 0.0: nothing to outrun. A closing
    obstacle against a point with NO retreat authority is ``inf`` — the most
    urgent case there is, not a division error.

    ``v_avail_floor`` is what keeps that limit from swallowing the whole
    robot. ``v_avail`` collapses with the leverage of the row, and on this arm
    the link4 control points have as little as 0.10 m/s along their weak
    normal — so an UNFLOORED ratio makes 0.15 m/s of estimation noise read as
    "cannot be outrun" on 2-4 % of directions there, and 0.30 m/s on a third
    of them (measured). A point that cannot retreat at 0.3 m/s cannot escape
    sideways at 0.3 m/s either: its problem is leverage, not direction, and
    swerving it is noise with a unit vector attached. The floor says so.
    """
    v = float(v_close)
    if v <= 0.0:
        return 0.0
    den = float(margin) * max(float(v_avail), float(v_avail_floor))
    if den <= 1e-9:
        return float('inf')
    return v / den


def blend_weight(r: float, *, ramp_start: float) -> float:
    """Smoothstep from 0 at ``r = ramp_start`` to 1 at ``r = 1``.

    Engaging BEFORE the situation is provably hopeless is the whole value: an
    evasion that starts at r = 1 starts at the moment it stops mattering.
    ``ramp_start ≥ 1`` degenerates to a step at r = 1 and is clamped just
    below it so the function stays continuous.
    """
    lo = min(float(ramp_start), 1.0 - 1e-6)
    if not np.isfinite(r):
        return 1.0
    if r <= lo:
        return 0.0
    if r >= 1.0:
        return 1.0
    s = (r - lo) / (1.0 - lo)
    return float(s * s * (3.0 - 2.0 * s))


def _plane_basis(v_hat: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    u1 = np.cross(v_hat, np.array([0.0, 0.0, 1.0]))
    if float(np.linalg.norm(u1)) < 1e-6:
        u1 = np.cross(v_hat, np.array([0.0, 1.0, 0.0]))
    u1 /= np.linalg.norm(u1)
    u2 = np.cross(v_hat, u1)
    u2 /= np.linalg.norm(u2)
    return u1, u2


def lateral_direction(
    v_obs_vec: np.ndarray,
    Jp: np.ndarray,
    qdot_max: np.ndarray,
    d_vec: np.ndarray,
    *,
    v_min: float = 0.15,
    qdot_min: Optional[np.ndarray] = None,
    n_samples: int = 72,
) -> Optional[np.ndarray]:
    """(3,) unit direction ⟂ v_obs the arm can move along fastest, or ``None``.

    The achievable speed along a unit ``e`` over the joint velocity box is the
    same separable maximum the retreat authority uses,
    ``Σᵢ max((eᵀJ_p)ᵢ·q̇min,ᵢ, (eᵀJ_p)ᵢ·q̇max,ᵢ)``, which is convex in ``e``. On
    the circle ⟂ v̂ it is maximised by sampling ``n_samples`` directions — 5°
    at the default, deterministic, and the objective is piecewise linear so a
    finer grid buys nothing a control loop can use.

    The maximum is the same for ``±e`` (a symmetric box). The sign is resolved
    toward ``d_⊥ = d − (d·v̂)v̂`` with ``d = p_robot − p_obs``: the side of the
    obstacle's line of travel the point is ALREADY on, so the escape grows the
    miss distance instead of crossing the path. Head-on (``‖d_⊥‖ ≈ 0``) every
    side is equal and the sign is fixed deterministically, so it cannot flip
    frame to frame and shake the arm.

    ``None`` when the obstacle is not meaningfully moving (``‖v_obs‖ < v_min``:
    "perpendicular to its velocity" would be a direction fitted to noise) or
    when no direction in the plane has any leverage.
    """
    v = np.asarray(v_obs_vec, dtype=np.float64).ravel()
    d = np.asarray(d_vec, dtype=np.float64).ravel()
    if v.size != 3 or d.size != 3 or not (np.all(np.isfinite(v)) and np.all(np.isfinite(d))):
        return None
    v_norm = float(np.linalg.norm(v))
    if v_norm < float(v_min):
        return None
    v_hat = v / v_norm
    u1, u2 = _plane_basis(v_hat)
    Jp = np.asarray(Jp, dtype=np.float64)
    g1, g2 = u1 @ Jp, u2 @ Jp
    th = np.linspace(0.0, np.pi, int(n_samples), endpoint=False)   # ±e share a value
    best, best_e = -1.0, None
    for c, s in zip(np.cos(th), np.sin(th)):
        f = retreat_speed_available(c * g1 + s * g2, qdot_max, qdot_min)
        if f > best + 1e-12:
            best, best_e = f, c * u1 + s * u2
    if best_e is None or best <= 1e-9:
        return None
    e = best_e / np.linalg.norm(best_e)
    d_perp = d - float(d @ v_hat) * v_hat
    if float(np.linalg.norm(d_perp)) > 1e-9:
        if float(e @ d_perp) < 0.0:
            e = -e
    elif float(e[int(np.argmax(np.abs(e)))]) < 0.0:
        e = -e
    return e


def escape_direction(
    n_hat: np.ndarray,
    v_obs_vec: np.ndarray,
    Jp: np.ndarray,
    qdot_max: np.ndarray,
    d_vec: np.ndarray,
    *,
    margin: float,
    ramp_start: float,
    v_min: float = 0.15,
    qdot_min: Optional[np.ndarray] = None,
    v_avail_floor: float = 0.0,
    v_close: Optional[float] = None,
) -> Tuple[np.ndarray, float, float]:
    """``(ê_escape, w, r)`` for one control point.

    ``v_close`` overrides the closing speed taken from ``v_obs_vec``. The
    caller passes the CONDITIONED estimate — the one the barrier and the
    retreat cap consume, after the deadband and the median — so the trigger
    cannot fire on a number the rest of the filter has already decided is
    noise. Without it this read the raw track vector and swerved the arm on a
    static obstacle while ``v_obs`` reaching the QP was exactly zero. The
    VECTOR is still used, for the direction: a lateral escape needs one, and
    only the track has it.

    ``ê_escape = normalise((1 − w)·n̂ + w·ê_lat)`` with ``w = blend_weight(r)``
    and ``r = outrun_ratio(n̂ᵀv_obs, v_avail)``. Continuous in everything: as
    the closing speed rises through the ramp the direction rotates smoothly
    from the normal retreat to the lateral escape. When no lateral direction
    exists (obstacle not moving, or no leverage) the answer is ``(n̂, 0, r)`` —
    today's behaviour, with the ratio still reported for diagnostics.
    """
    n = np.asarray(n_hat, dtype=np.float64).ravel()
    n = n / max(float(np.linalg.norm(n)), 1e-12)
    a = n @ np.asarray(Jp, dtype=np.float64)
    v = np.asarray(v_obs_vec, dtype=np.float64).ravel()
    v_c = float(n @ v) if v_close is None else float(v_close)
    r = outrun_ratio(v_c, retreat_speed_available(a, qdot_max, qdot_min),
                     margin=margin, v_avail_floor=v_avail_floor)
    w = blend_weight(r, ramp_start=ramp_start)
    if w <= 0.0:
        return n, 0.0, r
    e_lat = lateral_direction(v, Jp, qdot_max, d_vec, v_min=v_min, qdot_min=qdot_min)
    if e_lat is None:
        return n, 0.0, r
    e = (1.0 - w) * n + w * e_lat
    ne = float(np.linalg.norm(e))
    if ne < 1e-9:                       # n̂ and ê_lat exactly opposed at w = ½
        return e_lat, w, r
    return e / ne, w, r


def evasion_bias(
    e_escape: np.ndarray,
    Jp: np.ndarray,
    v_close: float,
    w: float,
    *,
    gain: float,
    accel: float,
    v_ref: float,
    max_bias: float,
    damping: float = 0.05,
) -> np.ndarray:
    """(nv,) acceleration bias to ADD to q̈_nom, or an exact zero vector.

        q̈_bias = gain · J_p⁺ · (accel · m · w · ê_escape),   m = clip(v_close/v_ref, 0, 1)

    ``m`` makes the bias vanish continuously as the obstacle stops closing,
    so a zero or receding v_obs yields EXACTLY zeros and the path is
    bit-identical to the flag being off. ``w`` makes it vanish while the
    obstacle is outrunnable: there the barrier's own retreat is the whole
    response and nothing is added on top of it (no fight with the retreat
    cap). ``J_p⁺`` is the damped pseudo-inverse ``J_pᵀ(J_pJ_pᵀ + λ²I)⁻¹`` so a
    near-singular point cannot turn a modest Cartesian request into an
    unbounded joint command; ``max_bias`` caps the norm on top of that.
    """
    m = min(max(float(v_close) / float(v_ref), 0.0), 1.0) if v_ref > 0.0 else (1.0 if v_close > 0 else 0.0)
    Jp = np.asarray(Jp, dtype=np.float64)
    nv = Jp.shape[1]
    if m <= 0.0 or w <= 0.0 or gain <= 0.0 or accel <= 0.0:
        return np.zeros(nv)
    e = np.asarray(e_escape, dtype=np.float64).ravel()
    JJt = Jp @ Jp.T + (float(damping) ** 2) * np.eye(3)
    q_bias = float(gain) * (Jp.T @ np.linalg.solve(JJt, float(accel) * m * float(w) * e))
    nb = float(np.linalg.norm(q_bias))
    if max_bias > 0.0 and nb > max_bias:
        q_bias *= max_bias / nb
    return q_bias
