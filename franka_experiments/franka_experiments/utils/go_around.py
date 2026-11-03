"""Go-around: slide the closest control point ALONG the obstacle instead of only backing off.

Why this exists
---------------
The QP ``min ‖q̈ − q̈_target‖`` under one active row ``aᵀq̈ ≥ b`` returns the
projection of the target onto the row's hyperplane: the correction is purely
along ``a = n̂ᵀJp``, i.e. straight away from the obstacle. Measured in the DIAG
Gazebo scene, 72-98 % of the path deviation near an obstacle pointed along n̂
whatever the barrier margins were. ``cbf_qp_assembly.tangential_bias`` does not
fix that: it amplifies the joint-space component of q̈_nom orthogonal to ``a``,
which vanishes when the task heads straight at the obstacle.

This module works in CARTESIAN space at the closest control point. Everything
in it is built to be SMOOTH: a first version (on/off speed loop, side re-chosen
every tick) made the hand swing ±0.2 m/s along the ball at ~2 Hz.

Side (``GoAroundState.t``)
    Chosen ONCE, when the barrier enters ``engage_margin``: the tangential part
    of the task's intent ``v + T·Jp q̈_nom`` or, head-on, the tangent the arm
    can move fastest in. Afterwards it is only re-projected on the current
    tangent plane (n̂ turns as the arm wraps around the ball) and may TURN
    toward the intent with time constant ``turn_tau`` — never flip: an intent
    on the opposite side is ignored. Forgotten only once h̄ is past
    ``release_factor·engage_margin``, so a barrier chattering at the engage
    boundary does not re-roll the side.

Push
    ``a_t = w·clip(k·(v_ref − t̂ᵀv), −a_max, a_max)``: a plain proportional
    speed loop, two-sided, low gain — no bang-bang. ``w`` is a smoothstep of
    the barrier depth, itself rate-limited.

Rate limit
    The returned joint bias changes by at most ``max_jerk·dt`` per tick.

Task push-in cut (``inward_cut``)
    The part of the TASK acceleration that drives the control point INTO the
    obstacle (n̂ᵀJp q̈_nom < 0) is removed in proportion to ``w·push_cut``, so
    the task stops pressing the arm against the barrier while the go-around
    slides it past. Tangential task motion is untouched.

Like every other bias in the filter this only moves the QP TARGET: no row is
changed, so it can never loosen a constraint; a wrong direction costs tracking.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional

import numpy as np


def _smoothstep(x: float) -> float:
    x = min(max(x, 0.0), 1.0)
    return x * x * (3.0 - 2.0 * x)


def _dls(Jp: np.ndarray, x: np.ndarray, damping: float) -> np.ndarray:
    """Damped least-squares joint acceleration producing Cartesian ``x``."""
    JJt = Jp @ Jp.T + (damping ** 2) * np.eye(3)
    return Jp.T @ np.linalg.solve(JJt, x)


@dataclass
class GoAroundState:
    t: Optional[np.ndarray] = None       # Cartesian side kept (unit, 3)
    w: float = 0.0                       # rate-limited engagement weight
    bias: Optional[np.ndarray] = None    # last joint bias returned (nv)
    engaged: bool = False
    extra: dict = field(default_factory=dict)


def _pick_side(intent_t: np.ndarray, P: np.ndarray, Jp: np.ndarray, v: np.ndarray,
               t_min: float):
    nt = float(np.linalg.norm(intent_t))
    if nt >= t_min:
        return intent_t / nt, 0
    U, _, _ = np.linalg.svd(P @ Jp)
    u = P @ U[:, 0]
    nu = float(np.linalg.norm(u))
    if nu < 1e-9:
        return None, 1
    u /= nu
    return (u if float(u @ (P @ v)) >= 0.0 else -u), 1


def go_around_step(state: GoAroundState, qddot_nom: np.ndarray, qdot: np.ndarray,
                   Jp: Optional[np.ndarray], n_hat: Optional[np.ndarray], h: float,
                   dt: float, *, engage_margin: float, speed: float, gain: float,
                   max_accel: float, max_bias: float, horizon_s: float, damping: float,
                   turn_tau: float = 1.0, release_factor: float = 1.5,
                   max_jerk: float = 10.0, w_rate: float = 4.0, push_cut: float = 1.0,
                   t_min: float = 0.03, diag: Optional[dict] = None):
    """Advance the go-around one tick. Returns ``(bias, inward_cut)``: two (nv,)
    joint accelerations to ADD to q̈_nom. ``Jp``/``n_hat`` None = no geometry
    (blind or nothing nearby): everything fades out at the rate limits."""
    nv = qddot_nom.shape[0]
    if state.bias is None:
        state.bias = np.zeros(nv)
    if diag is not None:
        diag.update(w=0.0, a=0.0, vt=0.0, head_on=0)
    geom = (Jp is not None and n_hat is not None and np.isfinite(h)
            and engage_margin > 0.0)

    # engagement weight: smoothstep of depth, then rate limited (both ways)
    w_target = _smoothstep((engage_margin - h) / engage_margin) if geom else 0.0
    dw = float(np.clip(w_target - state.w, -w_rate * dt, w_rate * dt))
    state.w = float(np.clip(state.w + dw, 0.0, 1.0))
    if geom and h < engage_margin:
        state.engaged = True
    elif (not geom) or h > release_factor * engage_margin:
        state.engaged = False
        state.t = None

    raw = np.zeros(nv)
    cut = np.zeros(nv)
    if geom and state.engaged and state.w > 0.0:
        n = np.asarray(n_hat, dtype=np.float64)
        n = n / max(float(np.linalg.norm(n)), 1e-12)
        P = np.eye(3) - np.outer(n, n)
        v = Jp @ qdot
        a_task = Jp @ qddot_nom
        intent_t = P @ (v + horizon_s * a_task)
        head = 0
        tp = None
        if state.t is not None:
            tp = P @ state.t
            ntp = float(np.linalg.norm(tp))
            tp = tp / ntp if ntp > 1e-6 else None
        if tp is None:
            tp, head = _pick_side(intent_t, P, Jp, v, t_min)
        else:
            nt = float(np.linalg.norm(intent_t))
            if nt >= t_min and float(intent_t @ tp) > 0.0:   # same side: may turn
                beta = min(dt / max(turn_tau, 1e-6), 1.0)
                tp = (1.0 - beta) * tp + beta * (intent_t / nt)
                tp = P @ tp
                tp /= max(float(np.linalg.norm(tp)), 1e-12)
        state.t = tp
        if tp is not None:
            vt = float(tp @ v)
            a_t = state.w * float(np.clip(gain * (speed - vt), -max_accel, max_accel))
            raw = _dls(Jp, a_t * tp, damping)
            nb = float(np.linalg.norm(raw))
            if max_bias > 0.0 and nb > max_bias:
                raw *= max_bias / nb
            if diag is not None:
                diag.update(w=state.w, a=a_t, vt=vt, head_on=head)
        # stop the task pressing INTO the obstacle (only the inward part)
        a_in = float(n @ a_task)
        if a_in < 0.0 and push_cut > 0.0:
            cut = _dls(Jp, (-state.w * push_cut * a_in) * n, damping)

    # rate limit the bias (jerk bound)
    d = raw - state.bias
    nd = float(np.linalg.norm(d))
    lim = max_jerk * dt
    if nd > lim > 0.0:
        d *= lim / nd
    state.bias = state.bias + d
    return state.bias.copy(), cut
