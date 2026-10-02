"""Singularity recovery: when the task is stuck in a singularity, go home in joint space.

WHY
---
Near a singularity the Cartesian task cannot be executed: σ_min(J) → 0 turns a
bounded Cartesian demand into an unbounded joint demand, the damped
pseudo-inverse (and the state governor / CBF singularity row) throttle it, and
the arm ends up with a growing task error and no motion. Nothing in the
Cartesian loop can leave that state, because the Jacobian is exactly the thing
that has lost the direction. A joint-space move has no Jacobian to invert, so
it is the one command that is still well posed there.

WHAT
----
A small state machine, pure numpy and clock-agnostic (the caller passes ``t``):

    IDLE ──stuck for stall_s──▶ RETURNING ──arrived──▶ SETTLING ──settle_s──▶ COOLDOWN ──▶ IDLE
                                    (quintic q → q_home)           (task resumes here)

* **Stuck** = σ_min below ``sigma_thr`` **and** the end-effector (almost) not
  moving **and** a real task error, all continuously for ``stall_s``. σ_min
  alone is not enough (a path may graze a singularity while moving); the three
  together are the "cannot get out" signature.
* **Return** is a quintic in joint space from the measured (q, q̇) to
  ``q_home`` with zero velocity/acceleration at the end. Its duration is the
  shortest one that keeps the sampled peak velocity under ``vmax`` and the peak
  acceleration under ``amax``.
* **Resume** is automatic: after ``settle_s`` holding ``q_home`` the machine
  reports a ``'resumed'`` event and the caller restarts its task from there.
  ``cooldown_s`` keeps the detector off long enough for the task to leave the
  neighbourhood, so a home that is itself near a singularity cannot trigger a
  loop of back-to-back recoveries.

``update`` returns the joint-space reference ``(q_d, dq_d, ddq_d)`` while the
recovery owns the arm and ``None`` while the task does.
"""

from __future__ import annotations

from typing import Optional, Tuple

import numpy as np


def quintic_coeffs(q0, v0, q1, T):
    """Coefficients c[0..5] (per joint) of q(τ)=Σ c_k τ^k with
    q(0)=q0, q'(0)=v0, q''(0)=0, q(T)=q1, q'(T)=q''(T)=0."""
    q0 = np.asarray(q0, dtype=np.float64)
    v0 = np.asarray(v0, dtype=np.float64)
    q1 = np.asarray(q1, dtype=np.float64)
    d = q1 - q0 - v0 * T
    c = np.zeros((6,) + q0.shape)
    c[0], c[1] = q0, v0
    c[3] = 10.0 * d / T ** 3
    c[4] = -15.0 * d / T ** 4
    c[5] = 6.0 * d / T ** 5
    return c


def quintic_eval(c, tau):
    """(q, dq, ddq) of the quintic ``c`` at time ``tau`` (clamped to ≥ 0)."""
    tau = max(float(tau), 0.0)
    q = (((((c[5] * tau + c[4]) * tau + c[3]) * tau + c[2]) * tau + c[1]) * tau
         + c[0])
    dq = ((((5.0 * c[5] * tau + 4.0 * c[4]) * tau + 3.0 * c[3]) * tau
           + 2.0 * c[2]) * tau + c[1])
    ddq = (((20.0 * c[5] * tau + 12.0 * c[4]) * tau + 6.0 * c[3]) * tau
           + 2.0 * c[2])
    return q, dq, ddq


def plan_duration(q0, v0, q1, vmax, amax, t_min=1.0, samples=200):
    """Shortest T ≥ t_min whose quintic respects ``vmax`` / ``amax`` per joint.

    ``vmax`` / ``amax`` are scalars or per-joint arrays. Sampled, not
    analytic, because a non-zero v0 moves the peak; T grows 10 % at a time.
    """
    q0 = np.asarray(q0, dtype=np.float64)
    q1 = np.asarray(q1, dtype=np.float64)
    v0 = np.asarray(v0, dtype=np.float64)
    dmax = float(np.max(np.abs(q1 - q0)))
    # quintic rest-to-rest peaks: 1.875·Δ/T and 5.77·Δ/T²
    T = max(t_min, 1.875 * dmax / float(np.min(vmax)) if dmax > 0 else t_min,
            np.sqrt(5.78 * dmax / float(np.min(amax))) if dmax > 0 else t_min)
    for _ in range(60):
        c = quintic_coeffs(q0, v0, q1, T)
        ok = True
        for tau in np.linspace(0.0, T, samples):
            _, dq, ddq = quintic_eval(c, tau)
            if np.any(np.abs(dq) > vmax) or np.any(np.abs(ddq) > amax):
                ok = False
                break
        if ok:
            return T
        T *= 1.1
    return T


class SingularityRecovery:
    IDLE, RETURNING, SETTLING, COOLDOWN = 'idle', 'returning', 'settling', 'cooldown'

    def __init__(self, *, sigma_thr: float = 0.07, speed_eps: float = 0.01,
                 err_thr: float = 0.02, stall_s: float = 1.5,
                 vmax=0.5, amax=2.0, t_min: float = 1.0,
                 settle_s: float = 0.5, cooldown_s: float = 5.0):
        if stall_s <= 0.0 or t_min <= 0.0:
            raise ValueError(f'stall_s={stall_s}, t_min={t_min} must be > 0')
        self.sigma_thr = float(sigma_thr)
        self.speed_eps = float(speed_eps)
        self.err_thr = float(err_thr)
        self.stall_s = float(stall_s)
        self.vmax = np.asarray(vmax, dtype=np.float64)
        self.amax = np.asarray(amax, dtype=np.float64)
        self.t_min = float(t_min)
        self.settle_s = float(settle_s)
        self.cooldown_s = float(cooldown_s)
        self.state = self.IDLE
        self.n_recoveries = 0
        self._stuck_since: Optional[float] = None
        self._t_start = 0.0
        self._T = 0.0
        self._c = None
        self._q_home = None
        self._t_phase = 0.0          # start of SETTLING / COOLDOWN
        self._event: Optional[str] = None

    # ── queries ────────────────────────────────────────────────────────────
    @property
    def active(self) -> bool:
        return self.state in (self.RETURNING, self.SETTLING)

    @property
    def duration(self) -> float:
        return self._T

    def pop_event(self) -> Optional[str]:
        """'started' | 'arrived' | 'resumed' once per transition, else None."""
        e, self._event = self._event, None
        return e

    def reset(self) -> None:
        self.state = self.IDLE
        self._stuck_since = None
        self._c = None

    # ── detection ──────────────────────────────────────────────────────────
    def is_stuck(self, sigma_min, ee_speed, cart_err) -> bool:
        return (np.isfinite(sigma_min) and sigma_min < self.sigma_thr
                and ee_speed < self.speed_eps and cart_err > self.err_thr)

    # ── the tick ───────────────────────────────────────────────────────────
    def update(self, t: float, q, qdot, q_home, sigma_min: float,
               ee_speed: float, cart_err: float
               ) -> Optional[Tuple[np.ndarray, np.ndarray, np.ndarray]]:
        if self.state == self.IDLE:
            if self.is_stuck(sigma_min, ee_speed, cart_err):
                if self._stuck_since is None:
                    self._stuck_since = t
                if t - self._stuck_since >= self.stall_s:
                    self._begin(t, q, qdot, q_home)
            else:
                self._stuck_since = None
        elif self.state == self.COOLDOWN:
            if t - self._t_phase >= self.cooldown_s:
                self.state = self.IDLE
                self._stuck_since = None

        if self.state == self.RETURNING:
            tau = t - self._t_start
            if tau >= self._T:
                self.state = self.SETTLING
                self._t_phase = t
                self._event = 'arrived'
            else:
                return quintic_eval(self._c, tau)
        if self.state == self.SETTLING:
            if t - self._t_phase >= self.settle_s:
                self.state = self.COOLDOWN
                self._t_phase = t
                self._event = 'resumed'
                return None
            z = np.zeros_like(self._q_home)
            return self._q_home.copy(), z, z.copy()
        return None

    def _begin(self, t, q, qdot, q_home) -> None:
        self._q_home = np.array(q_home, dtype=np.float64)
        q0 = np.array(q, dtype=np.float64)
        v0 = np.array(qdot, dtype=np.float64)
        self._T = plan_duration(q0, v0, self._q_home, self.vmax, self.amax,
                                self.t_min)
        self._c = quintic_coeffs(q0, v0, self._q_home, self._T)
        self._t_start = t
        self.state = self.RETURNING
        self.n_recoveries += 1
        self._stuck_since = None
        self._event = 'started'
