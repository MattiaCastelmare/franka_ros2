"""Modular timing laws: map elapsed time to a trajectory phase ``s``.

A timing law owns the scalar phase ``s`` and its time derivatives.  Each call to
:meth:`TimingLaw.step` advances the internal clock by ``dt`` seconds and returns
the triple ``(s, s_dot, s_ddot)``.  The *geometric* path (see
:mod:`franka_experiments.utils.trajectory`) is then evaluated as a pure function
of these via the chain rule::

    p = P(s)
    v = P'(s) * s_dot
    a = P''(s) * s_dot**2 + P'(s) * s_ddot

This cleanly separates *where* the reference goes (the path) from *how fast* it
gets there (the timing law).  No state machine, no per-phase branching.

Conventions
-----------
* ``s``      — dimensionless phase; one cyclic loop of a path spans ``Δs = 1``.
* ``s_dot``  — ds/dt   [1/s].
* ``s_ddot`` — d²s/dt² [1/s²].
"""

from __future__ import annotations


class TimingLaw:
    """Base class for timing laws.

    Subclasses override :meth:`step`.  ``reset`` re-seeds the phase (e.g. when
    (re)starting a trajectory) and is also where derived state (clocks, current
    rate) must be cleared.
    """

    def __init__(self) -> None:
        self._s = 0.0
        self._s_dot = 0.0

    def reset(self, s0: float = 0.0) -> None:
        self._s = float(s0)
        self._s_dot = 0.0

    @property
    def s(self) -> float:
        return self._s

    def step(self, dt: float):
        """Advance the phase by *dt* seconds; return ``(s, s_dot, s_ddot)``."""
        raise NotImplementedError


class LinearTimingLaw(TimingLaw):
    """Constant-rate phase with an optional C² soft-start.

    The steady phase rate is ``rate`` [1/s] (so a one-loop path of period
    ``Δs = 1`` is traversed in ``1/rate`` seconds).  During the first
    ``soft_start_s`` seconds the rate is ramped ``0 → rate`` with a *smootherstep*
    profile, so the reference leaves rest with zero velocity *and* zero
    acceleration — the smoothing lives entirely here, not in a separate state.

    ``s`` increases monotonically; periodicity is applied by the path, so this
    law drives continuous cyclic tracking.
    """

    def __init__(self, rate: float, soft_start_s: float = 0.0) -> None:
        super().__init__()
        self._rate = float(rate)
        self._ramp = max(0.0, float(soft_start_s))
        self._t = 0.0

    def reset(self, s0: float = 0.0) -> None:
        super().reset(s0)
        self._t = 0.0

    def step(self, dt: float):
        self._t += dt
        if self._ramp > 0.0 and self._t < self._ramp:
            x = self._t / self._ramp
            # smootherstep g(x) = 6x⁵ − 15x⁴ + 10x³  (g(0)=g'(0)=g''(0)=0, g(1)=1)
            g = x * x * x * (x * (x * 6.0 - 15.0) + 10.0)
            gp = 30.0 * x * x * (x * (x - 2.0) + 1.0)        # g'(x)
            s_dot = self._rate * g
            s_ddot = self._rate * gp / self._ramp
        else:
            s_dot = self._rate
            s_ddot = 0.0
        self._s_dot = s_dot
        self._s += s_dot * dt
        return self._s, s_dot, s_ddot


class ExponentialTimingLaw(TimingLaw):
    """First-order ease toward ``s_target`` — ``s_dot = k (s_target − s)``.

    Mirrors the old Cartesian exponential reference filter, but in phase space:
    ``s`` approaches ``s_target`` asymptotically and *settles* (s_dot → 0), so
    this is a one-shot / point-to-point law.  It does **not** cycle — use
    :class:`LinearTimingLaw` or :class:`TrapezoidalTimingLaw` for the pentagon.
    """

    def __init__(self, k: float, s_target: float = 1.0) -> None:
        super().__init__()
        self._k = float(k)
        self._target = float(s_target)

    def step(self, dt: float):
        s_dot = self._k * (self._target - self._s)
        s_ddot = -self._k * s_dot
        self._s_dot = s_dot
        self._s += s_dot * dt
        return self._s, s_dot, s_ddot


class TrapezoidalTimingLaw(TimingLaw):
    """Constant-acceleration ramp to a cruise rate, then constant rate.

    Ramps ``s_dot`` from 0 up to ``rate`` at constant ``accel`` [1/s²], then
    holds it.  Like :class:`LinearTimingLaw` but with a piecewise-constant
    ``s_ddot`` (a velocity trapezoid's leading edge); cyclic-capable.
    """

    def __init__(self, rate: float, accel: float) -> None:
        super().__init__()
        self._rate = float(rate)
        self._accel = max(1e-9, float(accel))

    def step(self, dt: float):
        if self._s_dot < self._rate:
            s_ddot = self._accel
            s_dot = min(self._rate, self._s_dot + self._accel * dt)
        else:
            s_ddot = 0.0
            s_dot = self._rate
        self._s_dot = s_dot
        self._s += s_dot * dt
        return self._s, s_dot, s_ddot
