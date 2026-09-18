"""Livelock detector: the task is pushing, the arm is not moving, for too long.

WHY THIS EXISTS
---------------
A CBF filter can be stuck without any row being violated and without any
solver failing. The commander asks for a motion straight into a barrier, the QP
removes the component along the barrier's normal, and if that was the WHOLE
command nothing is left: q̈_safe ≈ 0, q̇ ≈ 0, the reference stays parked, and
next tick the same thing happens. Every quantity the filter reports looks
healthy — the barrier is satisfied, slack is zero, the solve converged — and
the arm stands still for as long as the obstacle does. That is a local minimum
of the (nominal, barrier) pair, and no gain fixes it because there is nothing
wrong with either half.

The way out is a nudge in a direction the barrier does not care about: the
nullspace of the active rows, i.e. tangential motion around the obstacle. The
direction is built elsewhere (``ConstraintBuilder`` puts a nullspace-projected
lateral escape in the snapshot, reusing Phase 2's ``lateral_direction``); this
module decides WHEN and HOW MUCH, and it is deliberately dumb about the
robot: it sees two booleans per tick.

THE RULE
--------
    blocked  = the QP is bending the nominal (‖q̈_safe − q̈_nom‖ > dnorm_thr)
    moving   = the joints have DISPLACED by more than progress_thr over the
               last progress_window_s (see :class:`ProgressWindow`)

"moving" is a displacement over a window, NOT an instantaneous speed, on
purpose: measured in the wedge scenario, an arm pinned between two obstacles
jiggles at 0.1-0.6 rad/s while going nowhere — the barrier, the cap and the
nominal trade blows every tick — and a speed threshold never sees it as
stuck. Displacement over half a second does.

    blocked and not moving, continuously for > stall_s   →  ESCAPE starts
    escape ramps in over ramp_s, is held at most max_s,
    ends early the moment `blocked` clears (the QP no longer has to bend
    the nominal: the arm is free to go where the task wants),
    and cannot restart for cooldown_s afterwards.

The escape does NOT end on `moving` alone. Measured in the wedge scenario:
the nudge itself produces displacement within a few ticks, which read as
"progress resumed" and ended the escape after 0.3 s — before it had carried
the arm anywhere — and the task pulled it straight back in. Progress that
counts is the task getting its way again, i.e. `blocked` going false.

Every bound is explicit: a maximum magnitude (the caller's gain), a maximum
duration, a cooldown. The nudge is a bias on the objective, so it cannot cross a
barrier; the worst case is a bounded, logged sideways drift that did not help.

Pure Python, no ROS.
"""

from __future__ import annotations

from collections import deque
from typing import Optional

import numpy as np


class ProgressWindow:
    """Joint displacement over the last ``window_s`` seconds.

    ``push(t, q)`` then ``displacement()`` = ‖q(t) − q(t − window)‖. Until the
    window is full it reports the displacement since the first sample, which
    can only be an over-estimate of progress — so a fresh detector errs on
    the side of "moving" and never fires on startup.
    """

    def __init__(self, window_s: float) -> None:
        self.window_s = float(window_s)
        self._buf: deque = deque()

    def push(self, t: float, q: np.ndarray) -> float:
        q = np.array(q, dtype=np.float64, copy=True)
        self._buf.append((float(t), q))
        while len(self._buf) > 1 and t - self._buf[0][0] > self.window_s:
            self._buf.popleft()
        return float(np.linalg.norm(q - self._buf[0][1]))

    def reset(self) -> None:
        self._buf.clear()


class LivelockDetector:
    """Per-tick state machine. Call :meth:`update` once per QP tick.

    Args:
        stall_s: [s] how long "blocked and not moving" must persist.
        ramp_s: [s] the escape magnitude ramps 0 → 1 over this after triggering,
            so the nudge is not a step.
        max_s: [s] longest continuous escape; after it the detector rests.
        cooldown_s: [s] rest after an escape ends, whatever ended it.
    """

    IDLE, STALLING, ESCAPING, COOLDOWN, RELEASING = (
        'idle', 'stalling', 'escaping', 'cooldown', 'releasing')

    def __init__(self, *, stall_s: float, ramp_s: float, max_s: float,
                 cooldown_s: float) -> None:
        self.stall_s = float(stall_s)
        self.ramp_s = max(float(ramp_s), 1e-6)
        self.max_s = float(max_s)
        self.cooldown_s = float(cooldown_s)
        self.state = self.IDLE
        self._t0: Optional[float] = None       # when the current state began
        self._mag = 0.0                        # current escape magnitude
        self._rel = 0.0                        # magnitude the release started from
        self.n_escapes = 0                     # DIAGNOSTIC: escapes started
        self.last_reason = ''                  # DIAGNOSTIC: why the last one ended

    def update(self, t: float, *, blocked: bool, moving: bool) -> float:
        """Advance to time ``t``; return the escape magnitude in ``[0, 1]``."""
        t = float(t)
        if self.state == self.IDLE:
            if blocked and not moving:
                self.state, self._t0 = self.STALLING, t
            return 0.0
        if self.state == self.STALLING:
            if moving or not blocked:
                self.state, self._t0 = self.IDLE, None
                return 0.0
            if t - self._t0 >= self.stall_s:
                self.state, self._t0 = self.ESCAPING, t
                self.n_escapes += 1
                return 0.0
            return 0.0
        if self.state == self.ESCAPING:
            el = t - self._t0
            if not blocked:
                self._end(t, 'task no longer blocked')
                return self._mag
            if el >= self.max_s:
                self._end(t, 'max duration')
                return self._mag
            self._mag = min(el / self.ramp_s, 1.0)
            return self._mag
        if self.state == self.RELEASING:
            # Ramp OUT over the same ramp_s. Ending an escape by dropping the
            # magnitude to zero in one tick is a step of the full gain in
            # q̈_nom — the arm feels it exactly as it feels the start, and a
            # bias that ends abruptly is no smoother than one that starts
            # abruptly.
            self._mag = max(self._rel - (t - self._t0) / self.ramp_s, 0.0)
            if self._mag <= 0.0:
                self.state, self._t0, self._mag = self.COOLDOWN, t, 0.0
            return self._mag
        # COOLDOWN
        if t - self._t0 >= self.cooldown_s:
            self.state, self._t0 = self.IDLE, None
        return 0.0

    def _end(self, t: float, reason: str) -> None:
        self.last_reason = reason
        self.state, self._t0, self._rel = self.RELEASING, t, self._mag

    @property
    def escaping(self) -> bool:
        return self.state in (self.ESCAPING, self.RELEASING)

    def reset(self) -> None:
        self.state, self._t0 = self.IDLE, None
        self._mag = self._rel = 0.0
