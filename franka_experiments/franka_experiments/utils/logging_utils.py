"""Rate-limited logging, formatting, and lightweight profiling.

OWNS
----
Observability helpers that any node may use:
``ThrottledLogger`` (rate-limited log lines), ``vec_to_str``
(compact vector formatting) and ``PerfTimer`` (named-stage
wall-clock profiler, moved here from ``utils.node_utils`` in Phase 2).

DOES NOT OWN
------------
Anything domain-specific: no distance, no CBF, no message construction.
"""

from __future__ import annotations

from typing import Optional

import numpy as np


class ThrottledLogger:
    """Rate-limited logging wrapper.

    Usage::

        self._tlog = ThrottledLogger(self.get_logger(), period_s=1.0)

        # inside callback:
        if self._tlog.due(t):
            self._tlog.info(f'[t={t:.1f}s] some message')
    """

    def __init__(self, logger, *, period_s: float = 1.0) -> None:
        self._logger = logger
        self._period = period_s
        self._last_t: float = -period_s  # ensures first call fires

    def due(self, t: float) -> bool:
        """Return ``True`` (and update timestamp) if at least *period_s* elapsed."""
        if t - self._last_t >= self._period:
            self._last_t = t
            return True
        return False

    def info(self, msg: str) -> None:
        """Log at INFO level (unconditionally — gate with :meth:`due`)."""
        self._logger.info(msg)

    def debug(self, msg: str) -> None:
        """Log at DEBUG level (unconditionally — gate with :meth:`due`)."""
        self._logger.debug(msg)

    def warn(self, msg: str) -> None:
        """Log at WARN level (unconditionally — gate with :meth:`due`)."""
        self._logger.warn(msg)

    @property
    def last_t(self) -> float:
        return self._last_t

    @last_t.setter
    def last_t(self, value: float) -> None:
        self._last_t = value


def vec_to_str(v: Optional[np.ndarray], fmt: str = '.4f') -> str:
    """Format a numpy vector as a comma-separated string, or ``'?'``."""
    if v is None:
        return '?'
    return ', '.join(f'{x:{fmt}}' for x in v)


# MOVED here from utils/node_utils.py (Phase 2): a wall-clock profiler is
# observability, not perception-message construction.
import time  # noqa: E402


class PerfTimer:
    """Accumulates wall-clock timings for a set of named stages.

    Usage::

        perf = PerfTimer()
        with perf('tf'):
            ...
        print(perf.summary())   # 'tf=1.2ms  mask=3.4ms'
    """

    def __init__(self):
        self._ms: dict[str, float] = {}

    def __call__(self, key: str) -> '_TimerCtx':
        return _TimerCtx(self._ms, key)

    def summary(self) -> str:
        return '  '.join(f'{k}={v:.1f}ms' for k, v in self._ms.items())


class _TimerCtx:
    """Context manager returned by PerfTimer.__call__."""

    def __init__(self, store: dict, key: str):
        self._s = store
        self._k = key

    def __enter__(self):
        self._t0 = time.perf_counter()
        return self

    def __exit__(self, *_):
        self._s[self._k] = (time.perf_counter() - self._t0) * 1000.0


def format_velocity_summary(
    qdot: np.ndarray,
    ratio: np.ndarray,
    bite: np.ndarray,
) -> str:
    """Format a compact per-joint velocity-saturation summary for one log line.

    MOVED here from ``CBFSafetyFilter._fmt_vel`` in Phase 3; body unchanged, the
    two diagnostic arrays it used to read off ``self`` are now arguments.

    Args:
        qdot: (n,) measured joint velocities [rad/s].
        ratio: (n,) ``|qdot| / qdot_max`` per joint.
        bite: (n,) boolean mask — True where the velocity bound tightened that
            joint's acceleration box this tick.

    Returns:
        A single line: the worst joint (signed qdot and its ratio), all ratios
        in joint order, and an n-character bite mask (``X`` = biting).
    """
    k     = int(np.argmax(ratio))
    rats  = '/'.join(f'{r:.2f}' for r in ratio)
    mask  = ''.join('X' if b else '.' for b in bite)
    return (f'worst=j{k+1}:q̇={qdot[k]:+.2f}({ratio[k]:.2f}) '
            f'vrat=[{rats}] vbite={mask}')
