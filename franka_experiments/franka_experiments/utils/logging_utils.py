"""Rate-limited logging helper and formatting utilities."""

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
