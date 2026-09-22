"""Durations, not frame counts: perception tuning that survives a rate change.

WHY THIS EXISTS
---------------
Half the perception tuning in ``fr3_complete.yaml`` used to be counted in
FRAMES — ``max_missed: 5``, ``confirm_hits: 3`` of ``confirm_window: 5``, the
self-detection ``confirm``/``release``, the distance EMA's ``lpf_alpha``. Every
one of those numbers was chosen against the 30 Hz the depth stream happened to
run at, and every comment that justifies one says so in SECONDS: "0.5 m is what
a limb covers in the ~0.17 s a 5-frame coast lasts".

So the moment the depth profile changes, the tuning changes with it and nothing
says a word. At 90 fps the same numbers mean a 0.055 s coast, a 0.055 s birth
and three times less smoothing on the distance the CBF consumes — the arm
behaves differently and the config still reads as if nothing happened. The
failure has a precedent on this rig in the other direction, and it is written
down in ``launch_defaults.yaml``: with no profile pinned, librealsense picked
15 fps by itself, which silently DOUBLED every one of those durations.

WHAT THIS DOES
--------------
The config states durations; the frame counts are derived from the rate the
depth stream is actually delivering. :func:`frames_for` is that conversion, and
:class:`FrameRateEstimator` is the measurement behind it — because the nominal
rate is a claim about the camera, and a camera that has fallen back to 15 fps
makes that claim false. The estimator is the reason a wrong ``depth_rate_hz``
costs a log line instead of a silent retune of the safety layer.

WHY A MEASUREMENT AND NOT JUST THE CONFIGURED RATE
--------------------------------------------------
Both, in this order: the configured rate sizes the counters before the first
frame arrives (a tracker cannot wait for a rate estimate to have an opinion
about its first cluster), and the measurement corrects them once it has settled
— it is the only one of the two that knows about a driver fallback, a USB link
that cannot carry the profile, or a loaded box dropping every other frame.
"""

from __future__ import annotations

import math
from collections import deque
from statistics import median
from typing import Deque, Optional


def frames_for(seconds: float, rate_hz: float, *, minimum: int = 1) -> int:
    """How many frames ``seconds`` lasts at ``rate_hz``, at least *minimum*.

    Rounds to nearest rather than truncating: at 30 Hz a 0.167 s window is
    5.01 frames, and ``int()`` would make it 5 by luck and 4 for a 0.166 s
    config. The floor exists because zero frames of history, of coast or of
    confirmation are all degenerate — a counter that trips on nothing.
    """
    if not (rate_hz > 0.0) or not math.isfinite(rate_hz):
        raise ValueError(f'rate_hz must be finite and positive, got {rate_hz}')
    if not math.isfinite(seconds) or seconds < 0.0:
        raise ValueError(f'seconds must be finite and >= 0, got {seconds}')
    return max(int(minimum), int(round(float(seconds) * float(rate_hz))))


def ema_alpha_for(tau_s: float, dt: float) -> float:
    """EMA weight on the PREVIOUS value for time constant ``tau_s`` over ``dt``.

    ``alpha = exp(-dt / tau)``, the exact discretisation of a first-order lag,
    which is what makes the filter's bandwidth a property of the tuning instead
    of a property of the frame rate. The engine's convention is
    ``new = alpha * prev + (1 - alpha) * measurement``, so alpha is the weight
    of the HISTORY: alpha → 1 is heavy smoothing, alpha → 0 is none.

    ``tau_s <= 0`` returns 0.0 (no smoothing). A non-positive or unusable ``dt``
    is the caller's problem — it has the frame stamps and knows whether they are
    trustworthy — and returns 0.0 as well rather than inventing an interval.
    """
    if not (tau_s > 0.0) or not math.isfinite(tau_s):
        return 0.0
    if dt is None or not math.isfinite(dt) or dt <= 0.0:
        return 0.0
    return float(math.exp(-float(dt) / float(tau_s)))


def tau_for_alpha(alpha: float, rate_hz: float) -> float:
    """The time constant a legacy per-frame ``alpha`` meant at ``rate_hz``.

    The migration helper for the numbers already in the configs:
    ``lpf_alpha: 0.5`` at 30 Hz was a 48 ms lag, and that is the value to write
    down as a duration. Only useful for reading old tuning; nothing in the live
    path calls it.
    """
    if not (0.0 < alpha < 1.0):
        raise ValueError(f'alpha must be in (0, 1), got {alpha}')
    if not (rate_hz > 0.0):
        raise ValueError(f'rate_hz must be positive, got {rate_hz}')
    return -1.0 / (float(rate_hz) * math.log(float(alpha)))


class FrameRateEstimator:
    """The rate frames are actually arriving at: a MEDIAN over a window.

    Args:
        nominal_hz: the configured rate, returned by :attr:`hz` until the window
            is full, so the counters it sizes are never built on a partial
            measurement.
        window: intervals kept. The statistic is their MEDIAN, not their mean,
            and that is the whole design: a mean is moved by a stall, and a
            stall is exactly what a busy box produces. MEASURED on this rig —
            an EMA of dt over a stack startup read 6.8 Hz off a handful of
            100-200 ms hitches and rescaled a 0.167 s coast to ONE frame, which
            is a tracker that confirms every speckle. A median needs half the
            window to be slow before it moves at all, and half a window of slow
            frames is not a hitch, it is the rate.
        tol: relative change that counts as MATERIAL, i.e. that makes
            :meth:`add` return True. 0.20, because the rate this measures is the
            rate frames are PROCESSED at, and when the compute loop is the
            bottleneck that number breathes: a dry run on a loaded box wandered
            between 49 and 57 Hz, which at a 10% band retuned the safety layer
            every hundred milliseconds. A real profile change (30 → 15,
            30 → 90) is a factor of two and clears 20% at once.
        min_interval: frames between two reports. Tolerance alone cannot stop a
            rate drifting steadily across the band, and a retune is not
            something to do twice a second.

    Not thread-safe, and it does not need to be: it is fed from the one compute
    loop that consumes the frames.
    """

    def __init__(self, *, nominal_hz: float = 30.0, window: int = 120,
                 tol: float = 0.20, min_interval: int = 300) -> None:
        if not (nominal_hz > 0.0):
            raise ValueError(f'nominal_hz must be positive, got {nominal_hz}')
        if int(window) < 3:
            raise ValueError(f'window must be >= 3, got {window}')
        self.nominal_hz = float(nominal_hz)
        self.window = int(window)
        self.tol = float(tol)
        self.min_interval = max(0, int(min_interval))
        self._dts: Deque[float] = deque(maxlen=self.window)
        self._n = 0
        #: Sample index of the last report, so the cooldown ticks with the
        #: frames rather than with the wall clock.
        self._last_report_n: Optional[int] = None
        #: The rate the last material change was reported at. Compared against,
        #: not recomputed: without it a rate drifting slowly across the
        #: tolerance would report a change on every single frame.
        self._reported_hz = float(nominal_hz)

    @property
    def n_samples(self) -> int:
        return self._n

    @property
    def settled(self) -> bool:
        """True once the window is full. A partial window is a claim, not data."""
        return len(self._dts) >= self.window

    @property
    def hz(self) -> float:
        """The best available rate: measured once settled, nominal before."""
        m = self.measured_hz
        return self.nominal_hz if m is None else m

    @property
    def measured_hz(self) -> Optional[float]:
        """The measurement alone, or None while the window is filling."""
        if not self.settled:
            return None
        med = median(self._dts)
        return None if med <= 0.0 else 1.0 / med

    def add(self, dt: float) -> bool:
        """Feed one inter-frame interval. True when the rate MATERIALLY changed.

        A True is the signal to re-derive the frame counts; it fires at most
        once per material change and no more often than ``min_interval`` frames.

        Implausible intervals are ignored rather than clamped: a 0 dt is a
        duplicate stamp and a 2 s one is a stall, and either one in the window
        would describe something that is not the frame rate.
        """
        if dt is None or not math.isfinite(dt) or not (1e-4 < dt < 1.0):
            return False
        self._dts.append(float(dt))
        self._n += 1
        hz = self.measured_hz
        if hz is None:
            return False
        if abs(hz - self._reported_hz) <= self.tol * self._reported_hz:
            return False
        if (self._last_report_n is not None
                and self._n - self._last_report_n < self.min_interval):
            return False
        self._reported_hz = hz
        self._last_report_n = self._n
        return True

    def reset(self) -> None:
        self._dts.clear()
        self._n = 0
        self._reported_hz = self.nominal_hz
        self._last_report_n = None
