"""Collision monitoring and log collection, without ROS or controller dependencies.

This module coordinates the momentum observer and residual threshold detector;
their mathematical implementations remain in their own modules. Inputs are
measured joint states and the already aligned, gravity-compensated actuator
torque. No wall object, true contact force or tracking command is required.
"""

from dataclasses import dataclass

import numpy as np
import yaml

from fr3_sim.monitoring.collision_detector import CollisionDetector
from fr3_sim.monitoring.momentum_observer import GeneralizedMomentumObserver


@dataclass(frozen=True)
class ResidualSnapshot:
    """Read-only, timestamped estimate of the external joint torque.

    ``residual`` has the observer's sign: a positive component estimates an
    external torque acting positively on that joint. It is a filtered estimate,
    not the instantaneous torque at contact. ``filter_time_constant_s`` gives
    the nominal continuous-time time constant of each observer component.

    ``valid`` only describes initialization, sample continuity, freshness and
    finite numerical values. It is not a bound on model error or observer delay.
    A controller must explicitly check it before using ``residual``. Neither
    the returned arrays nor this snapshot can modify the monitor's state.
    """

    residual: np.ndarray
    timestamp: float | None
    age_s: float
    sample_interval_s: float | None
    valid: bool
    reason: str
    filter_time_constant_s: np.ndarray


def _readonly_copy(values):
    values = np.asarray(values, dtype=float).copy()
    values.flags.writeable = False
    return values


class CollisionMonitor:
    """Own monitoring configuration, observer/detector state and saved samples.

    Create one instance per run. Call ``update`` for every RUN measurement,
    before computing the next actuator command. It returns a detection message
    only on an onset event; logging the message remains the caller's job.
    """

    def __init__(self, robot, *, enabled=True, gain=30.0, threshold=0.1):
        self.enabled = bool(enabled)
        self.observer = GeneralizedMomentumObserver(robot, gain) if self.enabled else None
        self.detector = CollisionDetector(threshold) if self.enabled else None
        self._rows = []
        self._detection_callback = None
        self._sample_timing = []
        self._snapshot_gap_limit = None
        self._snapshot_checked_samples = 0
        self._snapshot_continuous_since = None
        self._snapshot_continuous_samples = 0

    def snapshot(self, t, *, warmup_s=0.2, max_gap_s=0.01, max_age_s=0.01):
        """Read the latest residual without advancing the observer or detector.

        ``t`` and all intervals use simulation seconds. A valid snapshot needs
        at least two measurements, ``warmup_s`` of continuous observations,
        no inter-sample gap above ``max_gap_s``, and age at most ``max_age_s``.
        A gap restarts the validity warmup; it does not reset the observer or
        alter detection/logging. The defaults can be overridden by the consuming
        controller. Querying later cannot complete warmup without new samples.

        The timestamps describe the measured states. They do not remove the
        observer's low-pass lag, actuator transport delay or model uncertainty.
        The actuator input has already been aligned by the caller of ``update``.
        No actual contact force is used, and detection thresholds do not gate
        this continuous estimate.
        """
        warmup_s, max_gap_s, max_age_s = (
            float(warmup_s), float(max_gap_s), float(max_age_s))
        if not np.isfinite(warmup_s) or warmup_s < 0.0:
            raise ValueError('snapshot warmup_s must be finite and nonnegative')
        if not np.isfinite(max_gap_s) or max_gap_s <= 0.0:
            raise ValueError('snapshot max_gap_s must be finite and positive')
        if not np.isfinite(max_age_s) or max_age_s < 0.0:
            raise ValueError('snapshot max_age_s must be finite and nonnegative')

        timestamp, interval = (self._sample_timing[-1]
                               if self._sample_timing else (None, None))
        t = float(t)
        age = t - timestamp if timestamp is not None else float('inf')
        residual = self.observer.residual if self.enabled else np.zeros(7)
        time_constant = (1.0 / self.observer.gain if self.enabled
                         else np.full(7, np.nan))

        # A controller normally keeps the same gap limit for a whole run.
        # Process only newly received samples; replay timing if that limit changes.
        if self._snapshot_gap_limit != max_gap_s:
            self._snapshot_gap_limit = max_gap_s
            self._snapshot_checked_samples = 0
            self._snapshot_continuous_since = None
            self._snapshot_continuous_samples = 0
        for sample_t, sample_dt in self._sample_timing[self._snapshot_checked_samples:]:
            if sample_dt is None or sample_dt > max_gap_s + 1e-12:
                self._snapshot_continuous_since = sample_t
                self._snapshot_continuous_samples = 1
            else:
                self._snapshot_continuous_samples += 1
        self._snapshot_checked_samples = len(self._sample_timing)

        if not self.enabled:
            reason = 'disabled'
        elif timestamp is None:
            reason = 'no_sample'
        elif not np.isfinite(t):
            reason = 'invalid_time'
        elif age < -1e-12:
            reason = 'future_sample'
        elif not (np.all(np.isfinite(residual))
                  and np.all(np.isfinite(self.observer.momentum))
                  and np.all(np.isfinite(self.observer.momentum_estimate))):
            reason = 'nonfinite'
        elif interval is not None and interval > max_gap_s + 1e-12:
            reason = 'sample_gap'
        elif self._snapshot_continuous_samples < 2:
            reason = 'initializing'
        elif timestamp - self._snapshot_continuous_since + 1e-12 < warmup_s:
            reason = 'warming_up'
        elif age > max_age_s + 1e-12:
            reason = 'stale'
        else:
            reason = 'ok'
        return ResidualSnapshot(
            residual=_readonly_copy(residual), timestamp=timestamp,
            age_s=age, sample_interval_s=interval, valid=reason == 'ok',
            reason=reason, filter_time_constant_s=_readonly_copy(time_constant))

    def set_detection_callback(self, callback):
        """Register one optional callable receiving ``t`` on each detection onset.

        Notification is synchronous with monitoring, independently of the
        controller update period. The callback should only record the event.
        """
        if callback is not None and not callable(callback):
            raise TypeError('detection callback must be callable or None')
        self._detection_callback = callback

    @classmethod
    def from_yaml(cls, robot, path, *, enabled=True):
        """Load the existing observer YAML, preserving its gain/threshold defaults."""
        config = {}
        if path:
            with open(path) as stream:
                config = yaml.safe_load(stream) or {}
        return cls(robot, enabled=enabled, gain=config.get('gain', 30.0),
                   threshold=config.get('threshold', 0.1))

    def update(self, t, q, dq, tau_applied):
        """Observe one sample and return an optional collision-onset message.

        ``tau_applied`` is the mean actuator torque over the measurement
        interval, after saturation and transport-delay alignment. The caller
        excludes wall torque; this module adds neither gravity nor a delay.
        A disabled monitor does nothing and returns ``None``.
        """
        if not self.enabled:
            return None
        previous_t = self.observer.time
        residual = self.observer.update(t, q, dq, tau_applied)
        self._sample_timing.append((float(t),
                                   None if previous_t is None else float(t) - previous_t))
        detected, event = self.detector.update(residual)
        residual_norm = float(np.linalg.norm(residual))
        self._rows.append({
            'residual': residual,
            'residual_norm': residual_norm,
            'momentum': self.observer.momentum.copy(),
            'momentum_estimate': self.observer.momentum_estimate.copy(),
            'collision_detected': detected,
            'detection_event': event,
            'collision_joints': self.detector.joints.copy(),
        })
        if event:
            if self._detection_callback is not None:
                self._detection_callback(float(t))
            joints = (np.flatnonzero(self.detector.joints) + 1).tolist()
            return (f'collision detected at t={t:.4f} s: joints={joints}, '
                    f'|r|={residual_norm:.4f} N m')
        return None

    def export_log(self, saved_times):
        """Return the existing NPZ fields aligned to saved plant samples.

        An interruption can leave one extra monitoring sample whose plant row
        was never appended. Discard it and derive detection times only from
        the rows actually being saved. No file I/O is performed here.
        """
        saved_times = np.asarray(saved_times)
        if not self._rows or saved_times.size == 0:
            return {}
        rows = self._rows[:len(saved_times)]
        out = {key: np.asarray([row[key] for row in rows]) for key in self._rows[0]}
        out['residual_threshold'] = self.detector.threshold.copy()
        out['detection_times'] = saved_times[out['detection_event']]
        return out

    def metadata(self, command_delay_s):
        """Describe monitoring using the actuator history's authoritative delay."""
        return {
            'enabled': self.enabled,
            'gain': self.observer.gain.tolist() if self.enabled else None,
            'threshold': self.detector.threshold.tolist() if self.enabled else None,
            'torque_signal': 'applied actuator torque, gravity compensated',
            'command_delay_s': command_delay_s,
        }
