"""Collision monitoring and log collection, without ROS or controller dependencies.

This module coordinates the momentum observer and residual threshold detector;
their mathematical implementations remain in their own modules. Inputs are
measured joint states and the already aligned, gravity-compensated actuator
torque. No wall object, true contact force or tracking command is required.
"""

import numpy as np
import yaml

from fr3_student_sim.collision_detector import CollisionDetector
from fr3_student_sim.momentum_observer import GeneralizedMomentumObserver


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
        residual = self.observer.update(t, q, dq, tau_applied)
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
