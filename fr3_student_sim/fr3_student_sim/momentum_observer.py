"""Generalized momentum observer, independent of the tracking controller.

Robotics 2, lecture 18, slide 29; Haddadin, De Luca and Albu-Schaeffer,
"Robot Collisions: A Survey on Detection, Isolation, and Identification"
(2017), equations 42--47.

With Pinocchio's convention Mdot = C + C.T:

    p = M(q) dq
    p_hat_dot = tau_motor + C(q, dq).T dq - g(q) + r
    r = K_O (p - p_hat)

The simulator already adds its own gravity compensation. ``tau_applied``
is therefore the actual, delayed, saturated actuator command WITHOUT this
compensation, and the equivalent observer drive is tau_applied + C.T dq.
It must exclude the virtual wall torque. Neither external forces nor joint
accelerations are inputs to this observer.
"""

import numpy as np


def joint_vector(value, name, *, positive=False):
    """Read one scalar or seven joint values, rejecting invalid settings."""
    values = np.asarray(value, dtype=float)
    if values.ndim == 0:
        values = np.full(7, float(values))
    if values.shape != (7,) or not np.all(np.isfinite(values)):
        raise ValueError(f'{name} must be a finite scalar or a vector of seven values')
    if positive and np.any(values <= 0.0):
        raise ValueError(f'{name} must be positive')
    return values.copy()


class GeneralizedMomentumObserver:
    """Diagonal-gain momentum observer, integrated with implicit Euler.

    ``update`` receives simulation timestamps and the average applied actuator
    torque over the preceding sample interval. The first measurement sets
    p_hat = p, even when the arm is moving, and returns a zero residual.
    Implicit integration keeps the observer stable for variable sample times.
    """

    def __init__(self, robot, gain=30.0):
        self.robot = robot
        self.gain = joint_vector(gain, 'observer gain', positive=True)
        self.time = None
        self.momentum = np.zeros(7)
        self.momentum_estimate = np.zeros(7)
        self.residual = np.zeros(7)

    def reset(self):
        """Initialize from the next measured momentum, rather than from rest."""
        self.time = None
        self.momentum = np.zeros(7)
        self.momentum_estimate = np.zeros(7)
        self.residual = np.zeros(7)

    def update(self, t, q, dq, tau_applied):
        t = float(t)
        q = joint_vector(q, 'q')
        dq = joint_vector(dq, 'dq')
        tau_applied = joint_vector(tau_applied, 'applied actuator torque')
        if not np.isfinite(t):
            raise ValueError('observer timestamp must be finite')
        self.momentum = self.robot.mass_matrix(q) @ dq
        if self.time is None:
            self.time = t
            self.momentum_estimate = self.momentum.copy()
            self.residual = np.zeros(7)
            return self.residual.copy()

        dt = t - self.time
        if dt <= 0.0:
            raise ValueError('observer timestamps must be strictly increasing')
        drive = tau_applied + self.robot.coriolis_matrix(q, dq).T @ dq
        self.momentum_estimate = (
            self.momentum_estimate + dt * (drive + self.gain * self.momentum)
        ) / (1.0 + dt * self.gain)
        self.residual = self.gain * (self.momentum - self.momentum_estimate)
        self.time = t
        return self.residual.copy()
