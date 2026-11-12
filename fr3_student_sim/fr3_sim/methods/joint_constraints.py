"""Acceleration bounds for torque control with an estimated joint disturbance.

All limits refer to the gravity-compensated actuator convention of this kit.
The robust bounds are conditional on the supplied acceleration-error envelope;
they cannot guarantee invariance under arbitrary impacts or missed deadlines.
"""

from dataclasses import dataclass

import numpy as np

from fr3_sim.simulation.fr3_model import DQ_MAX, Q_MAX, Q_MIN, TAU_MAX


def vector(value, size, name, *, positive=False, nonnegative=False):
    """Accept a finite scalar or a vector, with explicit configuration errors."""
    result = np.asarray(value, dtype=float)
    if result.ndim == 0:
        result = np.full(size, float(result))
    if result.shape != (size,) or not np.all(np.isfinite(result)):
        raise ValueError(f'{name} must contain {size} finite values or a scalar')
    if positive and np.any(result <= 0):
        raise ValueError(f'{name} must be positive')
    if nonnegative and np.any(result < 0):
        raise ValueError(f'{name} must be nonnegative')
    return result.copy()


@dataclass
class AccelerationBounds:
    lower: np.ndarray
    upper: np.ndarray
    disturbance: np.ndarray
    uncertainty: np.ndarray
    margin: np.ndarray
    predicted_q: np.ndarray
    predicted_dq: np.ndarray
    soft_lower: np.ndarray | None = None
    soft_upper: np.ndarray | None = None
    soft_scale: np.ndarray | None = None


class JointConstraints:
    """Combine position/velocity barriers, step prediction and acceleration caps.

    The QP variable is u = M^-1 (tau - c), whereas actual acceleration is
    u + M^-1 tau_ext. With estimated disturbance d and error |e| <= epsilon,
    every lower/upper bound L <= qdd <= U becomes
    L - d + epsilon <= u <= U - d - epsilon.

    Position barriers use two equal positive poles. Their invariance also
    requires a viable initial state, feasible torques and bounded errors.
    Delay prediction uses the previously applied actuator command, not a
    contact model or true external force.

    With ``soft_constraints.enabled``, only next-state position/velocity
    predictions form the hard interval. Acceleration, inner position barriers
    and velocity-rate barriers are returned as three separate soft groups.
    The default strict path retains the original merged bounds.
    """

    def __init__(self, params):
        self.margin = vector(params.get('position_margin', 0.15), 7,
                             'safety.position_margin', positive=True)
        if np.any(self.margin <= 0.05) or np.any(2 * self.margin >= Q_MAX - Q_MIN):
            raise ValueError('position_margin must exceed runner margin 0.05 rad and fit joints')
        self.q_lower, self.q_upper = Q_MIN + self.margin, Q_MAX - self.margin
        self.velocity = vector(params.get('velocity_max', DQ_MAX), 7,
                               'safety.velocity_max', positive=True)
        self.acceleration = vector(params.get('acceleration_max', 30.0), 7,
                                   'safety.acceleration_max', positive=True)
        self.torque = vector(params.get('torque_max', TAU_MAX), 7,
                             'safety.torque_max', positive=True)
        if np.any(self.velocity > DQ_MAX) or np.any(self.torque > TAU_MAX):
            raise ValueError('velocity/torque limits cannot exceed simulator limits')
        self.position_rate = float(params.get('position_barrier_rate', 8.0))
        self.velocity_rate = float(params.get('velocity_barrier_rate', 10.0))
        self.delay = float(params.get('command_delay_s', 0.003))
        for name, value in (('position_barrier_rate', self.position_rate),
                            ('velocity_barrier_rate', self.velocity_rate)):
            if not np.isfinite(value) or value <= 0:
                raise ValueError(f'safety.{name} must be positive and finite')
        if not np.isfinite(self.delay) or self.delay < 0:
            raise ValueError('safety.command_delay_s must be finite and nonnegative')
        soft = params.get('soft_constraints', {})
        self.soft_enabled = soft.get('enabled', False)
        if not isinstance(self.soft_enabled, bool):
            raise ValueError('safety.soft_constraints.enabled must be boolean')
        self.hard_margin = self.margin.copy()
        self.soft_scale = None
        if self.soft_enabled:
            self.hard_margin = vector(soft.get('hard_position_margin', 0.06), 7,
                                      'safety.soft_constraints.hard_position_margin', positive=True)
            if np.any(self.hard_margin <= 0.05) or np.any(self.hard_margin > self.margin):
                raise ValueError('hard_position_margin must exceed runner margin 0.05 rad '
                                 'and not exceed the preferred position_margin')
            # All three preference groups constrain acceleration, so their
            # normalization scales have units rad/s^2. Group order is fixed.
            self.soft_scale = np.vstack([
                vector(soft.get(name, self.acceleration), 7,
                       'safety.soft_constraints.' + name, positive=True)
                for name in ('acceleration_scale', 'position_scale', 'velocity_scale')
            ])
        self.hard_q_lower = Q_MIN + self.hard_margin
        self.hard_q_upper = Q_MAX - self.hard_margin

    def bounds(self, q, dq, M_inv, c, tau_applied, residual, uncertainty, dt):
        """Return hard bounds, optional soft rows and the delayed predicted state."""
        if not np.isfinite(dt) or dt <= 0:
            raise ValueError('prediction interval must be positive and finite')
        disturbance = M_inv @ residual
        previous_acceleration = M_inv @ (tau_applied - c + residual)
        delay = self.delay
        qp = q + delay * dq + 0.5 * delay**2 * previous_acceleration
        vp = dq + delay * previous_acceleration
        # Include acceleration uncertainty during command transport as well.
        q_error = 0.5 * delay**2 * uncertainty
        v_error = delay * uncertainty
        rate, vrate = self.position_rate, self.velocity_rate
        if self.soft_enabled:
            # Mandatory prediction of the next state, after transport delay.
            # These rows have NO slack. This is a local, constant-acceleration
            # prediction conditional on the disturbance/error envelope; it is
            # not a proof of future braking feasibility under arbitrary impacts.
            hard_lower = np.maximum(
                (-self.velocity - vp + v_error) / dt,
                2 * (self.hard_q_lower - qp + q_error - dt * (vp - v_error)) / dt**2,
            )
            hard_upper = np.minimum(
                (self.velocity - vp - v_error) / dt,
                2 * (self.hard_q_upper - qp - q_error - dt * (vp + v_error)) / dt**2,
            )
            # Keep each comfort preference separate, including contradictory
            # lower/upper values: the elastic QP decides its minimum relaxation.
            soft_lower = np.vstack([
                -self.acceleration,
                -2 * rate * (vp - v_error) - rate**2 * (qp - q_error - self.q_lower),
                -vrate * (vp - v_error + self.velocity),
            ])
            soft_upper = np.vstack([
                self.acceleration,
                -2 * rate * (vp + v_error) + rate**2 * (self.q_upper - qp - q_error),
                vrate * (self.velocity - vp - v_error),
            ])
            return AccelerationBounds(
                lower=hard_lower - disturbance + uncertainty,
                upper=hard_upper - disturbance - uncertainty,
                disturbance=disturbance.copy(), uncertainty=uncertainty.copy(),
                margin=np.minimum(q - Q_MIN, Q_MAX - q),
                predicted_q=qp, predicted_dq=vp,
                soft_lower=soft_lower - disturbance + uncertainty,
                soft_upper=soft_upper - disturbance - uncertainty,
                soft_scale=self.soft_scale.copy(),
            )
        lower = np.maximum.reduce([
            -self.acceleration,
            -2 * rate * (vp - v_error) - rate**2 * (qp - q_error - self.q_lower),
            -vrate * (vp - v_error + self.velocity),
            (-self.velocity - vp + v_error) / dt,
            2 * (self.q_lower - qp + q_error - dt * (vp - v_error)) / dt**2,
        ])
        upper = np.minimum.reduce([
            self.acceleration,
            -2 * rate * (vp + v_error) + rate**2 * (self.q_upper - qp - q_error),
            vrate * (self.velocity - vp - v_error),
            (self.velocity - vp - v_error) / dt,
            2 * (self.q_upper - qp - q_error - dt * (vp + v_error)) / dt**2,
        ])
        return AccelerationBounds(
            lower=lower - disturbance + uncertainty,
            upper=upper - disturbance - uncertainty,
            disturbance=disturbance.copy(), uncertainty=uncertainty.copy(),
            margin=np.minimum(q - Q_MIN, Q_MAX - q),
            predicted_q=qp, predicted_dq=vp)
