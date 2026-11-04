"""Six-dimensional circular tracking through the student's torque interface.

The simulator already compensates gravity. This controller returns
``M(q) @ qdd_command + coriolis(q, dq)`` and never adds gravity again.
"""
from __future__ import annotations

import numpy as np
import pinocchio as pin

from fr3_student_sim.controller_base import StudentController
from reference import CircleReference


def positive_gain(value: float, name: str, *, allow_zero: bool = False) -> float:
    """Read a finite gain and reject configuration errors before simulation."""
    result = float(value)
    if not np.isfinite(result) or result < 0 or (result == 0 and not allow_zero):
        bound = 'nonnegative' if allow_zero else 'positive'
        raise ValueError(f'{name} must be finite and {bound}')
    return result


def world_orientation_error(desired: np.ndarray, current: np.ndarray) -> np.ndarray:
    """Rotation vector from current to desired, expressed in the base frame."""
    return np.asarray(pin.log3(desired @ current.T), dtype=float).copy()


class Controller(StudentController):
    """Track a circle at the fingertip while holding its initial orientation.

    A damped inverse of the full geometric Jacobian converts translational and
    rotational acceleration commands into joint accelerations. A separate SVD
    nullspace projector keeps the arm near its initial posture without leaking
    the posture command into the Cartesian task. The runner applies torque
    limits and records both the requested and clipped torques.
    """

    def __init__(self, robot, params, dt):
        super().__init__(robot, params, dt)
        self.q_initial = np.asarray(params['initial_q'], dtype=float).reshape(7).copy()
        if not np.all(np.isfinite(self.q_initial)):
            raise ValueError('initial_q must contain seven finite joint positions')
        self.p_initial, self.rotation_desired = robot.pose(self.q_initial)
        circle = params.get('circle', {})
        self.reference = CircleReference(
            self.p_initial,
            radius=circle.get('radius', 0.04),
            period=circle.get('period', 8.0),
            ramp=circle.get('ramp', 1.0),
        )
        tracking = params.get('tracking', {})
        self.kp_position = positive_gain(tracking.get('kp_position', 100.0), 'kp_position')
        self.kd_position = positive_gain(tracking.get('kd_position', 20.0), 'kd_position')
        self.kp_orientation = positive_gain(
            tracking.get('kp_orientation', 64.0), 'kp_orientation')
        self.kd_orientation = positive_gain(
            tracking.get('kd_orientation', 16.0), 'kd_orientation')
        self.kp_posture = positive_gain(
            tracking.get('kp_posture', 4.0), 'kp_posture', allow_zero=True)
        self.kd_posture = positive_gain(
            tracking.get('kd_posture', 4.0), 'kd_posture', allow_zero=True)
        self.damping = positive_gain(
            tracking.get('jacobian_damping', 0.01), 'jacobian_damping')

    def task_acceleration_bias(self, q: np.ndarray, dq: np.ndarray) -> np.ndarray:
        """Full world-aligned ``Jdot(q, dq) @ dq`` at the controlled fingertip."""
        fid = self.robot.frame_id(self.robot.tool_frame)
        pin.forwardKinematics(self.robot.model, self.robot.data, q, dq, np.zeros(7))
        bias = pin.getFrameClassicalAcceleration(
            self.robot.model, self.robot.data, fid, pin.LOCAL_WORLD_ALIGNED)
        return np.concatenate((bias.linear.copy(), bias.angular.copy()))

    def inverse_and_nullspace(self, jacobian: np.ndarray):
        """Damped task inverse and an independent, undamped nullspace projector."""
        left, singular_values, right_t = np.linalg.svd(jacobian, full_matrices=True)
        inverse_values = singular_values / (singular_values ** 2 + self.damping ** 2)
        inverse = (right_t[:6].T * inverse_values) @ left.T
        tolerance = np.finfo(float).eps * max(jacobian.shape) * singular_values[0]
        rank = int(np.count_nonzero(singular_values > tolerance))
        basis = right_t[rank:].T
        return inverse, basis @ basis.T

    def update(self, t, q, dq):
        """Return seven joint torques for position and constant-orientation tracking."""
        q = np.asarray(q, dtype=float).reshape(7)
        dq = np.asarray(dq, dtype=float).reshape(7)
        if not np.all(np.isfinite(q)) or not np.all(np.isfinite(dq)):
            raise ValueError('joint position and velocity must be finite')

        p_ref, v_ref, a_ref = self.reference.sample(t)
        position, rotation = self.robot.pose(q)
        jacobian = self.robot.jacobian(q)
        twist = jacobian @ dq
        position_error = p_ref - position
        velocity_error = v_ref - twist[:3]
        orientation_error = world_orientation_error(self.rotation_desired, rotation)
        task_acceleration = np.concatenate((
            a_ref + self.kp_position * position_error + self.kd_position * velocity_error,
            self.kp_orientation * orientation_error - self.kd_orientation * twist[3:],
        ))
        inverse, nullspace = self.inverse_and_nullspace(jacobian)
        posture_acceleration = (
            self.kp_posture * (self.q_initial - q) - self.kd_posture * dq)
        qdd_command = (
            inverse @ (task_acceleration - self.task_acceleration_bias(q, dq))
            + nullspace @ posture_acceleration
        )
        torque = self.robot.mass_matrix(q) @ qdd_command + self.robot.coriolis(q, dq)

        # Each array is a snapshot: the runner retains these values until saving.
        self.log = {
            'p_ref': p_ref.copy(),
            'v_ref': v_ref.copy(),
            'a_ref': a_ref.copy(),
            'circle_center': self.reference.center.copy(),
            'p_initial': self.p_initial.copy(),
            'rotation_ref': self.rotation_desired.reshape(9).copy(),
            'rotation': rotation.reshape(9).copy(),
            'position_error': position_error.copy(),
            'velocity_error': velocity_error.copy(),
            'orientation_error': orientation_error.copy(),
            'orientation_error_norm': float(np.linalg.norm(orientation_error)),
            'qdd_command': qdd_command.copy(),
        }
        return torque
