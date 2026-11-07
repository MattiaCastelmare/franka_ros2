"""Shared student utility for directional mass and projected gradient torque.

The robot interface supplies ``effective_mass(q, normal)`` at the controlled
point, expressed in the same base frame as the fixed direction ``normal``.
Mass is measured in kg; its joint gradient is in kg/rad.
"""

import numpy as np


class DirectionalMassObjective:
    """Evaluate reflected mass and its central-difference joint gradient."""

    def __init__(self, robot, normal, gradient_step):
        normal = np.asarray(normal, dtype=float)
        if normal.shape != (3,) or not np.all(np.isfinite(normal)):
            raise ValueError('mass direction must contain three finite values')
        length = np.linalg.norm(normal)
        if not np.isfinite(length) or length <= 0.0:
            raise ValueError('mass direction must have a finite, nonzero norm')
        self.normal = (normal / length).copy()
        self.gradient_step = float(gradient_step)
        if not np.isfinite(self.gradient_step) or self.gradient_step <= 0.0:
            raise ValueError('mass gradient step must be finite and positive')
        self.robot = robot

    def evaluate(self, q):
        """Return mass [kg] and gradient [kg/rad] at ``q`` [rad].

        A seven-joint robot requires one nominal mass evaluation and fourteen
        evaluations at symmetric perturbations. The input configuration is
        unchanged; the gradient is neither normalized nor clipped.
        """
        q = np.asarray(q, dtype=float)
        if q.ndim != 1 or q.size == 0 or not np.all(np.isfinite(q)):
            raise ValueError('joint positions must be a finite, nonempty vector')
        mass = float(self.robot.effective_mass(q, self.normal))
        gradient = np.empty_like(q)
        h = self.gradient_step
        for joint in range(q.size):
            q_plus, q_minus = q.copy(), q.copy()
            q_plus[joint] += h
            q_minus[joint] -= h
            mass_plus = self.robot.effective_mass(q_plus, self.normal)
            mass_minus = self.robot.effective_mass(q_minus, self.normal)
            gradient[joint] = (mass_plus - mass_minus) / (2.0 * h)
        return mass, gradient


def projected_mass_torque(M, J, gradient, gain):
    """Return the mass-gradient torque [N m] in the dynamic task null space.

    ``J`` is the task Jacobian and ``M`` the joint inertia matrix. ``gradient``
    is in kg/rad, so ``gain`` is in N m rad/kg. For positive-definite ``M``
    and full-row-rank ``J``, the exact solves give ``J M^-1 tau_mass = 0``.
    No inverse damping, joint-limit objective or gradient scaling is added.
    """
    M = np.asarray(M, dtype=float)
    J = np.asarray(J, dtype=float)
    gradient = np.asarray(gradient, dtype=float)
    B = np.linalg.solve(M, J.T)
    Jbar_T = np.linalg.solve(J @ B, B.T)
    N_tau = np.eye(M.shape[0]) - J.T @ Jbar_T
    return -float(gain) * (N_tau @ gradient)
