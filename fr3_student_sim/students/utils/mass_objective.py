"""Shared student utility for directional mass and projected gradient torque.

The position Jacobian and its analytical time variation use world-aligned
axes at the controlled point, as does the fixed direction ``normal``.
Mass is measured in kg; its joint gradient is in kg/rad. No configuration
perturbations or numerical differences are used during control.
"""

import numpy as np


class DirectionalMassObjective:
    """Evaluate reflected mass and its analytical joint gradient.

    For the seven revolute coordinates, let j = J.T n, x = M^-1 j and
    m = 1 / (j.T x). The exact derivative contracts the position Hessian
    and inertia derivatives without assembling either tensor:

        grad(m) = -2 m^2 (Jdot(q, x).T n - C(q, x).T x).

    Here x is a virtual velocity, not the measured dq. The formula uses
    a fixed world direction, J = dp/dq, and the Coriolis convention
    Mdot = C + C.T. The gradient is neither normalized nor clipped.
    """

    def __init__(self, robot, normal):
        normal = np.asarray(normal, dtype=float)
        if normal.shape != (3,) or not np.all(np.isfinite(normal)):
            raise ValueError('mass direction must contain three finite values')
        length = np.linalg.norm(normal)
        if not np.isfinite(length) or length <= 0.0:
            raise ValueError('mass direction must have a finite, nonzero norm')
        self.normal = (normal / length).copy()
        self.robot = robot

    def evaluate(self, q, *, M=None, J=None):
        """Return mass [kg] and gradient [kg/rad] at ``q`` [rad].

        Optional M and J reuse the controller's nominal matrices at this q.
        All model outputs used below are snapshots, since Pinocchio shares
        its working data between dynamics and kinematics calls.
        """
        q = np.asarray(q, dtype=float)
        if q.ndim != 1 or q.size == 0 or not np.all(np.isfinite(q)):
            raise ValueError('joint positions must be a finite, nonempty vector')
        M = self.robot.mass_matrix(q) if M is None else np.asarray(M, dtype=float)
        J = self.robot.jacobian_pos(q) if J is None else np.asarray(J, dtype=float)
        j = J.T @ self.normal
        x = np.linalg.solve(M, j)
        mass = 1.0 / float(j @ x)
        Jdot = self.robot.jacobian_pos_time_variation(q, x)
        C = self.robot.coriolis_matrix(q, x)
        gradient = -2.0 * mass**2 * (Jdot.T @ self.normal - C.T @ x)
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
