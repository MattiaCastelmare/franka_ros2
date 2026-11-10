"""The posture baseline plus directional-mass minimization in torque nullspace.

Reuse the baseline's circle, Cartesian tracking, posture law and damping.
An optional fixed q_posture_ref selects the offline low-mass posture, separately
from initial_q, which still defines the start and the circle. Add the same
dynamically projected mass-gradient torque used by the hybrid experiment.
The observer only monitors collisions; this controller keeps tracking.
Gravity remains compensated by the simulator.
"""

import numpy as np

from controller_baseline import Controller as BaselineController
from utils.mass_objective import DirectionalMassObjective, projected_mass_torque


class Controller(BaselineController):
    """Combine a fixed preferred posture with online mass minimization."""

    def __init__(self, robot, params, dt):
        super().__init__(robot, params, dt)
        self.q0 = np.asarray(
            params.get('q_posture_ref', params['initial_q']), dtype=float).copy()
        if self.q0.shape != (7,) or not np.all(np.isfinite(self.q0)):
            raise ValueError('q_posture_ref must contain seven finite joint angles')
        mass = params.get('mass_minimization', {})
        self.mass_objective = DirectionalMassObjective(robot, self.mass_direction)
        self.mass_gain = float(mass.get('gain', 0.01))
        if not np.isfinite(self.mass_gain) or self.mass_gain < 0.0:
            raise ValueError('mass_minimization.gain must be finite and nonnegative')

    def update(self, t, q, dq):
        """Return baseline plus projected mass torque, without adding gravity."""
        tau_baseline = super().update(t, q, dq)
        J = self.robot.jacobian_pos(q)
        M = self.robot.mass_matrix(q)
        mass_kg, gradient = self.mass_objective.evaluate(q, M=M, J=J)
        tau_mass = projected_mass_torque(M, J, gradient, self.mass_gain)

        # qdd_null retains the baseline's posture+damping contribution. The
        # total acceleration includes mass torque, which is also logged separately.
        self.log['qdd_command'] = (
            self.log['qdd_command'] + np.linalg.solve(M, tau_mass))
        self.log.update({
            'mass_kg': mass_kg,
            'mass_gradient': gradient.copy(),
            'tau_mass': tau_mass.copy(),  # Requested before total-torque clipping.
            'normal_speed': float(-self.mass_direction @ (J @ dq)),
            'normal_speed_ref': float(-self.mass_direction @ self.log['v_ref']),
        })
        return tau_baseline + tau_mass
