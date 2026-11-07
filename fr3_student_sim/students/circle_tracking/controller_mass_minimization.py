"""Circular position tracking with directional reflected-mass minimization.

Reuse the circular controller's Cartesian tracking and nullspace damping.
Add the same dynamically projected mass-gradient torque as the straight
variant. The objective direction is the fixed wall normal in the base frame;
the fingertip orientation remains free. The simulator compensates gravity.
"""

import numpy as np

from controller_nullspace_damping import Controller as DampedCircleController
from utils.mass_objective import DirectionalMassObjective, projected_mass_torque


class Controller(DampedCircleController):
    """Add only mass minimization to the damped circular tracking law."""

    def __init__(self, robot, params, dt):
        super().__init__(robot, params, dt)
        mass = params.get('mass_minimization', {})
        self.mass_objective = DirectionalMassObjective(
            robot, params['wall_geometry']['normal'], mass.get('gradient_step', 1e-4))
        self.mass_gain = float(mass.get('gain', 0.01))
        if not np.isfinite(self.mass_gain) or self.mass_gain < 0.0:
            raise ValueError('mass_minimization.gain must be finite and nonnegative')

    def update(self, t, q, dq):
        """Return tracking, damping and mass torques, without adding gravity."""
        tau_tracking = super().update(t, q, dq)
        J = self.robot.jacobian_pos(q)
        M = self.robot.mass_matrix(q)
        mass_kg, gradient = self.mass_objective.evaluate(q)
        tau_mass = projected_mass_torque(M, J, gradient, self.mass_gain)

        # Include the requested mass contribution in the total acceleration log.
        self.log['qdd_command'] = (
            self.log['qdd_command'] + np.linalg.solve(M, tau_mass))
        self.log.update({
            'mass_kg': mass_kg,
            'mass_gradient': gradient.copy(),
            'tau_mass': tau_mass.copy(),  # Requested before the runner clips total torque.
            'normal_speed': float(-self.mass_objective.normal @ (J @ dq)),
            'normal_speed_ref': float(-self.mass_objective.normal @ self.log['v_ref']),
        })
        return tau_tracking + tau_mass
