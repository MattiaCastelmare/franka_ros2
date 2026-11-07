"""Mass-minimizing circle, then a horizontal retreat on the first detection.

The collision callback latches the first onset. The next controller update
starts the retreat immediately, without waiting for the robot to stop. Reuse
the existing damped position tracker, with no mass objective after detection.
Gravity remains compensated by the simulator.
"""

from types import SimpleNamespace

import numpy as np

from controller_mass_minimization import Controller as MassCircleController
from controller_nullspace_damping import Controller as DampedCircleController
from retreat_reference import RetreatReference


class Controller(MassCircleController):
    """Switch once from the circle to a retreat, then hold its endpoint."""

    def __init__(self, robot, params, dt):
        super().__init__(robot, params, dt)
        retreat = params.get('retreat', {})
        self.retreat_distance = float(retreat.get('distance', 0.03))
        self.retreat_duration = float(retreat.get('duration', 2.0))
        for name, value in (('distance', self.retreat_distance),
                            ('duration', self.retreat_duration)):
            if not np.isfinite(value) or value <= 0.0:
                raise ValueError(f'retreat.{name} must be finite and positive')

        # Out of the wall, horizontally in the base x-y plane. For the current
        # vertical wall this is -x; height is kept at its value at the trigger.
        direction = self.mass_objective.normal.copy()
        direction[2] = 0.0
        length = np.linalg.norm(direction)
        if length <= 1e-12:
            raise ValueError('horizontal retreat requires a horizontal component '
                             'of the wall normal')
        self.retreat_displacement = self.retreat_distance * direction / length

        self.first_collision_time = None
        self.retreat_start_time = None
        self.retreat_reference = None
        # Composition reuses the existing Cartesian law and nullspace damping
        # without changing either baseline controller or evaluating a mass gradient.
        self.retreat_tracker = DampedCircleController(robot, params, dt)

    def bind_collision_monitor(self, monitor):
        """Connect the detector directly to the controller; no true force input."""
        if not monitor.enabled:
            raise ValueError('hybrid retreat requires collision monitoring; '
                             'launch with observer:=true')
        monitor.set_detection_callback(self.on_collision)

    def on_collision(self, t):
        """Latch only the first detection; subsequent events do not restart retreat."""
        if self.first_collision_time is None:
            self.first_collision_time = float(t)

    def update(self, t, q, dq):
        """Return a torque without gravity; never resume the circular reference."""
        if self.first_collision_time is None:
            torque = super().update(t, q, dq)
        else:
            if self.retreat_start_time is None:
                self.retreat_start_time = float(t)
                self.retreat_reference = RetreatReference(
                    self.robot.position(q), self.retreat_displacement,
                    self.retreat_duration)
                # The damped tracker reads sample() and center. Preserve the
                # original circle center only as its existing log diagnostic.
                self.retreat_tracker.reference = SimpleNamespace(
                    sample=self.retreat_reference.sample,
                    center=self.reference.center.copy())

            elapsed = float(t) - self.retreat_start_time
            torque = self.retreat_tracker.update(elapsed, q, dq)
            self.log = dict(self.retreat_tracker.log)
            normal = self.mass_objective.normal
            J = self.robot.jacobian_pos(q)
            self.log.update({
                'mass_kg': float(self.robot.effective_mass(q, normal)),
                'mass_gradient': np.full(7, np.nan),
                'tau_mass': np.zeros(7),
                'normal_speed': float(-normal @ (J @ dq)),
                'normal_speed_ref': float(-normal @ self.log['v_ref']),
            })

        active = self.retreat_start_time is not None
        self.log.update({
            'retreat_active': float(active),
            'retreat_start_time': self.retreat_start_time if active else np.nan,
            # Completion describes the reference, not a measured arrival test.
            'retreat_complete': float(
                active and float(t) - self.retreat_start_time >= self.retreat_duration),
        })
        # Hold the endpoint after the reference ends. Existing runner stops and
        # duration remain in charge; detachment does not set done here.
        return torque
