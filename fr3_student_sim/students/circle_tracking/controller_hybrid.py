"""Circular mass minimization followed by a latched, damped contact release.

The controller subscribes to collision onsets and remembers the first one.
Before that event, reuse circular tracking with mass minimization. Afterwards,
command only joint damping; the simulator continues to compensate gravity.
"""

import numpy as np

from controller_mass_minimization import Controller as MassCircleController


class Controller(MassCircleController):
    """Switch once from TRACK to RELEASE and stay there until the run ends."""

    def __init__(self, robot, params, dt):
        super().__init__(robot, params, dt)
        reaction = params.get('reaction', {})
        damping = np.asarray(
            reaction.get('damping', [30.0, 30.0, 30.0, 20.0, 10.0, 10.0, 5.0]),
            dtype=float)
        if damping.ndim == 0:
            damping = np.full(7, float(damping))
        if (damping.shape != (7,) or not np.all(np.isfinite(damping))
                or np.any(damping <= 0.0)):
            raise ValueError('reaction.damping must be a positive scalar or seven '
                             'positive finite joint values')
        self.release_damping = damping.copy()  # [N m s/rad], not an acceleration gain.
        self.first_collision_time = None
        self.release_start_time = None

    def bind_collision_monitor(self, monitor):
        """Require detection and connect it directly to this controller's handler."""
        if not monitor.enabled:
            raise ValueError('hybrid controller requires collision monitoring; '
                             'launch with observer:=true')
        monitor.set_detection_callback(self.on_collision)

    def on_collision(self, t):
        """Latch the first onset even between controller updates; never re-arm."""
        if self.first_collision_time is None:
            self.first_collision_time = float(t)

    def update(self, t, q, dq):
        """Return the selected mode's torque, without gravity compensation."""
        if self.release_start_time is None and self.first_collision_time is not None:
            # This is the first RELEASE command time, which can follow detection
            # if control_rate is lower than the joint-state measurement rate.
            self.release_start_time = float(t)

        if self.release_start_time is None:
            torque = super().update(t, q, dq)
            self.log.update({
                'release_active': 0.0,
                'release_start_time': np.nan,
                'tau_release': np.zeros(7),
            })
            return torque

        # No Cartesian tracking, mass torque, nullspace damping or Coriolis
        # compensation in RELEASE. Damping acts directly on all seven joints.
        torque = -self.release_damping * np.asarray(dq, dtype=float)
        self._log_release(q, dq, torque)
        # Keep done=False: the runner's duration and existing stops end the run.
        return torque

    def _log_release(self, q, dq, torque):
        """Continue mass/velocity diagnostics without evaluating the mass gradient."""
        normal = self.mass_objective.normal
        J = self.robot.jacobian_pos(q)
        self.log = {
            'circle_center': self.reference.center.copy(),
            'p_initial': self.p_initial.copy(),
            'qdd_null': np.zeros(7),
            'mass_kg': float(self.robot.effective_mass(q, normal)),
            'tau_mass': np.zeros(7),
            'normal_speed': float(-normal @ (J @ dq)),
            'normal_speed_ref': np.nan,
            'release_active': 1.0,
            'release_start_time': self.release_start_time,
            'tau_release': torque.copy(),  # Requested before total-torque clipping.
        }
        # Inactive references and unevaluated diagnostics are not zero-valued
        # control targets. Preserve their shapes and mark them as unavailable.
        for key in ('p_ref', 'v_ref', 'a_ref', 'position_error', 'velocity_error'):
            self.log[key] = np.full(3, np.nan)
        for key in ('qdd_command', 'dq_null', 'mass_gradient'):
            self.log[key] = np.full(7, np.nan)
