"""Time alignment of actuator commands actually sent to the simulator.

The runner records commands after clipping and before adding the virtual wall.
Gazebo's known transport delay shifts their application times. Integrating
the held commands over each measurement interval also handles missed states;
using the latest raw controller output would give the wrong observer input.
"""

import numpy as np

from fr3_sim.monitoring.momentum_observer import joint_vector


class AppliedTorqueHistory:

    def __init__(self, delay=0.001):
        self.delay = float(delay)
        if not np.isfinite(self.delay) or self.delay < 0.0:
            raise ValueError('actuator delay must be finite and nonnegative')
        self.commands = []

    def record(self, t, torque):
        application_time = float(t) + self.delay
        if not np.isfinite(application_time):
            raise ValueError('command timestamp must be finite')
        if self.commands and application_time <= self.commands[-1][0]:
            raise ValueError('command timestamps must be strictly increasing')
        self.commands.append((application_time, joint_vector(torque, 'actuator torque')))

    def mean(self, start, end):
        """Average the applied torque on [start, end), including the delay."""
        start, end = float(start), float(end)
        if not np.isfinite(start) or not np.isfinite(end) or end <= start:
            raise ValueError('torque interval must have finite, increasing endpoints')
        integral = np.zeros(7)
        right = end
        for application_time, torque in reversed(self.commands):
            if application_time >= end - 1e-10:
                continue
            left = max(start, application_time)
            integral += (right - left) * torque
            right = left
            if right <= start:
                break
        # Before the first recorded command, the commanded actuator torque is zero.
        return integral / (end - start)
