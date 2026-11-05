"""Straight approach to a plane, with continuous position, speed and acceleration.

The reference reaches the plane at the configured approach speed, then brakes
smoothly over the requested penetration distance. It holds the endpoint after
the move; the controller decides when the experiment ends.
"""

import numpy as np


class StraightLineReference:
    """A smooth startup, constant-speed approach and smooth final braking.

    ``direction`` points into the wall and ``wall_distance`` is the distance
    from the start to its plane along that direction. All vectors are in the
    robot base frame. Distances are in m and times in s.
    """

    def __init__(self, start_position, direction, wall_distance,
                 approach_speed, ramp_time, penetration):
        self.start_position = np.asarray(start_position, dtype=float).copy()
        direction = np.asarray(direction, dtype=float)
        for name, vector in (('start_position', self.start_position),
                             ('direction', direction)):
            if vector.shape != (3,) or not np.all(np.isfinite(vector)):
                raise ValueError(f'{name} must contain three finite values')
        length = np.linalg.norm(direction)
        if not np.isfinite(length) or length <= 0.0:
            raise ValueError('approach direction must have a finite, nonzero norm')
        self.direction = direction / length
        for name, value in (('wall_distance', wall_distance),
                            ('approach_speed', approach_speed),
                            ('ramp_time', ramp_time), ('penetration', penetration)):
            value = float(value)
            if not np.isfinite(value) or value <= 0.0:
                raise ValueError(f'{name} must be finite and positive')
            setattr(self, name, value)

        # Integrating v * (3*u**2 - 2*u**3) over the startup gives v*T/2.
        self.ramp_distance = 0.5 * self.approach_speed * self.ramp_time
        if self.ramp_distance > self.wall_distance:
            raise ValueError('startup ramp reaches beyond the wall; reduce '
                             'line.ramp_time or line.approach_speed')
        cruise_time = (self.wall_distance - self.ramp_distance) / self.approach_speed
        self.contact_time = self.ramp_time + cruise_time
        self.braking_time = 2.0 * self.penetration / self.approach_speed
        self.move_duration = self.contact_time + self.braking_time
        self.end_position = (self.start_position
                             + (self.wall_distance + self.penetration) * self.direction)

    def sample(self, t):
        """Return position [m], velocity [m/s] and acceleration [m/s^2].

        The scheduled contact time is a property of the reference only. No
        contact force or collision-detection signal is used to generate it.
        """
        t = float(t)
        if not np.isfinite(t):
            raise ValueError('reference time must be finite')
        if t <= 0.0:
            return self.start_position.copy(), np.zeros(3), np.zeros(3)
        if t >= self.move_duration:
            return self.end_position.copy(), np.zeros(3), np.zeros(3)

        if t < self.ramp_time:
            u = t / self.ramp_time
            distance = self.approach_speed * self.ramp_time * (u**3 - 0.5 * u**4)
            speed = self.approach_speed * (3.0 * u**2 - 2.0 * u**3)
            acceleration = self.approach_speed / self.ramp_time * 6.0 * u * (1.0 - u)
        elif t < self.contact_time:
            distance = self.ramp_distance + self.approach_speed * (t - self.ramp_time)
            speed, acceleration = self.approach_speed, 0.0
        else:
            u = (t - self.contact_time) / self.braking_time
            distance = (self.wall_distance + self.approach_speed * self.braking_time
                        * (u - u**3 + 0.5 * u**4))
            speed = self.approach_speed * (1.0 - 3.0 * u**2 + 2.0 * u**3)
            acceleration = -self.approach_speed / self.braking_time * 6.0 * u * (1.0 - u)

        return (self.start_position + distance * self.direction,
                speed * self.direction, acceleration * self.direction)
