"""One straight approach to a plane, with a prescribed normal entry speed."""
from __future__ import annotations

import numpy as np


class PerpendicularApproachReference:
    """Accelerate, cruise across the plane, brake and briefly hold the endpoint.

    The plane normal points into free space. Motion is along its negative,
    keeping both tangential coordinates constant. Cubic velocity ramps give
    continuous position, velocity and acceleration. Braking starts at or beyond
    the plane, so reference speed at the crossing equals ``speed``.

    ``penetration`` is the final REFERENCE overlap, not actual robot penetration.
    The schedule uses static geometry only, with no force/detection feedback.
    All vectors are expressed in the robot base/world frame; units are SI.
    """

    def __init__(self, start_position, wall_point, wall_normal, *, speed=0.20,
                 acceleration_duration=0.50, penetration=0.010,
                 braking_distance=0.004, hold_duration=0.15):
        vectors = [np.asarray(v, dtype=float) for v in
                   (start_position, wall_point, wall_normal)]
        if any(v.shape != (3,) or not np.all(np.isfinite(v)) for v in vectors):
            raise ValueError('start position, wall point and normal must be finite 3-vectors')
        start, point, normal = vectors
        length = np.linalg.norm(normal)
        if not np.isfinite(length) or length <= 0.0:
            raise ValueError('wall normal must have a finite, nonzero norm')
        self.start_position = start.copy()
        self.normal = normal / length
        self.direction = -self.normal
        self.gap = float(self.normal @ (start - point))
        if self.gap <= 0.0:
            raise ValueError('perpendicular approach must start outside the wall')

        values = np.asarray([speed, acceleration_duration, penetration,
                             braking_distance, hold_duration], dtype=float)
        if values.shape != (5,) or not np.all(np.isfinite(values)):
            raise ValueError('approach parameters must be finite scalars')
        (self.speed, self.acceleration_duration, self.penetration,
         self.braking_distance, self.hold_duration) = values.tolist()
        if np.any(values[:4] <= 0.0) or self.hold_duration < 0.0:
            raise ValueError('speed, acceleration duration, penetration and braking '
                             'distance must be positive; hold duration must be nonnegative')
        if self.braking_distance > self.penetration:
            raise ValueError('braking_distance must not exceed penetration: '
                             'braking must start at or beyond the wall plane')
        self._ramp_distance = 0.5 * self.speed * self.acceleration_duration
        if self._ramp_distance > self.gap:
            raise ValueError('not enough free distance to reach approach.speed before '
                             'the wall; reduce speed or acceleration_duration')
        self.contact_time = (self.acceleration_duration
                             + (self.gap - self._ramp_distance) / self.speed)
        self._braking_start_distance = self.gap + self.penetration - self.braking_distance
        self.braking_start_time = (self.contact_time
                                   + (self.penetration - self.braking_distance) / self.speed)
        self.braking_duration = 2.0 * self.braking_distance / self.speed
        self.motion_end_time = self.braking_start_time + self.braking_duration
        self.end_time = self.motion_end_time + self.hold_duration
        self.end_position = start + self.direction * (self.gap + self.penetration)

    def sample(self, t):
        """Return independent position, velocity and acceleration arrays at time t."""
        t = float(t)
        if not np.isfinite(t):
            raise ValueError('reference time must be finite')
        if t <= 0.0:
            return self.start_position.copy(), np.zeros(3), np.zeros(3)
        if t < self.acceleration_duration:
            u = t / self.acceleration_duration
            distance = self.speed * self.acceleration_duration * (u**3 - 0.5 * u**4)
            velocity = self.speed * (3.0 * u**2 - 2.0 * u**3)
            acceleration = self.speed / self.acceleration_duration * (6.0 * u - 6.0 * u**2)
        elif t < self.braking_start_time:
            distance = self._ramp_distance + self.speed * (t - self.acceleration_duration)
            velocity, acceleration = self.speed, 0.0
        elif t < self.motion_end_time:
            u = (t - self.braking_start_time) / self.braking_duration
            distance = (self._braking_start_distance + self.speed * self.braking_duration
                        * (u - u**3 + 0.5 * u**4))
            velocity = self.speed * (1.0 - 3.0 * u**2 + 2.0 * u**3)
            acceleration = self.speed / self.braking_duration * (-6.0 * u + 6.0 * u**2)
        else:
            return self.end_position.copy(), np.zeros(3), np.zeros(3)
        return (self.start_position + self.direction * distance,
                self.direction * velocity, self.direction * acceleration)
