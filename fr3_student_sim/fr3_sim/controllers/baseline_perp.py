"""Baseline feedback with a single perpendicular approach instead of a circle."""
from __future__ import annotations

from fr3_sim.controllers.baseline import Controller as BaselineController
from fr3_sim.methods.perpendicular_reference import PerpendicularApproachReference


class Controller(BaselineController):
    """Reuse baseline posture/damping and torque law; replace only its reference.

    Neither the observer nor true contact forces change the command. The trial
    ends after the reference reaches its endpoint and completes a short hold.
    Runner saturation and safety stops continue to apply.
    """

    def _make_reference(self, start_position, params):
        wall = params['wall_geometry']
        return PerpendicularApproachReference(
            start_position, wall['point'], wall['normal'],
            **params.get('approach', {}),
        )

    def update(self, t, q, dq):
        torque = super().update(t, q, dq)
        reference = self.reference
        self.log.update({
            'normal_speed': float(-reference.normal @
                                  (self.log['v_ref'] - self.log['velocity_error'])),
            'normal_speed_ref': float(-reference.normal @ self.log['v_ref']),
            'approach_distance_ref': float(reference.direction @
                                           (self.log['p_ref'] - self.p_initial)),
            'reference_depth': float(reference.direction @
                                     (self.log['p_ref'] - self.p_initial) - reference.gap),
            'approach_complete': float(t >= reference.motion_end_time),
            'trial_complete': float(t >= reference.end_time),
        })
        if t >= reference.end_time:
            self.done = True
        return torque
