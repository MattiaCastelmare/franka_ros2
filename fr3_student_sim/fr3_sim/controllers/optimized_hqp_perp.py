"""The shared tracking/HQP law with a single perpendicular approach."""
from __future__ import annotations

from fr3_sim.controllers.optimized_hqp import HQPTrackingController
from fr3_sim.methods.perpendicular_reference import PerpendicularApproachReference


class Controller(HQPTrackingController):
    """Replace only the reference; keep HQP constraints and observer handling.

    The movement and final hold use the same schedule as baseline_perp. A QP
    failure still terminates immediately, with the parent's recorded reason.
    No true contact force is passed to the controller.
    """

    def __init__(self, robot, params, dt):
        wall = params['wall_geometry']
        reference = PerpendicularApproachReference(
            robot.position(params['initial_q']), wall['point'], wall['normal'],
            **params.get('approach', {}),
        )
        super().__init__(robot, params, dt, reference)

    def update(self, t, q, dq):
        torque = super().update(t, q, dq)
        reference = self.reference
        distance = float(reference.direction @
                         (self.log['p_ref'] - reference.start_position))
        self.log.update({
            'p_initial': reference.start_position.copy(),
            'approach_distance_ref': distance,
            'reference_depth': distance - reference.gap,
            'approach_complete': float(t >= reference.motion_end_time),
            'trial_complete': 0.0,
        })
        if not self.done and t >= reference.end_time:
            self.log['trial_complete'] = 1.0
            self.done = True
        return torque
