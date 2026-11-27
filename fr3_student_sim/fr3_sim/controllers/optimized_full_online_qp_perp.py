"""Perpendicular full-online mass reduction with one constrained QP."""
from fr3_sim.controllers.weighted_qp_tracking import WeightedQPTrackingController
from fr3_sim.methods.perpendicular_reference import PerpendicularApproachReference


class Controller(WeightedQPTrackingController):
    """Run one approach and stop after the reference's final hold."""

    def __init__(self, robot, params, dt):
        wall = params['wall_geometry']
        reference = PerpendicularApproachReference(
            robot.position(params['initial_q']), wall['point'], wall['normal'],
            **params.get('approach', {}),
        )
        super().__init__(robot, params, dt, reference, default_damping=12.0,
                         default_ramp=0.1, default_mass_gain=4.0)

    def update(self, t, q, dq):
        tau = super().update(t, q, dq)
        ref = self.reference
        trial_complete = not self.done and t >= ref.end_time
        if trial_complete:
            self.done = True
        distance = float(ref.direction @ (self.log['p_ref'] - ref.start_position))
        self.log.update({
            'p_initial': ref.start_position, 'approach_distance_ref': distance,
            'reference_depth': distance - ref.gap, 'approach_complete': float(t >= ref.motion_end_time),
            'trial_complete': float(trial_complete),
        })
        return tau
