"""Perpendicular HQP with online mass descent and no offline posture target."""
from __future__ import annotations

from fr3_sim.controllers.optimized_hqp_perp import Controller as PerpendicularController


class Controller(PerpendicularController):
    """Reuse the perpendicular controller, disabling only posture attraction.

    The analytical mass gradient is evaluated at the current configuration by
    the shared HQP law. Nullspace damping, startup ramp, tracking, observer and
    constraints retain their configured behavior. An offline target supplied
    in a custom configuration is ignored without modifying the caller's data.
    """

    def __init__(self, robot, params, dt):
        online_params = dict(params)
        online_params.pop('q_posture_ref', None)
        online_params['tracking'] = dict(params.get('tracking', {}))
        online_params['tracking']['posture_gain'] = 0.0
        super().__init__(robot, online_params, dt)
