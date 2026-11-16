"""Circular HQP with online mass descent and no offline posture target."""
from __future__ import annotations

from fr3_sim.controllers.optimized_hqp import Controller as CircularController


class Controller(CircularController):
    """Keep the shared HQP and circle reference, disabling offline attraction.

    Mass and its analytical gradient are evaluated at the current state.
    Tracking, damping, startup ramp, observer handling and constraints are
    inherited unchanged. Custom offline posture settings are ignored without
    modifying the caller's parameters.
    """

    def __init__(self, robot, params, dt):
        online_params = dict(params)
        online_params.pop('q_posture_ref', None)
        online_params['tracking'] = dict(params.get('tracking', {}))
        online_params['tracking']['posture_gain'] = 0.0
        super().__init__(robot, online_params, dt)
