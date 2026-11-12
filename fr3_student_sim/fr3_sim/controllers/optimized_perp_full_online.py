"""Perpendicular impedance/HQP with online mass minimization and no posture target."""
from __future__ import annotations

from fr3_sim.controllers.optimized_hqp_perp import Controller as PerpendicularHQPController


class Controller(PerpendicularHQPController):
    """Reuse the perpendicular controller while omitting postural attraction.

    ``initial_q`` still defines settling and the Cartesian reference start.
    It does not become a secondary posture reference. Ignore any externally
    supplied ``q_posture_ref`` so this variant always remains fully online.
    The mass-gradient term, nullspace damping and all constraints are retained.
    """

    def __init__(self, robot, params, dt):
        online_params = dict(params)
        online_params.pop('q_posture_ref', None)
        super().__init__(robot, online_params, dt)
