"""Circle experiment using the reusable impedance/HQP controller."""

from fr3_sim.controllers.cartesian_impedance import ImpedanceHQPController
from fr3_sim.methods.circle_reference import CircleReference


class Controller(ImpedanceHQPController):
    def __init__(self, robot, params, dt):
        circle = params.get('circle', {})
        reference = CircleReference(robot.position(params['initial_q']),
                                    radius=circle.get('radius', 0.04),
                                    period=circle.get('period', 4.0),
                                    ramp=circle.get('ramp', 1.0))
        super().__init__(robot, params, dt, reference)
