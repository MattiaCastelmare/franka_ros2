"""Base class for the students' controllers.

A controller is a Python file that defines ``class Controller(StudentController)``
and overrides :meth:`update`. The runner calls ``update`` at ``control_rate`` Hz
with the current simulation time and joint state, and sends the returned torque
to the robot. See ``students/my_controller.py``.
"""
from __future__ import annotations

import numpy as np

from fr3_student_sim.fr3_model import FR3Model


class StudentController:
    """What the runner gives you, as attributes:

    robot        FR3Model of the simulated robot (kinematics, M, C, g, m_eff ...)
    params       dict loaded from the task YAML (``config/task.yaml`` by default)
    dt           [s] control period (1 / control_rate)
    tau_applied  (7,) torque that acted on the robot since your previous update:
                 your commands clipped to TAU_MAX, with the simulator's 1 ms
                 transport delay accounted for. Use it in a momentum observer.
    log          dict: every entry you put here is saved at each step
                 (scalars or arrays of fixed size) and can be plotted.
    done         set it to True to end the run early (e.g. at first detachment).
    """

    def __init__(self, robot: FR3Model, params: dict, dt: float):
        self.robot = robot
        self.params = params
        self.dt = dt
        self.tau_applied = np.zeros(7)
        self.log = {}
        self.done = False

    def update(self, t: float, q: np.ndarray, dq: np.ndarray) -> np.ndarray:
        """Return the (7,) joint torque [N m] to command, WITHOUT gravity.

        t   [s] time since the start of the run
        q   (7,) joint positions [rad]
        dq  (7,) joint velocities [rad/s]
        """
        return np.zeros(7)

