"""Example 1: joint-space PD around the start pose, with joint 4 moving.

    ros2 launch fr3_student_sim student.launch.py controller:=examples/joint_pd.py wall:=false

tau = Kp (q_d - q) + Kd (dq_d - dq)       (no g(q): the simulator adds it)
"""
import numpy as np

from fr3_student_sim.controller_base import StudentController

KP = np.array([600.0, 600.0, 600.0, 600.0, 250.0, 150.0, 50.0])   # [N m/rad]
KD = np.array([50.0, 50.0, 50.0, 30.0, 25.0, 25.0, 15.0])          # [N m s/rad]


class Controller(StudentController):

    def __init__(self, robot, params, dt):
        super().__init__(robot, params, dt)
        self.q0 = np.array(params['initial_q'])

    def update(self, t, q, dq):
        # joint 4 oscillates by 0.3 rad with a 4 s period
        w = 2 * np.pi / 4.0
        q_d, dq_d = self.q0.copy(), np.zeros(7)
        q_d[3] += 0.3 * np.sin(w * t)
        dq_d[3] = 0.3 * w * np.cos(w * t)

        self.log['q_d'] = q_d
        return KP * (q_d - q) + KD * (dq_d - dq)
