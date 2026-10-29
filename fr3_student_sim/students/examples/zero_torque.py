"""Example 0: command zero torque.

The simulator adds g(q) to every command (like the real FR3), so with zero
torque the arm floats where it is. If it falls or rises, something is wrong.

    ros2 launch fr3_student_sim student.launch.py controller:=examples/zero_torque.py wall:=false
"""
import numpy as np

from fr3_student_sim.controller_base import StudentController


class Controller(StudentController):

    def update(self, t, q, dq):
        self.log['q_error'] = np.linalg.norm(q - np.array(self.params['initial_q']))
        return np.zeros(7)
