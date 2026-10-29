"""Example 2: the fingertip follows the circle with a Cartesian PD (a "virtual spring").

    ros2 launch fr3_student_sim student.launch.py controller:=examples/cartesian_pd.py wall:=false

    F   = K (p_d - p) + D (v_d - v)         force pulling the fingertip to the reference
    tau = J^T F - D_joint dq                (no g: the simulator adds it)

No dynamic model is used, so the tracking is only approximate (a few mm), and
the joint damping is what keeps the arm from drifting in its null space.
"""
import numpy as np

from fr3_student_sim.controller_base import StudentController

K = 3000.0        # [N/m]
D = 150.0         # [N s/m]
D_JOINT = 2.0     # [N m s/rad]


class Controller(StudentController):

    def __init__(self, robot, params, dt):
        super().__init__(robot, params, dt)
        self.p_start = robot.position(params['initial_q'])
        self.r = params['circle']['radius']
        self.w = 2 * np.pi / params['circle']['period']
        self.T_ramp = params['circle'].get('ramp', 1.0)

    def reference(self, t):
        """Circle in the x-z plane, from its -x point (the start) towards +x (the wall).
        The angular speed ramps up smoothly over T_ramp (no jump at t = 0)."""
        r, w, Tr = self.r, self.w, self.T_ramp
        if t < Tr:
            u = t / Tr
            th, dth = w * Tr * (u ** 3 - u ** 4 / 2), w * (3 * u ** 2 - 2 * u ** 3)
        else:
            th, dth = w * (t - Tr / 2), w
        c = self.p_start + np.array([r, 0.0, 0.0])
        p = c + r * np.array([-np.cos(th), 0.0, np.sin(th)])
        v = r * dth * np.array([np.sin(th), 0.0, np.cos(th)])
        return p, v

    def update(self, t, q, dq):
        p_d, v_d = self.reference(t)
        J = self.robot.jacobian_pos(q)
        p, v = self.robot.position(q), J @ dq

        F = K * (p_d - p) + D * (v_d - v)
        tau = J.T @ F - D_JOINT * dq

        self.log['p_ref'] = p_d
        return tau
