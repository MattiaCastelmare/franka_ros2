"""YOUR controller. This is the only file you need to edit.

Run it with
    ros2 launch fr3_student_sim student.launch.py
and look at the result with
    ros2 run fr3_student_sim plot_run runs/<the newest .npz>

What you get (see fr3_student_sim/controller_base.py):
    self.robot        the FR3 model:  position(q), pose(q), jacobian(q), jacobian_pos(q),
                      velocity(q, dq), jdot_dq(q, dq), mass_matrix(q), mass_matrix_inv(q),
                      coriolis_matrix(q, dq), coriolis(q, dq), gravity(q),
                      effective_mass(q, n), kinetic_energy(q, dq)
    self.params       the dict in config/task.yaml
    self.dt           nominal control period [s] (use t - t_previous for integrals)
    self.tau_applied  torque that acted on the robot since your last update
                      (your commands, clipped, 1 ms simulator delay included)
    self.log          put here anything you want saved and plotted
    self.done = True  ends the run

Rules of the simulated FR3 (same as the real robot):
  * the simulator ADDS g(q) to your torque: do NOT add gravity yourself;
  * torques are clipped to TAU_MAX = [87, 87, 87, 87, 12, 12, 12] N m;
  * the run stops (SAFETY STOP) if a joint gets within 0.05 rad of its limit
    or exceeds its speed limit DQ_MAX.

Dynamics you can rely on:   M(q) ddq + C(q, dq) dq = tau + tau_ext
(tau_ext is the wall force mapped to the joints: you cannot measure it).
"""
import numpy as np

from fr3_student_sim.controller_base import StudentController


class Controller(StudentController):

    def __init__(self, robot, params, dt):
        super().__init__(robot, params, dt)
        # TODO: initialise your variables here (gains, observer state, ...)

    def update(self, t, q, dq):
        # TODO: your control law. Return a (7,) torque in N m, without gravity.
        tau = np.zeros(7)
        return tau
