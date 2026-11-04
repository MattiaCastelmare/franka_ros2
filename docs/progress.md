# Progress: initial circular tracking simulation

## 2026-11-04

### Objective

The 7-joint FR3 already configured in the repository tracks a circle in the
x-z plane, with a radius of 4 cm, a steady-state period of 8 s and a startup
ramp of 1 s. Only the position of `fr3_fingertip`, expressed in the robot
base frame, is controlled. The gripper orientation is free; the wall is
disabled in the launch command below.

### Simulation files

| File | Responsibility |
| --- | --- |
| `fr3_student_sim/students/circle_tracking/controller.py` | Cartesian position tracking, velocity damping in the Jacobian nullspace and computation of the seven joint torques. |
| `fr3_student_sim/students/circle_tracking/reference.py` | New `CircleReference` class: desired circle position, velocity and acceleration, with a smooth startup. |
| `fr3_student_sim/config/circle_tracking.yaml` | Initial joint configuration, radius, period, ramp and controller gains. |

The controller follows the lecture equations, with `J` the 3x7 position
Jacobian and `J_pinv` its Moore-Penrose pseudoinverse:

```text
a = p_ddot_ref + Kd * (p_dot_ref - J * dq) + Kp * (p_ref - p)
N = I - J_pinv * J
qdd_command = J_pinv * (a - Jdot * dq) - Kd_nullspace * N * dq
tau = M(q) * qdd_command + c(q, dq)
```

The Cartesian feedback linearization and acceleration PD are from
`docs/lectures/13_CartesianControl.pdf`, slides 8 and 11. The pseudoinverse,
nullspace acceleration and velocity damping are from
`docs/lectures/02_KinematicRedundancy_2.pdf`, slides 7 and 13. The damping
coefficient remains 4 1/s; it damps internal motion without attracting the
joints to a desired posture. Orientation control, posture attraction and
the damped Jacobian inverse have been removed.

Gravity is already compensated by the simulator. The existing runner,
model, trajectory reference and launch file are reused without changes.

### Running the simulation

From the Ubuntu/WSL terminal:

```bash
docker start franka_ros2
docker exec -it franka_ros2 bash
```

Inside the container, after adding or removing package files:

```bash
cd /ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select fr3_student_sim
source /ros2_ws/install/setup.bash
```

To start circular tracking:

```bash
ros2 launch fr3_student_sim student.launch.py \
  controller:=circle_tracking/controller.py \
  task_config:=/ros2_ws/src/fr3_student_sim/config/circle_tracking.yaml \
  wall:=false
```

The arguments select the new controller and its configuration without changing
the launch defaults. Running `student.launch.py` without arguments still
selects the existing `my_controller.py` skeleton.

Use `wall:=true` in the same command to include the existing compliant wall.

By default, the launch opens the Gazebo window, runs for 20 simulated seconds
after settling, uses a control rate of 1000 Hz and a real-time factor of 0.5.
Press `Ctrl+C` to interrupt the simulation. In each new shell, run
`source /ros2_ws/install/setup.bash`. Changes to files already installed with
`--symlink-install` only require restarting the simulation.

### Validation of the simplified controller

The 13 existing offline tests pass in the ROS container. At a displaced,
moving robot state, the model's Cartesian acceleration matches the lecture
PD equation to numerical precision. A 20 s numerical integration with the
nominal robot dynamics, 1 ms steps and no wall gives a maximum position error
of 0.0044 mm, without torque saturation or joint-limit violations.

A headless Gazebo run with the default wall makes contact and reaches a peak
normal force of 11.14 N. The existing safety stop ends the run at 3.96 s:
joint 6 reaches 4.468 rad, within the 0.05 rad margin from its upper limit.
The position-only task does not constrain the orientation or restore the
initial posture during contact. No contact controller or joint-limit
avoidance has been added.

### Where to make changes

Edit `controller.py` to change the control law, `reference.py` to change the
trajectory reference and `config/circle_tracking.yaml` to change the parameters.
The existing infrastructure files and methods remain unchanged.
