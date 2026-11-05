# Progress: initial circular tracking simulation

## 2026-11-04

### Objective

The 7-joint FR3 already configured in the repository tracks a circle in the
x-z plane, with a radius of 4 cm, a steady-state period of 8 s and a startup
ramp of 1 s. Only the position of `fr3_fingertip`, expressed in the robot
base frame, is controlled. The gripper orientation is free; the wall is
disabled in the launch command below.

### Simulation files

| File                                                       | Responsibility                                                                                               |
| ---------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------ |
| `fr3_student_sim/students/circle_tracking/controller.py` | Cartesian position tracking with minimum-norm joint acceleration and computation of the seven joint torques. |
| `fr3_student_sim/students/circle_tracking/reference.py`  | New `CircleReference` class: desired circle position, velocity and acceleration, with a smooth startup.    |
| `fr3_student_sim/config/circle_tracking.yaml`            | Initial joint configuration, radius, period, ramp and controller gains.                                      |

The controller follows:

$$
\begin{aligned}
a &= \ddot{p}_{\mathrm{ref}}
   + K_d\left(\dot{p}_{\mathrm{ref}} - J\dot{q}\right)
   + K_p\left(p_{\mathrm{ref}} - p\right), \\
\ddot{q}_{\mathrm{cmd}} &= J^{\dagger}\left(a - \dot{J}\dot{q}\right), \\
\tau &= M(q)\ddot{q}_{\mathrm{cmd}} + c(q,\dot{q}).
\end{aligned}
$$

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
  wall:=true
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

### Where to make changes

Edit `controller.py` to change the control law, `reference.py` to change the
trajectory reference and `config/circle_tracking.yaml` to change the parameters.
The existing infrastructure files and methods remain unchanged.

## 2026-11-05

### Controller variants

The minimal controller in `students/circle_tracking/controller.py` is kept
as the reference. A new variant,
`students/circle_tracking/controller_nullspace_damping.py`, reuses it and
adds only velocity damping in the position Jacobian's null space:

$$
\begin{aligned}
N &= I - J^{\dagger}J, \\
\ddot{q}_{\mathrm{null}} &= -k_N N\dot{q}, \\
\ddot{q}_{\mathrm{cmd}} &= \ddot{q}_{\mathrm{minimal}} + \ddot{q}_{\mathrm{null}}, \\
\tau &= \tau_{\mathrm{minimal}} + M(q)\ddot{q}_{\mathrm{null}}.
\end{aligned}
$$

Its separate configuration is `config/circle_tracking_nullspace_damping.yaml`.
Initial pose, circle and Cartesian gains match the minimal configuration;
the added setting is `tracking.kd_nullspace`, initially 4 1/s. Setting it to
zero recovers the minimal command. The variant logs the nullspace velocity
and damping acceleration, and includes the latter in the total logged
acceleration command. It adds no posture objective or joint-limit avoidance.

The existing launch selects either controller through its `controller`
argument. Use `run_name:=circle_minimal` for the reference and
`run_name:=circle_nullspace_damping` for the damping variant to distinguish
the saved files. No changes were made to the baseline controller, reference,
observer, runner or wall for this addition.

### Run the nullspace damping controller

Inside the container, after building the package as described above:

```bash
source /ros2_ws/install/setup.bash

ros2 launch fr3_student_sim student.launch.py \
  controller:=circle_tracking/controller_nullspace_damping.py \
  task_config:=/ros2_ws/src/fr3_student_sim/config/circle_tracking_nullspace_damping.yaml \
  run_name:=circle_nullspace_damping \
  log_dir:=/ros2_ws/src/fr3_student_sim/runs \
  wall:=true
```

The log is saved in `/ros2_ws/src/fr3_student_sim/runs/` with a name starting
with `circle_nullspace_damping_` and ending in `.npz`.

### Plot the latest nullspace damping run

After the run finishes, these commands automatically select its most recent
log and save the tracking and observer PNGs without opening Linux windows:

```bash
source /ros2_ws/install/setup.bash
cd /ros2_ws/src/fr3_student_sim

latest_circle_log=$(ls -t runs/circle_nullspace_damping_*.npz | head -n 1)
ros2 run fr3_student_sim plot_run "$latest_circle_log" --no-show
```

The terminal prints the selected log, detection times and the two PNG paths.
The images are saved next to the `.npz` file; the observer figure ends in
`_observer.png`. There is no placeholder filename to replace in this command.

To also open the graph windows, run this in the same terminal:

```bash
ros2 run fr3_student_sim plot_run "$latest_circle_log"
```
