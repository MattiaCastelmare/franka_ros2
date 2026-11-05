# Progress: FR3 simulation experiments

## 2026-11-04

### Objective

The 7-joint FR3 already configured in the repository tracks a circle in the
x-z plane, with a radius of 4 cm, a steady-state period of 8 s and a startup
ramp of 1 s. Only the position of `fr3_fingertip`, expressed in the robot
base frame, is controlled. The gripper orientation is free. The launch command
below enables the compliant wall selected by `config/wall.yaml`.

The current wall is the vertical plane at $x=0.62$ m, with outward normal
$n=(-1,0,0)$, stiffness 20000 N/m and damping 15 N s/m. The circle starts
near $x=0.56$ m and reaches $x\approx0.64$ m, giving approximately 20 mm of
maximum reference intrusion into that plane. This is reference geometry,
not a prescribed actual penetration. With the current period and startup
ramp, the reference first reaches its farthest $+x$ point at
$(8+1)/2=4.5$ s after RUN starts.

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

Gravity is already compensated by the simulator. The minimal controller
returns the torque above through the existing student-controller interface;
the runner handles saturation, wall interaction, monitoring and log saving.

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

Use `wall:=false` in the same command to remove the wall from the experiment.

By default, the launch opens the Gazebo window and sets a maximum RUN
duration of 20 simulated seconds after settling, a nominal control rate of
1000 Hz and a real-time factor of 0.5. A controller's `done` flag, an existing
safety stop, an exception or `Ctrl+C` can end the run earlier. The optional
`stop_at_detachment` setting, disabled by default, can also end it at the
first detachment. In each new shell, run
`source /ros2_ws/install/setup.bash`. Changes to files already installed with
`--symlink-install` only require restarting the simulation.

### Where to make changes

Select each controller with its matching `task_config` YAML. The minimal
circular controller is the baseline; additional laws use separate controller
files and configurations. The circle reference is shared by the two circular
variants, while the straight experiment has its own reference module. Edit
the selected YAML to change its gains or trajectory settings.

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
the saved files. This damping variant reuses the baseline controller and
circle reference, and uses the shared runner, wall and collision monitor.

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

With the default collision monitoring enabled, the terminal prints the
selected log, detection times and the two PNG paths. The images are saved
next to the `.npz` file; the observer figure ends in `_observer.png`.
With `observer:=false`, only the tracking/contact figure is produced.

To also open the graph windows, run this in the same terminal:

```bash
ros2 run fr3_student_sim plot_run "$latest_circle_log"
```

### Straight approach with directional mass minimization

A separate experiment has been added in
`students/straight_mass_minimization/controller.py`. It controls the three
coordinates of `fr3_fingertip`; the gripper orientation is free. Its reference
and mass objective are separate modules,
`straight_line_reference.py` and `mass_objective.py`, and its parameters are
in `config/straight_mass_minimization.yaml`. The circular controllers and
their configurations are unchanged.

The runner now passes the controller an independent copy of the selected
wall's static `point` and `normal` through `self.params['wall_geometry']`.
The variant uses this geometry without changing it, including when
`wall:=false`. The wall object, its true force and the collision-detector
output are not supplied to the controller.

The reference starts from the configured initial fingertip position and
moves along the straight direction $s=-n$, where $n$ is the wall's unit
outward normal in the robot base frame. With the current vertical wall this
is the base $+x$ direction. A 0.5 s smooth ramp increases the speed from zero
to 0.02 m/s. The reference crosses the wall plane at that speed, then brakes
smoothly over a further 0.001 m and holds the endpoint for 0.2 s. These
settings come from `config/straight_mass_minimization.yaml`; distance to the
plane comes from the selected `wall_config` and the configured initial pose.

The scheduled end is computed from those settings rather than fixed at a
constant duration. Let $D=n^\top(p_0-p_{\mathrm{wall}})$, $v$ be
`line.approach_speed`, $T_r$ be `line.ramp_time`, $\delta_{\mathrm{ref}}$ be
`line.penetration`, and $T_h$ be `line.hold_time`. For the supported approach
with $D\geq vT_r/2$, the implemented reference timing is:

$$
\begin{aligned}
T_{\mathrm{plane}} &= \frac{D}{v}+\frac{T_r}{2}, \\
T_{\mathrm{brake}} &= \frac{2\delta_{\mathrm{ref}}}{v}, \\
T_{\mathrm{done}} &=
   \frac{D}{v}+\frac{T_r}{2}+\frac{2\delta_{\mathrm{ref}}}{v}+T_h.
\end{aligned}
$$

With the current plane at $x=0.62$ m and initial fingertip near $x=0.56$ m,
$D\approx0.06$ m: the reference reaches the plane at about 3.25 s, brakes
for 0.1 s and sets `done` at about 3.55 s, counted from RUN start. Changing
the wall or line configuration changes this schedule. The launch's default
20 s duration remains an upper bound; existing stop conditions can end the
run earlier. The scheduled `done` does not depend on collision detection or
actual contact. Speed, penetration and plane-crossing time describe the
reference, not guaranteed values of the robot's actual contact motion.

The controller keeps Cartesian tracking, nullspace velocity damping and the
mass-gradient torque as explicit contributions. Here $J$ is the $3\times7$
position Jacobian, $M$ the joint inertia matrix, and $c$ the Coriolis and
centrifugal torque vector:

$$
\begin{aligned}
a &= \ddot{p}_{\mathrm{ref}}
   + K_d\left(\dot{p}_{\mathrm{ref}}-J\dot q\right)
   + K_p\left(p_{\mathrm{ref}}-p\right), \\
N &= I-J^{\dagger}J, \\
\ddot q_{\mathrm{track}} &=
   J^{\dagger}\left(a-\dot J\dot q\right)-k_N N\dot q, \\
\tau_{\mathrm{track}} &= M\ddot q_{\mathrm{track}}+c(q,\dot q), \\
m(q,n) &= \frac{1}{n^\top J M^{-1}J^\top n}, \\
\bar J &= M^{-1}J^\top\left(JM^{-1}J^\top\right)^{-1}, \\
N_\tau &= I-J^\top\bar J^\top, \\
\tau_{\mathrm{mass}} &= -k_m N_\tau\nabla_q m(q,n), \\
\tau &= \tau_{\mathrm{track}}+\tau_{\mathrm{mass}}, \\
\ddot q_{\mathrm{cmd}} &=
   \ddot q_{\mathrm{track}}+M^{-1}\tau_{\mathrm{mass}}.
\end{aligned}
$$

The existing Pinocchio model evaluates mass in the fixed wall-normal
direction at the controlled fingertip. The gradient uses symmetric finite
differences with a step of $10^{-4}$ rad; it is neither normalized nor
clipped. Each update evaluates mass 15 times: once at the current configuration
and twice per joint for the gradient.
The mass gain starts at `mass_minimization.gain: 0.01`
[N m rad/kg], and `tracking.kd_nullspace` remains 4 [1/s]. For positive-definite
$M$ and full-row-rank $J$, the torque projector satisfies
$JM^{-1}\tau_{\mathrm{mass}}=0$ before saturation. The runner applies its
existing saturation to the requested total torque; gravity remains compensated
by the simulator. No additional posture, joint-limit or contact control is introduced.

The new user signals are saved automatically and displayed by the existing
plot command: `user_mass_kg`, `user_mass_gradient`, `user_tau_mass`,
`user_normal_speed` and `user_normal_speed_ref`. Normal speed is positive
towards the wall, along $s=-n$. `user_tau_mass` is the requested mass
contribution before saturation of the total torque; it is not a separate
measurement of applied torque. `user_qdd_command` contains the total requested
acceleration, and the tracking, nullspace and observer signals remain logged.

Setting `mass_minimization.gain` to zero in the same YAML removes only the
mass torque, providing the straight-tracking-and-damping baseline for a future
comparison. Mass and gradient are still evaluated and logged with zero gain.
The mass objective does not guarantee decreasing mass throughout
the run, avoidance of joint limits or a reduction of contact force.

### Run the straight mass-minimization controller

Inside the container, build once to install the added files:

```bash
cd /ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select fr3_student_sim
source /ros2_ws/install/setup.bash

ros2 launch fr3_student_sim student.launch.py \
  controller:=straight_mass_minimization/controller.py \
  task_config:=/ros2_ws/src/fr3_student_sim/config/straight_mass_minimization.yaml \
  run_name:=straight_mass_minimization \
  log_dir:=/ros2_ws/src/fr3_student_sim/runs \
  wall:=true
  control_rate:=1000 \
  real_time_factor:=0.20
```

Set real_time_factor to not lose logs.

After the run, save the tracking and observer figures for its latest log:

```bash
source /ros2_ws/install/setup.bash
cd /ros2_ws/src/fr3_student_sim

latest_straight_log=$(ls -t runs/straight_mass_minimization_*.npz | head -n 1)
ros2 run fr3_student_sim plot_run "$latest_straight_log" --no-show
```

### Collision-monitoring responsibilities

Collision monitoring is now coordinated by `fr3_student_sim/collision_monitor.py`.
Its `CollisionMonitor` loads the existing observer configuration, owns the
momentum observer and collision detector, and collects their output samples.
The observer and detector retain their separate mathematical implementations;
the monitor has no ROS or tracking-controller dependency.

For every received RUN joint-state measurement, the runner passes simulation
time, measured joint positions and velocities, and the reconstructed applied
actuator command to `CollisionMonitor.update`. Measurement noise is included
when configured. This update follows received samples, independently of the
tracking controller's nominal update period; 1000 received samples per second
are not guaranteed.

`AppliedTorqueHistory` reconstructs the observer input from the command
history after RUN torque clipping and before the virtual-wall contribution
is added. The resulting `tau_applied` excludes both the wall torque and the
gravity compensation added by the simulator. The history assumes the runner's
1 ms transport delay and averages the held commands over the elapsed
measurement interval, including when samples are missed. This is command
reconstruction, not measured motor-torque
telemetry from `/joint_states.effort`. The monitor returns a message only at
a detection onset, and the runner logs it.

The monitor exports the existing residual, momentum, threshold and detection
fields for the NPZ log, together with the existing observer metadata. When a
run is interrupted between monitoring and plant logging, it exports only the
monitoring samples corresponding to saved plant rows and derives detection
times from that same prefix.

The runner retains ROS communication, simulation clock and phases, actuator
history, delay alignment, measurement noise, controller execution, torque
saturation, safety stops and file I/O. No ROS node or launch argument was
added by this refactor; launch selection, controller laws, observer equations,
detector rules and plotting are reused.

### Momentum observer and collision detection

The implemented observer uses Pinocchio's inertia matrix and Coriolis matrix.
Writing $p_m$ for joint momentum, distinct from Cartesian position $p$, its
equations in the gravity-compensated simulator are:

$$
\begin{aligned}
p_m &= M(q)\dot q, \\
\hat p_m(t_0) &= p_m(t_0), \\
r &= K_O\left(p_m-\hat p_m\right), \\
\dot{\hat p}_m &=
   \tau_{\mathrm{applied}}+C(q,\dot q)^\top\dot q+r.
\end{aligned}
$$

The simulator adds gravity compensation to the actuator command, so the
observer uses the equivalent drive without an extra gravity term.
`coriolis_matrix` supplies $C$ with $\dot M=C+C^\top$; the observer uses
$C^\top\dot q$, not the vector $c=C\dot q$. It initializes from the first
RUN measurement even if velocity is nonzero, returns zero initial residual,
and then integrates with implicit Euler using elapsed simulation time. True
wall force, true external torque and joint accelerations are not observer
inputs.

`config/observer.yaml` currently sets `gain: 30.0` [1/s] and `threshold: 0.1`
[N m]. Each setting accepts a scalar shared by all seven joints or seven
positive joint-specific values. The separate detector reports a collision
when any $|r_i|$ exceeds its joint threshold, and marks each false-to-true
transition as a detection event. The residual norm is logged and plotted as
a diagnostic; it is not the detector's threshold signal. Detection does not
change the commanded torque or stop the run.

Logs contain the seven residuals, their Euclidean norm, measured-state and
estimated momentum, joint threshold flags, collision flags, event samples,
detection times and configured residual thresholds. With monitoring enabled,
the plot command produces seven joint-residual panels, the residual norm
and a detection timeline in the separate `_observer.png` figure, and marks
event times on the tracking/contact figure.

### Configuration files in use

| Configuration                                     | Role                                                                                                                                                                                                |
| ------------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `config/task.yaml`                              | Default task when `task_config` is omitted. Its top-level `Kp` and `Kd` fields are not read by the supplied controllers; the circular and straight variants read their `tracking` mappings. |
| `config/circle_tracking.yaml`                   | Initial pose, circular reference and Cartesian gains for the minimal circular controller.                                                                                                           |
| `config/circle_tracking_nullspace_damping.yaml` | Matching circular settings plus `tracking.kd_nullspace` for the damping variant.                                                                                                                  |
| `config/straight_mass_minimization.yaml`        | Initial pose, straight reference, Cartesian/nullspace gains and mass-objective settings for the straight variant.                                                                                   |
| `config/wall.yaml`                              | Shared plane geometry and contact-model stiffness/damping, selected with `wall_config`.                                                                                                           |
| `config/observer.yaml`                          | Momentum-observer gain and per-joint residual thresholds, selected with `observer_config`.                                                                                                        |
| `config/controllers.yaml`                       | Joint list for the Gazebo effort controller; it does not set the Python tracking gains.                                                                                                             |
| `config/batch_example.yaml`                     | Batch defaults and per-run overrides for `run_batch`; these defaults are separate from the launch defaults.                                                                                       |
