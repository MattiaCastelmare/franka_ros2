# Progress: FR3 simulation experiments

This document preserves the development history. The 2026-11-09 cleanup
retained baseline and hybrid retreat; the later optimized variant adds a
third active entry point. Earlier sections
describe the layout and defaults before that cleanup: archived `students/`,
`config/` and `test/` paths now live under `fr3_student_sim/trash/`, retaining
their relative structure. Shared tracking and reference implementations now
live in `students/utils/`. Use the current package README and the final
development entries below for the active files and commands.

The analytical-gradient implementation and timing reports added on
2026-11-10 supersede the finite-difference details in earlier entries.

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
circular controller remains the original reference; additional laws use separate
controller files and configurations. The circle reference is shared by the six circular
variants during tracking, while the straight experiment has its own reference
module. Edit the selected YAML to change its gains or trajectory settings.

## 2026-11-05

### Circular controller variants

The minimal controller in `students/circle_tracking/controller.py` is kept
as the reference. The six circular variants have separate controller files
and matching task configurations:

| Variant                                                     | Controller in `students/circle_tracking/` | Task configuration                                |
| ----------------------------------------------------------- | ------------------------------------------- | ------------------------------------------------- |
| Minimal position tracking                                   | `controller.py`                           | `config/circle_tracking.yaml`                   |
| Position tracking with nullspace velocity damping           | `controller_nullspace_damping.py`         | `config/circle_tracking_nullspace_damping.yaml` |
| Baseline with nullspace posture feedback and damping        | `controller_baseline.py`                  | `config/circle_tracking_baseline.yaml`          |
| Damped position tracking with directional mass minimization | `controller_mass_minimization.py`         | `config/circle_tracking_mass_minimization.yaml` |
| Mass-minimizing tracking with latched collision release     | `controller_hybrid.py`                    | `config/circle_tracking_hybrid.yaml`            |
| Mass-minimizing tracking with immediate horizontal retreat  | `controller_hybrid_retreat.py`            | `config/circle_tracking_hybrid_retreat.yaml`    |

The damping variant reuses the minimal controller and adds only velocity
damping in the position Jacobian's null space:

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

The existing launch selects the requested controller through its `controller`
argument. Use `run_name:=circle_minimal` for the reference and
`run_name:=circle_nullspace_damping` for the damping variant to distinguish
the saved files. This damping variant reuses the minimal reference controller and
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
selected log, detection times and the PNG paths. The images are saved
next to the `.npz` file; the observer figure ends in `_observer.png` and
the additional collision figure ends in `_collision.png`.
With `observer:=false`, the main and collision figures are produced.

To also open the graph windows, run this in the same terminal:

```bash
ros2 run fr3_student_sim plot_run "$latest_circle_log"
```

### Circular tracking with directional mass minimization

`students/circle_tracking/controller_mass_minimization.py` extends the
existing nullspace-damping variant. It keeps the same three-dimensional
position tracking, free gripper orientation, initial pose and circular
reference: radius 0.04 m, steady period 8 s and startup ramp 1 s. Its YAML,
`config/circle_tracking_mass_minimization.yaml`, keeps Cartesian gains
100 [1/s^2] and 20 [1/s] and nullspace damping 4 [1/s], and adds a mass gain
of 0.01 [N m rad/kg] and a symmetric-gradient step of $10^{-4}$ rad.

The mass direction $n$ is the selected wall's fixed unit outward normal in
the robot base frame. It stays fixed as the circular velocity changes
direction or reaches zero; it is not derived from that velocity. The new
variant adds the following torque to the existing damped circular command:

$$
\begin{aligned}
m(q,n) &= \frac{1}{n^\top J M^{-1}J^\top n}, \\
\bar J &= M^{-1}J^\top\left(JM^{-1}J^\top\right)^{-1}, \\
N_\tau &= I-J^\top\bar J^\top, \\
\tau_{\mathrm{mass}} &= -k_m N_\tau\nabla_q m(q,n), \\
\tau_{\mathrm{circle,mass}} &=
   \tau_{\mathrm{circle,damped}}+\tau_{\mathrm{mass}}, \\
\ddot q_{\mathrm{cmd}} &=
   \ddot q_{\mathrm{circle,damped}}+M^{-1}\tau_{\mathrm{mass}}.
\end{aligned}
$$

The mass evaluation, central-difference gradient and dynamic torque
projection now live in the shared `students/utils/mass_objective.py` module.
The controller loader makes the `students/` root importable, so the mass
controllers use `from utils.mass_objective import ...`. This relocation preserves the
shared mathematics and both controllers' mass computations and control laws.

This circular variant does not add a scheduled `done`, braking phase or
detector-triggered reaction. It uses the shared circular reference and the
existing RUN duration and stop conditions. The default 20 s remains a maximum
duration; existing safety stops, exceptions, optional first detachment or
`Ctrl+C` can end the run earlier. With the current wall at $x=0.62$ m, the
circle still has approximately 20 mm of nominal reference intrusion. The
mass objective does not guarantee avoiding joint limits or reducing contact
force, and it uses no true wall force. Collision monitoring remains independent.

The variant logs `user_mass_kg`, `user_mass_gradient`, `user_tau_mass`,
`user_normal_speed` and `user_normal_speed_ref`, together with the inherited
tracking and nullspace signals. `user_tau_mass` is the requested contribution
before saturation of the total torque, and `user_qdd_command` includes the
mass contribution. Both normal-speed signals are projections along $-n$:
positive towards the wall and negative away from it. Setting
`mass_minimization.gain: 0.0` in the same YAML recovers the nominal damped
circular command; mass and gradient are still computed and logged.

### Run the circular mass-minimization controller

After rebuilding the package for the added files as described above, run
inside the container:

```bash
source /ros2_ws/install/setup.bash

ros2 launch fr3_student_sim student.launch.py \
  controller:=circle_tracking/controller_mass_minimization.py \
  task_config:=/ros2_ws/src/fr3_student_sim/config/circle_tracking_mass_minimization.yaml \
  run_name:=circle_mass_minimization \
  log_dir:=/ros2_ws/src/fr3_student_sim/runs \
  wall:=true \
  control_rate:=1000 \
  real_time_factor:=0.20
```

The command keeps a nominal 1000 Hz controller rate and requests a slower
real-time factor to leave more computation time for the mass gradient. These
are per-run arguments; the launch defaults remain unchanged. A slower
real-time factor does not guarantee that every joint-state sample is received.

After the run, select its latest log and save the tracking and observer PNGs:

```bash
source /ros2_ws/install/setup.bash
cd /ros2_ws/src/fr3_student_sim

latest_circle_mass_log=$(ls -t runs/circle_mass_minimization_*.npz | head -n 1)
ros2 run fr3_student_sim plot_run "$latest_circle_mass_log" --no-show
```

### Straight approach with directional mass minimization

A separate experiment has been added in
`students/straight_mass_minimization/controller.py`. It controls the three
coordinates of `fr3_fingertip`; the gripper orientation is free. Its reference
is `students/straight_mass_minimization/straight_line_reference.py`; its mass
objective uses the shared `students/utils/mass_objective.py`. Its parameters
are in `config/straight_mass_minimization.yaml`. The minimal and damped circular
controllers retain their existing files and configurations.

The runner now passes the controller an independent copy of the selected
wall's static `point` and `normal` through `self.params['wall_geometry']`.
The variant uses this geometry without changing it, including when
`wall:=false`. The wall object and its true force are not supplied to the
controller. The straight variant does not use collision notifications to
change its law or scheduled endpoint.

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
  wall:=true \
  control_rate:=1000 \
  real_time_factor:=0.20
```

The per-run `real_time_factor:=0.20` request leaves more computation time
for the mass gradient while keeping the nominal controller rate at 1000 Hz.
It does not change launch defaults or guarantee reception of every sample.

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
a diagnostic; it is not the detector's threshold signal. The monitor itself
does not command torque or stop the run. Controllers without a reaction
policy continue their existing law; the hybrid variants described below
use the first-detection notification to select RELEASE or RETREAT.

Logs contain the seven residuals, their Euclidean norm, measured-state and
estimated momentum, joint threshold flags, collision flags, event samples,
detection times and configured residual thresholds. With monitoring enabled,
the plot command produces seven joint-residual panels, the residual norm
and a detection timeline in the separate `_observer.png` figure, and marks
event times on the tracking/contact figure.

### Configuration files before the cleanup

| Configuration                                     | Role                                                                                                                                                                                                |
| ------------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `config/task.yaml`                              | Default task when `task_config` is omitted. Its top-level `Kp` and `Kd` fields are not read by the supplied controllers; the circular and straight variants read their `tracking` mappings. |
| `config/circle_tracking.yaml`                   | Initial pose, circular reference and Cartesian gains for the minimal circular controller.                                                                                                           |
| `config/circle_tracking_nullspace_damping.yaml` | Matching circular settings plus `tracking.kd_nullspace` for the damping variant.                                                                                                                  |
| `config/circle_tracking_baseline.yaml`          | Matching circular settings with nullspace posture gain 25 [1/s^2] and damping 10 [1/s] for the baseline variant.                                                                                    |
| `config/circle_tracking_mass_minimization.yaml` | Matching damped circular settings plus the fixed-wall-normal mass gain and gradient step for the circular mass-minimization variant.                                                                |
| `config/circle_tracking_hybrid.yaml`            | Matching mass-minimizing circular TRACK settings plus seven joint-torque damping coefficients for the latched RELEASE phase.                                                                        |
| `config/circle_tracking_hybrid_retreat.yaml`    | Matching mass-minimizing circular TRACK settings plus horizontal retreat distance and reference duration for the latched RETREAT phase.                                                             |
| `config/straight_mass_minimization.yaml`        | Initial pose, straight reference, Cartesian/nullspace gains and mass-objective settings for the straight variant.                                                                                   |
| `config/wall.yaml`                              | Shared plane geometry and contact-model stiffness/damping, selected with `wall_config`.                                                                                                           |
| `config/observer.yaml`                          | Momentum-observer gain and per-joint residual thresholds, selected with `observer_config`.                                                                                                        |
| `config/controllers.yaml`                       | Joint list for the Gazebo effort controller; it does not set the Python tracking gains.                                                                                                             |
| `config/batch_example.yaml`                     | Batch defaults and per-run overrides for `run_batch`; these defaults are separate from the launch defaults.                                                                                       |

## 2026-11-07

### Hybrid circular tracking and collision release

The new `students/circle_tracking/controller_hybrid.py` extends the circular
mass-minimization controller with two modes, TRACK and RELEASE. Its separate
configuration is `config/circle_tracking_hybrid.yaml`. TRACK reuses the same
position-only circular reference, free orientation, Cartesian feedback,
nullspace damping and projected mass-gradient torque. Initial pose, radius
0.04 m, steady period 8 s, ramp 1 s, Cartesian gains 100 [1/s^2] and 20 [1/s],
nullspace gain 4 [1/s], mass gain 0.01 [N m rad/kg] and gradient step
$10^{-4}$ rad match the circular mass-minimization configuration. The mass
direction remains the wall's fixed unit normal in the base frame.

`CollisionMonitor.set_detection_callback` delivers each detection-onset
timestamp to a registered callback. The runner calls
`controller.bind_collision_monitor(monitor)` once during initialization;
the base hook is a no-op. The hybrid controller overrides that hook to
register `on_collision` and owns `first_collision_time`: `None` until its
handler latches the first timestamp. Later detections do not replace it.
Callbacks are delivered on received RUN measurements, independently of the
controller update period, so an onset between updates is retained by the
controller. Its next update selects RELEASE. The switch is latched for the
run: a later decrease of the residual does not resume tracking. The callback
contains a detector timestamp, not true wall force or a true-contact flag.
The observer equations, per-joint threshold rule and observer NPZ fields are
unchanged.

Let $t_R$ be the first update that computes a RELEASE command. The requested
torque is:

$$
\begin{aligned}
D_R &= \operatorname{diag}(30,30,30,20,10,10,5)
       \quad [\mathrm{N\,m\,s/rad}], \\
\tau_{\mathrm{hybrid}}(t,q,\dot q) &=
\begin{cases}
\tau_{\mathrm{circle,mass}}(t,q,\dot q), & \mathrm{TRACK}, \\
-D_R\dot q, & \mathrm{RELEASE}.
\end{cases}
\end{aligned}
$$

`reaction.damping` supplies these seven positive coefficients, in joint
order; a positive scalar can also apply to every joint. RELEASE damps all
seven joint velocities directly. It does not use Cartesian tracking,
nullspace damping, mass torque or Coriolis compensation. $D_R$ is a joint
torque damping matrix in N m s/rad, distinct from the TRACK nullspace
acceleration gain `tracking.kd_nullspace` in 1/s. Gravity remains compensated
by the simulator, and the runner still clips the requested total torque.

The controller leaves `done` false. RELEASE continues until the launch
duration or an existing stop condition ends the run. With the command below,
the maximum RUN duration is 20 s and first-detachment stopping is disabled.
The wall, observer, detector, logging and existing safety stops remain active
while the runner stays in RUN. The hybrid policy adds no force feedback or
joint-limit avoidance, and an existing safety stop can still end the run early.

The hybrid controller's binding hook checks that monitoring is enabled
before registering its callback. It rejects `observer:=false` before
tracking starts. The runner performs only the common startup binding; it
does not copy detection state or select a reaction in its RUN loop. The
ordinary tracking controllers inherit the no-op hook and may still run with
monitoring disabled.

The user log adds `user_release_active` (0 in TRACK, 1 in RELEASE),
`user_release_start_time` (NaN in TRACK, then $t_R$) and `user_tau_release`
(seven zeros in TRACK, then the requested damping torque before saturation).
The release timestamp is the first command-computation time; it can follow
the first detection time when the controller runs less often than measurements.
The existing actuator history still accounts for command-application delay.

In RELEASE, `user_mass_kg` continues as a diagnostic with one mass evaluation
per update, and `user_normal_speed` continues along the fixed $-n$ direction.
`user_tau_mass` and `user_qdd_null` are zero because those control contributions
are inactive. The reference arrays `user_p_ref`, `user_v_ref`, `user_a_ref`,
tracking errors, `user_qdd_command`, `user_mass_gradient`, `user_dq_null` and
`user_normal_speed_ref` are NaN: these targets or diagnostics are inactive
and are not evaluated in RELEASE. The constant circle center and initial
position remain available. Existing plots display the new mode and torque
signals with the other user data.

### Run the hybrid controller

After rebuilding the package for the added files, launch inside the container:

```bash
source /ros2_ws/install/setup.bash

ros2 launch fr3_student_sim student.launch.py \
  controller:=circle_tracking/controller_hybrid.py \
  task_config:=/ros2_ws/src/fr3_student_sim/config/circle_tracking_hybrid.yaml \
  run_name:=circle_hybrid \
  log_dir:=/ros2_ws/src/fr3_student_sim/runs \
  wall:=true \
  observer:=true \
  stop_at_detachment:=false \
  control_rate:=1000 \
  real_time_factor:=0.50 \
  duration:=10
```

After the run, save the figures for its latest log:

```bash
source /ros2_ws/install/setup.bash
cd /ros2_ws/src/fr3_student_sim

latest_hybrid_log=$(ls -t runs/circle_hybrid_*.npz | head -n 1)
ros2 run fr3_student_sim plot_run "$latest_hybrid_log" --no-show
```

### Hybrid circular tracking with immediate horizontal retreat

The separate `students/circle_tracking/controller_hybrid_retreat.py` and
`config/circle_tracking_hybrid_retreat.yaml` add TRACK and RETREAT modes.
`students/circle_tracking/retreat_reference.py` supplies the finite quintic
position reference and its endpoint hold.
TRACK reuses the circular mass-minimization controller with the same initial
pose, circular reference, Cartesian feedback, nullspace damping and fixed
wall-normal mass objective. Its initial gains remain 100 [1/s^2] for position,
20 [1/s] for velocity, 4 [1/s] for nullspace damping and 0.01 [N m rad/kg]
for mass minimization, with a gradient step of $10^{-4}$ rad.

The first GMO detection selects RETREAT directly. The controller retains the
first onset through the same monitor callback mechanism as the release
variant. At its first update after detection, it captures the current
fingertip position and starts the retreat reference. There is no preceding
waiting phase or condition requiring the robot to stop. RETREAT stays latched
for the run, including after the reference reaches its goal; later detector
events do not restart the motion or resume the circle. The existing
`controller_hybrid.py` and its RELEASE law remain separate and unchanged.

Let $t_A$ be the first RETREAT command update, $p_A=p(q(t_A))$ the position
captured at that update, and $n$ the wall's fixed unit outward normal in the
base frame. The retreat direction is its normalized horizontal projection:

$$
\begin{aligned}
n_h &= \frac{(n_x,n_y,0)^\top}{\sqrt{n_x^2+n_y^2}}, \\
p_{\mathrm{goal}} &= p_A+d\,n_h, \\
u(t) &= \operatorname{clip}\!\left(\frac{t-t_A}{T_A},0,1\right), \\
s(u) &= 10u^3-15u^4+6u^5.
\end{aligned}
$$

`retreat.distance` supplies $d=0.03$ m and `retreat.duration` supplies
$T_A=2.0$ s. With the current wall, $n_h=(-1,0,0)$, so the goal is 3 cm
along $-x$ from the captured position, with unchanged reference height. The
reference during the motion is:

$$
\begin{aligned}
p_{\mathrm{ref}}(t) &= p_A+d\,n_h\,s(u), \\
\dot p_{\mathrm{ref}}(t) &=
  \frac{d}{T_A}n_h\left(30u^2-60u^3+30u^4\right), \\
\ddot p_{\mathrm{ref}}(t) &=
  \frac{d}{T_A^2}n_h\left(60u-180u^2+120u^3\right).
\end{aligned}
$$

Reference velocity and acceleration are zero at both endpoints. After
$t_A+T_A$, the endpoint is held with zero reference velocity and acceleration.
These are reference conditions; the actual robot velocity at the mode switch
is not reset. The retreat starts from the measured position without requiring
zero measured velocity.

RETREAT uses a dedicated instance of the existing damped position-tracking
controller with this reference, preserving that controller's law. Writing
$J$ for the fingertip position Jacobian, its requested
command is:

$$
\begin{aligned}
a_A &= \ddot p_{\mathrm{ref}}
  +K_d\left(\dot p_{\mathrm{ref}}-J\dot q\right)
  +K_p\left(p_{\mathrm{ref}}-p\right), \\
N &= I-J^\dagger J, \\
\ddot q_A &= J^\dagger\left(a_A-\dot J\dot q\right)-k_N N\dot q, \\
\tau_A &= M(q)\ddot q_A+c(q,\dot q), \\
\tau_{\mathrm{hybrid,retreat}} &=
\begin{cases}
\tau_{\mathrm{circle,mass}}, & \mathrm{TRACK}, \\
\tau_A, & \mathrm{RETREAT}.
\end{cases}
\end{aligned}
$$

Here $K_p=100I_3$ [1/s^2], $K_d=20I_3$ [1/s] and $k_N=4$ [1/s].
Only three-dimensional position is controlled; orientation remains free.
The mass-gradient torque is inactive in RETREAT, while Coriolis compensation
and nullspace velocity damping remain active. Gravity compensation remains
in the simulator, and the runner still clips the requested torque.

The controller requires an enabled collision monitor through its binding
hook. The runner, launch interface, momentum-observer equations and detector
remain unchanged. The reference duration does not end the run: after 2 s the
controller holds the goal and leaves `done` false. Launch duration is still
an upper bound, and the existing safety stops, exceptions and `Ctrl+C` can end
the run earlier. The command below disables stopping at first detachment and
sets a maximum RUN duration of 20 s. This variant adds no force feedback or
joint-limit avoidance.

The log adds `user_retreat_active` (0 in TRACK, then latched to 1),
`user_retreat_start_time` (NaN in TRACK, then $t_A$) and
`user_retreat_complete` (0 until the reference duration ends, then 1).
Completion concerns the reference, not a measured-arrival condition.
The start timestamp records the first retreat command-computation time,
which can follow the detector timestamp. During RETREAT and its endpoint
hold, the position, velocity and acceleration references, tracking errors,
nullspace diagnostics and `user_qdd_command` remain valid. Mass is retained
as a diagnostic with one evaluation per update, along with actual and
reference normal speed along $-n$; the
`user_tau_mass` vector is zero and `user_mass_gradient` is NaN because the
gradient is not evaluated in RETREAT. Wall interaction, monitoring and
logging continue during the motion and the hold.

### Run the horizontal-retreat controller

Build the package for the added files, then launch inside the container:

```bash
cd /ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select fr3_student_sim
source /ros2_ws/install/setup.bash

ros2 launch fr3_student_sim student.launch.py \
  controller:=circle_tracking/controller_hybrid_retreat.py \
  task_config:=/ros2_ws/src/fr3_student_sim/config/circle_tracking_hybrid_retreat.yaml \
  run_name:=circle_hybrid_retreat \
  log_dir:=/ros2_ws/src/fr3_student_sim/runs \
  wall:=true \
  observer:=true \
  stop_at_detachment:=false \
  control_rate:=1000 \
  real_time_factor:=0.5 \
  duration:=10
```

Save the figures for the latest retreat log:

```bash
source /ros2_ws/install/setup.bash
cd /ros2_ws/src/fr3_student_sim

latest_retreat_log=$(ls -t runs/circle_hybrid_retreat_*.npz | head -n 1)
ros2 run fr3_student_sim plot_run "$latest_retreat_log" --no-show
```

## 2026-11-08

### Compact figure for the horizontal-retreat experiment

`plot_run` selects an eight-panel figure for `controller_hybrid_retreat.py`
logs containing `user_retreat_active`, `user_retreat_complete` and the required signals:

1. Cartesian position and its reference.
2. The three position-error components in mm.
3. The seven joint torques in N m.
4. The seven joint velocities in rad/s.
5. Normal contact force and penetration in mm, with penetration on a second axis.
6. Directional mass in kg.
7. Actual and reference normal speed together, positive towards the wall.
8. Phase: 0 TRACK, 1 RETREAT, 2 HOLD.

HOLD is a display label derived from `user_retreat_complete` for maintaining
the final reference. It does not indicate measured arrival or introduce a
new control mode. The full NPZ signals, controller laws, references and gains
are preserved. The separate observer figure and generic figures for the
other controllers are unchanged. The existing `latest_retreat_log` selection
and `plot_run "$latest_retreat_log" --no-show` command still apply; no new
plot option is needed.

## 2026-11-09

### Collision signals in a separate figure

Each `plot_run X.npz` now also writes `X_collision.png`, alongside the
existing `X.png` and, when observer data are present, `X_observer.png`.
The new offline `collision_plot.py` module separates signal preparation in
`collision_signals` from figure creation in `collision_figure`. It uses one
saved experiment without requiring a baseline run. Control laws, references,
gains, runner, observer and the saved NPZ data are unchanged.

The figure now has five panels: two columns in each of the first two rows,
then generalized positive contact work across both columns of the third row.

| Panel                                 | Signal                                                                   |
| ------------------------------------- | ------------------------------------------------------------------------ |
| Directional mass [kg]                 | The existing `user_mass_kg` data.                                      |
| Normal speed [m/s]                    | Actual velocity along the inward wall normal, positive towards the wall. |
| Contact force [N]                     | The recorded normal spring-damper force $F_n$.                         |
| Positive penetration [mm]             | $1000\max(\delta,0)$.                                                  |
| Generalized positive contact work [J] | Cumulative outward contact work $W_{\mathrm{out}}^+$.                  |

The position-error-norm panel has been removed from this collision figure.
The main figure's error panel and the NPZ reference/error signals remain
available. The offline helper still returns `position_error_mm`, preserving
inactive-reference NaNs; it is no longer drawn in `_collision.png`.

The default display window spans 0.5 s before and after the first recorded
sample with $F_n>0$. The time axis is relative to that contact and a dashed
gold line marks zero. This origin comes from the recorded contact force,
independently of GMO threshold crossings.

The plotted generalized work uses external joint torques
$\tau_{\mathrm{ext}}$ acting on the robot, with outward power given by the
negative of their scalar product with joint velocity:

$$
\begin{aligned}
P_{\mathrm{out}}^+(t) &=
  \max\!\left(-\tau_{\mathrm{ext}}(t)^\top\dot q(t),0\right), \\
W_{\mathrm{out}}^+(t) &=
  \int_{t_0}^{t}P_{\mathrm{out}}^+(s)\,\mathrm{d}s, \\
W_{\mathrm{out},0}^+ &= 0, \\
W_{\mathrm{out},i}^+ &= W_{\mathrm{out},i-1}^+
  +\frac{P_{\mathrm{out},i-1}^++P_{\mathrm{out},i}^+}{2}
   \left(t_i-t_{i-1}\right).
\end{aligned}
$$

The helper prefers logged `tau_ext`; existing virtual-wall logs use the
fallback `tau_wall`. It does not substitute an observer residual or actuator
torque. Missing external torque or `dq` leaves the work panel marked
`Not logged`. The scalar product sums all joint-power terms before taking
the positive part. With several external contacts, this represents net
aggregate outward power, rather than separately clipping each contact's
contribution. The helper result keeps its existing
`positive_contact_work_J` field.

Here $t_0$ is the first saved timestamp. Integration uses the whole saved
sequence and its actual, potentially unequal timestamp intervals before the
display is cropped. The cumulative signal does not decrease. For the current
wall, with $v_n=-n^\top\dot p$, $F_{\mathrm{wall}}=F_n n$ and
$\tau_{\mathrm{wall}}=J^\top F_{\mathrm{wall}}$, its power reduces to
$F_n\max(v_n,0)$, including elastic loading and damping during compression.
It remains an estimate from sampled contact data, not an exact accounting of
energy exchanged by the simulated actuators. The other four panels retain
their existing signals; normal speed, mass direction, force, penetration
and the force-positive contact zoom remain tied to the wall setup.

Missing mass is annotated as unavailable, without substituting zeros or
recomputing mass with Pinocchio. The main figure and the separate
observer figure remain available, and the same `plot_run ... --no-show`
command generates the new figure without a new option. With observer data
there are three PNGs; without them there are two.

### Default collision-monitor binding hook restored

`StudentController.bind_collision_monitor(monitor)` has been restored as a
default no-op. Ordinary tracking controllers can inherit it when the runner
performs its existing startup binding, avoiding the missing-method error.
The hybrid controllers keep their own overrides and collision callbacks.
The runner's binding call and all control laws remain unchanged.

### Directional-mass diagnostic in the damping controller

`students/circle_tracking/controller_nullspace_damping.py` now logs
`mass_kg` along the wall normal supplied by the runner in the base frame.
It uses the fingertip position Jacobian and inertia matrix already evaluated
for damping. The control law is unchanged: no mass gradient, minimization
torque or gain is added. The runner saves `user_mass_kg` automatically, and
the existing main and `_collision.png` figures display it without configuration
or plotter changes. A new experiment records this additional signal; old NPZ
files retain their original fields.

### Circular baseline with nullspace posture feedback

The separate `students/circle_tracking/controller_baseline.py` extends the
existing nullspace-damping controller and uses
`config/circle_tracking_baseline.yaml`. It keeps the same initial pose,
position-only circle (radius 0.04 m, steady period 8 s, ramp 1 s), Cartesian
gains 100 [1/s^2] and 20 [1/s], and free orientation. A copy of `initial_q`
provides the posture reference $q_0$. The new variant adds only the projected
posture acceleration to the inherited tracking and damping law:

$$
\begin{aligned}
a &= \ddot p_{\mathrm{ref}}
  +K_d\left(\dot p_{\mathrm{ref}}-J\dot q\right)
  +K_p\left(p_{\mathrm{ref}}-p\right), \\
N &= I-J^\dagger J, \\
\ddot q_{\mathrm{posture}} &= k_{p,N}N(q_0-q), \\
\ddot q_{\mathrm{null}} &= -k_{d,N}N\dot q+\ddot q_{\mathrm{posture}}, \\
\ddot q_{\mathrm{cmd}} &= J^\dagger\left(a-\dot J\dot q\right)
  +\ddot q_{\mathrm{null}}, \\
\tau_{\mathrm{baseline}} &= M(q)\ddot q_{\mathrm{cmd}}+c(q,\dot q).
\end{aligned}
$$

`tracking.kp_nullspace` sets $k_{p,N}=25$ [1/s^2] and
`tracking.kd_nullspace` sets $k_{d,N}=10$ [1/s]. These are also this
controller's defaults when the corresponding settings are omitted. The
simulator continues gravity compensation; the controller adds no gravity
term or mass-minimization torque.

The existing log schema is retained: `user_dq_null` is the inherited
projected velocity, `user_qdd_null` includes both damping and posture
feedback, and `user_qdd_command` contains the total requested acceleration.
`user_mass_kg` remains the inherited directional-mass diagnostic. The GMO and
detector keep recording detections without switching this controller's law.
Launch duration and existing safety conditions still end the run; posture
feedback does not guarantee joint-limit avoidance.

The minimal controller, original damping controller and their configurations
remain unchanged. Compared with the current circular mass-minimization
variant, this baseline adds posture feedback and changes nullspace damping
from 4 to 10 [1/s], so the difference is not limited to the mass objective.
No other controller's settings are changed automatically.

### Run the posture baseline

Build for the added files, then launch inside the container:

```bash
cd /ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select fr3_student_sim
source /ros2_ws/install/setup.bash

ros2 launch fr3_student_sim student.launch.py \
  controller:=circle_tracking/controller_baseline.py \
  task_config:=/ros2_ws/src/fr3_student_sim/config/circle_tracking_baseline.yaml \
  run_name:=circle_baseline \
  log_dir:=/ros2_ws/src/fr3_student_sim/runs \
  wall:=true \
  observer:=true \
  stop_at_detachment:=false \
  control_rate:=1000 \
  real_time_factor:=0.50 \
  duration:=20
```

This command requests a maximum RUN duration of 20 s. Existing safety stops,
exceptions or `Ctrl+C` can end the run earlier.

Save the figures for the latest baseline log with the existing plot command:

```bash
source /ros2_ws/install/setup.bash
cd /ros2_ws/src/fr3_student_sim

latest_baseline_log=$(ls -t runs/circle_baseline_*.npz | head -n 1)
ros2 run fr3_student_sim plot_run "$latest_baseline_log" --no-show
```

### Active experiments and archive cleanup

The active package now keeps only the circular baseline and hybrid-retreat
entry points. Their control laws, gains, references, collision policies and
logged signals are preserved. The other experiment files are archived in
`trash/`, retaining the original `students/`, `config/`, `test/` and `runs/` paths.
The preceding sections remain a record of those developments rather than
the current active inventory.

The two active entry points and five configuration files are:

| File                                                      | Role                                                                                        |
| --------------------------------------------------------- | ------------------------------------------------------------------------------------------- |
| `students/circle_tracking/controller_baseline.py`       | Position tracking with nullspace posture feedback and damping; directional-mass diagnostic. |
| `students/circle_tracking/controller_hybrid_retreat.py` | Mass-minimizing circular TRACK followed directly by latched RETREAT on first detection.     |
| `config/circle_tracking_baseline.yaml`                  | Baseline initial pose, circle and tracking/nullspace gains.                                 |
| `config/circle_tracking_hybrid_retreat.yaml`            | Hybrid circle, tracking, mass objective and retreat settings.                               |
| `config/wall.yaml`                                      | Shared wall geometry and stiffness/damping.                                                 |
| `config/observer.yaml`                                  | Momentum-observer gain and residual thresholds.                                             |
| `config/controllers.yaml`                               | Joint list for the Gazebo effort controller.                                                |

Tracking implementations have been extracted into shared student utilities:

| Shared file                             | Preserved implementation                                                                                                                                      |
| --------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `students/utils/circle_tracking.py`   | `PositionTrackingController`, `DampedTrackingController` and `MassTrackingController`, carrying the existing Cartesian, damping and mass-gradient laws. |
| `students/utils/circle_reference.py`  | Existing circular reference.                                                                                                                                  |
| `students/utils/retreat_reference.py` | Existing quintic retreat reference and endpoint hold.                                                                                                         |
| `students/utils/mass_objective.py`    | Existing directional mass, central-difference gradient and torque projection.                                                                                 |
| `students/utils/__init__.py`          | Shared utilities package.                                                                                                                                     |

The baseline and hybrid entry points import these implementations instead of
depending on other selectable experiment controllers. This moves the shared
code without changing its mathematics or the values in the two active task
configurations. The hybrid's existing retreat tracker supplies its mass
diagnostic without a duplicate mass evaluation.

The archive contains the former minimal, damping, standalone circular-mass,
hybrid-RELEASE and straight experiments, the original circle/retreat reference
files, the skeleton and examples, and their former checks. Its seven archived
YAML files are `task.yaml`, `batch_example.yaml`, `circle_tracking.yaml`,
`circle_tracking_nullspace_damping.yaml`,
`circle_tracking_mass_minimization.yaml`, `circle_tracking_hybrid.yaml` and
`straight_mass_minimization.yaml`, under `trash/config/`. Saved NPZ and PNG
results of the former nullspace-damping experiment are in `trash/runs/`.
`trash/README.md`
indexes the retained material. Archived paths are outside the active students
and configuration directories.

`student.launch.py` now defaults to
`circle_tracking/controller_baseline.py`. Without an explicit `task_config`,
it chooses the matching YAML for the selected baseline or hybrid-retreat
entry point. `run_batch` remains available with baseline defaults and matching
task selection per run unless a task file is supplied explicitly; its former
example template is archived. Wall, observer,
plotting and safety behaviour are preserved.

At this cleanup, the package README provided build, launch and plot commands
for both active experiments. Those commands requested a maximum RUN duration of 7 s,
nominal 1000 Hz control and real-time factor 0.20, with wall and observer
enabled and first-detachment stopping disabled. Logs retain the distinct
`circle_baseline_` and `circle_hybrid_retreat_` prefixes. The launch defaults
remained a maximum duration of 20 s and real-time factor 0.5; the documented
7 s and 0.20 values were explicit per-run overrides.

### Optimized circular tracking with the complete baseline law

The new `students/circle_tracking/controller_optimized.py` inherits the
complete baseline controller and adds the existing projected mass-gradient
torque. Its separate `config/circle_tracking_optimized.yaml` keeps the baseline
initial pose, radius 0.04 m, cruise period 8 s, ramp 1 s, Cartesian gains
100 [1/s^2] and 20 [1/s], posture gain 25 [1/s^2] and nullspace damping
10 [1/s]. The fixed posture reference remains a copy of `initial_q`, and
orientation remains free. The added mass settings are gain
0.01 [N m rad/kg] and central-difference step $10^{-4}$ rad.

With $n$ the fixed wall unit normal in the base frame and $J$ the fingertip
position Jacobian, the additional command is:

$$
\begin{aligned}
m_n(q) &= \left(n^\top J M^{-1}J^\top n\right)^{-1}, \\
\bar J &= M^{-1}J^\top\left(JM^{-1}J^\top\right)^{-1}, \\
N_\tau &= I-J^\top\bar J^\top, \\
\tau_m &= -k_m N_\tau\nabla_q m_n(q), \\
\tau_{\mathrm{optimized}} &= \tau_{\mathrm{baseline}}+\tau_m, \\
\ddot q_{\mathrm{cmd,optimized}} &=
  \ddot q_{\mathrm{cmd,baseline}}+M^{-1}\tau_m.
\end{aligned}
$$

The existing `students/utils/mass_objective.py` supplies directional mass,
its central-difference gradient and the dynamic torque projection. The
baseline posture and damping terms are preserved. This adds no further
objective, gravity term or collision reaction. The observer and detector
continue recording detections while the optimized controller keeps tracking.
The runner, clipping, safety stops and the hybrid-retreat controller are
unchanged.

The inherited `user_qdd_null` still describes baseline posture feedback and
damping only. `user_qdd_command` includes the mass torque's joint-acceleration
contribution. The log also contains `user_mass_kg`, `user_mass_gradient`,
`user_tau_mass`, `user_normal_speed` and `user_normal_speed_ref`.
`user_tau_mass` is requested before total torque clipping; both normal-speed
signals are positive towards the wall. With a full-row-rank task Jacobian,
setting `mass_minimization.gain: 0.0` recovers the nominal baseline command.
The mass gradient is still computed in that case, so the variant retains
additional computation rather than matching baseline timing.

The active inventory now has three controller entry points: baseline,
optimized and hybrid retreat. There are six active YAML files: their three
matching task configurations, plus `wall.yaml`, `observer.yaml` and
`controllers.yaml`. The launch still defaults to baseline; automatic task
selection in launch and batch now also recognizes the optimized controller.
The earlier two-entry cleanup inventory remains the record of that stage.

### Run the optimized controller

Build for the added files, then launch inside the container. The command uses
the same 20 s maximum RUN duration, 1000 Hz nominal controller rate and
real-time factor 0.50 as the current README baseline command:

```bash
cd /ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select fr3_student_sim
source /ros2_ws/install/setup.bash

ros2 launch fr3_student_sim student.launch.py \
  controller:=circle_tracking/controller_optimized.py \
  task_config:=/ros2_ws/src/fr3_student_sim/config/circle_tracking_optimized.yaml \
  run_name:=circle_optimized \
  log_dir:=/ros2_ws/src/fr3_student_sim/runs \
  wall:=true \
  observer:=true \
  stop_at_detachment:=false \
  control_rate:=1000 \
  real_time_factor:=0.50 \
  duration:=20
```

Existing safety stops, exceptions or `Ctrl+C` can end the run earlier.
Baseline and optimized keep the same tracking, posture and damping settings
for the comparison; only the optimized variant adds the mass objective.

Save its figures through the existing plotting command:

```bash
source /ros2_ws/install/setup.bash
cd /ros2_ws/src/fr3_student_sim

latest_optimized_log=$(ls -t runs/circle_optimized_*.npz | head -n 1)
ros2 run fr3_student_sim plot_run "$latest_optimized_log" --no-show
```

## 2026-11-10

### Analytical directional-mass gradient

`students/utils/mass_objective.py` now evaluates the exact joint gradient of
directional mass through model derivatives. It replaces the previous
central-difference computation, including the fourteen perturbed
configurations, while retaining the same fixed-normal objective and
projected torque law. Both optimized tracking and hybrid TRACK use the shared
implementation at their existing controller-update frequency. The current
configured mass gain remains 0.1 [N m rad/kg]; no control gain, collision
policy or tracking law is changed by this replacement.

For the fixed unit normal $n$ in the robot base frame, let $J$ be the
fingertip position Jacobian in world-aligned axes. The implementation uses:

$$
\begin{aligned}
j &= J(q)^\top n, \\
x &= M(q)^{-1}j, \\
m_n(q) &= \left(j^\top x\right)^{-1}, \\
\dot J(q,x) &= \sum_i x_i\frac{\partial J(q)}{\partial q_i}, \\
\nabla_q m_n(q) &= -2m_n(q)^2
  \left[\dot J(q,x)^\top n-C(q,x)^\top x\right].
\end{aligned}
$$

$x$ is a virtual velocity used to contract the model derivatives; it is not
the measured joint velocity or an additional velocity command. The robot
model exposes `jacobian_pos_time_variation(q, x)` using Pinocchio's
`LOCAL_WORLD_ALIGNED` frame convention. The same virtual velocity enters
`coriolis_matrix(q, x)`. The helper can reuse the caller's $M$ and $J$ and
keeps snapshots of model results while Pinocchio reuses its working data.
The resulting gradient is neither normalized nor clipped.

Active mass configurations no longer contain `gradient_step`, and the helper
has no perturbation-step parameter. An extra `gradient_step` in an older task
mapping is ignored. Previous logs remain readable with their original
signals. Earlier finite-difference formulas, evaluation counts and step
values in this document remain historical descriptions, superseded here.
The current active circle configurations use a cruise period of 4 s; the
earlier 8 s descriptions likewise refer to the earlier settings. The launch
commands, duration and requested real-time factors in the README are preserved.

### Controller-call timing and plant-sampling reports

The new `fr3_student_sim/timing.py` prepares timing summaries independently
of controller and contact diagnostics. Every saved RUN row now includes a
`controller_updated` flag. `ctrl_ms` measures wall time around the controller
call; it can remain held on rows without a call. Selecting flagged rows
prevents those held values from being counted as additional updates.

The summary contains actual update count, mean, 95th percentile and maximum
controller duration, plus the number and percentage of calls exceeding the
nominal period:

$$
\begin{aligned}
T_s &= \frac{1}{f_{\mathrm{control}}}, \\
N_{\mathrm{over}} &= \#\left\{k\mid t_{\mathrm{controller},k}>T_s\right\}.
\end{aligned}
$$

This comparison assumes real time at RTF = 1. It does not measure complete
ROS callback latency or establish a deadline for the slower simulation.
Requested RTF is not stored in metadata. The recorded controller period is
derived from `control_rate`, rather than inferred from repeated `ctrl_ms`.

Separate plant-stream statistics use the saved simulation timestamps and a
nominal 1 kHz stream. They count expected and missing samples over the saved
interval, intervals longer than 1 ms and the maximum interval, with the
corresponding percentages. Missing-sample count and gap-interval count are
kept distinct because one interval can contain multiple missing samples.

The runner prints the summary at the end and saves it as `meta['timing']`.
`plot_run` includes it in the existing printed metrics, using the same command
and `--no-show` option. The main keys are `controller_updates`,
`controller_ms_mean`, `controller_ms_p95`, `controller_ms_max`,
`controller_period_ms`, `controller_update_over_period_count`,
`controller_update_over_period_pct`, `expected_samples`, `missed_samples`,
`missed_samples_pct`, `sample_intervals_over_1ms`,
`sample_intervals_over_1ms_pct` and `sample_interval_ms_max`.

For legacy logs without update flags, per-call statistics are inferred only
for safe 1 kHz cases; otherwise they remain unavailable. The summary records
`controller_timing_source` and `controller_timing_scope` to identify this
selection and the real-time comparison. These additions report measured
timing without claiming a speed improvement or guaranteed update rate.
