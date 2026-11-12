# Progress: FR3 simulation experiments

This document preserves the development history. The 2026-11-09 cleanup
retained baseline and hybrid retreat; the later optimized variant adds a
third active entry point, followed by a fourth with preventive joint release.
Earlier sections
describe the layout and defaults before that cleanup: archived `students/`,
`config/` and `test/` paths now live under `fr3_student_sim/trash/`, retaining
their relative structure. Shared tracking and reference implementations now
live in `students/utils/`. Use the current package README and the final
development entries below for the active files and commands.

The analytical-gradient implementation and timing reports added on
2026-11-10 supersede the finite-difference details in earlier entries.
The later first-impact entry also supersedes the historical positive-work
panel and contact-window descriptions.

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

### Separate offline posture reference with online mass minimization

`controller_optimized.py` now accepts an optional `q_posture_ref`, separate
from `initial_q`. The initial configuration still defines settling and the
starting point and center of the circular reference. The additional setting
replaces only the fixed reference used by the optimized controller's posture
feedback. If omitted, it falls back to `initial_q`, preserving the previous
controller behaviour. Baseline and hybrid retreat are unchanged.

With $N=I-J^\dagger J$, the optimized nullspace acceleration becomes:

$$
\begin{aligned}
q_{\mathrm{posture}} &={}
\begin{cases}
q_{\mathrm{posture\_ref}}, & \text{when supplied}, \\
q_{\mathrm{initial}}, & \text{otherwise},
\end{cases} \\
\ddot q_{\mathrm{null}} &=
  k_{p,N}N\left(q_{\mathrm{posture}}-q\right)-k_{d,N}N\dot q.
\end{aligned}
$$

The existing posture and damping gains and the online projected analytical
mass-gradient torque are retained. `user_qdd_null` reflects the selected
posture reference and damping; `user_qdd_command` still includes the online
mass contribution. The reference is saved with task parameters in metadata.
No new collision reaction is added.

With an offline reference selected only for optimized, comparison against
baseline includes both the changed posture reference and online mass
minimization. Zero mass gain removes only the online mass torque: it recovers
the nominal baseline command only when the posture references and remaining
settings also match. The gradient computation is still performed.

### Offline posture generation at the reference contact point

`students/utils/optimize_posture.py` adds a standalone offline CLI. It finds
the first entering intersection $p_c$ of the circular reference with the
configured wall plane, including the existing startup-ramp timing. The mass
direction remains the fixed outward wall normal. This reference crossing
is distinct from measured contact and observer detection.

The selected baseline log supplies $q_s$, the joint configuration at the
sample immediately before its first $F_n>0$, and the logged mass $m_b$ at
that sample. The tool accepts matching baseline-law logs, including optimized
with zero mass gain and the original posture reference. It checks that
initial pose, circle, gains, wall geometry and the logged mass agree with the
selected task and model.

Both criteria keep the Cartesian contact position and box-constrain the
joint positions with an explicit margin $\Delta q$:

$$
\begin{aligned}
p(q) &= p_c, \\
q_{\min}+\Delta q &\le q\le q_{\max}-\Delta q, \\
m_{\mathrm{target}} &= (1-\rho)m_b.
\end{aligned}
$$

`closest-target` minimizes $\tfrac12\lVert q-q_s\rVert^2$ subject also to
$m_n(q)\le m_{\mathrm{target}}$. `minimum-mass` minimizes $m_n(q)$ directly
with the position equality and box bounds. Orientation is free. The local
SLSQP solution is accepted only if its mass meets the requested target and
its position and bounds satisfy the implemented tolerances. This does not
assert a global optimum or constrain the intervening dynamic trajectory.

The CLI requires `--task-config`, `--wall-config`, `--baseline-log`,
`--joint-margin`, `--selection` and `--output`. The explicit margin must
exceed the runner's 0.05 rad stop margin. Optional `--reduction` defaults to
0.30, and `--report` overrides the default output suffix `.posture.json`.
The generated YAML deep-copies the input task and adds `q_posture_ref`,
leaving the initial pose, gains and online analytical-gradient control intact.

The JSON report records the selected criterion, baseline source and mass,
mass target, candidate mass, reference contact point, candidate joint vector,
physical joint margins and solver status. It identifies the result as a
nominal static candidate rather than an achieved simulation result. Its
local constant-geometry diagnostic is likewise distinct from a trajectory
or joint-limit guarantee. A requested reduction describes this static
candidate; it does not establish the mass at actual impact.

### Active closest-target posture

`config/circle_tracking_optimized.yaml` now stores the generated fixed
`q_posture_ref`. The selected criterion is `closest-target`, with a search
margin of 0.15 rad and requested static mass reduction of 30%. With $q_s$
from the matching baseline-law log, the selected problem is:

$$
\begin{aligned}
q^\star &= \arg\min_q\ \frac12\lVert q-q_s\rVert^2, \\
\text{subject to}\quad
p(q) &= p_c, \\
m_n(q) &\le 0.70m_b, \\
q_{\min}+0.15 &\le q\le q_{\max}-0.15.
\end{aligned}
$$

The source is `runs/circle_optimized_20261110_155101.npz`, recorded with zero
mass gain and the original `initial_q` posture reference. It supplies a
baseline-law pre-contact mass of 7.187731092 kg. At the reference crossing
$p_c\approx(0.62,0,0.534600862)$ m, the selected static candidate has mass
5.031411765 kg, meeting the requested 30% reduction, and minimum physical
joint margin 0.231925375 rad. The reference crossing time is approximately
1.833305319 s; this is a reference time, not measured contact or detection.

The candidate joint vector and full offline diagnostics are retained in
`docs/optimized_posture.json`. The report marks the result as a nominal
static candidate, not achieved simulation behaviour. Its local
constant-geometry mass/posture balance estimate is also nominal and does not
establish margins or convergence during the actual motion.

The active configuration keeps the original `initial_q`, circle and gains;
the online analytical mass-gradient torque remains active. Baseline and
hybrid retreat are unchanged. Comparison against the original baseline now
includes both offline posture selection and online mass minimization. A new
offline posture and matching reference log are needed if model, wall,
initial configuration or circle settings change; launch does not recompute
the posture. The online controller does not import SciPy or run the offline
solver.

The existing optimized launch and plot commands still apply. To reproduce
the offline output without overwriting the active task, use:

```bash
source /ros2_ws/install/setup.bash
cd /ros2_ws/src/fr3_student_sim

python3 students/utils/optimize_posture.py \
  --task-config config/circle_tracking_optimized.yaml \
  --wall-config config/wall.yaml \
  --baseline-log runs/circle_optimized_20261110_155101.npz \
  --selection closest-target \
  --joint-margin 0.15 \
  --reduction 0.30 \
  --output /tmp/circle_tracking_optimized_offline.yaml \
  --report /tmp/circle_tracking_optimized_offline.posture.json
```

### First compression-release cycle metrics and collision figure

`fr3_student_sim/impact_metrics.py` centralizes the first-impact interval and
metrics used by plotting and console reports. The interval starts at the
first recorded sample with $F_n>0$. It ends at the first detachment or the
beginning of the first recompression after release, whichever occurs first.
The recompression boundary is the negative-to-positive normal-velocity
zero crossing after release, with its time linearly interpolated between
recorded samples. Its origin is recorded contact force rather than observer
detection. Later contacts and recompressions are outside the selected cycle.

The first-impact force peak, maximum penetration and normal impulse use the
same interval. Writing $t_c$ for onset and $t_e$ for the selected end, the
shared quantities include:

$$
\begin{aligned}
I_n &= \int_{t_c}^{t_e}F_n(t)\,\mathrm{d}t, \\
F_{n,\max} &= \max_{t_c\le t\le t_e}F_n(t), \\
\delta_{\max} &= \max_{t_c\le t\le t_e}\max\left(\delta(t),0\right), \\
E_n^- &= \frac12m_n(t_-)v_n(t_-)^2,\qquad t_-<t_c.
\end{aligned}
$$

$t_-$ is the timestamp of the recorded sample immediately before onset.
Both entry mass and speed come from that sample; no sample after contact is
substituted. The impulse uses trapezoidal integration with actual timestamp
intervals, including the selected interpolated cycle boundary. The energy
is a normal kinetic-energy estimate from existing log signals. No robot
dynamics or directional-mass model is recomputed offline.

The shared report fields are `contact`, `t_contact_s`, `first_impact_end_s`,
`first_impact_duration_s`, `first_impact_complete`, `first_impact_end_reason`,
`precontact_sample_s`, `entry_mass_kg`, `entry_speed_mps`,
`entry_normal_kinetic_energy_J`, `peak_force_N`, `normal_impulse_Ns` and
`peak_penetration_mm`. Plotting and the batch report use the same first-cycle
metrics. If onset is the first log row, pre-contact values remain unavailable
and the recorded cycle is marked incomplete.
Missing normal velocity leaves the cycle unavailable. If the log ends before
the selected cycle is complete, its report is explicitly marked incomplete.
Missing or incomplete data are not replaced by fabricated values.

`X_collision.png` now has six panels in three rows and two columns:

| Panel | Contents |
|---|---|
| Directional mass [kg] | Logged mass along the wall normal. |
| Normal speed [m/s] | Actual speed, positive towards the wall. |
| Normal force [N] | First-cycle force with its peak marked. |
| Penetration [mm] | Positive penetration with its maximum marked. |
| Cumulative normal impulse [N s] | Integral of force over the first cycle only. |
| Pre-contact energy card | $E_n^-$, entry mass and entry normal speed. |

The first five panels display 50 ms before onset through the selected cycle
end, using time relative to onset. The previous positive-work panel is
removed from this figure. Historical work formulas and figure descriptions
remain in the preceding development entries. The existing `X.png` main
figure, `X_observer.png` when monitoring data are present, NPZ contents,
controller laws and observer are unchanged. The plot command and its
`--no-show` option retain the same three output filenames.

### Optimized tracking with preventive joint-limit release

The new `students/circle_tracking/controller_optimized_joint_release.py`
and `config/circle_tracking_optimized_joint_release.yaml` add a separate
preventive-release variant. TRACK reuses the optimized controller's offline
posture, circular reference, gains and online analytical mass-gradient torque.
The existing baseline, optimized and hybrid-retreat controllers and their
configurations are retained.

At each controller update, the variant predicts the remaining distance to
each joint limit using a linear velocity lookahead. For joint $i$:

$$
\begin{aligned}
\hat d_i^- &= q_i-q_{\min,i}-T\max(-\dot q_i,0), \\
\hat d_i^+ &= q_{\max,i}-q_i-T\max(\dot q_i,0), \\
\hat d_i &= \min\left(\hat d_i^-,\hat d_i^+\right), \\
\mathrm{release}_k &= \mathrm{release}_{k-1}
  \ \lor\ \left(\min_i\hat d_i\le d_R\right).
\end{aligned}
$$

`joint_release.lookahead` supplies $T=0.10$ s and `joint_release.margin`
supplies $d_R=0.10$ rad. The latch starts false. Its trigger depends only on
the current joint configuration and velocity, independently of observer
residuals and detection events. Monitoring can be disabled for this variant;
the launch command below keeps it enabled for recording.

Once triggered, the controller stays in RELEASE for the remaining RUN and
requests only joint damping:

$$
\begin{aligned}
D_R &= \operatorname{diag}(30,30,30,20,10,10,5)
       \quad [\mathrm{N\,m\,s/rad}], \\
\tau(t) &={}
\begin{cases}
\tau_{\mathrm{optimized}}(t), & \mathrm{TRACK}, \\
-D_R\dot q(t), & \mathrm{RELEASE}.
\end{cases}
\end{aligned}
$$

`joint_release.damping` configures the seven coefficients. RELEASE disables
Cartesian tracking, posture feedback, mass torque and Coriolis compensation.
Gravity remains compensated by the simulator. The runner still clips the
requested torque and retains its existing joint-position and velocity stops.
The controller neither resumes tracking nor sets `done` at release. Wall
interaction, logging and enabled monitoring continue while the runner stays
in RUN.

This lookahead uses current velocity over a fixed horizon. It is not a
computed braking distance, and the added release policy does not guarantee
that a joint limit will be avoided. No QP, additional tracking term or
force-based trigger is introduced.

The added log fields are `user_release_active` (0 or 1),
`user_release_start_time` (NaN before release, then its first command-update
time), `user_release_joint` (0 before release, then the one-based joint with
the smallest predicted margin), and `user_tau_release` (seven zeros in
TRACK, requested damping torque before clipping in RELEASE). The triggering
joint and start time remain latched. The seven-component
`user_joint_limit_margin` stores $\min(q-q_{\min},q_{\max}-q)$, and
`user_joint_limit_predicted_margin` stores the predicted $\hat d$ at each
update, including during RELEASE.

Directional mass and actual normal speed remain available in RELEASE.
`user_tau_mass` is zero. The inactive `user_p_ref`, `user_v_ref`,
`user_a_ref`, position/velocity errors, `user_qdd_command`, `user_qdd_null`,
`user_dq_null`, `user_mass_gradient` and `user_normal_speed_ref` are NaN.
The circle center and initial position remain available as constant
diagnostics. The guard records its transition through these log fields
without console printing in the update path.

The active inventory now has four controller entry points and seven YAML
files: four task configurations plus wall, observer and Gazebo-controller
configuration. Launch and batch task selection recognize the new variant,
while the default controller remains baseline. The runner is unchanged.

### Run the preventive-release variant

Build for the new files, then launch inside the container:

```bash
cd /ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select fr3_student_sim
source /ros2_ws/install/setup.bash

ros2 launch fr3_student_sim student.launch.py \
  controller:=circle_tracking/controller_optimized_joint_release.py \
  task_config:=/ros2_ws/src/fr3_student_sim/config/circle_tracking_optimized_joint_release.yaml \
  run_name:=circle_optimized_joint_release \
  log_dir:=/ros2_ws/src/fr3_student_sim/runs \
  wall:=true \
  observer:=true \
  stop_at_detachment:=false \
  control_rate:=1000 \
  real_time_factor:=0.20 \
  duration:=20
```

The command requests a maximum RUN duration of 20 s and nominal 1000 Hz
control. Its real-time factor 0.20 leaves more computation time without
guaranteeing receipt of every sample; existing safety stops, exceptions or
`Ctrl+C` can end the run early. Other documented launch commands are unchanged.

Save the figures for its latest log:

```bash
source /ros2_ws/install/setup.bash
cd /ros2_ws/src/fr3_student_sim

latest_joint_release_log=$(ls -t runs/circle_optimized_joint_release_*.npz | head -n 1)
ros2 run fr3_student_sim plot_run "$latest_joint_release_log" --no-show
```

The existing main, observer and first-cycle collision PNGs keep their names
and metric definitions.

### Cartesian impedance with constrained hierarchical redundancy

`students/circle_tracking/controller_impedance_hqp.py` and
`config/circle_tracking_impedance_hqp.yaml` add a separate position-impedance
experiment. Existing controller entry points and configurations are retained.
The circle wrapper supplies a reference object with `sample(t)` returning
world-frame position, velocity and acceleration. The reusable controller
does not depend on the circle shape or `wall_geometry`; orientation remains
free. It keeps requesting the reference during contact, allowing Cartesian
error and modifying the command when joint constraints require it. There is
no collision-triggered latch, fixed retreat or automatic phase reset in this
variant. Sliding or returning to the reference depends on feasibility and
contact dynamics; neither behavior is prescribed by a wall-specific policy.

The implementation is split into:

| File | Added responsibility |
|---|---|
| `students/utils/cartesian_impedance.py` | Natural-inertia position impedance, disturbance snapshot consumption, secondary objective and command logging. |
| `students/utils/joint_constraints.py` | Position, velocity and acceleration bounds, delayed-state prediction and disturbance-error tightening. |
| `students/utils/hierarchical_qp.py` | Two OSQP problems with reusable sparsity, warm starts and independent numerical acceptance checks. |
| `students/circle_tracking/controller_impedance_hqp.py` | Circular-reference adapter for the reusable controller. |
| `fr3_student_sim/collision_monitor.py` | Timestamped, read-only residual snapshot API; existing observer and detector remain separate. |
| `fr3_student_sim/impedance_plot.py` | Main figure for the new experiment. |
| `fr3_student_sim/compare_runs.py` | JSON comparison of two saved experiments using shared first-cycle metrics and timing summaries. |

Launch and batch task selection recognize the new configuration. OSQP and
SciPy are declared runtime dependencies. The runner retains its command
saturation, observer update, contact model and safety stops; controller
termination can now include a specific `failure_reason` in the saved stop
reason.

For the position Jacobian $J$, define the natural Cartesian inertia and its
dynamically consistent inverse:

$$
\begin{aligned}
W &= JM^{-1}J^T, &\Lambda &= W^{-1}, \\
\bar J &= M^{-1}J^T\Lambda, &N &= I-\bar J J, \\
f_I &= K(p_d-p)+D(\dot p_d-J\dot q), \\
b &= \ddot p_d-\dot J\dot q+Wf_I.
\end{aligned}
$$

Here $K$ is in N/m and $D$ in N s/m. They are physical Cartesian gains,
separate from the acceleration-feedback gains of the baseline. Without
active constraints, the primary impedance action is

$$
\tau_I=c+J^T\left[\Lambda(\ddot p_d-\dot J\dot q)+f_I\right].
$$

The simulator adds gravity compensation, so no $g$ is added to the returned
command. No estimated external torque is subtracted from this impedance
law. The QP variable is the actuator-induced acceleration
$u=M^{-1}(\tau-c)$; its Cartesian target is $Ju\approx b$. External torque
therefore continues to affect the physical motion rather than being
cancelled to enforce rigid tracking. A singular Cartesian inertia produces
an explicit failure instead of silently changing the impedance with an
inverse regularization. This sampled, constrained implementation does not
establish a passivity guarantee.

Both QP levels impose the same hard bounds without safety slack variables.
The primary level minimizes

$$
\frac12\|Ju-b\|^2+\frac{\rho}{2}\|u\|^2,
$$

where the small configurable $\rho>0$ regularizes the primary problem.
Writing $u_1$ for its accepted solution, the secondary level minimizes
$\frac12\|u-u_s\|^2$ subject to $Ju=Ju_1$. Thus secondary motion preserves
the attained primary Cartesian command within the configured numerical
equality tolerance. The preservation refers to this regularized primary
solution, rather than to an ideal unregularized least-squares result.

The secondary target combines projected posture attraction, nullspace
damping and the existing analytical directional-mass gradient:

$$
\begin{aligned}
\tau_m &=-k_mN^T\nabla m_n, \\
u_s &= \bar Jb+N\left[k_q(q^*-q)-k_v\dot q\right]+M^{-1}\tau_m.
\end{aligned}
$$

The posture contribution is omitted entirely when `q_posture_ref` is absent;
it does not silently use `initial_q` instead. The supplied YAML retains the
previous offline posture as an optional preference, not as a constraint or
an achieved mass-reduction guarantee. `mass_minimization.direction` defines
an explicit fixed direction in world axes at the controlled point. It does
not read the wall normal and does not identify an unknown contact point or
surface normal. The mass objective, damping and optional posture share the
remaining secondary freedom; position, velocity and torque constraints may
prevent them from attaining their preferred motion.

Joint-position barriers use configured protected limits
$q_\ell=q_{\min}+m_q$, $q_u=q_{\max}-m_q$ and two equal positive poles
$\lambda$. At the predicted state $(q_p,v_p)$, their acceleration bounds are

$$
-2\lambda v_p-\lambda^2(q_p-q_\ell)
\;\le\;\ddot q\;\le\;
-2\lambda v_p+\lambda^2(q_u-q_p).
$$

Velocity barriers, next-step position/velocity predictions and the
configurable `safety.acceleration_max` cap are intersected with these bounds.
That acceleration setting is an experiment limit, not a robot datasheet
value. Delay prediction uses the preceding applied actuator command and
the estimated disturbance. The prediction step accounts for an observed
controller-update gap, and uncertainty during command transport tightens
the predicted position and velocity as well.

With the observer residual $r$ and estimated disturbance acceleration
$d=M^{-1}r$, the physical acceleration estimate is $u+d$. For an assumed
componentwise acceleration-error envelope $\varepsilon$, each physical
interval $L\le\ddot q\le U$ becomes

$$
L-d+\varepsilon\le u\le U-d-\varepsilon.
$$

The actuator constraints cover both the gravity-compensated command
$\tau=Mu+c$ and the nominal motor torque $\tau+g(q)$. Using a configured
limit vector $T$, the QP imposes their intersection as

$$
\left|Mu+c+\frac12g(q)\right|
\le T-\frac12|g(q)|.
$$

Gravity is evaluated for this bound and the motor-torque diagnostic only;
it remains absent from the returned command. The bound uses the current
nominal $g(q)$ and does not certify future motor torque during transport or
unknown impacts. The original runner clipping remains an independent
fallback.

`CollisionMonitor.snapshot(...)` returns the latest residual with its
simulation timestamp, age, sample interval, filter time constants, validity
flag and reason. Snapshot arrays are read-only copies. Validity requires
finite data, initialization, continuous measured history over a configurable
warmup and a sufficiently fresh sample. A large measurement gap restarts
this validity warmup without resetting the observer or changing collision
detection. Reading a snapshot does not advance either component.

The controller requires `observer:=true`. During initialization/warmup
it uses zero estimated disturbance and a separate uncertainty setting.
Stale, discontinuous or invalid measurements terminate the experiment with
a recorded cause. For valid snapshots, a configurable floor and a residual-rate-based
reserve account heuristically for filtering, sample age and command delay.
The residual derivative only enlarges this reserve; it does not extrapolate
or deconvolve the external torque. No true contact force enters the
controller. The observer remains filtered, and these empirical envelopes
cannot guarantee joint-limit invariance for arbitrary unknown impacts,
model errors or missed deadlines. Viable initial states and feasible
actuator bounds remain necessary.

An unavailable or rejected secondary solution falls back to the accepted
primary solution from the same update. An infeasible or rejected primary
solution records a failure reason, requests bounded damping and terminates
the experiment. It does not reuse a stale QP command or relax hard limits.
This termination fallback is not itself a proof of limit avoidance.

The added `user_` log fields include Cartesian impedance force, mass and its
analytical gradient, requested mass torque, actual/reference directional
speed, observer validity/timestamp/age, residual used in predictions,
acceleration uncertainty, physical joint margins, acceleration bounds,
command-induced and predicted physical acceleration, predicted motor
torque, Cartesian command error, QP statuses, iterations, constraint
violation and solve times. Requested mass torque is a secondary preference;
it is not a separately identifiable applied contribution after constrained
optimization.

`plot_run` produces the same three filenames for the new experiment:
`X.png`, `X_observer.png` and `X_collision.png`. The main figure shows
position/reference, tracking error, commanded torque, joint velocity,
contact force, directional mass, normal speed, joint margins, QP/observer
status and computation time. The observer figure retains the seven residuals
and detection events. The collision figure retains the shared first
compression-release metrics, impulse and pre-contact energy card.

`compare_runs` reads two existing NPZ files and writes a JSON report. It
compares the first-cycle peak force, impulse, penetration and pre-contact
energy together with entry mass/speed, tracking, saved joint margins,
torques, QP statuses and timing. It reports missing data and incomplete
cycles explicitly. Saved-sample constraint summaries are not a continuous
motion guarantee, and a comparison of different controller laws does not
isolate the causal effect of mass minimization.

### Run and plot the impedance/HQP experiment

After installing the declared dependencies, build and launch inside the
container:

```bash
cd /ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select fr3_student_sim
source /ros2_ws/install/setup.bash

ros2 launch fr3_student_sim student.launch.py \
  controller:=circle_tracking/controller_impedance_hqp.py \
  task_config:=/ros2_ws/src/fr3_student_sim/config/circle_tracking_impedance_hqp.yaml \
  run_name:=circle_impedance_hqp \
  log_dir:=/ros2_ws/src/fr3_student_sim/runs \
  wall:=true \
  observer:=true \
  stop_at_detachment:=false \
  control_rate:=1000 \
  real_time_factor:=0.20 \
  duration:=20
```

Save the three PNGs and compare the latest matching logs:

```bash
source /ros2_ws/install/setup.bash
cd /ros2_ws/src/fr3_student_sim

latest_impedance_log=$(ls -t runs/circle_impedance_hqp_*.npz | head -n 1)
ros2 run fr3_student_sim plot_run "$latest_impedance_log" --no-show

latest_baseline_log=$(ls -t runs/circle_baseline_hqp_comparison_[0-9]*.npz | head -n 1)
ros2 run fr3_student_sim compare_runs \
  "$latest_baseline_log" "$latest_impedance_log" \
  --output runs/impedance_hqp_comparison.json
```

The comparison command accepts any explicitly selected pair of logs; use
matching initial states, references, wall and execution settings when
interpreting its report. The configured duration remains a maximum:
controller infeasibility, existing safety stops, exceptions or `Ctrl+C`
can end the run earlier.

### Configurable acceleration cap for the impedance/HQP experiment

The impedance/HQP configuration now selects `safety.acceleration_max: 30.0`
rad/s² for all seven joints, replacing the initial 8 rad/s² setting. The
constraint helper's fallback matches that value. The protected joint margin
remains 0.15 rad. No other controller configuration, impedance gain, observer
gain, uncertainty reserve, wall parameter or trajectory parameter is changed.
The acceleration setting remains an experiment cap, not a datasheet value.

### Active experiments reduced to baseline and impedance/HQP

The active controller inventory now contains only
`students/circle_tracking/controller_baseline.py` and
`students/circle_tracking/controller_impedance_hqp.py`. The optimized,
preventive joint-release and hybrid-retreat entry points and their three
task YAML files have been removed. The remaining five YAML files are the
baseline and impedance/HQP tasks plus the shared wall, observer and Gazebo
effort-controller configurations.

Launch and batch task selection now list the two active experiments. The
unused retreat reference and mass-tracking subclass have been removed;
the baseline position-tracking/damping classes, circular reference,
analytical mass objective and impedance/HQP utilities remain available.
The offline posture search and `docs/optimized_posture.json` are retained
because the HQP configuration still uses that low-mass posture.

Plotting retains the baseline generic figure, the impedance/HQP ten-panel
figure, and their shared observer and first-cycle collision figures.
Historical logs remain readable without the removed controller files.
Legacy baseline-law log recognition is retained for offline provenance.

The README now documents the two remaining controllers, their five YAML
files, launch/plot/comparison commands and the retained offline posture
workflow. Its HQP and matched-baseline examples use real-time factor 0.10;
the launch default remains 0.5. The remaining controllers' laws, gains,
joint constraints and observer parameters are unchanged. Historical
development sections above are preserved.

### Python code grouped by responsibility

The ROS package remains `fr3_student_sim`; its Python namespace is now
`fr3_sim`. The former internal `fr3_student_sim/` code directory and
`students/` tree are replaced by five folders, without further experiment
subdirectories:

| Folder under `fr3_sim/` | Responsibility |
|---|---|
| `controllers/` | Baseline and impedance/HQP entry points, common interface, shared tracking/impedance laws and controller loading. |
| `methods/` | Reference generation, analytical mass objective, joint bounds, hierarchical QP and offline posture search. |
| `monitoring/` | Momentum observer, collision detector and their monitor/snapshot interface. |
| `simulation/` | Runner, batch execution, Pinocchio model, robot description, virtual wall, applied torque and process/path helpers. |
| `analysis/` | Plot generation, impact metrics, timing summaries and log comparison. |

The experiment files are `controllers/baseline.py` and
`controllers/impedance_hqp.py`; the common interface is `controllers/base.py`.
Plot modules are `analysis/plot_run.py`, `analysis/plot_collision.py` and
`analysis/plot_impedance.py`. Configuration, launch files, documentation,
run artifacts, resource marker and ROS metadata retain their root locations.

Package discovery, console entry points and imports use the new Python
namespace. ROS commands remain under `fr3_student_sim`, with executable
names `runner`, `plot_run`, `run_batch` and `compare_runs`. Canonical launch
selectors are `baseline.py` and `impedance_hqp.py`; the former
`circle_tracking/controller_baseline.py` and
`circle_tracking/controller_impedance_hqp.py` selectors are retained as
aliases that resolve to the relocated files. Task YAML selection follows
the selected controller. Historical log metadata remains readable.

The reorganization preserves the controller laws, gains, observer,
constraints and log-signal definitions. Offline posture search is invoked
as `python3 -m fr3_sim.methods.optimize_posture`. Python log analysis uses
`from fr3_sim.analysis.plot_run import load_run, metrics`. The README now
shows the code layout and updated commands.

After building and sourcing the package, launch the active experiments with:

```bash
ros2 launch fr3_student_sim student.launch.py \
  controller:=baseline.py \
  run_name:=circle_baseline \
  duration:=20

ros2 launch fr3_student_sim student.launch.py \
  controller:=impedance_hqp.py \
  run_name:=circle_impedance_hqp \
  real_time_factor:=0.10 \
  duration:=20
```

The existing `ros2 run fr3_student_sim plot_run LOG.npz --no-show` and
`ros2 run fr3_student_sim compare_runs BASELINE.npz CANDIDATE.npz` commands
retain their names and arguments.

### Smooth startup of the impedance/HQP secondary objectives

The impedance controller now uses `impedance.startup_ramp_duration`,
configured as 0.5 s, to activate posture attraction and mass-gradient torque
smoothly from RUN time zero. A zero duration selects immediate activation.
The ramp is evaluated in simulation time and has zero slope at both ends:

$$
\begin{aligned}
s &= \operatorname{clip}(t/T,0,1), &\alpha(t)&=3s^2-2s^3, \\
u_s &= \bar Jb+N\left[\alpha(t)k_q(q^*-q)-k_v\dot q\right]
       +M^{-1}\left[\alpha(t)\tau_{m,\mathrm{nom}}\right].
\end{aligned}
$$

Only the secondary attraction preferences are scaled before optimization.
Cartesian impedance, trajectory timing, velocity damping, residual handling
and every hard QP constraint remain active. The output of the constrained
solver is not scaled. After the ramp, the same-state control law matches
the previous full-strength law; the startup joint trajectory can differ.
The baseline controller is unchanged.

`user_startup_scale` stores the ramp factor; `user_tau_mass` includes that
factor while `user_mass_gradient` remains the analytical unscaled gradient.
The existing HQP status panel displays the startup factor when available,
without adding another figure. Older logs remain readable. Observer and
detector operation, thresholds and detection-event logging are unchanged.

### Perpendicular baseline approach

Added `fr3_sim/controllers/baseline_perp.py` and
`config/baseline_perp.yaml`. The new controller reuses the baseline Cartesian
acceleration feedback, projected posture attraction towards `initial_q` and
joint damping. The existing circle baseline and impedance/HQP experiments
retain their behavior. Directional mass is recorded without minimization,
and collision monitoring does not switch the controller.

The reusable `fr3_sim/methods/perpendicular_reference.py` generates position,
velocity and acceleration along the negative wall normal from the initial
fingertip position. The default approach reaches 0.20 m/s with a smooth
0.20 s speed ramp. It crosses the reference wall plane at constant speed,
continues for 6 mm and brakes over 4 mm, ending with 10 mm of reference
overlap. It holds that final reference for 0.15 s, then sets the controller
done. Speed, ramp duration, overlap, braking distance and hold duration are
configurable in the new YAML.

For the current initial pose and wall, the nominal reference reaches the
plane around 0.40 s, brakes around 0.43-0.47 s and finishes around 0.62 s
after RUN starts. Actual contact time, entry speed and penetration depend
on tracking and contact dynamics. Reference overlap is not measured wall
penetration. The termination time does not depend on observer detection or
true contact force; the launch duration and existing safety stops can still
end a run earlier.

Controller loading, default task selection and batch selection include the
new experiment. The new signals `user_normal_speed`,
`user_normal_speed_ref`, `user_approach_distance_ref`, `user_reference_depth`
and `user_approach_complete` accompany the existing baseline and mass logs.
The existing main, observer and first-impact collision figures accept the
new logs. The collision summary's `entry_speed_mps` reports measured speed
immediately before contact; use it when matching future experiments.

After building and sourcing the updated package, launch inside the container:

```bash
cd /ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select fr3_student_sim
source /ros2_ws/install/setup.bash

ros2 launch fr3_student_sim student.launch.py \
  controller:=baseline_perp.py \
  task_config:=/ros2_ws/src/fr3_student_sim/config/baseline_perp.yaml \
  run_name:=baseline_perp \
  log_dir:=/ros2_ws/src/fr3_student_sim/runs \
  wall:=true observer:=true stop_at_detachment:=false \
  control_rate:=1000 real_time_factor:=0.10 duration:=2
```

Add `gui:=false` for a headless run. The two-second launch duration is only
an upper bound; the default reference and hold finish earlier. Generate the
three PNGs from the latest matching log with:

```bash
source /ros2_ws/install/setup.bash
cd /ros2_ws/src/fr3_student_sim
latest_perp_log=$(ls -t runs/baseline_perp_[0-9]*.npz | head -n 1)
ros2 run fr3_student_sim plot_run "$latest_perp_log" --no-show
```

The README now includes the three active experiments and six active YAML
files. The offline preferred-posture search still uses the circular
reference; no perpendicular posture-search workflow has been added.

### Impedance/HQP controller in one module

Merged the reusable `ImpedanceHQPController` class and circular experiment's
`Controller` subclass into `fr3_sim/controllers/impedance_hqp.py`. The
separate `controllers/cartesian_impedance.py` module has been removed.
Reference generation, mass objective, joint constraints and QP numerical
helpers remain in `fr3_sim/methods/`.

The control law, gains, configuration, observer handling and logging remain
unchanged. The circular adapter still supplies its reference to the generic
class. Existing `controller:=impedance_hqp.py` launch commands and the plot
commands retain their names and arguments. The README describes the merged
module; historical development sections are preserved.

### Optimized HQP controller name

Renamed the controller module to `fr3_sim/controllers/optimized_hqp.py` and
its task configuration to `config/circle_tracking_optimized_hqp.yaml`.
`ImpedanceHQPController`, the control law, gains, observer handling and
numerical helpers are unchanged. The former selectors `impedance_hqp.py`,
`controller_impedance_hqp.py` and
`circle_tracking/controller_impedance_hqp.py` remain aliases for the renamed
controller. Historical logs and comparison reports retain their filenames.

Use the new canonical selector and log prefix after rebuilding and sourcing
the package:

```bash
source /ros2_ws/install/setup.bash
ros2 launch fr3_student_sim student.launch.py \
  controller:=optimized_hqp.py \
  task_config:=/ros2_ws/src/fr3_student_sim/config/circle_tracking_optimized_hqp.yaml \
  run_name:=circle_optimized_hqp \
  log_dir:=/ros2_ws/src/fr3_student_sim/runs \
  wall:=true observer:=true stop_at_detachment:=false \
  control_rate:=1000 real_time_factor:=0.10 duration:=20
```

Add `gui:=false` for a headless run. Generate the figures for the latest new
log with:

```bash
source /ros2_ws/install/setup.bash
cd /ros2_ws/src/fr3_student_sim
latest_hqp_log=$(ls -t runs/circle_optimized_hqp_[0-9]*.npz | head -n 1)
ros2 run fr3_student_sim plot_run "$latest_hqp_log" --no-show
```

The README uses the new module, configuration and log prefix for current
commands. Earlier development sections and historical report links remain
unchanged.

### Perpendicular approach with optimized HQP

Added `fr3_sim/controllers/optimized_hqp_perp.py` and
`config/optimized_hqp_perp.yaml`. The new entry point passes the shared
`PerpendicularApproachReference` to `ImpedanceHQPController` from
`controllers/optimized_hqp.py`. The Cartesian impedance law, two QP levels,
joint constraints, observer handling, damping, mass objective and existing
gains are reused without changing the circular controller or the baseline.

The reference matches `baseline_perp`: 0.20 m/s approach speed, a 0.20 s
speed ramp, 10 mm final reference overlap, 4 mm braking distance and a
0.15 s final hold. The scheduled end is around 0.62 s after RUN starts for
the current geometry. The final hold ends independently of detection and
true contact force. Launch duration, QP infeasibility and existing safety
stops can still terminate the experiment earlier.

The perpendicular configuration has a separate `q_posture_ref`. Its offline
workflow uses the matching perpendicular baseline, rather than the
circle-derived posture. Online mass minimization uses the available
secondary freedom. The 0.5 s secondary-objective startup ramp is retained.
A static 30% posture target does not establish the same reduction at actual
impact during the short approach.

Controller loading, default task selection and batch selection include the
new experiment. Logging combines existing HQP diagnostics with the shared
perpendicular-reference signals. The main HQP, observer and first-impact
collision PNGs use the existing plotting components. Log comparison
recognizes perpendicular approach parameters and distinguishes scheduled
completion after the hold from premature controller termination.

After rebuilding and sourcing the package, launch inside the container:

```bash
source /ros2_ws/install/setup.bash
ros2 launch fr3_student_sim student.launch.py \
  controller:=optimized_hqp_perp.py \
  task_config:=/ros2_ws/src/fr3_student_sim/config/optimized_hqp_perp.yaml \
  run_name:=optimized_hqp_perp \
  log_dir:=/ros2_ws/src/fr3_student_sim/runs \
  wall:=true observer:=true stop_at_detachment:=false \
  control_rate:=1000 real_time_factor:=0.10 duration:=2
```

Add `gui:=false` for a headless run. The launch duration is an upper bound;
the reference and final hold normally finish earlier. Generate the three
figures with:

```bash
source /ros2_ws/install/setup.bash
cd /ros2_ws/src/fr3_student_sim
latest_hqp_perp_log=$(ls -t runs/optimized_hqp_perp_[0-9]*.npz | head -n 1)
ros2 run fr3_student_sim plot_run "$latest_hqp_perp_log" --no-show
```

For comparison, run the perpendicular baseline with the same execution
settings, then use:

```bash
baseline_perp_log=$(ls -t runs/baseline_perp_[0-9]*.npz | head -n 1)
ros2 run fr3_student_sim compare_runs \
  "$baseline_perp_log" "$latest_hqp_perp_log" \
  --output runs/optimized_hqp_perp_comparison.json
```

The README now lists four experiments and seven active YAML files, with the
new launch, plot and comparison commands. Historical sections and reports
are preserved.

### Offline posture workflow for the perpendicular approach

Extended `fr3_sim/methods/optimize_posture.py` to recognize the shared
perpendicular reference as well as the circle. The optimizer uses the
selected reference's first entering wall-plane intersection and a matching
baseline log for its contact configuration and directional mass. The
`closest-target` selection, explicit 0.15 rad joint margin and configurable
mass-reduction constraint are retained. The circular HQP posture,
configuration and historical report remain separate.

Generate a perpendicular posture candidate and its static report with:

```bash
source /ros2_ws/install/setup.bash
cd /ros2_ws/src/fr3_student_sim
baseline_perp_log=$(ls -t runs/baseline_perp_[0-9]*.npz | head -n 1)

python3 -m fr3_sim.methods.optimize_posture \
  --task-config config/baseline_perp.yaml \
  --wall-config config/wall.yaml \
  --baseline-log "$baseline_perp_log" \
  --selection closest-target \
  --joint-margin 0.15 \
  --reduction 0.30 \
  --output /tmp/low_mass_perp_posture.yaml \
  --report docs/optimized_hqp_perp_posture.json
```

The output copies the perpendicular baseline task and adds `q_posture_ref`;
it does not automatically replace the HQP configuration. After a feasible
candidate is found, copy only that vector into
`config/optimized_hqp_perp.yaml`, retaining the impedance, QP and approach
settings. The report stores static constraints and baseline provenance.
Actual entry mass, speed and joint margins remain simulation measurements;
the offline target alone is not a guarantee of a 30% impact-mass reduction.

Recalculated the perpendicular preferred posture from
`baseline_perp_20261111_155628.npz` with `closest-target`, a 0.15 rad margin
and the 30% static reduction constraint. Applied the resulting
`q_posture_ref` to `config/optimized_hqp_perp.yaml` and saved its provenance
and static constraints in
[docs/optimized_hqp_perp_posture.json](optimized_hqp_perp_posture.json).
The circle configuration and `docs/optimized_posture.json` are unchanged.
This updates the posture preference without asserting the same mass
reduction during the simulated approach.

Completion checking uses `user_trial_complete` for finite approaches.
Earlier perpendicular logs without that signal retain readable first-impact
metrics but do not establish scheduled completion; use a new baseline run
for full completion validation.

### Longer acceleration ramp for the perpendicular experiments

Changed `approach.acceleration_duration` from 0.20 s to 0.50 s in both
`config/baseline_perp.yaml` and `config/optimized_hqp_perp.yaml`. The initial
pose, wall, 0.20 m/s reference speed, 10 mm final reference overlap, 4 mm
braking distance and 0.15 s hold are retained. Controller laws, gains,
observer settings and joint constraints are unchanged.

With the current geometry, the nominal reference reaches the wall at about
0.55 s, brakes from about 0.58 to 0.62 s and finishes around 0.77 s. The
existing 0.5 s HQP secondary-objective startup ramp now completes before
the nominal reference contact time. Actual contact time and entry speed
remain measured quantities.

The perpendicular offline posture workflow uses a fresh `baseline_perp`
log matching this longer ramp. Its report remains
`docs/optimized_hqp_perp_posture.json`; the resulting preference belongs in
`config/optimized_hqp_perp.yaml`. The circular configuration and posture
report remain separate. A static mass target still requires checking
against the measured mass and speed at actual impact.

Existing launch selectors, configurations and plotting commands retain
their names. `duration:=2` remains a maximum RUN duration; the finite
approach and hold finish earlier under the new timing. Earlier development
sections preserve the previous 0.20 s ramp and its original provenance.

Refreshed the perpendicular offline posture from
`baseline_perp_20261111_175507.npz`, matching the 0.50 s acceleration ramp.
Applied the new `q_posture_ref` to `config/optimized_hqp_perp.yaml` and
updated `docs/optimized_hqp_perp_posture.json` with the baseline provenance
and static search constraints. The search retains `closest-target`, the
0.15 rad joint margin and the 30% static mass-reduction target. The circular
HQP configuration and its posture report are unchanged.

### Hard physical constraints and soft preventive preferences

Added an optional three-level HQP mode, enabled in
`config/optimized_hqp_perp.yaml` through `safety.soft_constraints`. Predicted
position and velocity and the existing command/nominal motor torque bounds
remain hard. The hard position margin is 0.06 rad, preserving a 0.01 rad
buffer before the runner's 0.05 rad stop band. The 0.15 rad preventive
margin, 30 rad/s^2 acceleration preference and preventive rate barriers are
represented separately as relaxable rows. The observer residual, sample
age, command delay and uncertainty reserve continue to affect predictions;
true external force is not used in the controller.

The first QP minimizes normalized nonnegative lower/upper slack variables
while satisfying the hard constraints. The second preserves the optimal
slack and tracks Cartesian impedance. The third preserves the attained
Cartesian action while using remaining freedom for posture, analytical
mass minimization and nullspace damping:

$$
\ell_s-s^-\le A_su\le h_s+s^+,\qquad s^-,s^+\ge0.
$$

The row scales are configurable through `acceleration_scale`,
`position_scale` and `velocity_scale`. A failure of the hard safety level
still terminates the trial. If the tracking level fails after a verified
safety solution, the command comes from that same update's safety solution.
The runner and observer validity/exception guards are retained. The circle
configuration continues to use the existing strict two-level solver when
the optional mode is absent or disabled.

Logging now includes the lower/upper slack for all three seven-joint groups,
scalar maximum and normalized maximum slack, and safety-QP status,
iterations and timing. The existing ten-panel HQP plot distinguishes the
preferred margin, hard margin and runner stop band, and displays normalized
slack and safety status without adding a figure.

Comparison reports retain measured acceleration/preventive-margin
exceedances as diagnostics in soft mode, while checking hard margin, runner
stop band, velocity, command/motor torque and finite hard-QP diagnostics.
Hard and soft outcomes are reported separately; the circle's existing
acceptance criteria remain unchanged. Safety-QP timing is also summarized.

The fixed `q_posture_ref` is still computed offline by SciPy SLSQP. The
online HQP does not recompute that posture: it considers its attraction term
together with the analytical mass gradient at the current configuration.
Existing launch, plot and comparison commands remain unchanged.

### Perpendicular HQP without an offline posture preference

Historical development entry: this variant was subsequently removed, as
recorded below. Its launch commands are retained as history and no longer
select an installed controller.

Added `fr3_sim/controllers/optimized_perp_full_online.py` and
`config/optimized_perp_full_online.yaml`. The new adapter reuses the
perpendicular impedance/HQP controller while removing `q_posture_ref` from
its parameters. Its YAML omits the offline posture vector and the unused
posture gain. `initial_q` remains the settling configuration and reference
start, without becoming a secondary posture target.

The perpendicular reference, 0.50 s acceleration ramp, 0.20 m/s entry-speed
reference, 10 mm endpoint overlap, braking and final hold are retained.
Hard/soft constraints, observer handling, impedance gains, online analytical
mass gradient, mass gain, nullspace damping and 0.5 s mass-torque startup
ramp are inherited from `optimized_hqp_perp`. The final HQP level uses
online mass minimization and damping without attraction to an offline
posture. No offline posture search is required for this experiment.

Controller selection and task loading include the new entry point. The
existing HQP plotting layout and comparison recognize its logs, including
scheduled finite-trial completion and hard/soft diagnostics. Existing
controller behavior and offline posture reports are retained.

After rebuilding and sourcing the package, launch inside the container:

```bash
source /ros2_ws/install/setup.bash
ros2 launch fr3_student_sim student.launch.py \
  controller:=optimized_perp_full_online.py \
  task_config:=/ros2_ws/src/fr3_student_sim/config/optimized_perp_full_online.yaml \
  run_name:=optimized_perp_full_online \
  log_dir:=/ros2_ws/src/fr3_student_sim/runs \
  wall:=true observer:=true stop_at_detachment:=false \
  control_rate:=1000 real_time_factor:=0.10 duration:=2
```

Add `gui:=false` for a headless run. Generate the three figures with:

```bash
source /ros2_ws/install/setup.bash
cd /ros2_ws/src/fr3_student_sim
latest_online_perp_log=$(ls -t runs/optimized_perp_full_online_[0-9]*.npz | head -n 1)
ros2 run fr3_student_sim plot_run "$latest_online_perp_log" --no-show
```

### Removal of the full-online perpendicular variant

Removed `fr3_sim/controllers/optimized_perp_full_online.py`, its
`config/optimized_perp_full_online.yaml` task and the corresponding active
controller selectors and launch references. The active inventory now has
four controller variants for two experiments: circular `baseline.py` and
`optimized_hqp.py`, and perpendicular `baseline_perp.py` and
`optimized_hqp_perp.py`. Their control laws and settings are retained.

The README now lists the four active controllers and seven YAML files,
with current launch, plotting and comparison commands. Soft constraints
remain enabled in `optimized_hqp_perp.yaml`. Existing NPZ logs and PNGs of
the removed variant are retained; plotting and comparison keep historical
log recognition without loading the removed controller. Offline posture
reports remain separate and unchanged. Earlier development entries above
are preserved as history.

### Shared Cartesian acceleration tracking in baseline and optimized controllers

Changed the primary tracking request in `controllers/optimized_hqp.py`
and its perpendicular adapter to the same Cartesian acceleration-feedback
law used by the respective baseline:

$$
\begin{aligned}
a_{\mathrm{track}} &= \ddot p_d+K_p(p_d-p)+K_d(\dot p_d-J\dot q),\\
b &= a_{\mathrm{track}}-\dot J\dot q,\\
\tau &= M(q)u_{\mathrm{HQP}}+c(q,\dot q).
\end{aligned}
$$

Both optimized YAML files now configure `tracking.kp_position: 100.0`
and `tracking.kd_position: 20.0`, with units s^-2 and s^-1. The previous
Cartesian stiffness/damping feedback is removed. Secondary damping,
posture gain and startup duration move into the same `tracking` block
without changing their values. The reusable class is named
`HQPTrackingController`; the controller filenames and launch selectors
remain `optimized_hqp.py` and `optimized_hqp_perp.py`.

The HQP hierarchy, hard/soft constraints, dynamically consistent
nullspace projection, fixed `q_posture_ref`, analytical online mass
gradient, mass gain, observer usage, torque convention and termination
policies are retained. The circle keeps strict two-level HQP, while the
perpendicular experiment keeps its three-level soft-constraint mode.
Baseline controllers and reference trajectories are unchanged.

Logging records `user_tracking_acceleration` instead of the former virtual
impedance force. The existing plot component supports both current and
historical logs. The README now describes the shared tracking request and
new YAML keys. Historical impedance-era results and reports are retained
as provenance; new simulations are needed to establish mass and collision
metrics for this tracking law. The offline posture files are not regenerated
by this change.

After building and sourcing the package, the existing commands still apply:

```bash
source /ros2_ws/install/setup.bash
ros2 launch fr3_student_sim student.launch.py \
  controller:=optimized_hqp.py \
  run_name:=circle_optimized_hqp \
  real_time_factor:=0.10 duration:=20

ros2 launch fr3_student_sim student.launch.py \
  controller:=optimized_hqp_perp.py \
  run_name:=optimized_hqp_perp \
  real_time_factor:=0.10 duration:=2
```

The existing `ros2 run fr3_student_sim plot_run LOG.npz --no-show` and
`ros2 run fr3_student_sim compare_runs BASELINE.npz CANDIDATE.npz` commands
retain their names and arguments.
