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
files and configurations. The circle reference is shared by the five circular
variants during tracking, while the straight experiment has its own reference
module. Edit the selected YAML to change its gains or trajectory settings.

## 2026-11-05

### Circular controller variants

The minimal controller in `students/circle_tracking/controller.py` is kept
as the reference. The five circular variants have separate controller files
and matching task configurations:

| Variant | Controller in `students/circle_tracking/` | Task configuration |
|---|---|---|
| Minimal position tracking | `controller.py` | `config/circle_tracking.yaml` |
| Position tracking with nullspace velocity damping | `controller_nullspace_damping.py` | `config/circle_tracking_nullspace_damping.yaml` |
| Damped position tracking with directional mass minimization | `controller_mass_minimization.py` | `config/circle_tracking_mass_minimization.yaml` |
| Mass-minimizing tracking with latched collision release | `controller_hybrid.py` | `config/circle_tracking_hybrid.yaml` |
| Mass-minimizing tracking with immediate horizontal retreat | `controller_hybrid_retreat.py` | `config/circle_tracking_hybrid_retreat.yaml` |

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

### Configuration files in use

| Configuration                                     | Role                                                                                                                                                                                                |
| ------------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `config/task.yaml`                              | Default task when `task_config` is omitted. Its top-level `Kp` and `Kd` fields are not read by the supplied controllers; the circular and straight variants read their `tracking` mappings. |
| `config/circle_tracking.yaml`                   | Initial pose, circular reference and Cartesian gains for the minimal circular controller.                                                                                                           |
| `config/circle_tracking_nullspace_damping.yaml` | Matching circular settings plus `tracking.kd_nullspace` for the damping variant.                                                                                                                  |
| `config/circle_tracking_mass_minimization.yaml` | Matching damped circular settings plus the fixed-wall-normal mass gain and gradient step for the circular mass-minimization variant. |
| `config/circle_tracking_hybrid.yaml` | Matching mass-minimizing circular TRACK settings plus seven joint-torque damping coefficients for the latched RELEASE phase. |
| `config/circle_tracking_hybrid_retreat.yaml` | Matching mass-minimizing circular TRACK settings plus horizontal retreat distance and reference duration for the latched RETREAT phase. |
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
  real_time_factor:=0.20 \
  duration:=20.0
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
  real_time_factor:=0.20 \
  duration:=20.0
```

Save the figures for the latest retreat log:

```bash
source /ros2_ws/install/setup.bash
cd /ros2_ws/src/fr3_student_sim

latest_retreat_log=$(ls -t runs/circle_hybrid_retreat_*.npz | head -n 1)
ros2 run fr3_student_sim plot_run "$latest_retreat_log" --no-show
```
