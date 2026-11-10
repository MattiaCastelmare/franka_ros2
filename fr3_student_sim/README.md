# fr3_student_sim

Three FR3 experiments in Gazebo, all controlling only the position of
`fr3_fingertip` with joint torques. Gripper orientation remains free.

| Experiment                                                             | Controller in `students/circle_tracking/` | Configuration                                  |
| ---------------------------------------------------------------------- | ------------------------------------------- | ---------------------------------------------- |
| Circular tracking with nullspace posture feedback                      | `controller_baseline.py`                  | `config/circle_tracking_baseline.yaml`       |
| Baseline tracking with directional mass minimization | `controller_optimized.py` | `config/circle_tracking_optimized.yaml` |
| Mass-minimizing circular tracking, then immediate retreat on detection | `controller_hybrid_retreat.py`            | `config/circle_tracking_hybrid_retreat.yaml` |

The other experiments, their configurations and the former checks are kept
in [trash/](trash/README.md). [Progress](docs/progress.md) preserves their
development history. They are outside the active controller/configuration
directories.

## Run the experiments

Everything runs inside the `franka_ros2` Docker container. From the host:

```bash
docker start franka_ros2
docker exec -it franka_ros2 bash
```

Build after adding, removing or moving package files:

```bash
cd /ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select fr3_student_sim
source /ros2_ws/install/setup.bash
```

Baseline:

```bash
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

Optimized:

```bash
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

Hybrid retreat:

```bash
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
  duration:=20
```

These commands request at most 20 simulated seconds in RUN, after settling.
Existing safety stops, exceptions or `Ctrl+C` can end a run earlier. A slower
real-time factor leaves more computation time without guaranteeing reception
of every joint-state sample. Logs are saved with the selected run-name prefix.

The launch defaults to `circle_tracking/controller_baseline.py`. When
`task_config` is omitted, it selects the matching configuration for baseline,
optimized or hybrid retreat. In a new shell, source `/ros2_ws/install/setup.bash`.

Save figures for each experiment's latest log:

```bash
source /ros2_ws/install/setup.bash
cd /ros2_ws/src/fr3_student_sim

latest_baseline_log=$(ls -t runs/circle_baseline_*.npz | head -n 1)
ros2 run fr3_student_sim plot_run "$latest_baseline_log" --no-show

latest_optimized_log=$(ls -t runs/circle_optimized_*.npz | head -n 1)
ros2 run fr3_student_sim plot_run "$latest_optimized_log" --no-show

latest_retreat_log=$(ls -t runs/circle_hybrid_retreat_*.npz | head -n 1)
ros2 run fr3_student_sim plot_run "$latest_retreat_log" --no-show
```

## Controllers and configuration

All three experiments retain the same initial pose, vertical x-z circle of radius
0.04 m, current cruise period 4 s and startup ramp 1 s. Cartesian gains are
100 [1/s^2] and 20 [1/s]. The fingertip starts near (0.56, 0, 0.50) m and
the circular reference reaches x approximately 0.64 m. The current wall is
at x = 0.62 m, with outward normal (-1, 0, 0), stiffness 20000 N/m and
damping 15 N s/m. Its roughly 20 mm maximum reference overlap is distinct
from actual penetration under contact force.

The baseline adds projected posture feedback towards a copy of `initial_q`
to Cartesian tracking and nullspace velocity damping. Its posture gain is
25 [1/s^2] and nullspace damping is 10 [1/s], including when those settings
are omitted. It logs directional mass as a diagnostic and keeps tracking
after detections. `user_qdd_null` includes posture feedback and damping;
`user_qdd_command` contains the total requested acceleration.

The optimized controller inherits the complete baseline law, with the same
posture gain 25 [1/s^2] and damping 10 [1/s], and adds only the projected
directional-mass gradient torque. It reuses `utils.mass_objective`, with
the current configured gain 0.1 [N m rad/kg], an analytical gradient and the
fixed wall normal as its direction. Baseline and optimized therefore share pose, reference,
Cartesian gains, posture feedback and damping. Both keep tracking after
GMO detections without switching mode. The hybrid-retreat policy remains separate.

For optimized logs, `user_qdd_null` still contains only baseline posture
feedback and damping; `user_qdd_command` also includes the mass torque's
acceleration contribution. Additional fields are `user_mass_gradient`,
`user_tau_mass` (requested before total torque clipping), `user_normal_speed`
and `user_normal_speed_ref`, positive towards the wall. With a full-row-rank
task Jacobian, setting `mass_minimization.gain` to zero recovers the nominal
baseline command; the gradient is still evaluated, so computation cost remains
different. No timing equivalence is assumed.

The hybrid controller uses nullspace damping 4 [1/s] and the current
mass-gradient torque gain 0.1 [N m rad/kg] in TRACK. The fixed mass
direction is the wall normal in the base frame. The first GMO detection
latches RETREAT directly. At the next controller update it captures the
current fingertip position and starts a 3 cm horizontal retreat along the
projected outward normal, currently -x. There is no waiting phase or condition
requiring measured velocity to reach zero. `retreat.distance` [m] and
`retreat.duration` [s], initially 0.03 and 2.0, configure a quintic reference
with zero reference velocity and acceleration at its endpoints. Actual robot
velocity is not reset at the switch.

RETREAT reuses the damped position tracker without mass torque. After the
reference finishes, it holds the endpoint until the run ends, without
resuming the circle or setting `done`. `user_retreat_active` stays 1 during
that hold; `user_retreat_complete` indicates reference completion, not measured
arrival. `user_retreat_start_time` is the first retreat-command update time.
Mass and normal-speed diagnostics continue, `user_tau_mass` is zero and
`user_mass_gradient` is NaN in RETREAT. References, errors and requested joint
acceleration remain valid.

The baseline and hybrid TRACK settings differ in both posture feedback and
nullspace damping, so their comparison does not isolate the mass objective.
These controllers do not guarantee joint-limit avoidance or lower contact force.

The six active YAML files are:

| File                                           | Role                                                                        |
| ---------------------------------------------- | --------------------------------------------------------------------------- |
| `config/circle_tracking_baseline.yaml`       | Baseline initial pose, circle, Cartesian and nullspace gains.               |
| `config/circle_tracking_optimized.yaml` | Matching baseline settings plus directional-mass gain. |
| `config/circle_tracking_hybrid_retreat.yaml` | Hybrid initial pose, circle, tracking, mass objective and retreat settings. |
| `config/wall.yaml`                           | Shared plane geometry and contact stiffness/damping.                        |
| `config/observer.yaml`                       | Observer gain and per-joint residual thresholds.                            |
| `config/controllers.yaml`                    | Joint list for the Gazebo effort controller.                                |

Shared tracking classes are in `students/utils/circle_tracking.py`:
`PositionTrackingController`, `DampedTrackingController` and
`MassTrackingController`. The reference modules are
`students/utils/circle_reference.py` and `students/utils/retreat_reference.py`;
the mass objective stays in `students/utils/mass_objective.py`. This
reorganization preserves the control laws and current gains. The loader makes
`students/` importable so controllers can import `utils`.

The shared mass objective now computes the analytical joint gradient from
Pinocchio's position-Jacobian time variation and Coriolis matrix, evaluated
with a virtual velocity. It reuses the caller's inertia matrix and Jacobian
without perturbing joint positions. Optimized and hybrid TRACK evaluate it
at every controller update. Active YAML files no longer need `gradient_step`;
that extra key in an older task file is ignored. Previous NPZ logs remain
readable. This change preserves the projected mass-torque law and current gains.

## Runtime and collision monitoring

Gazebo compensates gravity. Controller torques therefore implement
`M(q) qdd + C(q, dq) dq = tau + tau_ext` without adding gravity themselves.
The runner settles the arm, executes the selected controller, clips its torque
to the FR3 limits, applies virtual-wall interaction and saves the log.
The existing safety stop ends a run if a joint comes within 0.05 rad of its
limit or exceeds its speed limit. The virtual wall acts only at the fingertip;
the displayed wall box has no Gazebo collision geometry.

`FR3Model` in `fr3_student_sim/fr3_model.py` supplies position, Jacobians,
inertia, Coriolis and gravity terms, and directional mass for the simulated
robot. The controller receives this model, the selected YAML as `self.params`,
nominal `self.dt`, and measured `q`, `dq` in `update(t, q, dq)`.
`self.params['wall_geometry']` contains a copy of the static wall point and
normal; true contact force is not a controller input. `self.log` entries are
saved as `user_<name>`. Existing NPZ files retain their original fields;
newly added diagnostics require a new experiment to be logged.

`CollisionMonitor` owns the generalized momentum observer and detector. The
mathematics remains in `momentum_observer.py` and `collision_detector.py`:
joint momentum is `M(q) dq`, the residual is `K_O @ (p - p_hat)`, and the
gravity-compensated update uses
`p_hat_dot = tau_applied + C.T @ dq + residual`. Pinocchio's Coriolis matrix
satisfies `Mdot = C + C.T`. Initialization uses measured momentum at the first
RUN sample; implicit Euler integration uses elapsed simulation time.

The observer's torque input is reconstructed from clipped command history,
averaged over the measurement interval with the assumed 1 ms simulator delay.
It excludes wall torque and simulator-added gravity. It is not measured motor
telemetry and never uses the true external force. Monitoring updates on every
received RUN joint-state sample independently of the controller period.

The current observer gain is 30 [1/s] and threshold is 0.1 [N m]; each accepts
a scalar or seven values. Detection occurs when any `abs(residual[i])` exceeds
its threshold. The residual norm is diagnostic only. The runner calls
`bind_collision_monitor(monitor)` once at startup. Its base implementation is
a no-op; the hybrid override requires monitoring and registers `on_collision`
to retain the first onset, including one between controller updates. The
controller owns the latch and reaction; the monitor does not command torque.

## Logs and figures

For `X.npz`, the existing plot command saves `X.png`, `X_collision.png` and,
when observer data are present, `X_observer.png`. It prints ground-truth first
contact facts and detection times. Logs keep state, references, requested and
clipped torques, wall contact data, observer signals and user diagnostics:

```python
from fr3_student_sim.plot_run import load_run

run = load_run('runs/<log>.npz')
run['t'], run['q'], run['dq'], run['p'], run['v']
run['tau_cmd'], run['tau'], run['tau_applied'], run['tau_wall']
run['F_n'], run['delta'], run['v_n'], run['contact']
run['residual'], run['residual_norm'], run['detection_times']
run['user_mass_kg'], run['meta']
run['ctrl_ms'], run['controller_updated'], run['meta']['timing']
```

`tau_applied` is the delayed, clipped actuator-command reconstruction; it
excludes the wall. `tau_wall` is the true external wall torque, retained for
offline analysis. Neither supplies true force feedback to the controllers.

The baseline and optimized main figures use the generic user-signal layout. Hybrid-retreat
logs with the required signals use eight panels: position and reference;
three position-error components [mm]; seven joint torques [N m]; seven joint
velocities [rad/s]; force and penetration [mm] on a second axis; mass [kg];
actual/reference normal speed together; and phase 0 TRACK, 1 RETREAT, 2 HOLD.
Normal speed is positive towards the wall. HOLD labels the final-reference
hold, not a new control mode or measured arrival.

The collision figure has five panels: mass and actual normal speed in the
first row, force and positive penetration [mm] in the second, and generalized
positive contact work [J] across the last row. Its time origin is the first
recorded sample with `F_n > 0`, marked by a dashed gold line, with a default
window of 0.5 s on either side. With no contact it shows the full recorded
interval. This origin is independent of GMO detection.

Work integrates `max(-tau_ext @ dq, 0)` using trapezoids over the whole saved
sequence and actual timestamp intervals before cropping. Logged `tau_ext`
takes precedence, with `tau_wall` as fallback for existing wall logs. External
torque acts on the robot; summing joint power before taking the positive part
gives net aggregate outward power. Missing external torque or `dq` produces
`Not logged`; no residual or actuator-torque substitution is made. For the
current wall the power equals `F_n * max(v_n, 0)`, including elastic loading
and damping during compression. It is a sampled contact-work estimate.
The other contact signals and contact-centred zoom remain tied to the wall.

`collision_plot.py` prepares these signals offline without recomputing mass
with Pinocchio or changing the NPZ. Missing mass is labelled unavailable.
Position-error norm remains available from its helper without being drawn
in the collision figure; inactive-reference NaNs are preserved. The observer
figure still shows the seven residuals, thresholds, norm and detection timeline.

## Timing reports

New logs mark actual controller calls with `controller_updated`; `ctrl_ms`
contains the measured wall time around the last controller call. Summaries
select only marked calls, so a held value on another plant sample is not
counted twice. `timing.py` reports `controller_updates`,
`controller_ms_mean`, `controller_ms_p95`, `controller_ms_max`, and the count
and percentage of calls exceeding `controller_period_ms = 1000/control_rate`.
This is a nominal real-time comparison at RTF = 1. It does not measure the
whole callback latency or establish whether the running simulation meets a
deadline. The requested real-time factor is not saved in metadata.

Plant sampling is reported separately in simulated time: expected and missing
1 kHz samples, missing percentage, the count/percentage of intervals over
1 ms, and the maximum interval. One gap can account for several missing
samples. These counts cover the interval represented by the saved log.

The runner prints timing summaries at the end and saves them in
`meta['timing']`. The same `plot_run ... --no-show` command prints these
metrics through its existing report, without a new option or figure. For old
logs without update flags, controller statistics are inferred only for safe
1 kHz cases; otherwise they are unavailable. `controller_timing_source`
records how the calls were selected. Timing fields describe measurements
from each run; the analytical-gradient change alone makes no performance claim.

## Launch options

| Option                               | Default                                    | Meaning                                                                                        |
| ------------------------------------ | ------------------------------------------ | ---------------------------------------------------------------------------------------------- |
| `controller`                       | `circle_tracking/controller_baseline.py` | Active controller path relative to `students/`, or absolute; `none` starts only the robot. |
| `task_config`                      | Matching selected experiment               | Explicit task YAML override.                                                                   |
| `wall`, `observer`               | `true`                                   | Enable wall interaction and monitoring; hybrid retreat requires the observer.                  |
| `wall_config`, `observer_config` | Shared YAML files                          | Override wall or observer settings.                                                            |
| `duration`                         | `20.0`                                   | Maximum RUN duration [s]; the commands above request 20 s.                                      |
| `stop_at_detachment`               | `false`                                  | Optionally end at first detachment.                                                            |
| `control_rate`                     | `1000`                                   | Nominal controller frequency [Hz].                                                             |
| `real_time_factor`                 | `0.5`                                    | Requested simulation speed; the commands above use 0.50.                                       |
| `run_name`, `log_dir`            | Controller name, package `runs/`         | Log prefix and destination.                                                                    |
| `gui`                              | `true`                                   | Open the Gazebo window.                                                                        |
| `partition`                        | Empty                                      | Separate Gazebo transport partition for parallel runs.                                         |

Optional realism settings are `joint_friction`, `mass_error`, `noise_q`,
`noise_dq` and `seed`; their defaults retain the nominal model and noiseless
measurements. `run_batch` remains available for a supplied batch YAML, with
baseline defaults and matching task selection; the former template is archived.

For a missing Gazebo window, check the host display permissions or use
`gui:=false`. Startup and GUI output paths are printed in the terminal.
If joint-state samples are missed, reduce `real_time_factor` or
`control_rate`. An exception in a controller stops the run and saves its log.
