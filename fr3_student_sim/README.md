# fr3_student_sim

Four controller variants for two FR3 experiments in Gazebo with joint-torque
control: circular tracking and perpendicular collision. All control the
position of `fr3_fingertip`; gripper orientation remains free.

| Experiment                                                         | Controller in `fr3_sim/controllers/` | Configuration                                 |
| ------------------------------------------------------------------ | -------------------------------------- | --------------------------------------------- |
| Circular tracking with nullspace posture feedback                  | `baseline.py`                        | `config/circle_tracking_baseline.yaml`      |
| Circular acceleration tracking with HQP and mass minimization      | `optimized_hqp.py`                   | `config/circle_tracking_optimized_hqp.yaml` |
| Perpendicular approach with the baseline control law               | `baseline_perp.py`                   | `config/baseline_perp.yaml`                 |
| Perpendicular acceleration tracking with HQP and mass minimization | `optimized_hqp_perp.py`              | `config/optimized_hqp_perp.yaml`            |

[Progress](docs/progress.md) retains the development history.

## Project layout

The ROS package is still named `fr3_student_sim`. Its Python code is grouped
under `fr3_sim`, with one level of folders for the main responsibilities:

```text
fr3_student_sim/
├── fr3_sim/
│   ├── controllers/   # experiment controllers and shared control laws
│   ├── methods/       # mass objective, QP, constraints and reference generation
│   ├── monitoring/    # momentum observer, collision detector and monitor
│   ├── simulation/    # runner, robot model, wall and simulator helpers
│   └── analysis/      # plots, metrics, timing and log comparison
├── config/
├── launch/
├── docs/
├── runs/
├── resource/
├── package.xml
├── setup.py
└── setup.cfg
```

`controllers/base.py` defines the common interface. `controllers/loading.py`
resolves controller names and loads their classes. The shared baseline
controller hierarchy is in `controllers/circle_tracking.py`; both baseline
and optimized use the primary feedback in `methods/cartesian_tracking.py`.
`controllers/optimized_hqp.py` contains both the reusable
`HQPTrackingController` law and the circular experiment's `Controller`
subclass. `controllers/optimized_hqp_perp.py` supplies the perpendicular
reference to that same reusable class.
Numerical helpers remain in `methods/`.
`simulation/fr3_model.py` wraps Pinocchio, while
`simulation/runner.py` connects controllers to Gazebo.

Plots are in `analysis/plot_run.py`, `analysis/plot_collision.py` and
`analysis/plot_impedance.py`. First-impact metrics, timing and comparisons
are kept beside them.

ROS executable names remain `runner`, `plot_run`, `run_batch` and
`compare_runs`, under the ROS package `fr3_student_sim`. Python imports use
the `fr3_sim` namespace.

## Run the experiments

Everything runs inside the `franka_ros2` Docker container. From the host:

```bash
docker start franka_ros2
docker exec -it franka_ros2 bash
```

Build after adding or removing package files, then source the installation:

```bash
cd /ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select fr3_student_sim
source /ros2_ws/install/setup.bash
```

Baseline:

```bash
ros2 launch fr3_student_sim student.launch.py \
  controller:=baseline.py \
  task_config:=/ros2_ws/src/fr3_student_sim/config/circle_tracking_baseline.yaml \
  run_name:=circle_baseline \
  log_dir:=/ros2_ws/src/fr3_student_sim/runs \
  wall:=true observer:=true stop_at_detachment:=false \
  control_rate:=1000 real_time_factor:=0.20 duration:=20
```

Optimized Cartesian tracking with HQP:

```bash
ros2 launch fr3_student_sim student.launch.py \
  controller:=optimized_hqp.py \
  task_config:=/ros2_ws/src/fr3_student_sim/config/circle_tracking_optimized_hqp.yaml \
  run_name:=circle_optimized_hqp \
  log_dir:=/ros2_ws/src/fr3_student_sim/runs \
  wall:=true observer:=true stop_at_detachment:=false \
  control_rate:=1000 real_time_factor:=0.20 duration:=20
```

Use real-time factor 0.20 for all four experiments to give HQP more wall-clock time
per simulated update and keep the requested timing settings equal within
each pair. Execution load still matters: a lower factor does not guarantee
receipt of every sample or QP feasibility. Add `gui:=false` to both members
of a pair for headless runs. Keep the rate and trajectory settings unchanged
when comparing real-time factors.

The circle commands request up to 20 simulated seconds in RUN, after settling.
Existing safety stops, controller termination, exceptions or `Ctrl+C` can end
a run earlier. Logs use the selected run-name prefix. In each new shell,
source `/ros2_ws/install/setup.bash`.

Perpendicular baseline approach:

```bash
ros2 launch fr3_student_sim student.launch.py \
  controller:=baseline_perp.py \
  task_config:=/ros2_ws/src/fr3_student_sim/config/baseline_perp.yaml \
  run_name:=baseline_perp \
  log_dir:=/ros2_ws/src/fr3_student_sim/runs \
  wall:=true observer:=true stop_at_detachment:=false \
  control_rate:=1000 real_time_factor:=0.20 duration:=2
```

Add `gui:=false` to run without the Gazebo window. This experiment finishes
after its approach, braking and short hold, nominally about 0.77 simulated
seconds after RUN starts with the default geometry. `duration:=2` is an
upper bound, not an additional final hold.

Perpendicular optimized HQP approach:

```bash
ros2 launch fr3_student_sim student.launch.py \
  controller:=optimized_hqp_perp.py \
  task_config:=/ros2_ws/src/fr3_student_sim/config/optimized_hqp_perp.yaml \
  run_name:=optimized_hqp_perp \
  log_dir:=/ros2_ws/src/fr3_student_sim/runs \
  wall:=true observer:=true stop_at_detachment:=false \
  control_rate:=1000 real_time_factor:=0.20 duration:=2
```

The approach and scheduled completion are the same as the perpendicular
baseline. Add `gui:=false` for a headless run. All HQP experiments require
the observer; QP infeasibility or existing safety stops can end them early.

Launch defaults to baseline. Omitting `task_config` selects the matching
controller YAML; the launch defaults themselves remain
1000 Hz, real-time factor 0.5 and duration 20 s.

The built-in controller selectors are `baseline.py`, `baseline_perp.py`,
`optimized_hqp.py` and `optimized_hqp_perp.py`. An absolute custom
controller-file path is also supported.

Save the three figures for the experiment you choose. In each new shell:

```bash
source /ros2_ws/install/setup.bash
cd /ros2_ws/src/fr3_student_sim
```

Circular baseline:

```bash
ros2 run fr3_student_sim plot_run "$(ls -t runs/circle_baseline_*.npz 2>/dev/null | head -n 1)" --no-show
```

Circular optimized HQP:

```bash
ros2 run fr3_student_sim plot_run "$(ls -t runs/circle_optimized_hqp_*.npz 2>/dev/null | head -n 1)" --no-show
```

Perpendicular baseline:

```bash
ros2 run fr3_student_sim plot_run "$(ls -t runs/baseline_perp_*.npz 2>/dev/null | head -n 1)" --no-show
```

Perpendicular optimized HQP:

```bash
ros2 run fr3_student_sim plot_run "$(ls -t runs/optimized_hqp_perp_*.npz 2>/dev/null | head -n 1)" --no-show
```

These plotting patterns include run-name suffixes such as `_tracking_check`.
The pattern `circle_optimized_hqp_[0-9]*.npz` only matches a timestamp directly
after `circle_optimized_hqp_`, so it excludes those preliminary logs. Check
`duration_s` and `stop_reason` before using a log for a comparison: the short
circular tracking check does not replace a full 20 s run. The comparison
commands below also select by experiment prefix; check the selected files
and their recorded settings before interpreting the report.

## Controllers and configuration

The two circle experiments retain the same initial pose, a vertical x-z circle of
radius 0.04 m, cruise period 2 s and startup ramp 1 s. The fingertip starts
near (0.56, 0, 0.50) m; the reference reaches x approximately 0.64 m. The
shared wall is at x = 0.62 m, with outward normal (-1, 0, 0), stiffness
20000 N/m and damping 15 N s/m. The reference's maximum overlap of roughly
20 mm differs from actual penetration under contact force.

The perpendicular experiments share the initial pose and wall, using the
straight reference described below instead of the circle.

The seven active YAML files are:

| File                                          | Role                                                                                                                                 |
| --------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------ |
| `config/circle_tracking_baseline.yaml`      | Baseline initial pose, circle, Cartesian and nullspace gains.                                                                        |
| `config/circle_tracking_optimized_hqp.yaml` | Cartesian acceleration feedback, optional posture, mass objective, joint constraints, residual validity/uncertainty and QP settings. |
| `config/baseline_perp.yaml`                 | Perpendicular approach speed, ramp, final overlap, braking distance, hold and baseline gains.                                        |
| `config/optimized_hqp_perp.yaml`            | The same perpendicular reference and baseline tracking gains, with HQP, posture, mass objective and constraints.                     |
| `config/wall.yaml`                          | Shared virtual plane and contact stiffness/damping.                                                                                  |
| `config/observer.yaml`                      | Observer gain and residual thresholds.                                                                                               |
| `config/controllers.yaml`                   | Gazebo effort-controller joint list.                                                                                                 |

### Baseline

The baseline uses Cartesian acceleration feedback with position gain
100 s^-2 and velocity gain 20 s^-1. Its projected secondary acceleration is
`N @ (25 * (initial_q - q) - 10 * dq)`, with `N = I - pinv(J) @ J`.
The preferred posture is always `initial_q`; posture feedback and damping
are active from startup, including during contact. The controller keeps
tracking after collision detections. It logs directional mass along the
wall normal without minimizing it. It does not implement joint constraints;
the runner's saturation and safety stops remain active.

Shared baseline classes are in `fr3_sim/controllers/circle_tracking.py`:
`PositionTrackingController` and `DampedTrackingController`. Its
`user_qdd_null` contains posture feedback plus damping;
`user_qdd_command` contains total requested acceleration.

### Perpendicular baseline

`baseline_perp.py` reuses the circular baseline's Cartesian
acceleration feedback, projected posture feedback and joint damping. Only
the reference and finite experiment duration differ. It logs directional
mass without minimizing it; the observer monitors collisions without
changing the control law. It has no additional joint-limit constraints.

`methods/perpendicular_reference.py` generates a straight approach along
the negative wall normal from the initial fingertip position. With the
shared vertical wall this is horizontal motion towards increasing x. The
configured reference speed is 0.20 m/s, reached with a smooth 0.50 s ramp.
It crosses the wall plane at constant speed, continues for 6 mm, then
smoothly brakes over 4 mm to a final reference overlap of 10 mm. The final
reference is held for 0.15 s before the controller ends the run. Parameters
are configurable in `config/baseline_perp.yaml`.

With an initial gap of about 60 mm, the nominal reference reaches the wall
at about 0.55 s, brakes from about 0.58 to 0.62 s and finishes around 0.77 s.
These are reference timings: contact force and tracking error determine the
actual contact time, speed and penetration. Check `entry_speed_mps` in the
collision metrics before comparing controllers; a 0.20 m/s reference does
not guarantee that exact measured speed. The reference braking can affect
the contact response, so future comparisons must reuse this same reference.

The brief hold ends independently of collision detection and true wall
force. Launch duration or the existing safety stops can end the experiment
earlier. Additional saved signals are `user_normal_speed`,
`user_normal_speed_ref`, `user_approach_distance_ref`,
`user_reference_depth` and `user_approach_complete`.

### Optimized HQP: the baseline Cartesian tracking law with constrained redundancy

Baseline and optimized controllers now request the same Cartesian
acceleration feedback, with position gain 100 s^-2 and velocity gain
20 s^-1 on each axis:

$$
\begin{aligned}
a_{\mathrm{track}} &= \ddot p_d+K_p(p_d-p)+K_d(\dot p_d-J\dot q),\\
b &= a_{\mathrm{track}}-\dot J\dot q,\qquad \tau=M(q)u_{\mathrm{HQP}}+c(q,\dot q).
\end{aligned}
$$

The optimized controller asks the HQP to attain `J @ u = b` as closely as
the higher-priority constraints permit. Cartesian gains are configured as
`tracking.kp_position` and `tracking.kd_position`. Both controller families call
the same `CartesianAccelerationTracking` implementation in
`methods/cartesian_tracking.py`. The feedback is not multiplied by
`Lambda^-1`; Cartesian inertia is used only for the optimized controller's
dynamically consistent secondary projection. Both families record
`user_tracking_acceleration`, before the joint-space solution and its
constraints. There is no
collision-triggered retreat or release state.

```yaml
tracking:
  kp_position: 100.0          # [1/s^2], matching the baseline
  kd_position: 20.0           # [1/s], matching the baseline
  nullspace_damping: 10.0     # unchanged secondary damping
  posture_gain: 25.0          # unchanged attraction towards q_posture_ref
  startup_ramp_duration: 0.5  # unchanged secondary-objective activation
```

The baseline still realizes its task with a pseudoinverse and kinematic
nullspace projector. The optimized controller retains its HQP and
dynamically consistent redundancy projector. A common primary tracking
request does not imply identical joint torques when constraints or
secondary objectives intervene.

Each experiment's baseline and optimized controller use the same reference
class and configured trajectory. Both circular YAML files set the period
to 2 s; the perpendicular pair shares the approach, braking and
final-hold schedule. Changing only the optimized gains in a custom YAML
would change its tracking law: keep each pair's `tracking.kp_position` and
`tracking.kd_position` equal when comparing them.

Both active optimized configurations use three QP levels: minimum
relaxation of preventive preferences under hard constraints, Cartesian
tracking while preserving that relaxation, and secondary posture/mass/
damping while preserving the achieved Cartesian action.
The configured secondary gains are posture 25 s^-2, damping 10 s^-1 and
mass gain 0.1 N m rad/kg.

`tracking.startup_ramp_duration` smoothly activates posture attraction and
mass torque at the beginning of RUN, using a cubic scale from zero to one.
Its current value is 0.5 s; setting it to zero restores immediate activation.
Cartesian tracking, nullspace damping and all QP constraints remain active
throughout. The final constrained command is not multiplied by the ramp.
`user_startup_scale` records the factor and appears in the HQP status panel;
`user_tau_mass` records the scaled secondary preference. The observer and
detector remain active, so this does not hide startup events. Retuning the
duration changes the startup joint motion; recheck residuals and entry mass.

`q_posture_ref` supplies the existing offline low-mass posture. Removing that
key omits posture attraction in this controller. `initial_q` still defines
the settling pose and reference start. The analytical mass objective uses the
explicit fixed world direction `mass_minimization.direction` at the tool;
it does not infer an unknown contact point or read true contact force.
The fixed posture is computed offline with SciPy SLSQP, not HQP. During a
run, the online HQP considers attraction towards that fixed posture together
with the analytical mass gradient evaluated at the current joint state.

Both optimized configurations prefer a 0.15 rad preventive position margin
and 30 rad/s^2 acceleration magnitude. Those preferences are relaxable;
the 0.06 rad hard position margin and configured velocity/torque limits
remain mandatory. The acceleration preference is an experiment parameter,
not a datasheet value.
Torque constraints cover both command torque and nominal motor torque
including gravity. The returned command does not add gravity: Gazebo
already compensates it.

The residual affects constraint predictions only; it is not subtracted from
the tracking command. `disturbance_estimate` configures validity checks and
the empirical acceleration-error reserve. The reserve accounts for the
observed residual rate, filter lag, sample age and command delay. It is not
a certified bound on arbitrary impacts. Timing variations can make the
robust constraints incompatible, even before a physical joint limit is
reached. A failed hard-safety QP terminates the run with bounded damping
and a saved reason. Failed tracking or secondary solves can use the same
update's verified preceding solution. Only the configured preventive
preferences have slack; hard limits are not relaxed.

Reusable components are:

| Module under `fr3_sim/`              | Responsibility                                                                                     |
| -------------------------------------- | -------------------------------------------------------------------------------------------------- |
| `methods/circle_reference.py`        | Circular position, velocity and acceleration references.                                           |
| `methods/perpendicular_reference.py` | Smooth straight approach, braking and final hold along the wall normal.                            |
| `methods/cartesian_tracking.py`      | Shared Cartesian acceleration feedback and gain validation for baseline and optimized controllers. |
| `methods/mass_objective.py`          | Directional mass, analytical gradient and projected mass torque.                                   |
| `controllers/optimized_hqp.py`       | Reference-independent Cartesian acceleration tracking/HQP law and circular experiment adapter.     |
| `methods/joint_constraints.py`       | Delayed-state prediction and tightened joint bounds.                                               |
| `methods/hierarchical_qp.py`         | Strict two-level HQP and optional three-level HQP with normalized soft-constraint slack.           |
| `methods/optimize_posture.py`        | Standalone offline preferred-posture search.                                                       |

Within `optimized_hqp.py`, `Controller` selects the circle reference with
`sample(t)`; `optimized_hqp_perp.py` supplies `PerpendicularApproachReference`
to `HQPTrackingController`. OSQP and
SciPy are declared runtime dependencies and are present in the existing
container. Pinocchio supplies the kinematic and dynamic model.

### Perpendicular optimized HQP

`optimized_hqp_perp.py` uses the same Cartesian acceleration tracking law, gains,
residual validity checks and secondary-objective startup ramp as
`optimized_hqp.py`, with the soft-constraint mode described below.
Its reference is the same straight approach as
`baseline_perp.py`: 0.20 m/s, a 0.50 s speed ramp, 10 mm final reference
overlap, 4 mm braking distance and a 0.15 s hold. The nominal scheduled end
is around 0.77 s. Tracking remains active during contact; scheduled
completion is independent of detection and true contact force.

The perpendicular YAML supplies its own fixed `q_posture_ref`, selected
offline for the matching approach and wall geometry. Use the
[offline posture workflow](#offline-posture-workflow) to regenerate it.
Online mass minimization remains active within the HQP priorities; a static
mass-reduction target does not guarantee the same reduction at impact. Compare
entry mass together with actual collision speed, first-impact metrics,
tracking and joint margins before drawing conclusions. The unchanged
0.5 s secondary startup ramp finishes before the nominal reference contact
time of 0.55 s.

### Constraint hierarchy shared by both optimized controllers

Both optimized YAML files enable `safety.soft_constraints`. Position and
velocity predictions and actuator torque bounds remain hard. The hard QP
position margin is 0.06 rad, leaving a 0.01 rad buffer before the runner's
0.05 rad stop band. Command torque and nominal motor torque including
gravity retain their existing limits. The 0.15 rad preventive margin,
30 rad/s^2 acceleration preference and rate-based preventive barriers are
soft: their minimum required violations are explicit slack variables.

```yaml
safety:
  position_margin: 0.15       # preferred preventive margin
  acceleration_max: 30.0     # preferred acceleration magnitude
  soft_constraints:
    enabled: true
    hard_position_margin: 0.06
    acceleration_scale: 30.0 # [rad/s^2] normalization of each soft row group
    position_scale: 30.0
    velocity_scale: 30.0
```

The three QP levels first minimize normalized slack under the hard
constraints, then track Cartesian acceleration while preserving the attained
slack, and finally use remaining freedom for posture, mass and nullspace
damping while preserving the Cartesian result. If the tracking level fails,
the same update's verified safety solution remains available. Failure of the
hard safety level still terminates the run; observer validity and exception
guards remain active. The residual and its uncertainty still enter
predictions, without using true contact force or subtracting the residual
from the commanded torque. Predictive constraints remain conditional on
the assumed disturbance envelope, not a guarantee under arbitrary impacts.

This mode is enabled in `circle_tracking_optimized_hqp.yaml` and
`optimized_hqp_perp.yaml`. Disabling or omitting the option explicitly
selects the earlier strict two-level behavior. The comparison and plotting
read the selected mode from the log; they do not infer it from the shape
of the trajectory.

Both optimized experiments use the ten-panel HQP main figure, observer
figure and first-impact collision figure. They record the shared HQP
diagnostics; the perpendicular adapter additionally records its approach
reference and scheduled trial completion. `user_soft_slack_lower` and `user_soft_slack_upper`
record all seven joints for acceleration, position-barrier and
velocity-barrier groups; scalar maximum and normalized maximum slack are
also logged. The existing status panel shows normalized slack and safety-QP
status. The joint-margin panel distinguishes the preferred margin, hard QP
margin and runner stop band; no extra PNG is introduced.

### Offline posture workflow

`fr3_sim/methods/optimize_posture.py` searches at the first entering
reference/wall-plane intersection for either a circle or perpendicular
approach, with free orientation and an explicit joint margin.
`closest-target` minimizes displacement from the
baseline's pre-contact configuration while meeting the requested mass
reduction; `minimum-mass` minimizes mass directly.

For a fresh baseline log matching the current circle and wall:

```bash
source /ros2_ws/install/setup.bash
cd /ros2_ws/src/fr3_student_sim
baseline_log=$(ls -t runs/circle_baseline_[0-9]*.npz | head -n 1)

python3 -m fr3_sim.methods.optimize_posture \
  --task-config config/circle_tracking_baseline.yaml \
  --wall-config config/wall.yaml \
  --baseline-log "$baseline_log" \
  --selection closest-target \
  --joint-margin 0.15 \
  --reduction 0.30 \
  --output /tmp/low_mass_posture.yaml \
  --report /tmp/low_mass_posture.json
```

The output copies the input baseline task and adds `q_posture_ref`. Copy that
joint vector into `config/circle_tracking_optimized_hqp.yaml` to use the new
posture, keeping its tracking and QP settings. The temporary output is a
baseline-style task, not a replacement HQP configuration. Regenerate the
posture when model, initial pose, reference or wall geometry changes.

For a perpendicular baseline log matching the current approach and wall:

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

After a feasible candidate is found, copy only `q_posture_ref` from the
temporary output into `config/optimized_hqp_perp.yaml`, preserving its
tracking, QP and approach settings. The optimizer does not update that HQP
configuration automatically. The report records the baseline provenance
and static search constraints; the circle configuration and its report
remain separate. A static 30% target does not guarantee a 30% reduction at
actual impact during the short approach.

Store the circle posture and static search provenance in
[docs/optimized_posture.json](docs/optimized_posture.json). A static solution
does not guarantee its mass reduction or margins throughout a simulation.

## Runtime and collision monitoring

Gazebo adds gravity compensation, so commanded dynamics are
`M(q) qdd + C(q, dq) dq = tau + tau_ext`. The runner settles the arm, updates
the selected controller, clips actuator commands, adds virtual-wall torque
and saves the log. It stops when a joint comes within 0.05 rad of its
physical position limit or exceeds its speed limit. The virtual wall acts
only at the fingertip; its displayed box has no Gazebo collision geometry.

`FR3Model` provides position, world-aligned Jacobians, their time variation,
inertia, Coriolis, gravity and directional mass. The controller receives the
model, selected YAML as `self.params`, nominal `self.dt`, and measured
`q`, `dq` in `update(t, q, dq)`. Static wall geometry is available separately;
true external force is not a controller input. Values in `self.log` are
saved as `user_<name>`.

The model and runner are in `fr3_sim/simulation/`. Observer, detector and
monitor implementations are in `fr3_sim/monitoring/`.

`CollisionMonitor` coordinates the separate momentum observer and threshold
detector. Momentum is `M(q) dq`, residual is `K_O @ (p - p_hat)`, and the
gravity-compensated observer uses
`p_hat_dot = tau_applied + C.T @ dq + residual`, with `Mdot = C + C.T`.
Initialization uses measured momentum at the first RUN sample; implicit
Euler integration uses elapsed simulation time.

The observer input is reconstructed from delayed, saturated actuator-command
history averaged over each measurement interval, assuming 1 ms transport
delay. It excludes wall torque and simulator-added gravity. It is not motor
telemetry. Monitoring advances on each received RUN joint-state sample.
The configured gain is 30 s^-1 and threshold 0.1 N m, as scalars or seven
joint values. Any joint exceeding its absolute threshold triggers detection;
the residual norm is diagnostic only.
Detection and hand-body warning messages are printed after the run ends to
keep console I/O out of the control callback. Monitoring, detection callbacks
and event logging still advance during RUN.

The runner binds the monitor once. Baseline does not consume residuals for
control. HQP reads `snapshot(t)`, containing a read-only residual, timestamp,
age, sample interval, filter time constants, validity and reason. Its YAML
requires 0.2 s warmup, with maximum gap/age 0.01 s. Initialization uses a zero
estimate plus a separate reserve; stale/discontinuous data terminate with a
recorded reason. Snapshot validity does not remove low-pass lag or establish
a disturbance-error guarantee. Threshold events do not switch its tracking.

## Logs, plots and comparison

For `X.npz`, `plot_run` writes `X.png`, `X_collision.png` and, when residuals
are logged, `X_observer.png`. The baseline main figure shows state and user
signals. The HQP main figure has ten panels: position/reference, position
error, joint torques, joint velocities, contact force, directional mass,
actual/reference normal speed, joint margins, QP/observer status and compute
time. `user_tracking_acceleration` stores the primary acceleration
reference. Extra QP and disturbance diagnostics remain in the NPZ.

The observer figure shows the seven residuals, thresholds, residual norm
and detection times. The collision figure has six panels: directional mass,
normal speed, force, penetration, normal impulse and pre-contact normal
kinetic energy. Metrics cover only the first compression-release cycle,
ending at detachment or the start of recompression, whichever occurs first.
Recompression uses the interpolated zero crossing from outward to inward
normal speed. The onset is first recorded `F_n > 0`, independent of observer
threshold events. Time panels include 50 ms before onset.

Mass and speed use the sample strictly before contact, with
`E_n = 0.5 * m_n * v_n**2`. Impulse uses trapezoids at actual timestamps.
Missing data and incomplete cycles are marked explicitly. Cumulative
positive contact work is no longer reported. True wall force/torque is
logged for offline evaluation only.

Use the four launch commands above to collect new pairs with the same
requested real-time factor (0.20), control rate and GUI mode. Keep the
initial state, reference, wall, observer and realism settings equal within
each pair. A comparison reads two saved NPZ files; it does not run a
simulation or replace their recorded settings with today's YAML files.

Compare the latest circular baseline and optimized logs:

```bash
source /ros2_ws/install/setup.bash
cd /ros2_ws/src/fr3_student_sim
ros2 run fr3_student_sim compare_runs \
  "$(ls -t runs/circle_baseline_*.npz 2>/dev/null | head -n 1)" \
  "$(ls -t runs/circle_optimized_hqp_*.npz 2>/dev/null | head -n 1)" \
  --output runs/optimized_hqp_comparison.json
```

Compare the latest perpendicular baseline and optimized logs:

```bash
source /ros2_ws/install/setup.bash
cd /ros2_ws/src/fr3_student_sim
ros2 run fr3_student_sim compare_runs \
  "$(ls -t runs/baseline_perp_*.npz 2>/dev/null | head -n 1)" \
  "$(ls -t runs/optimized_hqp_perp_*.npz 2>/dev/null | head -n 1)" \
  --output runs/optimized_hqp_perp_comparison.json
```

These commands also accept run-name suffixes such as `_tracking_check`.
They select the most recently modified file in each family, not an
automatically matched pair. Inspect each report's `log_path`,
`configured_duration_s`, `duration_s`, `stop_reason` and recorded scenario;
for the full circular experiment, both configured durations must be 20 s.
Replace the two quoted selections with explicit filenames when choosing a
specific pair. Missing inputs produce a CLI error before any log is loaded
or report written.

The report is printed as JSON and saved to `--output`, replacing that file
when it already exists. Reports are snapshots: rerun `compare_runs` after
an analysis change or after choosing new logs. Generating a new report
from an old NPZ does not rerun that experiment. The comparison creates no
PNG files; use the separate plotting commands for figures.

The JSON reports entry mass/speed, tracking, first-cycle force/impulse/
penetration, saved joint limits, torque saturation, QP statuses and timing.
It evaluates mass and speed at the sample immediately before the first
contact, and computes the reduction as `1 - optimized_mass / baseline_mass`.
The default mass-reduction target is 30%; the allowed relative entry-speed
difference is 5%. Override them with `--mass-reduction-target` and
`--entry-speed-tolerance` as fractions, for example `0.30` and `0.05`.

Read the fields under `comparison` as follows:

| Field                        | Meaning                                                                                                                                                                                                                                                                                |
| ---------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `primary_tracking.status`  | `same`, `different` or `unverifiable`, based on the recognized controller and explicit law/gains in the saved configuration. Scalars and equivalent three-axis gain vectors compare equally.                                                                                     |
| `scenario`                 | Per-field agreement of initial pose, circle/approach, wall, realism, control rate and configured duration.                                                                                                                                                                             |
| `target_metrics_met`       | Initial checks: recognized baseline, matching scenario and mass direction, complete first impacts, available precontact errors, positive entry speeds within tolerance and mass reduction at least the target. It does not include the later tracking-law, completion or limit checks. |
| `full_criterion_met`       | All implemented checks pass, including matching primary tracking, completion, sampled limits and the applicable QP/torque checks.                                                                                                                                                      |
| `checks`, `unmet_checks` | Individual pass/fail checks and the names of all failed checks. Inspect these before interpreting either summary flag.                                                                                                                                                                 |

A full comparison requires matching primary tracking law and Cartesian
gains. Unknown or incomplete law/gain metadata fail that check. This checks
the configuration recorded in the log, rather than identifying a source
revision. Matching primary tracking does not isolate the effect of the mass
gradient: HQP constraints and secondary posture policies still differ.

For soft-constraint runs it separately reports the hard margin, runner stop
band, safety-QP status, normalized slack and usage of each soft group.
Entering the preferred 0.15 rad margin or exceeding the preferred
acceleration is reported but does not by itself fail the hard-safety
criterion. Missing or nonfinite hard-constraint diagnostics do not establish
compliance. Logs recorded with strict constraints retain their acceleration
and protected-margin checks. Constraint checks concern saved samples, not
the continuous motion between them.

Comparison checks distinguish circular and perpendicular references. For a
finite approach, scheduled completion after the final hold counts as normal
completion; an earlier controller failure does not. Matching references do
not establish equal measured entry speeds: inspect the reported speeds.
Older perpendicular logs without `user_trial_complete` do not confirm
scheduled completion. Their first-impact metrics remain readable; rerun
the baseline with the current controller for full completion validation.

The full criterion does not check matching observer settings or real-time
factors, and it has no acceptance threshold for missed samples, controller
runtime or tracking-error RMS/max. These diagnostics are reported separately.
Force peak, impulse and penetration are measured but do not have automatic
improvement thresholds. Inspect `baseline.timing` and `candidate.timing`,
including `real_time_factor_measured` and `missed_samples_pct`, as well as
the tracking summaries. A true `full_criterion_met` is therefore limited to
the implemented checks, not a general validation of experimental timing or
proof that mass minimization alone caused an impact change.

## Timing and launch options

`controller_updated` identifies actual updates. `ctrl_ms` and the QP timing
fields measure wall-clock time; controller timing excludes observer,
virtual-wall evaluation and ROS publication. Summaries use only update rows
and report mean, p95, maximum, and counts over `1000/control_rate` ms. That
comparison assumes RTF = 1; it is distinct from lost simulated-time samples.
Plant timing separately records missing 1 kHz samples, gaps and maximum gap.
New logs also contain `callback_ms` for the complete RUN callback, including
monitoring, control, wall evaluation, publication and row construction. It
excludes executor/transport wait, final saving and deferred console output.
Metadata records `real_time_factor_requested` and `numeric_threads`;
comparison timing includes the requested wall-clock budgets and callback
statistics. At 1000 Hz and RTF 0.20, that budget is 5 ms per sample.
Older logs can lack these fields. Averages alone do not establish real-time
feasibility. The runner defaults numeric-library thread counts to one while
respecting explicit environment overrides.
For three-level runs, `user_qp_safety_ms` and the comparison summary also
report the wall time of the leading slack-minimization QP. This stage is
included in `Controller.update` timing.

| Option                               | Default                            | Meaning                                                                                                    |
| ------------------------------------ | ---------------------------------- | ---------------------------------------------------------------------------------------------------------- |
| `controller`                       | `baseline.py`                    | One of the four listed controllers, an absolute custom path, or `none` for simulator only.              |
| `task_config`                      | Matching controller YAML           | Override the selected task.                                                                                |
| `wall`, `observer`               | `true`                           | HQP requires the observer; baseline can run without it.                                                    |
| `wall_config`, `observer_config` | Shared YAML files                  | Override wall or observer parameters.                                                                      |
| `duration`                         | `20.0`                           | Maximum RUN duration in simulated seconds.                                                                 |
| `stop_at_detachment`               | `false`                          | Optionally end at first wall detachment.                                                                   |
| `control_rate`                     | `1000`                           | Nominal controller frequency in simulated Hz.                                                              |
| `real_time_factor`                 | `0.5`                            | Requested simulation speed; use 0.20 for all four experiments.                                             |
| `run_name`, `log_dir`            | Controller name, package `runs/` | Log prefix and destination.                                                                                |
| `gui`                              | `true`                           | Open Gazebo GUI.                                                                                           |
| `partition`                        | Empty                              | Gazebo transport partition.                                                                                |

Realism settings `joint_friction`, `mass_error`, `noise_q`, `noise_dq` and
`seed` default to the nominal noiseless model. `run_batch` accepts a supplied
batch YAML and selects the matching task for the four active controllers.
When samples are missed, reduce simulation speed and inspect timing. A
controller exception saves the current log and terminates the run.
