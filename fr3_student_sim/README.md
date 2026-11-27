# fr3_student_sim

Four controllers for two FR3 experiments in Gazebo with joint-torque
control: circular tracking and perpendicular collision. All control the
position of `fr3_fingertip`; gripper orientation remains free.

| Experiment                                                         | Controller in `fr3_sim/controllers/` | Configuration                                 |
| ------------------------------------------------------------------ | -------------------------------------- | --------------------------------------------- |
| Circular tracking with nullspace posture feedback and scheduled returns | `baseline.py`                        | `config/circle_tracking_baseline.yaml`      |
| Circular full-online mass reduction with one QP and scheduled returns | `optimized_full_online_qp_circular.py`  | `config/optimized_full_online_qp_circular.yaml` |
| Perpendicular approach with the baseline control law               | `baseline_perp.py`                   | `config/baseline_perp.yaml`                 |
| Perpendicular full-online mass reduction with one QP                | `optimized_full_online_qp_perp.py`  | `config/optimized_full_online_qp_perp.yaml` |

[Progress](docs/progress.md) retains the development history. Earlier controller
variants and their task YAMLs are archived under `fr3_student_sim/trash/`.

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
controller hierarchy is in `controllers/circle_tracking.py`. All four controllers
use the primary feedback in `methods/cartesian_tracking.py`. The single-QP
controller law is shared through `controllers/weighted_qp_tracking.py`.
`controllers/optimized_full_online_qp_circular.py` uses online mass descent
during each circle and returns physically to `initial_q` between laps,
using `CircularLapReference` and the single-QP update shared with
`controllers/optimized_full_online_qp_perp.py`.
Numerical helpers remain in `methods/`.
`simulation/fr3_model.py` wraps Pinocchio, while
`simulation/runner.py` connects controllers to Gazebo.

Plots are in `analysis/plot_run.py`, `analysis/plot_collision.py`,
`analysis/plot_all_collisions.py` and
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

Circular full-online QP

```bash
ros2 launch fr3_student_sim student.launch.py \
  controller:=optimized_full_online_qp_circular.py \
  task_config:=/ros2_ws/src/fr3_student_sim/config/optimized_full_online_qp_circular.yaml \
  run_name:=optimized_full_online_qp_circular \
  log_dir:=/ros2_ws/src/fr3_student_sim/runs \
  wall:=true observer:=true stop_at_detachment:=false \
  control_rate:=1000 real_time_factor:=0.20 duration:=20
```

This variant has no posture attraction during tracking. After each complete
circle it pauses to return physically to `initial_q`, then restarts the
reference and mass ramps. The return time is included in the 20 s run.
The observer can be enabled to record collision detection.

Use real-time factor 0.20 for all experiments to give the controller more wall-clock time
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
after its approach, braking and short hold, nominally about 0.57 simulated
seconds after RUN starts with the default geometry. `duration:=2` is an
upper bound, not an additional final hold.

Perpendicular full-online mass reduction with one QP:

```bash
ros2 launch fr3_student_sim student.launch.py \
  controller:=optimized_full_online_qp_perp.py \
  task_config:=/ros2_ws/src/fr3_student_sim/config/optimized_full_online_qp_perp.yaml \
  run_name:=optimized_full_online_qp_perp \
  log_dir:=/ros2_ws/src/fr3_student_sim/runs \
  wall:=true observer:=true stop_at_detachment:=false \
  control_rate:=1000 real_time_factor:=0.20 duration:=2
```

This variant uses the same approach reference and Cartesian PD gains. A single
QP weights tracking error against the acceleration preference from online mass
descent and nullspace damping. Joint braking limits the outward speed in
proportion to the distance from each joint limit, using the measured position
and velocity. The QP also enforces the hard torque limits. It uses no disturbance estimate, command
delay prediction, or separate acceleration cap. Set `observer:=true` to record
collision detection independently of the controller. The distinct log prefix
identifies these runs in the plot and comparison commands.

Launch defaults to baseline. Omitting `task_config` selects the matching
controller YAML; the launch defaults themselves remain
1000 Hz, real-time factor 0.5 and duration 20 s.

The four built-in selectors are listed in the table above. An absolute custom
controller-file path is also supported.
Save the figures for the experiment you choose. A complete first impact with
observer data produces up to five figures for circular runs and four for
perpendicular runs. In each new shell:

```bash
source /ros2_ws/install/setup.bash
cd /ros2_ws/src/fr3_student_sim
```

Circular baseline:

```bash
ros2 run fr3_student_sim plot_run "$(ls -t runs/circle_baseline_*.npz 2>/dev/null | head -n 1)" --no-show
```

Circular full-online QP

```bash
ros2 run fr3_student_sim plot_run "$(ls -t runs/optimized_full_online_qp_circular_*.npz 2>/dev/null | head -n 1)" --no-show
```

Perpendicular baseline:

```bash
ros2 run fr3_student_sim plot_run "$(ls -t runs/baseline_perp_*.npz 2>/dev/null | head -n 1)" --no-show
```

Perpendicular full-online mass reduction with one QP:

```bash
ros2 run fr3_student_sim plot_run "$(ls -t runs/optimized_full_online_qp_perp_*.npz 2>/dev/null | head -n 1)" --no-show
```

These patterns accept run-name suffixes such as `_tracking_check`. Check
`duration_s` and `stop_reason` before using a log for comparison. A short
circular tracking check cannot replace a full 20 s run.
## Controllers and configuration

The circular experiments share the same initial pose, a vertical x-z circle of
radius 0.04 m, cruise period 2 s and startup ramp 1 s. The fingertip starts
near (0.56, 0, 0.50) m; the reference reaches x approximately 0.64 m. The
shared wall is at x = 0.62 m, with outward normal (-1, 0, 0), stiffness
20000 N/m and damping 15 N s/m. The reference's maximum overlap of roughly
20 mm differs from actual penetration under contact force.

The perpendicular experiments share the initial pose and wall, using the
straight reference described below instead of the circle.

The active YAML files are:

| File                                          | Role                                                                                                                                 |
| --------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------ |
| `config/circle_tracking_baseline.yaml`      | Baseline initial pose, circle, scheduled return, Cartesian and nullspace gains.                                                      |
| `config/optimized_full_online_qp_circular.yaml` | Online circular tracking and physical return using one QP with joint braking and torque limits. |
| `config/baseline_perp.yaml`                 | Perpendicular approach speed, ramp, final overlap, braking distance, hold and baseline gains.                                        |
| `config/optimized_full_online_qp_perp.yaml` | Perpendicular reference, weighted tracking and mass objective, joint braking and torque limits. |
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
configured reference speed is 0.20 m/s, reached with a smooth 0.10 s ramp.
It crosses the wall plane at constant speed, continues for 6 mm, then
smoothly brakes over 4 mm to a final reference overlap of 10 mm. The final
reference is held for 0.15 s before the controller ends the run. Parameters
are configurable in `config/baseline_perp.yaml`.

With an initial gap of about 60 mm, the nominal reference reaches the wall
at about 0.35 s, brakes from about 0.38 to 0.42 s and finishes around 0.57 s.
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

### Full-online optimization with one QP

All four controllers use the same Cartesian acceleration reference, with
position gain 100 s^-2 and velocity gain 20 s^-1 on each axis.

$$
a_{\mathrm{track}}=\ddot p_d+K_p(p_d-p)+K_d(\dot p_d-J\dot q),
\qquad b=a_{\mathrm{track}}-\dot J\dot q.
$$

The baselines realize the task with a pseudoinverse and a kinematic
nullspace projector. The optimized controllers use the mass-weighted
pseudoinverse `Jbar` and its nullspace projector `N = I - Jbar @ J`.
Each update computes the directional effective mass and its analytical
gradient at the measured configuration. The projected gradient torque
and nullspace damping set the joint-acceleration preference. During
the circular RETURN, the preference also includes projected attraction
towards `initial_q`.

The solver in `controllers/weighted_qp_tracking.py` chooses the actuator
acceleration `u` by minimizing

$$
\frac{w}{2}\|Ju-b\|^2 + \frac{1}{2}\|u-u_{\mathrm{pref}}\|^2,
\qquad \tau=Mu+c.
$$

Both optimized YAMLs set `tracking.qp_weight` to 1000. The QP enforces
command and nominal motor-torque limits, including gravity in the latter.
Position and velocity enter the braking bounds through the current joint
measurements. In both default configurations, the physical position margin
checked in the log is 0.06 rad. The circular controller uses a further
0.09 rad buffer in the braking bounds to start braking sooner.

| Optimized experiment | Mass gain (N m rad/kg) | Nullspace damping (s^-1) | Mass ramp (s) |
| --- | ---: | ---: | ---: |
| Circular tracking | 2 | 24 | 0.10 |
| Perpendicular approach | 4 | 12 | 0.10 |

The two QP controllers scale mass torque with a cubic startup ramp. The
perpendicular trial uses the baseline's approach, braking and hold schedule.
For each comparison, use matching reference and Cartesian PD settings and
check the measured entry speeds.

### Circular full-online controller

`optimized_full_online_qp_circular.py` uses one weighted QP in each update of
both phases. During tracking it follows one complete `CircleReference`
revolution with online mass gain 2, nullspace damping 24 s^-1 and no posture
attraction, even if a custom YAML supplies `tracking.posture_gain` or
`q_posture_ref`. Cartesian gains remain 100/20. No offline optimizer is used.

The default circular baseline and full-online controllers share the same
return reference. At the end of a revolution, a 0.20 s quintic transition
joins the current position, velocity and acceleration to the starting point
at rest. The reference then holds that point as the joints move towards
`initial_q`. The baseline keeps its posture feedback and damping throughout.
In the full-online controller the mass term fades out during the transition,
and return attraction and damping blend to 25 s^-2 and 10 s^-1.

Both default configurations set `circle.lap_return.duration` to 2.0 s,
including braking. The 2.5 s revolution and 2.0 s return define a common
4.5 s schedule on the simulation clock. At each scheduled restart, every
joint must be within 0.001 rad of `initial_q` with speed at most 0.005 rad/s.
If the arm is outside those tolerances at the scheduled restart, the trial
ends with a recorded reason. For a custom `lap_return` without `duration`,
the restart occurs once the joints settle, subject to `timeout`.

The reference restarts on its local phase clock, with the same QP solver
instance across laps. The optional observer runs on the absolute RUN clock
for detection. The 1 s circle ramp
and 0.10 s mass/damping ramp restart on each lap. Return attraction ramps
over the 0.20 s braking transition. The QP uses joint braking bounds from
the measured position and velocity and enforces torque limits in both phases.
The existing 0.15 rad preventive clearance is used by the braking bounds,
with a 0.09 rad buffer above the 0.06 rad hard position margin.

`user_lap_index`, `user_lap_returning`, `user_lap_time`, `user_q0_error` and
`user_q0_speed` record the schedule and return criteria. The shared schedule
aligns the approaches across laps. Actual contact times and recontacts depend
on the measured motion and contact response. The comparison checks the
recorded circle settings and lap-phase diagnostics.

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

The runner uses the optional observer to record residuals, detect contact
and reconstruct contact force for analysis. The four active controllers
compute their commands from measured joint position and velocity.
## Logs, plots and comparison

For `X.npz`, `plot_run` writes `X.png`, `X_collision.png` and, when residuals
are logged, the two-column `X_observer.png`. A complete first impact with the
robot model, wall direction and observer gain available also produces
`X_force_estimation.png`. It compares the simulated normal force with the
estimate from the filtered residual and the estimate after filter inversion.
The baseline main figure shows state and user
signals. The optimized main figure has ten panels: position/reference, position
error, joint torques, joint velocities, contact force, directional mass,
actual/reference normal speed, joint margins, QP/observer status and compute
time. `user_tracking_acceleration` stores the primary acceleration
reference. Optimized logs also contain the QP status, bounds and solve time.

The observer figure shows the seven residuals, thresholds, residual norm
and detection times. With the FR3 model and wall normal available, it also
compares the normal force estimated from the residual with the simulated
wall force. For a single fingertip contact along the known unit normal `n`,
it first reconstructs the joint torque as `r[k] + (r[k] - r[k-1]) / (dt * K_O)`
using the recorded observer gain and sample interval. It projects that torque
onto `g = J_tip(q).T @ n` with `F_hat = (g @ torque) / (g @ g)`.
The first sample has no estimate. Logs without an observer gain use the
filtered residual directly. Filter inversion can amplify measurement noise
and is intended here for nominal simulations.

The collision figure has six panels: directional mass,
normal speed, force, penetration, normal impulse and pre-contact normal
kinetic energy. Metrics cover only the first compression-release cycle,
ending at detachment or the start of recompression, whichever occurs first.
Recompression uses the interpolated zero crossing from outward to inward
normal speed. The onset is first recorded `F_n > 0`, independent of observer
threshold events. Time panels include 50 ms before onset.

Both circular controllers additionally produce `X_all_collisions.png`:
the same six quantities over the entire run, with numbered contact episodes
starting at each transition to `F_n > 0`. Impulse resets at each contact and
uses the same compression-release end rules; energy uses the sample before
each contact. Incomplete cycles are marked explicitly. Observer detections
do not create additional contact episodes. The existing `X_collision.png`
and comparison metrics still describe only the first impact.
Each panel has one numeric label: the contact metric with the largest
absolute value across all episodes, retaining its original sign.

Mass and speed use the sample strictly before contact, with
`E_n = 0.5 * m_n * v_n**2`. Impulse uses trapezoids at actual timestamps.
Missing data and incomplete cycles are marked explicitly. Cumulative
positive contact work is no longer reported. True wall force/torque is
logged for offline evaluation only.

Use the baseline and full-online optimized launch commands above to collect new pairs with the same
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
  "$(ls -t runs/optimized_full_online_qp_circular_*.npz 2>/dev/null | head -n 1)" \
  --output runs/optimized_full_online_qp_circular_comparison.json
```

Compare the latest perpendicular baseline and optimized logs:

```bash
source /ros2_ws/install/setup.bash
cd /ros2_ws/src/fr3_student_sim
ros2 run fr3_student_sim compare_runs \
  "$(ls -t runs/baseline_perp_*.npz 2>/dev/null | head -n 1)" \
  "$(ls -t runs/optimized_full_online_qp_perp_*.npz 2>/dev/null | head -n 1)" \
  --output runs/optimized_full_online_qp_perp_comparison.json
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
when it already exists and is writable. Rerun `compare_runs` after an analysis
change or after choosing new logs. Generating a new report
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
revision. The comparison measures the overall effect of the control laws.
Constraints and posture or damping preferences can also affect impact metrics.

The single-QP runs report the solver status, hard joint margin and torque
checks. The checks use saved samples. Logs from archived controllers remain
readable, including their recorded soft-constraint diagnostics.
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
For optimized runs, `user_qp_ms` measures the single solve, which is included
in `Controller.update` timing.
| Option                               | Default                            | Meaning                                                                                                    |
| ------------------------------------ | ---------------------------------- | ---------------------------------------------------------------------------------------------------------- |
| `controller`                       | `baseline.py`                    | One of the listed controllers, an absolute custom path, or `none` for simulator only.                     |
| `task_config`                      | Matching controller YAML           | Override the selected task.                                                                                |
| `wall`, `observer`               | `true`                           | Enable the wall and collision observer. The observer is optional for the baselines and single-QP variants.  |
| `wall_config`, `observer_config` | Shared YAML files                  | Override wall or observer parameters.                                                                      |
| `duration`                         | `20.0`                           | Maximum RUN duration in simulated seconds.                                                                 |
| `stop_at_detachment`               | `false`                          | Optionally end at first wall detachment.                                                                   |
| `control_rate`                     | `1000`                           | Nominal controller frequency in simulated Hz.                                                              |
| `real_time_factor`                 | `0.5`                            | Requested simulation speed; use 0.20 for all experiments.                                                   |
| `run_name`, `log_dir`            | Controller name, package `runs/` | Log prefix and destination.                                                                                |
| `gui`                              | `true`                           | Open Gazebo GUI.                                                                                           |
| `partition`                        | Empty                              | Gazebo transport partition.                                                                                |

Realism settings `joint_friction`, `mass_error`, `noise_q`, `noise_dq` and
`seed` default to the nominal noiseless model. `run_batch` accepts a supplied
batch YAML and selects the matching task for the listed controllers.
When samples are missed, reduce simulation speed and inspect timing. A
controller exception saves the current log and terminates the run.
