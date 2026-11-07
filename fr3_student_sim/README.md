# fr3_student_sim

A Franka FR3 in Gazebo that you control with joint torques from **one Python
file**. You do not need to know ROS 2: you write a `Controller` class
(joint state in → torques out) and the package does everything else.

## What is in the package

| What | Where | You … |
|---|---|---|
| **Your controller** | `students/my_controller.py` | **write it** (the only code you edit) |
| Examples | `students/examples/`: `zero_torque.py` (the arm floats), `joint_pd.py` (joint PD), `cartesian_pd.py` (the fingertip follows the circle with a Cartesian "spring") | read, run, copy from |
| Task parameters | `config/task.yaml`: start pose, circle (4 cm, 8 s), gains of your 3R study. Reaches your controller as `self.params`; add your own keys | edit |
| The wall | `config/wall.yaml`: vertical plane at x = 0.62 m, `k_e` = 20000 N/m, `b_e` = 15 N s/m | edit |
| Collision monitoring | `config/observer.yaml`: momentum-observer gain and per-joint residual thresholds | configure |
| Robot model | `self.robot` (`fr3_student_sim/fr3_model.py`): kinematics, M, C, g, effective mass of the **simulated** robot | call it |
| Plots and contact facts | `ros2 run fr3_student_sim plot_run <log>` | run it |
| Many experiments in one go | `ros2 run fr3_student_sim run_batch <file>`, example in `config/batch_example.yaml` | run it |
| Checks of the package | `test/` (see Troubleshooting) | run them if something looks broken |

What the package does for you at every run: starts Gazebo with the FR3,
**compensates gravity** (as the real robot does), moves the arm to the start pose
(about 2 s, until it is at rest), calls your `update` at 1 kHz, clips your torques to the FR3 limits, applies
the wall force, monitors collisions independently of the tracking law, stops
the robot safely if something goes wrong, and saves a log.

The scene: at the start pose the tool points horizontally at the wall (+x),
fingertip at (0.56, 0, 0.50) m in the robot base frame (x forward, z up). The circle
lies in the vertical x-z plane and starts there. The wall is vertical, with
outward normal (-1, 0, 0), at x = 0.62 m. The desired circle reaches
x approximately 0.64 m, giving about 20 mm of reference overlap at its
farthest point. Actual penetration depends on the robot's contact motion.

## 1. Run it

Everything runs inside the `franka_ros2` Docker container.

```bash
# on the host, once per boot
cd ~/Git/franka_ros2 && docker compose up -d && xhost +local:docker

# open a terminal in the container (as many as you like)
docker exec -it franka_ros2 bash

# in the container, once (again only if you ADD a file)
cd /ros2_ws && colcon build --symlink-install --packages-select fr3_student_sim
```

Then, in every new terminal:

```bash
source /ros2_ws/install/setup.bash
cd /ros2_ws/src/fr3_student_sim

ros2 launch fr3_student_sim student.launch.py controller:=examples/cartesian_pd.py   # an example
ros2 launch fr3_student_sim student.launch.py                                        # YOUR controller
ros2 run fr3_student_sim plot_run runs/<newest file>.npz                             # look at the run
```

The Gazebo window opens and `duration` sets the maximum RUN length. A controller's
`done` flag, safety stop or `Ctrl+C` can end it earlier; the log is saved in
`fr3_student_sim/runs/`. Edits to your controller work at the
next launch (no rebuild).

The circle experiment has five separately selectable controllers:

| Variant | Controller in `students/circle_tracking/` | Task configuration |
|---|---|---|
| Minimal position tracking | `controller.py` | `config/circle_tracking.yaml` |
| Position tracking with nullspace velocity damping | `controller_nullspace_damping.py` | `config/circle_tracking_nullspace_damping.yaml` |
| Damped tracking with directional mass minimization | `controller_mass_minimization.py` | `config/circle_tracking_mass_minimization.yaml` |
| Mass-minimizing tracking with collision release | `controller_hybrid.py` | `config/circle_tracking_hybrid.yaml` |
| Mass-minimizing tracking with immediate horizontal retreat | `controller_hybrid_retreat.py` | `config/circle_tracking_hybrid_retreat.yaml` |

The damping variant reuses the minimal controller and adds only
`qdd_null = -kd_nullspace * (I - J_pinv @ J) @ dq`, contributing
`M(q) @ qdd_null` to its torque. The trajectory, Cartesian gains and observer
are the same. Its configuration starts with `kd_nullspace: 4.0` [1/s]; zero
recovers the minimal command. It logs `dq_null` and `qdd_null` in addition to
the existing signals, with `qdd_command` containing the total requested
acceleration. Damping does not impose a desired posture or enforce joint limits.

The mass-minimization variant extends that damping controller with the same
projected mass-gradient torque as the straight experiment. It uses the fixed
wall normal, gain 0.01 and gradient step 1e-4 rad. Setting the mass gain to
zero recovers the damped circular command. It keeps the existing duration
and stop conditions and logs mass, gradient, requested mass torque and normal
speeds for the existing plot command.

After building the package to install the new files, choose one command:

```bash
# Minimal reference controller:
ros2 launch fr3_student_sim student.launch.py \
  controller:=circle_tracking/controller.py \
  task_config:=/ros2_ws/src/fr3_student_sim/config/circle_tracking.yaml \
  run_name:=circle_minimal wall:=true

# Variant with nullspace damping:
ros2 launch fr3_student_sim student.launch.py \
  controller:=circle_tracking/controller_nullspace_damping.py \
  task_config:=/ros2_ws/src/fr3_student_sim/config/circle_tracking_nullspace_damping.yaml \
  run_name:=circle_nullspace_damping wall:=true

# Variant with nullspace damping and mass minimization:
ros2 launch fr3_student_sim student.launch.py \
  controller:=circle_tracking/controller_mass_minimization.py \
  task_config:=/ros2_ws/src/fr3_student_sim/config/circle_tracking_mass_minimization.yaml \
  run_name:=circle_mass_minimization wall:=true \
  control_rate:=1000 real_time_factor:=0.20
```

The run names give the log files distinct prefixes. Each log already records
the selected controller path and task parameters. For a comparison, keep
wall/observer settings, initial pose, trajectory and Cartesian gains equal
and change only the controller variant. New control laws get separate files;
gain changes belong in configuration files.

### Hybrid tracking and collision release

`students/circle_tracking/controller_hybrid.py` uses
`config/circle_tracking_hybrid.yaml`. TRACK inherits the circular
mass-minimization controller, with the same position-only reference, free
orientation, Cartesian gains, nullspace damping and fixed-wall-normal mass
objective. On the first GMO detection, it switches once to RELEASE and
commands only `tau = -D_R @ dq` on all seven joints. The positive diagonal
entries in `reaction.damping` are `[30, 30, 30, 20, 10, 10, 5]` [N m s/rad].
This is direct joint-torque damping, distinct from `tracking.kd_nullspace`
[1/s]; tracking, nullspace damping, mass torque and Coriolis compensation
are inactive in RELEASE. The simulator keeps its gravity compensation.

RELEASE stays latched until the run ends, even if the residual drops below
threshold. It does not set `done` or stop when contact is detected. The
command below keeps recording after detachment and has a maximum RUN duration
of 20 s; existing safety stops, exceptions or `Ctrl+C` can end it earlier.
Wall interaction, the observer and logging remain active. This controller
requires `observer:=true`; monitoring-disabled launch is rejected for it.

Build for the new files as described above, then run inside the container:

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

cd /ros2_ws/src/fr3_student_sim
latest_hybrid_log=$(ls -t runs/circle_hybrid_*.npz | head -n 1)
ros2 run fr3_student_sim plot_run "$latest_hybrid_log" --no-show
```

The log adds `user_release_active`, `user_release_start_time` and the seven
requested `user_tau_release` torques before clipping. The release timestamp
records the first RELEASE command update, which can follow the first
detection timestamp. In RELEASE, mass is still evaluated once per update and
normal speed remains logged. Mass torque and nullspace acceleration are zero;
inactive references, tracking errors, requested acceleration, mass gradient,
nullspace velocity and reference normal speed are marked NaN rather than
presented as active targets. The existing plots display these user signals.

### Hybrid tracking and immediate horizontal retreat

`students/circle_tracking/controller_hybrid_retreat.py` uses its separate
`config/circle_tracking_hybrid_retreat.yaml`. TRACK is the same circular
mass-minimization law. The first GMO detection latches RETREAT directly;
the first controller update after that event captures the current fingertip
position and starts the retreat reference. No waiting phase or velocity-stop
criterion precedes the retreat. This variant uses the same callback binding
as the release controller and requires `observer:=true`.

The goal is 3 cm from that captured position along the wall's outward normal
projected onto the horizontal x-y plane and normalized (currently -x).
`retreat.distance: 0.03` [m] and `retreat.duration: 2.0` [s] configure the
motion. `students/circle_tracking/retreat_reference.py` supplies a quintic
reference that starts and ends with zero reference velocity and
acceleration, then holds the goal until the run ends. RETREAT uses the existing
position-only tracking law with Cartesian gains 100 [1/s^2] and 20 [1/s],
nullspace damping 4 [1/s] and Coriolis compensation; orientation remains free.
Mass torque is zero during RETREAT. Simulator gravity compensation continues.
The controller does not resume the circle or set `done` when the 2 s reference
finishes; the launch duration and existing stop conditions still end the run.

Build for the added files, then launch and plot inside the container:

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

cd /ros2_ws/src/fr3_student_sim
latest_retreat_log=$(ls -t runs/circle_hybrid_retreat_*.npz | head -n 1)
ros2 run fr3_student_sim plot_run "$latest_retreat_log" --no-show
```

The log adds `user_retreat_active` (latched 0/1),
`user_retreat_start_time` (NaN before the first RETREAT command) and
`user_retreat_complete` (0/1 for completion of the reference, rather than a
measured-arrival condition). Position,
velocity and acceleration references, tracking errors and requested joint
acceleration remain valid during RETREAT and the final hold. Mass and normal
speed diagnostics continue; `user_tau_mass` is zero and
`user_mass_gradient` is NaN during RETREAT. Wall interaction, monitoring,
logging and existing safety stops remain active.

### Straight approach with directional mass minimization

`students/straight_mass_minimization/controller.py` is a separate position-only
experiment, with free orientation, its own reference module and
`config/straight_mass_minimization.yaml`. Mass-minimization tracking uses
the shared `students/utils/mass_objective.py`. The controller loader makes the
`students/` root importable for `from utils.mass_objective import ...`; the
helper's mathematics is unchanged by the move. The straight controller keeps
Cartesian acceleration tracking and nullspace damping, then adds the projected torque
`tau_mass = -gain * (I - J.T @ Jbar.T) @ mass_gradient`, where
`Jbar = M^-1 @ J.T @ (J @ M^-1 @ J.T)^-1`.

For trajectory geometry, the runner supplies a copy of the configured wall
point and normal through `self.params['wall_geometry']`. The reference moves towards that plane along
the inward normal (currently +x): 0.5 s startup, 0.02 m/s reference speed at
the plane, smooth braking over 1 mm of reference penetration, then a 0.2 s
hold. It ends on schedule through `Controller.done`, about 3.55 s with the
current wall; the default launch `duration: 20.0` remains an upper bound.
Collision detection does not trigger this stop. Actual contact
speed and penetration can differ from the reference.

The initial mass gain is 0.01 [N m rad/kg]; the central-difference gradient
step is `1e-4` rad, requiring 15 mass evaluations per update. Set
`mass_minimization.gain: 0.0` in the same YAML for a
future comparison with straight tracking and damping alone. The objective
does not guarantee decreasing mass, joint-limit avoidance or lower contact force.

Build once for the new files, then launch inside the container:

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

cd /ros2_ws/src/fr3_student_sim
latest_straight_log=$(ls -t runs/straight_mass_minimization_*.npz | head -n 1)
ros2 run fr3_student_sim plot_run "$latest_straight_log" --no-show
```

The existing log and plots include `user_mass_kg`, `user_mass_gradient`,
`user_tau_mass`, `user_normal_speed` and `user_normal_speed_ref`.
`user_tau_mass` is requested before the total torque is clipped;
`user_qdd_command` includes tracking, damping and the mass contribution.
The controller uses no true external force. Existing monitoring and safety
stops remain active; the circular controllers, wall and launch defaults are unchanged.

Options (`name:=value` after the launch command):

| option | default | meaning |
|---|---|---|
| `controller` | `my_controller.py` | file in `students/` (or an absolute path); `none` = only the robot, no controller (it floats and drifts) |
| `wall` | `true` | the wall on/off |
| `observer` | `true` | momentum observer and detector on/off; required by both hybrid controllers |
| `observer_config` | `config/observer.yaml` | observer gain and residual thresholds |
| `duration` | `20.0` | [s] length of the run |
| `stop_at_detachment` | `false` | end the run when the fingertip first leaves the wall |
| `task_config`, `wall_config` | `config/…` | other parameter files (e.g. one per experiment) |
| `run_name` | controller name | prefix of the log file |
| `log_dir` | `fr3_student_sim/runs` | where the logs go |
| `gui` | `true` | the Gazebo window |
| `real_time_factor` | `0.5` | simulation speed: 0.5 leaves Python the time to run at 1 kHz |
| `control_rate` | `1000` | [Hz] how often `update` is called |

**Realism** (all off by default, so the simulated robot is *exactly* your model and
every signal is perfect; a real robot is not, so test with them on):

| option | example | effect |
|---|---|---|
| `joint_friction` | `true` | joint friction your model does not know (~0.3 N m) |
| `mass_error` | `0.05` | each link of the simulated robot ±5 % heavier/lighter than in your model |
| `noise_q`, `noise_dq` | `0.0001`, `0.002` | noise [rad], [rad/s] on the `q`, `dq` you receive |
| `seed` | `0` | random seed of the above |

**Batch**: list experiments in a YAML file (see `config/batch_example.yaml`); they
run one after another without the window, and a table is printed at the end.

## 2. Write your controller

```python
import numpy as np
from fr3_student_sim.controller_base import StudentController

class Controller(StudentController):

    def __init__(self, robot, params, dt):
        super().__init__(robot, params, dt)
        self.q0 = np.array(params['initial_q'])

    def update(self, t, q, dq):                      # t [s], q [rad], dq [rad/s]: (7,) arrays
        tau = 100.0 * (self.q0 - q) - 10.0 * dq
        self.log['speed'] = np.linalg.norm(dq)       # saved in the log, plotted
        return tau                                   # (7,) [N m], WITHOUT gravity
```

| inside the class | |
|---|---|
| `self.robot` | `position(q)`, `pose(q)`, `jacobian(q)` (6×7), `jacobian_pos(q)` (3×7), `velocity(q, dq)`, `jdot_dq(q, dq)`, `mass_matrix(q)`, `mass_matrix_inv(q)`, `coriolis_matrix(q, dq)`, `coriolis(q, dq)`, `gravity(q)`, `effective_mass(q, n)`, `kinetic_energy(q, dq)`. Default point: `fr3_fingertip` (where the wall pushes); other points with `frame='…'`, e.g. `'fr3_hand_tcp'`, `'fr3_hand'`, `'fr3_link4'` |
| `self.params` | the dict in `task.yaml` |
| `self.dt` | nominal period; for integrals use `t - t_previous` (a sample can be missed) |
| `self.tau_applied` | torque that really acted since your last update (your command clipped, 1 ms simulator delay included) |
| `self.bind_collision_monitor(monitor)` | startup hook called once by the runner; a no-op unless the controller overrides it |
| `self.log` | anything you put here is saved at every step |
| `self.done = True` | ends the run |

Each hybrid controller overrides the binding hook, checks that the monitor is
enabled, and registers `on_collision`. Each owns `first_collision_time` [s since
RUN start], initially `None`, and latches only the first callback timestamp.
This state belongs to the selected hybrid controller, not the generic base class.

You can split your code into several files in `students/` (e.g. `observer.py`)
and import them from your controller with `from observer import …`.

The FR3 limits are available as numpy arrays:
`from fr3_student_sim.fr3_model import TAU_MAX, DQ_MAX, Q_MIN, Q_MAX`.

**The rules of the simulated FR3**

* The simulator **adds g(q)** to your torque: never add gravity yourself. What you
  control is `M(q) ddq + C(q, dq) dq = tau + tau_ext`.
* Torques are clipped to ±[87, 87, 87, 87, 12, 12, 12] N m.
* **SAFETY STOP** (run ends, robot held) if a joint comes within 0.05 rad of its limit
  or exceeds its speed limit (2.6 rad/s for joints 1–4): your controller is unstable
  or the arm drifts.
* 7 joints, 3 for a position task: a **4-dimensional null space**, which drifts unless
  you damp or use it.
* The wall pushes with `F = k_e δ + b_e max(v_n, 0)` at the **fingertip only**. You
  cannot measure it. If the hand turns and its body enters the wall, nothing pushes
  it: the run prints a warning. Keep the tool facing the wall.

## 3. The log

`plot_run` saves `<log>.png` and prints the ground-truth facts of the first contact
(time, entry speed, peak force, peak penetration, duration). Everything else your
study needs, you compute from the log (full state at 1 kHz):

```python
from fr3_student_sim.plot_run import load_run
run = load_run('runs/xxx.npz')
run['t'], run['q'], run['dq']       # time, joint state (true values, without noise)
run['tau']                          # your torque after clipping (run['tau_cmd'] before)
run['tau_applied']                  # actuator torque acting over the sample interval,
                                    # delayed and clipped; gravity compensated, wall excluded
run['p'], run['v']                  # fingertip position, velocity
run['F_n'], run['delta']            # wall force [N], penetration [m]; run['contact'] 0/1
run['tau_wall']                     # TRUE external joint torque of the wall (J^T F): compare
                                    # it with what your controller estimates
run['hand_depth']                   # how deep the hand body is in the wall (should stay ~0)
run['residual'], run['residual_norm'] # seven signed joint residuals and their Euclidean norm
run['momentum'], run['momentum_estimate'] # measured and observed M(q) dq
run['collision_joints']             # per-joint threshold flags
run['collision_detected']           # any joint over threshold
run['detection_event'], run['detection_times'] # onset samples and timestamps
run['residual_threshold']           # configured thresholds [N m], seven values
run['user_speed']                   # everything you put in self.log
run['meta']                         # all the parameters of the run
```

With monitoring enabled, `plot_run` also saves `<log>_observer.png`: seven
residual panels with positive/negative thresholds, the residual norm and a
detection timeline. Event times are marked on the figures and printed.

`CollisionMonitor` in `collision_monitor.py` coordinates the observer and
detector independently of ROS and the tracking controller. It loads the
existing observer YAML, owns both instances, collects their samples, returns
detection-onset messages, and exports the
existing NPZ fields and observer metadata. On interruption, it exports only
the samples corresponding to saved
plant rows. The mathematical implementations remain in `momentum_observer.py`
and `collision_detector.py`.

The runner binds the selected controller to the monitor once at startup,
then calls the monitor for every RUN measurement and handles its messages
and exceptions. `set_detection_callback` delivers onset timestamps to the
registered controller callback independently of its control update period.
The controller owns any detection latch and reaction policy; the runner does
not copy that state in its RUN loop. It retains ROS communication,
simulation time and phases, measurement noise, actuator history and delay alignment, controller
execution, torque clipping, safety stops and file I/O. No separate ROS node,
launch argument or plotting change is needed.

The observer is initialized with `p_hat = M(q) dq` at the first RUN sample
and uses simulation timestamps with implicit Euler integration. The runner
supplies actuator torque after clipping, averaged over the elapsed interval
with the simulator's 1 ms transport delay. The monitor adds no delay or
gravity term and never reads the true wall force or `tau_wall`.

The full momentum equation is `p_hat_dot = tau_motor + C.T @ dq - g + r`,
`r = K_O @ (p - p_hat)`. Since the simulator adds gravity compensation, the
implemented equivalent is `p_hat_dot = tau_applied + C.T @ dq + r`.
`coriolis_matrix` supplies Pinocchio's matrix with `Mdot = C + C.T`; using
`C @ dq` in its place would be incorrect. `/joint_states.effort` is a
transmitted wrench rather than separate actuator-command telemetry and is
not used as the observer input.

Configure `gain` [1/s] and `threshold` [N m] in `config/observer.yaml`; each
accepts a scalar or seven values. Defaults are 30 and 0.1. A collision is
reported when any `abs(residual[i]) > threshold[i]`. The monitor itself does
not command torque or stop the run. The ordinary tracking variants keep their
law; the selected hybrid controller uses the first-detection notification for
RELEASE or RETREAT.
These settings target the nominal simulation; noise,
friction or model errors require checking the free-motion residual before
selecting thresholds for an experiment.

## 4. Your project

Start with the examples: `zero_torque.py` (with `wall:=false`, the arm must stay
still), then `joint_pd.py` and `cartesian_pd.py`; change their gains, turn the wall on.

Then bring your planar-3R study to the FR3. You decide and justify:

* the task controller (which dynamics compensation; position only, or also orientation?)
  and what to do with the null space;
* how to detect the contact without a force sensor, with a threshold that also works
  with the realism options on;
* how to compute and use the effective mass and its variation with q;
* which experiments to run and which quantities to compare to answer your question on
  a 7-joint robot.

## Troubleshooting

* **No window**: run `xhost +local:docker` on the host, or use `gui:=false`.
* **Nothing happens / the run never starts**: Gazebo's own output is not shown in
  the terminal; its file is printed at the start (`Gazebo output (if something goes
  wrong): /tmp/fr3_student_…/gazebo.log`). Look there.
* **`Ctrl+C` while the simulation is still starting** may print one
  `[ERROR] [spawner-…]: process has died`: harmless, the start-up was interrupted.
* **"X% of the joint states were missed"**: your controller is too slow; use
  `real_time_factor:=0.25` or `control_rate:=500`.
* **Exception in Controller.update**: the traceback is printed, the robot stops safely,
  the log is saved.
* **The arm moves strangely right at the start**: your first torque has a jump; start
  your references and gains smoothly.
* **Two simulations at once** (e.g. two terminals): give each its own
  `partition:=name` and a different `export ROS_DOMAIN_ID=<n>`.
* **Is the package itself OK?** Run its checks (1 s, no Gazebo):
  `cd /ros2_ws/src/fr3_student_sim && python3 -m pytest test/`
