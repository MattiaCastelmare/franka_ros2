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
| The wall | `config/wall.yaml`: vertical plane at x = 0.6397 m, `k_e` = 20000 N/m, `b_e` = 15 N s/m | edit |
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
outward normal (-1, 0, 0), at x = 0.6397 m: 0.1 mm closer to the robot than
the previous position at x = 0.6398 m. The desired circle reaches x = 0.64 m,
giving a nominal 0.3 mm overlap at its farthest point.

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

The Gazebo window opens, the run lasts `duration` seconds, then everything closes
and the log is saved in `fr3_student_sim/runs/` (whatever folder you launch from). `Ctrl+C`
stops a run early (the log is still saved). Edits to your controller work at the
next launch (no rebuild).

The circle experiment has two separately selectable controllers:

| Variant | Controller in `students/circle_tracking/` | Task configuration |
|---|---|---|
| Minimal position tracking | `controller.py` | `config/circle_tracking.yaml` |
| Position tracking with nullspace velocity damping | `controller_nullspace_damping.py` | `config/circle_tracking_nullspace_damping.yaml` |

The damping variant reuses the minimal controller and adds only
`qdd_null = -kd_nullspace * (I - J_pinv @ J) @ dq`, contributing
`M(q) @ qdd_null` to its torque. The trajectory, Cartesian gains and observer
are the same. Its configuration starts with `kd_nullspace: 4.0` [1/s]; zero
recovers the minimal command. It logs `dq_null` and `qdd_null` in addition to
the existing signals, with `qdd_command` containing the total requested
acceleration. Damping does not impose a desired posture or enforce joint limits.

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
```

The run names give the log files distinct prefixes. Each log already records
the selected controller path and task parameters. For a comparison, keep
wall/observer settings, initial pose, trajectory and Cartesian gains equal
and change only the controller variant. New control laws get separate files;
gain changes belong in configuration files.

Options (`name:=value` after the launch command):

| option | default | meaning |
|---|---|---|
| `controller` | `my_controller.py` | file in `students/` (or an absolute path); `none` = only the robot, no controller (it floats and drifts) |
| `wall` | `true` | the wall on/off |
| `observer` | `true` | momentum observer and collision detector on/off; monitoring only |
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
| `self.log` | anything you put here is saved at every step |
| `self.done = True` | ends the run |

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

The observer and detector live in `momentum_observer.py` and
`collision_detector.py`, separately from the controller. The observer is
initialized with `p_hat = M(q) dq` at the first RUN sample and uses simulation
timestamps with implicit Euler integration. Its input is the actuator command
after clipping, averaged over the elapsed interval with the simulator's 1 ms
transport delay. It never reads the true wall force or `tau_wall`.

The full momentum equation is `p_hat_dot = tau_motor + C.T @ dq - g + r`,
`r = K_O @ (p - p_hat)`. Since the simulator adds gravity compensation, the
implemented equivalent is `p_hat_dot = tau_applied + C.T @ dq + r`.
`coriolis_matrix` supplies Pinocchio's matrix with `Mdot = C + C.T`; using
`C @ dq` in its place would be incorrect. `/joint_states.effort` is a
transmitted wrench rather than separate actuator-command telemetry and is
not used as the observer input.

Configure `gain` [1/s] and `threshold` [N m] in `config/observer.yaml`; each
accepts a scalar or seven values. Defaults are 30 and 0.1. A collision is
reported when any `abs(residual[i]) > threshold[i]`; it does not stop or
modify the controller. These settings target the nominal simulation; noise,
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
