# Franka ROS 2 Research Framework

[![CI](https://github.com/frankarobotics/franka_ros2/actions/workflows/ci.yml/badge.svg)](https://github.com/frankarobotics/franka_ros2/actions/workflows/ci.yml)

A ROS 2 Humble research framework for Franka robots, built on the official
[`frankarobotics/franka_ros2`](https://github.com/frankarobotics/franka_ros2) project.
This fork adds CBF safety filters, real-time controllers, Gazebo simulation, and
MuJoCo-based Safe-RL training with ONNX deployment to the real FR3.

**Start with the [Docker installation](#docker-container-installation)** for a
consistent environment containing ROS 2, MoveIt2, Gazebo, perception tools, and the
training stack. ROS 2 does not need to be installed on the host.

## Table of Contents

- [About](#about)
- [Research Extensions in This Fork](#research-extensions-in-this-fork)
- [Caution](#caution)
- [Installation](#installation)
  - [Docker Container Installation](#docker-container-installation)
    - [Prerequisites](#docker-prerequisites)
    - [Clone and configure](#clone-and-configure)
    - [Option A: Docker Compose](#option-a-docker-compose)
    - [Option B: VS Code Dev Containers](#option-b-vs-code-dev-containers)
    - [GUI access: RViz and Gazebo](#gui-access-rviz-and-gazebo)
    - [Build the workspace](#build-the-workspace)
    - [Daily workflow](#daily-workflow)
    - [Data persistence](#data-persistence)
    - [Shared workstations](#shared-workstations)
  - [Local Machine Installation](#local-machine-installation)
- [Test the Setup](#test-the-setup)
- [franka_experiments](#franka_experiments)
- [franka_rt_controllers](#franka_rt_controllers)
- [franka_simulation](#franka_simulation)
- [franka_rl](#franka_rl)
- [Documentation map](#documentation-map)
- [Troubleshooting](#troubleshooting)
  - [libfranka: UDP receive: Timeout error](#libfranka-udp-receive-timeout-error)
  - [Build permission errors](#build-permission-errors)
  - [GUI windows never appear](#gui-windows-never-appear)
- [Contributing](#contributing)
- [License](#license)
- [Contact](#contact)

## About

The upstream `franka_ros2` project provides the ROS 2 interface to Franka research
robots through **libfranka** and **ros2_control**. This fork extends that interface
for experiments in collision avoidance, perception-driven control, human–robot
interaction, and safe reinforcement learning.

Docker is the recommended development environment because it keeps ROS, system
libraries, and Python dependencies together. The Dockerfile also installs the
`franka_rl` training stack (PyTorch, MuJoCo, Gymnasium, Stable-Baselines3, ONNX
Runtime, and OSQP) inside the image.

## Research Extensions in This Fork

| Package | Purpose | Documentation |
|---|---|---|
| `franka_experiments` | CBF torque and velocity stacks, depth-camera distance estimation, experiment logging, and hand–eye calibration | [Package guide](./franka_experiments/README.md) |
| `franka_rt_controllers` | C++ `ros2_control` plugins for torque and velocity execution in the 1 kHz loop | [Controller overview](#franka_rt_controllers) |
| `franka_simulation` | Gazebo Fortress, RViz2, and MoveIt2 with position, velocity, acceleration, torque, and avoidance pipelines | [Simulation guide](./franka_simulation/README.md) |
| `franka_rl` | Standalone MuJoCo Safe-RL training behind a CBF filter, with ONNX export for deployment | [Training guide](./franka_rl/README.md) |

### Branch structure

- **`humble-mattia`** is the recommended branch for this fork. It contains the
  research packages, simulation tools, and Docker configuration.
- **`humble`** tracks the upstream Humble branch. It is used for synchronization
  with Franka Robotics rather than custom development; compatibility adjustments
  may temporarily differ from upstream.

## Caution

This package is in rapid development. Users should expect breaking changes and are encouraged to report any bugs via [GitHub Issues page](https://github.com/frankarobotics/franka_ros2/issues).

## Installation

Choose **Docker** for the complete development environment, or use the
[local installation](#local-machine-installation) to manage dependencies directly
on the host. The commands below target Linux; `franka_ros2` is not officially
supported on Windows.

### Docker Container Installation

The [Dockerfile](./Dockerfile) installs dependencies; it does **not** compile the
ROS 2 workspace. [Docker Compose](./docker-compose.yml) mounts your checkout at
`/ros2_ws/src`, and you build the workspace inside the running container.

#### Docker prerequisites

- **Linux x86-64** host. The image uses ROS 2 Humble on Ubuntu 22.04 and an
  `amd64` Pinocchio package repository.
- **Docker Engine** and **Docker Compose 2.30.0 or newer**, with permission to run
  Docker commands as your account. See the
  [Engine installation guide](https://docs.docker.com/engine/install/) and
  [Compose GPU requirements](https://docs.docker.com/reference/compose-file/services/#gpus).
- **NVIDIA GPU, host driver, and NVIDIA Container Toolkit** for the default
  configuration, which requests `gpus: all`. Follow NVIDIA's
  [installation and Docker configuration guide](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html).
- **A graphical session with X11 or XWayland** for RViz and Gazebo windows. Install
  `xauth` on the host for the [GUI access procedure](#gui-access-rviz-and-gazebo).
  Headless workflows can skip that procedure.

**Host — check Docker and the NVIDIA driver:**

```bash
docker info
docker compose version
nvidia-smi
```

**Without an NVIDIA GPU:** before starting either workflow, remove `gpus: all`
and the `NVIDIA_VISIBLE_DEVICES` / `NVIDIA_DRIVER_CAPABILITIES` entries from your
local `docker-compose.yml`. NVIDIA GPU access and CUDA training will be unavailable;
GUI rendering will depend on the host graphics setup.

For **real robot control**, use Docker Engine on Linux and configure the host's
real-time kernel and robot network according to the
[Franka documentation](https://frankarobotics.github.io/docs/), including the
[real-time kernel setup](https://frankarobotics.github.io/docs/doc/libfranka/docs/real_time_kernel.html).
The container shares the host kernel; building the image does not configure it.
Simulation and fake hardware do not require a real-time kernel.

#### Clone and configure

**Host — clone into a directory owned by your account:**

```bash
git clone -b humble-mattia https://github.com/MattiaCastelmare/franka_ros2.git
cd franka_ros2

export COMPOSE_PROJECT_NAME="franka_${USER}"
export FRANKA_CONTAINER="franka_${USER}"
export USER_UID="$(id -u)"
export USER_GID="$(id -g)"
```

Run the four exports in each host terminal used for this project. If you use this
checkout regularly, you can add them to `~/.bashrc`. Keep the same values when
building, starting, or stopping the environment.

`USER_UID` and `USER_GID` set the numeric identity of the image's `user` account at
build time, so it can write to your mounted checkout. `COMPOSE_PROJECT_NAME`
separates each account's Compose resources and image tag; `FRANKA_CONTAINER`
separates the container names. Changing only the project name does not change an
explicit container name.

Compose also reads UID/GID values from `.env`, but exported shell variables take
precedence. The VS Code workflow rewrites that file, so keep the four exports as
your common configuration for both workflows.

Choose **one** of the following ways to start the environment.

#### Option A: Docker Compose

**Host — repository root:**

```bash
docker compose config --quiet
docker compose up -d --build
docker compose logs -f franka_ros2
```

The first image build installs the ROS, perception, and training dependencies and
can take a while. Later builds reuse unchanged layers. At startup,
[the entrypoint](./franka_entrypoint.sh) imports missing `libfranka` and
`franka_description` into the mounted checkout using the versions in
[`franka.repos`](./franka.repos). Wait for the import to finish; press `Ctrl+C`
to leave the log viewer while keeping the container running. Existing checkouts
are skipped rather than updated to newer revisions.

Open a shell:

```bash
# Host — repository root
docker compose exec franka_ros2 bash
```

The shell starts as `user` in `/ros2_ws`. Continue with
[GUI access](#gui-access-rviz-and-gazebo) if needed, then
[build the workspace](#build-the-workspace).

#### Option B: VS Code Dev Containers

Install VS Code and its **Dev Containers** extension; see the
[official tutorial](https://code.visualstudio.com/docs/devcontainers/tutorial).

1. Complete [Clone and configure](#clone-and-configure), then launch `code .` from
   that host terminal so VS Code inherits the exported variables. If VS Code is
   already running, close its windows before launching it from the configured terminal.
2. Run **Dev Containers: Reopen in Container** from the Command Palette. VS Code
   uses the same Compose service and builds the image as needed.
3. Open an integrated terminal. The workspace folder is `/ros2_ws`, with the
   checkout under `src/`. Allow the startup dependency import to finish before building.
4. Complete [GUI access](#gui-access-rviz-and-gazebo) if needed, then follow the
   shared [workspace build](#build-the-workspace) instructions.

The configuration's `initializeCommand` rewrites the checkout's `.env` with your
host UID/GID before container creation. It does not set `COMPOSE_PROJECT_NAME` or
`FRANKA_CONTAINER`; those must still come from the configured host environment.

#### GUI access: RViz and Gazebo

Run this after the container starts and before launching a GUI. Use a host
terminal in **your active graphical session**, with the same Compose variables
as above. The X11 socket is already mounted by Compose; the commands below copy
your session's authorization cookie into the container.

**Host — repository root:**

```bash
# If xauth is missing on an Ubuntu/Debian host:
# sudo apt install xauth

echo "$DISPLAY"
franka_xauth_file=$(mktemp)
xauth nlist "$DISPLAY" | sed 's/^..../ffff/' | xauth -f "$franka_xauth_file" nmerge -
docker compose exec -T franka_ros2 bash -c \
  'umask 077; cat > /tmp/franka.xauth' < "$franka_xauth_file"
rm -f "$franka_xauth_file"
```

`DISPLAY` must be nonempty, and `xauth nlist "$DISPLAY"` must return a cookie.
If either is missing, see [GUI troubleshooting](#gui-windows-never-appear).

**Container — in each terminal that will launch GUI applications**, including
VS Code integrated terminals:

```bash
export XAUTHORITY=/tmp/franka.xauth
```

Repeat the cookie transfer after recreating the container or logging into a new
graphical session. Avoid `xhost +local:`, which grants display access to all local
accounts. If the container has an old `DISPLAY`, use the per-shell override in
[GUI troubleshooting](#gui-windows-never-appear).

#### Build the workspace

Both Docker workflows use the same build procedure. Interactive Bash terminals
source `/opt/ros/humble/setup.bash` automatically; source the workspace overlay
manually after each build and in each new terminal.

**Container — `/ros2_ws`:**

```bash
cd /ros2_ws
colcon build --symlink-install --executor parallel --parallel-workers 4 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

Four package workers are a starting point, not a memory guarantee: CMake can also
compile files in parallel within each package. Compose currently limits the
container to 20 GB of RAM. If compilation runs out of memory, reduce package
workers and limit compiler jobs for the default Makefile generator, for example:

```bash
# Container — /ros2_ws
MAKEFLAGS="-j2 -l2" colcon build --symlink-install --parallel-workers 2 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
```

The startup import is automatic, so a second manual `vcs import` is normally
unnecessary. If it failed or was interrupted, retry it before building:

```bash
# Container — /ros2_ws
vcs import src < src/franka.repos --recursive --skip-existing
```

`--recursive` includes libfranka's own submodules. Imported repositories are
written into your checkout on the host. MoveIt2 and `pymoveit2` are installed in
the image, so `extras.repos` is not required for this Docker workflow.

**Verify the build** without connecting a robot:

```bash
# Container — /ros2_ws, after sourcing install/setup.bash
ros2 pkg prefix franka_bringup
ros2 pkg prefix franka_simulation
```

Both commands should print paths under `/ros2_ws/install`. Then run a
[fake-hardware example](#test-the-setup) to check the launch and GUI setup.

**Optional: rebuild one package** after a successful full build:

```bash
# Container — /ros2_ws, with the existing overlay sourced
colcon build --packages-select franka_simulation --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

`--packages-select` does not build the selected package's dependencies. For a
partial build that includes workspace dependencies, use
`--packages-up-to franka_simulation` instead.

#### Daily workflow

**Host — repository root, with the same four exports:**

```bash
docker compose start
docker compose exec franka_ros2 bash
```

**Container — each new terminal:**

```bash
source /ros2_ws/install/setup.bash
# For GUI applications, after transferring the session cookie:
export XAUTHORITY=/tmp/franka.xauth
```

When finished, run `exit` to leave the shell. To stop the environment while
preserving its build, run `docker compose stop` **on the host**. Resume with
`docker compose start`.

Source edits are visible immediately through the bind mount; rebuild the affected
packages when required. Rebuild the image with `docker compose up -d --build`
when its dependencies or build arguments change. If Compose recreates the
container, rebuild the ROS 2 workspace too.

#### Data persistence

| Container path | Storage | Survives `stop` / `start` | Survives container removal or recreation |
|---|---|---|---|
| `/ros2_ws/src` | Your host checkout (bind mount) | Yes | Yes |
| `/ros2_ws/build`, `/ros2_ws/install`, `/ros2_ws/log` | Container filesystem | Yes | No |
| Other files outside mounted paths | Container filesystem | Yes | No |

Save experiment outputs, bags, and trained models under the mounted checkout
(e.g. `/ros2_ws/src/franka_rl/models`) or copy them to the host before removing
the container. Docker image dependencies remain available in the built image;
packages installed manually in a running container do not survive its removal.

Use `docker compose down` **on the host** only when you intend to remove the
container. It leaves the checkout intact but deletes the workspace build and
any other files stored only in the container. The same loss applies to
`docker compose up --force-recreate` and Dev Containers rebuilds that replace
the container.

#### Shared workstations

For accounts using the same Docker daemon, use **one checkout per account**, with
matching UID/GID and distinct Compose project and container names. The exports in
[Clone and configure](#clone-and-configure) provide those names automatically.

Separate project names prevent image-tag collisions, but do not change ownership
of mounted files. Use your own checkout rather than another account's working
tree. Check the identities with:

```bash
# Host — repository root
id
stat -c 'Checkout owner UID:GID: %u:%g (%n)' .
docker compose exec franka_ros2 id
docker ps -a --format 'table {{.Names}}\t{{.Image}}\t{{.Status}}'
```

The container user's numeric UID should match the checkout owner's UID. Avoid
removing images or containers belonging to other accounts. Separate Compose
projects still share the host network: coordinate simultaneous robot access and
ROS 2 discovery settings with other users.

### Local Machine Installation

Use this route if you want to manage ROS and all research dependencies on the
host. The Dockerfile is the dependency reference for the complete environment;
`rosdep` alone does not reproduce its Python perception and training stack.

1. Install **ROS 2 Humble** using the
   [Ubuntu installation guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html).
   Choose `ros-humble-desktop` for RViz and desktop tools, or `ros-humble-ros-base`
   for a minimal installation; install simulation tools separately as needed.
   Install the development tools and source ROS in your terminal:

   ```bash
   # Host
   sudo apt install ros-dev-tools
   source /opt/ros/humble/setup.bash
   ```

2. Create a workspace, clone this fork, and import its pinned source dependencies:

   ```bash
   # Host
   mkdir -p ~/franka_ros2_ws
   cd ~/franka_ros2_ws
   git clone -b humble-mattia https://github.com/MattiaCastelmare/franka_ros2.git src
   vcs import src < src/franka.repos --recursive --skip-existing
   ```

3. Resolve ROS dependencies. If `rosdep` has not been initialized on the machine,
   run `sudo rosdep init` once before these commands:

   ```bash
   # Host — ~/franka_ros2_ws
   rosdep update
   rosdep install --from-paths src --ignore-src --rosdistro humble -y
   ```

   Install the additional system and Python dependencies required by your chosen
   research packages, using the [Dockerfile](./Dockerfile) as a reference. Preserve
   its version constraints, especially for OSQP, qpsolvers, and the perception stack.

4. Build and source the workspace:

   ```bash
   # Host — ~/franka_ros2_ws
   colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
   source install/setup.bash
   ```

   Source `/opt/ros/humble/setup.bash` and your workspace's `install/setup.bash`
   in each new terminal before running ROS commands.

## Test the Setup

Run these commands in a terminal with the built workspace sourced. For Docker,
use a container terminal and complete [GUI access](#gui-access-rviz-and-gazebo)
before examples that open RViz or Gazebo.

### Run package tests

```bash
# Workspace root: /ros2_ws in Docker, or ~/franka_ros2_ws on the host
colcon test
colcon test-result --verbose
```

See the package test guides for test-specific dependencies and hardware requirements.

### Run a sample ROS 2 application

To verify that your setup works correctly without a robot, you can run the following command to use dummy hardware:

```bash
ros2 launch franka_fr3_moveit_config moveit.launch.py robot_ip:=dont-care use_fake_hardware:=true
```
You can use the arguments `load_gripper` to activate or deactivate the end-effector and `ee_id` to set which end-effector you want to use. By default, the Franka Hand is activated.

If you want to run this example with namespaces, you would need to use the argument `namespace` and manually write your namespace in `moveit.rviz` under `Move Group Namespace`.

### Run the CBF torque stack (franka_experiments)

The main research pipeline of this fork. It needs no real robot to start:

```bash
ros2 launch franka_experiments torque_control_stack.launch.py \
  use_fake_hardware:=true enable_camera:=false start_real_time_distance:=false
```

### Run the simulation environment (franka_simulation)

To test the Gazebo simulation with MoveIt2 and the avoidance pipeline:

```bash
ros2 launch franka_simulation move_group.launch.py
```

This starts Gazebo (Ignition), the robot state publisher, MoveIt2 `move_group`, RViz2, the online avoidance controller, the velocity blender, and the motion server. See the [franka_simulation](#franka_simulation) section for full details.

### Run a ROS 2 example controller

To run any example controller, make sure to add your desired configuration in `franka.config.yaml` and run:

```bash
ros2 launch franka_bringup example.launch.py controller_name:=your_desired_controller
```
You can select one of the controllers from `controllers.yaml`.

### Run Gazebo examples with ROS 2

If you want to use Gazebo to run your code, you can find some examples here: [franka_gazebo](./franka_gazebo/README.md)

---

## franka_experiments

The `franka_experiments` package is the **main research package** of this fork. It is a Python (`ament_python`) package implementing two end-to-end control stacks for the FR3, both built around **Control Barrier Function (CBF) safety filters** that project a nominal command onto the safe set at run time.

Full node-by-node reference: [`franka_experiments/README.md`](./franka_experiments/README.md).

### Torque stack (acceleration space) — `torque_control_stack.launch.py`

```
[Camera]  RealSense driver
    │
    ▼
real_time_distance  ──►  /cbf/per_link_distances
                                  │
pentagon_qddot_commander          │
  (or rl_policy_commander,        │
   motion_source:=rl)             │
    │                             │
    ▼                             ▼
/NS_1/qddot_nom  ──►  cbf_safety_filter  ──►  /NS_1/qddot_safe
                                                      │
                                               qddot_to_torque   τ = M(q)q̈ + C(q,q̇)q̇
                                                      │
                                                      ▼
                                              /NS_1/torque_cmd
                                                      │
                                            rt_torque_controller  ──►  HW  (+ firmware gravity)
```

The motion generator is selected with `motion_source`, and **exactly one** may publish `qddot_nom`:

| `motion_source` | Node | Notes |
|---|---|---|
| `pentagon` (default) | `pentagon_qddot_commander` | Analytic/MoveIt Cartesian path with avoidance-first shaping. Requires `move_group`. |
| `rl` | `rl_policy_commander` | Replays the ONNX Safe-RL policy trained in [`franka_rl`](#franka_rl). Pass `start_move_group:=false`. |

```bash
# Full stack on the real robot
ros2 launch franka_experiments torque_control_stack.launch.py robot_ip:=192.168.2.10

# Fake hardware, no camera
ros2 launch franka_experiments torque_control_stack.launch.py \
    use_fake_hardware:=true enable_camera:=false start_real_time_distance:=false

# Safe-RL policy, derated to 30 % authority for a first real run
ros2 launch franka_experiments torque_control_stack.launch.py \
    motion_source:=rl start_move_group:=false rl_action_scale:=0.3
```

### Velocity stack (kinematic level) — `velocity_cbf_control_stack.launch.py`

```
real_time_distance  ──►  /human_robot/multi_distance
                                  │
ee_pentagon_velocity_commander    │
    │                             ▼
/NS_1/tracking_qdot  ──►  cbf_velocity_filter  ──►  /NS_1/qdot_cmd
                                                            │
                                          rt_velocity_executor_controller  ──►  HW
```

Two-phase design: `bypass_cbf:=true` (default) passes the trajectory straight through so it can be verified without a camera; `bypass_cbf:=false` enables the full CBF QP and starts the camera and distance estimator.

### Nodes

| Node | Stack | Description |
|---|---|---|
| `pentagon_qddot_commander` | Torque (accel) | MoveIt-based Cartesian pentagon reference → `qddot_nom`, with Cartesian tracking correction |
| `rl_policy_commander` | Torque (accel) | Sim-to-real Safe-RL policy: rebuilds the 24-dim training observation and runs the exported ONNX actor with `onnxruntime` |
| `cbf_safety_filter` | Torque (accel) | HOCBF QP: min ‖q̈ − q̈_nom‖² subject to the barrier, joint, velocity and workspace rows |
| `qddot_to_torque` | Torque (accel) | Dynamics converter τ = M(q)·q̈ + C(q,q̇)·q̇ via Pinocchio |
| `pentagon_torque_commander` | Torque (OSCBF) | 6D Cartesian PD + damped-LS Jacobian torque commander |
| `cbf_oscbf_filter` | Torque (OSCBF) | Operational Space CBF (Morton & Pavone 2025): torque-level QP with task-space and null-space cost terms |
| `ee_pentagon_velocity_commander` | Velocity | Pentagon EE trajectory in velocity space (also `ee_circle_…`, `ee_random_waypoints_…`) |
| `cbf_velocity_filter` | Velocity | Velocity-level CBF QP; `bypass_cbf` for Phase-1 pass-through |
| `real_time_distance` | Shared | Depth-camera human–robot distance estimator (Flacco depth-space method) → `MultiLinkDistance` |
| `experiment_logger` | Shared | CSV + plot logger for joint states, torques and CBF values |
| `capsule_overlay_node` | Shared | RViz capsule geometry for the robot body |
| `handeye_calibration_node` | Shared | AprilTag hand–eye calibration (manual and automatic acquisition) |

### Launch files

| Launch file | Purpose |
|---|---|
| `torque_control_stack.launch.py` | Acceleration-space CBF pipeline (the canonical torque stack) |
| `velocity_cbf_control_stack.launch.py` | Velocity-space CBF pipeline, two-phase |
| `thales.launch.py` | Production velocity pipeline + rosbag recording co-located with the CSV logs |
| `minimal.launch.py` | Lightweight bringup for debugging: driver + RT velocity executor, no RViz |
| `handeye_calibration_bringup.launch.py` | Full hand–eye calibration pipeline (driver + AprilTag + calibration node) |

**Key launch arguments** (defaults in `franka_experiments/config/launch_defaults.yaml`, editable without touching Python):

| Argument | Default | Description |
|---|---|---|
| `robot_ip` | `192.168.1.10` | IP address of the real robot |
| `use_fake_hardware` | `false` | Run without a physical robot |
| `namespace` | `""` | ROS 2 namespace for all topics |
| `enable_camera` | `true` | Start the RealSense driver |
| `start_real_time_distance` | `true` | Start the distance estimator |
| `control_spawner_delay_s` | `10.0` | Seconds before the RT controller spawner |
| `motion_source` | `pentagon` | `pentagon` or `rl` (torque stack) |
| `rl_action_scale` | `1.0` | Derate for the RL policy, in (0, 1] |
| `lpf_alpha` | `0.3` | Torque low-pass coefficient in `rt_torque_controller` |
| `qdot_max` | `1.5` | Joint-velocity clamp in the velocity executor |

### Configuration

| File | Purpose |
|---|---|
| `fr3_control.yaml` | CBF gains and tuning for both stacks — mirrored by `franka_rl/config.yaml` and checked by the tests |
| `oscbf_params.yaml` | OSCBF QP weights and CBF gains |
| `fr3_complete.yaml` | Robot geometry (control points, meshes, frames) for `real_time_distance` |
| `fr3_distance.yaml` | Per-link distance thresholds |
| `launch_defaults.yaml` | Defaults for every launch argument above |
| `camera_*.yaml` / `depth_intrinsics.yaml` | Camera intrinsics and hand–eye extrinsics |

### Debug commands

```bash
# Verify the controller is active
ros2 control list_controllers

# List claimed command interfaces
ros2 control list_hardware_interfaces
```

---

## franka_rt_controllers

The `franka_rt_controllers` package provides the **real-time C++ `ros2_control` plugins** that execute the Python stacks' commands inside the 1 kHz loop. The Python nodes publish at 100–200 Hz; these controllers remove the resulting sample-and-hold jitter without ever allocating, locking, or logging in the RT path.

| Controller | Interfaces | Description |
|---|---|---|
| `rt_torque_controller` | `fr3_joint{1..7}/effort` | Reads 7 user torques (**without** gravity — the Franka firmware adds it), applies an optional low-pass filter (`lpf_alpha`), clips to per-joint limits, writes at 1 kHz |
| `rt_velocity_executor_controller` | `fr3_joint{1..7}/velocity` | Pure executor: reads 7 joint velocities from one non-RT topic, optional linear interpolation between samples, rate limiting (`max_accel`), smooth timeout ramp, final clamp (`qdot_max`). **No blending logic** |
| `cbf_torque_controller` | `fr3_joint{1..7}/effort` | Inverse dynamics from `qddot_safe` computed inside the RT loop (used rarely; the Python `qddot_to_torque` path is the default) |

All three use `RealtimeBuffer` for lock-free transfer from the non-RT subscriber to `update()`:

```
  Python nodes ──topic──▶ RealtimeBuffer ──readFromRT──▶ update() @ 1 kHz
                                                            │
                        interpolate → rate-limit → clamp → command_interfaces
```

### Launch files

```bash
# Robot driver + RT velocity executor controller
ros2 launch franka_rt_controllers rt_velocity_blender.launch.py

# Robot driver + RT torque controller
ros2 launch franka_rt_controllers rt_torque.launch.py
```

> **Note:** these controllers claim `fr3_joint{1..7}/velocity` or `/effort`. No other controller can claim the same interfaces at the same time — check with `ros2 control list_controllers` and deactivate any conflicting controller first.
>
> In normal use you do not launch them directly: the `franka_experiments` stacks spawn the right one for you.

### When to use which controller

| Scenario | Controller | Package |
|---|---|---|
| Real hardware, CBF torque stack | `rt_torque_controller` | `franka_rt_controllers` |
| Real hardware, CBF velocity stack | `rt_velocity_executor_controller` | `franka_rt_controllers` |
| Gazebo simulation | `fr3_arm_controller` / `fr3_velocity_controller` / `fr3_effort_controller` | `franka_simulation` |

---

## franka_simulation

The `franka_simulation` package provides a **Gazebo (Ignition) + RViz2 + MoveIt2 simulation** of the FR3, with four selectable control pipelines and an online collision avoidance pipeline. It is designed for developing and testing algorithms before deploying to real hardware.

Full pipeline-by-pipeline reference: [`franka_simulation/README.md`](./franka_simulation/README.md).

### Launch files

| Launch file | Pipeline | Controller |
|---|---|---|
| `sim_position.launch.py` | Position | `fr3_arm_controller` (joint trajectory) |
| `sim_velocity.launch.py` | Velocity | `fr3_velocity_controller` |
| `sim_acceleration.launch.py` | Acceleration | `fr3_velocity_controller` + `sim_acceleration_bridge` |
| `sim_torque.launch.py` | Torque | `fr3_effort_controller` (note the Gazebo gravity semantics documented in the package README) |
| `move_group.launch.py` | CBF / avoidance | MoveIt2 + avoidance controller + velocity blender + obstacle synchroniser |

```bash
ros2 launch franka_simulation move_group.launch.py
ros2 launch franka_simulation move_group.launch.py spawn_obstacles:=false enable_camera:=false
```

### Nodes

| Node | Description |
|---|---|
| `franka_motion_server` | MoveIt2-based motion planning server exposing the `MoveToPose`, `MoveToJoint` and `PlanGlobalPath` actions; publishes planned `JointTrajectory` messages |
| `franka_motion_client` | Python client library (`move_to_pose()`, `move_to_joint()`, `get_current_pose()`) |
| `online_avoidance_controller` | Capsule-based minimum distances via Pinocchio FK; publishes closest-constraint Jacobians, distances and RViz markers |
| `velocity_control_blender` | CBF-QP blending of tracking and avoidance velocities, with ḋ-constraint enforcement, risk-scaled filtering and trajectory rejoin |
| `obstacle_synchronizer` | Keeps obstacles in sync across URDF/Xacro, the MoveIt planning scene and the avoidance controller |
| `sim_acceleration_bridge` | Integrates commanded accelerations into velocity commands for the acceleration pipeline |
| `cartesian_*_mapper` | Default trajectory generators for the velocity / acceleration / torque pipelines (plus circle, figure-8 and sine variants) |
| `image_publisher`, `human_pose_node` | RealSense QoS adapter and MediaPipe human-pose overlay |

### Custom action interfaces

| Action | Description |
|---|---|
| `MoveToPose.action` | Move the end-effector to a target Cartesian pose |
| `MoveToJoint.action` | Move to a target joint configuration |
| `PlanGlobalPath.action` | Plan a global path and return the planned trajectory |

---

## franka_rl

`franka_rl` is a **standalone MuJoCo training module with no ROS 2 dependency**. It trains a **Safe Reinforcement Learning** policy (SAC, Stable-Baselines3) shielded by the *same* acceleration-level CBF filter that runs on the real robot in `franka_experiments/nodes/cbf_safety_filter.py` — safe exploration in simulation, safe execution on hardware.

Full guide: [`franka_rl/README.md`](./franka_rl/README.md) · architecture, validation status, gotchas and backlog: [`franka_sim_to_real_implementation_status.md`](./franka_sim_to_real_implementation_status.md).

```
franka_rl/
├── assets/franka_fr3/    # MuJoCo FR3 model incl. the Franka Hand (transforms from the real URDF)
├── envs/
│   ├── franka_cbf_env.py # Gymnasium env FrankaCBF-v0 (reach + moving obstacle)
│   └── cbf_filter.py     # AccelCBFFilter — mirrors the robot's HOCBF QP
├── scripts/              # evaluate_policy, compare_checkpoints, validate_cbf, validate_actuation
├── config.yaml           # mirrors `params:` in franka_experiments/config/fr3_control.yaml
├── train.py              # SAC training + checkpointing (step- and episode-spaced)
└── export_onnx.py        # actor → ONNX, validated against the SB3 policy
```

Run these commands **on the host, from the repository root**, with the same Compose
variables used during installation. Each command runs the training stack inside the container:

```bash
# Train
docker compose exec franka_ros2 bash -lc 'cd /ros2_ws/src && MUJOCO_GL=egl \
  python3 -m franka_rl.train --exp-name sac_v3 --total-timesteps 2000000'

# Score every checkpoint against the zero-action and random baselines
docker compose exec franka_ros2 bash -lc 'cd /ros2_ws/src && MUJOCO_GL=egl \
  python3 -m franka_rl.scripts.compare_checkpoints --model-dir franka_rl/models/sac_v3'

# Export a checkpoint for the robot
docker compose exec franka_ros2 bash -lc 'cd /ros2_ws/src && \
  python3 -m franka_rl.export_onnx --model franka_rl/models/sac_v3/best_model.zip'
```

Then deploy the exported policy onto the torque stack:

```bash
ros2 launch franka_experiments torque_control_stack.launch.py \
    motion_source:=rl start_move_group:=false \
    rl_onnx_model:=/path/to/best_model.onnx rl_action_scale:=0.3
```

> **Training artefacts are not versioned.** `franka_rl/models/`, `runs/`, `*.zip` and `*.onnx` are git-ignored: they are regenerated by `train.py` / `export_onnx.py`. To move a policy between machines, copy the `.onnx` **together with the `config.yaml` frozen next to it** — the deployment node falls back to the repository default otherwise, which may not be the configuration the policy was trained under.
>
> Sim and robot configurations are kept in sync by `franka_experiments/test/test_rl_policy.py`, which fails if a CBF gain drifts between `franka_rl/config.yaml` and `fr3_control.yaml`.

---

## Documentation map

| Document | Contents |
|---|---|
| [`franka_experiments/README.md`](./franka_experiments/README.md) | Node-by-node reference, topics, launch sequencing, configuration keys |
| [`franka_experiments/test/README.md`](./franka_experiments/test/README.md) | Test suite: what is checked and how to run it |
| [`franka_simulation/README.md`](./franka_simulation/README.md) | The four simulation pipelines, controllers, kinematics library |
| [`franka_rl/README.md`](./franka_rl/README.md) | Training, evaluation, the MuJoCo viewer, sim↔robot sync, gotchas |
| [`franka_sim_to_real_implementation_status.md`](./franka_sim_to_real_implementation_status.md) | Safe-RL + CBF sim-to-real: the three-step specification, what is built, how it was validated, gotchas and the prioritised backlog |
| [`franka_experiments/LEGACY.md`](./franka_experiments/LEGACY.md) | Inventory of everything unreachable from the torque stack, with the deletion units |

---

## Troubleshooting

### `libfranka: UDP receive: Timeout error`

Use Docker Engine on Linux for real robot control. Check the host's real-time
kernel configuration and robot network, including firewall rules for UDP traffic.
Docker Engine alone does not ensure that the control loop meets its deadlines.
See Franka's [real-time kernel setup](https://frankarobotics.github.io/docs/doc/libfranka/docs/real_time_kernel.html)
and [network troubleshooting](https://frankarobotics.github.io/docs/troubleshooting.html).

### Build permission errors

```text
CMake Error at .../extract-googletest.cmake:21 (file):
  file problem creating directory: /ros2_ws/src/libfranka/3rdparty/../ex-googletest1234
```

This can happen when the image's user UID differs from the owner of the mounted
checkout. libfranka downloads Google Test into `src/`, and Python packages may
write `*.egg-info` there during a symlink build, so the source tree must be writable.
Compare the numeric identities:

```bash
# Host — repository root, with this checkout's Compose variables
id
stat -c 'Checkout owner UID:GID: %u:%g (%n)' .
docker compose exec franka_ros2 id
```

For a checkout owned by your account, rebuild the image with your host UID/GID
and recreate the service. Save any outputs outside `/ros2_ws/src` first: this
replaces the container and requires rebuilding the workspace.

```bash
# Host — repository root; keep the existing project and container names
export USER_UID="$(id -u)"
export USER_GID="$(id -g)"
docker compose up -d --build --force-recreate
```

Then repeat [GUI access](#gui-access-rviz-and-gazebo), if needed, and
[build the workspace](#build-the-workspace). If the identities already match,
check ownership and write permissions on the affected files, including files
created by earlier root-owned builds. See [Shared workstations](#shared-workstations).

### GUI windows never appear

Compose captures `DISPLAY` when the container is created. If your graphical
session changes, the container may still target the old display. Compare the
values from a host terminal in your active graphical session:

```bash
# Host — repository root, with this checkout's Compose variables
echo "$DISPLAY"
docker compose exec franka_ros2 printenv DISPLAY
xauth nlist "$DISPLAY"
```

If the host's `DISPLAY` is empty, use a terminal in a graphical session with X11
or XWayland. If `xauth` returns no cookie, check the host's `XAUTHORITY` and session
configuration before proceeding.

Repeat the [cookie transfer](#gui-access-rviz-and-gazebo), then open a shell with
the current display and authorization file:

```bash
# Host — repository root
docker compose exec -e DISPLAY="$DISPLAY" -e XAUTHORITY=/tmp/franka.xauth franka_ros2 bash
```

Inside that shell, source `/ros2_ws/install/setup.bash` before launching ROS
applications. This updates GUI access for the shell without recreating the
container or losing the build. In an existing VS Code terminal, export the host's
current `DISPLAY` value and `XAUTHORITY=/tmp/franka.xauth` before launching a GUI.

For rqt plugins, use `ros2 run rqt_image_view rqt_image_view`; the plugin executable
may not be on `PATH` as a standalone command.

## Contributing

Contributions are welcome! Please see [CONTRIBUTING.md](https://github.com/frankarobotics/franka_ros2/blob/humble/CONTRIBUTING.md) for more details on how to contribute to this project.

## License

All packages of franka_ros2 are licensed under the Apache 2.0 license.

## Contact

For questions or support, please open an issue on the [GitHub Issues](https://github.com/frankarobotics/franka_ros2/issues) page.

See the [Franka Control Interface (FCI) documentation](https://frankarobotics.github.io/docs) for more information.
