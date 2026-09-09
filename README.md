# Franka ROS 2 Research Framework

[![CI](https://github.com/frankarobotics/franka_ros2/actions/workflows/ci.yml/badge.svg)](https://github.com/frankarobotics/franka_ros2/actions/workflows/ci.yml)

This repository provides a **ROS 2 integration framework for Franka Robotics research robots**, extending the official [`franka_ros2`](https://github.com/frankarobotics/franka_ros2) project with additional tools for:

- 🛡 **Control Barrier Function (CBF) safety filters** for real-time collision avoidance
- 🧪 **Robotics research experiments** on the real FR3
- 🖥 **Simulation environments** (Gazebo/Ignition + RViz2, and MuJoCo)
- 🤖 **MoveIt2 motion planning**
- 🧠 **Safe Reinforcement Learning** with sim-to-real deployment
- 📦 **Docker-based development environments**

> **This repository is a fork** of the official [frankarobotics/franka_ros2](https://github.com/frankarobotics/franka_ros2).
> It adds **four research-oriented packages** on top of the upstream codebase:
>
> | Package | Purpose |
> |---|---|
> | **`franka_experiments`** | Two end-to-end CBF safety stacks for the real FR3 (acceleration/torque and velocity): motion generators, HOCBF/OSCBF QP filters, depth-camera human–robot distance estimation, hand–eye calibration |
> | **`franka_rt_controllers`** | Real-time C++ `ros2_control` controllers running inside the 1 kHz loop: joint-torque executor, joint-velocity executor, and a C++ CBF torque controller |
> | **`franka_simulation`** | Gazebo (Ignition) + RViz2 + MoveIt2 simulation with four control pipelines (position, velocity, acceleration, torque) plus a CBF/avoidance pipeline |
> | **`franka_sim`** | Standalone MuJoCo module (no ROS 2) for training a Safe-RL policy against the *same* CBF filter that runs on the robot, exported to ONNX for deployment |

The goal of this fork is to provide a **reproducible robotics research environment** for developing and testing algorithms such as:

- safety-critical control (CBF / QP)
- motion control and collision avoidance
- perception-driven control
- human–robot interaction
- safe reinforcement learning and sim-to-real transfer

> **Note:** `franka_ros2` is not officially supported on Windows.

## Table of Contents
- [About](#about)
- [Research Extensions in This Fork](#research-extensions-in-this-fork)
- [Caution](#caution)
- [Setup](#setup)
  - [Local Machine Installation](#local-machine-installation)
  - [Docker Container Installation](#docker-container-installation)
    - [Shared workstation: several accounts on one PC](#shared-workstation-several-accounts-on-one-pc)
- [Test the Setup](#test-the-setup)
- [franka_experiments](#franka_experiments)
- [franka_rt_controllers](#franka_rt_controllers)
- [franka_simulation](#franka_simulation)
- [franka_sim](#franka_sim)
- [Documentation map](#documentation-map)
- [Troubleshooting](#troubleshooting)
  - [libfranka: UDP receive: Timeout error](#libfranka-udp-receive-timeout-error)
  - [colcon build fails in libfranka with a permission error](#colcon-build-fails-in-libfranka-with-a-permission-error)
  - [GUI windows never appear](#gui-windows-never-appear)
- [Contributing](#contributing)
- [License](#license)
- [Contact](#contact)

## About

The **franka_ros2** project provides the official **ROS 2 interface for Franka Robotics research robots**, built on top of the low-level **libfranka** control library.

It enables developers to control Franka robots within the **ROS 2 ecosystem**, providing access to:

- real-time robot control
- ROS 2 control interfaces
- integration with the ROS 2 toolchain (RViz2, MoveIt2, Gazebo)
- modular controllers and hardware abstractions

This repository is a **fork of the official [`frankarobotics/franka_ros2`](https://github.com/frankarobotics/franka_ros2)** project and extends it with additional features aimed at robotics research workflows.

The main additions of this fork include:

- **`franka_experiments`** — CBF safety filters and experiment nodes for the real FR3 (or fake hardware), covering both an acceleration/torque stack and a velocity stack
- **`franka_rt_controllers`** — real-time C++ `ros2_control` controllers that execute torque and velocity commands in the 1 kHz loop
- **`franka_simulation`** — Gazebo (Ignition) and RViz2 simulation with MoveIt2 planning, four control pipelines, and an online collision avoidance pipeline
- **`franka_sim`** — MuJoCo-based Safe-RL training module whose learned policies deploy back onto the ROS 2 torque stack through ONNX
- a **Docker-based development environment** for reproducible builds
- additional utilities for robotics experimentation

### Why Docker?

While it is possible to install all dependencies directly on the host system, the **Docker-based workflow** provides several advantages:

- reproducible development environments
- simplified dependency management
- reduced risk of library conflicts
- easier onboarding for new users

For these reasons, **using Docker is the recommended installation method** for this repository.
The training stack used by `franka_sim` (PyTorch + CUDA, MuJoCo, Gymnasium, Stable-Baselines3, ONNX Runtime, OSQP) lives **only** in the container image, not on the host.

## Research Extensions in This Fork

This fork extends the official `frankarobotics/franka_ros2` project with additional
research-oriented extensions. The four additional packages introduced in this fork are:

- **`franka_experiments`** — The main research package. Implements two end-to-end control stacks, both centred on **Control Barrier Function** safety filters that enforce collision avoidance at run time. The **torque stack** works in acceleration space (motion generator → HOCBF QP → inverse dynamics → 1 kHz torque controller); an alternative torque path uses the Operational Space CBF of Morton & Pavone (arXiv:2503.06736). The **velocity stack** works at the kinematic level (velocity commander → velocity-CBF QP → 1 kHz velocity executor). Shared infrastructure includes `real_time_distance`, a depth-camera human–robot distance estimator, plus experiment logging, RViz capsule visualisation, and an AprilTag hand–eye calibration pipeline.

- **`franka_rt_controllers`** — Real-time C++ `ros2_control` plugins that keep the hard deadline out of Python: `rt_torque_controller` (effort interfaces, optional low-pass filter, per-joint clipping; gravity is added by the Franka firmware), `rt_velocity_executor_controller` (velocity interfaces, interpolation, rate limiting, timeout ramp), and `cbf_torque_controller` (inverse dynamics from `qddot_safe` inside the RT loop). No heap allocations, mutexes, or logging in the RT path.

- **`franka_simulation`** — Gazebo (Ignition) + RViz2 simulation of the FR3 with MoveIt2 integration and four selectable control pipelines (position, velocity, acceleration, torque), plus a CBF/avoidance pipeline with a Pinocchio-based online avoidance controller, a velocity-blending CBF-QP, obstacle synchronisation with the MoveIt planning scene, and an optional RealSense + MediaPipe human pose pipeline.

- **`franka_sim`** — Standalone MuJoCo training module with **no ROS 2 dependency**. It reproduces the robot's acceleration-level CBF filter inside a Gymnasium environment, trains a SAC policy that explores *behind the same shield* it will meet on hardware, and exports the actor to ONNX. `rl_policy_commander` in `franka_experiments` replays that ONNX graph on the real robot (`motion_source:=rl`).

Docker support and the `.devcontainer` configuration are also developed in this fork.

### Branch structure

This fork follows a clear branching strategy to ensure portability and easy synchronization with the official repository:

- **`humble`**
  - Mirrors the official upstream branch `frankarobotics/franka_ros2:humble`
  - Intended to track the upstream branch, although temporary deviations may occur due to libfranka and firmware compatibility requirements
  - Not intended for custom development in this fork

- **`humble-mattia`** 
  - Stable branch including additional simulation packages, Docker extensions, and research tooling
  - This is the **recommended branch for users who want to clone and use this fork**
  - Actively maintained and periodically rebased/merged with upstream updates

## Caution
This package is in rapid development. Users should expect breaking changes and are encouraged to report any bugs via [GitHub Issues page](https://github.com/frankarobotics/franka_ros2/issues).

## Setup

## Franka ROS 2 Dependencies Setup

This repository contains a `.repos` file that helps you clone the required dependencies for Franka ROS 2.

## Prerequisites

## Local Machine Installation
1. **Install ROS 2 Development environment**

    _**franka_ros2**_ is built upon _**ROS 2 humble**_.

    To set up your ROS 2 environment, follow the official _**humble**_ installation instructions provided [**here**](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html).
    The guide discusses two main installation options: **Desktop** and **Bare Bones**.

    ### Choose **one** of the following:
    - **ROS 2 "Desktop Install"** (`ros-humble-desktop`)
      Includes a full ROS 2 installation with GUI tools and visualization packages (e.g., Rviz and Gazebo).
      **Recommended** for users who need simulation or visualization capabilities.

    - **"ROS-Base Install (Bare Bones)"** (`ros-humble-ros-base`)
      A minimal installation that includes only the core ROS 2 libraries.
      Suitable for resource-constrained environments or headless systems.

    ```bash
    # replace <YOUR CHOICE> with either ros-humble-desktop or ros-humble-ros-base
    sudo apt install <YOUR CHOICE>
    ```
    ---
    Also install the **Development Tools** package:
    ```bash
    sudo apt install ros-dev-tools
    ```
    Installing the **Desktop** or **Bare Bones** should automatically source the **ROS 2** environment but, under some circumstances you may need to do this again:
    ```bash
    source /opt/ros/humble/setup.sh
    ```

2. **Create a ROS 2 Workspace:**
   ```bash
   mkdir -p ~/franka_ros2_ws/src
   cd ~/franka_ros2_ws  # not into src
   ```
3. **Clone the Repositories:**
   ```bash
   git clone --recurse-submodules https://github.com/MattiaCastelmare/franka_ros2.git src
    ```
4. **Install the dependencies**
    ```bash
    vcs import src < src/franka.repos --recursive --skip-existing
    ```
5. **Detect and install project dependencies**
   ```bash
   rosdep install --from-paths src --ignore-src --rosdistro humble -y
   ```
6. **Build**
   ```bash
   # use the --symlinks option to reduce disk usage, and facilitate development.
   colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
   ```
7. **Adjust Enviroment**
   ```bash
   # Adjust environment to recognize packages and dependencies in your newly built ROS 2 workspace.
   source install/setup.sh
   ```

## Docker Container Installation
The **franka_ros2** package includes a `Dockerfile` and a `docker-compose.yml`, which allows you to use `franka_ros2` packages without manually installing **ROS 2**. Also, the support for Dev Containers in Visual Studio Code is provided.

For detailed instructions, on preparing VSCode to use the `.devcontainer` follow the setup guide from [VSCode devcontainer_setup](https://code.visualstudio.com/docs/devcontainers/tutorial).

1. **Clone the Repositories:**

    ```bash
    git clone -b humble-mattia https://github.com/MattiaCastelmare/franka_ros2.git
    cd franka_ros2
    ```
    `libfranka` and `franka_description` are **not** part of this repository — they are
    listed in `franka.repos` and pulled in later with `vcs import` (step 6 below), which
    is why a fresh clone does not contain them. This repo registers no git submodules, so
    `--recurse-submodules` has no effect here.
    We provide separate instructions for using Docker with Visual Studio Code or the command line. Choose one of the following options:

    Option A: Set up and use Docker from the command line (without Visual Studio Code).

    Option B: Set up and use Docker with Visual Studio Code's Docker support.

### Option A: using Docker Compose

  2. **Declare your user id and your own Compose project:**
      ```bash
      export COMPOSE_PROJECT_NAME=franka_$USER
      export USER_UID=$(id -u)
      export USER_GID=$(id -g)
      ```
      Add those lines to your `~/.bashrc` so every new shell has them.

      `USER_UID`/`USER_GID` are baked into the image at build time
      (`Dockerfile:127-128`) and must match the owner of your clone: bind mounts hand
      the kernel raw numeric uids, with no translation. `COMPOSE_PROJECT_NAME` gives you
      your own image tag, so that several accounts on one PC do not overwrite each
      other's image — see
      [Shared workstation](#shared-workstation-several-accounts-on-one-pc).

      A `.env` file holding the same two variables also works, but only on a
      single-user machine: shell variables take precedence over `.env`, and a clone
      shared between accounts would share its `.env` too.

  3. **Build the container:**
      ```bash
      docker compose build
      ```
  4. **Run the container:**
      ```bash
      docker compose up -d
      ```
  5. **Open a shell inside the container:**
      ```bash
      docker exec -it franka_ros2 /bin/bash
      ```
  6. **Clone the latests dependencies:**
      ```bash
      vcs import src < src/franka.repos --recursive --skip-existing
      ```
  7. **Build the workspace:**
      ```bash
      colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
      ```
  8. **Build only franka_simulation package:**
      ```bash
      colcon build --packages-select franka_simulation --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
      ```
  9. **Source the built workspace:**
      ```bash
      source install/setup.bash
      ```
  10. **When you are done, you can exit the shell and delete the container**:
      ```bash
      docker compose down -t 0
      ```

### Option B: using Dev Containers in Visual Studio Code

  2. **Open Visual Studio Code ...**

        Then, open folder  `franka_ros2`

  3. **Choose `Reopen in container` when prompted.**

      The container will be built automatically, as required.

  4. **Clone the latests dependencies:**
      ```bash
      vcs import src < src/franka.repos --recursive --skip-existing
      ```

  5. **Open a terminal and build the workspace:**
      The **first** time you build the workspace, on systems with ~32 GB of RAM we recommend parallel execution with four workers:
        ```bash
        colcon build --symlink-install --executor parallel --parallel-workers 4 --cmake-args -DCMAKE_BUILD_TYPE=Release
        ```
     The **others** time use the following command:
      ```bash
      colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
      ```
  7. **Source the built workspace environment:**
      ```bash
      source install/setup.bash
      ```


### Shared workstation: several accounts on one PC

A machine runs **one Docker daemon**, and images plus container names live in a single
namespace shared by every account on it. Three facts turn that into a conflict here:

1. The container user's uid is fixed at build time — `Dockerfile:127-128` runs
   `useradd --uid ${USER_UID}` — so an image built by one account carries that account's
   uid forever.
2. Bind mounts do not translate uids. `./:/ros2_ws/src` hands the kernel raw numbers, so
   the uid inside the container must *numerically equal* the owner of the files on the
   host.
3. The Compose project name defaults to the directory name — `franka_ros2` for everybody
   — so without the variables below every account targets the same image tag and the same
   container.

The result is a tug-of-war: whoever runs `docker compose build` last wins, and everyone
else's `colcon build` then fails with permission errors inside `src/`.

**Rule: one clone per account, one Compose project per account.**

Each user adds this to their own `~/.bashrc` and opens a new terminal:

```bash
export COMPOSE_PROJECT_NAME=franka_$USER   # your own image tag
export FRANKA_CONTAINER=franka_$USER       # your own container name
export USER_UID=$(id -u)
export USER_GID=$(id -g)
```

`FRANKA_CONTAINER` defaults to `franka_ros2`, which is the name every command in this
README uses. Exactly one account on the machine may leave it unset; every other account
has to set it, or `docker compose up` fails with *container name already in use*.

Then, from your **own** clone:

```bash
git clone -b humble-mattia https://github.com/MattiaCastelmare/franka_ros2.git \
  ~/Git/franka_ros2
cd ~/Git/franka_ros2
docker compose up -d --build
docker exec -it "${FRANKA_CONTAINER:-franka_ros2}" /bin/bash
```

Then, **inside** the container, pull the two out-of-tree packages and build:

```bash
vcs import src < src/franka.repos --recursive --skip-existing
colcon build --symlink-install --executor parallel --parallel-workers 24 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

`vcs import` writes into `/ros2_ws/src`, which is your clone on the host — another reason
the uid inside the container has to match its owner. MoveIt and pymoveit2 come from apt in
the image, so `extras.repos` is not needed for this flow.

The clone has to live in your own home directory. Separate Compose projects stop the
accounts from overwriting each other's image and container, but they do not change file
ownership: mounting somebody else's clone still leaves your container unable to write
into `src/`.

To see what belongs to whom:

```bash
docker ps -a --format 'table {{.Names}}\t{{.Image}}\t{{.Status}}'
docker exec <container> id     # the uid inside must equal the clone owner's uid
stat -c '%u %n' .              # owner of your clone
```

Do not delete another account's image (`franka_<user>-franka_ros2`) to reclaim disk
space: it is the only image matching their uid.

# Test the build
   ```bash
   colcon test
   ```
> Remember, franka_ros2 is under development.
> Warnings can be expected.

## Test the Setup

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
ros2 launch franka_experiments torque_control_stack.launch.py use_fake_hardware:=true
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
| `rl` | `rl_policy_commander` | Replays the ONNX Safe-RL policy trained in [`franka_sim`](#franka_sim). Pass `start_move_group:=false`. |

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
| `fr3_control.yaml` | CBF gains and tuning for both stacks — mirrored by `franka_sim/config.yaml` and checked by the tests |
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

## franka_sim

`franka_sim` is a **standalone MuJoCo training module with no ROS 2 dependency**. It trains a **Safe Reinforcement Learning** policy (SAC, Stable-Baselines3) shielded by the *same* acceleration-level CBF filter that runs on the real robot in `franka_experiments/nodes/cbf_safety_filter.py` — safe exploration in simulation, safe execution on hardware.

Full guide: [`franka_sim/README.md`](./franka_sim/README.md) · roadmap: [`franka_sim_to_real_roadmap.md`](./franka_sim_to_real_roadmap.md) · validation status: [`franka_sim_to_real_implementation_status.md`](./franka_sim_to_real_implementation_status.md).

```
franka_sim/
├── assets/franka_fr3/    # MuJoCo FR3 model incl. the Franka Hand (transforms from the real URDF)
├── envs/
│   ├── franka_cbf_env.py # Gymnasium env FrankaCBF-v0 (reach + moving obstacle)
│   └── cbf_filter.py     # AccelCBFFilter — mirrors the robot's HOCBF QP
├── scripts/              # evaluate_policy, compare_checkpoints, validate_cbf, validate_actuation
├── config.yaml           # mirrors `params:` in franka_experiments/config/fr3_control.yaml
├── train.py              # SAC training + checkpointing (step- and episode-spaced)
└── export_onnx.py        # actor → ONNX, validated against the SB3 policy
```

Everything runs **inside the container** (the training stack is in the image, not on the host):

```bash
# Train
docker exec -it franka_ros2 bash -lc 'cd /ros2_ws/src && MUJOCO_GL=egl \
  python3 -m franka_sim.train --exp-name sac_v3 --total-timesteps 2000000'

# Score every checkpoint against the zero-action and random baselines
docker exec -it franka_ros2 bash -lc 'cd /ros2_ws/src && MUJOCO_GL=egl \
  python3 -m franka_sim.scripts.compare_checkpoints --model-dir franka_sim/models/sac_v3'

# Export a checkpoint for the robot
docker exec -it franka_ros2 bash -lc 'cd /ros2_ws/src && \
  python3 -m franka_sim.export_onnx --model franka_sim/models/sac_v3/best_model.zip'
```

Then deploy the exported policy onto the torque stack:

```bash
ros2 launch franka_experiments torque_control_stack.launch.py \
    motion_source:=rl start_move_group:=false \
    rl_onnx_model:=/path/to/best_model.onnx rl_action_scale:=0.3
```

> **Training artefacts are not versioned.** `franka_sim/models/`, `runs/`, `*.zip` and `*.onnx` are git-ignored: they are regenerated by `train.py` / `export_onnx.py`. To move a policy between machines, copy the `.onnx` **together with the `config.yaml` frozen next to it** — the deployment node falls back to the repository default otherwise, which may not be the configuration the policy was trained under.
>
> Sim and robot configurations are kept in sync by `franka_experiments/test/test_rl_policy.py`, which fails if a CBF gain drifts between `franka_sim/config.yaml` and `fr3_control.yaml`.

---

## Documentation map

| Document | Contents |
|---|---|
| [`franka_experiments/README.md`](./franka_experiments/README.md) | Node-by-node reference, topics, launch sequencing, configuration keys |
| [`franka_experiments/test/README.md`](./franka_experiments/test/README.md) | Test suite: what is checked and how to run it |
| [`franka_simulation/README.md`](./franka_simulation/README.md) | The four simulation pipelines, controllers, kinematics library |
| [`franka_sim/README.md`](./franka_sim/README.md) | Training, evaluation, the MuJoCo viewer, sim↔robot sync, gotchas |
| [`franka_sim_to_real_roadmap.md`](./franka_sim_to_real_roadmap.md) | Sim-to-real architecture and plan |
| [`franka_sim_to_real_implementation_status.md`](./franka_sim_to_real_implementation_status.md) | What is built and how it was validated |
| [`CBF_PIPELINE_AUDIT.md`](./CBF_PIPELINE_AUDIT.md) | Audit of the CBF pipeline |
| [`SAFE_RL_CBF_HANDOVER.md`](./SAFE_RL_CBF_HANDOVER.md) | Safe-RL + CBF handover notes |

---

## Troubleshooting
### `libfranka: UDP receive: Timeout error`

If you encounter a UDP receive timeout error while communicating with the robot, avoid using Docker Desktop. It may not provide the necessary real-time capabilities required for reliable communication with the robot. Instead, using Docker Engine is sufficient for this purpose.

A real-time kernel is essential to ensure proper communication and to prevent timeout issues. For guidance on setting up a real-time kernel, please refer to the [Franka installation documentation](https://frankarobotics.github.io/docs/installation_linux.html#setting-up-the-real-time-kernel).

### `colcon build` fails in libfranka with a permission error

```
CMake Error at .../extract-googletest.cmake:21 (file):
  file problem creating directory: /ros2_ws/src/libfranka/3rdparty/../ex-googletest1234
```

The container user cannot write into the mounted source tree, because its uid differs
from the owner of your clone. libfranka downloads Google Test *into* `src/`
(`cmake/SetupGoogleTest.cmake`), and `franka_experiments` is an `ament_python` package,
so `--symlink-install` writes `*.egg-info` there as well — read-only access is not
enough. Confirm the mismatch:

```bash
docker exec <container> id     # uid inside the container
stat -c '%u %n' .              # owner of the clone on the host
```

Then rebuild the image with your own uid:

```bash
export COMPOSE_PROJECT_NAME=franka_$USER USER_UID=$(id -u) USER_GID=$(id -g)
docker rm -f "${FRANKA_CONTAINER:-franka_ros2}"
docker compose up -d --build
```

`docker compose down` only removes containers of the *current* project, so a container
created before you set `COMPOSE_PROJECT_NAME` must be removed by name with
`docker rm -f`. Recreating the container also wipes `/ros2_ws/build` and
`/ros2_ws/install`, which live inside the container and not in the mount, so the next
`colcon build` starts from scratch. See
[Shared workstation](#shared-workstation-several-accounts-on-one-pc).

### GUI windows never appear

RViz or `rqt_image_view` start without any error, yet no window shows up.
`docker-compose.yml` captures `DISPLAY` when the container is **created**. On a machine
with several graphical sessions the stored value can point at another user's screen, and
the window then opens there. Compare the two:

```bash
echo $DISPLAY                             # your session, on the host
docker exec <container> printenv DISPLAY
```

If they differ, recreate the container from your active session with
`docker compose up -d --force-recreate`, or override per command. Overriding also needs
an X cookie, because a display owned by another session refuses unauthorized clients
(`Authorization required, but no authorization protocol specified`):

```bash
: > /tmp/docker.xauth
xauth nlist "$DISPLAY" | sed -e 's/^..../ffff/' | xauth -f /tmp/docker.xauth nmerge -
docker cp /tmp/docker.xauth <container>:/tmp/docker.xauth
docker exec -it -e DISPLAY="$DISPLAY" -e XAUTHORITY=/tmp/docker.xauth <container> /bin/bash
```

Prefer this to `xhost +local:`, which opens your display to every account on the machine.
Note also that rqt plugin executables are not on `PATH`: start them with
`ros2 run rqt_image_view rqt_image_view`, not with the bare command name.

## Contributing

Contributions are welcome! Please see [CONTRIBUTING.md](https://github.com/frankarobotics/franka_ros2/blob/humble/CONTRIBUTING.md) for more details on how to contribute to this project.

## License

All packages of franka_ros2 are licensed under the Apache 2.0 license.

## Contact

For questions or support, please open an issue on the [GitHub Issues](https://github.com/frankarobotics/franka_ros2/issues) page.

See the [Franka Control Interface (FCI) documentation](https://frankarobotics.github.io/docs) for more information.

[def]: #docker-container-installation
