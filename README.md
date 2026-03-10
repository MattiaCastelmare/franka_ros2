# Franka ROS 2 Research Framework

[![CI](https://github.com/frankarobotics/franka_ros2/actions/workflows/ci.yml/badge.svg)](https://github.com/frankarobotics/franka_ros2/actions/workflows/ci.yml)

This repository provides a **ROS 2 integration framework for Franka Robotics research robots**, extending the official [`franka_ros2`](https://github.com/frankarobotics/franka_ros2) project with additional tools for:

- 🧪 **Robotics research experiments**
- 🖥 **Simulation environments (RViz2 + Gazebo)**
- 🤖 **MoveIt2 motion planning**
- 🛡 **Real-time collision avoidance**
- 📦 **Docker-based development environments**

> **This repository is a fork** of the official [frankarobotics/franka_ros2](https://github.com/frankarobotics/franka_ros2).
> It adds **three research-oriented packages** on top of the upstream codebase:
>
> | Package | Purpose |
> |---|---|
> | **`franka_simulation`** | Gazebo (Ignition) + RViz2 simulation with MoveIt2 integration, online collision avoidance, and velocity blending |
> | **`franka_experiments`** | Experiment launch files, velocity commander nodes, velocity blending, and hand–eye calibration utilities for real and simulated robots |
> | **`franka_rt_controllers`** | Real-time C++ `ros2_control` velocity blending controller running at 1 kHz, replacing slower Python-based blenders |

The goal of this fork is to provide a **reproducible robotics research environment** for developing and testing algorithms such as:

- motion control
- collision avoidance
- perception-driven control
- human–robot interaction

> **Note:** `franka_ros2` is not officially supported on Windows.

## Table of Contents
- [About](#about)
- [Research Extensions in This Fork](#research-extensions-in-this-fork)
- [Caution](#caution)
- [Setup](#setup)
  - [Local Machine Installation](#local-machine-installation)
  - [Docker Container Installation](#docker-container-installation)
- [Test the Setup](#test-the-setup)
- [franka_simulation](#franka_simulation)
- [franka_experiments](#franka_experiments)
- [franka_rt_controllers](#franka_rt_controllers)
- [Troubleshooting](#troubleshooting)
  - [libfranka: UDP receive: Timeout error](#libfranka-udp-receive-timeout-error)
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

- **`franka_simulation`** — simulation environments based on RViz2 and Gazebo (Ignition), with MoveIt2 motion planning and online collision avoidance
- **`franka_experiments`** — experiment launch files and velocity commander nodes for running real and simulated experiments, plus hand–eye calibration
- **`franka_rt_controllers`** — a real-time C++ `ros2_control` controller for velocity blending at 1 kHz
- a **Docker-based development environment** for reproducible builds
- additional utilities for robotics experimentation

### Why Docker?

While it is possible to install all dependencies directly on the host system, the **Docker-based workflow** provides several advantages:

- reproducible development environments
- simplified dependency management
- reduced risk of library conflicts
- easier onboarding for new users

For these reasons, **using Docker is the recommended installation method** for this repository.

## Research Extensions in This Fork

This fork extends the official `frankarobotics/franka_ros2` project with additional
research-oriented extensions. The three additional packages introduced in this fork are:

- **`franka_simulation`** — Provides RViz2 and Gazebo (Ignition) simulation environments with MoveIt2 integration. Includes a motion planning server, an online collision avoidance controller using Pinocchio, a velocity blending pipeline, obstacle synchronization, and an optional camera-based human pose detection pipeline. Designed for collision avoidance research in simulation before deploying to real hardware.

- **`franka_experiments`** — Provides launch files and ROS 2 nodes for running velocity-controlled experiments on the real Franka FR3 robot (or with fake hardware). Includes multiple velocity commander nodes (sinusoidal, pentagon trajectory, random waypoints), a Python-based velocity blender, and a full hand–eye calibration pipeline using AprilTags.

- **`franka_rt_controllers`** — Provides a real-time C++ `ros2_control` controller (`rt_velocity_blender_controller`) that performs velocity blending, rate limiting, timeout ramping, and velocity clamping inside the 1 kHz real-time loop. This replaces the Python-based velocity blender when deterministic real-time performance is required.

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
    git clone -b humble-mattia --recurse-submodules https://github.com/MattiaCastelmare/franka_ros2.git
    cd franka_ros2
    ```
    We provide separate instructions for using Docker with Visual Studio Code or the command line. Choose one of the following options:

    Option A: Set up and use Docker from the command line (without Visual Studio Code).

    Option B: Set up and use Docker with Visual Studio Code's Docker support.

### Option A: using Docker Compose

  2. **Save the current user id into a file:**
      ```bash
      echo -e "USER_UID=$(id -u $USER)\nUSER_GID=$(id -g $USER)" > .env
      ```
      It is needed to mount the folder from inside the Docker container.

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


# Test the build
   ```bash
   colcon test
   ```
> Remember, franka_ros2 is under development.
> Warnings can be expected.

## Test the Setup

### Run the simulation environment (franka_simulation)

To quickly test the extended simulation framework provided in this fork, launch the `franka_simulation` package:

```bash
ros2 launch franka_simulation move_group.launch.py
```

This starts Gazebo (Ignition), the robot state publisher, MoveIt2 `move_group`, RViz2, the online avoidance controller, the velocity blender, and the motion server. See the [franka_simulation](#franka_simulation) section for full details.

### Run a sample ROS 2 application

To verify that your setup works correctly without a robot, you can run the following command to use dummy hardware:

```bash
ros2 launch franka_fr3_moveit_config moveit.launch.py robot_ip:=dont-care use_fake_hardware:=true
```
You can use the arguments `load_gripper` to activate or deactivate the end-effector and `ee_id` to set which end-effector you want to use. By default, the Franka Hand is activated.

If you want to run this example with namespaces, you would need to use the argument `namespace` and manually write your namespace in `moveit.rviz` under `Move Group Namespace`.

### Run a ROS 2 example controller

To run any example controller, make sure to add your desired configuration in `franka.config.yaml` and run:

```bash
ros2 launch franka_bringup example.launch.py controller_name:=your_desired_controller
```
You can select one of the controllers from `controllers.yaml`.

### Run Gazebo examples with ROS 2

If you want to use Gazebo to run your code, you can find some examples here: [franka_gazebo](./franka_gazebo/README.md)

---

## franka_simulation

The `franka_simulation` package provides a complete **simulation environment** for the Franka FR3 robot, integrating **Gazebo (Ignition)**, **RViz2**, **MoveIt2**, and an **online collision avoidance** pipeline. It is designed for developing and testing collision avoidance algorithms in simulation before deploying to real hardware.

### What it provides

- Full Gazebo (Ignition) simulation of the Franka FR3 with `ros2_control`
- MoveIt2 integration for motion planning (OMPL)
- An online collision avoidance controller using Pinocchio and CBF/QP-based methods
- A velocity blending pipeline that merges tracking and avoidance velocities
- Obstacle synchronization between Gazebo, MoveIt planning scene, and the avoidance controller
- An optional RealSense camera + MediaPipe human pose detection pipeline
- Custom ROS 2 action interfaces (`MoveToPose`, `MoveToJoint`, `PlanGlobalPath`)

### Nodes

| Node | Description |
|---|---|
| `franka_motion_server` | MoveIt2-based motion planning server. Exposes `MoveToPose`, `MoveToJoint`, and `PlanGlobalPath` action servers. Plans trajectories using MoveIt2 and publishes them as `JointTrajectory` messages for the velocity blender. Collision checking in IK is delegated to the online avoidance controller. |
| `franka_motion_client` | Client library for the motion server. Provides a simple Python API (`move_to_pose()`, `move_to_joint()`, `get_current_pose()`) to interact with the action servers. |
| `online_avoidance_controller` | Computes minimum distances between robot body capsules (via Pinocchio FK) and obstacles in the planning scene. Publishes closest-constraint Jacobians, distance information, and RViz markers for visualization. Uses capsule-based robot geometry with configurable radii. |
| `velocity_control_blender` | Blends tracking velocities (from the motion server's trajectory) with avoidance corrections. Implements ḋ-constraint enforcement, CBF-based safety shaping, risk-scaled filtering, and trajectory rejoin logic. Publishes final joint-velocity commands to the `fr3_velocity_controller`. |
| `obstacle_synchronizer` | Reads obstacle geometry from URDF/Xacro files, publishes them as `CollisionObject` messages to the MoveIt planning scene, and broadcasts obstacle information to the avoidance controller via `/obstacle_scene`. |
| `image_publisher` | Republishes RealSense camera images from `/camera/camera/color/image_raw` to `/my_camera/image` with reliable QoS. Acts as a QoS adapter between the camera driver and downstream nodes. |
| `human_pose_node` | Subscribes to camera images, runs MediaPipe pose estimation, and publishes annotated images with human skeleton overlays to `/human_pose/image`. |

### Launch files

#### `move_group.launch.py` — Full simulation with MoveIt2 and collision avoidance

This is the **main launch file** for the simulation environment. It starts the complete pipeline:

- **Gazebo (Ignition)** with an empty world
- **Robot State Publisher** with the FR3 URDF (including `ros2_control` and Gazebo plugins)
- **MoveIt2 `move_group`** node with OMPL planning, kinematics, and trajectory execution
- **Static TF publishers** (`world → base → fr3_link0`)
- **Controller spawners**: `joint_state_broadcaster` and `fr3_velocity_controller`
- **Obstacle State Publisher + Gazebo obstacle spawn** (configurable)
- **Obstacle Synchronizer** — publishes obstacles to planning scene and avoidance controller
- **RViz2** with MoveIt visualization
- **Gazebo clock bridge** (`/clock`)
- **Online Avoidance Controller** — distance monitoring and constraint computation
- **Velocity Control Blender** — merges tracking + avoidance into final velocity commands
- **Motion Server** — MoveIt2 trajectory planning via custom action interfaces
- *(Optional)* **RealSense camera + image publisher + human pose node**
- *(Optional)* **Safe avoidance test** demo node

```bash
ros2 launch franka_simulation move_group.launch.py
```

**Key launch arguments:**

| Argument | Default | Description |
|---|---|---|
| `arm_id` | `fr3` | Robot model identifier |
| `load_gripper` | `true` | Whether to include the Franka Hand |
| `enable_moveit` | `true` | Enable MoveIt2 integration |
| `spawn_obstacles` | `true` | Spawn collision obstacles in the scene |
| `run_safe_test` | `false` | Run the safe avoidance test demo |
| `enable_camera` | `true` | Enable RealSense + image pipeline |

**Example with custom arguments:**
```bash
ros2 launch franka_simulation move_group.launch.py spawn_obstacles:=false enable_camera:=false
```

#### `franka_simulation.launch.py` — Basic Gazebo + RViz2 simulation (no MoveIt)

A simpler launch file for basic simulation without MoveIt2 or the avoidance pipeline:

- **Gazebo (Ignition)** with an empty world
- **Robot State Publisher** with the FR3 URDF
- **Controller spawners**: `joint_state_broadcaster` and `fr3_arm_controller` (joint trajectory controller)
- **RViz2** with Franka visualization

```bash
ros2 launch franka_simulation franka_simulation.launch.py
```

This is useful for testing basic joint-level control in Gazebo without the overhead of MoveIt2 or the avoidance stack.

### Custom action interfaces

The package defines three custom ROS 2 action interfaces:

| Action | Description |
|---|---|
| `MoveToPose.action` | Move the end-effector to a target Cartesian pose |
| `MoveToJoint.action` | Move to a target joint configuration |
| `PlanGlobalPath.action` | Plan a global path (returns the planned trajectory) |

### Architecture overview

The simulation pipeline operates as follows:

```
                         ┌──────────────────┐
                         │  Motion Server   │
                         │ (MoveIt2 plans)  │
                         └────────┬─────────┘
                                  │ JointTrajectory
                                  ▼
┌───────────────┐     ┌──────────────────────────┐     ┌──────────────────┐
│   Obstacle    │────▶│   Velocity Blender       │────▶│ Velocity         │
│ Synchronizer  │     │ (tracking + avoidance)   │     │ Controller       │
└───────────────┘     └──────────┬───────────────┘     │ (ros2_control)   │
                                 ▲                     └────────┬─────────┘
                      ┌──────────┴───────────┐                  │
                      │  Online Avoidance    │                  ▼
                      │  Controller          │          ┌───────────────┐
                      │ (Pinocchio + CBF/QP) │          │  Gazebo       │
                      └──────────────────────┘          │  (Ignition)   │
                                                        └───────────────┘
```

1. The **Motion Server** uses MoveIt2 to plan joint trajectories and publishes them to the velocity blender.
2. The **Online Avoidance Controller** continuously monitors distances between robot capsules and obstacles using Pinocchio FK, publishing closest-constraint Jacobians and distance data.
3. The **Velocity Blender** fuses tracking velocities (from the planned trajectory) with avoidance corrections (from the avoidance controller) using ḋ-constraint enforcement and CBF-based safety shaping. It publishes the final velocity command.
4. The **Velocity Controller** (a `ros2_control` `ForwardCommandController` for velocity) sends the commands to the simulated robot in Gazebo.
5. The **Obstacle Synchronizer** keeps obstacles synchronized across the MoveIt planning scene, the avoidance controller, and the Gazebo simulation.

### Typical workflow

1. Launch the full simulation: `ros2 launch franka_simulation move_group.launch.py`
2. Wait for all nodes to start (the launch file uses timed delays to ensure correct startup ordering)
3. Use the motion server actions to command the robot to target poses/joints
4. The velocity blender automatically handles obstacle avoidance during motion execution
5. Monitor the avoidance behavior in RViz2 (obstacle capsules, distance markers, etc.)

---

## franka_experiments

The `franka_experiments` package provides **launch files and ROS 2 nodes** for running velocity-controlled experiments on the Franka FR3 robot, both with real hardware and with fake (simulated) hardware. It is a Python-based (`ament_python`) package.

### What it provides

- A wrapper launch file that brings up the full Franka robot driver with velocity control
- Multiple velocity commander nodes for generating joint-velocity commands
- A Python-based velocity blender for merging tracking and avoidance velocity channels
- A hand–eye calibration pipeline using AprilTags
- Support for both the real-time C++ blender (`franka_rt_controllers`) and the legacy Python-based forward velocity controller

### Velocity control pipeline

The experiments package uses a **velocity-based control architecture**. Joint-velocity commands are published by commander nodes and consumed by a velocity controller running on the robot hardware interface:

```
  ┌──────────────────────┐
  │  Velocity Commander  │──▶ /tracking_qdot (Float64MultiArray)
  │  (Python node)       │
  └──────────────────────┘
              │
              ▼
  ┌──────────────────────┐
  │  Velocity Blender    │──▶ /fr3_forward_velocity_controller/commands
  │  (Python or RT C++)  │       OR
  └──────────────────────┘    /rt_velocity_blender_controller (hw interface)
              ▲
              │
  /avoidance_qdot ◀── (avoidance node, if running)
```

**Two blending modes are available:**

- **RT mode** (default, `use_rt_blender:=true`): Uses `rt_velocity_blender_controller` from `franka_rt_controllers` — C++ blending at 1 kHz inside the real-time loop.
- **Legacy mode** (`use_rt_blender:=false`): Uses `fr3_forward_velocity_controller` (a standard `ForwardCommandController`) with an optional Python `velocity_blender` node.

### Nodes

| Node | Description |
|---|---|
| `velocity_commander` | Publishes **sinusoidal** joint-velocity commands. Configurable amplitudes, frequencies, and offsets per joint. |
| `smooth_velocity_commander` | Like `velocity_commander` but with a **warmup phase** and **cosine-ramp envelope** for smooth startup. Publishes at 200 Hz. |
| `ee_pentagon_velocity_commander` | Tracks a **pentagon trajectory** in Cartesian space using Pinocchio Jacobian-based resolved-rate control. Publishes to `tracking_qdot`. |
| `ee_random_waypoints_velocity_commander` | Tracks **random Cartesian waypoints** within a configurable bounding box using Jacobian-based resolved-rate control with minimum-jerk time profiles. Publishes to `tracking_qdot`. |
| `velocity_blender` | Python velocity blender/mux. Subscribes to `tracking_qdot` and `avoidance_qdot`, blends them, applies per-joint clamping and watchdog safety, and publishes the result to the forward velocity controller. |
| `handeye_calibration_node` | Full hand–eye calibration pipeline. Supports manual (move-by-hand + ENTER) and automatic (velocity-based random waypoints) acquisition modes. Solves the AX=XB calibration problem using nonlinear SE(3) optimization with outlier filtering. |

### Launch files

#### `wrapper_forward_velocity.launch.py` — Main experiment launch file

This is the primary launch file for running experiments. It:

1. Includes `franka_bringup/franka.launch.py` to start the robot driver, URDF, and standard broadcasters
2. Spawns the velocity controller (RT blender or legacy forward velocity controller)
3. Optionally starts the Python velocity blender (legacy mode only)
4. Optionally starts RViz2, the RealSense camera pipeline, and the human pose node

```bash
# RT velocity blender with fake hardware (no real robot)
ros2 launch franka_experiments wrapper_forward_velocity.launch.py use_fake_hardware:=true

# RT velocity blender with real hardware
ros2 launch franka_experiments wrapper_forward_velocity.launch.py robot_ip:=192.168.2.10

# Legacy forward velocity controller
ros2 launch franka_experiments wrapper_forward_velocity.launch.py use_rt_blender:=false use_fake_hardware:=true

# With a namespace
ros2 launch franka_experiments wrapper_forward_velocity.launch.py use_fake_hardware:=true namespace:=NS_1
```

**Key launch arguments:**

| Argument | Default | Description |
|---|---|---|
| `use_rt_blender` | `true` | Use RT C++ blender (true) or legacy Python forward velocity (false) |
| `use_fake_hardware` | `false` | Use fake hardware interface (no real robot) |
| `robot_ip` | `192.168.1.10` | IP address of the real robot |
| `namespace` | `""` | ROS 2 namespace for the robot |
| `load_gripper` | `true` | Load the Franka Hand |
| `enable_camera` | `true` | Enable RealSense camera pipeline |
| `start_rviz` | `true` | Launch RViz2 |
| `qdot_max` | `1.5` | Maximum joint velocity (rad/s) for the RT blender |
| `alpha` | `0.5` | Blend weight: `alpha * tracking + (1 - alpha) * avoidance` |

Default values can be edited in `franka_experiments/config/launch_defaults.yaml` without modifying Python code.

#### `handeye_calibration_bringup.launch.py` — Hand–eye calibration pipeline

Launches the complete hand–eye calibration pipeline in a single command:

1. Includes `wrapper_forward_velocity.launch.py` to bring up the robot driver and velocity controller
2. Starts the `apriltag_node` for AprilTag detection from the camera
3. Starts the `handeye_calibration_node` (delayed to allow TF and driver startup)
4. Automatically shuts down all processes when calibration is complete

```bash
ros2 launch franka_experiments handeye_calibration_bringup.launch.py
```

**Launch arguments:**

| Argument | Default | Description |
|---|---|---|
| `apriltag_family` | `36h11` | AprilTag family |
| `apriltag_size` | `0.10` | Physical size of the tag in metres |
| `calibration_delay` | `3.0` | Seconds to wait before starting calibration |

### Running velocity commander nodes

After launching the robot with `wrapper_forward_velocity.launch.py`, you can start a velocity commander in a separate terminal:

```bash
# Sinusoidal velocity commands
ros2 run franka_experiments velocity_commander

# Smooth sinusoidal with warmup ramp
ros2 run franka_experiments smooth_velocity_commander

# Pentagon Cartesian trajectory
ros2 run franka_experiments ee_pentagon_velocity_commander

# Random Cartesian waypoints
ros2 run franka_experiments ee_random_waypoints_velocity_commander
```

### Debug commands

```bash
# Verify controller is active
ros2 control list_controllers

# List claimed command interfaces
ros2 control list_hardware_interfaces
```

---

## franka_rt_controllers

The `franka_rt_controllers` package provides a **real-time C++ `ros2_control` controller** for velocity blending on the Franka FR3 robot.

### Controller: `rt_velocity_blender_controller`

The `RtVelocityBlenderController` is a `ros2_control` `ControllerInterface` plugin that performs **tracking/avoidance velocity blending inside the 1 kHz real-time loop**, eliminating sample-and-hold jitter caused by non-RT Python publishers.

**Features:**
- Subscribes to two velocity topics (`tracking_qdot`, `avoidance_qdot`) and a blend weight topic (`blend_alpha`)
- Uses `RealtimeBuffer` for lock-free, allocation-free data transfer from non-RT subscribers to the RT `update()` loop
- Configurable blend weight: `alpha * tracking + (1 - alpha) * avoidance`
- Optional **linear interpolation** between consecutive low-rate samples to eliminate velocity step changes
- Optional **rate limiter** (`max_accel`) to bound per-joint jerk
- Optional **smooth timeout ramp**: if an input topic stops publishing, the contribution ramps to zero over a configurable duration
- Final per-joint **velocity clamp** (`qdot_max`)
- **No heap allocations, no mutexes, no logging** in the real-time path

**Architecture:**

```
  Python nodes ──topic──▶ RealtimeBuffer ──readFromRT──▶ update() @ 1 kHz
                                                            │
                    blend → interpolate → rate-limit → clamp → command_interfaces
```

### Launch file

```bash
ros2 launch franka_rt_controllers rt_velocity_blender.launch.py
```

This launch file:
1. Includes `franka_bringup/franka.launch.py` to load the URDF and start the hardware interface
2. Spawns the `rt_velocity_blender_controller`
3. Reads robot defaults from `franka_bringup/config/franka.config.yaml`

**Key launch arguments:**

| Argument | Default | Description |
|---|---|---|
| `robot_ip` | `192.168.2.10` | IP address of the robot |
| `use_fake_hardware` | `false` | Use fake hardware interface |
| `namespace` | `""` | ROS 2 namespace |
| `controllers_yaml` | `__auto__` | Path to controllers YAML (auto = package default) |

> **Note:** The `rt_velocity_blender_controller` claims `fr3_joint{1..7}/velocity` command interfaces. No other velocity controller can be active on the same joints at the same time. Deactivate any conflicting controller before launching.

### When to use which controller

| Scenario | Controller | Package |
|---|---|---|
| Real hardware, real-time blending at 1 kHz | `rt_velocity_blender_controller` | `franka_rt_controllers` |
| Real hardware, simple forward velocity | `fr3_forward_velocity_controller` + Python blender | `franka_experiments` |
| Gazebo simulation | `fr3_velocity_controller` + Python velocity blender | `franka_simulation` |

---

## Troubleshooting
### `libfranka: UDP receive: Timeout error`

If you encounter a UDP receive timeout error while communicating with the robot, avoid using Docker Desktop. It may not provide the necessary real-time capabilities required for reliable communication with the robot. Instead, using Docker Engine is sufficient for this purpose.

A real-time kernel is essential to ensure proper communication and to prevent timeout issues. For guidance on setting up a real-time kernel, please refer to the [Franka installation documentation](https://frankarobotics.github.io/docs/installation_linux.html#setting-up-the-real-time-kernel).

## Contributing

Contributions are welcome! Please see [CONTRIBUTING.md](https://github.com/frankarobotics/franka_ros2/blob/humble/CONTRIBUTING.md) for more details on how to contribute to this project.

## License

All packages of franka_ros2 are licensed under the Apache 2.0 license.

## Contact

For questions or support, please open an issue on the [GitHub Issues](https://github.com/frankarobotics/franka_ros2/issues) page.

See the [Franka Control Interface (FCI) documentation](https://frankarobotics.github.io/docs) for more information.

[def]: #docker-container-installation
