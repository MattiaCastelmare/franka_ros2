# ROS 2 Integration for Franka Robotics Research Robots

[![CI](https://github.com/frankarobotics/franka_ros2/actions/workflows/ci.yml/badge.svg)](https://github.com/frankarobotics/franka_ros2/actions/workflows/ci.yml)

> **Note:** _franka_ros2_ is not officially supported on Windows.

## Table of Contents
- [About](#about)
- [Fork Structure and Recommended Branch](#fork-structure-and-recommended-branch)
- [Caution](#caution)
- [Setup](#setup)
  - [Local Machine Installation](#local-machine-installation)
  - [Docker Container Installation](#docker-container-installation)
- [Test the Setup](#test-the-setup)
- [Troubleshooting](#troubleshooting)
  - [libfranka: UDP receive: Timeout error](#libfranka-udp-receive-timeout-error)
- [Contributing](#contributing)
- [License](#license)
- [Contact](#contact)

## About
The **franka_ros2** repository provides a **ROS 2** integration of **libfranka**, allowing efficient control of the Franka Robotics arm within the ROS 2 framework. This project is designed to facilitate robotic research and development by providing a robust interface for controlling the research versions of Franka Robotics robots.

For convenience, we provide Dockerfile and docker-compose.yml files. While it is possible to build **franka_ros2** directly on your local machine, this approach requires manual installation of certain dependencies, while many others will be automatically installed by the **ROS 2** build system (e.g., via **rosdep**). This can result in a large number of libraries being installed on your system, potentially causing conflicts. Using Docker encapsulates these dependencies within the container, minimizing such risks. Docker also ensures a consistent and reproducible build environment across systems. For these reasons, we recommend using Docker.

## Fork Structure and Recommended Branch

This repository is a **fork** of the official `frankarobotics/franka_ros2` project and includes additional **research-oriented extensions**, simulation tools, and Docker enhancements.

### Branches

- **humble**  
  Mirror of the official upstream branch. Not intended for development.

- **humble-mattia** (recommended)  
  Includes `franka_simulation`, extended Docker setup, and extra source dependencies.

### Recommended clone command

```
git clone -b humble-mattia https://github.com/MattiaCastelmare/franka_ros2.git
```

## Caution
This package is in rapid development. Users should expect breaking changes.

## Setup

### Extra dependencies

This fork introduces an additional `.repos` file:

- `extras.repos`: clones `moveit2` and `pymoveit2` as source dependencies.

```
vcs import src < src/extras.repos --recursive --skip-existing
```

## Test the Setup

### Build only franka_simulation

```
colcon build --packages-select franka_simulation --symlink-install
source install/setup.bash
```

## License
Apache 2.0
