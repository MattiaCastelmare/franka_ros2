# franka_experiments

Wrapper launch files and experiment nodes for Franka robots.

## Available launch files

| Launch file | Description |
|---|---|
| `wrapper_forward_velocity.launch.py` | Bringup + velocity controller (RT blender or legacy forward-velocity) |
| `random_waypoints_velocity.launch.py` | Random end-effector waypoint commander (requires a running velocity controller) |

## Quick start

### RT velocity blender (default)

```bash
# Fake hardware (no real robot needed)
ros2 launch franka_experiments wrapper_forward_velocity.launch.py use_fake_hardware:=true

# With namespace
ros2 launch franka_experiments wrapper_forward_velocity.launch.py use_fake_hardware:=true namespace:=NS_1

# Real hardware (default)
ros2 launch franka_experiments wrapper_forward_velocity.launch.py robot_ip:=192.168.2.10
```

### Legacy forward-velocity controller

```bash
ros2 launch franka_experiments wrapper_forward_velocity.launch.py use_rt_blender:=false use_fake_hardware:=true
```

### Random waypoints commander

Requires a velocity controller already running (e.g. via `wrapper_forward_velocity.launch.py`):

```bash
ros2 launch franka_experiments random_waypoints_velocity.launch.py
ros2 launch franka_experiments random_waypoints_velocity.launch.py namespace:=NS_1 num_waypoints:=20
```

## Debug commands

```bash
# Verify controller is active
ros2 control list_controllers

# List claimed command interfaces
ros2 control list_hardware_interfaces

# With namespace NS_1:
ros2 control list_controllers -c /NS_1/controller_manager
ros2 control list_hardware_interfaces -c /NS_1/controller_manager

# Watch joint states
ros2 topic echo /joint_states
```
