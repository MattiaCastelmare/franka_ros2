# franka_experiments

Wrapper launch files for Franka robots experiments.

## joint_velocity_example_controller — How it works

### Architecture

`JointVelocityExampleController` is a **closed-loop demo controller** — it does
**NOT** accept external commands via topics. The velocity trajectory is
generated internally in `update()`.

Source files:
- `franka_example_controllers/src/joint_velocity_example_controller.cpp`
- `franka_example_controllers/include/franka_example_controllers/joint_velocity_example_controller.hpp`

### Command interfaces (ros2_control)

The controller claims **7 velocity command interfaces** via `command_interface_configuration()`:

| Interface name               |
|------------------------------|
| `fr3_joint1/velocity`        |
| `fr3_joint2/velocity`        |
| `fr3_joint3/velocity`        |
| `fr3_joint4/velocity`        |
| `fr3_joint5/velocity`        |
| `fr3_joint6/velocity`        |
| `fr3_joint7/velocity`        |

### State interfaces (ros2_control)

It reads **14 state interfaces** via `state_interface_configuration()`:

- `fr3_jointN/position` (N=1..7)
- `fr3_jointN/velocity` (N=1..7)

### Motion pattern

In `update()` the controller writes velocity to **joint 4 and joint 5 only**
(indices 3, 4). All other joints get 0.0. The velocity follows a sinusoidal
pattern with period 8 seconds and amplitude 0.1 rad/s.

### No external topic / subscriber

There is **no subscriber** and **no input topic**. You cannot send velocity
commands via `ros2 topic pub`. The `command_interfaces_` are written directly
in the real-time `update()` loop — this is the standard ros2_control pattern
for demo controllers.

### Debug commands

```bash
# Verify controller is active
ros2 control list_controllers

# List claimed command interfaces
ros2 control list_hardware_interfaces

# With namespace NS_1:
ros2 control list_controllers -c /NS_1/controller_manager
ros2 control list_hardware_interfaces -c /NS_1/controller_manager

# Watch joint states (you should see joint 4 and 5 moving)
ros2 topic echo /joint_states
```

### Launching

```bash
# Fake hardware (no real robot needed)
ros2 launch franka_experiments wrapper_velocity.launch.py use_fake_hardware:=true

# With namespace
ros2 launch franka_experiments wrapper_velocity.launch.py use_fake_hardware:=true namespace:=NS_1
```

### Next steps

To send **custom** velocity commands, a new controller (or a
`ForwardCommandController`) that subscribes to a topic would be needed.
The current `joint_velocity_example_controller` is a self-contained demo only.
