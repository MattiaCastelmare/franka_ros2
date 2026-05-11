# franka_simulation — Test Infrastructure

Validation scripts and launch files for the four simulation pipelines.
All tests require a working ROS 2 + Gazebo environment and the workspace to be built.

```
test/
├── launch/
│   ├── test_velocity_pipeline.launch.py
│   ├── test_acceleration_pipeline.launch.py
│   ├── test_torque_pipeline.launch.py
│   └── test_cbf_pipeline.launch.py
├── scripts/
│   ├── test_velocity_publisher.py
│   ├── test_acceleration_publisher.py
│   ├── test_torque_publisher.py
│   └── check_pipeline.sh
├── config/
│   └── test_publishers.yaml
└── README.md  ← this file
```

---

## Pipeline 1 — Velocity

### Single command (automated)

```bash
ros2 launch franka_simulation test_velocity_pipeline.launch.py
```

What happens:
1. Gazebo + RViz start with FR3 robot
2. `fr3_velocity_controller` is spawned and activated
3. After 10 s: `test_velocity_publisher` starts publishing sinusoidal velocities
4. `fr3_joint1` oscillates at ±0.15 rad/s, 0.1 Hz for 30 s
5. Publisher stops and publishes zeros

### Manual test

```bash
# Terminal 1: start simulation
ros2 launch franka_simulation sim_velocity.launch.py

# Terminal 2: publish a constant velocity on joint 1
ros2 topic pub /fr3_velocity_controller/commands std_msgs/Float64MultiArray \
  "{data: [0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"
```

### Validation

```bash
# Check controllers and topics
./test/scripts/check_pipeline.sh velocity

# Monitor topic
ros2 topic hz /fr3_velocity_controller/commands
ros2 topic echo /joint_states --field velocity
```

### Expected results

| Check | Expected |
|-------|----------|
| `fr3_velocity_controller` state | `active` |
| `/joint_states` rate | ~1000 Hz (Gazebo) |
| `/fr3_velocity_controller/commands` rate | 50 Hz (from publisher) |
| Joint 1 velocity in `/joint_states` | tracks commanded value |
| Robot visible in RViz | yes, moving |

### Pass criteria

- `ros2 control list_controllers` shows `fr3_velocity_controller [active]`
- Robot visibly moves in Gazebo and RViz when commands are published
- `/joint_states.velocity[0]` changes when commands are sent

---

## Pipeline 2 — Acceleration

### Single command (automated)

```bash
ros2 launch franka_simulation test_acceleration_pipeline.launch.py
```

What happens:
1. Gazebo + velocity pipeline start (same as velocity test)
2. `sim_acceleration_bridge` starts (integrates q̈ → q̇)
3. After 10 s: `test_acceleration_publisher` sends sinusoidal accelerations
4. Bridge integrates and clamps to velocity limits
5. `fr3_joint1` accelerates/decelerates following the integrated profile

### Manual test

```bash
# Terminal 1
ros2 launch franka_simulation sim_acceleration.launch.py

# Terminal 2: publish constant acceleration on joint 1
ros2 topic pub /sim_acceleration_bridge/accel_cmd std_msgs/Float64MultiArray \
  "{data: [0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"
```

### Key topics

| Topic | Type | Direction |
|-------|------|-----------|
| `/sim_acceleration_bridge/accel_cmd` | Float64MultiArray (7) | **IN** — joint accelerations [rad/s²] |
| `/fr3_velocity_controller/commands`  | Float64MultiArray (7) | **OUT** — integrated joint velocities |

### Expected results

| Check | Expected |
|-------|----------|
| `fr3_velocity_controller` state | `active` |
| `sim_acceleration_bridge` node | running |
| Bridge output rate | 100 Hz |
| Joint velocity | ramps up then clamps at `qdot_max` |

---

## Pipeline 3 — Torque

### Single command (automated)

```bash
ros2 launch franka_simulation test_torque_pipeline.launch.py
```

What happens:
1. Gazebo starts with `gazebo_effort:=true` (effort command interface enabled)
2. `fr3_effort_controller` is spawned
3. After 12 s: `test_torque_publisher` starts
4. Phase 0 (10 s): zeros — robot holds current pose (Gazebo physics = equilibrium)
5. Phase 1 (20 s): ±2 Nm on joint 1 — robot oscillates
6. Phase 2: zeros — robot stops

### Manual test

```bash
# Terminal 1
ros2 launch franka_simulation sim_torque.launch.py

# Terminal 2: publish zeros (equilibrium test)
ros2 topic pub /fr3_effort_controller/commands std_msgs/Float64MultiArray \
  "{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"

# Terminal 2: apply small torque on joint 1
ros2 topic pub /fr3_effort_controller/commands std_msgs/Float64MultiArray \
  "{data: [2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"
```

### IMPORTANT — gravity semantics

**Gazebo manages gravity via physics simulation.**
The `fr3_effort_controller` adds commanded torques directly to the physics engine.

- Publishing zeros → robot at physics equilibrium (arm does NOT fall)
- Send only task torques: τ_cmd = M(q)·q̈ + C(q,q̇)·q̇
- Do NOT add gravity compensation g(q) — Gazebo handles it

This is different from real hardware where `rt_torque_controller` receives τ without g(q)
because the Franka firmware compensates gravity internally.

### Prerequisite check

```bash
# Verify effort command interface is available
ros2 control list_hardware_interfaces | grep effort
```

Expected output includes lines like:
```
fr3_joint1/effort [available] [claimed]
```

If not visible, the xacro was compiled without `gazebo_effort:=true`.

### Expected results

| Check | Expected |
|-------|----------|
| `fr3_effort_controller` state | `active` |
| Effort interfaces | `available` and `claimed` |
| Robot with zeros | holds pose (Gazebo equilibrium) |
| Robot with ±2 Nm | joint 1 oscillates |

---

## Pipeline 4 — CBF / Capsules

### Single command

```bash
ros2 launch franka_simulation test_cbf_pipeline.launch.py
```

Or with specific options:

```bash
# Without obstacles (test pure CBF kinematics)
ros2 launch franka_simulation test_cbf_pipeline.launch.py spawn_obstacles:=false

# Without MoveIt (test avoidance controller only)
ros2 launch franka_simulation test_cbf_pipeline.launch.py enable_moveit:=false
```

### Validation

```bash
# Check CBF pipeline
./test/scripts/check_pipeline.sh cbf

# Monitor CBF distances
ros2 topic echo /avoidance/min_distance

# Check capsule markers in RViz
# Add display: MarkerArray → /robot_capsules_markers
```

### Expected RViz visualization

- Robot model (URDF-based) moving
- Yellow/green cylinders overlapping each arm link (robot capsules)
- Obstacle markers if `spawn_obstacles:=true`

### Expected topics active

| Topic | Publisher | Rate |
|-------|-----------|------|
| `/avoidance/min_distance` | online_avoidance_controller | ~100 Hz |
| `/avoidance/closest_constraint` | online_avoidance_controller | ~100 Hz |
| `/robot_capsules_markers` | online_avoidance_controller | ~100 Hz |
| `/fr3_velocity_controller/commands` | velocity_control_blender | ~100 Hz |

### Pass criteria

- All four topics above are active
- `/avoidance/min_distance` > 0 when no collision
- CBF kicks in (velocity commands change) when robot approaches obstacle

---

## Standalone publisher nodes

All test publishers can run standalone without the full test launch:

```bash
# Standalone velocity publisher
ros2 run franka_simulation test_velocity_publisher \
  --ros-args -p rate_hz:=50.0 -p amplitude:=0.1 -p duration_s:=60.0

# Standalone acceleration publisher
ros2 run franka_simulation test_acceleration_publisher \
  --ros-args -p amplitude:=0.5 -p active_joint:=1

# Standalone torque publisher
ros2 run franka_simulation test_torque_publisher \
  --ros-args -p amplitude:=1.0 -p zeros_duration_s:=5.0
```

---

## Common issues

### Gazebo does not start
- Check `GZ_SIM_RESOURCE_PATH` is set (launch files set it automatically)
- Verify `franka_description` is installed: `ros2 pkg list | grep franka_description`

### Controller not spawning
- Wait longer — controllers spawn after Gazebo + robot are ready (timers in launch)
- Check: `ros2 control list_controllers`
- If spawner errors, check YAML syntax: `ros2 param get /controller_manager update_rate`

### Torque pipeline: effort interface not available
- Verify `sim_torque.launch.py` was used (not `sim_velocity.launch.py`)
- The xacro must be compiled with `gazebo_effort:=true`
- Run: `ros2 control list_hardware_interfaces | grep effort`
- If empty: potential gz_ros2_control bug — check installed version

### Robot falls in torque pipeline
- Normal for very large torques or wrong signs
- Start with zeros to verify equilibrium
- Use conservative amplitude (< 5 Nm for joint 1)

### RViz shows TF errors (no parent frame)
- Fixed frame must be `fr3_link0`
- Verify RSP is publishing: `ros2 topic echo /joint_states --once`

### Topics not visible
- Source the workspace: `source install/setup.bash`
- Check if Gazebo simulation is paused (click play in Gazebo GUI)
