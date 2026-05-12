# franka_experiments — Test Infrastructure

Validation launch files and scripts for the three experiment pipelines.
All tests use `use_fake_hardware:=true` — no physical FR3 robot required.

```
test/
├── launch/
│   ├── test_velocity_fake.launch.py
│   ├── test_torque_fake.launch.py
│   └── test_oscbf_fake.launch.py
├── scripts/
│   └── check_topics.sh
├── config/
│   └── test_defaults.yaml
└── README.md  ← this file
```

---

## Pipeline 1 — Velocity (fake hardware)

### Single command

```bash
ros2 launch franka_experiments test_velocity_fake.launch.py
```

### What happens

1. `franka_bringup` starts with `use_fake_hardware:=true`
2. `rt_velocity_executor_controller` spawns in namespace `NS_1`
3. After 20 s: `ee_pentagon_velocity_commander` starts
4. Pentagon trajectory in the YZ plane of `fr3_link0` (radius=0.15 m, cycle=15 s)
5. Warm-up (2 s) + cosine ramp (2 s) → full velocity commands

### Topics to monitor

| Topic | Type | Rate | Description |
|-------|------|------|-------------|
| `/NS_1/joint_states` | JointState | ~1000 Hz | Fake encoder feedback |
| `/NS_1/qdot_cmd` | Float64MultiArray (7) | ~200 Hz | Commander output (joint velocities) |

### Validation

```bash
./test/scripts/check_topics.sh velocity
```

### Manual test

```bash
# Terminal 1
ros2 launch franka_experiments test_velocity_fake.launch.py

# Terminal 2: inspect commanded velocities
ros2 topic echo /NS_1/qdot_cmd

# Terminal 3: check controller is active
ros2 control list_controllers
```

### Expected results

| Check | Expected |
|-------|----------|
| `rt_velocity_executor_controller` | `active` |
| `/NS_1/qdot_cmd` rate | ~200 Hz |
| Topic content | 7 joint velocities, smooth and bounded by `qdot_max=0.3 rad/s` |
| No errors in logs | No `stale joint-state` warnings after ramp |

### Pass criteria

- Controller is active
- `/NS_1/qdot_cmd` publishes at ≥100 Hz after warmup
- No NaN or inf values in topic

---

## Pipeline 2 — Torque (fake hardware)

### Single command

```bash
ros2 launch franka_experiments test_torque_fake.launch.py
```

### What happens

1. `franka_bringup` starts with `use_fake_hardware:=true`
2. `rt_torque_controller` spawns, listens on `/NS_1/torque_cmd`
3. After 20 s: `pentagon_torque_commander` starts
4. Pentagon trajectory at ±0.3 m radius via 6D Cartesian PD → inverse dynamics
5. Published torques: τ = M(q)·q̈ + C(q,q̇)·q̇ (no gravity — added by rt_torque_controller)

### Topics to monitor

| Topic | Type | Rate | Description |
|-------|------|------|-------------|
| `/NS_1/joint_states` | JointState | ~1000 Hz | Fake encoder |
| `/NS_1/torque_cmd` | Float64MultiArray (7) | ~100 Hz | τ_nom (task torques, no gravity) |

### Validation

```bash
./test/scripts/check_topics.sh torque
```

### Gravity compensation note

```
pentagon_torque_commander  →  τ_nom = M(q)·q̈ + C(q,q̇)·q̇  (no g(q))
rt_torque_controller       →  τ_hw  = τ_filtered             (no g(q) added in code)
Franka firmware            →  handles gravity compensation at 1 kHz
```

See `refactoring_code.md §10.2` for the full analysis of gravity handling.

### Expected results

| Check | Expected |
|-------|----------|
| `rt_torque_controller` | `active` |
| `/NS_1/torque_cmd` rate | ~100 Hz |
| Torque magnitudes | < 30 Nm at startup, bounded by joint limits |

---

## Pipeline 3 — OSCBF (fake hardware)

### Phase 1 — bypass (default, safe)

```bash
ros2 launch franka_experiments test_oscbf_fake.launch.py
```

What happens:
- `oscbf_filter` starts in bypass mode (no QP)
- `pentagon_torque_commander` sends τ_nom → filter passes through unchanged
- `torque_safe` == `torque_cmd`

### Phase 2 — OSCBF QP (joint-position CBF)

```bash
ros2 launch franka_experiments test_oscbf_fake.launch.py bypass_filter:=false
```

What happens:
- `oscbf_filter` solves OSCBF-QP at 100 Hz
- Joint-position CBF enforces joint limits as safety constraints
- `torque_safe` ≠ `torque_cmd` when approaching limits

### Phase 3 — full obstacle CBF

```bash
ros2 launch franka_experiments test_oscbf_fake.launch.py \
    bypass_filter:=false enable_obstacle_cbf:=true
```

Note: this starts `real_time_distance` and camera nodes.
With fake hardware, distance data is not real — for functional validation only.

### Topics to monitor

| Topic | Type | Rate | Description |
|-------|------|------|-------------|
| `/NS_1/torque_cmd` | Float64MultiArray (7) | ~100 Hz | τ_nom from commander |
| `/NS_1/torque_safe` | Float64MultiArray (7) | ~100 Hz | τ* from OSCBF filter |
| `/NS_1/joint_states` | JointState | ~1000 Hz | Fake encoder |

Phase 3 additionally:
| `/cbf/per_link_distances` | MultiLinkDistance | ~30 Hz | Capsule distances |

### Validation

```bash
# Phase 1
./test/scripts/check_topics.sh oscbf

# Phase 3
./test/scripts/check_topics.sh oscbf_obstacle
```

### Checking filter is active (Phase 2+)

```bash
# Difference between torque_cmd and torque_safe indicates QP is modifying torques
ros2 topic echo /NS_1/torque_cmd --field data
ros2 topic echo /NS_1/torque_safe --field data
```

### Expected results

| Phase | torque_safe | QP solve time | CBF constraint |
|-------|-------------|---------------|----------------|
| 1 (bypass) | == torque_cmd | 0 ms | none |
| 2 (joint CBF) | may differ | < 1 ms | joint limits |
| 3 (obstacle) | differs near obstacle | < 2 ms | joint + obstacle |

---

## Common checks

### Is the controller active?

```bash
ros2 control list_controllers
```

Expected for velocity test:
```
joint_state_broadcaster[active]  joint_state_broadcaster/JointStateBroadcaster
rt_velocity_executor_controller[active]  franka_rt_controllers/RtVelocityExecutorController
```

Expected for torque/OSCBF tests:
```
joint_state_broadcaster[active]  joint_state_broadcaster/JointStateBroadcaster
rt_torque_controller[active]  franka_rt_controllers/RtTorqueController
```

### Is fake hardware working?

```bash
ros2 topic echo /NS_1/joint_states --once
```

Expected: 7 joint positions near zero (home configuration), velocities near zero.

---

## Common issues

### Controller not spawning (timeout)

```bash
# Check controller_manager is reachable
ros2 control list_controllers

# Check logs for the spawner
ros2 node list | grep controller
```

If the spawner times out, try increasing `control_spawner_delay_s`:
```bash
ros2 launch franka_experiments test_torque_fake.launch.py \
    control_spawner_delay_s:=20.0
```

### "No joint state received yet"

The commander checks for `/NS_1/joint_states`. Verify:
```bash
ros2 topic echo /NS_1/joint_states --once
```

If the topic is in a different namespace, pass `namespace:=<your_ns>` to the launch.

### OSCBF filter reports "no distances" (Phase 3)

With fake hardware there is no real camera. The filter will log warnings
about missing `/cbf/per_link_distances` data. This is expected for fake HW testing —
the QP runs with default (large) distances, so no avoidance kicks in.

### Pinocchio model not found

Commander nodes build the Pinocchio model from the FR3 xacro.
Ensure `franka_description` is installed:
```bash
ros2 pkg list | grep franka_description
```

### "stale joint-state" warning from commander

This is normal during the warmup phase while fake hardware initializes.
Disappears after 2–5 seconds.
