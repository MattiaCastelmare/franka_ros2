# franka_experiments — Test Infrastructure

Validation launch files and scripts for the three experiment pipelines.
All tests use `use_fake_hardware:=true` — no physical FR3 robot required.

```
test/
├── launch/
│   ├── test_velocity_fake.launch.py
│   ├── test_torque_fake.launch.py
│   ├── test_oscbf_fake.launch.py
│   └── test_rl_fake.launch.py          # Safe-RL (ONNX policy) accel pipeline
├── scripts/
│   └── check_topics.sh
├── config/
│   └── test_defaults.yaml
├── smoke_cbf_safety_filter.py          # node-level, no bringup
├── smoke_rl_policy_commander.py        # node-level, no bringup
├── test_avoidance.py                   # pytest (pure numpy)
├── test_cbf_hard_constraints.py
├── test_cbf_velocity_filter.py
├── test_rl_policy.py                   # sim↔real observation/action contract
└── README.md  ← this file
```

Pure-python unit tests run with `pytest test/` (68 tests, no ROS graph
needed beyond a sourced workspace).

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

> ⚠️ **`test_oscbf_fake.launch.py` does not exist in this repository** (it never
> did — see `git log`). The commands in this section are the intended interface
> and are kept as the specification for that launch file; until it is written,
> start `pentagon_torque_commander` + `cbf_oscbf_filter` manually. The
> `check_topics.sh oscbf` / `oscbf_obstacle` checks below do work against a
> manually started stack.

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

## Pipeline 4 — Safe-RL / ONNX policy (fake hardware)

Deployment side of `franka_sim_to_real_roadmap.md` Step 3: the SAC policy
trained in `franka_sim/` against *this* CBF filter, replayed on the robot by
`rl_policy_commander`.

### Prerequisite — an exported policy

```bash
cd /ros2_ws/src && export PYTHONPATH=/ros2_ws/src MUJOCO_GL=egl
python3 -m franka_sim.train --total-timesteps 400000 --exp-name sac_deploy
python3 -m franka_sim.export_onnx --model franka_sim/models/sac_deploy/best_model.zip
# score the artifact that actually ships:
python3 -m franka_sim.scripts.evaluate_policy \
    --model franka_sim/models/sac_deploy/best_model.onnx --episodes 50
```

### Node-level smoke test (no bringup, ~10 s)

```bash
python3 test/smoke_rl_policy_commander.py           # auto-discovers a policy
```

Checks the warm-up gate, the 100 Hz rate, `|q̈| ≤ q̈_max`, that the node's
command is **bit-equal** to an independently rebuilt observation + ONNX
inference (the sim↔real observation contract), that `d_min` propagates from
`MultiLinkDistance`, and that a stale perception / joint-state stream forces
zeros.

### Full pipeline with fake hardware

```bash
ros2 launch franka_experiments test_rl_fake.launch.py \
    rl_onnx_model:=/ros2_ws/src/franka_sim/models/sac_deploy/best_model.onnx
```

```bash
./test/scripts/check_topics.sh rl        # topic/node validation
```

This is `torque_control_stack.launch.py motion_source:=rl` with fake hardware
and perception off — camera, `real_time_distance` and `move_group` are not
started, so the commander runs its documented "perception never started" path
(parked synthetic obstacle) and the CBF keeps only its hard state/workspace
rows.

### Topics to monitor

| Topic | Type | Rate | Description |
|-------|------|------|-------------|
| `/NS_1/qddot_nom` | Float64MultiArray (7) | ~100 Hz | policy output `a·q̈_max` |
| `/NS_1/qddot_safe` | Float64MultiArray (7) | ~100 Hz | after the CBF QP |
| `/NS_1/torque_cmd` | Float64MultiArray (7) | ~100 Hz | τ = M·q̈ + C·q̇ |
| `/NS_1/rl_status` | Float64MultiArray (6) | ~100 Hz | `[infer_ms, tick_ms, d_min, dist, target_idx, gate]` |

### Expected results

| Check | Expected |
|-------|----------|
| `rt_torque_controller` | `active` |
| `/NS_1/qddot_nom` rate | ~100 Hz, jitter std < 0.5 ms |
| `rl_status[0]` (inference) | < 0.5 ms per tick |
| `rl_status[5]` (gate) | 1 during warm-up, then 0 |
| First 3 s | all-zero commands (warm-up) |

### Timing / jitter evidence

`rl_policy_commander` writes `franka_logs/rl_policy_run_<stamp>.csv` with the
full observation plus `tick_ms` / `infer_ms` per tick:

```bash
python3 scripts/plot_rl_timing.py          # newest CSV → stats + figure
```

Reference numbers from a 120 s fake-hardware run:

```
loop period        mean  9.998 ms  std 0.159  p99 10.421  max 12.520
ONNX inference     mean  0.138 ms  std 0.041  p99  0.268  max  1.066
ticks > 1.5x nominal: 0.00 %
```

> **Fake-hardware limitation** — mock hardware does not integrate effort
> commands, so the arm does not move and the observation stays frozen. This
> test validates the *command chain* (nodes, topics, rates, controller
> activation, gating), not closed-loop behaviour. Closed-loop policy + CBF
> behaviour is validated in MuJoCo via `franka_sim.scripts.evaluate_policy`.

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
