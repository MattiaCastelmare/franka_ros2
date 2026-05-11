# Implementation Log — franka_simulation Refactoring

> Started: 2026-05-09  
> Branch: `humble-mattia`  
> Source of truth: `refactoring_code.md`

---

## Phase 1 — Bug Fixes in move_group.launch.py

### 1.1 Remove duplicate `velocity_control_blender` node (no params, dead code)
- **Lines**: 580–585 removed (bare `Node(velocity_control_blender)` without params)
- **Reason**: `delayed_blender` at line 607+ already instantiates the correct version with `velocity_blender_params_file` and `avoidance_params_file`. The bare node would have started a second instance with default params, conflicting.
- **Status**: ✅ DONE

### 1.2 Remove unused `motion_server_action` OpaqueFunction
- **Line**: 598 removed (`OpaqueFunction(function=lambda context: [motion_server])`)
- **Reason**: Already commented out in LaunchDescription (line 675). Completely dead code.
- **Status**: ✅ DONE

---

## Phase 2 — sim_velocity.launch.py

- **Created**: `launch/sim_velocity.launch.py`
- **Pipeline**: Gazebo + RSP + joint_state_broadcaster + fr3_velocity_controller + clock_bridge + RViz
- **No CBF**, no MoveIt — minimal clean velocity control for external commanders
- **Status**: ✅ DONE

---

## Phase 3 — Torque Pipeline

### 3.1 Add `fr3_effort_controller` to controllers.yaml
- **Status**: ✅ DONE

### 3.2 Create sim_torque.launch.py
- **Pipeline**: Gazebo (`gazebo_effort:=true`) + RSP + joint_state_broadcaster + fr3_effort_controller + clock_bridge + RViz
- **Status**: ✅ DONE

---

## Phase 4 — Acceleration Pipeline

### 4.1 Create `scripts/sim_acceleration_bridge.py`
- **Node**: integrates q̈_cmd → q̇, forwards to fr3_velocity_controller
- **Status**: ✅ DONE

### 4.2 Create `launch/sim_acceleration.launch.py`
- **Pipeline**: sim_velocity pipeline + sim_acceleration_bridge node
- **Status**: ✅ DONE

---

## Phase 5 — Rename + Cleanup

### 5.1 Rename `franka_simulation.launch.py` → `sim_position.launch.py`
- **Status**: ✅ DONE

### 5.2 Delete dead source files
- Deleted: `src/real_time_distance.py` (duplicate of franka_rt_controllers version)
- Deleted: `src/utils.py` (never imported, orphan)
- Deleted: `test/prova.py` (scratch/debug file)
- **Status**: ✅ DONE

---

## Phase 6 — Test Infrastructure

### 6.1 franka_simulation/test/

- **Created**: `test/scripts/test_velocity_publisher.py` — sinusoidal velocity publisher for velocity pipeline
- **Created**: `test/scripts/test_acceleration_publisher.py` — sinusoidal acceleration publisher for acceleration pipeline
- **Created**: `test/scripts/test_torque_publisher.py` — phased torque publisher (zeros then sinusoidal) for torque pipeline
- **Created**: `test/scripts/check_pipeline.sh` — bash validator for all four pipelines
- **Created**: `test/launch/test_velocity_pipeline.launch.py` — sim_velocity + auto test publisher
- **Created**: `test/launch/test_acceleration_pipeline.launch.py` — sim_acceleration + auto test publisher
- **Created**: `test/launch/test_torque_pipeline.launch.py` — sim_torque + auto test publisher
- **Created**: `test/launch/test_cbf_pipeline.launch.py` — move_group CBF pipeline wrapper
- **Created**: `test/config/test_publishers.yaml` — conservative test parameters
- **Created**: `test/README.md` — full documentation with commands and expected results
- **Modified**: `CMakeLists.txt` — install test scripts as executables + install test/launch and test/config

### 6.2 franka_experiments/test/

- **Created**: `test/launch/test_velocity_fake.launch.py` — velocity pipeline with fake hardware
- **Created**: `test/launch/test_torque_fake.launch.py` — torque pipeline with fake hardware
- **Created**: `test/launch/test_oscbf_fake.launch.py` — OSCBF pipeline (Phase 1/2/3) with fake hardware
- **Created**: `test/scripts/check_topics.sh` — topic validator for all experiment pipelines
- **Created**: `test/config/test_defaults.yaml` — test parameters (fake hw, namespace, commander params)
- **Created**: `test/README.md` — full documentation with per-pipeline instructions
- **Modified**: `setup.py` — install test/launch and test/config in ament share

- **Status**: ✅ DONE

---

## Summary of All Changes

| Action | File | Notes |
|--------|------|-------|
| Modified | `launch/move_group.launch.py` | Removed duplicate blender node + dead OpaqueFunction |
| Created | `launch/sim_position.launch.py` | Renamed from franka_simulation.launch.py |
| Deleted | `launch/franka_simulation.launch.py` | Replaced by sim_position.launch.py |
| Created | `launch/sim_velocity.launch.py` | New velocity pipeline |
| Created | `launch/sim_torque.launch.py` | New torque pipeline |
| Created | `launch/sim_acceleration.launch.py` | New acceleration pipeline |
| Created | `scripts/sim_acceleration_bridge.py` | q̈ → q̇ integrator node |
| Modified | `config/controllers.yaml` | Added fr3_effort_controller |
| Deleted | `src/real_time_distance.py` | Dead duplicate |
| Deleted | `src/utils.py` | Dead orphan |
| Deleted | `test/prova.py` | Scratch file |
| Created | `test/scripts/test_velocity_publisher.py` | Velocity test publisher |
| Created | `test/scripts/test_acceleration_publisher.py` | Acceleration test publisher |
| Created | `test/scripts/test_torque_publisher.py` | Torque test publisher |
| Created | `test/scripts/check_pipeline.sh` | Pipeline validation script |
| Created | `test/launch/test_velocity_pipeline.launch.py` | Automated velocity test |
| Created | `test/launch/test_acceleration_pipeline.launch.py` | Automated acceleration test |
| Created | `test/launch/test_torque_pipeline.launch.py` | Automated torque test |
| Created | `test/launch/test_cbf_pipeline.launch.py` | CBF pipeline test wrapper |
| Created | `test/config/test_publishers.yaml` | Test publisher parameters |
| Created | `test/README.md` | Test documentation |
| Modified | `CMakeLists.txt` | Install test scripts + launch + config |
