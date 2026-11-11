# Dependencies of the two packages — Stage 0

> **Evidence level.** Every row below is either **Verified** (read in `package.xml`, `setup.py`,
> `CMakeLists.txt` or an import statement, with the file named) or explicitly marked **Inferred**.
> Runtime behaviour (does the import actually succeed in the container? which version is installed?)
> is **Unverified** at Stage 0. Generated companion tables: [import_graph.md](import_graph.md)
> (per-module), [launch_inventory.md](launch_inventory.md) (launch-time packages).

## 1. Declared ROS dependencies (Verified: `package.xml`)

| Package | `franka_experiments` (`ament_python`) | `franka_simulation_experiments` (`ament_cmake`) |
|---|---|---|
| Build type | `ament_python` (`setup.py` installs `franka_experiments`, `.nodes`, `.utils`; `launch/`, `config/*.yaml|*.rviz`, `scripts/*.py|*.sh`, `test/launch`, `test/config` to `share/`) | `ament_cmake`; `CMakeLists.txt` installs `launch/`, `config/*.yaml`, every `scripts/*.py` **without the `.py` suffix** into `lib/franka_simulation_experiments/`, and `scripts/simexp/` as a package in the same directory |
| ROS core | `launch`, `launch_ros`, `ament_index_python`, `rclpy`, `std_msgs`, `std_srvs`, `sensor_msgs`, `geometry_msgs`, `visualization_msgs`, `tf2_ros`, `trajectory_msgs`, `cv_bridge` | `rclpy`, `launch`, `launch_ros`, `launch_testing`, `sensor_msgs`, `std_msgs`, `geometry_msgs`, `controller_manager`, `controller_manager_msgs`, `joint_state_broadcaster`, `joint_state_publisher`, `robot_state_publisher`, `ros_gz_sim`, `ros_gz_bridge` |
| Franka workspace | `franka_msgs`, `franka_description`, `franka_simulation`, `franka_bringup`, `franka_fr3_moveit_config` | `franka_experiments`, `franka_simulation`, `franka_description`, `franka_msgs`, `franka_rt_controllers`, `franka_ign_ros2_control` |
| Other | `moveit_msgs`, `pymoveit2` | — |
| Commented out | `realsense2_camera`, `apriltag_ros` (still used by the calibration launch files, see §3) | — |
| Test | `python3-pytest` (`extras_require={'test': ['pytest']}` so `colcon test` runs pytest) | `python3-pytest` (`pytest.ini`) |

**Observation (Verified, to be discussed in Stage 7):** `franka_experiments/package.xml` does **not**
declare `franka_rt_controllers`, although the default torque pipeline ends in its C++ controller
and `utils/launch_support.py` exports `generate_rt_controllers_yaml`. The dependency is reached only
through launch-time `controller_manager` spawning.

## 2. Dependency direction between the two packages (Verified: imports)

```
franka_simulation_experiments ──imports──▶ franka_experiments (library + scripts)
           │                                         │
           └──imports at runtime──▶ franka_simulation/scripts/utils ◀──sys.path shim──┘ (capsule_overlay only)
```

- `franka_simulation_experiments` → `franka_experiments` **library** (normal Python imports from the
  installed package): `utils.cbf_diagnostics`, `utils.cbf_hard_limits`, `utils.cbf_state_rows`,
  `utils.config`, `utils.distance_engine`, `utils.distance_utils`, `utils.kinematics`,
  `utils.launch_support`, `utils.perception_msgs`, `utils.trajectory_trace`.
- `franka_simulation_experiments` → `franka_experiments/scripts/*.py` **by source path**:
  `simexp/emulation.py` inserts `../../../franka_experiments/scripts` into `sys.path` to import
  `cbf_scenarios`; `simexp/estimator_metrics.py` does the same for `compare_vobs`. This works only
  from a source checkout laid out as `src/<pkg>`, which is an **undocumented assumption** recorded for Stage 7.
- `franka_experiments` never imports `franka_simulation_experiments` (Verified: no `simexp` import in
  `franka_experiments/`). The dependency is one-directional.

## 3. Runtime and launch-time dependencies on other workspace packages

| Dependency | How it is reached | Where (Verified) | Doc depth |
|---|---|---|---|
| `franka_simulation/scripts/utils/*` (installed as `lib/franka_simulation/utils`) | `sys.path.insert` + `from utils.X import …` | `utils/simulation_imports.py` (re-exports `avoidance_math`, `closest_constraint`, `ros_publishers`, `ros_setup`, `avoidance_core`, `rviz_markers`; source marker says single consumer `capsule_overlay_node`); `simexp/sim_config.py::franka_simulation_utils` (`gz_world`, `gz_robot_description`) | executed parts in full (Stage 5/6) |
| `franka_simulation/launch/gz_fr3_plant.launch.py` | `IncludeLaunchDescription` | `franka_simulation_experiments/launch/gz_plant.launch.py` | full (Stage 6) |
| `franka_simulation` `image_publisher` executable | `Node(...)` | `franka_experiments/launch/minimal.launch.py` | interface |
| `franka_rt_controllers` plugins `RtTorqueController` (`rt_torque_controller.cpp`), `CBFTorqueController` (`cbf_torque_controller.cpp`), `RtVelocityExecutorController` (**in `rt_velocity_blender_controller.cpp`**, spawned under the name `rt_velocity_executor_controller`) | `controller_manager` spawner | `torque_control_stack.launch.py`, `velocity_cbf_control_stack.launch.py`, `minimal.launch.py`, `thales.launch.py`, calibration launch files; YAML generated by `utils/launch_support.py::generate_rt_controllers_yaml` (type string `franka_rt_controllers/RtVelocityExecutorController`, L182) | full (Stage 4/5) |
| `franka_msgs` messages `MultiLinkDistance`, `LinkDistance`, `MultiDistance`, `HumanRobotDistance` | Python import | 27 files incl. tests (see import graph) | interface + field semantics (Stage 1/2) |
| `franka_rl/` (on disk) = `franka_sim` (in code) | directory search from `__file__` | `utils/rl_policy.py::find_sim_root`; policy checkpoints/ONNX | interface only |
| `apriltag_ros`, `realsense2_camera`, `rviz2`, `rqt_image_view`, `tf2_ros static_transform_publisher` | `Node(...)` / include | calibration and `minimal` launch files | interface only |
| `franka_bringup`, `franka_description`, `franka_fr3_moveit_config` | URDF/xacro and robot bringup | launch files (Stage 1) | interface only |

## 4. Third-party Python libraries (Verified: imports; versions Unverified)

| Library | Importing files (incl. tests) | Role (Inferred from module names; confirmed in later stages) |
|---|---:|---|
| `numpy` | 220 | everywhere |
| `pinocchio` | 32 | kinematics, Jacobians, dynamics (`utils/kinematics.py` and consumers) |
| `scipy` | 21 | `cKDTree` distance queries, rotations, calibration solvers |
| `yaml` | 56 | configuration loading |
| `cv2`, `cv_bridge` | 12, 14 | depth/colour images, calibration |
| `trimesh` | 7 | link meshes (self-masking / self-shadow) |
| `qpsolvers`, `osqp` | 5, 5 | QP solving; version lock `qpsolvers==4.3.3` + `osqp<1.0` per project notes (Unverified here) |
| `onnxruntime` | 2 | RL policy inference |
| `matplotlib`, `psutil`, `rosbag2_py` | 6, 1, 1 | offline scripts |

## 5. Message and interface types seen by the extractor

`sensor_msgs`, `std_msgs`, `geometry_msgs`, `visualization_msgs`, `builtin_interfaces`,
`rcl_interfaces`, `rosgraph_msgs`, `tf2_msgs`, `controller_manager_msgs`, `apriltag_msgs`,
`realsense2_camera_msgs`, `moveit_msgs`, `std_srvs`, `franka_msgs`. The resolution of each
topic to its type, producer, consumers, rate and QoS is the deliverable of chapter 02 (Stage 1).
