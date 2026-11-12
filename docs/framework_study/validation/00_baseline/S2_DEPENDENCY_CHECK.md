# 00_baseline — S2: container start-up and read-only dependency verification

> Approved by the user on 2026-11-12 (S2 only). Executed: `docker start franka_ros2` and read-only
> probes (`docker exec` with `PYTHONDONTWRITEBYTECODE=1`, cwd `/tmp`; `colcon list/graph` with
> `--log-base /dev/null`). **Not executed:** any build, clean, pytest, Gazebo or ROS node. No source,
> configuration, build/install/log file was changed (`git status` identical before and after; the S1
> evidence re-verified: `sha256sum -c` 22/22 OK, still write-protected).

## 1. Start-up

| Item | Result |
|---|---|
| `docker start franka_ros2` | **running** since 2026-11-12 10:56:15 UTC; only process `/bin/bash` |
| Entrypoint `vcs import … --skip-existing` | ran 10:56:19–21 UTC; skipped `franka_description` and `libfranka` (both exist); wrote nothing |
| Repository in the container | bind mount `/home/mattia/Git/franka_ros2 → /ros2_ws/src` (rw); container Git sees `humble-mattia` `f7089911decf`; a sampled file hash equals the host's |
| Container disk | overlay, 285 GB free; `/ros2_ws/build` 1.1 GB, `install` 18 MB, `log` 82 MB |

A false alarm during start-up, recorded for completeness: two "new" untracked paths
(`meshes.zip`, `robots/fr3/fr3.urdfros2`) printed by a combined command belong to the **gitignored
`franka_description` checkout**, where they have existed since 2026-05-08 (the `.urdfros2` file is
empty). Nothing appeared in the repository.

## 2. Dependencies (all versions observed in the container)

| Area | Observed | Expected / used by the source | Verdict |
|---|---|---|---|
| ROS 2 | Humble (`ROS_DISTRO=humble`), default RMW, Fast DDS profile `/ros2_ws/src/fastdds_no_shm.xml` present; `GZ_VERSION=fortress` | Humble, Fortress | OK |
| Python | 3.10.12 | 3.10 (install layout `lib/python3.10`) | OK |
| numpy | **1.26.4** (user site) | 1.26.4 (Dockerfile pin) | OK |
| scipy | 1.15.3 | unpinned | OK |
| osqp | **0.6.7.post3** | `>=0.6.2,<1.0` | OK |
| qpsolvers | **4.3.3**, `available_solvers == ['osqp']` | 4.3.3; build-time assertion | OK |
| Pinocchio (Python) | 4.0.0 from `/opt/ros/humble` (`ros-humble-pinocchio`), with coal 3.0.3 | Pinocchio (the Dockerfile also installs robotpkg 4.1.0) | OK, see O-10 |
| Other Python used by the in-scope source | PyYAML 5.4.1, OpenCV 4.11.0, onnxruntime 1.23.2, torch 2.13.0+cu130, mujoco 3.4.0, trimesh 5.1.1, protobuf 4.25.9, matplotlib, psutil, mediapipe, pytest 6.2.5 (apt) | — | OK |
| ROS Python/message packages imported by the in-scope source | every one of the 45 third-party top-level modules imported by `franka_experiments`, `franka_simulation_experiments` and `franka_simulation` (scripts + launch) imports; the 6 that do not are repository-local (`approach_probe`, `ball_closed_loop`, `ball_throw_eval`, `range_noise_calibration`, `smoothness_report` in `franka_experiments/scripts/`, resolved by path) and the workspace package `franka_msgs` (from the install) | — | **no missing dependency** |
| Gazebo | Ignition Gazebo **6.18.0** (`libignition-gazebo6`), `ign` CLI | `ignition-gazebo6` (Fortress) | OK |
| ros_gz | 0.244.26 (`ros_gz_sim`, `ros_gz_bridge` in `/opt/ros/humble`) | `exec_depend` | OK |
| ros2_control | ros2-control 2.54.3, ros2-controllers 2.54.1, controller_manager 2.54.3, joint_state_broadcaster, robot_state_publisher in `/opt/ros/humble` | `exec_depend` | OK |
| Franka (workspace) | `franka_description` 1.3.0 (gitignored checkout `1ccde30`), `libfranka` 0.15.0 (`155ddac`), `franka_hardware` in `install/` | `franka.repos` pins 1.3.0 / 0.15.0; `find_package(Franka 0.13.3)` | OK |
| C++ tool chain | gcc/g++ 11.4.0, CMake 3.22.1 | — | OK |
| colcon | `/usr/bin/colcon`, `python3-colcon-common-extensions` 0.3.0 | — | OK |
| vcstool | 0.3.0 | entrypoint | OK |

`tf_transformations` is not installed, and no in-scope file imports it.

## 3. Workspace and source

| Check | Result |
|---|---|
| Branch / HEAD / baseline | `humble-mattia` / `f708991` / `ba68077` (check_state consistent; 0 non-documentation changes) |
| Package resolution (`ros2 pkg prefix`) | the six in-scope packages and `franka_description`, `franka_hardware` → `/ros2_ws/install/<pkg>`; ros_gz, controller_manager, JSB, RSP → `/opt/ros/humble` |
| Controller plugins | ament index `controller_interface__pluginlib__plugin/franka_rt_controllers` → `franka_rt_controllers.xml` (`RtTorqueController`, …) and `rt_velocity_blender_controller.xml`; hardware plugin index → `ign_hardware_plugins.xml` |
| Gazebo system plugins | `IGN_GAZEBO_SYSTEM_PLUGIN_PATH` is empty in the shell; `gz_fr3_plant.launch.py` L95–102 sets it to itself + `LD_LIBRARY_PATH`, which contains `install/<pkg>/lib`. `libfranka_scenario_obstacles.so` and `libfranka_ign_ros2_control-system.so` are symlinks into `build/` today |
| Plugin link dependencies (`ldd`) | `libfranka_rt_controllers.so` → `install/franka_msgs` typesupport; `libcbf_torque_controller.so` → Pinocchio 4.0.0 + coal 3.0.3 from `/opt/ros/humble`; `libfranka_ign_ros2_control-system.so` → ignition-gazebo6, kdl_parser, orocos-kdl; `libfranka_scenario_obstacles.so` → ignition-gazebo6. No unresolved library |
| Cached CMake settings | `CMAKE_BUILD_TYPE=Release` in all three C++ build dirs; `pinocchio_DIR=/opt/ros/humble/lib/x86_64-linux-gnu/cmake/pinocchio` |
| Python import resolution (install sourced) | `franka_experiments` → **install** from `/tmp`, `/ros2_ws/src` and `/ros2_ws/src/franka_simulation_experiments`; → **source** only from `/ros2_ws/src/franka_experiments` (F-7-5 confirmed; see O-9). `franka_msgs` → install; its `LinkDistance` has `shadow_valid` |
| F-7-4 / F-6-3 | **unchanged**. `check_install_provenance.py` 3/3 PASS (best match `42df80e`, mixed, stale files listed). `check_installed.py`: `fr3_control.yaml`, `fr3_complete.yaml`, `conditions.yaml`, `scenarios.yaml` DIFFER; sim programs and `simexp` 27/27 same; `cbf_safety_filter`, `real_time_distance`, `qddot_to_torque` DIFFER → `INSTALLED != SOURCE`. Nothing was removed |

### Dependency graph (colcon, `--base-paths src`)

Topological order of the six: `franka_msgs` → `franka_ign_ros2_control` → `franka_rt_controllers` →
`franka_simulation` → `franka_experiments` → `franka_simulation_experiments`.
Workspace packages they need but that are **not** rebuilt: `franka_description` (symlink install of the
gitignored checkout), `libfranka` and `franka_hardware` (build dependencies of
`franka_ign_ros2_control`), `franka_semantic_components` (of `franka_rt_controllers`),
`franka_fr3_moveit_config`, `franka_bringup` (exec dependencies; `franka_bringup` is skipped in
simulation: `torque_control_stack.launch.py` L515–523, `external_plant:=true`).
Packages above `franka_msgs`: 15, of which four non-rebuilt ones **link** its libraries:
`franka_hardware`, `franka_gripper`, `franka_robot_state_broadcaster`, `franka_example_controllers`.
None of them is loaded by the Gazebo pipeline (Derived from the launch code above and from `ldd` of
the sim plugins).

## 4. Observations added in S2 (observations, not findings)

- **O-9** `PYTHONPATH` in the container ends with `:` (Dockerfile ENV + ROS setup), so the current
  directory is on `sys.path` for every Python process. Together with F-7-5 this makes the import
  source depend on the cwd; from `src/franka_simulation_experiments`, `franka_experiments` comes from
  the install (harmless once install == source).
- **O-10** Two Pinocchio installations: `ros-humble-pinocchio` 4.0.0 (first on `PYTHONPATH` and
  `LD_LIBRARY_PATH`, used by Python and by the existing `libcbf_torque_controller.so`) and robotpkg
  4.1.0 in `/opt/openrobots` (installed by the Dockerfile, not used). A rebuild must find 4.0.0 again.
- **O-11** Four non-rebuilt packages link `franka_msgs` (list above). Rebuilding `franka_msgs` is safe
  for them only if the regenerated message definitions are identical (to be checked after S3).
- **O-12** `franka_description` checkout: untracked `meshes.zip` and empty `robots/fr3/fr3.urdfros2`
  (2026-05-08); not installed (symlink install of the tracked files), not used.
- **O-13** `tf_transformations` absent; unused by the in-scope source.

No missing dependency and no version conflict that affects the in-scope code was found.

## 5. Recommendation for S3 (not executed)

See the S2 report in CHECKPOINTS C-054 and the procedure in `S3_BUILD_PLAN.md`.
