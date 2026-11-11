# 14 — Build, launch and test

> Stage 7, 2026-11-11. Baseline `ba68077`; documentation HEAD `476109b`. Labels as in
> [00_INDEX.md](00_INDEX.md) §2. Commands carry one of three tags:
> - **[EXECUTED 2026-11-11]**: run in this stage, with its output quoted;
> - **[READ-ONLY PROBE]**: run in this stage, and inspected state without changing it;
> - **[NOT EXECUTED]**: written from the source and documentation, never run here. Several of these
>   (`colcon build`, `pytest`, `ros2 launch`) need the user's approval under CLAUDE.md.

---

## 1. Intuition — the code you read is not necessarily the code that runs

Every earlier chapter documents the **source** at `ba68077`. A robot, a Gazebo cell or a test
executes something else: a **build** of some source, copied into an **install tree** inside a
container. Three translations separate what was read from what runs:
- colcon turns the source into an install tree;
- the shell's `source install/setup.bash` and the current directory decide which copy Python
  imports;
- launch files read configuration from the install tree, not from the source.

On 2026-11-11 each of these translations was wrong in a way that no error message reveals:
- the install tree was a mixture, built mostly from another branch (F-7-4);
- a test run from the repository root imports that mixture (F-7-5);
- the configuration a simulation would read had `d_safe` = 0.15 m, not the documented 0.10 m
  (F-6-3).

A refactor validated through any of these paths validates the wrong code. This chapter makes the
three translations explicit, shows how to detect each one, and gives a procedure that ties a run to
a Git revision. There is no mathematics in this chapter; the template's part 2 does not apply.

---

## 2. Workspace structure

The repository **is** the `src/` directory of a colcon workspace. `docker-compose.yml` bind-mounts
the repository root at `/ros2_ws/src`; `build/`, `install/` and `log/` live in `/ros2_ws`, inside
the container's writable layer (Verified, `docker-compose.yml` `volumes`; `mount` probe).

**Consequence (Derived).** The install tree is not versioned, not shared with the host, and lost
when the container is recreated. Nothing records which source it was built from (§9).

### 2.1 Packages

17 directories hold a `package.xml` (Verified, `ls */package.xml`). The documented scope (00_INDEX §1)
covers two of them, plus the parts of three dependencies that execute:

| Package | Build type | Role for this study |
|---|---|---|
| `franka_experiments` | ament_python | **target**: nodes, utils, launch, config, scripts, tests |
| `franka_simulation_experiments` | ament_cmake (Python programs) | **target**: Gazebo harness, `simexp`, sim tests |
| `franka_rt_controllers` | ament_cmake (C++) | dependency in scope: `RtTorqueController`, `CBFTorqueController`, `RtVelocityExecutorController` (ch. 09, 10) |
| `franka_simulation` | ament_cmake (C++ + Python) | dependency in scope: `gz_fr3_plant.launch.py`, the `ScenarioObstacles` plugin, `depth_sim_adapter`, `gz_world`/`gz_robot_description` (ch. 12) |
| `franka_msgs` | ament_cmake (rosidl) | dependency in scope: `MultiDistance`, `MultiLinkDistance`, `LinkDistance`, `HumanRobotDistance` (ch. 02) |
| `franka_bringup`, `franka_hardware`, `franka_description`, `franka_gripper`, `franka_fr3_moveit_config`, `franka_robot_state_broadcaster`, `franka_semantic_components`, `franka_example_controllers`, `franka_ros2`, `franka_gazebo/*` | ament_cmake | upstream Franka stack; interface-level only |
| `fr3_student_sim`, `integration_launch_testing`, `libfranka` | ament_python / ament_cmake / cmake | out of scope |

Also outside the scope: `franka_rl/` has no `package.xml` and is not a ROS package (F-0-10).
`franka.repos` pins `franka_description` 1.3.0 and `libfranka` 0.15.0. `franka_entrypoint.sh` runs
`vcs import … --skip-existing` at every container start, so a checked-in copy wins over the pinned
version (Verified, both files).

### 2.2 Declared dependencies

- **`franka_experiments/package.xml`** (Verified):
  - `exec_depend`: launch, launch_ros, ament_index_python, rclpy, std/std_srvs/sensor/geometry/
    visualization/trajectory/moveit msgs, tf2_ros, cv_bridge, franka_msgs, pymoveit2,
    franka_fr3_moveit_config, franka_description, franka_simulation, franka_bringup;
  - `realsense2_camera` and `apriltag_ros` are commented out;
  - Python packages (numpy, Pinocchio, qpsolvers, OSQP, PyYAML, OpenCV, onnxruntime) are not
    declared; they come from the Dockerfile (§3);
  - `franka_rt_controllers` is **not** declared, although the torque and velocity stacks spawn its
    plugins (F-0-6).
- **`franka_simulation_experiments/package.xml`** (Verified): `exec_depend` on franka_experiments,
  franka_simulation, franka_description, franka_msgs, franka_rt_controllers,
  franka_ign_ros2_control, controller_manager(+msgs), the joint-state broadcaster and publisher,
  robot_state_publisher, ros_gz_sim, ros_gz_bridge, rclpy, launch(+ros, +testing), and the
  sensor/std/geometry messages.
- **Undeclared coupling** (F-0-5, Verified): `simexp/emulation.py` and `simexp/estimator_metrics.py`
  import `franka_experiments/scripts/{cbf_scenarios,compare_vobs}.py` by a **relative source path**.
  That works only in the `src/<pkg>` layout, never from a pure install.

---

## 3. Docker environment

| Item | Value (Verified, `Dockerfile` / `docker-compose.yml`) |
|---|---|
| Base image | `ros:humble-ros-base` (Ubuntu 22.04, Python 3.10) |
| apt | ros2_control stack, MoveIt, ros_gz (Fortress, `GZ_VERSION=fortress`), `libignition-gazebo6-dev`, realsense2, apriltag_ros, `python3-colcon-common-extensions`, `ros-humble-ros-testing` |
| Pinocchio | `robotpkg-py310-pinocchio` from robotpkg; `PYTHONPATH`, `LD_LIBRARY_PATH` and `PATH` prefixed with `/opt/openrobots` |
| pip (user) | `numpy==1.26.4`, `opencv-contrib-python==4.11.0.86`, `mediapipe==0.10.20`, `qpsolvers[osqp]==4.3.3`, `osqp>=0.6.2,<1.0`, `torch`, `mujoco==3.4.0`, `gymnasium==0.29.1`, `stable-baselines3`, `onnx`, `onnxruntime`, `protobuf>=4.25.3,<5`, `trimesh` |
| Build-time assertion | `'osqp' in qpsolvers.available_solvers`, or the image build fails. The velocity and OSCBF filters would otherwise raise `SolverNotFound` at run time |
| Source in the image | none: `vcs import` + `rosdep install` run against a temporary `src`, which is then deleted. **The image contains no build of this repository** |
| Runtime | `network_mode: host`, `privileged`, `gpus: all`, memory limit 20 GiB, `SYS_NICE`, `rtprio 99`, `memlock -1`, `FASTRTPS_DEFAULT_PROFILES_FILE=/ros2_ws/src/fastdds_no_shm.xml` |

**Observed in the running container** [READ-ONLY PROBE]: numpy 1.26.4, scipy 1.15.3, qpsolvers
4.3.3, osqp 0.6.7.post3, Pinocchio 4.0.0, pytest 6.2.5 (as `python3 -m pytest`; no `pytest` script
on `PATH`). Container `franka_ros2`, up 7 days on 2026-11-11.

---

## 4. How colcon installs each package

### 4.1 `franka_experiments` (`setup.py`, ament_python) — Verified

| Installed to | From | Mechanism |
|---|---|---|
| `lib/python3.10/site-packages/franka_experiments/{,nodes,utils}` | the Python package | `packages=[…]` |
| `lib/franka_experiments/<name>` | 22 `console_scripts` | entry points; `setup.cfg` `install_scripts=$base/lib/franka_experiments` makes them `ros2 run`-able |
| `share/franka_experiments/launch/` | `launch/*.launch.py` | `data_files` glob |
| `share/franka_experiments/config/` | `config/*.yaml`, `config/*.rviz` (top level only, so `config/safety/*.md` is **not** installed) | `data_files` glob |
| `share/franka_experiments/scripts/` | `scripts/*.py`, `scripts/*.sh` | `data_files` glob |
| `share/franka_experiments/test/{launch,config}/` | `test/launch/*.launch.py`, `test/config/*.yaml` | `data_files` glob |

Two consequences (Derived):
- a glob is evaluated **at build time**, so a file added later is absent until the next build;
- in copy mode (`symlink_install=False`) every installed file is a snapshot.

### 4.2 `franka_simulation_experiments` (`CMakeLists.txt`, ament_cmake) — Verified

| Installed to | From |
|---|---|
| `share/franka_simulation_experiments/launch/` | `install(DIRECTORY launch/)` |
| `share/franka_simulation_experiments/config/` | `install(DIRECTORY config/ FILES_MATCHING PATTERN "*.yaml")`, so `conditions.d/*.yaml` is included |
| `lib/franka_simulation_experiments/<stem>` | every `scripts/*.py`, via `install(PROGRAMS … RENAME <stem>)`: the `.py` extension is dropped |
| `lib/franka_simulation_experiments/simexp/` | `install(DIRECTORY scripts/simexp … PATTERN "__pycache__" EXCLUDE)` |

`file(GLOB …)` is evaluated **at configure time**, so a new script needs a re-configure
(`--cmake-force-configure`), not just a rebuild (Derived; the container's build log shows this flag
in use).

### 4.3 Copy mode versus symlink mode

| | `--symlink-install` | default (copy) |
|---|---|---|
| ament_python modules | the egg-link points at the source | copied into `site-packages` |
| `data_files` / `install(DIRECTORY)` | symlinks into the source | copies |
| compiled C++ (`.so`) | symlink to `build/` | copied from `build/` |
| A source edit is visible | at once for Python and config (C++ still needs a build) | only after the next build |
| A file deleted from the source | its dangling symlink stays | its copy **stays** (an install never deletes) |

Python resolution and data-file symlinking are Inferred from colcon-ros behaviour. The `.so`
symlink of `franka_simulation` is Verified [READ-ONLY PROBE].

**Observed** [READ-ONLY PROBE] in `/ros2_ws/log/build_*/logger_all.log`: the same packages were
built in **both** modes at different times. Examples:
- `--symlink-install --packages-select franka_experiments` (7 times);
- `--packages-select franka_experiments franka_simulation_experiments` without symlinks (the last
  build, 2026-11-10 14:42 UTC).

The install mode of each package is therefore whatever its most recent build used. Today
`franka_simulation`'s scripts are symlinks (from 2026-10-29), while the two target packages are
copies (F-7-4).

### 4.4 Where the running code reads its configuration (Verified, earlier chapters)

| Reader | Path it reads |
|---|---|
| `utils/config.py::load_package_config` (all nodes; ch. 03) | `get_package_share_directory('franka_experiments')/config/<file>`: the **install** |
| the torque stack's generated YAML (`launch_support.write_temp_yaml`; ch. 03, 09) | built at launch time from the installed `config/*.yaml` and `launch_defaults.yaml` |
| `simexp.sim_config.default_paths` (ch. 12, F-6-3) | `share/franka_simulation_experiments/config/` and `share/franka_experiments/config/` |
| the Stage 2–7 verification scripts | the **source**, with a guard that refuses an installed copy (F-2-2) |

Editing a YAML file in the source therefore changes nothing at run time until the package is
rebuilt in copy mode (lead "colcon copies config/", Verified as the mechanism of F-6-3).

---

## 5. Source versus install: the F-6-3 / F-7-4 discrepancy

### 5.1 Why it occurs — the reconstructed sequence (Verified, [READ-ONLY PROBE] + `git reflog`)

| Time (+0100) | Event | Evidence |
|---|---|---|
| 2026-11-09 19:17 | `42df80e` "tracking period reduced to 4s" committed on `fr3-student-sim` | `git log` |
| 2026-11-10 15:37:30 | `ba68077` committed on `humble-mattia`: **the documented baseline** | `git log` |
| 2026-11-10 15:39:00 | `checkout: moving from humble-mattia to fr3-student-sim`; then `pull --ff-only` → `42df80e` | `git reflog` |
| 2026-11-10 15:39:00 | mtime of the installed `fr3_control.yaml` (14:39:00 UTC): the file was written by that checkout, and the copy-mode install preserved its mtime | `stat` in the container |
| 2026-11-10 15:42 | `colcon build --packages-select franka_experiments franka_simulation_experiments`, `symlink_install=False` | `log/latest_build/logger_all.log` |
| 2026-11-10 17:12:54 | `checkout: moving from fr3-student-sim to humble-mattia` (back to `ba68077`) | `git reflog`; the source `fr3_control.yaml` mtime 16:12:54 UTC |
| since then | no build | `log/latest_build` unchanged |

The two branches have diverged: `ba68077` is not an ancestor of `42df80e`, and they are 34 and 15
commits apart. `tools/stage7_verification/check_install_provenance.py` [EXECUTED 2026-11-11]
compares all 156 installed files that map to a source path:

| Revision | Byte-identical | Differ | Absent from that revision |
|---|---:|---:|---:|
| `42df80e` (`fr3-student-sim`) | **141** | 1 | 14 |
| `ba68077` (baseline) | 126 | 27 | 3 |
| other branch tips | ≤ 116 | | |

The 14 files absent from `42df80e` were installed by earlier builds of `humble-mattia` (Nov 1–10)
and never removed. Among them:
- `static_scene.py`, `surface_velocity.py`, `go_around.py`, `run_metadata.py`, `cbf_diagnostics.py`;
- a pre-commit `self_shadow.py`;
- `fr3_control.yaml.orig` and `.bak_baseline`.

So the install is **neither** revision (F-7-4). The 27 baseline differences include
`cbf_safety_filter.py`, `cbf_state_rows.py`, `fr3_control.yaml` (`d_safe` 0.15 vs 0.10),
`conditions.yaml`, `scenarios.yaml`, `torque_control_stack.launch.py` and `real_time_distance.py`.

The general mechanism (Derived):
1. one container, one install tree, several branches checked out in the same bind-mounted source;
2. builds in copy mode snapshot whatever is checked out;
3. installs only add or overwrite;
4. nothing stamps the install with the revision it came from.

### 5.2 How to detect it

| Check | What it tells you | Tag |
|---|---|---|
| `python3 -I docs/framework_study/tools/stage7_verification/check_install_provenance.py` | which revision each installed file equals; mixed trees; stale leftovers | [EXECUTED 2026-11-11]: 3/3 PASS, best match `42df80e`, mixed |
| `docs/framework_study/tools/stage6_verification/run_all.sh` → `check_installed.py` | byte-compare of the installed configs, programs and stack nodes with the **current** source | [EXECUTED 2026-11-11]: `INSTALLED != SOURCE` |
| `docker exec franka_ros2 grep -h "Command line arguments" /ros2_ws/log/latest_build/logger_all.log` | the last build's packages and `symlink_install` flag | [READ-ONLY PROBE] |
| `docker exec franka_ros2 stat -c '%y %n' /ros2_ws/install/franka_experiments/share/franka_experiments/config/fr3_control.yaml` | the mtime of the installed copy, to compare with `git reflog --date=iso` | [READ-ONLY PROBE] |
| `cd /ros2_ws/src/franka_experiments && python3 -c "import franka_experiments; print(franka_experiments.__file__)"` | which copy Python imports from the cwd you will test from (§7.3) | [READ-ONLY PROBE] |

### 5.3 How to guarantee that a run corresponds to a Git revision — proposed procedure [NOT EXECUTED]

Every step below is a proposal. `colcon build` needs the user's approval (CLAUDE.md), and none of it
was run in Stage 7.

```bash
# On the host — the source must be exactly the revision under test
cd ~/Git/franka_ros2
git status --porcelain -- franka_experiments franka_simulation_experiments \
    franka_rt_controllers franka_simulation franka_msgs     # must print nothing
REV=$(git rev-parse HEAD)

# In the container — a CLEAN build, so leftovers cannot survive (F-7-4)
docker exec -it franka_ros2 bash -lc '
  cd /ros2_ws &&
  rm -rf build/franka_experiments build/franka_simulation_experiments \
         install/franka_experiments install/franka_simulation_experiments &&
  colcon build --packages-select franka_experiments franka_simulation_experiments \
               franka_rt_controllers franka_simulation franka_msgs \
               --cmake-args -DCMAKE_BUILD_TYPE=Release'
#   Copy mode, deliberately: what was built is what runs, and a later edit of the source
#   cannot leak into a run without a rebuild. Symlink mode is convenient for development but
#   makes "which revision ran" depend on the working tree at run time.

# Stamp the install (proposed convention; nothing in the code reads it)
docker exec franka_ros2 bash -lc "echo $REV > /ros2_ws/install/BUILT_FROM_REVISION"

# Verify, byte for byte
python3 -I docs/framework_study/tools/stage7_verification/check_install_provenance.py
#   expected after such a build: the best match is $REV with 0 differing and 0 absent files.
#   Check 2 then FAILs by design ("install is NOT a build of the baseline"): re-assess F-6-3.
docs/framework_study/tools/stage6_verification/run_all.sh   # check_installed → INSTALLED == SOURCE
```

`SIMULATION_VALIDATION_PROTOCOL.md` Phase 1 makes the last two checks mandatory before any
simulation run. Rebuilding `franka_rt_controllers` and `franka_simulation` is included because:
- their compiled plugins cannot be compared byte for byte with the source;
- `libfranka_scenario_obstacles.so` was built on 2026-11-05 from an uncommitted tree (F-7-4).

### 5.4 How to verify that configuration files were updated after a build [NOT EXECUTED as a procedure; each command is a read-only probe]

```bash
docker exec franka_ros2 bash -lc '
  for f in /ros2_ws/src/franka_experiments/config/*.yaml /ros2_ws/src/franka_experiments/config/*.rviz; do
    cmp -s "$f" /ros2_ws/install/franka_experiments/share/franka_experiments/config/$(basename "$f") \
      || echo "STALE $(basename "$f")"; done
  for f in $(cd /ros2_ws/src/franka_simulation_experiments/config && find . -name "*.yaml"); do
    cmp -s /ros2_ws/src/franka_simulation_experiments/config/$f \
           /ros2_ws/install/franka_simulation_experiments/share/franka_simulation_experiments/config/$f \
      || echo "STALE $f"; done'
```

An empty output means every source YAML equals its installed copy. Files present only in the install
(such as `fr3_control.yaml.orig`) are found by `check_install_provenance.py`, not by this loop. The
running node's view can be cross-checked with `ros2 param get /cbf_safety_filter d_safe` [NOT
EXECUTED; needs a running graph].

### 5.5 How to reproduce the environment [NOT EXECUTED]

```bash
docker compose build                          # Dockerfile pins (§3); no repository build inside
docker compose up -d                          # bind-mounts the repo at /ros2_ws/src
docker exec -it franka_ros2 bash              # the entrypoint runs `vcs import franka.repos --skip-existing`
#   then the clean build of §5.3
```

Record with every run:
- the host revision and `git status --porcelain`;
- the image ID (`docker inspect -f '{{.Image}}' franka_ros2`);
- the Python package versions of §3;
- the output of `check_install_provenance.py`.

Two inputs are **not** pinned:
- `torch` and other unpinned pip packages (`absl-py`, `trimesh`, `tqdm`, `rich`);
- the `apt` packages, which track the Humble archive at build time.

Two images built on different days can therefore differ (Inferred).

---

## 6. Launch files

| Launch file | Package | Starts (summary) | Status (ch. 15) | Chapter |
|---|---|---|---|---|
| `torque_control_stack.launch.py` | fe | Pipeline A on hardware: franka bringup, RT torque controller, perception, `pentagon_qddot_commander`/RL, `cbf_safety_filter`, `qddot_to_torque`, ISO nodes, logger, bag, pin | CURRENT | 01, 02, 03, 09 |
| `velocity_cbf_control_stack.launch.py` | fe | Pipeline B: `ee_pentagon_velocity_commander` → `cbf_velocity_filter` → `RtVelocityExecutorController` | ALTERNATIVE | 10 |
| `minimal.launch.py` | fe | driver + bare velocity executor (or `CBFTorqueController` with `use_torque_controller:=cbf`) | CURRENT (bring-up, calibration base) | 01, 09 |
| `thales.launch.py` | fe | velocity-pipeline bring-up + logging | ALTERNATIVE: a manual entry point of Pipeline B (README §5.3: "production velocity pipeline + rosbag"). Whether it still runs on hardware needs user confirmation (ch. 01 §6) | 01 |
| `bag_replay.launch.py` | fe | a recorded bag through the shipped perception + CBF, no robot | CURRENT (offline tool) | 13 |
| `handeye_calibration_bringup.launch.py`, `handeye_eye_in_hand_calibration.launch.py` | fe | calibration (eye-to-hand, eye-in-hand) | CURRENT (manual tooling) | 05 |
| `gz_cbf_stack.launch.py` | fse | Gazebo plant + the same Pipeline A nodes + sim nodes | CURRENT (simulation) | 12 |
| `gz_plant.launch.py`, `gz_circle_demo.launch.py` | fse | plant only, and a circle demo with markers | CURRENT (sim tooling) | 12 |
| `test/launch/test_{torque,velocity,rl}_fake.launch.py` | fe | each pipeline on fake hardware | test-only, manual | 14 §7 |
| `test/launch/launch_test_graph_contract.py` | fse | `launch_testing` of the Gazebo graph against the Phase 0 contract | test-only, slow | 12, 14 §7 |

**Gazebo launch dependencies** (Verified, ch. 12 §2):
- `gz_cbf_stack` includes `franka_simulation/launch/gz_fr3_plant.launch.py`;
- that file needs `franka_ign_ros2_control` (the `IgnitionSystem` hardware plugin), `ros_gz_sim`
  `create`, `ros_gz_bridge` `parameter_bridge` (`/clock`, camera), the
  `libfranka_scenario_obstacles.so` system plugin, `franka_description` (xacro), and
  `GZ_VERSION=fortress`;
- `ROS_DOMAIN_ID` and `IGN_PARTITION` isolate batch cells (`simexp.procs`).

**Real-time helpers** (Verified):
- `scripts/pin_rt_thread.sh` (73 lines) waits up to `TIMEOUT_S` (default 60) for a
  `controller_manager/ros2_control_node` process that owns a `SCHED_FIFO` thread. It then
  `taskset`s **only that thread** onto CPU `${1:-3}`. It needs `isolcpus` on the kernel command line
  to buy anything;
- `torque_control_stack` (L477) and the eye-in-hand calibration launch start it;
- `simexp`'s `pin_process.py` is its Gazebo counterpart.

---

## 7. Automated tests

### 7.1 Suites and how they are run

| Suite | Location | Runner configuration (Verified) | How to run [NOT EXECUTED in Stage 7] |
|---|---|---|---|
| franka_experiments unit tests | `franka_experiments/test/test_*.py` (96 files), plus 2 helper modules (`_cbf_builder_harness.py`, `_launch_snapshot.py`), 3 `smoke_*.py` scripts (not collected: no `test_` prefix) and `test/config/test_defaults.yaml` | `setup.cfg [tool:pytest] norecursedirs = launch` | `cd /ros2_ws/src/franka_experiments && python3 -m pytest test` (**this cwd**, see §7.3) |
| fake-hardware launches | `franka_experiments/test/launch/*.launch.py` | not collected (norecursedirs) | `ros2 launch franka_experiments test_torque_fake.launch.py` (manual; installed via `data_files`) |
| simulation unit tests | `franka_simulation_experiments/test/test_*.py` (16) | `pytest.ini`: `addopts = -m "not slow" -p no:cacheprovider`; `conftest.py` puts `scripts/` and `franka_simulation/scripts` first on `sys.path` | `cd /ros2_ws/src/franka_simulation_experiments && python3 -m pytest` |
| Gazebo tests | `franka_simulation_experiments/test/slow/test_phase{1..5}_*.py`, `test/launch/launch_test_graph_contract.py` | marker `slow`, opt-in | `python3 -m pytest test -m slow`: **starts Gazebo** (prohibited in Stage 7) |
| colcon | — | `ament_python` / `ament_cmake` test hooks | `colcon test --packages-select …`: its working directory and import resolution were **not** examined (Unverified) |

### 7.2 Inventory (static; `tools/stage7_verification/test_traceability.py` [EXECUTED 2026-11-11])

128 test-tree Python files hold 1487 test functions. The tags below are derived from imports and
string patterns, and a file can carry several:

| Tag | Files | Meaning | Runs without |
|---|---:|---|---|
| pure | 74 | numpy/scipy/yaml only | — (host-capable if the import chain has no ROS module) |
| ros | 22 | imports `rclpy`, `launch*`, `*_msgs` or `ament_index_python` | needs the sourced workspace |
| config | 25 | reads a shipped YAML (`fr3_control.yaml`, `conditions.yaml`, …) | — |
| textual | 20 | reads production **source as text** | — |
| gazebo | 5 | `slow` marker or `ros2 launch` strings | needs Gazebo |
| data | 3 | reads `sim_runs/`, logs or bags | needs untracked data |
| pinocchio | 2 | imports Pinocchio directly (many more import it through production modules) | needs the container |
| ml | 1 | torch / onnxruntime / mujoco | needs the ML stack |

**Stale imports: none.** Every name a test imports from a repository module is defined at the
module's top level at the baseline. So no test is *uncollectable* at `ba68077` (Verified,
static). That says nothing about whether the tests **pass**: no test was executed in Stage 7, and no
recorded test result at the baseline exists (Unverified).

### 7.3 Which code a test run exercises (F-7-5)

[READ-ONLY PROBE] with `source /ros2_ws/install/setup.bash`:

| Current directory | `import franka_experiments` resolves to |
|---|---|
| `/ros2_ws/src/franka_experiments` | `/ros2_ws/src/franka_experiments/franka_experiments/`: the **source** |
| `/ros2_ws/src` (repository root) | `/ros2_ws/install/franka_experiments/lib/python3.10/site-packages/franka_experiments/`: the **mixed install** (F-7-4) |

From the repository root, `franka_experiments/` is a directory without `__init__.py`, so it is only
a namespace portion. Python's path finder keeps scanning and prefers the regular package further
along `sys.path` (Derived from the import system; observed above). For `franka_simulation_experiments`,
`conftest.py` pins `simexp` to the source. The `franka_experiments` modules that `simexp` imports
follow the cwd rule.

**Rule for every future run:**
1. run each suite from its own package directory;
2. add a guard to the run log, as the verification scripts do:
   `python3 -c "import franka_experiments as m; print(m.__file__)"`.

### 7.4 Kinds of tests with limited regression value (Verified by reading)

- **Replica tests** re-implement the code they claim to test, so a change to the production code
  cannot fail them:
  - `test_phase_governor.py` (the whole file): "the two functions under test are reproduced here …
    because importing the node needs rclpy". It mirrors
    `PentagonQddotCommander._governor_sigma` and the error cap, and its docstring still says
    `d_safe = 0.20`;
  - `test_cbf_velocity_filter.py::test_gamma_continuous` inlines `select_gamma`;
  - `test_cbf_state_rows.py` L732: a helper "Mirror of `CBFSafetyFilter._obstacle_speed`".
- **Textual tests** (20 files) assert on the *source text* of production files, for example that a
  launch argument is declared or that a default reads `false`. They pin wording: a refactor that
  keeps behaviour but changes text fails them, and a behaviour change that keeps the text passes.
  `test_node_self_references.py` (an AST lint: every `self._foo()` a node calls is defined) is the
  useful kind of static test.
- **Outdated wording, behaviour still pinned correctly:** `test_iso_dsafe_floor.py`'s docstring
  says the shipped `d_safe` is 0.15 m. The test itself reads `fr3_control.yaml`, so its assertion
  (the ISO floor C + Z_d + Z_r = 0.92 m refuses the shipped 0.10 m) is still meaningful.

The full test-to-component matrix is in
[tools/stage7_verification/results/test_traceability.md](tools/stage7_verification/results/test_traceability.md).
The safety-critical gaps are in [REFACTORING_READINESS.md](REFACTORING_READINESS.md) §6.

---

## 8. Runtime diagnostics (where to look when a run misbehaves)

| Tool | What it reports | Chapter |
|---|---|---|
| `ros2 control list_controllers` (polled by the stack launch) | controller states; `pentagon_qddot_commander` waits on it | 02, 09 |
| `/NS_1/cbf_status` (`CbfStatus` fields) | per-tick QP state: level, slack, fault, row counts, d_obs_min | 13 |
| `CBFDIAG` log lines (`logging_utils.format_cbf_diag`) | throttled QP diagnostics | 13 |
| `scripts/pipeline_rate_monitor.py` | live rate and jitter of every pipeline topic | 13 |
| `scripts/iso_preflight_check.py` | refuses to start with an inconsistent ISO configuration | 11 |
| `simexp/graph_contract.py`, `launch_test_graph_contract.py` | the running graph against a reference capture | 12 |
| `experiment_logger` CSV + manifest | per-tick record (F-5-21 caveats) | 13 |
| `tools/check_state.py` (docs) | baseline vs HEAD, ledger integrity, links | 00 |
| `stage6_verification/check_installed.py`, `stage7_verification/check_install_provenance.py` | install vs source; install provenance | 12, this chapter |

---

## 9. Practical example: one parameter through the three translations

Take `d_safe`, the obstacle-clearance margin of every barrier (ch. 07):
1. **Source at the baseline:** `franka_experiments/config/fr3_control.yaml` L66, `d_safe: 0.10`.
2. **Install today:** `share/franka_experiments/config/fr3_control.yaml` says `d_safe: 0.15`. It is
   the `fr3-student-sim` copy, byte-identical to `git show 42df80e:…`.
3. **Run:** `load_package_config` reads the share copy (§4.4). `gz_cbf_stack` would then evaluate
   every scenario against 0.15 m, and `cbf_safety_filter` would tighten every obstacle row by 5 cm
   more than chapter 07 describes. The installed `cbf_safety_filter.py` itself differs from the
   baseline (565 diff lines).
4. **Test:** `python3 -m pytest franka_experiments/test` from `/ros2_ws/src` imports the installed
   modules **and** reads `fr3_control.yaml` through paths relative to the test file (the source,
   0.10). The test therefore mixes the baseline configuration with the other branch's code (Derived
   from F-7-5 and `test_iso_dsafe_floor.CONFIG`).

---

## 10. Consequences of modification

- **Adding, renaming or deleting a file** in either target package changes the install only after a
  rebuild (a re-configure for `franka_simulation_experiments` scripts). A deletion **never**
  propagates to an existing copy-mode install: the old module stays importable, so a test that still
  imports it passes (§4.3, F-7-4). Always use the clean build of §5.3.
- **Changing a YAML file** has no run-time effect without a rebuild (§4.4). A refactor that moves a
  parameter between files must check §5.4 afterwards.
- **Adding a console script** needs `setup.py` `entry_points`. **Adding a sim program** needs only a
  file in `scripts/` plus a re-configure. Its installed name drops `.py`.
- **Adding a Python dependency** needs the Dockerfile, because `package.xml` does not carry Python
  packages. The `qpsolvers`/`osqp` pin is load-bearing: the image build asserts it.
- **Declaring `franka_rt_controllers`** in `franka_experiments/package.xml` (F-0-6) changes colcon's
  build order only. It is safe, but it is a change and goes through the protocol.
- **Tests:**
  - the replica tests (§7.4) must be rewritten against production code *before* the code they mirror
    is refactored, or they give false assurance;
  - the textual tests must be reviewed with every textual refactor.
- **Environment:** the protocol's Phase 1 depends on `check_install_provenance.py` and
  `check_installed.py`. If the install layout changes (Python version, `lib/python3.10`), update their
  path maps.

---

## 11. Findings from this chapter

| ID | Summary | Where |
|---|---|---|
| F-7-4 | The install is a mixed tree, mostly the `fr3-student-sim` build of 2026-11-10 15:42; it is the cause of F-6-3 | §5.1 |
| F-7-5 | From the repository root, tests import the installed copy, not the source | §7.3 |
| F-0-5, F-0-6, F-2-2, F-6-3 | (earlier) relative-path import of scripts; undeclared dependency; stale install; installed configs differ | §2.2, §5 |

The full records are in [16_issues_and_discrepancies.md](16_issues_and_discrepancies.md).
