# 00_baseline — S3: clean six-package build (result)

> Approved by the user on 2026-11-12 (S3 only). **Result: BUILD SUCCEEDED** (6/6 packages, colcon
> exit 0, 34.9 s). This record covers only the build and its immediate sanity checks. Source/install
> equivalence and runtime environment checks belong to **S4, which is not executed and not approved**.
> The install is therefore **not yet declared validated**.

## 1. Preconditions (all passed)

`humble-mattia` / HEAD `f7089911decf` / baseline `ba68077` (ancestor; 0 non-documentation paths
changed; `git status` outside `docs/framework_study/` empty); `check_state` consistent; S1 archives
22/22 checksums OK (verified twice, the second time immediately before the move); container
`franka_ros2` running with only `/bin/bash`; no Gazebo, launch, ros2_control or colcon process on
the host or in the container; `01_build/` did not exist.

## 2. What was done (exact commands)

Host (evidence, read-only for the repository): `mkdir ~/franka_baseline_evidence/baseline_f708991/01_build`;
`git rev-parse HEAD`, `git status --porcelain=v1 --untracked-files=all`, `git stash list`,
`docker diff franka_ros2` before and after; a timestamp marker file; the environment dump.

Container (one `docker exec -w /ros2_ws -e PYTHONDONTWRITEBYTECODE=1 franka_ros2 bash -c '…'`):

```bash
set -eo pipefail
PK="franka_msgs franka_ign_ros2_control franka_rt_controllers franka_simulation franka_experiments franka_simulation_experiments"
Q=/ros2_ws/quarantine_s3_20261112
[ -e "$Q" ] && exit 8
mkdir -p "$Q/build" "$Q/install"; touch "$Q/COLCON_IGNORE"
for p in $PK; do for t in build install; do
  [ -d "/ros2_ws/$t/$p" ] && [ ! -L "/ros2_ws/$t/$p" ] || exit 7
  mv "/ros2_ws/$t/$p" "$Q/$t/$p"; done; done
source /opt/ros/humble/setup.bash
env | grep -E "^(PYTHONPATH|AMENT_PREFIX_PATH|CMAKE_PREFIX_PATH|LD_LIBRARY_PATH|COLCON_PREFIX_PATH)=" | grep -q /ros2_ws && exit 6
colcon build --base-paths src --packages-select $PK \
             --cmake-args -DCMAKE_BUILD_TYPE=Release --event-handlers console_cohesion+
```

**Two deliberate differences from `S3_BUILD_PLAN.md` §3, both safety-only:**
1. **Quarantine instead of `rm -rf`**, at the user's preference: the 12 package directories were
   *moved* (same filesystem, no copy) into `/ros2_ws/quarantine_s3_20261112/{build,install}/<pkg>`
   (63 MB). The quarantine root carries `COLCON_IGNORE`, so colcon never discovers the old
   `package.xml` copies. Nothing was deleted. Inside the quarantine, the old `install/` symlinks into
   `build/` now dangle (their targets moved too); the S1 tars remain the faithful backup.
2. **`--base-paths src`**: limits package discovery to the repository (the same set as the S2
   `colcon list --base-paths src`), so no directory under `/ros2_ws` other than `src/` can contribute
   a package.

Pre-move checks: each of the 12 paths was a real directory (not a symlink) whose `realpath` is the
same path under `/ros2_ws/{build,install}`; no other installed package had a symlink into any of them.

## 3. Build environment (`01_build/build_environment.txt`)

Only `/opt/ros/humble/setup.bash` sourced. `AMENT_PREFIX_PATH=/opt/ros/humble`; `CMAKE_PREFIX_PATH`
unset; `PYTHONPATH` = `/opt/ros/humble/…` + `/opt/openrobots/lib/python3.10/site-packages` + trailing
`:` (O-9); `LD_LIBRARY_PATH` = `/opt/ros/humble/…` + `/opt/openrobots/lib` + trailing `:`. **No variable
contained `/ros2_ws`** (guard in the script). Colcon itself added the installed, non-rebuilt workspace
dependencies (`franka_hardware`, `libfranka`, `franka_description`, `franka_semantic_components`, …) as
designed.

## 4. Result

| Package (colcon order) | Started after | Time | Status |
|---|---|---:|---|
| `franka_msgs` | — | 11.7 s | finished |
| `franka_simulation` | `franka_msgs` | 15.5 s | finished |
| `franka_ign_ros2_control` | `franka_msgs` (via `franka_hardware` already installed) | 16.7 s | finished |
| `franka_rt_controllers` | `franka_msgs` | 22.4 s | finished |
| `franka_experiments` | `franka_simulation` | 0.86 s | finished |
| `franka_simulation_experiments` | the others | 0.61 s | finished |

Wall time 11:14:36 → 11:15:11 UTC (35 s); `Summary: 6 packages finished [34.9s]`; colcon exit 0. The
three C++ packages after `franka_msgs` built in parallel; the topological constraints of S2 held.

**Warnings (no compiler or linker warning, no error):**
- `warning: build_py/install_lib: byte-compiling is disabled, skipping` (4×, `franka_experiments`):
  the intended effect of `PYTHONDONTWRITEBYTECODE=1`. The install has no `.pyc`; Python compiles in
  memory at run time (and will not write `__pycache__` into the install while that variable is set).
- `CMake Warning … FindBoost.cmake:2201 … No header defined for python310` (2×, `franka_rt_controllers`,
  via `/opt/ros/humble/…/eigenpy/boost.cmake`): a known benign header-check message of eigenpy's Boost
  search. It shows that Pinocchio/eigenpy were found under `/opt/ros/humble` (O-10; confirmed in S4).

## 5. Immediate sanity checks (non-runtime)

| Check | Result |
|---|---|
| Six install and build directories exist | yes |
| Install mode | **copies only**: 0 symlinks in all six (`franka_msgs` 552 files, `franka_ign_ros2_control` 38, `franka_rt_controllers` 43, `franka_simulation` 281, `franka_experiments` 183, `franka_simulation_experiments` 61) |
| Controller plugins | `install/franka_rt_controllers/lib/libfranka_rt_controllers.so` (1 147 912 B), `libcbf_torque_controller.so` (1 534 720 B); descriptors `franka_rt_controllers.xml`, `rt_velocity_blender_controller.xml`; pluginlib index entry present |
| Gazebo plugins | `install/franka_ign_ros2_control/lib/libfranka_ign_ros2_control-system.so` (674 272 B), `libign_hardware_plugins.so` (757 576 B), `ign_hardware_plugins.xml`; `install/franka_simulation/lib/libfranka_scenario_obstacles.so` (475 824 B) |
| Messages | `install/franka_msgs/lib/libfranka_msgs__rosidl_typesupport_cpp.so` and the other typesupport libraries |
| Executables | 22 `franka_experiments` entry points; 10 sim programs + `simexp/` |
| Known leftovers gone | `fr3_control.yaml.orig/.bak_baseline`, `run_control_regression` absent. `self_shadow.py` is present **and is a baseline file** (`git ls-files`); installed copy == source |
| Installed `d_safe` | `0.10` (`share/franka_experiments/config/fr3_control.yaml` L66), as in the source (was 0.15) |
| Source tree | no file in the repository (tracked, untracked or gitignored, incl. `sim_runs/`) newer than the pre-build marker, outside `docs/framework_study/` |
| Git | `git status` byte-identical before/after; HEAD `f708991`; 3 stashes unchanged |
| Container | `docker diff` before/after: no change outside `/ros2_ws/{build,install,log,quarantine_s3_20261112}` |
| Logs | old colcon logs untouched; new `log/build_2026-11-12_11-14-36`; `log/latest*` symlinks repointed by colcon |

Byte sizes of the four plugins equal those of the pre-rebuild binaries (INFO only, not evidence of
equality; S4 compares symbols).

## 6. Evidence

`~/franka_baseline_evidence/baseline_f708991/01_build/` (15 MB, 14 files, `SHA256SUMS`, write-protected;
SHA-256 of `SHA256SUMS`: `fe9b8406572945e312c8a7a81027a81ba82ed128619d1ef14dde2f63badd942f`):
`colcon_build_console.log` (complete console, 2228 lines), `colcon_log_build_2026-11-12_11-14-36.tar`
(colcon's own logs, per package), `build_environment.txt`, `HEAD.txt`, `git_status_{before,after}.txt`,
`git_stash_{before,after}.txt`, `docker_diff_{before,after}.txt`, `install_six_packages_sha256.txt`
(1158 installed files), `quarantine_listing.txt`, `s1_reverified.txt`, `host_marker_before_build`.

## 7. Not done (S4 and later, each needs approval)

Source/install equivalence (`probe_workspace_state.py`, `check_install_provenance.py`,
`check_installed.py`), message-ABI comparison with the S1 archive, `ldd`/CMake-cache/symbol checks of
the plugins, import guard, runtime checks, tests, Gazebo. The quarantine is kept until the user
decides (it can be removed only with approval).
