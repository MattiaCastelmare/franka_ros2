# 00_baseline — S1 preservation manifest (pre-rebuild state)

> **Step S1 of the baseline plan (checkpoint C-052), approved by the user on 2026-11-12. Only S1 was
> executed.** Nothing was built, started, launched, tested, deleted, moved or committed. The container
> `franka_ros2` stayed **stopped** throughout: every read used `docker cp <c>:<path> -`.

## 1. Pinned revisions

| Item | Value |
|---|---|
| Documented source baseline | `ba68077052e8d0fb3d4606c05dff553b75baa641` (ancestor of HEAD: yes) |
| Documentation HEAD | `f7089911decf30103eaef4e4d4b6e3afbc1f91d1` = `origin/humble-mattia` |
| Paths outside `docs/framework_study/` and `CLAUDE.md` changed since the baseline | none |
| Working tree at S1 | uncommitted documentation only: `CHECKPOINTS.md`, `PROGRESS.md` (C-052), untracked `tools/baseline_prep/`, and this directory |
| Stashes | 3, untouched (`stash@{0..2}` refs recorded in `provenance/git_stash_list.txt`) |
| Other branch whose build is installed (F-7-4) | `fr3-student-sim` = `42df80e5855e` |
| Gitignored vcs checkouts | `franka_description` `1ccde30d5a30` (1.3.0); `libfranka` `155ddac80565` (0.15.0) |

The intended rebuild revision is the working tree at `f708991`, whose ROS 2 source is identical to
`ba68077` (check_state: 0 non-documentation paths changed).

## 2. Evidence location (outside Git)

`~/franka_baseline_evidence/baseline_f708991/00_pre_rebuild/` — 1.2 GB, 22 files, write-protected
(`chmod -R a-w`) after checksumming. Verify with
`cd ~/franka_baseline_evidence/baseline_f708991/00_pre_rebuild && sha256sum -c SHA256SUMS`.
SHA-256 of `SHA256SUMS` itself: `84e0773268d35393a74b146603255b062dcf887ca91c5ebfa41f47f57173751d`.

| File | Bytes | SHA-256 | Content |
|---|---:|---|---|
| `container/ros2_ws_install.tar` | 11 687 424 | `0b5b56cb00ce6eb515954f023cc99ce4e1f374fe5fc5718c13ced1b7ffe83ec9` | `/ros2_ws/install` (2331 entries; symlinks kept as links) |
| `container/ros2_ws_build.tar` | 1 092 067 840 | `5fcfafe745d8f10dfedd84bcbb773b0a6fa4684452ca7179d17ceb51f81b8562` | **all** of `/ros2_ws/build` (5662 entries): a superset of the six in-scope packages, and the targets of every `install/ → build/` link |
| `container/ros2_ws_log.tar` | 83 362 816 | `bd8058b5f388f00ab3a4c0bddded945440b551237b5da0f34aee8ea573146f4e` | `/ros2_ws/log` (97 colcon builds; the F-7-4 evidence) |
| `container/container_inspect.json`, `image_inspect.json`, `container_ps.txt` | | | container state (exited 137, finished 2026-11-11 17:14:22 UTC), image `3ab2ed057f0e`, mounts, writable layer 2.93 GB |
| `container/container_dpkg_status.txt`, `container_key_apt_versions.txt` | | | apt state; e.g. libignition-gazebo6 6.18.0, ros-gz 0.244.26, ros2-control 2.54.3, ros2-controllers 2.54.1, rclpy 3.3.22, pinocchio 4.0.0 |
| `container/container_python_user_dist_info.txt` | | | 73 user pip packages; qpsolvers 4.3.3, osqp 0.6.7.post3, numpy 1.26.4, scipy 1.15.3, torch 2.13.0, mujoco 3.4.0, protobuf 4.25.9 (match the Dockerfile pins) |
| `container/probe_workspace_state.txt` | | | `tools/baseline_prep/probe_workspace_state.py` re-run at S1: identical to `results/observed_2026-11-12.txt` except the order of two mount lines, so the container did not change between C-052 and S1 |
| `provenance/git_*` | | | revisions, `git status --porcelain`, stash refs, untracked list, last 40 reflog entries |
| `provenance/working_tree_docs.patch` | 7020 | see SHA256SUMS | `git diff HEAD --binary` (tracked documentation edits of C-052) |
| `provenance/untracked_docs_copy/`, `untracked_docs_sha256.txt` | | | copies of the untracked `tools/baseline_prep/` files (the patch does not contain untracked files) |
| `provenance/host_environment.txt`, `host_lscpu*.txt` | | | kernel 6.17.0-35, i7-14700HX (28 logical CPUs), intel_pstate, EPP `performance` on all CPUs, profile `performance`, no `isolcpus`, Docker 29.1.3, RTX 4070 Laptop / driver 580.126.09, RAM 31 GiB, disk 285 GB free |

The three archive checksums are given in full; every other checksum is in `SHA256SUMS`.

**What the archives do not contain.** `install/` holds symlinks into `/ros2_ws/src` (the repository)
and `/opt/ros` (the image). Their *contents* are not in the tars: the repository side is pinned by
`f708991` + the patch, the image side by the image ID. The container's other state (user pip
packages, `/opt`, `~/.ros`) is not touched by the planned S3 and is recorded only as versions.

## 3. Files identified for backup, and their status

| What | Affected by the plan? | Status |
|---|---|---|
| `/ros2_ws/install/<6 pkgs>`, `/ros2_ws/build/<6 pkgs>` (deleted by S3) | yes | **preserved** (whole trees) |
| `/ros2_ws/log` (F-7-4 evidence; S3 adds a build, deletes nothing) | no | preserved |
| uncommitted documentation (C-052) | no | patch + copies preserved; not committed (the user commits) |
| stashes | no | refs recorded; untouched |
| `franka_simulation_experiments/sim_runs/` (69 GB, gitignored, older revisions, F-6-1) | no | not copied; untouched |
| `franka_description/`, `libfranka/` gitignored checkouts | no (entrypoint `vcs import --skip-existing` skips them) | SHAs recorded |
| the container itself | no (S2 is `docker start`, never `compose up --build`) | not exported (13.2 GB virtual); image ID recorded |

## 4. Observations

O-1…O-8 (CHECKPOINTS C-052) remain **observations, not findings**, as decided by the user. S1 adds
no observation that changes them. Facts recorded at S1, for later decisions (not choices):
- CPU topology (`host_lscpu_topology.txt`): CPUs 0–15 = 8 hyper-threaded P-cores (pairs 0/1 … 14/15;
  cores 4 and 6 = CPUs 8, 9, 12, 13 reach 5.5 GHz, the others 5.3 GHz), CPUs 16–27 = 12 E-cores
  (3.9 GHz). No isolated CPU. Input for the `--filter-cpu` decision.
- Governor `powersave` with EPP `performance` under intel_pstate (the dynamic intel_pstate mode).

## 5. Still requiring approval

S2 (`docker start franka_ros2` + read-only dependency checks), S3 (clean rebuild; whether all six
packages are needed), S4 (provenance checks), S5 (tests), the runs, `--filter-cpu`, cells, seeds,
thresholds (Q-6-3), and the treatment of F-6-5 / F-6-11 (Q-6-4).
