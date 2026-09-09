# franka_sim — Safe RL + CBF, Sim-to-Real (MuJoCo)

Standalone training module (no ROS 2 dependency) for a **Safe Reinforcement
Learning** policy shielded by the **same acceleration-level Control Barrier
Function (CBF)** filter that runs on the real FR3 in
`franka_experiments/nodes/cbf_safety_filter.py`. A policy trained here against
the shield meets the identical safety filter on hardware → *safe exploration*
in sim, *safe execution* on the robot.

Architecture and roadmap: `../franka_sim_to_real_roadmap.md`.
What is actually built and how it was validated:
`../franka_sim_to_real_implementation_status.md`.

---

## 0. Start here (the three commands you actually use)

Everything runs **inside the `franka_ros2` container** — the training stack
(torch+CUDA, mujoco, gymnasium, SB3, onnx, osqp) lives in the image, not on the
host. The host has no numpy; do not try to run any of this outside Docker.

```bash
# ── once per boot, on the HOST ──────────────────────────────────────────────
cd ~/Git/franka_ros2
USER_UID=$(id -u) USER_GID=$(id -g) docker compose up -d
xhost +local:docker            # only needed if you want the MuJoCo window
```

**Run these on the HOST**, in a normal terminal — `docker exec` is what reaches
*into* the container, so it does not work from inside one.

```bash
# ── 0. never start a second run on top of a first one ───────────────────────
docker exec franka_ros2 pgrep -af "franka_sim.train" || echo "free"
```

Two runs with the same `--exp-name` overwrite each other's checkpoints and halve
the throughput fighting over the GPU (measured: 250 -> 101 fps). Always check.

```bash
# ── 1a. TRAIN, watching the logs live (Ctrl+C stops it) ─────────────────────
docker exec -it franka_ros2 bash -lc 'cd /ros2_ws/src && MUJOCO_GL=egl \
  python3 -m franka_sim.train --exp-name sac_v3 --total-timesteps 2000000'

# same, but also saving the log to a file
docker exec -it franka_ros2 bash -lc 'cd /ros2_ws/src && MUJOCO_GL=egl \
  python3 -m franka_sim.train --exp-name sac_v3 --total-timesteps 2000000 2>&1 \
  | tee /ros2_ws/src/franka_sim/runs/sac_v3_train.log'
```

Closing the terminal kills a foreground run. To survive it, detach instead —
but then the logs go to the file, not to your screen, which is the whole reason
for the `tail -f`:

```bash
# ── 1b. TRAIN detached (~2.5 h for 2M steps on the RTX 4070) ────────────────
docker exec -d franka_ros2 bash -lc 'cd /ros2_ws/src && MUJOCO_GL=egl \
  nohup python3 -m franka_sim.train --exp-name sac_v3 --total-timesteps 2000000 \
  > /ros2_ws/src/franka_sim/runs/sac_v3_train.log 2>&1'

tail -f franka_sim/runs/sac_v3_train.log        # follow it from the host
```

Either way you get a block like this every ~2000 steps:

```
| rollout/              |          |
|    ep_rew_mean        | -183     |
|    success_rate       | 0.14     |   <- the task metric
| safety/               |          |
|    collision_rate     | 0        |
|    min_surface_dist   | 0.0988   |
|    mean_intervention  | 4.5      |
| time/                 |          |
|    episodes_completed | 88       |
|    fps                | 250      |
|    total_timesteps    | 35402    |
| train/                |          |
|    actor_loss         | 13.6     |
|    critic_loss        | 0.0601   |
```

```bash
# ── 2. TEST the newest snapshot, with the viewer + the full metrics table ───
docker exec franka_ros2 bash -lc 'cd /ros2_ws/src && MUJOCO_GL=egl \
  python3 -m franka_sim.scripts.evaluate_policy \
    --latest franka_sim/models/sac_v3 --episodes 10 --render'

# ── 3. COMPARE every checkpoint of the run, against the baselines ───────────
docker exec franka_ros2 bash -lc 'cd /ros2_ws/src && MUJOCO_GL=egl \
  python3 -m franka_sim.scripts.compare_checkpoints \
    --model-dir franka_sim/models/sac_v3 --episodes 5'
```

Stop a run: `docker exec franka_ros2 pkill -f franka_sim.train`. Nothing is
lost — the checkpoints already on disk stay valid.

### Want to see something move right now?

`models/sac_v2` is still on disk and still runs:

```bash
docker exec franka_ros2 bash -lc 'cd /ros2_ws/src && MUJOCO_GL=egl \
  python3 -m franka_sim.scripts.evaluate_policy \
    --latest franka_sim/models/sac_v2 --episodes 10 --render'
```

**But do not quote its numbers.** `sac_v2` (2026-08-31, 400 k steps) was trained
under `d_safe=0.20`, a faster obstacle, no gripper mass and the bare flange as
its end effector — four things that have since changed. It loads because the
observation is still 24-dim, and `--latest` picks up its *frozen* `config.yaml`
so it replays close to its own training conditions. It is a demo that the
pipeline runs, not a result.

That frozen config is also why `sac_v2` still shows the **old fast obstacle**
(1.13 m/s). To watch it against the current slow one, override the config:

```bash
docker exec franka_ros2 bash -lc 'cd /ros2_ws/src && MUJOCO_GL=egl \
  python3 -m franka_sim.scripts.evaluate_policy \
    --latest franka_sim/models/sac_v2 --config franka_sim/config.yaml \
    --episodes 10 --render'
```

---

## 1. Layout

```
franka_sim/
├── config.yaml                 # ONE config: env, task, obstacle, reward, CBF, RL
├── assets/franka_fr3/          # Menagerie FR3 MJCF + the Franka Hand (added here)
│   ├── fr3.xml                 # arm + hand/fingers; NOT pristine Menagerie any more
│   └── scene_cbf.xml           # scene + mocap obstacle (human proxy) + target marker
├── envs/
│   ├── cbf_filter.py           # accel HOCBF QP (raw OSQP) — mirror of the real node
│   └── franka_cbf_env.py       # gymnasium.Env: FR3 reach + moving obstacle, shielded
├── train.py                    # SAC + CUDA + TensorBoard + step/episode checkpoints
├── export_onnx.py              # SAC actor → ONNX (validated vs SB3), for deployment
├── models/<exp>/               # best_model, final_model, frozen config, checkpoints/
├── runs/<exp>_1/               # TensorBoard event files
└── scripts/
    ├── validate_cbf.py         # reduced-model proof the shield holds d ≥ d_safe
    ├── validate_actuation.py   # regression guard: the action must control the arm
    ├── evaluate_policy.py      # score one policy (or zero/random) + --latest
    └── compare_checkpoints.py  # score EVERY checkpoint of a run, one table
```

---

## 2. Training

```bash
python3 -m franka_sim.train --exp-name sac_v3 --total-timesteps 2000000
```

| Flag | Meaning |
|---|---|
| `--exp-name` | run name → `models/<name>/`, `runs/<name>_1/`. Default: timestamp |
| `--total-timesteps` | overrides `rl.total_timesteps` (2 M) |
| `--checkpoint-every-episodes N` | overrides `rl.checkpoint_freq_episodes` (200). `0` disables |
| `--no-episode-onnx` | skip the ONNX export of each episode checkpoint |
| `--resume <path.zip>` | continue from a snapshot |
| `--seed`, `--device` | override the config |

### What gets logged, and where

| Where | What |
|---|---|
| stdout / your `.log` | SB3 table (`ep_rew_mean`, `success_rate`, fps, losses), a `safety/` block, and one `[episode-ckpt]` line per snapshot |
| `runs/<exp>_1/` | TensorBoard: `tensorboard --logdir franka_sim/runs`, then open `localhost:6006` |
| `models/<exp>/checkpoints/` | `sac_ep000200.zip` **+ `.onnx`** every 200 episodes; `sac_*_steps.zip` every 50 k steps |
| `models/<exp>/best_model.zip` | best eval score, refreshed every 25 k steps |
| `models/<exp>/final_model.zip` | end of run |
| `models/<exp>/config.yaml` | the config frozen at launch — this is what makes a run reproducible |

The `safety/*` curves — `collision_rate`, `min_surface_dist`,
`mean_surface_dist`, `cbf_active_frac`, `mean_intervention`, `mean_slack` — are
the paper's safe-exploration evidence.

**Why episode checkpoints as well as step checkpoints.** Episodes end early on
success and on collision, so a fixed number of steps holds a very different
amount of task experience early vs late in a run (measured: 1502–2500 steps per
5 episodes). Snapshots spaced by episodes are the ones worth comparing, and each
is exported to ONNX immediately — a checkpoint you cannot hand to
`rl_policy_commander` is a checkpoint you cannot actually test.

---

## 3. Testing a policy

```bash
# newest snapshot in a run — resolved by MTIME, not by filename, because
# best_model is rewritten whenever eval improves
python3 -m franka_sim.scripts.evaluate_policy --latest franka_sim/models/sac_v3 \
    --episodes 25 --render

# or an explicit artifact
python3 -m franka_sim.scripts.evaluate_policy \
    --model franka_sim/models/sac_v3/best_model.onnx --episodes 50
```

Prints: success rate, mean/median final EE error, episode return and length,
collision rate, min/mean surface distance, CBF-active fraction, mean
intervention `‖q̈_safe − q̈_nom‖`, mean slack, and per-step inference time.

`--latest` also picks up that run's frozen `config.yaml` automatically, so you
score the policy under the shield it was trained with.

### Always score the baselines

```bash
python3 -m franka_sim.scripts.evaluate_policy --model zero   --episodes 50
python3 -m franka_sim.scripts.evaluate_policy --model random --episodes 50
```

**Read every safety number against the `zero` row, never in isolation.** Part of
any collision rate is the obstacle sweeping into an arm that cannot clear it —
a barrier can only bound the *robot's* motion, so no CBF can prevent that. The
zero-action baseline (an arm that never moves) is the only measurement of how
large that share is. `compare_checkpoints` puts both baselines in the table and
warns explicitly if a policy collides *more* than a motionless arm.

This is also the check that once caught a silent actuation bug: three completely
different action streams scoring identically meant the action never reached the
plant. If a trained policy ties the zero baseline, suspect the plant before the
policy.

### Comparing a whole run

```bash
python3 -m franka_sim.scripts.compare_checkpoints \
    --model-dir franka_sim/models/sac_v3 --episodes 5 --render-best
```

| Flag | Meaning |
|---|---|
| `--episodes` | per checkpoint. Keep it small (5) — this is a progress check, not the 50-episode benchmark |
| `--last N` | only the last N snapshots |
| `--no-baselines` | drop the zero/random rows |
| `--render-best` | after the table, replay the best snapshot in the viewer |

---

## 4. The MuJoCo viewer

`--render` / `--render-best` open a live MuJoCo window.

* Run `xhost +local:docker` on the **host** once per session. `DISPLAY` and the
  X11 socket are already wired in `docker-compose.yml`.
* The viewer uses GLFW and needs a real `DISPLAY` **whatever `MUJOCO_GL` is set
  to** — verified: with `DISPLAY` removed it fails with
  `could not initialize GLFW` both with and without `MUJOCO_GL=egl`.
  `MUJOCO_GL=egl` only selects the *offscreen* backend (`render_mode='rgb_array'`)
  and is harmless alongside the window, which is why every command here carries
  it: it makes the headless runs work and costs nothing in the windowed ones.
* The metrics table is printed before the environment is closed, so you get
  your numbers even if the window misbehaves on teardown.
* **One viewer per process.** Opening a second one in the same interpreter
  segfaults MuJoCo, which is why `compare_checkpoints` renders only at the end,
  after the table.

### Playback speed

The viewer is paced to **wall-clock real time** (`env.render_speed`, default
`1.0` in `config.yaml`). Before this existed the loop ran as fast as the CPU
allowed — measured **2.4x real time** — so every motion, the obstacle most
visibly, looked much faster than it was and speeds could not be judged by eye.

```yaml
env:
  render_speed: 1.0     # 1.0 = real time · 0.5 = slow motion · 2.0 = fast-forward
```

Measured: `0.5 -> 0.47x`, `1.0 -> 0.87x`, `2.0 -> 1.55x` real time (values above
1.0 are capped by how fast the sim can actually run). **Training and headless
evaluation are untouched** — the pacing lives in `render()`, which only executes
under `render_mode='human'`; headless still runs at ~12.7x real time.

---

## 5. Environment (`FrankaCBF-v0`)

| | |
|---|---|
| **Action** `a ∈ [−1,1]⁷` | nominal joint acceleration `q̈_nom = a · q̈_max` — the input the real `cbf_safety_filter` receives on `/NS_1/qddot_nom` |
| **Observation** (24) | `[q(7), q̇(7), ee_pos(3), target(3), obstacle(3), d_min(1)]` |
| **`ee_pos`** | `hand_tcp_site` ≡ URDF `fr3_hand_tcp`, the Franka Hand grasp centre |
| **Shield** | every step: `q̈_safe = CBF.filter(q, q̇, q̈_nom, obstacles)` before actuation |
| **Actuation** | `q̈_safe → τ = M(q)q̈ + C(q,q̇)q̇ + g(q)` (`mj_inverse`, recomputed every substep) — the same chain as `qddot_to_torque` + `rt_torque_controller` + firmware gravity |
| **Reward** | `−‖ee−target‖ + success − effort − CBF_intervention − slack − jerk`, collision penalty |
| **Episode** | 5 s @ 100 Hz; terminate on collision (`d<0`) or success |
| **Obstacle** | kinematic sphere, 0.20 Hz × 0.20 m (~0.25 m/s peak) |

The obstacle is a mocap sphere (`contype=0`, never a physical MuJoCo contact);
"collision" is surface distance `< 0`, handled by the reward — exactly how the
real MultiLinkDistance pipeline treats the human point cloud.

### The gripper

The MJCF carries the Franka Hand: meshes converted from
`franka_description/meshes/robot_ee/franka_hand_white` (DAE → OBJ, MuJoCo cannot
read DAE), transforms **read from the real URDF through Pinocchio rather than
guessed**, and cross-checked — sim and robot FK agree to **7e-16 m** at both the
flange and the TCP.

* **Fingers are rigid, parked fully open.** Slide joints would push `nv` past 7,
  which the whole actuation chain assumes (`mj_inverse` over all DOFs, the
  7-vector action, `data.ctrl` indexing, the QP width). This task reaches, it
  never grasps. Open is also the widest, most conservative footprint.
* **The hand is a CBF control point** (`fr3_hand`, radius 0.13 — the measured
  bounding sphere of hand + fingers). This mirrors the robot, where
  `fr3_complete.yaml` maps `fr3_link8` to the hand collision mesh; the gripper
  *is* covered by the real perception pipeline.
* `assets/franka_fr3/fr3.xml` is therefore **no longer pristine Menagerie** —
  do not overwrite it from upstream without re-adding the hand.

---

## 6. CBF filter (`cbf_filter.AccelCBFFilter`)

Same math as the robot: HOCBF barrier `h = d − d_safe` (relative degree 2), per
obstacle row `aᵢᵀq̈ + s ≥ −k1(aᵢᵀq̇) − k0·h̄ᵢ − ċᵢ` (soft, slack-relaxable), a
hard state-limit box (velocity/position braking + slew continuity) and a hard
workspace box, solved by raw OSQP. Gains (`k0=25, k1=10.5, d_safe=0.15, ρ=1000`)
and limits come from `config.yaml`.

**The workspace box is SIM-ONLY.** `workspace_face_rows` lost its last live
importer on the robot in commit `4d4d450`, so hardware enforces no Cartesian
box. Training inside one is deliberate and conservative — the policy learns a
region *smaller* than the robot allows — but it is not a guarantee that survives
sim-to-real. `test_workspace_box_is_sim_only` fails if the robot ever gets
workspace rows back without them being re-mirrored here.

---

## 7. Deploying to the robot

```bash
ros2 launch franka_experiments torque_control_stack.launch.py \
    motion_source:=rl start_move_group:=false \
    rl_onnx_model:=/ros2_ws/src/franka_sim/models/sac_v3/best_model.onnx \
    rl_action_scale:=0.3
```

`rl_action_scale` derates the policy (`q̈_nom = a·q̈_max·scale`); **0.3 for a
first run on real hardware**, with the camera and `real_time_distance` on, a
single conservative target, and `rl_status` + `cbf_status` on screen.

`rl_policy_commander` rebuilds the identical 24-dim observation from robot
topics and publishes into the same `cbf_safety_filter` the policy trained
against. The shared contract lives in `franka_experiments/utils/rl_policy.py`.

---

## 8. Keeping sim and robot in sync

`config.yaml`'s `cbf:` and `joint_limits:` blocks **mirror**
`franka_experiments/config/fr3_control.yaml`. Treat the robot's file as the
source of truth and mirror *into* this one, never the reverse.

Run this after any change on either side:

```bash
docker exec franka_ros2 bash -lc 'source /opt/ros/humble/setup.bash && \
  cd /ros2_ws/src/franka_experiments && python3 -m pytest test/test_rl_policy.py -q'
```

| Test | Catches |
|---|---|
| `test_real_configs_are_in_sync` | drifted CBF gains / joint limits, **and missing keys on either side** |
| `test_workspace_box_is_sim_only` | workspace rows reappearing on the robot |
| `test_ee_frame_matches_sim_ee_site` | `env.ee_site` and the node's `ee_frame` naming different points |
| `test_observation_*`, `test_action_*` | observation layout / action scaling drift |

A renamed key is indistinguishable from a deleted one, which is how a real
drift once hid: the sync test raised `KeyError` instead of failing. It now
asserts presence on both sides before comparing.

Sanity checks worth re-running after touching `step()` or the MJCF:

```bash
python3 -m franka_sim.envs.franka_cbf_env         # env self-check + random rollout
python3 -m franka_sim.scripts.validate_cbf        # shield holds at d_safe
python3 -m franka_sim.scripts.validate_actuation  # the action controls the arm
```

---

## 9. Gotchas

* **Never run this on the host** — no numpy/mujoco there. Always `docker exec`.
* **A stale `install/` will lie to you.** Sourcing `/ros2_ws/install/setup.bash`
  with an unbuilt workspace once produced 9 phantom test failures. After editing
  anything under `franka_experiments/`:
  `docker exec franka_ros2 bash -lc 'source /opt/ros/humble/setup.bash && cd /ros2_ws && colcon build --packages-select franka_experiments --symlink-install'`
* **`qpsolvers` / `osqp` are version-locked** (`qpsolvers==4.3.3`, `osqp<1.0`) in
  the Dockerfile. Newer qpsolvers imports a symbol that only exists in osqp 1.x,
  leaving `qpsolvers.available_solvers == []` and the ROS-side filters dying
  with `SolverNotFound`.
* **Every policy trained before 2026-09-04 is void** — `d_safe`, the obstacle
  regime, the gripper mass and the EE frame all changed.
