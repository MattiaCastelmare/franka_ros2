# franka_sim — Safe RL + CBF, Sim-to-Real (MuJoCo)

Standalone training module (no ROS 2 dependency) for a **Safe Reinforcement
Learning** policy shielded by the **same acceleration-level Control Barrier
Function (CBF)** filter that runs on the real FR3 in
`franka_experiments/nodes/cbf_safety_filter.py`. A policy trained here against
the shield meets the identical safety filter on hardware → *safe exploration*
in sim, *safe execution* on the robot.

See `../franka_sim_to_real_roadmap.md` for the full architecture and roadmap.

## Layout

```
franka_sim/
├── config.yaml                 # ONE config: task, CBF (mirrors fr3_control.yaml), RL
├── assets/franka_fr3/          # Menagerie FR3 MJCF (vendored, unmodified)
│   └── scene_cbf.xml           # scene + mocap obstacle (human proxy) + target marker
├── envs/
│   ├── cbf_filter.py           # accel HOCBF QP (raw OSQP) — mirror of the real node
│   └── franka_cbf_env.py       # gymnasium.Env: FR3 reach + moving obstacle, CBF-shielded
├── train.py                    # SAC + CUDA + TensorBoard + checkpoints/eval/safety curves
├── export_onnx.py              # SAC actor → ONNX (validated vs SB3), for deployment
└── scripts/
    ├── validate_cbf.py         # reduced-model proof the shield holds d ≥ d_safe
    ├── validate_actuation.py   # regression guard: the action must control the arm
    └── evaluate_policy.py      # score a .onnx/.zip (or zero/random baseline)
```

Roadmap status: **Steps 1, 2 and 3 DONE** — environment + CBF, training + ONNX
export, and the ROS 2 deployment node
`franka_experiments/nodes/rl_policy_commander.py` (see
`../franka_sim_to_real_implementation_status.md`).

## Environment (`FrankaCBF-v0`)

| | |
|---|---|
| **Action** `a ∈ [−1,1]⁷` | nominal joint acceleration `q̈_nom = a · q̈_max` (the input the real `cbf_safety_filter` receives on `/NS_1/qddot_nom`) |
| **Observation** (24) | `[q(7), q̇(7), ee_pos(3), target(3), obstacle(3), d_min(1)]` |
| **Shield** | every step: `q̈_safe = CBF.filter(q, q̇, q̈_nom, obstacles)` before actuation |
| **Actuation** | `q̈_safe → τ = M(q)q̈ + C(q,q̇)q̇ + g(q)` (`mj_inverse`, recomputed every substep) — the same chain as `qddot_to_torque` + `rt_torque_controller` + firmware gravity on the robot |
| **Reward** | `−‖ee−target‖ + success − effort − CBF_intervention − slack − jerk`, collision penalty |
| **Episode** | 5 s @ 100 Hz; terminate on collision (`d<0`) or success; obstacle sweeps the workspace |

The obstacle is a kinematic sphere (`contype=0`, never a physical MuJoCo
contact); "collision" = surface distance `< 0`, handled by reward — exactly how
the real MultiLinkDistance pipeline treats the human point cloud.

## CBF filter (`cbf_filter.AccelCBFFilter`)

Same math as the robot: HOCBF barrier `h = d − d_safe` (relative degree 2),
per obstacle row `aᵢᵀq̈ + s ≥ −k1(aᵢᵀq̇) − k0·h̄ᵢ − ċᵢ` (soft, slack-relaxable),
hard state-limit box (velocity/position braking + slew continuity) and hard
workspace box, solved by raw OSQP. Gains (`k0=25, k1=10.5, d_safe=0.20, ρ=1000`)
and limits are read from `config.yaml`, kept in sync with `fr3_control.yaml`.

## Run (inside the `franka_ros2` container)

```bash
docker compose up -d && docker exec -it franka_ros2 /bin/bash
cd /ros2_ws/src && export PYTHONPATH=/ros2_ws/src MUJOCO_GL=egl

# env self-check + random rollout
python3 -m franka_sim.envs.franka_cbf_env

# CBF guarantee proof (PASS = shield stops at d_safe, passthrough penetrates)
python3 -m franka_sim.scripts.validate_cbf

# actuation regression guard — the action must actually control the arm.
# Run this after ANY change to step()/the MJCF: a broken actuation path makes
# every policy identical and no training metric shows it (see the status doc).
python3 -m franka_sim.scripts.validate_actuation

# train (SAC on GPU) — full run from config.yaml, or a quick smoke run
python3 -m franka_sim.train                              # full (2M steps)
python3 -m franka_sim.train --total-timesteps 5000 --exp-name smoke
tensorboard --logdir franka_sim/runs                     # convergence + safety/ curves

# export the trained actor to ONNX (validated against the SB3 policy)
python3 -m franka_sim.export_onnx --model franka_sim/models/<exp>/best_model.zip

# score the EXPORTED graph (the artifact the robot runs) in the same env
python3 -m franka_sim.scripts.evaluate_policy \
    --model franka_sim/models/<exp>/best_model.onnx --episodes 50

# ALWAYS score the baselines too — a policy that ties the zero-action baseline
# is not a policy, it is a broken plant (this is how the actuation bug surfaced)
python3 -m franka_sim.scripts.evaluate_policy --model zero   --episodes 50
python3 -m franka_sim.scripts.evaluate_policy --model random --episodes 50
```

`train.py` logs standard SB3 curves plus `safety/*` (collision rate, min/mean
surface distance, CBF-active fraction, intervention, slack) — the paper's
safe-exploration evidence. `export_onnx.py` traces only the Actor (obs→action in
[−1,1]⁷) so the robot needs only `onnxruntime`.

The training stack (torch+CUDA, mujoco, gymnasium, stable-baselines3, onnx,
osqp) is baked into the container image via the repo `Dockerfile`.

## Deploy (Step 3)

```bash
ros2 launch franka_experiments torque_control_stack.launch.py \
    motion_source:=rl start_move_group:=false \
    rl_onnx_model:=/ros2_ws/src/franka_sim/models/<exp>/best_model.onnx
```

`franka_experiments/nodes/rl_policy_commander.py` rebuilds the *identical*
24-dim observation from robot topics and publishes `q̈_nom = a·q̈_max` on
`/NS_1/qddot_nom`, straight into the same `cbf_safety_filter` the policy was
trained against. The shared contract (observation layout, action scaling,
obstacle-slot reconstruction, config-drift checks) lives in
`franka_experiments/utils/rl_policy.py` and is unit-tested by
`franka_experiments/test/test_rl_policy.py`.

**Keep `config.yaml` in sync with `franka_experiments/config/fr3_control.yaml`**
— `test_rl_policy.py::test_real_configs_are_in_sync` fails the build if the CBF
gains, workspace box or joint limits drift apart, and the deployment node warns
at startup if the frozen training config disagrees with the robot's limits.
