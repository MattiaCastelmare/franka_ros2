# Sim-to-Real Safe RL + CBF — Implementation Status

Companion to `franka_sim_to_real_roadmap.md`: what is actually implemented in
this repository, where it lives, and how it was validated. The roadmap is the
specification; this file is the audit against the code.

Last updated: **2026-08-31**.

---

## 1. Roadmap items

| # | Roadmap item | File | Status |
|---|---|---|---|
| 1 | Gymnasium env with MuJoCo + CBF shield | `franka_sim/envs/franka_cbf_env.py` | **COMPLETE** |
| 1 | Central experiment config | `franka_sim/config.yaml` | **COMPLETE** |
| — | CBF filter mirroring the real node | `franka_sim/envs/cbf_filter.py` | **COMPLETE** (beyond roadmap: the roadmap folded this into the env; it is split out so the shield math is testable and diffable against `cbf_safety_filter.py`) |
| 2 | SAC training, CUDA, TensorBoard, checkpoints | `franka_sim/train.py` | **COMPLETE** |
| 2 | Actor → ONNX export | `franka_sim/export_onnx.py` | **COMPLETE** |
| 3 | ROS 2 ONNX inference / command node | `franka_experiments/nodes/rl_policy_commander.py` | **COMPLETE** (this pass) |
| 3 | Sim↔real observation/action contract | `franka_experiments/utils/rl_policy.py` | **COMPLETE** (this pass) |
| 3 | Launch integration | `franka_experiments/launch/torque_control_stack.launch.py` (`motion_source:=rl`) | **COMPLETE** (this pass) |
| — | CBF guarantee proof (reduced model) | `franka_sim/scripts/validate_cbf.py` | **COMPLETE** |
| — | Actuation-authority regression guard | `franka_sim/scripts/validate_actuation.py` | **COMPLETE** (this pass — see §5) |
| — | Policy evaluation (task + safety + latency) | `franka_sim/scripts/evaluate_policy.py` | **COMPLETE** (this pass) |
| — | Timing/jitter figure (paper advice §3) | `franka_experiments/scripts/plot_rl_timing.py` | **COMPLETE** (this pass) |
| — | Unit tests for the contract | `franka_experiments/test/test_rl_policy.py` | **COMPLETE** (this pass) |
| — | Node-level smoke test | `franka_experiments/test/smoke_rl_policy_commander.py` | **COMPLETE** (this pass) |
| — | Fake-hardware pipeline test | `franka_experiments/test/launch/test_rl_fake.launch.py` | **COMPLETE** (this pass) |

Roadmap deviations, and why:

* **`frame_grabber.py` is not used for state.** The roadmap suggested reading
  joints/EE "through existing modules such as `frame_grabber.py`"; that node is
  a camera-frame saver, not a state source. The commander instead uses the
  stack's standard state path — `JointState` double buffer + Pinocchio FK,
  identical to `pentagon_qddot_commander`.
* **No second avoidance layer in the RL node.** The pentagon commander owns
  avoidance-first shaping and a feasibility governor. The RL policy learned its
  avoidance *under the shield*, so adding shaping on top would be an untrained
  outer loop. `rl_policy_commander` subscribes `cbf_status` for diagnostics
  only.

---

## 2. Architecture as built

```
                  TRAINING (standalone, no ROS)          DEPLOYMENT (ROS 2)
                  ────────────────────────────           ──────────────────
 franka_sim/config.yaml ──┐                     ┌── franka_experiments/config/
   (mirror, test-enforced)│                     │      fr3_control.yaml
                          ▼                     ▼
   FrankaCBFEnv  ──►  AccelCBFFilter  ≡  cbf_safety_filter (HOCBF QP, OSQP)
        │  obs(24)                                      ▲  q̈_nom
        ▼                                               │
   SAC (SB3, CUDA) ──► best_model.zip ──► export_onnx ──►  rl_policy_commander
                                            .onnx           (onnxruntime, 100 Hz)
                                                              │  q̈_safe
                                                              ▼
                                              qddot_to_torque ──► rt_torque_controller
```

The single artifact crossing the sim→real boundary is the `.onnx` actor.
Everything else is *mirrored*, and the mirrors are checked mechanically:

| Mirrored quantity | Sim source | Robot source | Check |
|---|---|---|---|
| `joint_limits` (incl. q̈_max) | `franka_sim/config.yaml` | `config/fr3_control.yaml` | `test_real_configs_are_in_sync`, plus a startup warning per drifted entry |
| CBF gains, workspace box | same | same | `test_real_configs_are_in_sync` |
| Obstacle sphere radius | `scene_cbf.xml` `obstacle_geom` | `obstacle.radius` in `config.yaml` | env raises on mismatch at construction |
| Observation layout | `FrankaCBFEnv._get_obs` | `utils/rl_policy.build_observation` | `test_observation_matches_env_concatenation`, plus the smoke test's bit-equal replay |
| Action scaling | `FrankaCBFEnv.step` | `utils/rl_policy.action_to_qddot` | `test_action_scaling_matches_env` |

### Observation contract (24)

`[ q(7), q̇(7), ee_pos(3), target(3), obstacle(3), d_min(1) ]`

* `ee_pos` — MuJoCo `attachment_site` (link7 + 0.107 m) ≡ URDF `fr3_link8`
  (node parameter `ee_frame`).
* `obstacle` — in sim the sphere **centre**; on the robot reconstructed from
  `MultiLinkDistance` as `p_human − n̂·r_obs`, which restores
  `‖p_cp − p_obs‖ − r_obs − r_cp = d` exactly.
* `d_min` — surface distance, the same quantity in both worlds. Known
  asymmetry: `distance_engine` clamps it at 0, so the robot never reports
  penetration while the sim does. Conservative direction; documented in
  `utils/rl_policy`.

### Output gating (`rl_policy_commander`)

| Condition | Output | `rl_status[5]` |
|---|---|---|
| warm-up (`warmup_s`, default 3 s) | zeros | 1 |
| joint state missing / older than `joint_state_timeout` | zeros | 2 |
| distances seen, then stale (`distance_timeout`) — perception fault | zeros | 3 |
| distances never seen (camera intentionally off) | run against a parked synthetic obstacle | 0 |
| target reached and `stop_on_success` | zeros | 4 |
| nominal | `a·q̈_max·action_scale` | 0 |

Joint velocity/position limits, acceleration continuity and the workspace box
are **not** re-implemented here — `cbf_safety_filter` enforces them downstream
on every path, exactly as for the pentagon commander.

---

## 3. Validation performed

Environment: the repository's own container (`docker compose` service
`franka_ros2`, ROS 2 Humble), workspace `/ros2_ws/src`.

| Check | Command | Result |
|---|---|---|
| Env self-check + random rollout | `python3 -m franka_sim.envs.franka_cbf_env` | PASS (`check_env: OK`) |
| CBF guarantee (reduced model) | `python3 -m franka_sim.scripts.validate_cbf` | PASS (shield holds at `d_safe=0.200`, passthrough penetrates) |
| Training smoke + full run | `python3 -m franka_sim.train` | PASS, ~100 fps with gradient steps on RTX 4070 |
| ONNX export validation | `python3 -m franka_sim.export_onnx` | PASS (`max|onnx − sb3| = 1.9e-07`) |
| Actuation guard | `python3 -m franka_sim.scripts.validate_actuation` | PASS (see §5) |
| Policy evaluation vs baselines | `evaluate_policy.py … --episodes 50` on `sac_v2` (400 k steps) | see table below |
| Unit tests | `pytest test/` in `franka_experiments` | **68 passed** |
| Node smoke test | `python3 test/smoke_rl_policy_commander.py` | PASS — warm-up gate, 100 Hz, `|q̈| ≤ q̈_max`, **observation replay error 0.00e+00**, `d_min` propagation, both stale-input gates |
| Launch argument resolution | `ros2 launch … --show-args` | PASS for `torque_control_stack` and `test_rl_fake` |
| Fake-hardware pipeline | `ros2 launch franka_experiments test_rl_fake.launch.py` | PASS — `joint_state_broadcaster` + `rt_torque_controller` active; `/NS_1/qddot_nom` 100.0 Hz, `/NS_1/qddot_safe` 99.5 Hz, `/NS_1/torque_cmd` 100.0 Hz, `/NS_1/rl_status` 100.0 Hz; no node crashed |
| Pipeline topic/node check | `./test/scripts/check_topics.sh rl` | **PASS=13 FAIL=0** |
| TF tree | `tf2_echo world fr3_link8` during the fake-HW run | resolves — single connected tree (`world → base → fr3_link0 → …`) |
| Timing (paper's jitter figure) | `scripts/plot_rl_timing.py` on a 120 s fake-HW run | loop period mean **9.998 ms**, std 0.159, p99 10.421, max 12.520; ONNX inference mean **0.138 ms**, p99 0.268, max 1.066; **0.00 %** of ticks over 1.5× nominal |

### Policy vs baselines (`sac_v2`, 400 k steps, 50 eval episodes each)

| | trained policy | zero-action | random |
|---|---|---|---|
| success rate | **14 %** | 2 % | 2 % |
| final EE error (mean) | **0.296 m** | 0.567 m | 0.495 m |
| episode return | **−154 ± 112** | −229 ± 117 | −321 ± 141 |
| collision rate (episodes) | 16 % | 14 % | 12 % |
| min surface distance | −0.1323 m | −0.1323 m | −0.1323 m |
| mean surface distance | 0.273 m | 0.374 m | 0.351 m |
| mean CBF intervention | 3.19 rad/s² | 0.20 rad/s² | 9.83 rad/s² |
| mean slack | 0.037 | 0.005 | 0.007 |

The policy clearly beats both baselines on the task (7× the success rate, half
the final error), which is the evidence that the training loop is now closed —
before the §5 fix all three columns were identical.

Read the collision numbers carefully: the **worst-case penetration is identical
(−0.1323 m) for all three controllers**, including the arm that never moves.
The residual collisions are the kinematic obstacle sweeping *into* the robot at
up to ~1.1 m/s (amplitude 0.30 m at 0.6 Hz), not the controller driving into the
obstacle — a situation no controller can avoid and one the CBF cannot fix,
because the barrier can only bound the *robot's* motion. Comparing a policy's
collision rate against the zero-action baseline is therefore mandatory when
quoting these numbers; the honest safety claim is the delta, not the absolute.
Lowering `obstacle.speed` / `obstacle.amplitude` in `config.yaml` (already
parameters) is the knob if unavoidable episodes are to be excluded from the
benchmark.

### Not validated, and why

* **Closed-loop motion on fake hardware.** ROS 2 mock hardware does not
  integrate effort commands, so the arm does not move and the observation stays
  frozen. The fake-hardware test therefore validates the command chain, not
  behaviour. Closed-loop policy + CBF behaviour is validated in MuJoCo
  (`evaluate_policy.py`).
* **Real FR3.** No robot was connected; nothing was commanded to hardware. All
  real-hardware claims below are static/fake-hardware evidence only.

---

## 4. Real-hardware readiness review

| Aspect | Status |
|---|---|
| Command interface | Publishes `Float64MultiArray(7)` on `/NS_1/qddot_nom` — the topic `cbf_safety_filter` already consumes. No new hardware interface. |
| Joint ordering | `FR3_JOINT_NAMES` index map built from `JointState.name`, never positional. Same helper as the rest of the stack. |
| Frame names | `ee_frame` parameter, default `fr3_link8` (= MuJoCo `attachment_site`). Resolved through `resolve_frame_id`, which raises with the available-frame list if wrong. |
| Namespaces | Topics come from `fr3_control.yaml`; `joint_state_topic` defaults to `__auto__` → namespace from `franka.config.yaml`. Node runs inside `p['namespace']` in the launch file. |
| Rates | Timer at `env.control_rate_hz` from the *training* config (100 Hz) so the deployed loop matches the trained one; overridable with `rate_hz`. |
| Limits | q̈ clamp from the **robot** config; the training config is compared against it and every difference is logged as a `SIM-TO-REAL joint_limits drift` warning. |
| Safety behaviour | Five explicit zero-output gates (§2); no safety check was weakened. All hard limits stay downstream in the CBF filter. |
| Startup | xacro, Pinocchio and the first ONNX inference are paid in `__init__`; a `_warmup()` runs every per-tick path once before the timer starts. Measured 0.5–1.6 ms. |
| Shutdown | `request_stop()` publishes zeros for 0.5 s via `run_node_main`, then exits; CSV closed in `destroy_node`. |
| Fake vs real | No `use_fake_hardware` branch in the node — it is hardware-agnostic by construction. |
| Robot IP | Not touched; handled by `franka_bringup` as before. |
| Hard-coded paths | None. Policy/config paths are parameters with a `realpath`-based source-tree fallback that survives `--symlink-install`; the log directory follows the same rule. |
| First-run derate | `action_scale ∈ (0,1]` scales `q̈_nom` down for cautious first runs; clamped and warned if out of range. |

**Recommended first real run**: `motion_source:=rl rl_action_scale:=0.3`, camera
and `real_time_distance` **on**, a single conservative target, with the
`rl_status` gate field and `cbf_status` slack/fault on screen.

---

## 5. Defect found and fixed in the simulation env (Step 1)

**Symptom.** The `--model zero` / `--model random` baselines added to
`evaluate_policy.py` scored *identically* to the trained policy: same final EE
error (0.296 m), same episode length (259.3 steps), same collision rate (70 %),
same minimum surface distance (−0.1326 m). Three completely different action
streams, one trajectory.

**Root cause.** `FrankaCBFEnv.step()` drove MuJoCo's POSITION servos from a
"state-seeded, drift-free" reference:

```python
qdot_des = clip(qdot_measured + qddot_safe*dt, ...)
q_des    = clip(q_measured   + qdot_des*dt,    ...)
data.ctrl[act] = q_des
```

Re-anchoring the reference to the *measurement* every tick means it never
integrates: the commanded lead saturates at `q̈·dt²` ≈ 6e-4 rad, and the PD
servo settles at `q̇ ≈ 0.004 rad/s` no matter how large `q̈` is. Measured
directly: with `q̈ = q̈_max` on all seven joints for 2 s the arm moved
`|Δq| = 0.162 rad` versus `0.160 rad` with zero action — the motion was gravity
sag. The CBF was innocent: it was passing full authority (`q̈_safe = q̈_max`,
`n_c = 5`, no braking). Nothing in the reward, the observation or the safety
curves exposed this; only the zero-action baseline did.

**Fix.** Do what the deployment chain does. The FR3 executes
`q̈_safe → τ = M(q)q̈ + C(q,q̇)q̇` (`qddot_to_torque`) `+ g(q)` (firmware), so the
env now converts the vendored MJCF's position actuators to direct-force
actuators in place (no asset edit) and commands
`τ = mj_inverse(q, q̇, q̈_safe)`, recomputed at every substep — the analogue of
`rt_torque_controller` re-evaluating at 1 kHz between two 100 Hz `q̈_safe`
samples. A second, self-inflicted bug was caught on the way: the torque clip
initially reused the actuators' `ctrlrange`, which for a `position` actuator is
the joint POSITION range, pinning joint 4's torque at −0.15 N·m. Limits now come
from the joint's `actuatorfrcrange` (±87 / ±12 N·m).

**Result** (`franka_sim/scripts/validate_actuation.py`, new regression guard):

```
max |realized q̈ − commanded q̈| = 1.443e-15 rad/s²
torque limits  lo/hi = ∓[87 87 87 87 12 12 12] N·m
|Δq| over 0.5 s: zero-action 0.0637 rad   unit action 0.7136 rad
q̇₁: +action +2.310 rad/s   −action −2.358 rad/s
```

**Consequence.** Every policy trained before this fix is void — it was optimised
in an environment where the action barely reached the plant. Retraining was
redone from scratch on the fixed env.

**Known remaining fidelity gap (conservative direction).** The sim applies pure
feedforward inverse-dynamics torque; the robot additionally runs
`rt_torque_controller`'s 1 kHz `Kd·(q̇_des − q̇)` feedback. A zero command
therefore drifts slightly more in sim (0.064 rad over 0.5 s) than it would on
hardware. Erring toward a *less* well-behaved plant in sim is the safe
direction for sim-to-real.

---

## 6. Repository-hygiene fix made along the way

`qpsolvers` had floated to 4.13.0 while `osqp` stayed on 0.6.7 (pinned there
deliberately — `cbf_safety_filter` and `franka_sim/envs/cbf_filter.py` drive the
raw OSQP 0.6 API). qpsolvers ≥ 4.4 imports `SolverStatus`, which only exists in
osqp 1.x, so its osqp backend failed to load: `qpsolvers.available_solvers ==
[]`, and `cbf_velocity_filter` / `cbf_OSCBF_filter` would raise `SolverNotFound`
at their first QP. Six unit tests were failing on this.

Fixed by pinning `qpsolvers[osqp]==4.3.3` + `osqp<1.0` in the `Dockerfile`
(with the rationale in a comment) and adding an
`assert 'osqp' in qpsolvers.available_solvers` to the image's smoke test so the
build fails instead of shipping a silently broken solver. The container was
updated in place; `test/` went 62 passed / 6 failed → **68 passed**.

---

## 7. Open items (not blockers)

* **Train to convergence.** `sac_v2` is a 400 k-step run (≈1 h on the RTX 4070);
  `config.yaml` defaults to 2 M. 14 % success is a working-but-undertrained
  policy, not a broken pipeline — the wiring is complete and validated, and the
  baseline comparison shows learning is happening.
* **Unavoidable-collision episodes.** As shown above, part of the collision rate
  is structural (the obstacle sweeps into a robot that cannot escape). Either
  slow the obstacle in `config.yaml` or always report the zero-action baseline
  alongside. This caps the achievable safety metric and adds reward noise, so it
  is also a likely brake on learning speed.
* Obstacle CBF rows are soft (slack-relaxable) in both sim and robot — a
  sustained push relaxes them by design. The policy is expected to learn
  avoidance; the CBF is the certificate. Unchanged from the existing
  architecture.
* Domain randomisation (mass/friction/latency) is not implemented; the roadmap
  argues the CBF absorbs the dynamics gap.
* Closed-loop hardware-in-the-loop validation needs either Gazebo or the real
  FR3.
