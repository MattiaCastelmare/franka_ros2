# Sim-to-Real Safe RL + CBF — Implementation Status

The reference document for the Safe RL + CBF sim-to-real work: what is
implemented, where it lives, how it was validated, and what is left. §1 keeps
the original three-step specification as a table, so this file is self-contained.

Last updated: **2026-09-17**.

> **How this file goes stale, and how to tell.** It did once, badly. Between
> 2026-09-14 and 2026-09-17 the robot side gained the ISO 10218 layer, an
> acceleration-authority cap, the firmware velocity envelope and a state
> governor, while `franka_sim/` stood still — and this document went on
> claiming the two shields were equivalent. §11 is the changelog that exists so
> the next divergence shows up as a diff rather than as a re-audit. The
> mechanical check is `pytest test/test_rl_policy.py`: it fails when the two
> configs drift, when a robot shield family is not declared, or when the EE
> frames stop naming the same point.

---

## 1. Roadmap items

| # | Roadmap item | File | Status |
|---|---|---|---|
| 1 | Gymnasium env with MuJoCo + CBF shield | `franka_sim/envs/franka_cbf_env.py` | **COMPLETE** |
| 1 | Central experiment config | `franka_sim/config.yaml` | **COMPLETE** |
| — | CBF filter mirroring the real node | `franka_sim/envs/cbf_filter.py` | **COMPLETE, but a documented SUBSET** — see §2.1 |
| 2 | SAC training, CUDA, TensorBoard, checkpoints | `franka_sim/train.py` | **COMPLETE** |
| 2 | Actor → ONNX export | `franka_sim/export_onnx.py` | **COMPLETE** |
| 3 | ROS 2 ONNX inference / command node | `franka_experiments/nodes/rl_policy_commander.py` | **COMPLETE** |
| 3 | Sim↔real observation/action contract | `franka_experiments/utils/rl_policy.py` | **COMPLETE** |
| 3 | Launch integration | `torque_control_stack.launch.py` (`motion_source:=rl`) | **COMPLETE** |
| — | CBF guarantee proof (reduced model) | `franka_sim/scripts/validate_cbf.py` | **COMPLETE** |
| — | Actuation-authority regression guard | `franka_sim/scripts/validate_actuation.py` | **COMPLETE** |
| — | Policy evaluation (task + safety + latency) | `franka_sim/scripts/evaluate_policy.py` | **COMPLETE** |
| — | Benchmark table with baseline deltas | `franka_sim/scripts/benchmark.py` | **COMPLETE** (2026-09-17) |
| — | Timing/jitter figure | `franka_experiments/scripts/plot_rl_timing.py` | **COMPLETE** |
| — | Unit tests for the contract | `franka_experiments/test/test_rl_policy.py` | **COMPLETE** (29 tests) |
| — | `franka_sim` regression suite | `franka_sim/tests/` | **COMPLETE** (2026-09-17, 38 tests) |
| — | Node-level smoke test | `franka_experiments/test/smoke_rl_policy_commander.py` | **COMPLETE** |
| — | Fake-hardware pipeline test | `franka_experiments/test/launch/test_rl_fake.launch.py` | **COMPLETE** |
| — | Domain randomisation / latency / obs noise | `franka_sim/envs/randomization.py` | **COMPLETE, default OFF** (2026-09-17) — §2.2 |
| — | Actuation feedback fidelity (Kd/Kp/ffScale) | `FrankaCBFEnv._servo_torque` | **COMPLETE, default OFF** (2026-09-17) — §2.2 |
| — | Optional-fidelity regression guards | `franka_sim/tests/test_optional_fidelity.py` | **COMPLETE** (12 tests) |

Roadmap deviations, and why:

* **`frame_grabber.py` is not used for state.** That node is a camera-frame
  saver, not a state source. The commander uses the stack's standard state
  path — `JointState` double buffer + Pinocchio FK, identical to
  `pentagon_qddot_commander`.
* **No second avoidance layer in the RL node.** The policy learned its
  avoidance *under the shield*, so shaping on top would be an untrained outer
  loop. `rl_policy_commander` subscribes `cbf_status` for diagnostics only.

---

## 2. Architecture as built

```
                  TRAINING (standalone, no ROS)          DEPLOYMENT (ROS 2)
                  ────────────────────────────           ──────────────────
 franka_sim/config.yaml ──┐                     ┌── franka_experiments/config/
   (mirror, test-enforced)│                     │      fr3_control.yaml
                          ▼                     ▼
   FrankaCBFEnv  ──►  AccelCBFFilter  ⊂  cbf_safety_filter (HOCBF QP, OSQP)
        │  obs(51)                                      ▲  q̈_nom
        ▼                                               │
   SAC (SB3, CUDA) ──► best_model.zip ──► export_onnx ──►  rl_policy_commander
                                            .onnx           (onnxruntime, 100 Hz)
                                                              │  q̈_safe
                                                              ▼
                                              qddot_to_torque ──► rt_torque_controller
```

Note the **⊂**, not **≡**. See §2.1.

The single artifact crossing the sim→real boundary is the `.onnx` actor.
Everything else is *mirrored*, and the mirrors are checked mechanically:

| Mirrored quantity | Sim source | Robot source | Check |
|---|---|---|---|
| `joint_limits` (incl. q̈_max) | `franka_sim/config.yaml` | `config/fr3_control.yaml` | `test_real_configs_are_in_sync`, plus a startup warning per drifted entry |
| CBF gains, `d_safe`, box shape | same | same | `test_real_configs_are_in_sync` (14 keys) |
| Acceleration cap vs action scale | `cbf.qddot_max_abs` | `params.qddot_max_abs` | `test_accel_cap_applies_to_the_box_and_not_to_the_action` |
| Shield family coverage | `cbf.shield_parity` | every `enable_*`/`*_enabled` | `test_shield_families_are_declared` |
| Workspace box (sim-only) | `cbf.ws_*` | absent | `test_workspace_box_is_sim_only` |
| Obstacle sphere radius | `scene_cbf.xml` `obstacle_geom` | `obstacle.radius` in `config.yaml` | env raises on mismatch at construction |
| Observation layout | `franka_sim/envs/obs_layout.py` | `utils/rl_policy.ObsSpec` (mirror) | `test_observation_layout_mirrors_franka_sim` + the smoke test's bit-equal replay |
| Action scaling | `FrankaCBFEnv.step` | `utils/rl_policy.action_to_qddot` | `test_action_scaling_matches_env` |
| EE frame | `env.ee_site` | node `ee_frame` | `test_ee_frame_matches_sim_ee_site` |

### 2.1 The shield is a SUBSET, and that is the honest claim

The robot's `cbf_safety_filter` has grown well past the sim's shield. Mirroring
it would mean maintaining a second copy of a ~6900-line subsystem that is still
changing, so the asymmetry is deliberate — but it is **declared**, in
`franka_sim/config.yaml` under `shield_parity:`, and
`test_shield_families_are_declared` fails when a robot flag is added, removed,
or flipped without being reclassified.

| Mechanism | Robot (as launched) | Sim |
|---|---|---|
| Obstacle HOCBF rows | ✅ per-family slack, `multi_obstacle_k: 2`, 11 CPs | ✅ one shared slack, 1 row/CP, 6 CPs |
| Hard state box + slew | ✅ | ✅ (same math, same parameters) |
| Self-collision rows | ✅ | ❌ |
| Joint-limit rows | ✅ (soft rows) | ❌ (box only) |
| Singularity row (σ_min) | ✅ | ❌ |
| Retreat cap, link-speed rows | ✅ | ❌ |
| State governor (scales q̈_nom) | ✅ | ❌ |
| Zone ladder, velocity standoff | ✅ | ❌ |
| Lateral/outrun evasion, livelock escape | ✅ (objective biases) | ❌ |
| `v_obs` in ḣ, uncertainty margin | ✅ | ❌ |
| Tangential bias (`cbf_tangential_gain: 0.5`) | ✅ | ❌ |
| Workspace box | ❌ | ✅ (sim-only, conservative) |
| ISO 10218 layer | present, `iso_enabled: false` | ❌ |

**Say "trained under a subset of the deployed shield", never "trained under the
deployed shield".** Everything in the robot-only rows can bend, attenuate or
veto the policy's `q̈_nom` on hardware in a way it never met in training. The
CBF remains the certificate either way; what changes is what the *policy* can
be said to have learned.

A related trap: seven of those flags are `false` in `fr3_control.yaml` and
`true` in `launch_defaults.yaml`. **A run's actual configuration can only be
read from the launch file**, not from the YAML.

### 2.2 Optional fidelity layers — both ship OFF

Two mechanisms exist but are disabled by default, because turning either on
changes the training distribution and therefore invalidates every number in
§3.1. `test_both_layers_off_is_bit_identical_to_no_config` asserts that with
them off the environment is byte-for-byte the one `sac_v4` trained on.

**Actuation law** (`actuation.enabled`). The default path applies pure
feedforward inverse dynamics. On, it mirrors `rt_torque_controller.cpp` term
for term: an integrated velocity reference clamped to the firmware envelope,
anti-windup clamps on both the velocity (`e_max`) and position (`p_max`)
errors, a position loop built on q̈_SAFE (never the nominal — that would fight
the barrier), and `ffScale`, a directional smoothstep that fades only a
feedforward pushing further into the envelope.

Gravity is deliberately outside the faded term: `mj_inverse` returns
M q̈ + C q̇ + g, the robot's τ_ff is that minus g(q), and g(q) is added back
unscaled. A gate that scaled gravity would drop the arm exactly when it
engaged. `test_actuation_on_does_not_fade_gravity` pins this.

Residual gap: MuJoCo substeps at 2 ms (500 Hz) against the controller's 1 kHz.
`env.sim_timestep: 0.001` closes it at 2× the compute.

**Randomisation** (`randomization.enabled`, plus a per-block switch), ranked as
§10 ranks the gaps: latency, observation noise, joint sensor noise, dynamics.

All of it is applied to the **observation only** — never to the CBF rows, which
keep MuJoCo's true geometry. The robot's perception is noisy, but its filter
still treats what it receives as the truth; corrupting the shield's own inputs
would simulate a *worse* safety filter than the one deployed. It is the policy
that must become robust to a noisy estimate.
`test_noise_never_touches_the_cbf_rows` asserts the trajectory is unchanged
while the observation stream is not.

The obstacle path reproduces the real chain in order: delay, then sensor noise,
then the engine's LPF, then the clamp at zero — the clamp last, because that is
where `distance_engine` puts it, so the simulated robot never reports
penetration either. A dropped frame holds the last good value rather than
teleporting the obstacle.

### Observation contract (51, extended 2026-09-18)

```
[ q(7), q̇(7), ee_pos(3), target(3), obstacle(3), d_min(1) ]   base, slots 0..23
+ v_obs(3)                        `obs.obstacle_velocity`
+ [dᵢ, n̂ᵢ] × 6                    `obs.control_point_geometry`
```

The layout lives in **`franka_sim/envs/obs_layout.py`** and is mirrored by
`utils/rl_policy.ObsSpec`; `test_observation_layout_mirrors_franka_sim` loads
the sim module off disk and fails on any one-sided edit.

Both blocks are **prefix extensions**: slots 0..23 keep their meaning and their
offsets. `rl_policy_commander` resolves the spec from the `config.yaml` frozen
*beside the policy*, so `sac_v2/v3/v4` still deploy as 24-dim through the same
node, unflagged — verified, and the smoke test's replay error is 0.00e+00 on
both widths.

* `ee_pos` — MuJoCo `hand_tcp_site` ≡ URDF `fr3_hand_tcp`, the Franka Hand
  grasp centre (node parameter `ee_frame`). MuJoCo and Pinocchio agree to
  7e-16 m. `test_ee_frame_matches_sim_ee_site` fails if the two sides ever name
  different points.
* `obstacle` — in sim the sphere **centre**; on the robot reconstructed from
  `MultiLinkDistance` as `p_human − n̂·r_obs`, which restores
  `‖p_cp − p_obs‖ − r_obs − r_cp = d` exactly.
* `d_min` — surface distance. Known asymmetry: `distance_engine` clamps it at
  0, so the robot never reports penetration while the sim does. Conservative
  direction; documented in `utils/rl_policy`. Now also clipped above at
  `clip_distance`.
* `v_obs` — finite difference of the **observed** centre, so latency, noise and
  the LPF propagate into it exactly as they do on the robot, which
  differentiates the estimate it receives rather than any ground truth. Zero on
  the first tick of an episode and after a perception outage.
* `[dᵢ, n̂ᵢ]` — per control point, in `cbf.control_points` order, and literally
  the numbers the CBF rows are built from (`test_control_point_geometry_is_
  what_the_cbf_rows_are_built_from`). The robot reports several control points
  per link; the nearest one per link maps onto the sim's single sphere at the
  body origin. A link with nothing near it reads `(clip_distance, 0, 0, 0)` —
  a zero normal is a "no direction known" token, and both sides emit it.
  `robot_link` in `cbf.control_points` carries the sim→URDF name map
  (`fr3_hand` → `fr3_link8`).

#### Why these two blocks exist

**The state was not Markovian.** The obstacle moves and the base layout carried
only its position. From one frame "approaching" and "receding" are the same
observation with opposite optimal actions. That is a state defect, not a
learning one, and no choice of RL algorithm recovers from it.

**The policy was less informed than the filter it must pre-empt.** The shield
reasons over 6×(dᵢ, n̂ᵢ); the policy received `min(dᵢ)` and a sphere centre. It
could not tell WHICH link was threatened, nor which way it was about to be
pushed — exactly what "go around on this side" requires. Choosing a side is a
global decision; the QP is a one-step local projection along n̂ and cannot make
it. `_build_obstacles` already computed all of it, so the block costs nothing.

It also changes what the observation can **describe**. `obstacle(3)` presumes
one sphere of a radius fixed at training time; `(dᵢ, n̂ᵢ)` per link is "how far
is matter from this link, and in which direction" — agnostic to obstacle count
and shape, and what the point-cloud pipeline natively produces. This is the
block that makes a policy scene-agnostic rather than single-sphere.

**This invalidates trained policies**: the input width changed. `sac_v4` remains
the result of record until a retrain under the new layout is benchmarked.

### Action contract, and the acceleration cap

`a ∈ [−1,1]⁷ → q̈_nom = a · q̈_max · action_scale`, with **q̈_max the uncapped
per-joint limit** (17 rad/s² on joints 5 and 7).

The cap is a separate thing and applies to the **QP box only**:
`qddot_max_abs: 10.0`. This is not a simplification — it is exactly the robot's
arrangement, where `rl_policy_commander` scales by `fr3_control.yaml`'s
`joint_limits` and `cbf_safety_filter`'s box then clips. Capping both would
change what `a = 1` means; capping neither is the bug §5.3 describes.

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
are **not** re-implemented here — `cbf_safety_filter` enforces them downstream.

---

## 3. Validation performed

Environment: the repository's own container (`docker compose` service
`franka_ros2`, ROS 2 Humble), workspace `/ros2_ws/src`.

| Check | Command | Result |
|---|---|---|
| Env self-check + random rollout | `python3 -m franka_sim.envs.franka_cbf_env` | PASS (`check_env: OK`; full 500 steps, `min d_min=0.101`, no collision) |
| CBF guarantee (reduced model) | `python3 -m franka_sim.scripts.validate_cbf` | PASS (shield holds at `d_safe=0.150`, passthrough penetrates to −2.25) |
| Actuation guard | `python3 -m franka_sim.scripts.validate_actuation` | PASS — see §5.1 |
| `franka_sim` regression suite | `python3 -m pytest franka_sim/tests -q` | **38 passed** |
| ONNX export validation | `python3 -m franka_sim.export_onnx --model .../sac_v3/best_model.zip` | PASS (`max\|onnx − sb3\| = 5.7e-06`) |
| ROS-side unit tests | `pytest test/` in `franka_experiments` | **915 passed, 0 failed** |
| Node smoke test | `python3 test/smoke_rl_policy_commander.py` | PASS — warm-up gate, 100 Hz, `\|q̈\| ≤ q̈_max`, **observation replay error 0.00e+00**, `d_min` propagation, both stale-input gates |
| Launch argument resolution | `ros2 launch … --show-args` | PASS — 64 args on `torque_control_stack`, and on `test_rl_fake` |
| Fake-hardware pipeline | `ros2 launch franka_experiments test_rl_fake.launch.py` | PASS with `sac_v4` — 9 nodes; `joint_state_broadcaster` + `rt_torque_controller` **active**; `/NS_1/qddot_nom` 100.0 Hz, `/NS_1/qddot_safe` 99.7 Hz, `/NS_1/torque_cmd` 99.8 Hz, `/NS_1/rl_status` 100.0 Hz; `rl_status` gate 0 |
| Pipeline topic/node check | `./test/scripts/check_topics.sh rl` | PASS=13 FAIL=0 |
| Timing (jitter figure) | `scripts/plot_rl_timing.py` on a 105 s fake-HW run | loop period mean **10.002 ms**, std 0.325, p50 10.000, p99 10.686; ONNX inference mean **0.177 ms**, p99 0.340; **0.01 %** of ticks over 1.5x nominal |

### 3.1 Policy vs baselines

`benchmark.py`, 50 episodes per controller, seed 12345, obstacle regime
0.20 Hz × 0.20 m (peak 0.251 m/s), `d_safe = 0.15`, reset clearance = `d_safe`.

**`sac_v4` (2 M steps, trained on the fixed contract) — the result of record:**

| | trained policy | zero-action | random | Δ vs zero |
|---|---|---|---|---|
| success rate | **86.0 %** | 0.0 % | 4.0 % | — |
| final EE error (mean) | **0.0665 m** | 0.3740 m | 0.5023 m | — |
| episode return | **−24.6** | −142.9 | −302.2 | — |
| episode length | 148.5 | 500.0 | 483.9 | — |
| collision rate | **0.0 %** | 0.0 % | 0.0 % | +0.0 pp |
| min surface distance | **+0.0807 m** | +0.0561 m | +0.0838 m | +0.0245 m |
| mean surface distance | 0.2241 m | 0.2364 m | 0.3047 m | −0.0123 m |
| mean CBF intervention | **1.876** | 0.032 | 9.364 | — |
| mean slack | 0.00263 | 0.00004 | 0.00040 | — |

**The shield held `d > 0` in 150/150 episodes across three controllers**,
including a random one. Before the §5.2 fixes, all three penetrated to an
identical −0.1467 m.

**The policy avoids, it does not lean on the barrier.** Over 2495 steps of
deterministic rollout the obstacle rows were active on every step (6–9 rows),
yet:

* the CBF intervention was **exactly zero on 53.7 % of steps** (median 0.000),
  and above 1 rad/s² on 37.2 %;
* the **minimum `d_min` over the whole run was +0.1167 m** — the arm never
  reached `d_safe`, let alone crossed it;
* only **3.0 %** of steps were inside `d_safe = 0.15 m` at all.

That is the division of labour the architecture claims: the policy owns the
avoidance, the CBF is the certificate that almost never has to fire. Compare
`mean intervention` 1.876 against `sac_v3`'s 3.016 and the random policy's
9.364 on the identical benchmark.

**Training cost of the hardware-faithful contract.** `sac_v4` first reached
`eval success ≥ 0.5` at **1.00 M steps**, against `sac_v3`'s **375 k** — 2.7×
slower to take off. The two are then indistinguishable: over the matched window
1.3 M–1.57 M, `sac_v3` scored 0.56 and `sac_v4` 0.58. The dominant cause is the
`qddot_max_abs` cap (§5.3), the one change in the re-sync that *removes* action
authority rather than adding a geometric constraint. **The cap costs training
time, not final performance** — worth stating explicitly, because it is the
argument for keeping it.

**For contrast, `sac_v3` (trained pre-fix) on this same benchmark:** 82.0 %
success, 0.0 % collisions, intervention 3.016. It transfers well, which is
itself evidence that the contract changes did not break the task — but it was
optimised against a shield and a reset distribution that no longer exist, so it
is not the result of record.

**Historical** — the same `sac_v3` on the *pre-fix* benchmark scored 56.0 %
success, 20.0 % collisions and −0.1467 m worst penetration, identical on the
last two to both baselines. That is what a saturated safety metric looks like.

### 3.1.0 Half the benchmark needed no avoidance — read both columns

The table above samples the obstacle uniformly in its box, independently of
where the target is. Measured over those same 50 episodes: **the obstacle sweep
never came within `r_obs + d_safe` of the straight EE→target line in 25 of
them, and actually intersected it in 1.** Half the benchmark is solvable by
ignoring the obstacle and driving to the target, and the median path is only
0.270 m long.

So the 86 % is substantially a REACHING score. `task.blocking_fraction: 1.0`
forces every episode's obstacle onto the path; the same `sac_v4`, same seeds,
same shield:

| | uniform (50 % obstructed) | **always obstructed** |
|---|---|---|
| success rate | 86.0 % | **56.0 %** |
| final EE error | 0.0665 m | 0.1208 m |
| episode length | 148.5 | 272.4 |
| **collision rate** | 0.0 % | **0.0 %** |
| min surface distance | +0.0807 m | +0.0678 m |
| mean CBF intervention | 1.876 | **3.539** |

**The safety claim survives; the task claim does not.** Zero collisions either
way, with the shield doing roughly twice the work — but the success rate falls
30 points once avoidance is actually required. Quote the regime, always.

Note `sac_v4` was TRAINED at `blocking_fraction: 0.0`, so 56 % is a transfer
result onto a harder distribution, not its ceiling. Training at 0.5–0.7 (a mix,
so unobstructed reaches stay in the distribution) is the obvious next
experiment and is listed in §10.

### 3.1.1 The collision rate is ~0, not 0

`benchmark.py` reports 0.0 % over 50 episodes, and the deterministic rollout
above never left the safe set. The **training** log is the honest qualifier:
over 226 dumps of `sac_v4`'s run, **7 (3.1 %) recorded a negative
`min_surface_dist`**, worst −0.0043 m, i.e. roughly one step in 67 000.

Every one of those dumps also carries a `mean_slack` 5–20× the typical value.
That is the designed mechanism working, not a hole in the sampler: **obstacle
rows are soft**, the QP may pay slack to relax them, and under a stochastic
exploration policy it occasionally does. The comparison that matters is
−0.1467 m before the §5.2 fixes against −0.0043 m after — a factor of 34, and
no longer identical to the baselines.

So: quote "no collisions in 50 evaluation episodes" or "worst-case penetration
4.3 mm during exploration". Do not write "the CBF guarantees zero collisions" —
soft rows do not promise that, and the log says otherwise.

### 3.2 Not validated, and why

* **Closed-loop motion on fake hardware.** ROS 2 mock hardware does not
  integrate effort commands, so the arm does not move and the observation stays
  frozen. The fake-hardware test validates the *command chain* — nodes, topics,
  rates, controller activation, gating, TF — **not** behaviour. Closed-loop
  behaviour is validated in MuJoCo.
* **Real FR3.** No robot was connected; nothing was commanded to hardware. All
  real-hardware claims in this file are static or fake-hardware evidence only.
* **`d_safe = 0.15` on hardware.** Raised from 0.10 on 2026-09-17 (§5.3). The
  whole test suite passes at the new value, but no test covers closed-loop
  behaviour on a real arm, and the change affects the pentagon, velocity and
  OSCBF pipelines too. **Run supervised before trusting it.**

---

## 4. Real-hardware readiness review

| Aspect | Status |
|---|---|
| Command interface | Publishes `Float64MultiArray(7)` on `/NS_1/qddot_nom` — the topic `cbf_safety_filter` already consumes. No new hardware interface. |
| Joint ordering | `FR3_JOINT_NAMES` index map built from `JointState.name`, never positional. |
| Frame names | `ee_frame` parameter, default `fr3_hand_tcp`. Resolved through `resolve_frame_id`, which raises with the available-frame list if wrong. |
| Namespaces | Topics from `fr3_control.yaml`; `joint_state_topic` defaults to `__auto__` → namespace from `franka.config.yaml`. |
| Rates | Timer at `env.control_rate_hz` from the *training* config (100 Hz); overridable with `rate_hz`. |
| Limits | q̈ clamp from the **robot** config; every difference against the training config is logged as a `SIM-TO-REAL joint_limits drift` warning. |
| Policy selection | `onnx_model` unset → newest `.onnx` under `franka_sim/models` by mtime, **logged at WARN** with the resolved path. Pin it with `rl_onnx_model:=<path>` for any real run. |
| Safety behaviour | Five explicit zero-output gates (§2). All hard limits stay downstream in the CBF filter. |
| Startup | xacro, Pinocchio and the first ONNX inference are paid in `__init__`; `_warmup()` runs every per-tick path once. Measured 0.5–1.6 ms. |
| Shutdown | `request_stop()` publishes zeros for 0.5 s, then exits; CSV closed in `destroy_node`. |
| Fake vs real | No `use_fake_hardware` branch in the node — hardware-agnostic by construction. |
| Hard-coded paths | None. Paths are parameters with a `realpath`-based source-tree fallback that survives `--symlink-install`. |
| First-run derate | `action_scale ∈ (0,1]` scales `q̈_nom` down; clamped and warned if out of range. |

**Recommended first real run**: `motion_source:=rl rl_action_scale:=0.3`
`rl_onnx_model:=<explicit path>`, camera and `real_time_distance` **on**, a
single conservative target, with the `rl_status` gate field and `cbf_status`
slack/fault on screen.

---

## 5. Defects found and fixed

### 5.1 Actuation authority (2026-08-31)

**Symptom.** The `zero` and `random` baselines scored *identically* to the
trained policy — same final EE error, same episode length, same collision rate.
Three different action streams, one trajectory.

**Root cause.** `step()` drove MuJoCo's POSITION servos from a state-seeded
reference. Re-anchoring the reference to the measurement every tick means it
never integrates: the commanded lead saturates at `q̈·dt²` ≈ 6e-4 rad and the
servo settles at `q̇ ≈ 0.004 rad/s` no matter how large `q̈` is.

**Fix.** Do what the deployment chain does. The env converts the vendored
MJCF's position actuators to direct-force actuators in place and commands
`τ = mj_inverse(q, q̇, q̈_safe)`, recomputed every substep — the analogue of
`rt_torque_controller` re-evaluating at 1 kHz between two 100 Hz samples. A
second bug was caught on the way: the torque clip initially reused the
actuators' `ctrlrange`, which for a `position` actuator is the joint POSITION
range, pinning joint 4's torque at −0.15 N·m. Limits now come from the joint's
`actuatorfrcrange` (±87 / ±12 N·m).

**Result** (`validate_actuation.py`, now also `franka_sim/tests/`):

```
max |realized q̈ − commanded q̈| = 1.499e-15 rad/s²
torque limits  lo/hi = ∓[87 87 87 87 12 12 12] N·m
|Δq| over 0.5 s: zero-action 0.0000 rad   unit action 0.6148 rad
q̇₁: +action +2.145 rad/s   −action −2.190 rad/s
```

**Consequence.** Every policy trained before this fix is void.

### 5.2 The benchmark measured the reset, not the controller (2026-09-17)

**Symptom.** `sac_v3`, a random policy and a motionless arm all scored a
collision rate of exactly 20.0 % with an identical worst penetration of
−0.1467 m — while their task metrics differed enormously (56 % / 0 % / 0 %
success). The §5.1 signature, from a different cause.

**Root cause, part one — the obstacle teleported.** `reset()` parked the mocap
sphere at `_obs_base`, but `_advance_obstacle()` computed
`base + amp·sin(phase)` with `phase` seeded uniformly in [0, 2π). The first
tick therefore displaced the obstacle by up to the full amplitude in one
control period: **measured mean 0.113 m, max 0.200 m per 10 ms tick — 11 to
20 m/s, against a configured peak of 0.251 m/s.** No barrier can bound a 20 m/s
obstacle that lands inside `d_safe`. This is also why lowering
`obstacle.speed` from 0.6 Hz to 0.20 Hz never fixed the collision rate: the
jump is set by `amplitude`, which the config deliberately kept.

**Root cause, part two — episodes began in violation.** The obstacle base was
sampled uniformly in a box overlapping the arm's home pose with no feasibility
check. Measured over 50 episodes: **9 started already penetrating**, 21 started
within 0.05 m, the median episode started at `d_min = 0.063 m` — inside
`d_safe = 0.15` — and **every collision in the benchmark happened at step 1**.
It also broke the barrier's own premise: forward invariance is a statement
about trajectories that START in the safe set, and most did not.

**Root cause, part three — unreachable targets.** Targets were sampled
independently of the obstacle. Over 20 000 draws, **4.29 %** landed inside
`r_obs + d_safe` at *every* phase of the sweep: unreachable without driving
`h < 0`, a hard ceiling on the success rate and pure reward noise.

**Fix.** `_obstacle_at()` is now the single phase→position map both `reset` and
`step` use, so there is no teleport. `_sample_episode()` rejection-samples
until the start state clears `d_safe` and the target is not always-blocked,
bounded by `reset_max_tries` with a counted fallback (`env.reset_fallbacks`).
Transiently blocked targets (41.6 %) are kept on purpose — waiting one out is
the behaviour the policy should learn.

**Result:**

| | before | after |
|---|---|---|
| episodes penetrating at reset | 9/50 | **0/50** |
| reset `d_min` (min / median) | −0.134 / 0.063 m | **0.153 / 0.201 m** |
| zero-action collisions | 10/50 | **0/50** |
| first-tick obstacle jump (mean / max) | 0.113 / 0.200 m | **0.0011 / 0.0025 m** |
| implied first-tick speed | 11–20 m/s | **0.11–0.25 m/s** |

**Consequence.** Every policy trained before this fix is void, and every
collision number reported before it measured the reset distribution.

**Correction to the record.** §5.1 used to claim a "known remaining fidelity
gap (conservative direction)": that a zero command drifts 0.064 rad over 0.5 s
in sim through gravity sag, erring toward a less well-behaved plant. **That was
this defect.** It was the CBF retreating from a badly-placed obstacle. With a
clean reset, `|Δq| = 0.000000` over 0.5 s with zero CBF intervention —
`mj_inverse` at `q̇ = 0` is exact gravity compensation, so the feedforward-only
sim has no sag at all. The earlier "gravity drift grew 0.026 → 0.094 rad when
the hand was added" measured the same artifact. The real actuation gap is §10
P1 item 1, and it is about the controller's feedback law, not about drift.

### 5.3 The sim-to-real contract had silently drifted (2026-09-17)

Eight robot-side commits landed between 2026-09-14 and 2026-09-17 while
`franka_sim/` stood still. Four divergences resulted; all are now mirrored and
covered by `test_real_configs_are_in_sync`.

* **`d_safe`** — sim 0.15, robot 0.10. `test_real_configs_are_in_sync` had been
  failing for three days. **Resolved by raising the ROBOT to 0.15**: a policy
  must meet the shield it trained under, and moving the sim down would have
  voided the trained policy for no safety gain. This changes live behaviour for
  every pipeline that reads `d_safe` and moves every zone-ladder rung (the
  `zone_r_*` boundaries are multiples of it). **Not yet validated on hardware.**
* **`qddot_max_abs: 10.0`** (robot commit `4606e39`) — the robot capped
  acceleration authority at libfranka's `kMaxJointAcceleration`, but the sim
  still executed 17 rad/s² on joints 5 and 7. The policy was training against
  **41 % more action range on those joints than it gets on hardware**, and no
  test covered it. Now mirrored, box-only (§2).
* **Firmware velocity envelope + effective position limits** (robot commit
  `f5a59f8`, after five logged hardware aborts) — the robot obeys libfranka's
  position-dependent velocity curve and clamps its effective joint limits to
  `min(mechanical, FR3_VEL_Q_REF)`. The sim used a flat `q̇_max` and the
  mechanical stops, so it explored joint states the firmware simply refuses.
  Now mirrored, behind `cbf.firmware_envelope`.
* **Control-point coverage** — the robot monitors 11 points across segments
  3→4 … 7→8; the sim had 5 and none on the link3→link4 segment, making it
  **optimistic** about forearm approaches. `fr3_link3` added.

### 5.3b The QP solver was not reproducible (2026-09-17)

**Symptom.** A regression test comparing two identical 40-step rollouts failed
about **2 runs in 5 under pytest**, and never in a plain script. The
discrepancy started at ~5e-6 in the `d_min` slot and the closed loop amplified
it to 1.6e-4.

**Root cause.** OSQP ships `adaptive_rho: 1` with `adaptive_rho_interval: 0`,
which means *"re-adapt rho on a schedule derived from the measured SETUP
TIME"* (verified against the installed osqp 0.6.7). The solver's iteration path
is therefore a function of wall-clock timing, not only of its inputs. pytest's
output capture, assertion rewriting and import work perturb exactly the timing
OSQP samples, which is why the plain script never reproduced it — and why a
deliberate CPU-saturation test did not either.

**Consequence, and why it is worth a section.** `train.py --seed` did not
guarantee a reproducible run: two identical seeds on the same machine could
diverge through the shield. For a repository whose claim is a *safety* filter,
"the same inputs give the same q̈_safe" is not a nicety.

**Fix.** `adaptive_rho_interval=25` pinned in `cbf_filter.py` (25 is OSQP's own
`check_termination` default, the cadence at which it already evaluates
residuals). This changes the iteration SCHEDULE, never the problem: the
solution still satisfies the same `eps_abs`/`eps_rel` = 1e-3 tolerance. The
strict bit-equality test then passes repeatedly, and
`test_solver_is_reproducible` pins the property.

**Open, for the robot.** `cbf_safety_filter.py` still uses the default, so the
QP it solves on hardware is reproducible only up to solver tolerance. That is
far below every safety margin in the system, but it means a CBFDIAG line is not
bit-comparable across runs. Mirroring the pin is a one-line change and is NOT
made here: it touches the live safety filter and belongs in a supervised
session. Tracked in §9.

### 5.4 `motion_source:=rl` could not start (2026-09-17)

`launch_defaults.yaml` documented `rl_onnx_model: ""` as "newest model under
`franka_sim/models`" and the launch file repeated the claim, but
`rl_policy_commander` raised `RuntimeError` on an empty path — so the shipped
defaults killed the node before it published anything. `find_latest_model()`
now implements the documented behaviour and logs the resolved path at WARN.

### 5.5 Solver version skew (2026-08-31)

`qpsolvers` had floated to 4.13.0 while `osqp` stayed on 0.6.7 (pinned
deliberately — the filters drive the raw OSQP 0.6 API). qpsolvers ≥ 4.4 imports
`SolverStatus`, which only exists in osqp 1.x, so its osqp backend failed to
load: `qpsolvers.available_solvers == []`, and the velocity/OSCBF filters would
raise `SolverNotFound` at their first QP.

Fixed by pinning `qpsolvers[osqp]==4.3.3` + `osqp<1.0` in the `Dockerfile` and
adding `assert 'osqp' in qpsolvers.available_solvers` to the image smoke test,
so the build fails instead of shipping a silently broken solver.

### 5.6 The gripper (2026-09-04)

`fr3.xml` was arm-only, ending at `fr3_link7`. The Franka Hand is now in the
MJCF, converted from `franka_description/meshes/robot_ee/franka_hand_white`
(DAE → OBJ via trimesh; MuJoCo cannot read DAE). Transforms were **read from
the real URDF through Pinocchio, not guessed** — sim and robot FK agree to
**7e-16 m** at both the flange and the TCP over random configurations.

* **Fingers are rigid, parked fully open.** Slide joints would push `nv` past
  7, which the entire actuation chain assumes. The task reaches, it never
  grasps. Open is also the widest, most conservative footprint.
* **The EE moved to the hand TCP.** `env.ee_site: hand_tcp_site` and the node's
  `ee_frame` default `fr3_hand_tcp`.
* **The hand is a CBF control point** (`fr3_hand`, radius 0.13 — the measured
  bounding sphere of hand + fingers). This mirrors the robot, where
  `fr3_complete.yaml` maps `fr3_link8` to the hand collision mesh.

---

## 6. Current models

| Run | Steps | Status |
|---|---|---|
| `sac_v2` | 400 k | **VOID** — `d_safe=0.20`, fast obstacle, no gripper, flange EE |
| `sac_v3` | 2 M | **VOID for results** — trained pre-§5.2/§5.3. Loads and runs; its `.onnx` exists; use it only as a "before" snapshot |
| `sac_v4` | 2 M | **CURRENT** — trained on the fixed contract, 2026-09-17, 4 h 19 m on the RTX 4070. Final eval 90 % success / −29.4 reward; benchmark 86 % / 0 collisions (§3.1) |

**A policy is void whenever the env or the mirrored config changed under it.**
`train.py` freezes `config.yaml` next to each model and the node prefers that
frozen copy, so an old model still *loads* and reproduces its own training
conditions — that is a reproducibility feature, not a licence to compare its
numbers with a newer run's.

---

## 7. Gotchas and traps

1. **Fake hardware does not integrate effort commands.** The arm never moves
   and the observation stays frozen. Fake-HW tests validate the *command
   chain*, **not** closed-loop behaviour. Do not read a fake-HW "success" as
   motion validation.
2. **Real `d_min` saturates at 0** (the engine clamps); sim reports negative
   penetration. Conservative, documented, not "fixed".
3. **`qpsolvers` and `osqp` must not float.** If the velocity or OSCBF pipeline
   dies with `SolverNotFound`, check
   `python3 -c "import qpsolvers; print(qpsolvers.available_solvers)"` **first** —
   an empty list is version skew, not a code regression.
4. **The `qpsolvers` pin is in the `Dockerfile`, but an older image predates
   it.** Run `docker compose build` once, or patch a fresh container with
   `pip3 install "qpsolvers[osqp]==4.3.3" "osqp>=0.6.2,<1.0"`.
5. **`build/` and `install/` are CONTAINER-LOCAL and disposable.**
   `docker-compose.yml` mounts `./:/ros2_ws/src`, and `/ros2_ws/build` and
   `/ros2_ws/install` sit one level *above* that mount — so they do **not**
   live on the host and do **not** survive container recreation. Source,
   `franka_logs/`, `franka_sim/models/` and `franka_sim/runs/` are under the
   mount and do persist. After a fresh container, `colcon build` before
   running anything: without it most of `test/` cannot even import
   `franka_msgs`, and only the pure-numpy tests run.
6. **A stale `install/` will lie to you.** Sourcing it with an unbuilt or
   half-built workspace produces phantom failures. If a `--symlink-install`
   build fails partway, delete that package's `build/` and `install/` trees
   before retrying — the second attempt otherwise dies on `File exists`.
7. **Never use a MuJoCo actuator's `ctrlrange` as a force limit** unless you
   know it is a force actuator. For a `<position>` actuator it is the joint
   *position* range. This is the trap behind §5.1.
8. **`franka_sim` is not a ROS package.** Do not "fix" this by adding a
   `package.xml` — the whole point is that training does not need ROS. Paths
   are resolved via a `realpath` walk-up, and `franka_sim/tests/` runs under
   plain pytest rather than `colcon test`.
9. **The container may hold stray processes between runs.** Note that
   `pkill -f "topic pub"` also kills the `docker exec` shell whose command line
   contains that string, and `pgrep -af "franka_sim.train"` matches its own
   shell — check `ps aux` rather than trusting either.
10. **The training config frozen next to the model is authoritative.**
    `train.py` copies `config.yaml` into `models/<exp>/`; the node prefers it
    over `franka_sim/config.yaml`, because that is the config the run used.
11. **A run's feature flags come from the LAUNCH file, not the YAML.** Seven
    shield mechanisms are `false` in `fr3_control.yaml` and `true` in
    `launch_defaults.yaml`.
12. **Always quote the obstacle regime with a safety number.** The same policy
    scored 20 % and 0 % collisions under two regimes of this benchmark.
    `benchmark.py` puts the regime in its header for exactly this reason.

---

## 8. Reproducing the validation

```bash
# once per boot, on the HOST
cd ~/Git/franka_ros2 && USER_UID=$(id -u) USER_GID=$(id -g) docker compose up -d
docker exec franka_ros2 bash -lc 'source /opt/ros/humble/setup.bash && \
  cd /ros2_ws && colcon build --packages-up-to franka_experiments --symlink-install'

# sim guards (no ROS needed)
docker exec franka_ros2 bash -lc 'cd /ros2_ws/src && PYTHONPATH=/ros2_ws/src MUJOCO_GL=egl \
  python3 -m pytest franka_sim/tests -q'
docker exec franka_ros2 bash -lc 'cd /ros2_ws/src && PYTHONPATH=/ros2_ws/src MUJOCO_GL=egl \
  python3 -m franka_sim.scripts.validate_cbf'
docker exec franka_ros2 bash -lc 'cd /ros2_ws/src && PYTHONPATH=/ros2_ws/src MUJOCO_GL=egl \
  python3 -m franka_sim.scripts.validate_actuation'

# the sim-to-real contract + the whole ROS suite
docker exec franka_ros2 bash -lc 'source /ros2_ws/install/setup.bash && \
  cd /ros2_ws/src/franka_experiments && python3 -m pytest test/ -q'

# the benchmark table
docker exec franka_ros2 bash -lc 'cd /ros2_ws/src && PYTHONPATH=/ros2_ws/src MUJOCO_GL=egl \
  python3 -m franka_sim.scripts.benchmark --latest franka_sim/models/sac_v4 \
  --episodes 50 --markdown -'

# node + pipeline
docker exec franka_ros2 bash -lc 'source /ros2_ws/install/setup.bash && \
  cd /ros2_ws/src/franka_experiments && python3 test/smoke_rl_policy_commander.py'
```

---

## 9. Open items (not blockers)

* **Shield parity.** The sim trains under a subset (§2.1). Closing it means
  porting self-collision, joint-limit, singularity, retreat-cap and link-speed
  rows plus the state governor — a second copy of a subsystem that is still
  changing. Declared and test-pinned instead.
* **Obstacle CBF rows are soft** (slack-relaxable) in both sim and robot — a
  sustained push relaxes them by design. The policy is expected to learn
  avoidance; the CBF is the certificate.
* **`d_safe = 0.15` is unvalidated on hardware** (§3.2).
* **The robot's QP is not bit-reproducible** (§5.3b). The sim pins
  `adaptive_rho_interval`; `cbf_safety_filter.py` does not. One line, but it
  touches the live filter — decide it with the arm available.
* **Closed-loop hardware-in-the-loop validation** needs either Gazebo or the
  real FR3.

---

## 10. Improvement backlog (prioritised)

### P0 — makes the science defensible

1. **Report the baseline delta everywhere.** `benchmark.py` now emits it
   directly, with the obstacle regime in the header. Use it instead of
   `evaluate_policy` for anything quotable. *(Done 2026-09-17.)*
2. **Train to convergence on the fixed contract.** `sac_v4`. *(Done
   2026-09-17.)*

### P1 — closes real sim-to-real gaps

3. ~~**Mirror `rt_torque_controller`'s actual law in the sim actuation.**~~
   **DONE 2026-09-17, default OFF** (§2.2). Enabling it is a plant change:
   retrain and re-benchmark. Original note kept for the detail: The sim
   applies pure feedforward `mj_inverse`; the robot runs
   `τ = ffScale(τ_ff)·τ_ff + Kp·p + Kd·e` at 1 kHz, with `d_gains`
   `{30,30,30,25,10,10,5}`, `p_gains` `{120,120,120,100,40,40,20}`, an
   anti-windup clamp `p_max = 0.15`, a `qdot_margin = 0.95` envelope ceiling on
   the integrated reference and an `ff_fade_band = 0.25` directional
   feedforward fade. It also integrates its own reference from `/qddot_safe`.
   This is a bigger gap than the old "add the Kd term" framing suggested. Keep
   it configurable and default-off so the current baseline stays reproducible.
4. ~~**Domain randomisation**~~ and 5. ~~**Observation-noise realism**~~ —
   **DONE 2026-09-17, default OFF** (§2.2): latency, observation noise, joint
   sensor noise and dynamics, each with its own switch. What remains is
   EXPERIMENTAL, not implementation: pick the latency budget from
   `scripts/latency_budget.py` on real hardware, then retrain with it on and
   measure what the robustness costs in task performance.
6. **Hardware-in-the-loop.** Gazebo (`franka_simulation` has
   `sim_torque.launch.py` and `sim_acceleration_bridge.py`;
   `rt_torque_controller` has a `gazebo` argument) would give closed-loop
   validation without the real FR3.

### P1b — the benchmark's difficulty

7a. ~~**Give the policy the state the task needs.**~~ **DONE 2026-09-18**
    (observation contract above): obstacle velocity + per-control-point
    `(dᵢ, n̂ᵢ)`, 24 → 51 dims, prefix-compatible. Retrain and re-benchmark
    before quoting anything; `sac_v4`'s 86 % / 56 % is the number to beat.

7b. **Train with `blocking_fraction` > 0.** The obstacle currently obstructs
    the direct path in only half the sampled episodes (§3.1.0), so both the
    training signal and the benchmark are diluted with episodes that need no
    avoidance. `sac_v4` transfers to the always-blocking benchmark at 56 %;
    training on a 0.5–0.7 mix should raise that, and it is a cheap experiment
    (one retrain). Keep some unobstructed episodes: the robot meets those too.

### P2 — task and reward

7. **Reward shaping review.** Currently
   `−‖ee−target‖ + success − effort − intervention − slack − jerk − ‖q̇‖²`, with
   a one-off collision penalty and termination. Candidates: potential-based
   shaping for the distance term, curriculum on obstacle speed, and
   reconsidering `terminate_on_success` (episodes end early, so the policy
   never learns to *hold* a target — relevant because the deployment node
   supports target sequences with dwell).
8. **Multi-target episodes in sim**, matching the node's `target_sequence`.
9. **Orientation.** The task is position-only; the observation carries no EE
   orientation and the reward does not constrain it.

### P3 — engineering polish

10. **CI.** Wiring `colcon build` + `colcon test` + `franka_sim/tests` +
    `validate_actuation` + `validate_cbf` into it would have caught §5.1, §5.3
    and §5.5.
11. **Write `test_oscbf_fake.launch.py`** so the documented Pipeline 3 commands
    work — or drop Pipeline 3 (`franka_experiments/LEGACY.md`, deletion unit C).
12. **Address the `franka_simulation` linter failures** (49 copyright, 747
    flake8, 259 pep257) — or exclude that package from linting so `colcon test`
    is green and regressions are visible.
13. **Four sources of truth for feature flags** (`fr3_control.yaml`,
    `launch_defaults.yaml`, hard-coded launch thresholds, launch `_DEFAULTS.get`
    fallbacks) and **five for the controller gains** now that the sim mirrors
    them. See `LEGACY.md` §4.

---

## 11. Changelog

### 2026-09-17 — contract re-sync, benchmark repair, retrain, optional fidelity

*What changed:* robot `d_safe` 0.10 → 0.15; sim gained `qddot_max_abs`,
`state_box_relax_s`, `accel_box_clip_to_limits`, `firmware_envelope` and the
`fr3_link3` control point; obstacle teleport and reset feasibility fixed;
`shield_parity` register added; `find_latest_model` implemented;
`benchmark.py` and `franka_sim/tests/` added; the actuation law and the four
randomisation mechanisms added **default-off** (§2.2); `sac_v4` trained
(4 h 19 m, 86 % success / 0 collisions) and re-validated on fake hardware.

*What it invalidated:* `sac_v2` and `sac_v3` as sources of results, and every
collision number reported before it. `sac_v4` is the retrain. The optional
layers invalidated nothing — they are bit-identical to the previous env while
disabled, which is test-enforced.

*What needs hardware:* the `d_safe` change, for every pipeline. Nothing else in
this batch touches the robot.

*Corrected here:* the claim that the sim has gravity sag (§5.2), and "the CBF
guarantees zero collisions" — it is ~0, not 0 (§3.1.1).

### 2026-09-14 — documentation consolidation

Six point-in-time documents deleted, their live content folded here and into
`LEGACY.md`. **This is the edit after which the file went stale:** eight
robot-side commits landed over the next three days without a corresponding
`franka_sim` change, which §5.3 cleans up.

### 2026-09-04 — gripper, obstacle regime, episode checkpoints

Franka Hand added to the MJCF; EE moved to the hand TCP; obstacle slowed to
0.30 Hz / 0.20 m; `EpisodeCheckpointCallback` added. Voided every policy
trained before it.

### 2026-08-31 — Step 3 and the actuation defect

Deployment node, contract module, launch integration, and the §5.1 actuation
fix that voided every policy before it.
