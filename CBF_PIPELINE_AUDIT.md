# CBF Safety Pipeline — Read-Only Audit

**Repo:** `~/Git/franka_ros2`
**Branch:** `humble-mattia`
**HEAD:** `bf9e458` — *"Ignore rosbag directory"*
**Working tree:** clean, no uncommitted changes
**Audit date:** 2026-09-01
**Scope:** read-only. No source file was modified, no node was built or launched.

---

## Table of contents

- [Phase 0 — Inventory](#phase-0--inventory)
- [Phase 1 — What changed recently](#phase-1--what-changed-recently)
- [Phase 2 — Data flow as actually written](#phase-2--data-flow-as-actually-written)
- [Phase 3 — The CBF logic itself](#phase-3--the-cbf-logic-itself)
- [Phase 4 — Parameters](#phase-4--parameters)
- [Phase 5 — Findings](#phase-5--findings)
- [Bottom line](#bottom-line)

---

## Phase 0 — Inventory

### IN SCOPE (CBF chain)

| Path (from repo root) | Lines | Last commit | Role |
|---|---|---|---|
| `franka_experiments/franka_experiments/nodes/pentagon_qddot_commander.py` | 1557 | 2026-09-01 (`7714509`) | Motion gen. 100 Hz → `qddot_nom`; also avoidance shaping + feasibility governor β |
| `franka_experiments/franka_experiments/nodes/cbf_safety_filter.py` | 959 | 2026-09-01 (`7714509`) | **The CBF node.** 3 threads: I/O, 50 Hz constraint build, QP loop → `qddot_safe` |
| `franka_experiments/franka_experiments/nodes/qddot_to_torque.py` | 205 | 2026-09-01 (`7714509`) | τ = M·q̈ + C·q̇ (no gravity) → `torque_cmd` |
| `franka_experiments/franka_experiments/utils/cbf_hard_limits.py` | 137 | 2026-09-01 (`7714509`) | **New file.** `hard_accel_box`, `apply_slew_limit`, `workspace_face_rows` |
| `franka_experiments/franka_experiments/utils/cbf_kinematics.py` | 66 | 2026-06-10 | Pinocchio wrapper: FK, point Jacobian `Jp`, `J̇p` |
| `franka_experiments/franka_experiments/utils/avoidance.py` | 163 | 2026-09-01 (`7714509`) | `feasibility_beta_target`, `tangential_redirect`, `influence_weight` |
| `franka_experiments/config/fr3_control.yaml` | 115 | 2026-09-01 (`7714509`) | Topics, CBF params, joint limits |
| `franka_experiments/launch/torque_control_stack.launch.py` | 495 | 2026-09-01 (`7714509`) | The only launch that wires the full chain |
| `franka_rt_controllers/src/rt_torque_controller.cpp` | 323 | 2026-07-29 | 1 kHz C++: τ = τ_ff + Kd·(q̇_des − q̇) |
| `franka_msgs/msg/LinkDistance.msg` / `MultiLinkDistance.msg` | 8 / 2 | — | CBF ↔ perception interface |

### BOUNDARY (perception — read only, documented as an interface, no changes proposed)

| Path | Lines | Role for this audit |
|---|---|---|
| `franka_experiments/franka_experiments/nodes/real_time_distance.py` | 458 | Publishes `/cbf/per_link_distances` — **its publish *gating* is the key finding**, see B1 |
| `franka_experiments/franka_experiments/utils/distance_engine.py` | 393 | The `lpf_*` filters that shape `ld.distance` before the QP |
| `franka_experiments/franka_experiments/utils/node_utils.py` | 244 | `build_cp_messages` — sets `ld.valid` / `ld.confidence` |
| `franka_experiments/config/fr3_complete.yaml` | 141 | `thresholds`, `lpf_alpha`, `fallback_distance` |

### OUT OF SCOPE (not read for changes)

`cbf_velocity_filter.py`, `cbf_OSCBF_filter.py`, `cbf_qp.py`, `cbf_constraints.py`, `cbf_utils.py`
(legacy velocity-level path, not in the torque stack), `franka_sim/*` (simulation),
`cbf_torque_controller.cpp`, `distance_visualization_node.py`, and all RealSense / mask /
deprojection / segmentation / hand-tracking code.

> **Scope caveat:** `real_time_distance.py`, `node_utils.build_cp_messages` and
> `distance_engine.py` were treated as *perception*, i.e. out of bounds for change
> proposals — but the single most severe finding (B1) lives in `real_time_distance.py:346`.
> It is documented here as an interface fact; the fix decision is the owner's.

---

## Phase 1 — What changed recently

`git status`: **clean**, up to date with `origin/humble-mattia`.

Only two commits since June touched CBF code, and effectively **one commit is responsible for
the current state**.

### `7714509` "WIP" (2026-09-01)

90 files, +1.69 M lines (mostly `franka_sim` mesh assets). CBF-relevant deltas:

| # | File | Semantic change (fact from diff) | Suspicion |
|---|---|---|---|
| **1** | `qddot_to_torque.py:109` | Subscription changed **`topics['qddot_safe']` → `topics['qddot_nom']`**; the CBF is now bypassed unless the launch file remaps. Compensating remap added at `torque_control_stack.launch.py:328`. | **Critical fragility** |
| **2** | `pentagon_qddot_commander.py:807-826` | **New feasibility governor**: reads `cbf_status`, `fault=True ⇒ β=0` (`avoidance.py:133-134`) → virtual time frozen, robot stops on the path. Did not exist before. | **Highest — almost certainly the "arm stops"** |
| **3** | `cbf_safety_filter.py:591-603, 613-616` | **New HARD workspace-box rows** on `fr3_link8` (slack column forced to 0). Cannot be relaxed → can make the QP *primal infeasible* → braking + `fault=1` → (via #2) β=0. | High |
| **4** | `cbf_safety_filter.py:739-749, 917-930` | `_update_velocity_box` replaced by `hard_accel_box` (adds position braking curve √(2ηa·h)) ∩ `apply_slew_limit` (\|q̈−q̈_prev\| ≤ 5 rad/s²), applied to **every** output path via `_finalize_and_publish`. | Medium — new jerk cap, new failure surface |
| **5** | `cbf_safety_filter.py:485-496, 564` | h̄ now uses `ld.distance` (engine-filtered) + `ld.direction`, instead of recomputing ‖p_r − p_h‖. **Correctness improvement**, but the barrier now depends on the perception LPF/outlier logic. | Improvement + new coupling |
| **6** | `cbf_safety_filter.py:784` | `elif obs stale` → **`if obs stale`** — the braking fallback now fires even when workspace rows are fresh. Strictly more braking than before. | Medium |
| **7** | `pentagon_qddot_commander.py` params | `path_type` default `'cricle'` (typo) → `'circle'`; `plane` `'xy'` → `'front'`. Launch also now forces `center=[0.4,0,0.45]`, `radius=0.25`. | Different trajectory than before |
| **8** | `fr3_control.yaml:79-97` | New `hard_*` and `ws_*` blocks. Comment on `distance_ema_alpha` flipped from *"DEPRECATED/UNUSED: not read by any code"* to *"EMA weight on SCALAR distance"* — **the new comment is wrong; it is still unread.** | Doc lie |
| 9 | `cbf_safety_filter.py` | Removed `VELHI` diagnostic, `_qp_fail_count`, `diag_vel_ratio_thr`. | Cosmetic |
| 10 | `distance_engine.py:76-86, 293-327` | Approach-spike rate-limit + 1-frame confirmation (`lpf_v_max_approach`, `lpf_outlier_recover_*`). | Improvement |

**Fact vs inference:** items 1, 3, 4, 5, 6, 7, 8 are read directly off the diff.
The *ranking* — that #2 combined with the perception publish gate is what stops the arm — is
**inference**, argued in [B1](#b1--the-perception-node-stops-publishing-when-no-obstacle-is-in-008070-m).

### Earlier commits (not suspects)

`7c8677d`, `5a1ab52`, `201fd2a` (June 2026) introduced the HOCBF J̇q̇ term, the OSQP
persistent-instance migration, and `cbf_min_leverage`. They are consistent with today's code.

---

## Phase 2 — Data flow as actually written

### Publishers / subscribers (read from source, not from comments)

| Topic | Type | Publisher (QoS) | Subscriber(s) (QoS) | Rate |
|---|---|---|---|---|
| `/NS_1/joint_states` | `sensor_msgs/JointState` | `joint_state_broadcaster` (CM default) | `cbf_safety_filter.py:309-311` (depth 1, RELIABLE)<br>`qddot_to_torque.py:101-106` (depth 10)<br>`pentagon:569` (depth 10) | CM rate |
| `/cbf/per_link_distances` | `franka_msgs/MultiLinkDistance` | `real_time_distance.py:186-188` (**depth 1, BEST_EFFORT**) | `cbf_safety_filter.py:315-318` (depth 1, BEST_EFFORT)<br>`pentagon:578-582` (depth 1, BEST_EFFORT) | ≈camera 30 Hz, **conditionally** (see B1) |
| `/NS_1/qddot_nom` | `Float64MultiArray[7]` | `pentagon:590` (depth 10, RELIABLE) — topic from `qddot_safe_topic` param defaulting to `topics['qddot_nom']` (`pentagon:129`) | `cbf_safety_filter.py:312-314` (depth 1, RELIABLE) | 100 Hz (`pentagon:134`) |
| `/NS_1/qddot_safe` | `Float64MultiArray[7]` | `cbf_safety_filter.py:320-321` (depth 10, RELIABLE) | `qddot_to_torque` **via remap** (depth 10)<br>`rt_torque_controller.cpp:210-211` `accel_sub_` (`SensorDataQoS().keep_last(1)` = BEST_EFFORT) | `qp_rate_hz` = **100 Hz** (`fr3_control.yaml:23`) |
| `/NS_1/cbf_status` | `Float64MultiArray[4]` | `cbf_safety_filter.py:333-334` (depth 10) | `pentagon:584-587` (depth 1) | 100 Hz |
| `/NS_1/torque_cmd` | `Float64MultiArray[7]` | `qddot_to_torque.py:99` (depth 10, RELIABLE) | `rt_torque_controller.cpp:204-205` `command_sub_` (BEST_EFFORT, keep_last 1) | event-driven off `qddot_safe` → 100 Hz |

`cbf_status` payload: `[n_c, slack, fault_braking, min obstacle h̄ (99.0 = none active)]`
(`cbf_safety_filter.py:336-337, 866-870`).

### Remapping — the single point the whole safety chain hangs on

```python
# torque_control_stack.launch.py:323-329
qddot_to_torque_node = Node(
    package='franka_experiments',
    executable='qddot_to_torque',
    name='qddot_to_torque',
    output='screen',
    remappings=[('/NS_1/qddot_nom', '/NS_1/qddot_safe')],
)
```

This remap is what makes the chain correct. Without it, `qddot_to_torque.py:109` reads
`topics['qddot_nom']` and the CBF becomes a passive observer.

**Namespace resolution (verified):** `_DEFAULTS = {**launch_defaults, **bringup_defaults}`
(`torque_control_stack.launch.py:74`) and `franka_bringup/config/franka.config.yaml` sets
`namespace: "NS_1"` → the bringup defaults win → the controller node is
`/NS_1/rt_torque_controller`, so its relative `command_topic: torque_cmd` (`ros.py:499`)
resolves to `/NS_1/torque_cmd`. ✔ Matches.
**Fragile:** launching with `namespace:=""` breaks every absolute `/NS_1/...` topic
hard-coded in `fr3_control.yaml`, *and* silently breaks the remap key above.

### Perception → CBF interface

- **Entry: topic only, no TF** in the CBF node. `cbf_safety_filter.py:476-497` reads
  `ld.robot_link_name`, `ld.distance`, `ld.closest_point_robot`, `ld.direction`,
  `ld.confidence`, filtering on `ld.valid`.
- **Frame:** `fr3_link0` (`fr3_complete.yaml:2`, stamped into `mld_msg.header.frame_id` at
  `node_utils.py:204`). The CBF's Pinocchio model root is also `fr3_link0` → consistent, no
  transform applied.
- **Timestamps: the message header stamp is never read.** `_on_distances` stamps the snapshot
  with `self._now()` (receipt wall time, `cbf_safety_filter.py:497`). Sensor-to-CBF latency is
  therefore invisible; only *"did a message arrive in the last 0.5 s"* is checked.
- **Staleness handling:** `cbf_safety_filter.py:511` (drop obstacle rows) and `:784-800`
  (**replace nominal with braking, set `fault_braking = 1`**).

### Actual chain

```
 RealSense depth ──► real_time_distance (30 Hz)
                       │  DistanceEngine.compute → LPF/outlier (distance_engine.py:247)
                       │  build_cp_messages (node_utils.py:95)
                       │  ⚠ PUBLISH GATE: only if ∃ CP with 0.08 ≤ d ≤ 0.70 m
                       ▼  (real_time_distance.py:336-347 → 375)
              /cbf/per_link_distances  [MultiLinkDistance, BEST_EFFORT, depth 1]
                       │
        ┌──────────────┴───────────────────────────────┐
        ▼                                              ▼
 pentagon_qddot_commander (100 Hz)              cbf_safety_filter
   • tangential redirect (:882-897)               ├─ I/O thread: snapshots
   • null-space repulsion (:943-950)              ├─ 50 Hz: Pinocchio FK + Jp, J̇p
   • governor β ← cbf_status (:807-826)           │        → A, h̄, ċ, G   (:501)
        │                                         └─ 100 Hz: OSQP          (:704)
        ▼                                                   │
  /NS_1/qddot_nom ────────────────────────────────────────► │
                                                            ▼
                                                   /NS_1/qddot_safe
                                                            │
                    ┌───────────────────────────────────────┤
                    ▼ (remap qddot_nom→qddot_safe)          ▼ accel_sub_
             qddot_to_torque                          rt_torque_controller (1 kHz)
             τ = M q̈ + C q̇  (hand:=true)               τ = τ_ff + Kd·(q̇_des−q̇)
                    │                                       ▲
                    └────► /NS_1/torque_cmd ────────────────┘ command_sub_
                                                            │
                                                     Franka FCI (adds g(q))
                                                            │
                                   /NS_1/cbf_status ────────┘ (→ pentagon governor)
```

### Explicit answers to the two verification questions

**1. Which topic does `qddot_to_torque` actually subscribe to?**

- In the **source**: `topics['qddot_nom']` = `/NS_1/qddot_nom` (`qddot_to_torque.py:109`).
  **The CBF is bypassed at source level.**
- In the **launch**: remapped to `/NS_1/qddot_safe` (`torque_control_stack.launch.py:328`), so
  the deployed chain *is* correct.
- This polarity inverted in commit `7714509`. Pre-WIP the node read `qddot_safe` directly and
  needed no remap.

**2. Are there QoS mismatches anywhere?**

**None that break connectivity.** Every RELIABLE publisher is read by BEST_EFFORT or RELIABLE
subscribers (RELIABLE pub ⊇ BEST_EFFORT sub is compatible). The only BEST_EFFORT publisher
(`/cbf/per_link_distances`) is read only by BEST_EFFORT subscribers.
All profiles are VOLATILE — **no TRANSIENT_LOCAL anywhere**, so a late-joining node receives
nothing until the next sample (irrelevant at these rates).

---

## Phase 3 — The CBF logic itself

### 1. Barrier function h(x)

`cbf_safety_filter.py:564` — `h = ob.d - self._d_safe`

- `ob.d` = `ld.distance` (`:488`) = the engine's **filtered surface distance** =
  `max(‖p_pix − p_cp‖ − radius_cp − dilation_margin, 0)` (`distance_engine.py:211`),
  EMA/outlier-processed (`:247-337`).
  Point-to-point in depth space against the nearest un-masked depth pixel — **not**
  point-to-mesh.
- **Control points:** per-segment CPs from `fr3_complete.yaml:15-62` (segments 3–7, 2–3 CPs
  each, radius 0.05 m). One `LinkDistance` per `end_link`, best CP wins
  (`node_utils.py:139-142`).
- **Jacobian point** `ob.pr` = `ld.closest_point_robot` = the **CP centre**
  (`node_utils.py:184-186`, `r.point`), not the surface point. This is *correct*, since
  ḋ_surface = ḋ_centre for a fixed radius.
- **Safety margins — three, all additive:**
  | Margin | Value | Applied at |
  |---|---|---|
  | `d_safe` | 0.20 m | barrier, `cbf_safety_filter.py:564` |
  | CP `radius` | 0.05 m | engine, `distance_engine.py:211` |
  | dilation margin | depth-dependent (18 px EE / 2 px body) | engine, `distance_engine.py:190` |

### 2. Derivative

- **ḣ = aᵀq̇** with `a = n̂ᵀ Jp` (`cbf_safety_filter.py:550`). Recomputed with the fresh q̇
  every QP tick (`:781`).
- **ḧ = aᵀq̈ + ċ**, `ċ = n̂ᵀ(J̇p q̇)` (`:568`), frozen at the 50 Hz snapshot.
- **Jacobians:** `CBFKinematics.point_jacobian` (`cbf_kinematics.py:43-66`) in
  `LOCAL_WORLD_ALIGNED` (base/world-oriented at the frame origin), rigidly transported to the
  CP: `Jp = Jv − [r]× Jw`, `J̇p = J̇v − [r]× J̇w` — the code notes this is exact only for
  fixed r (`:65`).
- **Human velocity is assumed ZERO.** There is no ṗ_human term anywhere; ḣ contains only robot
  motion. The obstacle is treated as static within the barrier dynamics.

### 3. Class-K function / relative degree

Relative degree **2** → **exponential/HOCBF with a linear class-K pair**
(`cbf_safety_filter.py:771-783`):

```
ḧ + k1·ḣ + k0·h ≥ 0
⇒  aᵀq̈ ≥ −k1(aᵀq̇) − k0·h̄ − ċ
QP row (G x ≤ u, G = [−A | −soft]):   h_qp = k1(aᵀq̇) + k0·h̄ + ċ
```

Sign chain verified by hand: `−a q̈ − s ≤ h_qp ⇔ a q̈ + s ≥ −h_qp` and
`−h_qp = −k1 aᵀq̇ − k0 h̄ − ċ`. **The signs are correct.** The `ċ` term is *added* to `h_qp`,
matching the derivation.

- `k0 = 25.0`, `k1 = 10.5` (`fr3_control.yaml:37-38`).
  Δ = k1² − 4k0 = 10.25 > 0, ζ ≈ 1.05, poles ≈ −4.0 and −6.5 rad/s. Mildly overdamped, as
  documented in the YAML comment.
- The same `k0`/`k1` are reused for the workspace rows (`cbf_hard_limits.py:122`).

### 4. QP formulation

`cbf_safety_filter.py:243-260` (build), `:812-828` (solve)

- **Decision variables:** `x ∈ ℝ⁸ = [q̈ (7) ; s (1)]`.
  **One scalar slack shared by all obstacle rows** — not per-row.
- **Cost:** `P = diag(1,1,1,1,1,1,1, ρ)`, `q = [−q̈_nom ; 0]`
  → `½‖q̈ − q̈_nom‖² + ½ρs²`, with ρ = 1000 (`fr3_control.yaml:17`).
  This is a **quadratic** slack penalty, deliberately divergent from OSCBF's linear ρᵀt
  (documented at `:236-242`).
- **Constraint rows** (`_osqp_A`, `:630-650`; bounds `_osqp_lu`, `:652-662`):
  1. `n_obs` **soft** obstacle rows: `−aᵢᵀq̈ − s ≤ h_qp,i`
  2. `n_c − n_obs` **hard** workspace-face rows: `−aⱼᵀq̈ ≤ h_qp,j`
     (slack column = 0, `:615-616`)
  3. `7` box rows on q̈: `box_lo ≤ q̈ ≤ box_hi`, where the box =
     static ±q̈_max ∩ one-step velocity bound ∩ position braking curve √(2ηa·h)
     (`hard_accel_box`, `cbf_hard_limits.py:39-78`)
     ∩ slew \|q̈ − q̈_prev\| ≤ 5 (`apply_slew_limit`, `:81-104`)
  4. `1` box row on slack: `0 ≤ s ≤ 1e6`
- **Not present:** no joint-torque limit row, no null-space / regularization term, no per-row
  slack. `max_tau_delta: 15.0` (`fr3_control.yaml:100`) is **never read by any code**, despite
  the comment claiming it is "encoded as QP linear constraint".
- **OSQP settings:** `warm_start=True`, `max_iter=20000`, `verbose=False`; everything else
  default (`:816-818`). Persistent instance; `setup()` only when `n_c` changes the sparsity
  pattern (`:813`), otherwise `update(q, l, u, Ax)` (`:819-823`).
  The CBF block is stored with a **fully dense sparsity pattern** so `update(Ax=...)` stays
  valid when a Jacobian entry passes through zero (`:636-641`) — correct and non-obvious.

### 5. Failure handling

| Failure | Detected at | Published | `cbf_status` |
|---|---|---|---|
| Joint state older than 0.1 s | `:724` | `_finalize_and_publish(zeros)` — clipped into **last tick's** box | returns early, **no status published** |
| `qddot_nom` older than 0.5 s | `:752` | QP solved with `q̈_nom := −k_brake·q̇` | fault = **0** |
| Distance older than 0.5 s | `:784` | QP solved with `q̈_nom := −k_brake·q̇`; obstacle rows already dropped, workspace rows kept | fault = **1** |
| OSQP status ≠ `OSQP_SOLVED`, or `x` None/non-finite | `:845-855` | `−k_brake·q̇`, then clipped; `_osqp_prob = None`, `_prev_nc = −1` (forces clean `setup()` next tick) | fault = **1** |
| No constraints at all (`_con is None`) | `:768` | Pure passthrough of `q̈_nom` through the box | fault = 0, n_c = 0 |

There is **no last-value-persist path** — every tick publishes. All paths funnel through
`_finalize_and_publish` (`:917-930`), which clips into the box and updates the slew anchor.
This part is well constructed.

### 6. Filtering / gating on the distance signal

**In the perception node, before the CBF ever sees it:**

| Stage | Where | Parameters |
|---|---|---|
| Radius + dilation-margin subtraction, floored at 0 | `distance_engine.py:211` | `radius` 0.05 m, `robot_mask_dilate_px` 2, `ee_mask_dilate_px` 18 |
| Asymmetric EMA: closer → raw immediately; farther → `α·d_prev + (1−α)·d_raw` | `distance_engine.py:318-332` | `lpf_alpha = 0.5` (`fr3_complete.yaml:111`) |
| Approach-rate limit + 1-frame confirmation | `distance_engine.py:293-327` | `lpf_v_max_approach` **2.0 (default, not in any YAML)**, `lpf_outlier_recover_frac` **0.10 (default)**, `lpf_outlier_recover_abs` **0.03 (default)** |
| Direction EMA + renormalise | `distance_engine.py:339-346` | same α = 0.5 |
| Hold last value when no finite measurement | `distance_engine.py:279-288` | none — **unbounded hold, never expires** |
| **Range gate on publication** | `real_time_distance.py:336-347` | `min_thresh` 0.08 m, `max_thresh` 0.70 m |

**In the CBF node:**

| Gate | Where | Parameter |
|---|---|---|
| `ld.valid` required | `:495` | — |
| `‖n̂‖ ≥ 0.5` | `:528` | hard-coded |
| `d > obstacle_horizon` → skip | `:539` | `cbf_obstacle_horizon` 1.2 m |
| `conf < min_confidence` → skip | `:539` | `min_confidence` 0.2 — **dead, see B4** |
| `‖a‖ < cbf_min_leverage` → drop | `:559` | `cbf_min_leverage` 0.05 m/rad |
| finite check on a, h, ċ | `:570` | — |

**No hysteresis anywhere.** The old `cbf_activation_margin` / `cbf_hysteresis` step gate was
retired for this node (`fr3_control.yaml:46-53`); activation is now continuous via the linear
HOCBF.

---

## Phase 4 — Parameters

### `params:` block of `fr3_control.yaml`, as consumed by `cbf_safety_filter`

| Name | Code default | YAML value | Read at | Used? | Units / frame |
|---|---|---|---|---|---|
| `qp_rate_hz` | 200.0 | **100.0** | `:180` | yes | Hz — **YAML ≠ the 200 Hz in the brief and in the module docstring `:9`** |
| `cbf_update_rate_hz` | 50.0 | 50.0 | `:181` | yes | Hz |
| `d_safe` | 0.20 | 0.20 | `:182` | yes | m, surface-to-surface |
| `cbf_obstacle_horizon` | 1.2 | 1.2 | `:186` | yes | m |
| `k0_cbf` | 25.0 | 25.0 | `:187` | yes | s⁻² |
| `k1_cbf` | 10.5 | 10.5 | `:188` | yes | s⁻¹ |
| `rho_slack` | 1000.0 | 1000.0 | `:189` | yes | (m/s²)⁻² — quadratic penalty |
| `qp_solver` | `'osqp'` | osqp | `:190` | **read, never branched on** — OSQP is hard-coded | — |
| `distance_timeout` | 0.5 | 0.5 | `:191` | yes | s, receipt-time |
| `nom_timeout` | 0.5 | 0.5 | `:192` | yes | s |
| `joint_state_timeout` | 0.1 | 0.1 | `:193` | yes | s |
| `k_brake` | 3.0 | 3.0 | `:194` | yes | s⁻¹ |
| `min_confidence` | 0.2 | 0.2 | `:195` | **effectively dead** — `node_utils.py:198` sets `ld.confidence = 1.0` unconditionally | — |
| `cbf_min_leverage` | 0.05 | 0.05 | `:198` | yes | m/rad |
| `hard_v_margin` | 0.9 | 0.9 | `:206` | yes | fraction of q̇_max |
| `hard_q_margin` | 0.05 | 0.05 | `:207` | yes | rad |
| `hard_brake_eta` | 0.7 | 0.7 | `:208` | yes | fraction |
| `max_qddot_delta` | 5.0 | 5.0 | `:209` | yes | rad/s² **per QP tick** (⇒ 500 rad/s³ at 100 Hz) |
| `ws_enable` | True | true | `:214` | yes | — |
| `ws_min` / `ws_max` | `[.05,−.6,.05]` / `[.75,.6,.95]` | same | `:215-218` | yes | m, **`fr3_link0` frame** |
| `ws_margin` | 0.02 | 0.02 | `:219` | yes | m |
| `ws_horizon` | 0.25 | 0.25 | `:220` | yes | m |
| `ws_ee_link` | `fr3_link8` | fr3_link8 | `:221` | yes | frame **origin**, not TCP |

### Dead parameters — present in `fr3_control.yaml`, read by nothing in the torque stack

| Name | Value | Status |
|---|---|---|
| `d_safe_default` | 0.1 | Dead here (`cbf_constraints.py:108` has its own 0.15 default; that module is the velocity path) |
| `control_rate_hz` | 100.0 | Dead |
| `ema_alpha` | 0.4 | Dead (explicitly marked DEPRECATED at `:29-30`) |
| `gamma` | 1.0 | Dead in the HOCBF path |
| `cbf_activation_margin` / `cbf_hysteresis` | 0.10 / 0.05 | Dead for `cbf_safety_filter`; live only for `cbf_velocity_filter` (not launched) |
| `qp_smooth_weight` | 0.2 | Dead |
| **`max_tau_delta`** | **15.0** | **Dead** — its comment claims it is "encoded as QP linear constraint". **There is no torque constraint in the QP at all.** |
| **`distance_ema_alpha`** | **0.5** | **Dead** — commit `7714509` *removed* the comment that correctly said so, replacing it with one implying it works |
| `diag_disable_gc` | ROS param, default False | Live (`:358-363`), diagnostic only |

### Parameters used in code but absent from YAML (silent defaults)

| Name | Silent default | Read at | Consequence |
|---|---|---|---|
| `lpf_v_max_approach` | 2.0 m/s | `distance_engine.py:84` | Spike gate threshold untunable from config |
| `lpf_outlier_recover_frac` | 0.10 | `distance_engine.py:85` | idem |
| `lpf_outlier_recover_abs` | 0.03 m | `distance_engine.py:86` | idem |

### `lpf_*` exposure

**No `lpf_*` parameter is in `fr3_control.yaml`.** The distance LPF is configured entirely from
`fr3_complete.yaml`:

- `lpf_alpha: 0.5` — present at `fr3_complete.yaml:111`, inside the `distance:` section.
- `lpf_v_max_approach`, `lpf_outlier_recover_frac`, `lpf_outlier_recover_abs` — **absent from
  every YAML**, running on hard-coded defaults.
- **Unrelated homonym:** `lpf_alpha: 0.3` in `launch_defaults.yaml:40` is the **torque** LPF for
  `rt_torque_controller`. It is passed into the generated controller YAML (`ros.py:501`) and
  **the C++ never reads it** — `rt_torque_controller.cpp:120` explicitly notes that
  `lpf_alpha` / `tau_max_scale` / `urdf_path` are passed without the C++ using them. Dead.

---

## Phase 5 — Findings

### 5.1 Certain bugs

#### B1 — The perception node stops publishing when no obstacle is in [0.08, 0.70] m

…and the CBF reads that silence as a safety fault, which the commander escalates into a
full stop.

**Facts:**

- `real_time_distance.py:336-341` filters CP results to those with
  `min_thresh ≤ d ≤ max_thresh` (0.08 / 0.70 m, `fr3_complete.yaml:97-98`).
- `:342-347` — if that list is empty → `_publish_fallback(...)` → **`return`**.
- `_publish_fallback` (`:441-445`) publishes **only** `HumanRobotDistance` on `dist_pub`.
  It does **not** touch `per_link_dist_pub`.
- `per_link_dist_pub.publish` appears exactly **once** in the file, at `:375`, inside the
  branch that requires a non-empty `valid` list.
- Therefore `/cbf/per_link_distances` **goes silent** whenever the human is farther than
  0.70 m or closer than 0.08 m, or when TF/depth yields no control points
  (`:296-297`, `:334-335` also `return` without publishing).

**Consequence chain:**

1. `cbf_safety_filter.py:784` — `obs` is not None (a message arrived earlier) and
   `now − obs.stamp > 0.5` → `qddot_nom := −k_brake·q̇`, **`fault_braking = 1.0`**.
2. `:868` publishes `fault = 1` on `cbf_status`.
3. `pentagon:816-822` reads it; `avoidance.py:133-134`: `if fault: return 0.0`
   → **β_target = 0**.
4. `pentagon:823-824` rate-limits β down at 4 s⁻¹ → β = 0 in 250 ms →
   `_timing.step(β·dt)` freezes virtual time → **the path stops and never resumes**,
   because the only thing that clears `fault` is a fresh distance message, which only
   arrives if a human re-enters the 0.08–0.70 m band.

The code comment at `cbf_safety_filter.py:794-796` asserts the opposite — *"when obstacles are
simply out of range obs is fresh … so we skip it and keep normal passthrough — no spurious
braking"*. **That assumption is false given the publish gate.**

This is the mechanism behind the "arm stops" deadlock, and it is now *worse* than before
`7714509` because the governor did not exist then.

**Asymmetry that makes it intermittent:** if the camera pipeline has **never** published,
`self._obs is None` and the `:784` guard is skipped entirely — the stack runs fine. The
deadlock only arms itself after the first human detection.

#### B2 — `qddot_to_torque` subscribes to `qddot_nom` in source; only a launch remap keeps the CBF in the loop

`qddot_to_torque.py:109` reads `topics['qddot_nom']`. The remap at
`torque_control_stack.launch.py:328` is the *sole* thing routing the safe command.

Any of the following silently bypasses the CBF with **no error, no warning**, and a log line at
`:116` that literally prints `qddot_nom ← /NS_1/qddot_nom`:

- running the node directly
- a new launch file
- a test harness
- a namespace change that makes the absolute remap key stop matching

Commit `7714509` inverted this from the safe default (pre-WIP the node read `qddot_safe`
directly and needed no remap). For a safety filter, defaulting to "bypassed" is the wrong
polarity.

#### B3 — `ld.valid` is false at exactly zero distance, dropping the constraint at contact

- `node_utils.py:197`: `ld.valid = math.isfinite(d) and d > 0.0 and di is not None`
- `distance_engine.py:211`: `surface = np.maximum(dist_3d - radii[i] - margin_m, 0.0)` — the
  surface distance is **clamped to exactly 0.0** once the obstacle reaches the capsule.
- `cbf_safety_filter.py:495`: `for ld in msg.links if ld.valid` — so at d = 0 the row is
  **dropped**, and the barrier stops constraining precisely at the moment of contact.

h̄ can never go below `−d_safe = −0.20`; the QP loses the row before that.

#### B4 — `min_confidence` is a dead gate

`node_utils.py:198` sets `ld.confidence = 1.0` unconditionally for every link.
`cbf_safety_filter.py:539` compares it against `min_confidence = 0.2`. The test can never fire.

(`build_cp_messages` *does* compute a real confidence via `find_pt_confidence` for the
`MultiDistance` message at `:167` — just not for `MultiLinkDistance`.)

#### B5 — The "hold last value" path produces an indefinitely stale barrier that reports `valid=True`

`distance_engine.py:279-288`: when a CP has no finite measurement (e.g. the ROI has zero valid
depth pixels, `:168-169`), the engine emits the **previous smoothed distance and direction**
with `closest_obstacle_point = None`. `node_utils.py:197` then still marks it `valid`
(d is finite, di is not None). The `_lpf` dict is never aged or expired (only cleared on
resolution change, `:378-382`).

So the CBF can be constrained by a phantom obstacle at a distance measured arbitrarily long
ago, with no way to tell.

#### B6 — `rt_torque_controller` has no staleness check on the feedforward torque

`rt_torque_controller.cpp:61-62` reads `command_buf_` and `:103` applies `input.tau[i]`
unconditionally. Only `accel_buf_` is age-checked (`:66-67`, `accel_timeout` 0.1 s).

If the Python chain dies while `τ_ff ≠ 0`, the 1 kHz loop keeps commanding the last torque
forever. Gravity is firmware-compensated (`:77-78`), so a stale non-zero τ_ff is a *net
accelerating* torque, not a hold.

#### B7 — YAML / docstring claims that contradict the code

- `fr3_control.yaml:99-100` — *"Torque-rate: encoded as QP linear constraint"*. There is no
  torque row in the QP; `max_tau_delta` is unread.
- `fr3_control.yaml:105-106` — `distance_ema_alpha` described as live; it is unread. Commit
  `7714509` deleted the correct "DEPRECATED/UNUSED" comment.
- `cbf_safety_filter.py:9` (and the original brief) say 200 Hz; `fr3_control.yaml:23` sets
  `qp_rate_hz: 100.0`. This matters: `_dt_qp = 1/100 = 10 ms` feeds the one-step velocity bound
  in `hard_accel_box` and the effective jerk cap (`max_qddot_delta` per tick).
- `qddot_to_torque.py:17-19` — *"It is meant to replace `cbf_safety_filter` while the CBF
  formulation is being developed"* — a stale docstring that actively misdescribes the deployed
  architecture.

### 5.2 What would make the filter silently pass through the nominal command

1. **B2** — missing/mismatched remap ⇒ complete bypass, indistinguishable from a working CBF in
   `ros2 node list`.
2. **B1's other half** — during the 0.5 s *before* `distance_timeout` expires, stale-but-not-yet
   -timed-out distances give `n_c = 0` (obstacle rows dropped at `:511-512`) with **no** fault
   flag, so the QP is a pure box-clipped passthrough. Up to 500 ms of unfiltered command at
   every perception hiccup.
3. **B3 / B4** — a link that reports `valid=False` (including at contact) contributes no row at
   all.
4. `cbf_min_leverage = 0.05` (`:559`) silently drops rows where the obstacle normal is nearly
   orthogonal to the achievable Cartesian motion. The warning at `:579-582` is throttled to 2 s.
5. **The publish gate again:** an obstacle at 0.75–1.2 m is *inside* `cbf_obstacle_horizon` but
   never reaches the CBF, so the "continuous, gradual HOCBF engagement from afar" described at
   `:530-538` cannot actually happen — engagement is still a step at 0.70 m.

### 5.3 What would make it over-conservative or oscillatory

1. **The hard workspace rows will engage on the commanded trajectory.**
   Launch forces `center_xyz = [0.4, 0, 0.45]`, `radius = 0.25`, `plane='front'` (YZ) → the EE
   bottoms out at z = 0.20 m. With `ws_min[2] = 0.05` and `ws_margin = 0.02`, h = 0.13 m <
   `ws_horizon = 0.25` ⇒ **a hard row is emitted on every lower arc of the circle**.
   With k0 = 25 that alone demands z̈ ≥ −3.25 m/s²; with a downward 0.5 m/s it demands
   z̈ ≥ +2.0 m/s².
   A hard row **cannot be relaxed by the slack** (`:615`) — if it conflicts with the q̈ box,
   OSQP returns infeasible ⇒ braking ⇒ `fault = 1` ⇒ β = 0
   (**a second, independent path into the same deadlock as B1**).
   Also note `ws_ee_link = fr3_link8` is the **flange origin**, while the commander tracks
   `fr3_hand_tcp` (`pentagon:132`) — roughly 10 cm further out, so the box is applied to a
   different point than the one being controlled.
2. **Slew limit interacting with fallbacks.** `max_qddot_delta = 5.0` rad/s² per 10 ms tick.
   Every fallback (`−k_brake·q̇`, zeros) is forced through it (`:927`). Since `_qddot_prev` is
   the anchor, entering/leaving braking is rate-limited — when `fault` chatters (B1 at the
   0.70 m boundary), β oscillates 0↔1 at 1–4 s⁻¹ while q̈ ramps at ±500 rad/s³. A plausible
   source of visible juddering near the detection boundary.
3. **`n_c` churn forces OSQP `setup()`.** `:813` rebuilds the whole problem (alloc + scaling +
   factorization) whenever `n_c` changes. Workspace rows flicker as the EE crosses `ws_horizon`
   and obstacle rows flicker at the 0.70 m publish gate, so `setup()` can fire repeatedly — the
   `_qp_tick gap` warnings at `:716` would show this.
4. **k1 acts on a 50 Hz-old `a`.** `:781` recomputes `con.A @ qdot` with fresh q̇ but a 20 ms-old
   `A` and a 20 ms-old `ċ`. With k1 = 10.5 amplifying that term, a fast approach mixes a stale
   direction into the bound.
5. **The human is modelled as static.** ḣ omits the obstacle's own velocity entirely (§3.2).
   Against a hand moving at 1 m/s toward a stationary arm, ḣ reads ≈ 0 and the barrier only
   reacts through `k0·h̄` — i.e. it behaves like a *position* CBF, not a velocity-anticipating
   one, exactly in the case the anticipation was meant for.
   **This is the largest gap between what the code is and what a correct CBF of this
   architecture should be.**
6. **Braking is isotropic, not directional.** Every fault path uses `−k_brake·q̇`
   (`:755`, `:797`, `:854`) — it decelerates *all* joints, including motion that is carrying the
   arm *away* from the human.

### 5.4 Open questions and the runtime evidence that settles them

| # | Question | Command / log line |
|---|---|---|
| Q1 | Does `/cbf/per_link_distances` actually go silent when the human steps back? **Confirms or kills B1 — run this first.** | `ros2 topic hz /cbf/per_link_distances` while walking from 0.5 m to 1.5 m and back. Expect: ~30 Hz → *"no new messages"* → ~30 Hz. |
| Q2 | Is `fault` latching? | `ros2 topic echo /NS_1/cbf_status` — watch `data[2]`. Paired with the filter log `distance stale → braking fallback (CBF inactive)` (`:799`). |
| Q3 | Does the commander freeze on it? | `pentagon` CSV / its `beta` diagnostic field; or `ros2 topic echo /NS_1/qddot_nom` — a frozen path shows q̈_nom collapsing to the PD hold term. |
| Q4 | Is the remap live in the actual run? | `ros2 node info /qddot_to_torque` → Subscribers must show `/NS_1/qddot_safe`, **not** `/NS_1/qddot_nom`. The node's own startup line prints `qddot_nom ← /NS_1/qddot_nom` (pre-remap name) — **that log is misleading, don't trust it**. `ros2 topic info /NS_1/qddot_safe --verbose` should list 2 subscribers. |
| Q5 | Do the workspace rows fire on the circle? | Filter log `tick=… n_c=…` (`:837-842`): `n_c > 0` with **no** `CBFDIAG` line following it means the active rows are workspace-only (CBFDIAG is gated on `con.n_obs > 0`, `:892`). |
| Q6 | Is the QP going infeasible? | `QP not solved (…) → braking output` at `:848`, and `iter=… status=…` in the tick line. |
| Q7 | Is the 100 Hz loop keeping time? | `_qp_tick gap: … ms` warnings (`:716`, threshold 30 ms at 100 Hz) and `ros2 topic hz /NS_1/qddot_safe`. |
| Q8 | Is the commanded acceleration realized? | `CBFDIAG` fields `qdd_cmd_rad` vs `qdd_real_rad` and `trk_err` (`:910-915`). `qdd_real_rad ≪ qdd_cmd_rad` while pushing away ⇒ the torque chain, not the CBF, is the problem. |
| Q9 | Does `ws_ee_link=fr3_link8` resolve in the `hand:=false` model? | Startup error `ws_ee_link "fr3_link8" not found in model — workspace rows DISABLED` (`:228-230`). Absence of that line = rows are active. |
| Q10 | Is a gripper physically mounted? | `qddot_to_torque` builds M(q) with `hand:=true` (`kinematics.py:44`) while `franka.config.yaml` sets `load_gripper: "false"`. If no hand is fitted, M(q) over-estimates wrist inertia and τ is systematically high. Compare `/NS_1/torque_cmd` against the measured torque in `franka_robot_state` at rest. |

---

## Bottom line

Two things changed on 2026-09-01 that together produce the observed behaviour.

1. `qddot_to_torque` was flipped to subscribe to the *unfiltered* topic, with a launch remap as
   the only thing restoring correctness (**B2**).
2. The commander gained a governor that hard-freezes the trajectory whenever the CBF raises
   `fault`, and the CBF raises `fault` every time the perception node stops publishing — which
   it does by design whenever no control point sits in the 0.08–0.70 m window (**B1**).

The perception layer itself is behaving as written. The mismatch is that the CBF reads
*"no message"* as *"safety chain broken"* rather than *"nothing nearby"*, and the new governor
escalates that to a full stop.

The **HOCBF math itself is correct** — sign chain, relative-degree-2 formulation, the J̇q̇ term,
the QP row assembly, and the dense-pattern OSQP update were all checked line by line.
The problems are all at the interfaces.
