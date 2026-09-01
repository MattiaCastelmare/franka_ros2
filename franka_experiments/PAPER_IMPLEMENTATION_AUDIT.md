# PAPER ↔ IMPLEMENTATION AUDIT — `franka_experiments`

**Scope:** technical-scientific audit of the `franka_experiments` ROS 2 package
against the five reference papers in `franka_experiments/Papers/`.
**Reviewer stance:** senior robotics / manipulation-control reviewer *and* technical
co-author.
**Date of audit:** 2026-07-10.
**Method:** all five PDFs read in full; ~18 k lines of Python read directly
(nodes, utils, configs, launch, tests); import/call-graph traced to separate
live code from dead code. The README was used only as a hint (Rule 4).

> **Evidence labels used throughout:**
> **[EF]** = evidenza forte (found directly in code/paper),
> **[EM]** = evidenza media (plausible inference from several clues),
> **[IP]** = ipotesi (cannot be verified from the repo alone).

---

## 1. Executive summary

`franka_experiments` is **not** an RL / imitation-learning / diffusion framework
(the prompt's framing does not apply — there is **zero** learning code: no
torch/tf/jax, no policy, no reward, no replay buffer — **[EF]**, verified by
grep). It is a **Control-Barrier-Function (CBF) safety-filtering framework for the
Franka FR3**, implemented as a real-time ROS 2 pipeline with a depth-camera
human-distance estimator. Judged *as a control-systems engineering artefact* it is
**strong**: careful real-time design, hard state-limit shielding, lock-free
multi-rate threading, honest in-code documentation of every design compromise.
Judged *as a faithful reproduction of its five reference papers*, it is **uneven**,
and judged *as a publishable scientific contribution today* it is **not yet ready**
— for reasons that are fixable and, in one case, genuinely promising.

**Which paper is it closest to?**
The framework is closest in *spirit* to **Ferraguti et al. 2022 (RAM tutorial)** —
the generic "CBF-QP as a minimally-invasive safety filter on top of a nominal
controller" methodology — and it contains one **faithful, mathematically exact**
implementation of the **Morton & Pavone 2025 (OSCBF)** task-consistent torque cost
(`cbf_OSCBF_filter.py`). It is *farthest* from the two papers whose actual
scientific content is ISO/TS 15066 Power-and-Force-Limiting (**Ferraguti 2020**)
and the depth-space distance metric (**Flacco 2012**): those specific algorithms
are **replaced**, not reproduced.

**Where it is well aligned.**
- The **acceleration-space HOCBF filter** (`cbf_safety_filter.py`) is a correct
  higher-order CBF (relative degree 2, linear class-K), solved with a warm-started
  native-OSQP loop at 100 Hz, with a genuinely careful *hard state-limit box*
  (velocity + position viability + slew continuity). **[EF]**
- The **OSCBF torque cost** is an *exact* algebraic replica of Morton & Pavone
  Eq. 37–38 (verified by hand: `P = (W_jM⁻¹Nᵀ)ᵀ(W_jM⁻¹Nᵀ)+(W_oJM⁻¹)ᵀ(W_oJM⁻¹)`,
  `Nᵀ = I−JᵀΛJM⁻¹`, `Λ = (JM⁻¹Jᵀ)⁻¹`). **[EF]**
- The **avoidance-first commander** (`pentagon_qddot_commander.py` +
  `utils/avoidance.py`) is an **original synthesis** not present in any of the five
  papers (see §8). **[EF]**
- Real-hardware ROS 2 integration, self-collision capsule geometry parsed from the
  *official* `franka_description` URDF, RealSense depth pipeline. **[EF]**

**Where it is weak or incoherent.**
- **The two ISO/TS 15066 papers are essentially not implemented.** There is no
  relative velocity `v_rel`, no PFL velocity bound `v_PFL=F_max/√(μk)`, no
  reduced-mass/energy term, no ellipse/paraboloid safe set. All barriers are
  distance-only `h=d−d_safe`. Any claim of "ISO/TS 15066 compliance" would be
  **unsupported by the code**. **[EF]**
- **The distance engine is not Flacco's depth-space method.** It uses a plain 3-D
  Euclidean nearest-obstacle-pixel distance in the camera frame; the paper's
  depth-space metric (Eq. 4, gray-area/occlusion handling), repulsive-vector
  sigmoid, and obstacle-velocity *pivot* are absent. **[EF]**
- **The safety claim is not measured.** All evaluation is *tracking* fidelity
  (EE/joint RMS in `experiment_logger.py` + `plot_franka_log.py`). There is **no**
  min-distance metric, **no** barrier-violation (`h<0`) count, **no** baseline,
  **no** ablation, **no** multi-trial statistics, **no** seeds. **[EF]**
- **OSCBF as an integrated system is incomplete and inactive by default.**
  `oscbf_params.yaml` ships `enable_obstacle_cbf: false`, `enable_ws_cbf: false`,
  `enable_vel_cbf: false` → the OSCBF filter enforces *only* joint-position limits
  out of the box. Singularity avoidance, self-collision, whole-body sphere
  containment, and the dynamic-obstacle velocity inflation of the paper are **not**
  implemented, and its obstacle path re-introduces the raw-`closest_point_human`
  phantom-origin bug that `cbf_safety_filter.py` explicitly documents as fixed.
  **[EF]**
- **Dead/legacy code coexists with live code** without markers a newcomer could
  trust: `utils/cbf_qp.py` and `utils/cbf_constraints.py` (both full CBF
  implementations) are imported by *no* node; `utils/self_collision.py` is unit-
  tested but wired into *no* live filter. **[EF]**

**Single biggest risk if you wrote a paper from this code today.**
You would be claiming safety guarantees (and possibly ISO/TS 15066 relevance) that
the *evaluation* does not demonstrate and that parts of the *code* do not implement.
A reviewer would reject on: (a) no safety metric / no baseline / no ablation;
(b) barrier is distance-only while two cited papers require relative velocity /
energy; (c) formal CBF forward-invariance is undermined in practice by the quadratic
slack, the `h_p ≥ 0` relaxation, and discrete-time/latency effects that are
acknowledged in comments but never quantified.

**Top 5 actions to do immediately (detail in §6–7 and the ACTIONS file):**
1. **Instrument safety, not just tracking** — log `min_t h(t)`, violation events,
   slack energy, intervention rate; add them to `plot_franka_log.py`/`report.md`.
2. **Define and freeze a benchmark scenario** (the two rosbags in `datasets/` are a
   start) with a scripted moving obstacle, run *N≥10* seeded trials.
3. **Add ≥2 baselines** you already have the parts for: pure APF/Flacco repulsion,
   and CBF-only-without-avoidance-first — to isolate what the avoidance-first layer
   buys you.
4. **Pick ONE method identity and make it whole** — commit to the *avoidance-first
   HOCBF* as the contribution (it is the novel part) and either (i) implement the
   ISO/TS 15066 `v_rel` barrier to legitimately connect to Ferraguti, or (ii) drop
   the ISO/TS 15066 framing entirely.
5. **Quarantine dead code** (`cbf_qp.py`, `cbf_constraints.py`) and **either wire
   `self_collision.py` into the QP or move it out** — a reviewer reading the repo
   must not mistake unused code for the method.

---

## 2. Repository map

Package root: `/home/mattia/Git/franka_ros2/franka_experiments/` (an
`ament_python` ROS 2 package inside the larger `franka_ros2` workspace). Everything
scientifically relevant lives here; the only external runtime dependency for the
control loop is the **C++** package `franka_rt_controllers` (`rt_torque_controller`,
`rt_velocity_executor_controller`) which is **not** in this package. **[EF]**

### 2.1 Two-and-a-half control stacks

```
TORQUE STACK (canonical — launch/torque_control_stack.launch.py)      [EF]
  pentagon_qddot_commander ──/NS_1/qddot_nom──► cbf_safety_filter ──/NS_1/qddot_safe──►
      (MoveIt path + avoidance-first shaping)        (HOCBF QP, h=d−d_safe)
  qddot_to_torque ──/NS_1/torque_cmd──► rt_torque_controller (C++, +g(q), LPF) ──► HW
      (τ = M q̈ + C q̇)
  real_time_distance ──/cbf/per_link_distances──► (cbf_safety_filter AND commander)

TORQUE STACK 2 (OSCBF — configured in thales.launch.py, NOT the default path)  [EM]
  pentagon_torque_commander ──/NS_1/torque_cmd──► cbf_OSCBF_filter ──/NS_1/torque_safe──► rt_torque_controller
      (6D Cartesian PD + damped-LS)                 (Dynamic-OSCBF QP; obstacle/ws/vel DISABLED by default)

VELOCITY STACK (velocity_cbf_control_stack.launch.py / thales.launch.py)      [EF]
  ee_pentagon_velocity_commander ──/NS_1/tracking_qdot──► cbf_velocity_filter ──/NS_1/qdot_cmd──► rt_velocity_executor_controller
      (Cartesian velocity path)                  (ZCBF RD1: a q̇ ≥ −γ h)
```

### 2.2 Key files (live)

| Concern | File | Role | Alignment target |
|---|---|---|---|
| **HOCBF accel filter** | `nodes/cbf_safety_filter.py` (959) | RD2 HOCBF QP, hard box, workspace rows, native OSQP | RAM'22, OSCBF Eq.5, Landi'19 |
| **OSCBF torque filter** | `nodes/cbf_OSCBF_filter.py` (963) | Dynamic-OSCBF cost + HOCBF constraints | **Morton&Pavone'25** |
| **ZCBF velocity filter** | `nodes/cbf_velocity_filter.py` (621) | RD1 velocity CBF QP | Ferraguti'20/'22, OSCBF kinematic |
| **Avoidance-first commander** | `nodes/pentagon_qddot_commander.py` (1557) | MoveIt path + tangential redirect + null-space repulsion + feasibility governor | **novel** (cf. Flacco'12, De Luca null-space) |
| **Distance estimator** | `nodes/real_time_distance.py` (458) + `utils/distance_engine.py` (393) | depth → per-CP distance | Flacco'12 (loosely) |
| **Avoidance math** | `utils/avoidance.py` (163) | `tangential_redirect`, `feasibility_beta_target`, `influence_weight` | **novel** |
| **Hard limits** | `utils/cbf_hard_limits.py` (137) | viability accel box + slew + workspace rows | (engineering, no paper) |
| **CBF kinematics** | `utils/cbf_kinematics.py` (66) | Pinocchio point-Jacobian `J_p`, `J̇_p` | all |
| **Self-collision geometry** | `utils/self_collision.py` (222) | official-capsule parse + seg/seg distance | OSCBF Eq.51, Landi'19 — **UNWIRED** |
| **Torque converter** | `nodes/qddot_to_torque.py` (205) | `τ = M q̈ + C q̇` (no g) | (engineering) |
| **Logging / plots** | `nodes/experiment_logger.py` (604), `franka_logs/plot_franka_log.py` | CSV + tracking figures + report.md | (tracking only) |

### 2.3 Dead / legacy / unwired code — **[EF]** (verified by grep of imports)

| File | Status | Evidence |
|---|---|---|
| `utils/cbf_qp.py` (`CBFQP`) | **dead** — imported by no node | grep: only self-reference |
| `utils/cbf_constraints.py` (`build_simple_accel_cbf_constraints`, `build_hocbf_constraints`, `predict_state`) | **dead** — `cbf_safety_filter` builds its QP inline; only mentioned in an OSCBF comment | grep |
| `utils/self_collision.py` | **implemented + unit-tested, wired into NO filter** | imported only by `test/test_cbf_hard_constraints.py` |
| `utils/cbf_utils.py::select_gamma` | **live** — used by `cbf_velocity_filter` only | grep |
| `nodes/cbf_OSCBF_filter.py` obstacle/ws/vel CBFs | **implemented, disabled by default** | `oscbf_params.yaml` |

### 2.4 Config / reproducibility surface

`config/fr3_control.yaml` (HOCBF gains, hard-limit margins, workspace box, joint
limits table), `config/oscbf_params.yaml` (OSCBF weights/gains — mostly disabled),
`config/fr3_complete.yaml` (control points per segment, mesh paths, zones),
`config/fr3_distance.yaml`, camera intrinsics/extrinsics. `datasets/` has **2**
rosbags (`bag_01_static`, `bag_02_moving_robot`). `scripts/` has **only**
`bag_to_mp4.py` — **no reproduction / benchmark / seed-sweep script**. **[EF]**

---

## 3. Catalogo dei paper

### P1 — Morton & Pavone, *"Safe, Task-Consistent Manipulation with Operational Space Control Barrier Functions"*, arXiv:2503.06736, 2025 (`OSCBF.pdf`)
- **Abstract (my words):** a CBF safety filter posed in *operational space* whose
  QP objective is **task-consistent** — it minimally perturbs the null-space and
  operational-space accelerations rather than the raw control input, so the filter
  does not degrade the task hierarchy at the limit of safety. Scales to 100s of
  constraints at >1 kHz via Jax/CBFpy.
- **Key contributions:** (1) task-consistent CBF objective for OSC in both joint
  and operational space; (2) kinematic-OSCBF (velocity) and dynamic-OSCBF (torque);
  (3) 5 constraint families + appendix (self-collision, velocity-inflated dynamic
  obstacles).
- **Must-replicate details:** cost `P_QP = NM⁻ᵀWⱼᵀWⱼM⁻¹Nᵀ + M⁻ᵀJᵀWₒᵀWₒJM⁻¹`
  (Eq.38); `Nᵀ=I−JᵀΛJM⁻¹`; HOCBF `L_fh₂+L_gh₂u ≥ −α₂h₂` for RD2 (Eq.5); slack QP
  (Eq.6); constraints Eq.43–48, appendix Eq.49–52 (γ=0.25 velocity inflation).
  Tuning that "worked well": `α=α₂=10`, `Wⱼ=Wₒ=I`.
- **Setup / metrics:** Franka Panda, 1 kHz torque, teleop; safety shown as
  `h(z)>0` maintained across up to 168 concurrent constraints; control-rate tables.
- **Weak points:** input-constrained feasibility only "works in practice"; sphere
  collision model is coarse (21 spheres).

### P2 — Ferraguti et al., *"A Control Barrier Function Approach for Maximizing Performance While Fulfilling to ISO/TS 15066 Regulations"*, IEEE RA-L 2020 (`A_Control_Barrier_Function_Approach…pdf`)
- **Abstract (my words):** encode ISO/TS 15066 **Power-and-Force-Limiting** as a
  *dynamic* safe set in the **(distance, relative-velocity)** plane and enforce it
  with a Zeroing-CBF safety filter, so the robot runs faster than SSM/PFL rules
  normally allow while never exceeding the PFL contact-energy limit.
- **Key contributions:** (1) real-time PFL safe set from current `d` and `v_rel`;
  (2) ZCBF filter enforcing it non-conservatively; (3) UR5 + depth experiments.
- **Must-replicate details:** state `χ=[d, v_rel]`; `v_PFL=F_max/√(μk)` (Eq.5),
  reduced mass `μ=(m_h⁻¹+m_r⁻¹)⁻¹`; deceleration curve `d=½(v_rel−v*)²/a_max`
  (Eq.6); safe region as **n≈100 intersecting ellipses/paraboloids** →
  ZCBF `hᵢ=1−(d−Dᵢ)²/aᵢ²−(v_rel−Vᵢ)²/bᵢ²`, `h=Σhᵢ` (Eq.21–22); accel-level QP
  `min‖ẍᵈᵉˢ−J̇q̇−Jq̈‖²` s.t. CBF + accel/vel bounds (Eq.30); `α(h)=γh`.
- **Setup / metrics:** UR5, RealSense D415, Nuitrack skeleton, OROCOS+ROS, CVXGEN;
  `F_max=60 N, k=40000 N/m, m_h=2 kg, m_r=10 kg`.
- **Weak points:** point-to-point (EE↔hand) only; forward-invariance assumes start
  in the safe set.

### P3 — Ferraguti, Landi, Singletary, Lin, Ames, Secchi, Bonfè, *"Safety and Efficiency in Robotics: The Control Barrier Functions Approach"*, IEEE RAM 2022 (`Safety_and_Efficiency…pdf`)
- **Abstract (my words):** tutorial on CBFs for robotics — from the double
  integrator to industrial manipulators — culminating in an energy-based CBF and a
  delay-compensated kinematic CBF validated on a FANUC with VICON.
- **Key contributions / must-replicate:** the canonical CBF-QP safety filter
  `u*=argmin‖u−u_des‖² s.t. ∂h/∂x·(f+gu) ≥ −α(h)` (Eq.3); manipulator obstacle CBF
  `h=‖p−p_obs‖²−(R+R_w)²` (Eq.9); **energy-based CBF** `h_D=−½q̇ᵀD(q)q̇+γ_c h`
  (Eq.14, RD1, guarantees safety through true dynamics); **delay-compensated
  kinematic CBF** `h(d,ḋ)` with `d=` distance, plus a Savitzky-Golay predictor
  (Eq.17–18).
- **Setup / metrics:** simulations (Code Ocean) + FANUC CR-15iA, VICON MoCap;
  CBF value plots vs `γ`; ~19 µs QP solve.
- **Weak points:** tutorial, not a new method; perception decoupled via MoCap.

### P4 — Flacco, Kröger, De Luca, Khatib, *"A Depth Space Approach to Human-Robot Collision Avoidance"*, IEEE ICRA 2012 (`Flacco_2012_ICRA.pdf`)
- **Abstract (my words):** compute robot↔obstacle distances **directly in depth
  space** (fast, occlusion-aware) and turn them into Cartesian **repulsive vectors**
  that steer a redundant manipulator while it executes a Cartesian task.
- **Key contributions / must-replicate:** depth-space distance metric with
  gray-area handling `‖D(P,O)‖=√(v_x²+v_y²+v_z²)` (Eq.4, only when `d_o>d_p`);
  repulsive sigmoid `v=V_max/(1+e^{(‖D‖(2/ρ)−1)α})` (Eq.7); **multiple control
  points** = spheres along the arm (radius subtracted); EE avoidance
  `ẋ_c=ẋ_d+V_R(P_EE)`, `q̇=J#ẋ_c` (Eq.10–11); **body avoidance = joint-velocity
  constraints** via risk function (Eq.12–14); obstacle-velocity **pivot** algorithm;
  Reflexxes jerk-limited trajectory generation.
- **Setup / metrics:** KUKA LWR IV, Kinect (30 Hz), 1 ms control; ~689 Hz repulsion.
- **Weak points:** APF-style local minima acknowledged; velocity estimation is
  expensive/heuristic.

### P5 — Landi, Ferraguti, Costi, Bonfè, Secchi, *"Safety Barrier Functions for Human-Robot Interaction with Industrial Manipulators"*, ECC 2019 (`Safety_Barrier_Functions…pdf`)
- **Abstract (my words):** whole-body HRI collision avoidance via **Reciprocal
  Barrier Functions** on **capsule subsets** of the arm, solved as an acceleration
  QP that minimally corrects a nominal Cartesian acceleration.
- **Key contributions / must-replicate:** whole-body via subset decomposition
  (subset `i` = first `i` links, midpoint `x_{c,i}`, Jacobian `J_{p,i}`); capsule
  distance `‖x_{c,i}−x_h‖−D_s` (Eq.8–9); **RBF** `B=−ln(h/(1+h))+a_Eb_Eḣ²/(1+b_Eḣ)²`
  (Eq.10); accel QP with per-subset constraints (Eq.17).
- **Setup / metrics:** UR5, Asus Xtion, OpenNI2+Nite2, 25 Hz, `D_s=0.2 m`.
- **Weak points:** RBF blows up at the boundary (numerics); requires accurate
  human capsules.

---

## 4. Paper-to-code mapping

Scale: **5** very faithful · **4** faithful, minor diffs · **3** partially coherent
· **2** only inspired / incomplete · **1** substantially not implemented.

### 4.1 Morton & Pavone 2025 — OSCBF  →  `cbf_OSCBF_filter.py`

**A. What the paper requires:** task-consistent OSC cost (Eq.37/38); null-space
projector `Nᵀ`, operational mass `Λ`; HOCBF RD2 constraints; slack QP; the 5
constraint families (singularity, EE containment, joint limits, whole-body
collision, whole-body containment) + appendix (velocity limits, self-collision,
velocity-inflated dynamic obstacles).

**B. Where it is in the code:**
- Cost: `_OSCBFQPBuilder.build_cost` (lines 203–245) — `Λ` at 218–222, `Nᵀ` at 225,
  `P_arm=A_jᵀA_j+A_oᵀA_o` at 232. **Verified by hand: exact match to Eq.38 with
  scalar `W`.** **[EF]**
- Joint-position HOCBF: `_build_joint_position_cbf` (299–342) — verified consistent
  with Eq.5 given `q̈=M⁻¹(τ−C)` (gravity excluded). **[EF]**
- Joint-velocity RD1 CBF: `_build_joint_velocity_cbf` (345–373). EE workspace RD2:
  `_build_ee_workspace_cbf` (376–420) ↔ Eq.46. Obstacle RD2: `_build_obstacle_cbf`
  (423–526).
- Slack QP: `_OSCBFQPBuilder.solve` (247–294).

**C. Alignment: cost = 5/5; integrated system = 3/5.**

**D. Gap analysis**

| Paper component | In code? | Where | Quality | Divergence | Sci. impact | Priority |
|---|---|---|---|---|---|---|
| Task-consistent cost Eq.38 | ✅ | `build_cost` | **Exact** | scalar `Wⱼ,Wₒ` (paper allows diagonal PD) | low | Low |
| `Nᵀ`, `Λ` | ✅ | 218–225 | good | `Λ` reg `1e-6` (paper JIT/Jax) | low | Low |
| Joint-pos HOCBF | ✅ | 299–342 | good | — | low | — |
| Joint-vel / EE-box / obstacle CBF | ✅ but **OFF** | 345–526 | ok | `enable_*: false` in `oscbf_params.yaml` → only joint-pos active | **high** (method looks active but isn't) | **High** |
| Singularity CBF `μ=Πσ−ε` (Eq.43) | ❌ | — | — | absent | med | Med |
| Whole-body **sphere-decomposition** collision (Eq.47) | ⚠ partial | obstacle path uses per-link depth CP, not the paper's 21-sphere self-model | — | different collision model | med | Med |
| Self-collision (Eq.51) | ❌ in OSCBF (geometry exists unwired in `self_collision.py`) | — | — | not enforced | med | Med |
| Dynamic-obstacle velocity inflation `−γ‖v_rel‖`, γ=0.25 (Eq.52) | ❌ | — | — | absent | med | Med |
| Filtered vs raw obstacle point | ❌ | `_build_obstacle_cbf` uses raw `closest_point_human` (462–465) | **bug** | re-introduces phantom-origin at base when a CP holds its last value — the exact failure `cbf_safety_filter.py:476-497` documents as fixed | **high** | **High** |
| 1 kHz rate | ⚠ | 100 Hz timer | ok | Python+OSQP, not Jax; fine at this constraint count | low | Low |

**E. Technical verdict:** *replica parziale.* The hard, novel part — the
task-consistent cost — is an **exact reproduction** and is a real asset. But as an
integrated safety system OSCBF here is **joint-limits-only by default**, its
obstacle path has a known-class bug, and 3 of the paper's constraint families are
missing. Not sufficient to claim "we reproduce OSCBF" without turning the
constraints on, fixing the obstacle point source, and adding singularity +
self-collision.

---

### 4.2 Ferraguti et al. 2022 — RAM tutorial  →  `cbf_safety_filter.py`, `cbf_velocity_filter.py`

**A. What the paper requires (as a *methodology* to follow):** the CBF-QP safety
filter `min‖u−u_des‖² s.t. Lie-derivative CBF condition ≥ −α(h)`; correct handling
of **relative degree** for manipulators (RD2 → HOCBF); manipulator obstacle CBF;
optionally the **energy-based CBF** (Eq.14) and **delay-compensated `h(d,ḋ)`**
(Eq.17).

**B. Where it is in the code:**
- CBF-QP filter, accel space: `cbf_safety_filter._qp_tick` (704–915) — `min ½‖q̈−q̈_nom‖²+½ρs²`
  s.t. HOCBF rows `aᵀq̈+s ≥ −k1(aᵀq̇)−k0h̄−ċ`. This is the RAM'22 filter with a
  *linear-class-K HOCBF* (RD2) and matches the tutorial's manipulator treatment. **[EF]**
- CBF-QP filter, velocity space: `cbf_velocity_filter._solve_cbf_qp` (384+) —
  `min ½‖q̇−q̇_nom‖²+½ρs² s.t. aᵀq̇ ≥ −γh` (RD1 ZCBF). **[EF]**
- Delay compensation: `cbf_constraints.predict_state` (Eq.-style `q+q̇Δ+½q̈Δ²`) —
  **exists but is dead code** (unused). **[EF]**

**C. Alignment: 3/5.**

**D. Gap analysis**

| Paper component | In code? | Where | Divergence | Priority |
|---|---|---|---|---|
| CBF-QP min-norm filter | ✅ | both filters | quadratic slack `½ρs²` (paper/OSCBF linear `ρt`) — documented, deliberate | Low |
| Relative-degree-2 HOCBF | ✅ | `cbf_safety_filter` | linear class-K, `k0=25,k1=10.5` | — |
| Obstacle CBF | ✅ | per-link `h=d−d_safe` | paper uses `‖p−p_obs‖²−R²`; monotone-equivalent | Low |
| Energy-based CBF (Eq.14) | ❌ | — | not implemented | Med |
| Delay-compensated `h(d,ḋ)` (Eq.17) + predictor | ⚠ dead | `predict_state` unused | perception delay (~30 Hz camera) not compensated in the barrier | **High** |
| `α(h)=γh` tuning study | ⚠ | `select_gamma` schedules γ by clearance (velocity filter only) | no swept-γ safety plot as in the paper | Med |

**E. Verdict:** *replica credibile della metodologia generale.* The framework
faithfully instantiates the tutorial's central pattern (min-invasive CBF-QP on top
of a nominal controller, correct RD handling). It does **not** implement the two
*advanced* devices the tutorial highlights for real manipulators — the energy-based
CBF and the delay-compensated barrier — even though camera latency is exactly the
regime where they matter here.

---

### 4.3 Flacco et al. 2012 — depth-space  →  `real_time_distance.py` + `utils/distance_engine.py`

**A. What the paper requires:** depth-space distance metric with occlusion/gray-area
handling (Eq.4); repulsive-vector sigmoid (Eq.7); multiple control points as spheres
(radius subtracted); EE repulsion `ẋ_c=ẋ_d+V_R`; body avoidance as joint-velocity
constraints; obstacle-velocity pivot; Reflexxes trajectory generation.

**B. Where it is in the code:**
- Depth pipeline + control points: `distance_engine.compute` (100–243) — unprojects
  obstacle pixels, projects control points, per-CP nearest pixel, **radius
  subtracted** (`surface = dist_3d − radius − margin`, line 211). Control points per
  segment defined in `fr3_complete.yaml` (`control_points: 2/3`, `radius: 0.05`).
  Robot self-pixels excluded via dilated mask (`mask_builder.py`). **[EF]**
- **What is *not* Flacco:** the distance is a plain 3-D Euclidean
  `‖p_cam−cp_cam‖` (line 209–211), **not** the depth-space metric Eq.4 with the
  `d_o>d_p` gray-area rule. No repulsive-vector sigmoid. No pivot / obstacle
  velocity. No Reflexxes. The output (`distance`, EMA-smoothed `direction`) feeds a
  **CBF**, not a potential field. **[EF]**

**C. Alignment: 2/5.**

**D. Gap analysis**

| Paper component | In code? | Where | Divergence | Priority |
|---|---|---|---|---|
| Depth camera + unprojection | ✅ | `distance_engine` | camera-frame subtraction (efficient) | — |
| Multiple control points (spheres, r subtracted) | ✅ | `fr3_complete.yaml`, `compute` | faithful in spirit | — |
| Robot-body pixel exclusion | ✅ | `mask_builder.py` (dilation) | improved (downsampled dilation) | — |
| **Depth-space metric Eq.4 (gray area)** | ❌ | plain 3-D Euclidean | occluded-obstacle case handled differently | Med |
| **Repulsive sigmoid Eq.7** | ❌ | — | replaced by CBF | (by design) |
| Obstacle velocity / pivot | ❌ | direction EMA only | dynamic obstacles not velocity-compensated | **High** (safety-relevant) |
| Reflexxes jerk-limited OTG | ❌ | — | replaced by slew-rate QP box | Low |
| Anti-spike / LPF robustness | ✅ **beyond paper** | `_lpf_pass` + approach-spike confirmation | *original robustness layer* | (asset) |

**E. Verdict:** *re-interpretation.* The **perception architecture** (depth camera +
sphere control points + mask exclusion) is legitimately Flacco-inspired and, in the
LPF/anti-spike logic, improves on it. But the paper's **named algorithmic
contributions** (depth-space distance math, repulsive vectors, velocity pivot) are
absent. Do not cite Flacco as "the distance method"; cite it as inspiration for the
control-point/depth-camera setup.

---

### 4.4 Landi et al. 2019 — Safety Barrier Functions HRI  →  `cbf_safety_filter.py` (whole-body) + `utils/self_collision.py` (unwired)

**A. What the paper requires:** whole-body avoidance via capsule *subsets*, RBF
`B=−ln(h/(1+h))+…`, acceleration QP with per-subset constraints.

**B. Where it is in the code:**
- Whole-body **per-link** avoidance IS present: `cbf_safety_filter._update_constraints`
  (501–621) builds one HOCBF row per `MultiLinkDistance` link (link-point Jacobian
  `n̂ᵀJ_p`). This captures the paper's *core idea* (constrain the whole body, not
  just the EE). **[EF]**
- Capsule geometry: `self_collision.py` parses the **official** `*_sc` capsules and
  does segment/segment distance (Ericson) — high quality, unit-tested — but it is
  **self-collision** and **wired into no live QP**. **[EF]**

**C. Alignment: 2/5.**

**D. Gap analysis**

| Paper component | In code? | Where | Divergence | Priority |
|---|---|---|---|---|
| Whole-body (not just EE) | ✅ | per-link CBF rows | per-link points, not subset midpoints | Low |
| Capsule model | ⚠ | `self_collision.py` | present, **unwired**; used for *self*-collision not human capsules | Med |
| **RBF `B=−ln(h/(1+h))+…`** | ❌ | linear-class-K HOCBF instead | different barrier family (ZCBF-style, vanishes at boundary) | Low (design choice) |
| Subset Jacobians `J_{p,i}` | ❌ | link-point Jacobians | equivalent enough | Low |
| Accel QP min-correction | ✅ | `_qp_tick` | matches | — |

**E. Verdict:** *inspired / partial.* The whole-body acceleration-QP philosophy is
reproduced; the specific RBF formulation and capsule-subset construction are not,
and the capsule code that *would* connect to this paper is dead. Cite Landi for the
whole-body accel-QP idea, not for the RBF.

---

### 4.5 Ferraguti et al. 2020 — ISO/TS 15066 PFL  →  **no dedicated implementation**

**A. What the paper requires:** state `χ=[d, v_rel]`; PFL bound `v_PFL=F_max/√(μk)`;
reduced mass; deceleration curve; ellipse/paraboloid safe-set approximation; ZCBF
`h=Σhᵢ`; accel QP `min‖ẍᵈᵉˢ−J̇q̇−Jq̈‖²`.

**B. Where it is in the code:** **Nowhere.** Grep for `v_rel|vrel|iso.?ts|15066|pfl|
reduced_mass|energy.?based` → **0 hits** in `franka_experiments/`. The closest
artefact is the accel-QP *structure* in `cbf_safety_filter._qp_tick`
(`min‖q̈−q̈_nom‖²` s.t. HOCBF) which shares the paper's QP *skeleton* but uses a
**distance-only** barrier `h=d−d_safe`, no relative velocity, no PFL, no ellipses,
no energy/force model. **[EF]**

**C. Alignment: 2/5** (only the generic accel-CBF-QP scaffolding is shared).

**D. Gap analysis**

| Paper component | In code? | Divergence | Priority |
|---|---|---|---|
| Accel-level CBF-QP filter | ✅ (structure) | shared skeleton only | — |
| State `χ=[d, v_rel]` | ❌ | barrier ignores relative velocity | **High** (this *is* the paper) |
| `v_PFL=F_max/√(μk)`, reduced mass, energy | ❌ | no force/energy model at all | **High** |
| Ellipse/paraboloid safe set (Eq.21–22) | ❌ | single linear barrier | Med |
| ISO/TS 15066 compliance claim | ❌ | unsupported | **High** |

**E. Verdict:** *implementazione non sufficiente per un claim di riproduzione.* The
framework is **not** an ISO/TS 15066 system. Either implement the `[d,v_rel]`
PFL barrier (a well-scoped, high-value addition — see Proposal 3) or remove the
ISO/TS 15066 framing from README/paper. As written, citing Ferraguti 2020 as
"implemented" would be indefensible in review.

---

### 4.6 Alignment scoreboard

| Paper | Alignment | One-line verdict |
|---|---|---|
| Morton & Pavone 2025 (OSCBF) | **3/5** (cost 5/5) | Exact task-consistent cost; constraint suite reduced + off by default; obstacle bug |
| Ferraguti 2022 (RAM tutorial) | **3/5** | Faithful to the generic CBF-QP methodology; advanced variants (energy, delay) missing |
| Flacco 2012 (depth-space) | **2/5** | Depth-camera + control-point setup inspired by it; the depth-space math + repulsion replaced |
| Landi 2019 (SBF-HRI) | **2/5** | Whole-body accel-QP idea present; RBF + capsule-subsets absent (capsule code dead) |
| Ferraguti 2020 (ISO/TS 15066) | **2/5** | Only the QP skeleton is shared; `v_rel`/PFL/energy/ellipses not implemented |

---

## 5. Analisi trasversale del framework

| # | Dimension | Rating | Evidence & reasoning |
|---|---|---|---|
| 1 | **Modularity** | 7/10 | Clean nodes/utils split; pure-numpy math modules (`avoidance.py`, `cbf_hard_limits.py`, `self_collision.py`) are testable in isolation. **But** three parallel CBF implementations (`cbf_safety_filter` inline, `cbf_qp.py`, `cbf_constraints.py`) with no "this is the real one" marker. **[EF]** |
| 2 | **Architecture clarity** | 8/10 | Excellent in-code docstrings; the multi-rate lock-free design in `cbf_safety_filter.py` is documented to a rare standard. Cross-node data flow is clear from launch files. **[EF]** |
| 3 | **Reproducibility** | 3/10 | No seeds, no one-command experiment runner, no pinned deps (README `pip install` is unversioned), no config hashing, no result manifest. `datasets/` has 2 rosbags but no script that turns them into a paper figure. **[EF]** |
| 4 | **Train/eval/deploy separation** | N/A→6/10 | No training (not ML). Deploy (launch) is clean; "eval" is only the tracking logger. No held-out scenario set. **[EF]** |
| 5 | **Logging quality** | 6/10 | `experiment_logger.py` writes a wide CSV; `plot_franka_log.py` auto-generates figures + `report.md`. **But** it logs/plots **tracking**, not **safety** (no `h(t)`, no min-distance, no slack, no violation). **[EF]** |
| 6 | **Configurability** | 8/10 | Rich YAML; most gains/margins exposed; launch args for camera/fake-hw/namespace. OSCBF fully parameterised. **[EF]** |
| 7 | **Testability** | 5/10 | Good **unit** tests of pure math (`test_avoidance.py`, `test_cbf_hard_constraints.py`, seg/seg, slew, viability box, with random property checks). **No** integration test that asserts a CBF invariant (`h≥0`) over a simulated trajectory; **no** QP-feasibility regression. **[EF]** |
| 8 | **Experiment management** | 2/10 | Timestamped output folders exist (`franka_logs/plots_*`), but no experiment registry, no parameter capture per run, no comparison harness. **[EF]** |
| 9 | **Real-robot robustness** | 8/10 | This is the strongest axis: staleness timeouts, braking fallbacks, hard viability box, slew continuity, SCHED_FIFO elevation, warm-up to hide first-call latency, anti-spike distance filter. All safety-relevant and well-reasoned. **[EF]** |
| 10 | **Technical debt** | 5/10 | Dead code (`cbf_qp.py`, `cbf_constraints.py`), unwired `self_collision.py`, "TEMPORARY" isolation-test branch inside the live commander (`pentagon_qddot_commander.py:801`), a diagnostic `q̈_real` estimator carried in the hot path. **[EF]** |
| 11 | **Methodological-error risk** | 6/10 | Real risks: (a) **quadratic** slack + `h_p≥0` relaxation + discrete-time → forward-invariance is *softly* enforced, not guaranteed (all acknowledged in comments, none quantified); (b) `ċ` (`J̇q̇`) frozen at the 50 Hz snapshot while `aᵀq̇` refreshes at 100 Hz — a documented inconsistency; (c) obstacle direction from a single argmin pixel is noisy. **[EF]/[EM]** |
| 12 | **Reusable strengths for a paper** | — | Avoidance-first shaping (novel); exact OSCBF cost; hard viability shielding; official-capsule self-collision math; real FR3 + RealSense pipeline. |

---

## 6. Cosa manca per un paper serio

### 6.1 Rigore sperimentale — **the critical gap**
- **No safety metric.** You must report, at minimum: `min_t h(t)` per trial,
  number/duration of `h<0` events, slack energy `∫s²`, minimum robot↔obstacle
  distance, and intervention rate (fraction of ticks with an active binding CBF
  row). None exist today. **[EF]**
- **No baselines.** A CBF-avoidance paper needs at least: (i) *no filter* (unsafe
  reference), (ii) *pure APF/Flacco repulsion*, (iii) *CBF-only* (no avoidance-first
  shaping). You have the parts for all three.
- **No ablations.** The avoidance-first layer has ≥3 separable components
  (tangential redirect, null-space repulsion, feasibility governor) — each must be
  toggled and measured.
- **No multi-seed / statistics.** No confidence intervals, no repeated trials, no
  obstacle-trajectory randomization. Control is deterministic, but obstacle motion
  and start states are not controlled/seeded.
- **No train/test-style split.** At least a *held-out* obstacle scenario the gains
  were **not** tuned on.
- **No sim-vs-real.** `use_fake_hardware:=true` exists; there is no reported
  sim↔real gap.

### 6.2 Rigore implementativo
- **Pin dependencies** (Pinocchio, qpsolvers/osqp, numpy, scipy versions).
- **One-command reproduction** (`ros2 launch … + record + plot` → figure) per
  experiment; today it is manual.
- **Determinism / config capture:** hash and store the exact YAML + git SHA per run
  in the output folder.
- **CI:** the unit tests should run in CI; add a headless `use_fake_hardware`
  integration smoke test that asserts `h≥0`.
- **Automatic report** already half-exists (`report.md`) — extend it to safety.

### 6.3 Novità scientifica
- Today, *taken constraint-by-constraint*, the framework reads as a **partial
  re-implementation** of known CBF methods (OSCBF cost + generic RAM'22 filter +
  Flacco-ish perception). That alone is **not publishable** — reviewers will ask
  "what is new vs. Morton & Pavone / Ferraguti?".
- **However**, the **avoidance-first architecture** (steer-not-brake at the
  commander + CBF as a *certificate* + **feasibility-governed** speed, never
  distance-governed) is a genuinely under-explored combination and is the realistic
  path to novelty (§8).

---

## 7. Roadmap per paper publishable + possibile contributo SOTA

Proposals are ranked by (impact × feasibility). P0 = do first.

### Proposta 1 — Safety-instrumented, baselined, ablated benchmark
**Idea.** Build the evaluation the method deserves: a fixed FR3 + scripted moving
obstacle scenario (extend the two rosbags), logging safety metrics, run with N≥10
seeded obstacle trajectories, against 3 baselines and 3 ablations.
**Motivazione.** Without this there is no paper — every reviewer asks for it first.
**Relazione coi paper.** Provides the missing empirical spine for the OSCBF/RAM'22
comparison and legitimises any novelty claim.
**Modifiche al codice.** `experiment_logger.py` + `plot_franka_log.py` (+safety
signals; subscribe `/NS_1/cbf_status`, `/cbf/per_link_distances`); new
`scripts/run_benchmark.py` + `scripts/aggregate.py`; a `bypass`/ablation param
matrix (mostly already exposed).
**Difficoltà** Media · **Rischio** Basso · **Impatto** performance n/a, robustness↑↑,
novelty (enables it), publishability↑↑↑ · **Priorità P0.**
**Esperimenti.** For each of {unsafe ref, APF, CBF-only, CBF+avoidance-first} ×
{static, slow, fast obstacle}: report `min h`, violations, min-distance, tracking
RMS, task-completion time, intervention rate — mean±std over seeds.

### Proposta 2 — Formalise & measure "avoidance-first + CBF certificate"
**Idea.** Promote the current implicit architecture to the paper's thesis: the
*commander* generates a **feasible, task-consistent avoidance direction** (tangential
redirect + null-space repulsion), the CBF is a **minimal certificate**, and speed is
reduced **only** from feasibility evidence (slack/fault/manipulability), never from
distance. Prove: (a) fewer/zero hard CBF interventions vs CBF-only, (b) higher task
throughput at equal safety, (c) no "parking in front of obstacles".
**Motivazione.** This is the one thing not in any of the five papers; it directly
attacks the classic CBF failure mode (the filter parks the robot at the boundary).
**Relazione coi paper.** *Extends* OSCBF (task-consistency) and Flacco (repulsion)
and *combines* them under a CBF certificate (RAM'22) — a defensible new angle.
**Modifiche al codice.** Mostly present (`avoidance.py`, commander lines 805–958);
needs: clean parameterisation, the governor's stability argument, and metrics from
P1.
**Difficoltà** Media · **Rischio** Medio (must show it beats CBF-only convincingly)
· **Impatto** novelty↑↑↑, publishability↑↑ · **Priorità P0/P1.**
**Esperimenti.** Ablations from P1 + a "boundary-parking" metric (time spent within
ε of `h=0`) + governor auto-resume traces.

### Proposta 3 — Implement the ISO/TS 15066 `[d, v_rel]` PFL barrier (make Ferraguti'20 real, or drop it)
**Idea.** Add a relative-velocity, energy-aware barrier: track `v_rel` from the
per-CP direction and joint velocities, add the PFL bound `v_PFL=F_max/√(μk)` and the
`χ=[d,v_rel]` safe set (start with the single-parabola `h(d,ḋ)` from RAM'22 Eq.17
before the 100-ellipse version).
**Motivazione.** Turns a currently *false* alignment into a real one and gives the
paper a certifiable-safety hook that industrial reviewers value.
**Relazione coi paper.** Directly implements Ferraguti 2020 / RAM'22 §experimental.
**Modifiche al codice.** New barrier builder in `cbf_safety_filter` (a `v_rel` row
alongside `h=d−d_safe`); `real_time_distance` already publishes closest points and
directions — add relative velocity from `J_p q̇`.
**Difficoltà** Media · **Rischio** Basso · **Impatto** novelty↑ (relative to your
current code), publishability↑ · **Priorità P1.** *Alternative if descoping: delete
all ISO/TS 15066 language from README and paper.*

### Proposta 4 — Wire self-collision + singularity into the live QP
**Idea.** Connect `self_collision.py` (already parses official capsules) as CBF rows
in `cbf_safety_filter`/`cbf_OSCBF_filter`, and add the OSCBF singularity CBF
`h=√det(JJᵀ)−ε` (the commander already computes `w=√det(JJᵀ)`).
**Motivazione.** Two OSCBF constraint families that are *almost free* to add and
make the "whole-body safety" claim honest.
**Relazione coi paper.** OSCBF Eq.43, Eq.51.
**Difficoltà** Bassa–Media · **Rischio** Basso · **Impatto** robustness↑, OSCBF
alignment 3→4 · **Priorità P1.**
**Esperimenti.** Show `min self-clearance > 0` and no manipulability collapse over
the benchmark.

### Proposta 5 — Fix the OSCBF obstacle path + turn constraints on
**Idea.** Use the *filtered* distance/direction (as `cbf_safety_filter` does),
enable obstacle/ws/vel CBFs by default in a tested config, add the velocity-inflated
dynamic-obstacle barrier (γ=0.25, Eq.52).
**Difficoltà** Bassa · **Rischio** Basso · **Impatto** OSCBF becomes a real
reproduction · **Priorità P1.**

### Proposta 6 — Delay-compensated barrier (RAM'22 Eq.17–18)
**Idea.** Compensate the ~30 Hz camera latency in the barrier with the Savitzky-
Golay-style predictor the paper uses; `predict_state` already exists (dead) as a
starting point.
**Difficoltà** Media · **Rischio** Medio · **Impatto** safety-at-speed↑ ·
**Priorità P2.**

### Proposta 7 — Quantify the guarantee gap
**Idea.** Empirically characterise how far the *practical* filter is from the
*theoretical* CBF: sweep `γ`/`k0,k1`, plot `min h` vs latency and vs slack penalty
`ρ`; document when `h<0` occurs. Honesty about this is itself a contribution and
disarms the strongest reviewer objection.
**Difficoltà** Bassa · **Rischio** Basso · **Impatto** publishability↑ ·
**Priorità P2.**

---

## 8. Possibile contributo originale / stato dell'arte

**Is there a realistic original contribution?** **Yes — one, clearly.** Not the CBF
math (that reproduces OSCBF/Ferraguti), but the **avoidance-first control
architecture**: a *task-consistent motion generator that produces a feasible
avoidance direction*, wrapped by a *CBF certificate*, with speed modulated by a
*feasibility governor* (slack/fault/manipulability) rather than by distance. In the
five papers, the CBF is always the *primary* avoidance mechanism (which is what makes
CBF robots "park" at the boundary); here the CBF is demoted to a certificate and the
*commander* does the steering. That inversion, with the feasibility governor and the
explicit "slow down only as last resort" principle, is not in any of P1–P5 and speaks
directly to a known weakness of CBF filters. **[EF] for the code; [IP] for novelty
vs. the wider literature — a proper related-work search is required.**

**Strongest paper angle.** *"Avoidance-first, certificate-second"*: keep the OSCBF
task-consistent cost as the certificate, put the intelligence in the commander,
and show it dominates CBF-only on throughput at equal safety while eliminating
boundary-parking.

**Credible vs. too-weak claims.**
- *Credible:* fewer hard CBF interventions & no boundary-parking vs CBF-only;
  task-consistency preserved under avoidance; auto-resume after obstacle clears;
  real FR3 + RealSense demonstration.
- *Too weak / unsupportable today:* "formally guarantees safety" (slack + latency);
  "ISO/TS 15066 compliant" (no `v_rel`/energy); "reproduces OSCBF" (reduced/off
  constraints); "novel distance estimator" (it is simplified Flacco).

**Provisional title.**
> **"Avoidance-First Manipulation: Task-Consistent Reactive Steering with Control-Barrier-Function Certificates for Human-Robot Coexistence on the Franka FR3"**

**Abstract (draft, ~12 lines).**
> Control-Barrier-Function (CBF) safety filters guarantee collision avoidance but,
> being minimally invasive, they tend to *stop* the robot at the boundary of the
> safe set, sacrificing task throughput. We propose an *avoidance-first*
> architecture in which a task-consistent motion generator produces a **feasible**
> avoidance direction — rotating the commanded end-effector acceleration into the
> obstacle's tangent plane and injecting null-space repulsion at zero task cost —
> while a CBF QP acts only as a **certificate** on the resulting command. Crucially,
> the robot's speed is modulated by a **feasibility governor** driven by QP slack,
> safety-chain faults, and manipulability, *never* by raw distance, so the arm slows
> only when no admissible motion remains and resumes automatically. We implement the
> full pipeline in real time on a Franka FR3 with a RealSense depth estimator, using
> the task-consistent operational-space CBF cost of Morton & Pavone as the
> certificate. Across static and dynamic-obstacle benchmarks we show equal or better
> safety (minimum barrier value, zero violations) at substantially higher task
> throughput and near-zero boundary-parking compared with a standard CBF filter and
> an artificial-potential-field baseline.

**3–5 contributions.**
1. Avoidance-first architecture separating *direction generation* (commander) from
   *safety certification* (CBF).
2. A *feasibility governor* for speed (slack/fault/manipulability), not distance.
3. Task-consistent tangential redirection + null-space repulsion that preserves the
   OSCBF task hierarchy.
4. Real-time FR3 + depth-camera system + an open safety-instrumented benchmark.
5. An honest characterisation of the practical-vs-theoretical CBF guarantee gap.

**Experiments.** §7-P1 matrix. **Baselines:** unsafe reference, APF/Flacco,
CBF-only, (stretch) OSCBF-as-published. **Ablations:** −redirect, −null-repulsion,
−governor, distance-governed vs feasibility-governed. **Rejection risks:** novelty
vs OSCBF questioned (mitigate: the certificate is OSCBF, the *architecture* is the
claim); safety not formally guaranteed (mitigate: report the gap, don't overclaim);
single-robot/single-lab (mitigate: sim + real, seeds, release code+bags).

**8–12 week plan:** see `PAPER_SUBMISSION_PLAN.md`.

---

## 9. Tabella finale Stop / Keep / Build

| KEEP (già buono) | FIX (prima di qualsiasi paper) | BUILD (alto valore / novelty) |
|---|---|---|
| Avoidance-first commander (`avoidance.py`, commander 805–958) | Add safety metrics to logger/plots (`experiment_logger.py`, `plot_franka_log.py`) | Safety-instrumented, seeded, baselined, ablated **benchmark** (P1) |
| Exact OSCBF task-consistent cost (`cbf_OSCBF_filter.build_cost`) | Enable + fix OSCBF obstacle path (raw→filtered point; `enable_*` on) | Formalise **avoidance-first + certificate + feasibility governor** as the paper (P2) |
| HOCBF accel filter + hard viability box (`cbf_safety_filter`, `cbf_hard_limits`) | Remove/quarantine dead code (`cbf_qp.py`, `cbf_constraints.py`) | Implement **`[d,v_rel]` PFL barrier** or drop ISO/TS 15066 framing (P3) |
| Official-capsule self-collision math (`self_collision.py`) | **Wire** self-collision into the live QP (or move it out) | **Singularity + self-collision** CBF rows (P4) |
| Anti-spike distance LPF (`distance_engine`) | Pin deps; one-command repro; seed obstacle trajectories | **Delay-compensated** barrier for the 30 Hz camera (P6) |
| Real-time robustness (timeouts, fallbacks, SCHED_FIFO, warm-up) | Correct the "distance method = Flacco" claim in README | Characterise the **theoretical-vs-practical guarantee gap** (P7) |
| Rich YAML configurability | Remove "TEMPORARY" isolation branch from the live commander | Sim↔real gap study with `use_fake_hardware` |

---

### Appendix A — Facts vs. hypotheses ledger
- **[EF]** No learning code; no `v_rel`/ISO-TS-15066/energy terms; OSCBF cost = exact
  Eq.38; OSCBF constraints off by default; dead `cbf_qp.py`/`cbf_constraints.py`;
  `self_collision.py` unwired; distance = 3-D Euclidean not depth-space; evaluation =
  tracking only; 2 rosbags; unit-tests present, no CBF-invariant integration test.
- **[EM]** OSCBF is not the canonical live pipeline (config + memory notes); `ċ`
  frozen-at-snapshot vs `aᵀq̇` refreshed introduces a small model inconsistency.
- **[IP]** Novelty of avoidance-first *relative to the full literature* (needs a
  related-work search beyond these five papers); whether the practical guarantee gap
  is large enough to matter at the target speeds (must be measured — P7).
