# Appendix B — Equation index

> **Generated** (Stage 7) by `tools/stage7_verification/gen_equation_index.py` from the numbered equations of chapters 04–13. Do not edit by hand: change the chapter and regenerate. Baseline `ba68077`. The chapter is the evidence (its labels, its `file::function` citations, its verification scripts); this index only points to it. "Code" lists the code tokens the chapter writes next to the equation, which is where to look first, not a complete call graph.

122 numbered equations in 10 chapters; 106 carry at least one code pointer in their chapter; 16 do not (listed at the end).

## [04_math_foundations.md](04_math_foundations.md)

| Eq. | Line | Statement (from the chapter) | Code (chapter pointers) | Also cited in |
|---|---:|---|---|---|
| (4.1) | 115 | ⁰T_i(q) = ⁰T_{i−1}(q) · X_i · Rot_z(q_i),  ⁰T_0 = I | `compute_ee_fk`, `forwardKinematics`, `updateFramePlacement`, `data`, `compute_arm_jacobian` | — |
| (4.2) | 143 | [ v_f ; ω_f ] = J_f(q) q̇,  J_f = [ J_v ; J_ω ] ∈ ℝ^{6×n}. | `compute_ee_fk`, `forwardKinematics`, `updateFramePlacement`, `data`, `compute_arm_jacobian` | — |
| (4.3) | 157 | J_v[:, j] = a_j × (o_f − o_j),  J_ω[:, j] = a_j  (and 0 for joints not upstream of f). | — | — |
| (4.4) | 301 | rigid tip J_P \| `point_jacobian_pos` / `point_jacobian`: `J6[:3] − skew(r) @ J6[3:] | `point_jacobian_pos`, `point_jacobian`, `check_kinematics_dynamics.py`, `check_geometry_and_so3.py`, `so3_log` | — |
| (4.5) | 302 | rigid tip J̇_P \| `point_jacobian`: `Jvd − skew(r) @ Jwd − skew(cross(ω, r)) @ Jw`, with ω = `getFrameVeloci… | `fr3_link5`, `point_jacobian_pos`, `point_jacobian`, `check_kinematics_dynamics.py`, `check_geometry_and_so3.py` | — |
| (4.6) | 180 | p̈_P = J_P q̈ + J̇_P q̇. | `fr3_link5`, `check_kinematics_dynamics.py`, `check_geometry_and_so3.py`, `so3_log`, `check_home_example.py` | — |
| (4.7) | 297 | , interpolated sample \| `p0, p1 = data.oMf[f0].translation, data.oMf[f1].translation`; `(1−t)p0 + t p1 | `seg_idx`, `cp_idx`, `radius` | — |
| (4.8) | 299 | J \| `getFrameJacobian(f0, LWA)[:3]`, `getFrameJacobian(f1, LWA)[:3]`, then `(1−t)J0 + t J1 | `point_jacobian_pos`, `point_jacobian` | — |
| (4.9) | 211 | M(q) q̈ + C(q, q̇) q̇ + g(q) = τ + τ_ext, | — | — |
| (4.10) | 222 | τ_ff = M_aa(q) q̈_a + [nle(q, q̇) − g(q)]_a, | `pentagon_qddot_commander._compute_tau_des`, `MultiLinkDistance`, `qddot_to_torque`, `qddot_safe`, `KeyError` | — |
| (4.11) | 231 | log(R)^∨ = θ a,  with a = vee(R − Rᵀ)/(2 sin θ) for 0 < θ < π. | `transform_ee_to_frame`, `ref`, `so3_log`, `pin.log3`, `dls_solve` | — |
| (4.12) | 237 | log(R_d R_cᵀ) = −R_d log(R_dᵀ R_c), | — | — |
| (4.13) | 245 | q̇ = Jᵀ (J Jᵀ + λ² I)⁻¹ v  minimises ‖J q̇ − v‖² + λ² ‖q̇‖². | `so3_log`, `pin.log3`, `dls_solve`, `pinv`, `JointStateManager` | — |
## [05_perception_and_distance.md](05_perception_and_distance.md)

| Eq. | Line | Statement (from the chapter) | Code (chapter pointers) | Also cited in |
|---|---:|---|---|---|
| (5.1) | 74 | p_C = Z · K⁻¹ [u, v, 1]ᵀ = ( (u − c_x) Z / f_x, (v − c_y) Z / f_y, Z ). | `distance_engine.compute`, `image_rect_raw` | — |
| (5.2) | 80 | p_B = R_b p_C + t_b,  and back with  p_C = R_bᵀ (p_B − t_b). | `ee_link`, `fr3_link8`, `np.minimum.at` | — |
| (5.3) | 93 | d_i = max( min_{o ∈ O} [ ‖o − c_i‖ − r_i − m(o) ], 0 ), | `distance_utils.define_robot_segments`, `compute_closest_distance_from_segments`, `direction`, `distance`, `valid` | — |
| (5.4) | 96 | robot**). | — | — |
| (5.5) | 155 | keep ⇔ ¬mask(u, v) ∨ [ 0 < Z(u, v) < (z_rob(u, v) − b) − τ ], | `depth_gate_bias_m`, `classify_masked_rays`, `ssb_guard_m`, `robot_mask_dilate_px`, `ee_mask_dilate_px` | — |
| (5.6) | 169 | m(o) = M_px(o) · Z(o) / f_x,  M_px = 24 if the pixel's provenance is the EE dilation, else 12. | `check_perception_numbers.py`, `ee_source_mask`, `segment_links`, `build_cp_messages`, `test_cp_label_agreement.py` | — |
| (5.7) | 527 | F(Δt) = [[1, Δt, Δt²/2], [0, 1, Δt], [0, 0, 1]] ⊗ I₃, | `obstacle_tracker.KalmanTrack` | — |
| (5.8) | 529 | Q(Δt) = q · [[Δt⁵/20, Δt⁴/8, Δt³/6], [Δt⁴/8, Δt³/3, Δt²/2], [Δt³/6, Δt²/2, Δt]] ⊗ I₃. | `check_tracker.py` | — |
| (5.9) | 560 | P⁰_j = Σ_i μ_{i\|j} [P_i + (x_i − x⁰_j)(x_i − x⁰_j)ᵀ]; | `obstacle_tracker.IMMTrack` | — |
| (5.10) | 562 | per-model Joseph update; Λ_j = N(y_j; 0, S_j), computed in the log domain; μ_j ∝ c_j Λ_j; | `obstacle_tracker.IMMTrack` | — |
| (5.11) | 564 | x = Σ μ_j x_j,  P = Σ μ_j [P_j + (x_j − x)(x_j − x)ᵀ]. | `mode_prob`, `ballistic_prediction` | — |
| (5.12) | 631 | g(τ) = ‖p(τ) − c‖ − r − r_obj,  r_obj = 0.035 m. | `ballistic_prediction.predict_trajectory`, `ballistic_prediction.add_predicted_hits` | — |
| (5.13) | 652 | spread = ‖max_k (p_h,k − p_r,k) − min_k (p_h,k − p_r,k)‖. | — | — |
| (5.14) | 703 | X B_i = A_i Y,  E_i = (X B_i)⁻¹ (A_i Y) ≈ I. | `handeye_solver.initial_guess`, `handeye_solver.refine` | — |
| (5.15) | 730 | ᴮT_E(i) · X · ᶜT_T(i) = Y,  X = ᴱT_C,  Y = ᴮT_T. | `handeye_solver.solve`, `test_handeye_solver.py` | — |
## [06_nominal_motion.md](06_nominal_motion.md)

| Eq. | Line | Statement (from the chapter) | Code (chapter pointers) | Also cited in |
|---|---:|---|---|---|
| (6.1) | 84 | p_d = P(s), | `pentagon_qddot_commander._tick`, `_step_nominal` | — |
| (6.2) | 85 | v_d = P′(s)·ṡ, | `GeometricPath.velocity` | — |
| (6.3) | 86 | a_d = P″(s)·ṡ² + P′(s)·s̈. | `GeometricPath.velocity` | — |
| (6.4) | 137 | e = sat_{0.08 m}(p_d − p),  ė = v_d − J_v q̇; | `pentagon_qddot_commander._step_nominal` | — |
| (6.5) | 149 | ẍ_cmd = [ a_d + K_p e + K_d ė ;  K_pR e_R + K_dR ė_R ]. | `path_kp_cart`, `path_kd_cart` | — |
| (6.6) | 166 | q̈_task = J⁺(ẍ_cmd − J̇ q̇), | `pentagon_qddot_commander._step_nominal` | — |
| (6.7) | 167 | λ² = λ²_min + (λ²_max − λ²_min)(1 − w/w₀)²  if w = √det(JJᵀ) < w₀, else λ²_min, | `pentagon_qddot_commander._step_nominal` | — |
| (6.8) | 168 | J⁺ = Jᵀ(JJᵀ + λ²I₆)⁻¹  (coded as `solve(JJᵀ + λ²I, J)ᵀ`), | `pentagon_qddot_commander._step_nominal` | — |
| (6.9) | 169 | q̈_null = N(−k_N(q − q_home) − d_N q̇),  N = I₇ − J⁺J, | `pentagon_qddot_commander._step_nominal` | — |
| (6.11) | 170 | q̈ = clip(q̈_task + q̈_null, ±q̈_max). | `manip_thr` | — |
| (6.10) | 177 | F-3-14: N is not an exact projector.** With the damped J⁺, | `pentagon_qddot_commander._step_nominal` | — |
## [07_cbf_obstacle_barrier.md](07_cbf_obstacle_barrier.md)

| Eq. | Line | Statement (from the chapter) | Code (chapter pointers) | Also cited in |
|---|---:|---|---|---|
| (7.1) | 88 | d = d_ctr − r − m(o),  with r = 0.05 m (sphere radius) and m(o) the mask-dilation margin | `distance_engine.DistanceEngine.compute` | — |
| (7.3) | 93 | h = d − d_safe,  safe set C = {h ≥ 0}. | `cbf_state_rows.ConstraintBuilder.build`, `build`, `build_row_rhs` | — |
| (7.4) | 107 | ḣ = aᵀq̇ − v_n,  a := J_pᵀn̂ ∈ ℝ⁷ (the row of A),  v_n := n̂ᵀv_o. | `_obstacle_speed`, `build`, `build_row_rhs` | — |
| (7.5) | 137 | ḧ = aᵀq̈ + n̂ᵀJ̇_p q̇ + κ − n̂ᵀa_o,  κ := ṅ̂ᵀv_rel = ‖(I − n̂n̂ᵀ)v_rel‖² / ρ_c ≥ 0. | `jdot_qdot` | 16 |
| (7.6) | 157 | ḧ + k₁ḣ + k₀h ≥ 0,  s² + k₁s + k₀ = (s + α₁)(s + α₂). | — | 08 |
| (7.7) | 165 | aᵀq̈ ≥ −k₁(aᵀq̇ − v_n) − k₀h̄ − c − κ. | `build_row_rhs`, `build_osqp_bounds`, `build_qp_objective` | 08, 16 |
| (7.8) | 180 | Why real roots matter, and how much complex roots cost.** Write the closed loop of a binding | — | 08, 16 |
| (7.9) | 191 | Where the row starts to push.** For a static obstacle (v_n = 0) and ignoring c, the bound | `d_safe`, `_h_smooth` | — |
| (7.11) | 200 | Rest speed inside the barrier.** With h̄ < 0 and a binding row, the k₀ term keeps | `h_pen_row`, `minimal_l1`, `d_safe`, `_h_smooth` | 08, 16 |
| (7.14) | 277 | Tracker estimate** (`_obstacle_speed_tracked`). It is 0 unless the track is old enough, | `_obstacle_speed_tracked`, `obstacle_velocity_min_frames` | 16 |
| (7.13) | 287 | Residual estimate** (`_obstacle_speed`). Using (7.4) backwards, | `_obstacle_speed` | — |
| (7.12) | 318 | h̄ = h − h_brake − h_unc − h_lat − h_rng − h_std − h_ev. | `cbf_state_rows.ConstraintBuilder.build`, `cbf_state_rows.evidence_margin` | 16 |
| (7.15) | 355 | v_cap = min(1.1, 0.05 + 1.0·v_o + 2.0·max(−h̄, 0)·min(v_o/0.3, 1) + 1.1·clip(h̄/0.15, 0, 1)), | `retreat_cap_speed`, `retreat_cap_rhs` | 08 |
| (7.10) | 380 | h_qp = k₁(A q̇_cbf − v_obs) + k₀·max(h̄, h_pen) + c [+ b_ff] [+ κ]  (barrier rows), | `build_row_rhs`, `build_osqp_bounds` | 08, 16 |
## [08_constraints_and_qp.md](08_constraints_and_qp.md)

| Eq. | Line | Statement (from the chapter) | Code (chapter pointers) | Also cited in |
|---|---:|---|---|---|
| (8.1) | 71 | min_{q̈, s}  ½(q̈ − q̈_nom)ᵀW(q̈ − q̈_nom) + penalty(s) | `row_l1` | — |
| (8.2) | 76 | x = [q̈ (7) ; s_obs, s_sc, s_qlim, s_sing, s_cap, s_spd, s_ssb (7 family slacks) ; | `obstacle_row_slack_slots`, `row_l1` | 16 |
| (8.3) | 88 | ½xᵀPx + qᵀx = ½‖q̈ − q̈_nom‖² + Σ_g ½ρ_g s_g² + Σ_k (½ε σ_k² + ρ_l1 σ_k) − ½‖q̈_nom‖². | `cbf_qp_assembly.build_qp_objective` | — |
| (8.4) | 109 | −aᵢᵀq̈ − mᵢ s_{col(i)} ≤ h_qp,i   ⇔   aᵢᵀq̈ + mᵢ s_{col(i)} ≥ −h_qp,i. | `build_osqp_bounds` | — |
| (8.9) | 196 | h = q_max − m − q (a = −e_j) or h = q − m − q_min (a = +e_j), c = 0, | `joint_limit_row_horizon`, `joint_limit_row_margin` | — |
| (8.10) | 206 | h = ‖p_a − p_b‖ − r_a − r_b − m_sc,  n̂ = (p_a − p_b)/‖·‖,  a = n̂ᵀ(J_a − J_b), | `self_collision.segment_segment_closest`, `cbf_state_rows.SelfCollisionRowBuilder.build` | 15, 16 |
| (8.11) | 215 | h = σ − 0.05,  a = ∇_q σ (forward differences, ε = 1e-4),  c = min(ȧᵀq̇, 0), | `cbf_singularity.SingularityRowBuilder` | 15 |
| (8.15) | 225 | ŝᵀJq̈ − s ≤ (v_cap − ŝᵀJq̇)/T, | `ShadowRowBuilder`, `ssm_speed_cap` | — |
| (8.7) | 161 | q̄̈ ← min(q̄̈, (v̄ − q̇)/T₊),  q̲̈ ← max(q̲̈, (v̲ − q̇)/T₋), | `ssm_speed_cap`, `fr3_velocity_envelope`, `state_box_relax_s`, `hard_accel_box` | 09 |
| (8.5) | 157 | v̄ = min(0.7·q̇_max, √(2 a_auth h_up), 0.7·ū_env(q)), | `hard_accel_box` | 09 |
| (8.8) | 181 | q̈ ∈ [q̈_prev − δ_eff, q̈_prev + δ_eff],  δ_eff = `max_qddot_delta`·min(Δt_tick/Δt_nom, 10), | `workspace_face_rows`, `max_qddot_delta`, `apply_slew_limit`, `_qp_tick`, `utils/rate_scaling.py` | — |
| (8.6) | 159 | ū_env(q) = min(L, max(0, −v_off + √(s·(q_ref,up − q)))),  l̲_env symmetric | `fr3_velocity_envelope` | 09 |
| (8.12) | 260 | λ = (b − aᵀq̈_n)/(‖a‖² + 1/ρ),  s = (b − aᵀq̈_n)/(1 + ρ‖a‖²) > 0. | `obstacle_slack_max`, `ssb_slack_max` | — |
| (8.13) | 291 | (1 + ρ_obs‖a‖²) s_obs − ρ_cap‖a‖² s_cap = b_obs − aᵀq̈_n,   s_obs + s_cap = b_obs − b_cap, | `check_slack_and_priority.py`, `obstacle_slack_max`, `ssb_slack_max` | — |
| (8.14) | 310 | λ_hard = (b − aᵀq̈_n)/‖a‖²  (unconstrained case), | `obstacle_slack_max`, `ssb_slack_max`, `max_qddot_delta` | 16 |
| (8.16) | 385 | q̈ = clip(−k_brake·q̇, lo, hi),  (lo, hi) = state box ∩ slew window (`_brake_box`, | `_brake_box`, `brake_slew_limited`, `check_box_and_brake.py`, `max_qddot_delta`, `position_brake_eta` | 16 |
## [09_torque_realization.md](09_torque_realization.md)

| Eq. | Line | Statement (from the chapter) | Code (chapter pointers) | Also cited in |
|---|---:|---|---|---|
| (9.1) | 35 | M(q) q̈ + C(q, q̇) q̇ + g(q) = τ + τ_ext, | — | — |
| (9.2) | 50 | τ* = M(q) q̈* + C(q, q̇) q̇ + g(q) = RNEA(q, q̇, q̈*). | `qddot_to_torque._compute_tau` | 16 |
| (9.3) | 55 | τ_cmd = M(q) q̈* + C(q, q̇) q̇ = RNEA(q, q̇, q̈*) − g(q). | `qddot_to_torque._compute_tau`, `cbf_hard_limits.FR3_MAX_JOINT_ACCEL` | — |
| (9.4) | 84 | τ = M_arm(q) q̈_safe + (nle − g)_arm(q, q̇) = M q̈_safe + C(q, q̇) q̇. | `accept_iterate`, `qddot_to_torque`, `iso_enabled` | 16 |
| (9.5) | 136 | q̇_des ← q̇_des + q̈_safe·Δt,  q̇_des ← clip(q̇_des, q̇_floor(q), q̇_ceil(q)), | `rt_torque_controller.cpp::RtTorqueController::update` | — |
| (9.6) | 145 | τᵢ = φᵢ(τ_ff,i)·τ_ff,i + K_p,i pᵢ + K_d,i eᵢ, | `qddot_to_torque`, `RtTorqueController`, `CBFTorqueController` | — |
## [10_alternative_controllers.md](10_alternative_controllers.md)

| Eq. | Line | Statement (from the chapter) | Code (chapter pointers) | Also cited in |
|---|---:|---|---|---|
| (10.1) | 43 | velocity:      ḣ + γh ≥ 0,                       ḣ = n̂ᵀJ_p q̇               (decision q̇) | `cbf_velocity_filter._control_loop`, `cbf_OSCBF_filter._build_obstacle_cbf` | — |
| (10.9) | 193 | q̈ = M⁻¹(τ − Cq̇)   (the τ published excludes g; something downstream must add it). | — | — |
| (10.2) | 82 | h = d − d_safe (surface gap),  ḣ = n̂ᵀJ_p q̇  (static obstacle: no v_obs term), | `ld.direction`, `closest_point_robot`, `_compute_point_jacobian` | — |
| (10.3) | 88 | min_{q̇, s} ½‖q̇ − q̇_nom‖² + ½ρs² | `cbf_velocity_filter._control_loop` | — |
| (10.4) | 97 | γ = clip(conf, 0.5, 1)·(2 + 10·e^{−18·max(d − d_safe, 0)})  (`cbf_utils.select_gamma`), | `cbf_utils.select_gamma`, `select_gamma` | — |
| (10.5) | 105 | q̇_cmd = q̇_prev + clip(q̇_QP − q̇_prev, ±q̈_max·Δt), | `cbf_velocity_filter._control_loop` | — |
| (10.6) | 128 | q̇_cmd(t) = linear interpolation from the previous sample to the newest one over the | `max_accel`, `qdot_max` | — |
| (10.7) | 162 | ẍ = [K_p(p_d − p) + K_d(v_d − J_v q̇) ;  K_pR e_R − K_dR J_ω q̇], | — | — |
| (10.8) | 180 | min ½‖w_j M⁻¹Nᵀ(τ − τ_n)‖² + ½‖w_o J M⁻¹(τ − τ_n)‖² + ½ρs²  (w_j = w_o = 1, ρ = 1000). | — | — |
| (10.10) | 197 | Aτ ≥ b ⇔ q̈ᵢ ≥ −(α₁+α₂)q̇ᵢ − α₁α₂(qᵢ − q_min,i), and the mirrored upper row. | `check_oscbf.py` | — |
| (10.11) | 204 | h = ‖p_r − p_h‖ − 0.15,  ḣ = n̂ᵀJ_p q̇,  c = n̂ᵀJ̇_p q̇ + ṅ̂ᵀJ_p q̇, | `cbf_OSCBF_filter._build_obstacle_cbf` | — |
| (10.12) | 353 | ḧ ≥ −(α₁+α₂)ḣ − α₁α₂h | `cbf_OSCBF_filter._build_obstacle_cbf` | — |
## [11_iso_safety_layer.md](11_iso_safety_layer.md)

| Eq. | Line | Statement (from the chapter) | Code (chapter pointers) | Also cited in |
|---|---:|---|---|---|
| (11.1) | 63 | S_p = S_h + S_r + S_s + C + Z_d + Z_r. | `iso_ssm.protective_separation` | — |
| (11.2) | 68 | S_h = v_h (T_r + T_s),  S_r = v_r T_r,  S_s = v_r²/(2a_s),  T_s = v_r/a_s. | `iso_ssm.protective_separation`, `iso_ssm.stopping_distance` | — |
| (11.3) | 72 | S_r + S_s = v_r T_r + v_r²/(2a_s)   (`iso_ssm.stopping_distance`), | `iso_ssm.stopping_distance` | — |
| (11.4) | 88 | v_cap(d, v_h) = √(b² + 2a_s(d − K − v_h T_r)) − b,  clipped to [0, v_max], | `diag_ssm_sp`, `check_iso.py` | — |
| (11.5) | 97 | K = 0.85 + 0.06 + 0.01 = 0.92 m (shipped). | `iso_ssm.protective_separation` | — |
| (11.6) | 104 | v_PFL = F_max / √(μ k). | `iso_pfl_speed.py`, `iso_constants_measure.py`, `reaction`, `latency_budget.py` | — |
| (11.7) | 159 | t_stand = T_r + v_h/a_s = 2.1 s,  standoff_max = min(v_h·t_stand, 2.0 m) = 2.0 m, | `cbf_state_rows`, `enable_velocity_standoff`, `link_speed_max`, `iso_v_pfl`, `retreat_cap_max_speed` | — |
| (11.8) | 173 | ‖ṗ_i‖ ≤ min( v_cap(d_i, v_o,i), link_speed_cap(d_sc,min) ), | `link_speed_row` | — |
| (11.9) | 245 | v_closing,i = max(−n̂ᵀ J_p,i q̇, 0),  cap_i = v_cap(d_i, v_app,i) (with v_max = `link_speed_max`). | `link_speed_max` | — |
## [12_simulation.md](12_simulation.md)

| Eq. | Line | Statement (from the chapter) | Code (chapter pointers) | Also cited in |
|---|---:|---|---|---|
| (12.21) | 517 | δ = (d_safe − gap_min + 0.001 m) · unit(spawn − cp_nearest) | `fr3_control.yaml`, `enforce_start_clearance`, `motion`, `pentagon`, `scenario` | — |
| (12.20) | 200 | t_start = t_now + start_lead_s + start_offset_s,  start_lead_s = 0.25 s, start_offset_s from the seed. | `gz_plant`, `spawn_rt_controller`, `pin_filter`, `filter_cpu`, `Shutdown` | — |
| (12.19) | 214 | , not imposed. A slow run keeps the stack's timing in sim seconds, but the stack's | `reflex_trips`, `rate_clips`, `rtf`, `rtf_p10`, `qdd_projection.frac` | — |
| (12.15) | 268 | m = Σ m_i,  c = Σ m_i c_i / m,  I = Σ [I_i + m_i(‖c_i − c‖² 𝟙 − (c_i − c)(c_i − c)ᵀ)] | `_rigid_open_fingers`, `fr3_hand`, `_drop_finger_control`, `_set_initial_q`, `initial_value` | — |
| (12.16) | 274 | ΔM_jj − I_a,j = Σ_{k>j} I_a,k (1 + (ẑ_j·ẑ_k)²)/2 ∈ [Σ_{k>j} I_a,k / 2, Σ_{k>j} I_a,k] | `franka_simulation gz_robot_description._add_armature` | — |
| (12.1) | 312 | R_link = R_opt · R_LINK_FROM_OPTICAL^T,  t_link = t_opt | — | — |
| (12.2) | 314 | hfov = 2·atan(W / 2f_x) | — | — |
| (12.10) | 345 | σ_z(z) = z² σ_d / (f_px b),  b = 0.095 m, σ_d = 0.20 px | `sensor_range_uncertainty`, `wait_for_tf` | — |
| (12.11) | 703 | over CPs and obstacles at sample k, and Q_m the q̈_safe messages received at bag times τ_m: | `min_gap`, `min_h`, `contact` | — |
| (12.7) | 453 | t = max(0, t_sim − t_start) | `franka_simulation scenario_obstacles_system.cpp PreUpdate`, `simTime`, `PreUpdate` | — |
| (12.3) | 458 | static:  p = p0, v = 0;  linear: p = p0 + v0 t, v = v0 | `simexp.obstacle_laws.position`, `simexp.obstacle_laws.velocity` | — |
| (12.4) | 460 | ballistic:  p = p0 + v0 t + ½ g t²,  v = v0 + g t   (exact) | `simexp.obstacle_laws.position`, `simexp.obstacle_laws.velocity` | — |
| (12.5) | 462 | oscillate:  p = p0 + â A sin(ωt + φ),  v = â A ω cos(ωt + φ),  ω = 2π/T_p | `simexp.obstacle_laws.position`, `simexp.obstacle_laws.velocity` | — |
| (12.6) | 464 | wander:  p_i = p0_i + B_i sin(ω₀ r_i t + φ_i),  r = (1, φ_golden, 1 + √2),  ω₀ = 2π/T_p | `simexp.obstacle_laws._wander` | — |
| (12.8) | 466 | pursuit:  v_k = s · unit(x_target − p_k),  p_{k+1} = p_k + v_k · min(Δt, t) | `simexp.obstacle_laws.pursue`, `fr3_link7` | — |
| (12.9) | 492 | v0 = (x_fl − p0)/T_f − ½ g T_f   ⇒   p(T_f) = x_fl | — | — |
| (12.12) | 705 | min_h = min_{k∈W} g_k − d_safe | `min_gap`, `min_h`, `contact`, `neg_duration_s` | — |
| (12.13) | 707 | neg_duration_s = #{k∈W : g_k − d_safe < 0} · median_{k∈W}(t_{k+1} − t_k) | `contact`, `neg_duration_s`, `min_h_cp`, `t_min_h`, `task_final_err` | — |
| (12.14) | 709 | contact = ∃ k∈W : g_k < 0 | `min_h`, `contact`, `neg_duration_s`, `min_h_cp`, `t_min_h` | protocol |
| (12.17) | 711 | jerk_m = ‖Q_{m+1} − Q_m‖ / median(τ_{m+1} − τ_m),  jerk_p99 = P99(jerk) | `pentagon.p95_m`, `jerk_p99`, `qp_rate_hz`, `qp_rate_ok`, `condition.cbf_params.qp_rate_hz` | protocol |
| (12.18) | 713 | qp_rate_hz = #{/cbf_status messages with τ ∈ W} / (t_end − t_start),  ok ⇔ \|qp_rate_hz/target − 1\| ≤ 0.05 | `jerk_p99`, `qp_rate_hz`, `qp_rate_ok`, `condition.cbf_params.qp_rate_hz`, `slack_max_any` | — |
| (12.22) | 715 | frac = (q̈_real · q̈_cmd) / ‖q̈_cmd‖²,  q̈_real = box₁₀ₘₛ(d q̇/dt), counted when ‖q̈_cmd‖ ≥ 0.5 rad/s² | `rtf`, `rtf_p10`, `qdd_projection.frac`, `runner.digest_cell`, `lag/bias/rms` | protocol |
## [13_observability_and_tools.md](13_observability_and_tools.md)

| Eq. | Line | Statement (from the chapter) | Code (chapter pointers) | Also cited in |
|---|---:|---|---|---|
| (13.1) | 76 | q̈_k = α (q̇_k − q̇_{k−1})/(t_k − t_{k−1}) + (1 − α) q̈_{k−1},  α = `accel_lpf_alpha` = 0.35, | `accel_lpf_alpha`, `qddot_safe_topic`, `check_observability.py`, `test_experiment_logger_truth_off_identical`, `test_experiment_log_columns` | — |
| (13.2) | 85 | v_TCP = ‖J_v(q) q̇‖,  ω_TCP = ‖J_ω(q) q̇‖  at `tcp_link` = `fr3_link8` (`_cartesian`). | `tcp_link`, `fr3_link8`, `_cartesian` | — |
| (13.3) | 191 | f = q̈_realᵀ q̈_cmd / ‖q̈_cmd‖²,  judged only when ‖q̈_cmd‖ > 0.5 rad/s², | — | — |
| (13.4) | 299 | event → consumption:   wait ~ U[0, T_c],  E = T_c/2,  max = T_c; | — | — |

## Equations without a code pointer next to them

Each is either a derivation step, an assumption or a definition used by a neighbouring equation (the chapter section explains which), or a gap to close in a later documentation pass. None of them is a finding by itself.

- (4.3) — [04_math_foundations.md](04_math_foundations.md) line 157: J_v[:, j] = a_j × (o_f − o_j),  J_ω[:, j] = a_j  (and 0 for joints not upstream of f).
- (4.9) — [04_math_foundations.md](04_math_foundations.md) line 211: M(q) q̈ + C(q, q̇) q̇ + g(q) = τ + τ_ext,
- (4.12) — [04_math_foundations.md](04_math_foundations.md) line 237: log(R_d R_cᵀ) = −R_d log(R_dᵀ R_c),
- (5.4) — [05_perception_and_distance.md](05_perception_and_distance.md) line 96: robot**).
- (5.13) — [05_perception_and_distance.md](05_perception_and_distance.md) line 652: spread = ‖max_k (p_h,k − p_r,k) − min_k (p_h,k − p_r,k)‖.
- (7.6) — [07_cbf_obstacle_barrier.md](07_cbf_obstacle_barrier.md) line 157: ḧ + k₁ḣ + k₀h ≥ 0,  s² + k₁s + k₀ = (s + α₁)(s + α₂).
- (7.8) — [07_cbf_obstacle_barrier.md](07_cbf_obstacle_barrier.md) line 180: Why real roots matter, and how much complex roots cost.** Write the closed loop of a bindi
- (9.1) — [09_torque_realization.md](09_torque_realization.md) line 35: M(q) q̈ + C(q, q̇) q̇ + g(q) = τ + τ_ext,
- (10.9) — [10_alternative_controllers.md](10_alternative_controllers.md) line 193: q̈ = M⁻¹(τ − Cq̇)   (the τ published excludes g; something downstream must add it).
- (10.7) — [10_alternative_controllers.md](10_alternative_controllers.md) line 162: ẍ = [K_p(p_d − p) + K_d(v_d − J_v q̇) ;  K_pR e_R − K_dR J_ω q̇],
- (10.8) — [10_alternative_controllers.md](10_alternative_controllers.md) line 180: min ½‖w_j M⁻¹Nᵀ(τ − τ_n)‖² + ½‖w_o J M⁻¹(τ − τ_n)‖² + ½ρs²  (w_j = w_o = 1, ρ = 1000).
- (12.1) — [12_simulation.md](12_simulation.md) line 312: R_link = R_opt · R_LINK_FROM_OPTICAL^T,  t_link = t_opt
- (12.2) — [12_simulation.md](12_simulation.md) line 314: hfov = 2·atan(W / 2f_x)
- (12.9) — [12_simulation.md](12_simulation.md) line 492: v0 = (x_fl − p0)/T_f − ½ g T_f   ⇒   p(T_f) = x_fl
- (13.3) — [13_observability_and_tools.md](13_observability_and_tools.md) line 191: f = q̈_realᵀ q̈_cmd / ‖q̈_cmd‖²,  judged only when ‖q̈_cmd‖ > 0.5 rad/s²,
- (13.4) — [13_observability_and_tools.md](13_observability_and_tools.md) line 299: event → consumption:   wait ~ U[0, T_c],  E = T_c/2,  max = T_c;
