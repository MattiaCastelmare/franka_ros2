"""High-level control pipeline for velocity_control_blender.

This module contains the full control-loop logic, keeping the ROS node minimal.
"""

from __future__ import annotations

from typing import Optional
import time
import numpy as np

from . import velocity_blender_ros_helpers as vh
from .velocity_blender_state import BlenderRuntimeState
from .velocity_blender_core import (
    compute_d_dot_min_from_distance,
    project_multi_constraints_with_box,
    nearest_point_on_polyline,
    interpolate_at_s,
)


def step(
    *,
    rt: BlenderRuntimeState,
    params: object,
    now_wall: float,
    logger: object,
) -> Optional[np.ndarray]:
    """Run one control step.

    Returns a joint-velocity command (np.ndarray) or None.
    """
    n = int(getattr(params, "n_dof"))

    # TEST-REACTIVE: set True to force reactive mode even without trajectory.
    FORCE_REACTIVE_TEST = False  # TEST-REACTIVE
    reactive_enable = bool(getattr(params, "reactive_enable")) or bool(FORCE_REACTIVE_TEST)

    # ===== PAUSE MODE =====
    handled, qdot, reset_smoothing = vh.handle_pause_mode(
        pause_enable=bool(getattr(params, "pause_enable")),
        paused=bool(rt.paused),
        n_dof=n,
    )
    if handled:
        if reset_smoothing:
            rt.qdot_prev = np.zeros(n)
        return np.array(qdot, dtype=float).reshape(n)

    # ===== MODALITÀ REACTIVE (B) =====
    handled, qdot, _reset_smoothing = vh.handle_no_trajectory_mode(
        active=bool(rt.active),
        trajectory_points_len=int(len(rt.trajectory_points)),
        hold_position_without_trajectory=bool(getattr(params, "hold_position_without_trajectory")) and (not reactive_enable),
        reactive_enable=bool(reactive_enable),
        qdot_avoid=np.array(rt.qdot_avoid, dtype=float).reshape(n),
        reactive_deadband=float(getattr(params, "reactive_deadband")),
        max_vel=float(getattr(params, "max_vel")),
        n_dof=n,
    )
    if handled:
        if bool(reactive_enable):
            beta_avoid = float(getattr(params, "avoidance_input_filter_beta", getattr(params, "velocity_filter_beta")))
            rt.qdot_avoid_filt = vh.update_lpf(
                np.array(rt.qdot_avoid_filt, dtype=float).reshape(n),
                np.array(rt.qdot_avoid, dtype=float).reshape(n),
                float(beta_avoid),
            )
            qdot_cmd = vh.update_lpf(
                np.array(rt.qdot_prev, dtype=float).reshape(n),
                np.array(rt.qdot_avoid_filt, dtype=float).reshape(n),
                float(getattr(params, "velocity_filter_beta")),
            )
            qdot_cmd = np.clip(
                np.array(qdot_cmd, dtype=float).reshape(n),
                -float(getattr(params, "max_vel")),
                float(getattr(params, "max_vel")),
            )

            # Multi-constraint projection (avoidance-only safety)
            try:
                rows = np.array(rt.constraints_rows, dtype=float).reshape(-1, n)
                ds = np.array(rt.constraints_d, dtype=float).reshape(-1)
                if (rows.size > 0) and (ds.size > 0):
                    influence_params = vh.make_influence_params_from_attrs(params)
                    d_dot_min_list = [
                        compute_d_dot_min_from_distance(float(di), influence_params)
                        for di in list(ds)
                    ]
                    qdot_cmd, _v_pre, _v_post, _infeas = project_multi_constraints_with_box(
                        qdot_des=np.array(qdot_cmd, dtype=float).reshape(n),
                        G=np.array(rows, dtype=float).reshape(-1, n),
                        b=np.array(d_dot_min_list, dtype=float).reshape(-1),
                        max_abs_vel=float(getattr(params, "max_vel")),
                        iters=int(getattr(params, "cbf_projection_iters")),
                        eps=float(getattr(params, "cbf_eps")),
                    )
                    qdot_cmd = np.clip(
                        np.array(qdot_cmd, dtype=float).reshape(n),
                        -float(getattr(params, "max_vel")),
                        float(getattr(params, "max_vel")),
                    )
            except Exception:
                pass

            rt.qdot_prev = np.array(qdot_cmd, dtype=float).reshape(n)

            # TEST-REACTIVE: throttled log (1 Hz)
            try:
                now = float(time.time())
                if (now - float(rt.reactive_log_wall)) >= 1.0:
                    rt.reactive_log_wall = now
                    qdot_avoid = np.array(rt.qdot_avoid, dtype=float).reshape(n)
                    av = float(np.linalg.norm(qdot_avoid))
                    bv = float(np.linalg.norm(qdot_cmd))
                    d_closest = float(rt.closest_d)
                    haz = str(rt.closest_hazard or "none")
                    traj_len = int(len(rt.trajectory_points))
                    logger.info(
                        "[TEST-REACTIVE] "
                        f"d_closest={d_closest:.3f} haz='{haz}' "
                        f"|qdot_avoid|={av:.4f} |qdot_cmd|={bv:.4f} max_vel={float(getattr(params, 'max_vel')):.3f} "
                        f"active={bool(rt.active)} traj_len={traj_len}"
                    )
                    if (av <= 1e-4) and (d_closest < float(getattr(params, "d_infl"))):
                        logger.warn(
                            "[TEST-REACTIVE] avoidance ~0 while within d_infl; check /obstacle_scene updates"
                        )
            except Exception:
                pass
            return np.array(qdot_cmd, dtype=float).reshape(n)

        return np.array(qdot, dtype=float).reshape(n)

    # ------------------------------------------------------------------
    # EMERGENCY OVERRIDE (hard safety) + safety signal
    # ------------------------------------------------------------------
    now = float(now_wall)

    (
        safety,
        staging_clip,
        d_raw_closest,
        d_eff_for_weights,
        d_eff_for_stop,
        j_row_raw,
        j_row_use,
        j_norm_use,
        rt.closest_j_row_filt,
        rt.closest_j_row_filt_init,
    ) = vh.compute_safety_and_jrow(
        n_dof=int(n),
        closest_d=float(rt.closest_d),
        closest_j_row=np.array(rt.closest_j_row, dtype=float).reshape(-1),
        closest_hazard=str(rt.closest_hazard or "none"),
        distance_inflation=float(getattr(params, "distance_inflation")),
        risk_d_far=float(getattr(params, "risk_d_far")),
        risk_d_mid=float(getattr(params, "risk_d_mid")),
        risk_d_near=float(getattr(params, "risk_d_near")),
        stop_distance=float(getattr(params, "stop_distance")),
        avoidance_disabled=bool(getattr(params, "avoidance_disabled")),
        closest_j_row_filt=np.array(rt.closest_j_row_filt, dtype=float).reshape(n),
        closest_j_row_filt_init=bool(rt.closest_j_row_filt_init),
    )

    # ------------------------------------------------------------------
    # PENETRATION EMERGENCY (signed distance < 0)
    # ------------------------------------------------------------------
    handled_pen, qdot_pen = vh.handle_penetration_emergency(
        penetration_emergency_enable=bool(getattr(params, "penetration_emergency_enable")),
        d_raw_closest=float(d_raw_closest),
        j_norm_use=float(j_norm_use),
        hazard=str(safety.hazard),
        penetration_emergency_d_dot=float(getattr(params, "penetration_emergency_d_dot")),
        penetration_emergency_depth_ref=float(getattr(params, "penetration_emergency_depth_ref")),
        penetration_emergency_max_vel_fraction=float(getattr(params, "penetration_emergency_max_vel_fraction")),
        max_vel=float(getattr(params, "max_vel")),
        j_row_use=np.array(j_row_use, dtype=float).reshape(-1),
    )
    if bool(handled_pen) and (qdot_pen is not None):
        rt.qdot_prev = np.zeros(n)
        return np.array(qdot_pen, dtype=float).reshape(n)

    # ------------------------------------------------------------------
    # STOP gate handler
    # ------------------------------------------------------------------
    stop_state = vh.StopGateState(
        stop_active=bool(rt.stop_active),
        stop_enter_wall=rt.stop_enter_wall,
        stop_phase=str(rt.stop_phase),
        stop_warn_wall=float(rt.stop_warn_wall),
        stop_d_dot_last=float(rt.stop_d_dot_last),
        stop_log_wall=float(rt.stop_log_wall),
    )
    handled_stop, qdot_stop, qdot_prev_new, stop_state = vh.handle_stop_gate(
        now_wall=float(now),
        n_dof=int(n),
        stop_distance=float(getattr(params, "stop_distance")),
        stop_release_distance=float(getattr(params, "stop_release_distance")),
        d_eff_for_stop=float(d_eff_for_stop),
        avoidance_disabled=bool(getattr(params, "avoidance_disabled")),
        hazard=str(safety.hazard),
        j_row_use=np.array(j_row_use, dtype=float).reshape(-1),
        j_norm_use=float(j_norm_use),
        emergency_d_dot=float(getattr(params, "emergency_d_dot")),
        emergency_max_vel_fraction=float(getattr(params, "emergency_max_vel_fraction")),
        max_vel=float(getattr(params, "max_vel")),
        velocity_filter_beta=float(getattr(params, "velocity_filter_beta")),
        velocity_filter_beta_near=float(getattr(params, "velocity_filter_beta_near")),
        d_eff_for_weights=float(d_eff_for_weights),
        d_infl=float(getattr(params, "d_infl")),
        cbf_projection_iters=int(getattr(params, "cbf_projection_iters")),
        cbf_eps=float(getattr(params, "cbf_eps")),
        normal_correction_max=float(getattr(params, "normal_correction_max")),
        qdot_prev=np.array(rt.qdot_prev, dtype=float).reshape(n),
        state=stop_state,
        logger=logger,
        d_raw_closest=float(d_raw_closest),
    )
    rt.stop_active = bool(stop_state.stop_active)
    rt.stop_enter_wall = stop_state.stop_enter_wall
    rt.stop_phase = str(stop_state.stop_phase)
    rt.stop_warn_wall = float(stop_state.stop_warn_wall)
    rt.stop_d_dot_last = float(stop_state.stop_d_dot_last)
    rt.stop_log_wall = float(stop_state.stop_log_wall)
    rt.qdot_prev = np.array(qdot_prev_new, dtype=float).reshape(n)

    if bool(handled_stop) and (qdot_stop is not None):
        return np.array(qdot_stop, dtype=float).reshape(n)

    if not bool(getattr(params, "avoidance_disabled")):
        handled, qdot_em, rt.er_state, reset_smoothing, emergency_now = vh.handle_emergency_override(
            now_wall=float(now),
            d_eff=float(d_raw_closest),
            j_row=np.array(j_row_raw, dtype=float).reshape(-1),
            hazard=str(safety.hazard),
            j_norm=float(np.linalg.norm(j_row_raw)),
            max_vel=float(getattr(params, "max_vel")),
            state=rt.er_state,
            params=vh.make_emergency_params_from_attrs(params),
        )
        if reset_smoothing:
            rt.qdot_prev = np.zeros(n)
        if handled and (qdot_em is not None):
            rt.diag.update_edge_counters(
                emergency_now=bool(emergency_now),
                stop_gate_now=bool(rt.diag.prev_stop_gate),
            )
            return np.array(qdot_em, dtype=float).reshape(n)

        rt.diag.update_edge_counters(
            emergency_now=bool(rt.diag.prev_emergency),
            stop_gate_now=bool(getattr(staging_clip, "stop_gate", False)),
        )
    else:
        rt.diag.update_edge_counters(
            emergency_now=False,
            stop_gate_now=False,
        )

    # ------------------------------------------------------------------
    # PATH FOLLOWING WITH REJOIN
    # ------------------------------------------------------------------
    pts = rt.trajectory_points
    last_idx = max(0, int(len(pts)) - 1)

    rt.progress, rt.current_index, q_target = vh.update_progress_and_select_target(
        q=np.array(rt.q, dtype=float).reshape(n),
        pts=pts,
        s_cum=rt.traj_s,
        last_idx=int(last_idx),
        current_index=int(rt.current_index),
        progress=rt.progress,
        rejoin_enable=bool(getattr(params, "rejoin_enable")),
        rejoin_search_ahead_points=int(getattr(params, "rejoin_search_ahead_points")),
        rejoin_lookahead_distance_rad=float(getattr(params, "rejoin_lookahead_distance_rad")),
        rejoin_lookahead_points=int(getattr(params, "rejoin_lookahead_points")),
        nearest_point_fn=nearest_point_on_polyline,
        interpolate_fn=interpolate_at_s,
    )

    error = q_target - rt.q
    error_norm = float(np.linalg.norm(error))

    # ------------------------------------------------------------------
    # Progress-based stall detection
    # ------------------------------------------------------------------
    try:
        prog_s = float(getattr(rt.progress, "progress_s", 0.0))
        stall_state = vh.StallProgressState(
            last_progress_s=rt.stall_prog_s_last,
            last_progress_wall=rt.stall_prog_wall_last,
            stalled=bool(rt.stall_prog_flag),
        )
        stall_state = vh.update_stall_detection(
            now_wall=float(now),
            progress_s=float(prog_s),
            state=stall_state,
        )
        rt.stall_prog_s_last = stall_state.last_progress_s
        rt.stall_prog_wall_last = stall_state.last_progress_wall
        rt.stall_prog_flag = bool(stall_state.stalled)
    except Exception:
        rt.stall_prog_flag = False

    # Completion
    q_final = pts[last_idx]
    final_err, complete = vh.check_trajectory_completion(
        q=rt.q,
        q_final=q_final,
        final_threshold=float(getattr(params, "final_threshold")),
    )
    if bool(complete):
        logger.info(f"✅ Traiettoria completata! Errore finale: {final_err:.4f} rad")
        rt.active = False
        return np.zeros(n, dtype=float)

    threshold = float(getattr(params, "waypoint_threshold"))

    # ===== 1) Tracking =====
    qdot_tracking = vh.compute_tracking_command(
        kp=float(getattr(params, "kp")),
        q_target=q_target,
        q=np.array(rt.q, dtype=float).reshape(n),
    )

    v_n = np.zeros(n)
    v_t = np.array(qdot_tracking, dtype=float).reshape(n)
    d_dot_track = 0.0
    try:
        if float(j_norm_use) > 1e-6:
            n_dir = (np.array(j_row_use, dtype=float).reshape(n)) / float(j_norm_use)
            d_dot_track = float(n_dir @ np.array(qdot_tracking, dtype=float).reshape(n))
            v_n = float(d_dot_track) * n_dir
            v_t = np.array(qdot_tracking, dtype=float).reshape(n) - np.array(v_n, dtype=float).reshape(n)
    except Exception:
        v_n = np.zeros(n)
        v_t = np.array(qdot_tracking, dtype=float).reshape(n)
        d_dot_track = 0.0

    v_n_norm = float(np.linalg.norm(v_n))
    v_t_norm = float(np.linalg.norm(v_t))

    # ===== 2) Avoidance velocity =====
    b = vh.compute_avoidance_input_beta(
        beta_far=float(getattr(params, "avoidance_input_filter_beta")),
        beta_near=float(getattr(params, "avoidance_input_filter_beta_near")),
        w=float(getattr(staging_clip, "w_total", 0.0)),
    )
    rt.qdot_avoid_filt = vh.update_lpf(rt.qdot_avoid_filt, rt.qdot_avoid, b)

    influence_params = vh.make_influence_params_from_attrs(params)
    qdot_des, dbg = vh.blend_tracking_and_avoidance(
        now_wall=float(now),
        d_raw=float(safety.d_raw),
        d_eff=float(d_eff_for_weights),
        d_infl=float(getattr(params, "d_infl")),
        j_row=np.array(j_row_use, dtype=float).reshape(-1),
        j_norm=float(j_norm_use),
        staging=staging_clip,
        qdot_tracking=np.array(qdot_tracking, dtype=float).reshape(n),
        qdot_avoid_filt=np.array(rt.qdot_avoid_filt, dtype=float).reshape(n),
        error_norm=float(error_norm),
        threshold=float(threshold),
        recovery_until_wall=float(rt.er_state.recovery_until_wall),
        influence_params=influence_params,
        stall_detected=bool(rt.stall_prog_flag),
        idx=int(rt.current_index),
        n_points=int(len(pts)),
    )

    # ANTI-CANCELLATION FALLBACK
    try:
        if (
            (str(safety.hazard) != "none")
            and (float(d_eff_for_weights) < float(getattr(params, "d_infl")))
            and (float(error_norm) > float(max(threshold, float(getattr(params, "tangent_escape_err_min")))))
            and (float(j_norm_use) > 1e-6)
        ):
            cmd_pre = np.array(qdot_des, dtype=float).reshape(n)
            cmd_pre_norm = float(np.linalg.norm(cmd_pre))

            t_vec = np.array(qdot_tracking, dtype=float).reshape(n)
            a_vec = np.array(rt.qdot_avoid_filt, dtype=float).reshape(n)
            t_norm = float(np.linalg.norm(t_vec))
            a_norm = float(np.linalg.norm(a_vec))
            eps = 1e-9

            cos_ta = float("nan")
            if (t_norm > eps) and (a_norm > eps):
                cos_ta = float(np.dot(t_vec, a_vec) / (t_norm * a_norm))

            cmd_small = float(max(0.03, 1.5 * float(getattr(params, "tangent_tracking_cmd_small"))))
            if (
                (cmd_pre_norm < cmd_small)
                and (t_norm > 3.0 * cmd_small)
                and (a_norm > 3.0 * cmd_small)
                and (np.isfinite(cos_ta) and (cos_ta < -0.35))
            ):
                n_dir = (np.array(j_row_use, dtype=float).reshape(n)) / float(j_norm_use)
                t_n = float(n_dir @ t_vec)
                a_n = float(n_dir @ a_vec)
                t_tan = t_vec - float(t_n) * n_dir
                a_norm_vec = float(a_n) * n_dir
                a_tan = a_vec - a_norm_vec

                cmd_rebuilt = np.array(t_tan, dtype=float).reshape(n) + np.array(a_norm_vec, dtype=float).reshape(n)

                try:
                    tan_ref = np.array(t_tan, dtype=float).reshape(n)
                    if float(np.linalg.norm(tan_ref)) < 1e-6:
                        tan_ref = np.array(cmd_rebuilt, dtype=float).reshape(n) - float((n_dir @ cmd_rebuilt)) * n_dir
                    if float(np.linalg.norm(tan_ref)) > 1e-6:
                        if float(np.dot(tan_ref, a_tan)) > 0.0:
                            cmd_rebuilt = cmd_rebuilt + float(getattr(params, "avoidance_tangent_weight")) * np.array(a_tan, dtype=float).reshape(n)
                except Exception:
                    pass

                qdot_des = np.array(cmd_rebuilt, dtype=float).reshape(n)

                try:
                    if isinstance(dbg, dict):
                        dbg["anti_cancel"] = True
                        dbg["anti_cancel_cos_track_avoid"] = float(cos_ta)
                        dbg["anti_cancel_cmd_pre_norm"] = float(cmd_pre_norm)
                        dbg["anti_cancel_cmd_post_norm"] = float(np.linalg.norm(qdot_des))
                        dbg["anti_cancel_track_tan_norm"] = float(np.linalg.norm(t_tan))
                        dbg["anti_cancel_avoid_norm_norm"] = float(np.linalg.norm(a_norm_vec))
                except Exception:
                    pass
            else:
                try:
                    if isinstance(dbg, dict):
                        dbg["anti_cancel"] = False
                        if np.isfinite(cos_ta):
                            dbg["anti_cancel_cos_track_avoid"] = float(cos_ta)
                except Exception:
                    pass
    except Exception:
        pass

    # TRACKING TANGENTIAL ESCAPE
    try:
        if (
            bool(getattr(params, "tangent_tracking_escape_enable"))
            and (str(safety.hazard) != "none")
            and (float(d_eff_for_weights) < float(getattr(params, "d_infl")))
            and (float(error_norm) > float(max(threshold, float(getattr(params, "tangent_escape_err_min")))))
        ):
            cmd_norm_pre = float(np.linalg.norm(np.array(qdot_des, dtype=float).reshape(n)))
            cmd_small = float(max(0.0, float(getattr(params, "tangent_tracking_cmd_small"))))
            vt_min = float(max(0.0, float(getattr(params, "tangent_tracking_vt_min"))))

            if (cmd_norm_pre < cmd_small) and (float(d_dot_track) < 0.0) and (float(v_t_norm) > vt_min):
                denom = float(max(1e-6, float(getattr(params, "d_infl")) - float(getattr(params, "stop_distance"))))
                x = float(np.clip((float(getattr(params, "d_infl")) - float(d_eff_for_weights)) / denom, 0.0, 1.0))
                smooth = float(3.0 * x * x - 2.0 * x * x * x)
                alpha_t = float(np.clip(float(getattr(params, "tangent_tracking_alpha_max")) * smooth, 0.0, float(getattr(params, "tangent_tracking_alpha_max"))))

                vt_max = float(max(0.0, float(getattr(params, "tangent_tracking_vt_max"))))
                vt_used = np.array(v_t, dtype=float).reshape(n)
                if float(v_t_norm) > max(1e-9, vt_max):
                    vt_used *= float(vt_max) / (float(v_t_norm) + 1e-9)

                qdot_des = np.array(qdot_des, dtype=float).reshape(n) + float(alpha_t) * vt_used

                try:
                    if isinstance(dbg, dict):
                        dbg["track_vn_norm"] = float(v_n_norm)
                        dbg["track_vt_norm"] = float(v_t_norm)
                        dbg["track_d_dot"] = float(d_dot_track)
                        dbg["tangent_track_escape"] = True
                        dbg["tangent_track_alpha"] = float(alpha_t)
                        dbg["tangent_track_vt_used_norm"] = float(np.linalg.norm(vt_used))
                except Exception:
                    pass
            else:
                try:
                    if isinstance(dbg, dict):
                        dbg["track_vn_norm"] = float(v_n_norm)
                        dbg["track_vt_norm"] = float(v_t_norm)
                        dbg["track_d_dot"] = float(d_dot_track)
                        dbg["tangent_track_escape"] = False
                except Exception:
                    pass
    except Exception:
        pass

    # ------------------------------------------------------------------
    # FILTERS + LIMITS
    # ------------------------------------------------------------------
    multi_rows = np.array(rt.constraints_rows, dtype=float).reshape(-1, n)
    multi_ds = np.array(rt.constraints_d, dtype=float).reshape(-1)
    if (multi_rows.size > 0) and (multi_ds.size > 0):
        multi_d_dot_min = np.array(
            [compute_d_dot_min_from_distance(float(di), influence_params) for di in list(multi_ds)],
            dtype=float,
        ).reshape(-1)
    else:
        multi_d_dot_min = None

    qdot, qdot_prev_new, dbg = vh.apply_final_filters_and_limits(
        qdot_des=np.array(qdot_des, dtype=float).reshape(n),
        qdot_prev=np.array(rt.qdot_prev, dtype=float).reshape(n),
        velocity_filter_beta=float(getattr(params, "velocity_filter_beta")),
        velocity_filter_beta_near=float(getattr(params, "velocity_filter_beta_near")),
        max_vel=float(getattr(params, "max_vel")),
        d_eff=float(d_eff_for_weights),
        d_infl=float(getattr(params, "d_infl")),
        j_row=np.array(j_row_use, dtype=float).reshape(-1),
        j_norm=float(j_norm_use),
        cbf_projection_iters=int(getattr(params, "cbf_projection_iters")),
        cbf_eps=float(getattr(params, "cbf_eps")),
        normal_correction_max=float(getattr(params, "normal_correction_max")),
        qdot_tracking_hint=np.array(qdot_tracking, dtype=float).reshape(n),
        multi_j_rows=multi_rows if (multi_rows.size > 0) else None,
        multi_d_dot_min=multi_d_dot_min,
        dbg=dbg,
    )
    rt.qdot_prev = np.array(qdot_prev_new, dtype=float).reshape(n)

    rt.diag.count_infeasible(dbg)
    rt.diag.maybe_log_stall(
        logger,
        diag_enable=bool(getattr(params, "diag_enable")),
        now_wall=float(time.time()),
        diag_period_s=float(getattr(params, "diag_period_s")),
        dbg=dbg,
        cmd_norm=float(np.linalg.norm(qdot)),
        d_infl=float(getattr(params, "d_infl")),
        diag_cmd_norm_eps=float(getattr(params, "diag_cmd_norm_eps")),
    )

    # Multi-constraint diagnostics (1 Hz)
    try:
        now = float(time.time())
        if (now - float(rt.multi_log_wall)) >= 1.0:
            rt.multi_log_wall = now
            n_active = int(multi_rows.shape[0]) if (multi_rows is not None) else 0
            if n_active > 0:
                d_list = list(multi_ds) if (multi_ds is not None) else []
                d_list = [float(x) for x in d_list]
                d_list.sort()
                d_min = d_list[0] if len(d_list) > 0 else 999.0
                d_2 = d_list[1] if len(d_list) > 1 else d_min
                av = float(np.linalg.norm(np.array(rt.qdot_avoid_filt, dtype=float).reshape(n)))
                bv = float(np.linalg.norm(np.array(qdot, dtype=float).reshape(n)))
                vpre = int(dbg.get("cbf_multi_violations_pre", 0))
                vpost = int(dbg.get("cbf_multi_violations_post", 0))
                infeas = int(dbg.get("cbf_multi_infeasible", 0))
                logger.info(
                    "[BLENDER-MULTI] "
                    f"N_active={n_active} d_min={d_min:.3f} d_2={d_2:.3f} "
                    f"|qdot_avoid|={av:.4f} |qdot_cmd|={bv:.4f} "
                    f"viol_pre={vpre} viol_post={vpost} infeas={infeas}"
                )
                if (av <= 1e-4) and (d_min < float(getattr(params, "d_infl"))):
                    logger.warn(
                        "[BLENDER-MULTI] active constraints but avoidance ~0; check controller output"
                    )
    except Exception:
        pass

    rt.diag.maybe_log_robust(
        logger,
        diag_enable=bool(getattr(params, "diag_enable")),
        now_wall=float(time.time()),
        d_raw=float(safety.d_raw),
        d_eff=float(d_eff_for_weights),
        inflation=float(safety.inflation),
        hazard=str(safety.hazard),
    )

    return np.array(qdot, dtype=float).reshape(n)
