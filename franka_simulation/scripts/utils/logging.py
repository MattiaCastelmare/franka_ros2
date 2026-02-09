"""Centralized logging helpers.

These helpers keep node files short while preserving the exact log content.
"""

from __future__ import annotations

from typing import Any


def log_controller_config(*, logger: Any, params: Any, qp_available: bool, cbf_state: Any) -> None:
    """Log controller configuration at startup.

    Intentionally matches the previous `get_logger().info(...)` lines.
    """
    logger.info("📊 Parametri CARICATI (da file YAML o default):")
    logger.info(f"   influence_distance: {params.influence_distance}")
    logger.info(f"   safety_margin: {params.safety_margin}")
    logger.info(f"   k_null (nullspace_gain): {params.k_null}")
    logger.info(f"   k_tan (tangential_gain): {params.k_tan}")
    logger.info(f"   max_qdot (max_joint_velocity): {params.max_qdot}")
    logger.info(f"   d_aggr (aggressive_distance): {params.d_aggr}")
    logger.info(f"   k_aggr (aggressive_gain_scale): {params.k_aggr}")
    logger.info(
        "   capsule geometry: "
        f"radii={params.capsule_radii} | fractions={params.capsule_fractions}"
    )
    logger.info(
        "   box distance: "
        f"iters={params.box_projection_iters} | spread(enable={params.repulsion_spread_enable}, samples={params.repulsion_spread_samples}, half_len={params.repulsion_spread_half_length})"
    )
    logger.info(
        "   extra safety: "
        f"ground(enable={params.enable_ground}, z={params.ground_z}, d_infl={params.ground_infl}, d_safe={params.ground_safe}, k={params.k_ground}) | "
        f"self(enable={params.enable_self}, d_infl={params.self_infl}, d_safe={params.self_safe}, k={params.k_self}, skip_adj={params.self_skip_adjacent})"
    )

    logger.info(
        "   CBF-QP safety filter: "
        f"d_safe={params.cbf_d_safe}, d_buffer_in={params.cbf_d_buffer_in}, d_buffer_out={params.cbf_d_buffer_out}, "
        f"alpha={params.cbf_alpha} (risk-scaled [{params.cbf_alpha_min},{params.cbf_alpha_max}]), K={params.cbf_K}, use_qp={params.cbf_use_qp} (available={bool(qp_available)}), beta_lpf={params.cbf_beta_lpf}"
    )

    logger.info(
        "   Risk zones: "
        f"far={params.risk_d_far:.3f} mid={params.risk_d_mid:.3f} near={params.risk_d_near:.3f} "
        f"stop_in={params.stop_d_in:.3f} stop_out={params.stop_d_out:.3f} | "
        f"qp_damping=[{params.cbf_qp_damping_min},{params.cbf_qp_damping_max}] | "
        f"beta_lpf_far={params.cbf_beta_lpf_far} beta_lpf_near={params.cbf_beta_lpf_near}"
    )

    logger.info(
        "   Robustness: "
        f"distance_inflation={params.distance_inflation:.3f}m | "
        f"hazard_hold_time_s={params.hazard_hold_time_s:.3f}s | hazard_switch_delta_m={params.hazard_switch_delta_m:.3f}m"
    )

    # Note: cbf_state is passed in to allow future extensions (and matches requested signature).
    _ = cbf_state
