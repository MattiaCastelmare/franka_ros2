"""Diagnostics publishing helpers.

This module centralizes diagnostic publishers so nodes stay readable.
"""

from __future__ import annotations

from typing import Any, Optional

import numpy as np

from std_msgs.msg import Float64MultiArray, String


def publish_cbf_diagnostics(
    *,
    pubs: Any,
    cbf_state: Any,
    G: Optional[np.ndarray],
    m_active: int,
    active_best: Optional[dict],
    d_min_raw: float,
) -> None:
    """Publish min-distance, hazard string, and active constraint Jacobian row.

    NOTE: Behavior is intentionally kept identical to the previous version:
    - Always publish both raw and filtered min distance.
    - When stop gate is active OR there are no active constraints -> publish jac_zero.
    - Otherwise publish G[0, :] (the most critical ACTIVE constraint row).

    `pubs` must provide: min_dist_raw_pub, min_dist_pub, hazard_pub, jac_pub.
    """
    hazard_msg = String()

    pubs.min_dist_raw_pub.publish(Float64MultiArray(data=[float(d_min_raw)]))
    pubs.min_dist_pub.publish(Float64MultiArray(data=[float(getattr(cbf_state, "d_min_filt", 999.0))]))

    jac_zero = Float64MultiArray(data=[0.0] * 7)

    if bool(getattr(cbf_state, "stop_gate_active", False)):
        hazard_msg.data = "stop_gate"
        pubs.hazard_pub.publish(hazard_msg)
        pubs.jac_pub.publish(jac_zero)
        return

    if (active_best is None) or (int(m_active) <= 0) or (G is None):
        hazard_msg.data = "none"
        pubs.hazard_pub.publish(hazard_msg)
        pubs.jac_pub.publish(jac_zero)
        return

    hazard_msg.data = str(active_best.get("hazard", "none"))
    pubs.hazard_pub.publish(hazard_msg)
    pubs.jac_pub.publish(Float64MultiArray(data=np.array(G, dtype=float)[0, :].reshape(-1).tolist()))
