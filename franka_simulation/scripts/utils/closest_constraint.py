"""Publish the coherent closest constraint pair for downstream blending.

This is extracted from the controller to keep the node high-level.

Behavior note
-------------
This publishing is intentionally independent from whether the CBF is active.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Optional, Sequence

import numpy as np

from std_msgs.msg import Float64MultiArray, String

from .avoidance_math import build_cbf_constraints


@dataclass
class ClosestConstraintHoldState:
    last_hazard: str = "none"
    last_d: float = 999.0
    last_j_row: Optional[np.ndarray] = None
    last_switch_wall: float = 0.0


def publish_closest_constraint(
    *,
    candidates: Sequence[dict],
    model: Any,
    data: Any,
    q: np.ndarray,
    pubs: Any,
    params: Any,
    d_min_default: float,
    hold_state: Optional[ClosestConstraintHoldState] = None,
    now_wall: Optional[float] = None,
) -> Optional[dict]:
    """Publish:

    - `/avoidance/closest_constraint`: Float64MultiArray [d_closest, j_row_0..j_row_6]
    - `/avoidance/closest_hazard`: String

    `pubs` must provide: closest_constraint_pub, closest_hazard_pub.
    `params` must provide `cbf_params` (CbfFilterParams).
    """
    try:
        candidates_list = list(candidates)
        if len(candidates_list) <= 0:
            pubs.closest_constraint_pub.publish(Float64MultiArray(data=[999.0] + [0.0] * 7))
            msg = String(); msg.data = "none"
            pubs.closest_hazard_pub.publish(msg)
            if hold_state is not None:
                hold_state.last_hazard = "none"
                hold_state.last_d = 999.0
                hold_state.last_j_row = None
            return None

        cp = params.cbf_params
        # Build constraints for all candidates, then select the FIRST closest one whose
        # Jacobian row is usable (finite + non-zero). This prevents publishing an
        # all-zero `j_row` that would disable safety in the blender.
        #
        # NOTE: We use active_threshold=1e9 to keep behavior "closest wins" and avoid
        # depending on controller/blender influence distances.
        act = []
        for c in candidates_list:
            try:
                d = float(c.get("d", 1e9))
                if not bool(np.isfinite(d)):
                    continue
                act.append(c)
            except Exception:
                continue
        act.sort(key=lambda x: float(x.get("d", 1e9)))

        Gc, _, mc, _best_c_unused = build_cbf_constraints(
            list(act),
            float(1e9),
            K=int(len(act)) if len(act) > 0 else 1,
            cbf_eps=float(cp.cbf_eps),
            cbf_d_safe=float(cp.cbf_d_safe),
            approach_speed_limit=float(cp.cbf_approach_speed_limit),
            alpha_min=float(cp.cbf_alpha_min),
            alpha_max=float(cp.cbf_alpha_max),
            risk_d_far=float(cp.risk_d_far),
            risk_d_mid=float(cp.risk_d_mid),
            risk_d_near=float(cp.risk_d_near),
            stop_distance=float(cp.stop_d_in),
            model=model,
            data=data,
            q=q,
        )

        best_i = None
        try:
            for i in range(int(mc)):
                gi = np.array(Gc[i, :], dtype=float).reshape(-1)
                if (gi.shape[0] != 7) or (not bool(np.all(np.isfinite(gi)))):
                    continue
                if float(np.linalg.norm(gi)) > 1e-6:
                    best_i = int(i)
                    break
        except Exception:
            best_i = None

        if (best_i is not None) and (0 <= int(best_i) < len(act)):
            best_c = act[int(best_i)]
            d_closest = float(best_c.get("d", float(d_min_default)))
            j_row_closest = np.array(Gc[int(best_i), :], dtype=float).reshape(-1)
            hazard = str(best_c.get("hazard", "none"))

            # Optional: hold-time hysteresis on hazard switching (prevents oscillations in the blender).
            # IMPORTANT: when we "hold", we hold the whole (d, j_row, hazard) triplet to keep coherence.
            if (hold_state is not None) and (now_wall is not None):
                try:
                    hold_s = float(getattr(params, "hazard_hold_time_s", 0.0))
                    delta_m = float(getattr(params, "hazard_switch_delta_m", 0.0))

                    # Hysteresis rule (robust against chatter):
                    # - switch immediately only if the new hazard is clearly closer by `delta_m`
                    # - otherwise, keep the previous winner for `hold_s` seconds
                    if hazard != str(hold_state.last_hazard):
                        new_is_clearly_better = float(d_closest) < (float(hold_state.last_d) - float(delta_m))
                        hold_active = (float(now_wall) - float(hold_state.last_switch_wall)) < float(max(0.0, hold_s))

                        if (not new_is_clearly_better) and bool(hold_active):
                            # Hold previous coherent triplet
                            hazard = str(hold_state.last_hazard)
                            if hold_state.last_j_row is not None:
                                j_row_closest = np.array(hold_state.last_j_row, dtype=float).reshape(-1)
                            d_closest = float(hold_state.last_d)
                        else:
                            # Commit switch
                            hold_state.last_hazard = str(hazard)
                            hold_state.last_d = float(d_closest)
                            hold_state.last_j_row = np.array(j_row_closest, dtype=float).reshape(-1)
                            hold_state.last_switch_wall = float(now_wall)
                    else:
                        # Same hazard: keep state aligned with the freshest coherent data.
                        hold_state.last_d = float(d_closest)
                        hold_state.last_j_row = np.array(j_row_closest, dtype=float).reshape(-1)
                except Exception:
                    pass

            pubs.closest_constraint_pub.publish(Float64MultiArray(data=[float(d_closest)] + j_row_closest.tolist()))
            msg = String(); msg.data = str(hazard)
            pubs.closest_hazard_pub.publish(msg)
            return {
                "d": float(d_closest),
                "j_row": np.array(j_row_closest, dtype=float).reshape(-1),
                "hazard": str(hazard),
            }

        # If we could not compute a usable constraint, prefer holding the last coherent
        # triplet (if available) instead of publishing an invalid "no hazard" signal.
        if hold_state is not None and hold_state.last_j_row is not None:
            try:
                j_row_hold = np.array(hold_state.last_j_row, dtype=float).reshape(-1)
                if (j_row_hold.shape[0] == 7) and (float(np.linalg.norm(j_row_hold)) > 1e-6):
                    pubs.closest_constraint_pub.publish(
                        Float64MultiArray(data=[float(hold_state.last_d)] + j_row_hold.tolist())
                    )
                    msg = String(); msg.data = str(hold_state.last_hazard)
                    pubs.closest_hazard_pub.publish(msg)
                    return {
                        "d": float(hold_state.last_d),
                        "j_row": j_row_hold,
                        "hazard": str(hold_state.last_hazard),
                    }
            except Exception:
                pass

        pubs.closest_constraint_pub.publish(Float64MultiArray(data=[999.0] + [0.0] * 7))
        msg = String(); msg.data = "none"
        pubs.closest_hazard_pub.publish(msg)
        return None
    except Exception:
        return None
