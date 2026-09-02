import os

import numpy as np
import yaml
from ament_index_python.packages import get_package_share_directory

from franka_experiments.utils.math_utils import skew  # noqa: F401 — re-exported
# MOVED to utils/config.py (Phase 2). Re-exported under the original name so
# every existing call site keeps working unchanged.
from franka_experiments.utils.config import (  # noqa: F401
    load_package_config as load_robot_config,
)


def select_gamma(zone: str, confidence: float,
                 d: float = None, d_safe: float = 0.20) -> float:
    """Return gamma for the ZCBF bound  b = -gamma * h.

    When d and d_safe are provided, gamma is a continuous exponential function
    of the clearance delta = d - d_safe:

        gamma(delta) = gamma_max - (gamma_max - gamma_min) * exp(-K * delta)

    Calibrated so that:
      delta  = 0   m  (at boundary):  gamma ≈ gamma_max = 12.0
      delta  = 0.1 m:                 gamma ≈ 6.0
      delta >> 0   (far):             gamma ≈ gamma_min = 2.0

    This avoids the discrete zone jumps that caused jerk when noisy distances
    crossed zone thresholds.  The legacy zone string is kept as a fallback.
    """
    GAMMA_MIN = 2.0
    GAMMA_MAX = 12.0
    K         = 18.0   # decay rate [1/m]

    if d is not None and d_safe is not None:
        delta = max(float(d) - float(d_safe), 0.0)
        # HIGH gamma when close (delta≈0), LOW gamma when far (delta large):
        #   gamma(0) = gamma_max,  gamma(∞) = gamma_min
        base  = GAMMA_MIN + (GAMMA_MAX - GAMMA_MIN) * np.exp(-K * delta)
    else:
        _zone_base = {'warning': 4.0, 'danger': 8.0, 'critical': 12.0}
        base = _zone_base.get(zone, 3.0)

    conf_scale = max(0.5, min(1.0, float(confidence)))
    return float(base * conf_scale)