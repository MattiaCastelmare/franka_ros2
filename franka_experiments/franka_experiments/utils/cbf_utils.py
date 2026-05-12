import os

import yaml
from ament_index_python.packages import get_package_share_directory

from franka_experiments.utils.math_utils import skew  # noqa: F401 — re-exported


def load_robot_config(file):
    pkg_share = get_package_share_directory('franka_experiments')
    filename = os.path.join(pkg_share, 'config', f'fr3_{file}.yaml')
    with open(filename, 'r') as f:
        data = yaml.safe_load(f)
    return data


def select_gamma(zone, confidence):
    base = 3.0

    if zone == "warning":
        base = 4.0
    elif zone == "danger":
        base = 8.0
    elif zone == "critical":
        base = 15.0

    conf_scale = max(0.3, min(1.0, confidence))
    return base * conf_scale