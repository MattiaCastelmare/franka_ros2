import numpy as np
import yaml


def load_robot_config(file):
    filename = f'/ros2_ws/src/franka_experiments/config/fr3_{file}.yaml'
    with open(filename, 'r') as f:
        data = yaml.safe_load(f)
    return data

def skew(v):
    return np.array([
        [0.0, -v[2], v[1]],
        [v[2], 0.0, -v[0]],
        [-v[1], v[0], 0.0]
    ])

def select_gamma(zone, confidence):
    base = 3.0

    if zone == "warning":
        base = 4.0
    elif zone == "danger":
        base = 8.0
    elif zone == "critical":
        base = 15.0

    # Optional: reduce aggressiveness if confidence is low
    conf_scale = max(0.3, min(1.0, confidence))
    return base * conf_scale