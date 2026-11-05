"""Append-only schema of /cbf_status, shared with experiment manifests.

The first eleven fields keep their historical positions. Ages and tick gaps
use the node's ROS clock (simulation time in SIM); solve duration is wall time.
Residuals are max(G*x-h, 0) per family, evaluated AFTER clipping the command.
They have each family's own units and must not be compared across families.
NaN means no measurement, rather than a measured zero age or residual.
"""
from __future__ import annotations

import numpy as np

FAMILIES = ('obs', 'sc', 'qlim', 'sing', 'cap', 'spd')
STATUS_FIELDS = (
    'n_rows', 'slack_max', 'fault_braking', 'n_violated', 'd_obstacle_min',
    'ssm_sp_min', 'ssm_vcap_min', 'v_closing_max', 'iso_stop_latched',
    'v_obs_conditioned', 'v_obs_hdot',
    'fallback_level', 'slew_conflict_joints', 'solve_wall_ms', 'tick_gap_ros_s',
    'joint_receipt_age_s', 'distance_capture_age_s', 'distance_receipt_age_s',
    'constraint_snapshot_age_s',
    *(f'slack_{g}' for g in FAMILIES),
    *(f'violation_without_slack_{g}' for g in FAMILIES),
    *(f'violation_with_slack_{g}' for g in FAMILIES),
    'constraint_build_wall_ms',
)
STATUS_SCHEMA_VERSION = 2


def family_violations(G, rhs, command, slack, groups):
    """Unrelaxed and slack-relaxed positive residuals of the emitted command."""
    raw = np.zeros(len(FAMILIES))
    relaxed = np.zeros(len(FAMILIES))
    if G is not None and len(G):
        r = G[:, :len(command)] @ command - rhs
        rs = r + G[:, len(command):] @ slack
        for g in range(len(FAMILIES)):
            mask = groups == g
            if np.any(mask):
                raw[g] = max(0.0, float(np.max(r[mask])))
                relaxed[g] = max(0.0, float(np.max(rs[mask])))
    return raw, relaxed
