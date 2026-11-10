"""Append-only schema of /cbf_status, shared with experiment manifests.

The first eleven fields keep their historical positions. Ages and tick gaps
use the node's ROS clock (simulation time in SIM); solve duration is wall time.
Residuals are max(G*x-h, 0) per family, evaluated AFTER clipping the command.
They have each family's own units and must not be compared across families.
NaN means no measurement, rather than a measured zero age or residual.
"""
from __future__ import annotations

import numpy as np

#: The six families whose slack/violation triples occupy their HISTORICAL
#: positions in STATUS_FIELDS. Do not add to this tuple — the three triples it
#: generates are interleaved, so a seventh name here would SHIFT every
#: `violation_*` column and silently re-interpret every bag recorded before it.
FAMILIES = ('obs', 'sc', 'qlim', 'sing', 'cap', 'spd')
#: Every family the QP actually has, in decision-vector order (this must stay
#: equal to ``cbf_state_rows.GROUP_NAMES``; a test pins it). The extra ones are
#: appended to the END of the schema instead, one column each.
FAMILIES_ALL = FAMILIES + ('ssb',)
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
    # ── schema v3: the Self-Shadow Barrier family (APPENDED) ─────────────
    # At the END, not beside the other families' columns, so that every field
    # of v2 keeps its index and a v2 bag still zips correctly against the
    # first 38 names. A v2 producer simply stops short of these three, which
    # reads as "not measured" exactly like any other missing tail.
    *(f'slack_{g}' for g in FAMILIES_ALL[len(FAMILIES):]),
    *(f'violation_without_slack_{g}' for g in FAMILIES_ALL[len(FAMILIES):]),
    *(f'violation_with_slack_{g}' for g in FAMILIES_ALL[len(FAMILIES):]),
)
STATUS_SCHEMA_VERSION = 3


def family_violations(G, rhs, command, slack, groups):
    """Unrelaxed and slack-relaxed positive residuals of the emitted command.

    Indexed by the FAMILY INDEX (``cbf_state_rows.G_*``), so both arrays are
    ``len(FAMILIES_ALL)`` long and the caller slices whichever columns the
    schema asks for. Sized off the family list and not off a literal, because a
    family added to the QP and forgotten here would report 0.0 — a measured
    zero — for a row that was violated.
    """
    n = len(FAMILIES_ALL)
    raw = np.zeros(n)
    relaxed = np.zeros(n)
    if G is not None and len(G):
        r = G[:, :len(command)] @ command - rhs
        rs = r + G[:, len(command):] @ slack
        for g in range(n):
            mask = groups == g
            if np.any(mask):
                raw[g] = max(0.0, float(np.max(r[mask])))
                relaxed[g] = max(0.0, float(np.max(rs[mask])))
    return raw, relaxed
