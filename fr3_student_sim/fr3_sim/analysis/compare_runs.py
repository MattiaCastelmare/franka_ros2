"""Compare first-impact, tracking, constraints and timing in two saved runs.

    ros2 run fr3_student_sim compare_runs baseline.npz candidate.npz --output report.json

Contact metrics reuse ``first_impact`` and therefore cover only the first
compression/release cycle. Ground-truth contact signals are used for offline
evaluation only. Constraint checks concern saved samples, not the motion
between samples, and differentiated velocities are acceleration estimates.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from fr3_sim.simulation.fr3_model import DQ_MAX, Q_MAX, Q_MIN, TAU_MAX
from fr3_sim.analysis.impact_metrics import first_impact
from fr3_sim.analysis.timing import timing_metrics


# Include removed controller names only to keep historical logs readable.
_PERP_CONTROLLERS = ('baseline_perp.py', 'controller_baseline_perp.py',
                     'optimized_hqp_perp.py', 'optimized_3phases_perp.py',
                     'optimized_perp_full_online.py')
_HQP_TRACKING_CONTROLLERS = ('optimized_hqp.py', 'optimized_hqp_perp.py',
                             'optimized_3phases_perp.py',
                             'optimized_perp_full_online.py', 'impedance_hqp.py',
                             'controller_impedance_hqp.py')
_ACCELERATION_TRACKING_CONTROLLERS = (
    'baseline.py', 'baseline_perp.py', 'controller_baseline.py',
    'controller_baseline_perp.py', 'controller_optimized.py',
) + _HQP_TRACKING_CONTROLLERS


def _controller_name(metadata):
    return str(metadata.get('controller', '')).replace('\\', '/').rsplit('/', 1)[-1]


def load_run(path):
    """Load numeric NPZ arrays and JSON metadata without enabling pickle."""
    with np.load(path, allow_pickle=False) as archive:
        run = {key: archive[key] for key in archive.files if key != 'meta'}
        run['meta'] = json.loads(str(archive['meta']))
    return run


def _array(run, key, width=None):
    if key not in run:
        return None
    values = np.asarray(run[key], dtype=float)
    tail = () if width is None else (width if isinstance(width, tuple) else (width,))
    expected = (len(run['t']),) + tail
    if values.shape != expected:
        raise ValueError(f'{key} must have shape {expected}, got {values.shape}')
    return values


def _finite_max(values, axis=None):
    if values is None or not np.size(values) or not np.all(np.isfinite(values)):
        return None
    return np.max(values, axis=axis)


def _tracking_stats(t, error, mask):
    if error is None or not np.any(mask):
        return {'samples': 0, 'rms_m': None, 'max_m': None}
    times = t[mask]
    norm = np.linalg.norm(error[mask], axis=1)
    if not np.all(np.isfinite(norm)):
        return {'samples': int(len(norm)), 'rms_m': None, 'max_m': None}
    rms = (np.sqrt(np.trapz(norm**2, times) / (times[-1] - times[0]))
           if len(times) > 1 else norm[0])
    return {'samples': int(len(norm)), 'rms_m': float(rms), 'max_m': float(np.max(norm))}


def _vector_setting(value, *, default=None):
    if value is None:
        return default
    values = np.asarray(value, dtype=float)
    if values.ndim == 0:
        values = np.full(7, float(values))
    if values.shape != (7,) or not np.all(np.isfinite(values)):
        return default
    return values


def _qp_status(run, key, updates):
    values = _array(run, key)
    if values is None:
        return {'available': False, 'failed_updates': None, 'status_counts': None}
    values = values[updates]
    finite = np.isfinite(values)
    unique, counts = np.unique(values[finite], return_counts=True)
    # OSQP: 1=solved, 2=solved inaccurate; 0 denotes a skipped secondary solve.
    accepted = (0, 1, 2) if key.endswith('secondary_status') else (1, 2)
    return {'available': True,
            'failed_updates': int(np.count_nonzero(~finite | ~np.isin(values, accepted))),
            'status_counts': {str(int(code)): int(count) for code, count in zip(unique, counts)},
            'nonfinite_updates': int(np.count_nonzero(~finite))}


def summarize_run(run):
    """Summarize one log, keeping unavailable diagnostics as ``None``."""
    t = np.asarray(run['t'], dtype=float)
    if (t.ndim != 1 or not len(t) or not np.all(np.isfinite(t))
            or np.any(np.diff(t) <= 0.0)):
        raise ValueError('run timestamps must be nonempty, finite and increasing')
    meta = run.get('meta', {})
    params = meta.get('params', {})
    impact = first_impact(run)['metrics']
    q, dq = _array(run, 'q', 7), _array(run, 'dq', 7)
    q_finite = q is not None and bool(np.all(np.isfinite(q)))
    dq_finite = dq is not None and bool(np.all(np.isfinite(dq)))
    margins = np.minimum(q - Q_MIN, Q_MAX - q) if q_finite else None
    position_bad = np.any(margins < -1e-9, axis=1) if q_finite else None
    velocity_bad = (np.any(np.abs(dq) > DQ_MAX + 1e-9, axis=1)
                    if dq_finite else None)
    acceleration = (np.gradient(dq, t, axis=0, edge_order=1)
                    if dq_finite and len(t) > 1 else None)

    safety = params.get('safety', {})
    soft_settings = safety.get('soft_constraints', {})
    soft_enabled = bool(soft_settings.get('enabled', False))
    margin_value = safety.get('joint_margin', safety.get('position_margin'))
    if margin_value is None:
        margin_value = params.get('joint_release', {}).get('margin')
    protected_margin = _vector_setting(margin_value)
    protected = margins - protected_margin if margins is not None and protected_margin is not None else None
    hard_margin = (_vector_setting(soft_settings.get('hard_position_margin'))
                   if soft_enabled else None)
    hard_protected = margins - hard_margin if margins is not None and hard_margin is not None else None
    configured_velocity = _vector_setting(safety.get('velocity_max'))
    configured_acceleration = _vector_setting(safety.get('acceleration_max'))
    configured_torque = _vector_setting(safety.get('torque_max'))
    limits = {
        'scope': 'saved samples only; no guarantee between samples',
        'finite_q': q_finite, 'finite_dq': dq_finite,
        'physical_q_min_rad': Q_MIN.tolist(), 'physical_q_max_rad': Q_MAX.tolist(),
        'physical_dq_max_rad_s': DQ_MAX.tolist(),
        'runner_stop_margin_rad': 0.05,
        'runner_position_stop_samples': int(np.count_nonzero(np.any(margins < 0.05 - 1e-9, axis=1)))
        if margins is not None else None,
        'soft_constraints_enabled': soft_enabled,
        'protected_margin_is_soft': soft_enabled,
        'acceleration_limit_is_soft': soft_enabled,
        'min_physical_margin_rad': float(np.min(margins)) if margins is not None else None,
        'min_physical_margin_per_joint_rad': np.min(margins, axis=0) if margins is not None else None,
        'position_violation_samples': int(np.count_nonzero(position_bad)) if position_bad is not None else None,
        'position_violation_joints': (np.flatnonzero(np.any(margins < -1e-9, axis=0)) + 1).tolist()
        if margins is not None else None,
        'velocity_violation_samples': int(np.count_nonzero(velocity_bad)) if velocity_bad is not None else None,
        'velocity_violation_joints': (np.flatnonzero(np.any(np.abs(dq) > DQ_MAX + 1e-9, axis=0)) + 1).tolist()
        if dq_finite else None,
        'max_abs_dq_per_joint_rad_s': _finite_max(np.abs(dq), axis=0) if dq is not None else None,
        'configured_protected_margin_rad': protected_margin,
        'min_protected_margin_rad': float(np.min(protected)) if protected is not None else None,
        'protected_margin_violation_samples': int(np.count_nonzero(np.any(protected < -1e-9, axis=1)))
        if protected is not None else None,
        'configured_hard_position_margin_rad': hard_margin,
        'hard_position_margin_violation_samples': int(np.count_nonzero(np.any(hard_protected < -1e-9, axis=1)))
        if hard_protected is not None else None,
        'configured_velocity_max_rad_s': configured_velocity,
        'configured_velocity_violation_samples': int(np.count_nonzero(
            np.any(np.abs(dq) > configured_velocity + 1e-9, axis=1)))
        if dq_finite and configured_velocity is not None else None,
        'configured_acceleration_max_rad_s2': configured_acceleration,
        'acceleration_estimate_violation_samples': int(np.count_nonzero(
            np.any(np.abs(acceleration) > configured_acceleration + 1e-6, axis=1)))
        if acceleration is not None and configured_acceleration is not None else None,
        'max_abs_acceleration_estimate_per_joint_rad_s2': _finite_max(np.abs(acceleration), axis=0)
        if acceleration is not None else None,
        'acceleration_estimate_method': 'nonuniform finite differences of logged dq; not exact plant acceleration',
    }

    error = _array(run, 'user_position_error', 3)
    if error is None and 'user_p_ref' in run and 'p' in run:
        error = _array(run, 'user_p_ref', 3) - _array(run, 'p', 3)
    before = (t < impact['t_contact_s'] if impact['t_contact_s'] is not None
              else np.zeros(len(t), dtype=bool))
    tracking = {'rms_definition': 'time-weighted norm using trapezoids at actual timestamps',
                'full_run': _tracking_stats(t, error, np.ones(len(t), dtype=bool)),
                'precontact': _tracking_stats(t, error, before)}
    normal_speed = _array(run, 'v_n')
    tracking['normal_speed_min_mps'] = float(np.min(normal_speed)) if normal_speed is not None else None
    tracking['normal_speed_max_mps'] = _finite_max(normal_speed)

    raw, applied = _array(run, 'tau_cmd', 7), _array(run, 'tau', 7)
    saturated = (np.abs(raw - applied) > 1e-9 if raw is not None and applied is not None
                 and np.all(np.isfinite(raw)) and np.all(np.isfinite(applied)) else None)
    torque = {
        'limits_Nm': TAU_MAX.tolist(),
        'configured_limits_Nm': configured_torque,
        'max_abs_command_per_joint_Nm': _finite_max(np.abs(raw), axis=0) if raw is not None else None,
        'max_command_limit_ratio': _finite_max(np.abs(raw) / TAU_MAX) if raw is not None else None,
        'max_command_limit_ratio_per_joint': _finite_max(np.abs(raw) / TAU_MAX, axis=0) if raw is not None else None,
        'max_configured_limit_ratio': _finite_max(np.abs(raw) / configured_torque)
        if raw is not None and configured_torque is not None and np.all(configured_torque > 0.0) else None,
        'saturated_samples': int(np.count_nonzero(np.any(saturated, axis=1))) if saturated is not None else None,
        'saturated_samples_pct': 100.0 * float(np.mean(np.any(saturated, axis=1))) if saturated is not None else None,
        'saturated_samples_per_joint': np.count_nonzero(saturated, axis=0) if saturated is not None else None,
        'scope': 'gravity-compensated actuator command before/after runner clipping; wall torque excluded',
    }
    motor = _array(run, 'user_tau_motor_predicted', 7)
    torque['nominal_motor_including_gravity_logged'] = motor is not None
    torque['max_nominal_motor_limit_ratio'] = (
        _finite_max(np.abs(motor) / configured_torque)
        if motor is not None and configured_torque is not None else None)
    updated = _array(run, 'controller_updated')
    updates = updated.astype(bool) if updated is not None else np.ones(len(t), dtype=bool)
    qp = {'count_scope': 'controller updates' if updated is not None else 'saved rows; legacy update flags unavailable',
          'primary': _qp_status(run, 'user_qp_primary_status', updates),
          'secondary': _qp_status(run, 'user_qp_secondary_status', updates),
          'safety': _qp_status(run, 'user_qp_safety_status', updates)}
    violation = _array(run, 'user_qp_constraint_violation')
    qp['max_logged_hard_constraint_violation'] = _finite_max(violation[updates]) if violation is not None else None
    hard_lower, hard_upper = _array(run, 'user_qdd_lower', 7), _array(run, 'user_qdd_upper', 7)
    commanded_acceleration = _array(run, 'user_qdd_command', 7)
    bounds_available = all(value is not None for value in (hard_lower, hard_upper, commanded_acceleration))
    bounds_finite = bounds_available and all(np.all(np.isfinite(value[updates]))
                                          for value in (hard_lower, hard_upper, commanded_acceleration))
    qp['hard_bound_diagnostics_available'] = bounds_available
    qp['hard_bound_diagnostics_finite'] = bool(bounds_finite)
    qp['max_logged_hard_acceleration_bound_violation'] = (float(max(
        0.0, np.max(hard_lower[updates] - commanded_acceleration[updates]),
        np.max(commanded_acceleration[updates] - hard_upper[updates])))
        if bounds_finite and np.any(updates) else None)
    slack = _array(run, 'user_soft_slack_max')
    normalized_slack = _array(run, 'user_soft_slack_normalized_max')
    slack_lower = _array(run, 'user_soft_slack_lower', (3, 7))
    slack_upper = _array(run, 'user_soft_slack_upper', (3, 7))
    slack_finite = (slack is not None and normalized_slack is not None and np.any(updates)
                    and slack_lower is not None and slack_upper is not None
                    and np.all(np.isfinite(slack[updates]))
                    and np.all(np.isfinite(normalized_slack[updates]))
                    and np.all(np.isfinite(slack_lower[updates]))
                    and np.all(np.isfinite(slack_upper[updates])))
    qp['soft_constraints'] = {
        'enabled': soft_enabled, 'finite_diagnostics_available': bool(slack_finite),
        'max_slack_rad_s2': _finite_max(slack[updates]) if slack is not None else None,
        'max_normalized_slack': _finite_max(normalized_slack[updates]) if normalized_slack is not None else None,
        'updates_with_slack': int(np.count_nonzero(normalized_slack[updates] > 1e-6))
        if slack_finite else None,
        'updates_with_slack_pct': float(100 * np.mean(normalized_slack[updates] > 1e-6))
        if slack_finite else None,
    }
    if slack_finite:
        physical_slack = np.maximum(slack_lower[updates], slack_upper[updates])
        qp['soft_constraints']['groups'] = {
            name: {'max_slack_rad_s2': float(np.max(physical_slack[:, group])),
                   'updates_with_slack': int(np.count_nonzero(np.any(
                       physical_slack[:, group] > 1e-6, axis=1)))}
            for group, name in enumerate(('acceleration', 'position_barrier', 'velocity_barrier'))}
    safety_ms = _array(run, 'user_qp_safety_ms')
    if safety_ms is not None:
        selected_ms = safety_ms[updates]
        qp['safety_timing_ms'] = {
            'mean': float(np.mean(selected_ms)) if np.all(np.isfinite(selected_ms)) and len(selected_ms) else None,
            'p95': float(np.percentile(selected_ms, 95)) if np.all(np.isfinite(selected_ms)) and len(selected_ms) else None,
            'max': _finite_max(selected_ms),
        }
    observer_valid = _array(run, 'user_observer_valid')
    qp['observer_valid_updates_pct'] = (100.0 * float(np.mean(observer_valid[updates] > 0.5))
                                         if observer_valid is not None and np.any(updates) else None)
    logged_margin = run.get('user_joint_limit_margin')
    qp['min_logged_joint_limit_margin_rad'] = (float(np.min(logged_margin))
                                               if logged_margin is not None and np.all(np.isfinite(logged_margin)) else None)
    for key in ('user_qdd_predicted', 'user_acceleration_uncertainty'):
        values = _array(run, key, 7)
        qp['max_abs_' + key.removeprefix('user_') + '_per_joint_rad_s2'] = (
            _finite_max(np.abs(values), axis=0) if values is not None else None)

    configured_duration = meta.get('duration')
    requested = float(configured_duration) if configured_duration is not None else None
    stop_reason = str(meta.get('stop_reason', ''))
    full_duration = (requested is not None and np.isfinite(requested)
                     and stop_reason == 'duration reached' and t[-1] >= requested - 1e-6)
    trial_complete = _array(run, 'user_trial_complete')
    # Finite approaches deliberately finish before the launch-duration cap.
    # An explicit endpoint/hold flag distinguishes normal completion from an
    # early QP failure; older logs without this flag remain unconfirmed.
    scheduled_complete = (_controller_name(meta) in _PERP_CONTROLLERS
                          and stop_reason == 'controller set done'
                          and trial_complete is not None
                          and np.isfinite(trial_complete[-1])
                          and trial_complete[-1] >= 0.5)
    timing = timing_metrics(run) if 'ctrl_ms' in run else {'unavailable': 'ctrl_ms not logged'}
    wc = _array(run, 'wall_clock')
    timing['real_time_factor_measured'] = ((t[-1] - t[0]) / (wc[-1] - wc[0])
                                           if wc is not None and len(t) > 1 and wc[-1] > wc[0] else None)
    return {'controller': meta.get('controller'), 'samples': int(len(t)),
            'duration_s': float(t[-1]), 'sampled_span_s': float(t[-1] - t[0]),
            'configured_duration_s': requested, 'stop_reason': stop_reason,
            'full_duration_reached': bool(full_duration),
            'scheduled_trial_complete': bool(scheduled_complete),
            'run_completed': bool(full_duration or scheduled_complete),
            'safety_stop': 'SAFETY STOP' in stop_reason.upper(),
            'first_impact': impact, 'tracking': tracking, 'limits': limits,
            'torque': torque, 'qp': qp, 'timing': timing}


def _same(a, b):
    if isinstance(a, dict) and isinstance(b, dict):
        return set(a) == set(b) and all(_same(a[key], b[key]) for key in a)
    if isinstance(a, (list, tuple)) and isinstance(b, (list, tuple)):
        return len(a) == len(b) and all(_same(x, y) for x, y in zip(a, b))
    if isinstance(a, (int, float)) and isinstance(b, (int, float)):
        return bool(np.isclose(a, b, rtol=1e-9, atol=1e-12))
    return a == b


def _primary_tracking(metadata):
    """Identify recorded tracking settings without guessing historical defaults.

    HQP filenames have survived the change from physical impedance gains to
    acceleration feedback. Their names alone do not establish the control law.
    """
    out = {'law': 'unknown', 'gains': {}, 'metadata_complete': False,
           'reason': 'explicit primary-tracking gains are not available'}
    name = _controller_name(metadata)
    if name not in _ACCELERATION_TRACKING_CONTROLLERS:
        out['reason'] = 'controller implementation is not recognized'
        return out
    params = metadata.get('params', {})
    if not isinstance(params, dict):
        return out
    tracking, impedance = params.get('tracking'), params.get('impedance')
    has_tracking = isinstance(tracking, dict) and any(
        key in tracking for key in ('kp_position', 'kd_position'))
    has_impedance = isinstance(impedance, dict) and bool(impedance)
    if has_tracking and has_impedance:
        out['reason'] = 'both acceleration-tracking and legacy impedance settings are recorded'
        return out
    if has_tracking:
        out['law'] = ('three_phase_tracking_impedance' if name == 'optimized_3phases_perp.py'
                      else 'cartesian_acceleration_feedback')
        settings, keys = tracking, ('kp_position', 'kd_position')
    elif has_impedance and name in _HQP_TRACKING_CONTROLLERS:
        out['law'] = 'cartesian_impedance'
        settings, keys = impedance, ('stiffness', 'damping')
    else:
        return out
    for key in keys:
        value = settings.get(key)
        try:
            gain = np.asarray(value, dtype=float)
            if gain.ndim == 0:
                gain = np.full(3, float(gain))
        except (TypeError, ValueError, OverflowError):
            gain = None
        valid = (gain is not None and gain.shape == (3,)
                 and np.all(np.isfinite(gain)) and np.all(gain > 0.0))
        out['gains'][key] = gain.tolist() if valid else None
    complete = all(value is not None for value in out['gains'].values())
    if out['law'] == 'three_phase_tracking_impedance':
        # The approach gains alone do not identify this controller's contact
        # law or when its transition occurs. Never classify it as the same
        # baseline feedback simply because both YAMLs contain 100/20.
        phase = params.get('three_phases', {})
        phase = phase if isinstance(phase, dict) else {}
        nonnegative = {'contact_distance', 'contact_penetration'}
        policy = {}
        for key in ('transition_distance', 'contact_distance', 'contact_confirmation_s',
                    'contact_hold_duration', 'contact_timeout', 'contact_penetration',
                    'stiffness_normal', 'stiffness_tangent', 'damping_normal', 'damping_tangent'):
            raw = phase.get(key)
            try:
                value = float(raw)
                valid = (not isinstance(raw, bool) and np.isfinite(value)
                         and (value >= 0.0 if key in nonnegative else value > 0.0))
            except (TypeError, ValueError, OverflowError):
                value, valid = None, False
            policy[key] = value if valid else None
        out['phase_policy'] = policy
        complete = complete and all(value is not None for value in policy.values())
        if complete:
            complete = (policy['contact_distance'] < policy['transition_distance']
                        and policy['contact_confirmation_s'] < policy['contact_timeout'])
        out['metadata_complete'] = bool(complete)
        out['reason'] = ('explicit approach gains and three-phase contact policy are complete'
                         if complete else
                         'explicit valid approach gains and three-phase contact policy are required')
    else:
        out['metadata_complete'] = complete
        out['reason'] = ('explicit primary-tracking gains are complete'
                         if complete else
                         'primary-tracking gains must be explicit positive finite scalars or three-vectors')
    return out


def _compare_primary_tracking(baseline_metadata, candidate_metadata):
    baseline, candidate = map(_primary_tracking, (baseline_metadata, candidate_metadata))
    out = {'baseline': baseline, 'candidate': candidate, 'status': 'unverifiable',
           'same': None, 'gain_matches': None,
           'reason': 'the same primary tracking cannot be verified from the recorded metadata'}
    if not baseline['metadata_complete'] or not candidate['metadata_complete']:
        return out
    if baseline['law'] != candidate['law']:
        out.update(status='different', same=False,
                   reason=('primary tracking formulations differ: '
                           f"{baseline['law']} versus {candidate['law']}"))
        return out
    matches = {key: _same(baseline['gains'][key], candidate['gains'][key])
               for key in baseline['gains']}
    policy_matches = _same(baseline.get('phase_policy'), candidate.get('phase_policy'))
    same = all(matches.values()) and policy_matches
    if 'phase_policy' in baseline or 'phase_policy' in candidate:
        out['phase_policy_matches'] = policy_matches
    out.update(status='same' if same else 'different', same=same, gain_matches=matches,
               reason=('the recorded primary tracking law and Cartesian gains match'
                       if same else 'the recorded Cartesian primary-tracking gains or phase policies differ'))
    return out


def _baseline_identity(run):
    meta = run.get('meta', {})
    name = _controller_name(meta)
    if name in ('baseline.py', 'baseline_perp.py', 'controller_baseline.py',
                'controller_baseline_perp.py'):
        return True, name
    # Historical zero-gain logs remain usable; no removed controller is imported.
    params = meta.get('params', {})
    gain = params.get('mass_minimization', {}).get('gain')
    initial = params.get('initial_q')
    posture = params.get('q_posture_ref', initial)
    if (name == 'controller_optimized.py' and gain == 0.0 and initial is not None
            and _same(posture, initial)):
        return True, 'controller_optimized.py with zero mass gain and initial-q posture'
    return False, 'expected baseline controller or optimized with gain=0 and q_posture_ref=initial_q'


def _mass_direction_matches_contact(run):
    meta = run.get('meta', {})
    params = meta.get('params', {})
    wall_normal = meta.get('wall', {}).get('normal')
    direction = params.get('mass_minimization', {}).get('direction',
                params.get('wall_geometry', {}).get('normal', wall_normal))
    if direction is None or wall_normal is None:
        return False
    direction, wall_normal = np.asarray(direction, float), np.asarray(wall_normal, float)
    if direction.shape != (3,) or wall_normal.shape != (3,):
        return False
    product = np.linalg.norm(direction) * np.linalg.norm(wall_normal)
    return bool(np.isfinite(product) and product > 0.0
                and abs(float(direction @ wall_normal)) / product >= 1.0 - 1e-9)


def compare_runs(baseline, candidate, *, entry_speed_tolerance=0.05, mass_reduction_target=0.30):
    """Return a JSON-compatible report with separate metric and full-run criteria."""
    if not np.isfinite(entry_speed_tolerance) or entry_speed_tolerance < 0.0:
        raise ValueError('entry-speed tolerance must be finite and nonnegative')
    if not np.isfinite(mass_reduction_target) or not 0.0 <= mass_reduction_target <= 1.0:
        raise ValueError('mass-reduction target must be between zero and one')
    summaries = {'baseline': summarize_run(baseline), 'candidate': summarize_run(candidate)}
    a, b = baseline.get('meta', {}), candidate.get('meta', {})
    pa, pb = a.get('params', {}), b.get('params', {})
    reference_key = 'approach' if 'approach' in pa or 'approach' in pb else 'circle'
    scenario_fields = {
        'initial_q': (pa.get('initial_q'), pb.get('initial_q')),
        reference_key: (pa.get(reference_key), pb.get(reference_key)),
        'wall': (a.get('wall'), b.get('wall')),
        'realism': (a.get('realism'), b.get('realism')),
        'control_rate': (a.get('control_rate'), b.get('control_rate')),
        'duration': (a.get('duration'), b.get('duration')),
    }
    scenario = {key: {'match': va is not None and vb is not None and _same(va, vb),
                      'baseline': va, 'candidate': vb}
                for key, (va, vb) in scenario_fields.items()}
    same_scenario = all(field['match'] for field in scenario.values())
    identity, identity_reason = _baseline_identity(baseline)
    ia, ib = summaries['baseline']['first_impact'], summaries['candidate']['first_impact']
    ma, mb = ia['entry_mass_kg'], ib['entry_mass_kg']
    va, vb = ia['entry_speed_mps'], ib['entry_speed_mps']
    reduction = (1.0 - mb / ma if ma is not None and mb is not None and ma > 0.0 else None)
    speed_difference = (abs(vb - va) / abs(va)
                        if va is not None and vb is not None and abs(va) > 1e-12 else None)
    checks = {
        'recognized_baseline': identity,
        'same_scenario': same_scenario,
        'mass_directions_match_contact_normal': (_mass_direction_matches_contact(baseline)
                                                   and _mass_direction_matches_contact(candidate)),
        'both_first_impacts_complete': bool(ia['contact'] and ib['contact']
                                             and ia['first_impact_complete'] and ib['first_impact_complete']),
        'both_precontact_tracking_errors_available': all(
            summary['tracking']['precontact']['rms_m'] is not None for summary in summaries.values()),
        'positive_comparable_entry_speeds': bool(va is not None and vb is not None and va > 0.0 and vb > 0.0
                                                and speed_difference is not None
                                                and speed_difference <= entry_speed_tolerance),
        'measured_mass_reduction_at_least_target': reduction is not None and reduction >= mass_reduction_target - 1e-12,
    }
    metrics_met = all(checks.values())
    primary_tracking = _compare_primary_tracking(a, b)
    checks['same_primary_tracking'] = primary_tracking['same'] is True
    for label, summary in summaries.items():
        limits = summary['limits']
        if _controller_name(summary) in _PERP_CONTROLLERS:
            checks[label + '_run_completed'] = summary['run_completed']
        else:
            checks[label + '_full_duration'] = summary['full_duration_reached']
        checks[label + '_no_safety_stop'] = not summary['safety_stop']
        checks[label + '_sampled_physical_limits_respected'] = bool(
            limits['finite_q'] and limits['finite_dq'] and limits['position_violation_samples'] == 0
            and limits['velocity_violation_samples'] == 0)
    candidate_summary = summaries['candidate']
    candidate_limits = candidate_summary['limits']
    elastic = candidate_limits['soft_constraints_enabled']
    if elastic:
        checks['candidate_hard_position_margin_respected'] = (
            candidate_limits['configured_hard_position_margin_rad'] is not None
            and candidate_limits['hard_position_margin_violation_samples'] == 0)
        for label, summary in summaries.items():
            checks[label + '_runner_position_stop_band_avoided'] = (
                summary['limits']['runner_position_stop_samples'] == 0)
    elif candidate_limits['configured_protected_margin_rad'] is not None:
        checks['candidate_protected_position_margin_respected'] = (
            candidate_limits['protected_margin_violation_samples'] == 0)
    if candidate_limits['configured_velocity_max_rad_s'] is not None:
        checks['candidate_configured_velocity_limit_respected'] = (
            candidate_limits['configured_velocity_violation_samples'] == 0)
    # Differentiation is not an exact acceleration sensor. Nevertheless an
    # observed excess must prevent a positive full-criterion claim.
    if not elastic and candidate_limits['acceleration_estimate_violation_samples'] is not None:
        checks['candidate_sampled_acceleration_limit_respected'] = (
            candidate_limits['acceleration_estimate_violation_samples'] == 0)
    command_ratio = candidate_summary['torque']['max_configured_limit_ratio']
    if command_ratio is not None:
        checks['candidate_command_torque_limit_respected'] = command_ratio <= 1.0 + 1e-5
    motor_ratio = candidate_summary['torque']['max_nominal_motor_limit_ratio']
    if motor_ratio is not None:
        checks['candidate_nominal_motor_torque_limit_respected'] = motor_ratio <= 1.0 + 1e-5
    qp = candidate_summary['qp']
    if elastic:
        checks['candidate_safety_qp_updates_solved'] = (
            qp['safety']['available'] and qp['safety']['failed_updates'] == 0)
        feasibility = float(pb.get('qp', {}).get('feasibility_tolerance', 2e-4))
        bound_violation = qp['max_logged_hard_acceleration_bound_violation']
        checks['candidate_logged_hard_bounds_accepted'] = (
            qp['hard_bound_diagnostics_available'] and qp['hard_bound_diagnostics_finite']
            and bound_violation is not None and bound_violation <= feasibility + 1e-9)
        checks['candidate_soft_constraint_diagnostics_available'] = (
            qp['soft_constraints']['finite_diagnostics_available'])
        # Missing command/motor telemetry cannot establish hard torque compliance.
        checks['candidate_command_torque_limit_respected'] = (
            command_ratio is not None and command_ratio <= 1.0 + 1e-5)
        checks['candidate_nominal_motor_torque_limit_respected'] = (
            motor_ratio is not None and motor_ratio <= 1.0 + 1e-5)
    if qp['primary']['available']:
        checks['candidate_primary_qp_updates_solved'] = qp['primary']['failed_updates'] == 0
        feasibility = float(pb.get('qp', {}).get('feasibility_tolerance', 2e-4))
        violation = qp['max_logged_hard_constraint_violation']
        checks['candidate_logged_qp_hard_constraints_accepted'] = (
            violation is not None and violation <= feasibility + 1e-9)
    elif str(b.get('controller', '')).endswith(
            ('optimized_hqp.py', 'optimized_hqp_perp.py', 'optimized_3phases_perp.py',
             'optimized_perp_full_online.py',
             'impedance_hqp.py', 'controller_impedance_hqp.py')):
        checks['candidate_primary_qp_updates_solved'] = False
    comparison = {'baseline_identity_reason': identity_reason, 'scenario': scenario,
                  'primary_tracking': primary_tracking,
                  'measured_mass_reduction_fraction': reduction,
                  'mass_reduction_target_fraction': mass_reduction_target,
                  'entry_speed_relative_difference': speed_difference,
                  'entry_speed_tolerance_fraction': entry_speed_tolerance,
                  'target_metrics_met': metrics_met,
                  'full_criterion_met': bool(all(checks.values())),
                  'checks': checks, 'unmet_checks': [key for key, passed in checks.items() if not passed],
                  'sampled_acceleration_check': {
                      'configured_bound_respected': (
                          candidate_limits['acceleration_estimate_violation_samples'] == 0
                          if candidate_limits['acceleration_estimate_violation_samples'] is not None else None),
                      'included_in_full_criterion': not elastic,
                      'reason': ('soft acceleration preference; exceedances are reported separately from hard safety compliance'
                                 if elastic else 'sampled finite-difference check; observed excess rejects success, absence of excess is not a continuous-time certificate'),
                  },
                  'interpretation': primary_tracking['reason'] + '; overall controller comparison: '
                      'HQP, constraints and secondary posture/damping can also differ, '
                      'so this does not isolate the causal effect of mass minimization'}
    return _json_safe({'definition': 'first compression-release cycle; precontact mass/speed from preceding sample',
                       'constraint_scope': 'saved samples only, not a continuous-time safety certificate',
                       'baseline': summaries['baseline'], 'candidate': summaries['candidate'],
                       'comparison': comparison})


def _json_safe(value):
    if isinstance(value, dict):
        return {str(key): _json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple, np.ndarray)):
        return [_json_safe(item) for item in value]
    if isinstance(value, (np.bool_, bool)):
        return bool(value)
    if isinstance(value, (np.integer, int)):
        return int(value)
    if isinstance(value, (np.floating, float)):
        return float(value) if np.isfinite(value) else None
    return value


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('baseline', type=Path)
    parser.add_argument('candidate', type=Path)
    parser.add_argument('--output', type=Path, help='write the complete JSON report here')
    parser.add_argument('--entry-speed-tolerance', type=float, default=0.05,
                        help='maximum relative entry-speed difference (default 0.05)')
    parser.add_argument('--mass-reduction-target', type=float, default=0.30,
                        help='required directional-mass reduction (default 0.30)')
    args = parser.parse_args(argv)
    for label, path in (('baseline', args.baseline), ('candidate', args.candidate)):
        if not path.is_file():
            parser.error(f'{label} log file not found: {path}')
    report = compare_runs(load_run(args.baseline), load_run(args.candidate),
                          entry_speed_tolerance=args.entry_speed_tolerance,
                          mass_reduction_target=args.mass_reduction_target)
    report['baseline']['log_path'] = str(args.baseline)
    report['candidate']['log_path'] = str(args.candidate)
    encoded = json.dumps(report, indent=2, allow_nan=False)
    if args.output:
        try:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(encoded + '\n', encoding='utf-8')
        except OSError as exc:
            parser.error(
                f'cannot write report to {args.output}: {exc}. '
                'Choose another writable --output path or omit --output to print JSON.')
    print(encoded)


if __name__ == '__main__':
    main()
