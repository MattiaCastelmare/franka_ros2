#!/usr/bin/env python3
"""Effective `cbf_safety_filter` parameters per launch configuration (chapters 06 and 07).

Prints, for the obstacle-barrier and nominal-shaping keys, the value the node ends up with in:
  yaml        fr3_control.yaml alone (what a reader of the YAML would assume)
  hw          torque_control_stack.launch.py with launch_defaults.yaml (a default hardware launch)
  sim:kf      gz_cbf_stack with condition `kf` (the default of run_gz_scenarios.py --conditions)
  sim:min_l1  gz_cbf_stack with condition `minimal_l1` (the reduced HOCBF arm)
and checks the facts the chapters rely on (FACT). The overrides are computed by the launch file's
own helper functions (see _common3.py); nothing is copied by hand.

Needs: Python 3, PyYAML, numpy. Host. Read-only.
"""
import os
import sys

sys.dont_write_bytecode = True
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _common3 as C3  # noqa: E402

C = C3.C
R = C.Report('effective_params (chapters 06-07: what is actually ON)')

CONFIGS = [('yaml', 'yaml'), ('hw', 'hw_default'), ('sim:kf', 'sim:kf'), ('sim:min_l1', 'sim:minimal_l1')]
KEYS = [
    # barrier core
    'd_safe', 'k0_cbf', 'k1_cbf', 'cbf_obstacle_horizon', 'min_confidence', 'cbf_min_leverage',
    'cbf_h_recovery_alpha', 'cbf_hdot_filter_alpha', 'cbf_update_rate_hz', 'qp_rate_hz',
    # obstacle velocity
    'obstacle_velocity_enabled', 'obstacle_velocity_source', 'obstacle_velocity_residual_floor',
    'obstacle_velocity_residual_floor_gap', 'obstacle_velocity_median', 'obstacle_velocity_median_s',
    'obstacle_velocity_min_frames', 'obstacle_velocity_min_span_s', 'obstacle_input_rate_hz',
    'obstacle_velocity_fast_trust', 'obstacle_velocity_track_deadband', 'obstacle_velocity_max',
    'obstacle_velocity_normal_rot_max', 'obstacle_velocity_identity_jump',
    'enable_vobs_in_hdot', 'vobs_hdot_max',
    # margins on h
    'enable_velocity_feedforward', 'enable_uncertainty_margin', 'uncertainty_k_sigma',
    'link_speed_reaction_s', 'uncertainty_margin_alpha', 'enable_latency_compensation',
    'latency_t_blind', 'latency_k_sigma', 'latency_margin_max', 'enable_sensor_range_uncertainty',
    'enable_velocity_standoff', 'velocity_standoff_time_s', 'velocity_standoff_max',
    'margin_gate_on_approach', 'margin_gate_ref_speed', 'enable_evidence_margin',
    # extra terms / mechanisms
    'enable_curvature_term', 'curvature_obstacle_radius', 'enable_obstacle_memory',
    'enable_zone_ladder', 'retreat_excess_speed_max', 'enable_weighted_slack',
    'obstacle_slack_mode', 'rho_slack', 'retreat_cap_enabled', 'link_speed_rows_enabled',
    'ssb_rows_enabled', 'iso_enabled',
    # nominal shaping inside the filter
    'state_governor_enabled', 'cbf_tangential_gain', 'enable_go_around', 'enable_lateral_evasion',
    'enable_outrun_evasion', 'enable_livelock_escape', 'k_brake',
]

tables = {}
for name, cfg in CONFIGS:
    tables[name] = C3.effective_params(cfg)

w = max(len(k) for k in KEYS)
print(f'{"key":{w}s} | ' + ' | '.join(f'{n:>10s}' for n, _ in CONFIGS))
for k in KEYS:
    row = []
    for n, _ in CONFIGS:
        v = tables[n].get(k, '—')
        row.append(f'{str(v):>10s}')
    mark = '' if len({str(tables[n].get(k)) for n, _ in CONFIGS}) == 1 else '   *'
    print(f'{k:{w}s} | ' + ' | '.join(row) + mark)
print('(* = differs between configurations)')

hw, y, kf, ml1 = tables['hw'], tables['yaml'], tables['sim:kf'], tables['sim:min_l1']
nine = ['enable_lateral_evasion', 'enable_livelock_escape', 'enable_uncertainty_margin',
        'enable_zone_ladder', 'enable_vobs_in_hdot', 'enable_velocity_standoff']
R.fact('03 §5.1: six enable_* flags are false in the YAML and true in a default hardware launch',
       all((not y[k]) and hw[k] is True for k in nine), str({k: (y[k], hw[k]) for k in nine}))
R.fact('03 §5.1: obstacle_velocity_source residual -> tracker; normal guard 0 -> 0.15; identity jump 0 -> 0.10',
       (y['obstacle_velocity_source'], hw['obstacle_velocity_source']) == ('residual', 'tracker')
       and (y['obstacle_velocity_normal_rot_max'], hw['obstacle_velocity_normal_rot_max']) == (0.0, 0.15)
       and (y['obstacle_velocity_identity_jump'], hw['obstacle_velocity_identity_jump']) == (0.0, 0.10))
off_everywhere = ['enable_velocity_feedforward', 'enable_curvature_term', 'enable_evidence_margin',
                  'enable_obstacle_memory', 'enable_sensor_range_uncertainty', 'enable_weighted_slack',
                  'enable_outrun_evasion', 'ssb_rows_enabled', 'iso_enabled']
R.fact('mechanisms OFF in all four configurations (implemented, not shipped)',
       all(not t[k] for t in tables.values() for k in off_everywhere), ', '.join(off_everywhere))
R.fact('on in yaml and hw (no launch flag), off in minimal_l1: go-around, state governor, rate caps, '
       'tangential bias',
       all(y[k] and hw[k] and not ml1[k] for k in ('enable_go_around', 'state_governor_enabled',
                                                     'retreat_cap_enabled', 'link_speed_rows_enabled'))
       and hw['cbf_tangential_gain'] > 0 and ml1['cbf_tangential_gain'] == 0.0)
R.fact('latency compensation: on in YAML and hw, off in minimal_l1',
       y['enable_latency_compensation'] and hw['enable_latency_compensation']
       and not ml1['enable_latency_compensation'])
diff_kf = sorted(k for k in hw if hw[k] != kf[k])
R.fact('sim:kf (batch default) is identical to the hardware default', not diff_kf,
       f'{len(diff_kf)} keys differ: {diff_kf}')
R.fact('gains: d_safe 0.10 m, k0 25, k1 10.5 in every configuration',
       all((t['d_safe'], t['k0_cbf'], t['k1_cbf']) == (0.1, 25.0, 10.5) for t in tables.values()))
R.fact('obstacle_input_rate_hz = 90 from the camera profile 848x480x90 (hw and sim)',
       hw['obstacle_input_rate_hz'] == 90.0 and kf['obstacle_input_rate_hz'] == 90.0)
R.fact('minimal_l1 slack mode row_l1, every other configuration family slack',
       ml1['obstacle_slack_mode'] == 'row_l1'
       and all(t['obstacle_slack_mode'] == 'family' for n, t in tables.items() if n != 'sim:min_l1'))
sys.exit(R.finish())
