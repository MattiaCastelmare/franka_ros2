#!/usr/bin/env python3
"""Conditions: what each experiment arm really changes (chapter 12 §6). Host, read-only.

Production code: `simexp.conditions.load/resolve/params_arg`, `simexp.batch.CONDITIONS`,
`simexp.closed_loop.rate_verdict`, and `C3.effective_params('sim:<c>')`, which executes the launch
helpers of `torque_control_stack` / `gz_cbf_stack` on the source YAML (Stage 3 helper).

  1. hw_default -> sim:full differs in exactly qp_rate_hz 300 -> 100 and enable_go_around on -> off  (NUM)
  2. full -> minimal: the number of effective parameters that change, and the minimal flags that are
     NO-OPS (minimal sets them, but full already has the same value)                              (NUM)
  3. F-6-2: params_arg writes obstacle_row_osqp_eps_rel as '1e-05'; yaml.safe_load (the parser of
     _cbf_param_overrides and of launch_ros) returns a str; yaml.dump writes it back unquoted     (FACT)
  4. F-6-4: the Phase 5 grid conditions (batch.CONDITIONS) have no qp_rate_hz in their cbf_params,
     so closed_loop's rate gate returns UNKNOWN for every one of their cells; they also run the
     stack default 300 Hz with go-around ON, unlike full/minimal                                (FACT)
  5. precedence: global cbf < condition cbf < --set; --set may only replace keys already present  (FACT)
"""
import os
import sys

sys.dont_write_bytecode = True
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _common6 as C6  # noqa: E402
import yaml  # noqa: E402
from simexp import batch, closed_loop, conditions  # noqa: E402

C3 = C6.C3
R = C6.C.Report('check_conditions (chapter 12 §6)')
cfg = conditions.load()
names = list(cfg['conditions'])
R.info(f'{len(names)} conditions: {names}')

# 1
hw, full = C3.effective_params('hw_default'), C3.effective_params('sim:full')
d1 = {k: (hw[k], full[k]) for k in hw if hw[k] != full[k]}
R.fact(f'1. hw_default -> sim:full changes {d1}',
       d1 == {'qp_rate_hz': (300.0, 100.0), 'enable_go_around': (True, False)})

# 2
mini = C3.effective_params('sim:minimal')
d2 = sorted(k for k in full if full[k] != mini[k])
R.close('2. full -> minimal: number of effective parameters that change', len(d2), 19, 0, kind='NUM')
R.info('2. changed: ' + ', '.join(f'{k} {full[k]!r}->{mini[k]!r}' for k in d2))
mc = conditions.resolve('minimal')['cbf_params']
noop = sorted(k for k, v in mc.items() if k in full and full[k] == v and k not in d2)
R.fact(f'2. minimal sets {len(noop)} parameters to the value full already has (no-ops): {noop}',
       {'enable_velocity_feedforward', 'enable_sensor_range_uncertainty'} <= set(noop))

# 3 F-6-2
pa = conditions.resolve('minimal_l1')['stack_args']['cbf_param_overrides']
kv = dict(x.split('=', 1) for x in pa.split(';'))
v_txt = kv['obstacle_row_osqp_eps_rel']
v_launch = yaml.safe_load(v_txt)
hit = [n for n in names if 'obstacle_row_osqp_eps_rel' in (conditions.resolve(n).get('cbf_params') or {})]
R.fact(f"3. F-6-2: params_arg writes {v_txt!r}; yaml.safe_load gives {type(v_launch).__name__} {v_launch!r}; "
       f"yaml.dump writes {yaml.dump({'p': v_launch})!r}; C3 effective type "
       f"{type(C3.effective_params('sim:minimal_l1')['obstacle_row_osqp_eps_rel']).__name__}; {len(hit)} conditions",
       v_txt == '1e-05' and isinstance(v_launch, str) and yaml.dump({'p': v_launch}) == 'p: 1e-05\n'
       and len(hit) == 16)
R.fact("3. the spec declares it a float and the node calls float() on it (utils/config.py, cbf_safety_filter.py)",
       "'obstacle_row_osqp_eps_rel': ('float'" in C6.src('franka_experiments/franka_experiments/utils/config.py')
       and 'float(self.P.obstacle_row_osqp_eps_rel)' in C6.src(
           'franka_experiments/franka_experiments/nodes/cbf_safety_filter.py'))
R.fact("3. a dotted literal would be a float: yaml.safe_load('1.0e-05') is float",
       isinstance(yaml.safe_load('1.0e-05'), float))

# 4 F-6-4
tgt = {c: (conditions.resolve(c).get('cbf_params') or {}).get('qp_rate_hz') for c in batch.CONDITIONS}
eff = {c: (C3.effective_params(f'sim:{c}')['qp_rate_hz'], C3.effective_params(f'sim:{c}')['enable_go_around'])
       for c in batch.CONDITIONS}
R.fact(f'4. F-6-4: batch.CONDITIONS {batch.CONDITIONS}: qp_rate_hz in cbf_params {tgt}; effective (rate, go_around) {eff}',
       all(v is None for v in tgt.values()) and all(v == (300.0, True) for v in eff.values()))
R.fact('4. closed_loop.rate_verdict(achieved, None) is (None, None): the gate cannot fail such a cell',
       closed_loop.rate_verdict(290.0, None) == (None, None) and closed_loop.rate_verdict(94.0, 100.0)[0] is False
       and closed_loop.rate_verdict(95.0, 100.0)[0] is True)
R.fact("4. summarize_run takes the target from the manifest's condition.cbf_params.qp_rate_hz only",
       "((man.get('condition') or {}).get('cbf_params') or {}).get('qp_rate_hz')"
       in C6.src('franka_simulation_experiments/scripts/simexp/closed_loop.py'))

# 5
fk = next(iter(cfg['cbf']))                       # a global cbf flag
r_set = conditions.resolve('full', {fk: not bool(conditions.resolve('full')['toggles'][fk]),
                                    'qp_rate_hz': 77.0})
R.fact(f'5. --set overrides a global cbf flag ({fk}) and a cbf_params key the condition already has '
       '(qp_rate_hz=77 reaches cbf_params)',
       r_set['cbf_params'].get('qp_rate_hz') == 77.0
       and r_set['toggles'][fk] != conditions.resolve('full')['toggles'][fk])
def _raises(over):
    try:
        conditions.resolve('full', over)
        return False
    except KeyError:
        return True
R.fact('5. --set of a fr3_control.yaml key that the condition does not list (k0_cbf on full) raises: '
       '--set cannot add a YAML-only parameter', _raises({'k0_cbf': 7.5}) and _raises({'no_such_flag_xyz': 1}))
sys.exit(R.finish())
