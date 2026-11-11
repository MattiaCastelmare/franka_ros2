#!/usr/bin/env python3
"""Is the INSTALLED workspace the documented source? (chapter 12 §2.4; protocol Phase 1). Container.

`gz_cbf_stack` reads every configuration file through `simexp.sim_config.default_paths`, i.e. from the
installed `share/` directories, and starts the installed programs. colcon copies `config/` (no
`--symlink-install`), so a source edit only reaches a simulation after a rebuild. This compares,
byte for byte, the source files the simulation depends on with their installed copies. It is a
STATE check: its result describes the container on the day it runs, not the code.

  1. configuration files (fr3_control, fr3_complete, conditions, scenarios, sim overrides)       (FACT)
  2. the sim programs and simexp modules                                                            (FACT)
  3. the stack nodes (cbf_safety_filter, real_time_distance, qddot_to_torque)                       (FACT)
"""
import filecmp
import glob
import os
import sys

sys.dont_write_bytecode = True
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _common6 as C6  # noqa: E402

R = C6.C.Report('check_installed (chapter 12 §2.4)')
INSTALL = os.environ.get('FS_INSTALL', '/ros2_ws/install')
REPO = C6.C.REPO
if not os.path.isdir(INSTALL):
    R.info(f'no install tree at {INSTALL}: skipped')
    sys.exit(R.finish())


def cmp(src_rel, inst_rel):
    s, i = os.path.join(REPO, src_rel), os.path.join(INSTALL, inst_rel)
    if not os.path.exists(i):
        return 'MISSING'
    return 'same' if filecmp.cmp(s, i, shallow=False) else 'DIFFERS'


share = lambda pkg, rel: f'{pkg}/share/{pkg}/{rel}'  # noqa: E731
cfg = [('franka_experiments/config/fr3_control.yaml', share('franka_experiments', 'config/fr3_control.yaml')),
       ('franka_experiments/config/fr3_complete.yaml', share('franka_experiments', 'config/fr3_complete.yaml')),
       ('franka_simulation_experiments/config/conditions.yaml',
        share('franka_simulation_experiments', 'config/conditions.yaml')),
       ('franka_simulation_experiments/config/scenarios.yaml',
        share('franka_simulation_experiments', 'config/scenarios.yaml')),
       ('franka_simulation/config/sim/perception_sim.yaml', share('franka_simulation', 'config/sim/perception_sim.yaml')),
       ('franka_simulation/config/sim/depth_adapter.yaml', share('franka_simulation', 'config/sim/depth_adapter.yaml'))]
res1 = {s: cmp(s, i) for s, i in cfg}
R.info('1. ' + '; '.join(f'{os.path.basename(k)}: {v}' for k, v in res1.items()))
lib = 'franka_simulation_experiments/lib/franka_simulation_experiments'
progs = [(f'franka_simulation_experiments/scripts/{n}.py', f'{lib}/{n}') for n in
         ('episode_supervisor', 'ground_truth_node', 'reflex_monitor', 'scenario_commander', 'emulated_perception',
          'run_gz_scenarios')]
progs += [(os.path.relpath(p, REPO), f'{lib}/simexp/{os.path.basename(p)}')
          for p in sorted(glob.glob(os.path.join(C6.SIMEXP_SCRIPTS, 'simexp', '*.py')))]
res2 = {s: cmp(s, i) for s, i in progs}
R.info(f'2. sim programs and simexp: {sum(v == "same" for v in res2.values())}/{len(res2)} same; differing: '
       f'{[os.path.basename(k) for k, v in res2.items() if v != "same"]}')
nodes = {}
for n in ('cbf_safety_filter', 'real_time_distance', 'qddot_to_torque'):
    hits = glob.glob(os.path.join(INSTALL, 'franka_experiments', '**', 'nodes', f'{n}.py'), recursive=True)
    nodes[n] = ('MISSING' if not hits else 'same' if filecmp.cmp(
        os.path.join(REPO, 'franka_experiments', 'franka_experiments', 'nodes', f'{n}.py'), hits[0], shallow=False)
        else 'DIFFERS')
R.info(f'3. stack nodes: {nodes}')
ok = all(v == 'same' for v in (*res1.values(), *res2.values(), *nodes.values()))
R.info('INSTALLED == SOURCE' if ok else 'INSTALLED != SOURCE: a simulation started now would NOT run the '
       'documented baseline (rebuild first; protocol Phase 1)')
R.fact('the check ran and compared every listed file', len(res1) == 6 and len(nodes) == 3)
sys.exit(R.finish())
