"""Run one Gazebo cell (a fresh Gazebo per cell) and digest it. No rclpy.

``run_cell`` launches gz_cbf_stack.launch.py in its own process group with a
finite ``duration_s``. The episode supervisor ends the launch, and the group is
torn down whatever happens (simexp.procs).

``digest_cell`` reads the run back: the shipped digests (experiment_summary.py,
smoothness_report.py, both run UNCHANGED as subprocesses), the reflex summary,
the RTF, the CBFDIAG ratio, the joint-space command/realised projection, and
the pentagon tracking error when there is one. Everything is labelled SIM.
"""
from __future__ import annotations

import json
import os
import subprocess
import sys
import time

from . import analysis as A
from . import procs


def experiments_src() -> str:
    for cand in ('/ros2_ws/src/franka_experiments',
                 os.path.normpath(os.path.join(os.path.dirname(os.path.realpath(__file__)),
                                               '..', '..', '..', 'franka_experiments'))):
        if os.path.isdir(os.path.join(cand, 'scripts')):
            return cand
    raise FileNotFoundError('franka_experiments source tree not found')


def run_cell(run_dir: str, *, condition: str, motion: str = '', scenario: str = '',
             seed: int = 0, duration_s: float | None = None, perception: bool | str = True,
             scene: str = '', sets: dict | None = None, timeout_s: float = 600.0,
             domain_id: int = 77, extra_args: dict | None = None) -> dict:
    """Empty motion / scene and duration_s None = the scenario's own (gz_cbf_stack).

    ``perception``: True / False, or the gz_cbf_stack mode string
    ('true' | 'false' | 'emulated').
    """
    os.makedirs(run_dir, exist_ok=True)
    if isinstance(perception, bool):
        perception = 'true' if perception else 'false'
    args = {'condition': condition, 'motion': motion, 'scenario': scenario, 'seed': str(seed),
            'duration_s': '' if duration_s is None else str(duration_s),
            'perception': str(perception), 'scene': scene, 'run_dir': run_dir}
    if sets:
        args['set'] = ';'.join(f'{k}={v}' for k, v in sets.items())
    args.update(extra_args or {})
    # `ros2 launch` refuses `name:=` with nothing after it: empty = the default.
    L = procs.Launch(['franka_simulation_experiments', 'gz_cbf_stack.launch.py',
                      *[f'{k}:={v}' for k, v in args.items() if str(v) != '']],
                     os.path.join(run_dir, 'launch.log'),
                     # every node's own log next to the run (real_time_distance
                     # logs its status lines to file, not to the screen)
                     env={'ROS_DOMAIN_ID': str(domain_id),
                          'ROS_LOG_DIR': os.path.join(run_dir, 'ros_log')})
    t0 = time.time()
    L.start()
    timed_out = False
    try:
        while L.alive():
            if time.time() - t0 > timeout_s:
                timed_out = True
                break
            time.sleep(1.0)
    finally:
        L.stop()
    left = procs.leftovers(L.partition)
    return {'run_dir': run_dir, 'args': args, 'wall_s': time.time() - t0,
            'timed_out': timed_out, 'returncode': L.proc.returncode, 'leftovers': left}


def _run_script(cmd, out_path, cwd, env=None) -> int:
    with open(out_path, 'w') as f:
        p = subprocess.run(cmd, cwd=cwd, stdout=f, stderr=subprocess.STDOUT,
                           env=dict(os.environ, **(env or {})), timeout=600)
    return p.returncode


def digest_cell(run_dir: str, *, shipped_digests: bool = True) -> dict:
    f = A.find_files(run_dir)
    man = A.load_json(f['manifest']) or {}
    ep = man.get('episode') or {}
    out = {'SIM': True, 'plant': 'gazebo', 'run_dir': run_dir, 'manifest': man}
    reflex = A.load_json(f['reflex']) if f['reflex'] else None
    out['reflex'] = None if reflex is None else {
        k: reflex.get(k) for k in ('verdict', 'n_trips', 'counts', 'worst', 'n_samples',
                                   'sim_span_s')}
    out['rtf'] = None if reflex is None else reflex.get('rtf')
    if f['launch_log']:
        rows = A.parse_cbfdiag(f['launch_log'])
        out['cbfdiag'] = {'qdd_ratio_0.5': A.qdd_ratio(rows, 0.5),
                          'qdd_ratio_0.1': A.qdd_ratio(rows, 0.1),
                          'slack_max': A.slack_max(rows)}
    if f['bag'] and ep.get('t_start_sim') is not None:
        out['qdd_projection'] = A.qdd_projection(f['bag'], ep['t_start_sim'], ep['t_end_sim'])
    if f['pentagon_csv']:
        out['pentagon_tracking'] = A.pentagon_tracking(
            f['pentagon_csv'], t_from=ep.get('t_start_sim'))
    if shipped_digests:
        src = experiments_src()
        env = {'PYTHONPATH': src + os.pathsep + os.environ.get('PYTHONPATH', '')}
        if f['logger_dir']:
            out['experiment_summary_rc'] = _run_script(
                [sys.executable, os.path.join(src, 'scripts', 'experiment_summary.py'),
                 f['logger_dir']], os.path.join(run_dir, 'experiment_summary.txt'), src, env)
        if f['bag']:
            out['smoothness_report_rc'] = _run_script(
                [sys.executable, os.path.join(src, 'scripts', 'smoothness_report.py'), f['bag']],
                os.path.join(run_dir, 'smoothness_report.txt'), src, env)
            out['smoothness_verdicts'] = _verdicts(os.path.join(run_dir, 'smoothness_report.txt'))
    with open(os.path.join(run_dir, 'digest.json'), 'w') as fh:
        json.dump(out, fh, indent=1, default=float)
    return out


def _verdicts(path: str) -> dict:
    out = {}
    with open(path, errors='replace') as f:
        for line in f:
            s = line.strip()
            if s.startswith('[') and ']' in s:
                tag, rest = s[1:].split(']', 1)
                out[rest.split(':')[0].strip()] = tag.strip()
            elif s.startswith('overall:'):
                out['overall'] = s.split(':', 1)[1].strip()
            elif s.startswith('EE tracking error'):
                out['ee_tracking_cm'] = s.split(':', 1)[1].strip()
    return out
