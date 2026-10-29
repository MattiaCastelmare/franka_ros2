"""Run a list of experiments one after the other (no Gazebo window).

    cd /ros2_ws/src/fr3_student_sim
    ros2 run fr3_student_sim run_batch config/batch_example.yaml

The batch file::

    defaults:                       # launch options for every run
      controller: my_controller.py
      duration: 16
      stop_at_detachment: true
    runs:
      - name: slow_circle           # also the prefix of the log file
        task: {circle: {period: 12.0}}             # merged into config/task.yaml
      - name: soft_wall
        task: {my_gain: 2.0}                       # any key your controller reads
        wall: {stiffness: 1500}                    # merged into config/wall.yaml
        launch: {noise_dq: 0.001}                  # extra launch options for this run

Each run gets its own task/wall YAML in ``runs/batch_configs/``; the logs go to
``runs/`` as usual (both in the fr3_student_sim folder). At the end a table of the ground-truth contact facts is printed.
"""
from __future__ import annotations

import argparse
import copy
import glob
import os
import subprocess
import sys

import yaml
from ament_index_python.packages import get_package_share_directory

from fr3_student_sim.paths import runs_dir
from fr3_student_sim.plot_run import load_run, metrics


def merge(base: dict, over: dict) -> dict:
    """Recursive dict merge: ``over`` wins, nested dicts are merged."""
    out = copy.deepcopy(base)
    for k, v in (over or {}).items():
        out[k] = merge(out[k], v) if isinstance(v, dict) and isinstance(out.get(k), dict) else v
    return out


def _yaml(path):
    with open(path) as f:
        return yaml.safe_load(f) or {}


def main():
    ap = argparse.ArgumentParser(description='Run a list of fr3_student_sim experiments.')
    ap.add_argument('batch', help='batch YAML file')
    ap.add_argument('--gui', action='store_true', help='show the Gazebo window for each run')
    a = ap.parse_args()

    share = get_package_share_directory('fr3_student_sim')
    batch = _yaml(a.batch)
    defaults = dict(batch.get('defaults', {}))
    task0 = _yaml(defaults.pop('task_config', os.path.join(share, 'config', 'task.yaml')))
    wall0 = _yaml(defaults.pop('wall_config', os.path.join(share, 'config', 'wall.yaml')))
    logs = runs_dir()
    cfg_dir = os.path.join(logs, 'batch_configs')
    os.makedirs(cfg_dir, exist_ok=True)

    results = []
    for i, run in enumerate(batch['runs']):
        name = run['name']
        task_path = os.path.join(cfg_dir, f'{name}_task.yaml')
        wall_path = os.path.join(cfg_dir, f'{name}_wall.yaml')
        with open(task_path, 'w') as f:
            yaml.safe_dump(merge(task0, run.get('task')), f, sort_keys=False)
        with open(wall_path, 'w') as f:
            yaml.safe_dump(merge(wall0, run.get('wall')), f, sort_keys=False)
        opts = merge(defaults, run.get('launch'))
        opts.update({'run_name': name, 'task_config': task_path, 'wall_config': wall_path,
                     'log_dir': logs,
                     'gui': 'true' if a.gui else 'false'})
        cmd = ['ros2', 'launch', 'fr3_student_sim', 'student.launch.py'] + [
            f'{k}:={str(v).lower() if isinstance(v, bool) else v}' for k, v in opts.items()]
        print(f'\n[{i + 1}/{len(batch["runs"])}] {name}', flush=True)
        before = set(glob.glob(os.path.join(logs, f'{name}_*.npz')))
        proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        for line in proc.stdout:          # show only what the runner says
            if 'student_runner' in line or 'Traceback' in line or 'Error' in line:
                sys.stdout.write('   ' + line.split(']: ', 1)[-1])
        proc.wait()
        new = sorted(set(glob.glob(os.path.join(logs, f'{name}_*.npz'))) - before)
        results.append((name, metrics(load_run(new[-1])) if new else None))

    keys = ['stop_reason', 't_contact_s', 'entry_speed_mps', 'peak_force_N',
            'peak_penetration_mm', 'contact_duration_s', 'max_hand_depth_mm', 'missed_samples_pct']
    print('\n' + f'{"run":24s}' + ''.join(f'{k[:16]:>18s}' for k in keys))
    for name, m in results:
        if m is None:
            print(f'{name:24s}  NO LOG (see the messages above)')
            continue
        cells = []
        for k in keys:
            v = m.get(k, '-')
            cells.append(f'{v:18.4g}' if isinstance(v, float) else f'{str(v)[:17]:>18s}')
        print(f'{name:24s}' + ''.join(cells))


if __name__ == '__main__':
    main()
