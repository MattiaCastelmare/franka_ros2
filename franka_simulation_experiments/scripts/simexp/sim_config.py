"""Per-run configuration of the Gazebo experiments: files, paths, provenance.

Everything a run needs that is DERIVED rather than shipped is generated here,
into the run directory, from the files it is derived from:

* the rt_torque_controller YAML, by franka_experiments' own
  ``launch_support.generate_rt_controllers_yaml`` (hardware gains, 1000 Hz,
  ``gazebo: true``) — the plugin's second ``<parameters>`` file (plan H4);
* the world SDF, by franka_simulation's ``utils/gz_world.py`` from the template,
  sim extrinsics and D455 depth intrinsics the ``scene`` selects (plan H8;
  see :data:`SCENES`);
* the merged perception YAML (fr3_complete.yaml + sim overrides + condition).

No rclpy import: the launch files and the tests both use it.
"""
from __future__ import annotations

import copy
import os
import subprocess
import sys
import time

import yaml

Q0 = (0.0, -0.70, 0.0, -2.35, 0.0, 1.57, 0.70)   # cbf_scenarios.Q0


def share(pkg: str) -> str:
    from ament_index_python.packages import get_package_share_directory
    return get_package_share_directory(pkg)


def franka_simulation_utils():
    """Import franka_simulation's installed ``utils`` package (gz_world, ...)."""
    from ament_index_python.packages import get_package_prefix
    lib = os.path.join(get_package_prefix('franka_simulation'), 'lib', 'franka_simulation')
    if lib not in sys.path:
        sys.path.insert(0, lib)
    import utils.gz_world as gz_world
    import utils.gz_robot_description as gz_robot_description
    return gz_world, gz_robot_description


def default_paths() -> dict:
    fs, fe = share('franka_simulation'), share('franka_experiments')
    return dict(
        world_template=os.path.join(fs, 'worlds', 'fr3_cbf_lab.sdf'),
        world_template_diag=os.path.join(fs, 'worlds', 'fr3_cbf_lab_diag.sdf'),
        camera_extrinsics_sim=os.path.join(fs, 'config', 'sim', 'camera_extrinsics_sim.yaml'),
        camera_extrinsics_diag=os.path.join(fs, 'config', 'sim',
                                            'camera_extrinsics_diag.yaml'),
        depth_intrinsics_diag=os.path.join(fs, 'config', 'sim',
                                           'depth_intrinsics_diag.yaml'),
        perception_sim=os.path.join(fs, 'config', 'sim', 'perception_sim.yaml'),
        depth_adapter=os.path.join(fs, 'config', 'sim', 'depth_adapter.yaml'),
        depth_intrinsics=os.path.join(fe, 'config', 'depth_intrinsics.yaml'),
        fr3_complete=os.path.join(fe, 'config', 'fr3_complete.yaml'),
        fr3_control=os.path.join(fe, 'config', 'fr3_control.yaml'),
    )


#: The scenes a run can be built on: which world template, which camera
#: calibration and depth intrinsics it is generated from, and whether the
#: furniture stays. The lab was re-arranged and the camera moved between the two
#: recordings, so the numbers are never shared: `rig` is the lab of
#: rosbag/irim1_raw, `diag` the DIAG robotics lab of datasets/experiments_irim2.zip
#: (2026-10-09), and an old run stays reproducible on the scene it used.
#: `*_bare` is the same camera over an empty floor (plan H9).
SCENES = {
    'rig':        dict(template='world_template', extrinsics='camera_extrinsics_sim',
                       intrinsics='depth_intrinsics', furniture='rig'),
    'bare':       dict(template='world_template', extrinsics='camera_extrinsics_sim',
                       intrinsics='depth_intrinsics', furniture='bare'),
    'diag':       dict(template='world_template_diag',
                       extrinsics='camera_extrinsics_diag',
                       intrinsics='depth_intrinsics_diag', furniture='rig'),
    'diag_bare':  dict(template='world_template_diag',
                       extrinsics='camera_extrinsics_diag',
                       intrinsics='depth_intrinsics_diag', furniture='bare'),
}


def scene_paths(scene: str, paths: dict | None = None) -> dict:
    """The files ``scene`` is built from, and whether its furniture stays.

    Everything that has to agree with the simulated camera reads it from here:
    the world generator (the Gazebo sensor pose and FOV) and the launch files
    (real_time_distance's ``camera_extrinsics_yaml``). Those two must never be
    given different scenes' calibrations.
    """
    try:
        s = SCENES[scene]
    except KeyError:
        raise ValueError(f'scene={scene!r}: expected one of {sorted(SCENES)}') from None
    P = paths or default_paths()
    return dict(template=P[s['template']], extrinsics=P[s['extrinsics']],
                intrinsics=P[s['intrinsics']], furniture=s['furniture'], name=scene)


def sim_runs_root() -> str:
    """Where SIM outputs go: never next to hardware logs or bags (ground rule 5).

    ``$FRANKA_SIM_RUNS_DIR``, else ``sim_runs/`` in this package's SOURCE tree
    (under the container mount, so visible on the host; git-ignored), else
    ``~/franka_sim_runs``.
    """
    env = os.environ.get('FRANKA_SIM_RUNS_DIR')
    if env:
        return env
    here = os.path.dirname(os.path.realpath(__file__))
    for cand in (os.path.dirname(os.path.dirname(here)),
                 '/ros2_ws/src/franka_simulation_experiments'):
        if os.path.isfile(os.path.join(cand, 'package.xml')) and \
                os.path.isdir(os.path.join(cand, 'scripts')):
            return os.path.join(cand, 'sim_runs')
    return os.path.join(os.path.expanduser('~'), 'franka_sim_runs')


def make_run_dir(root: str | None = None, name: str = 'run') -> str:
    root = root or sim_runs_root()
    d = os.path.join(root, time.strftime('%Y%m%d'), f'{time.strftime("%H%M%S")}_{name}')
    os.makedirs(d, exist_ok=True)
    return d


def rt_controllers_yaml(run_dir: str, *, arm_id: str = 'fr3') -> str:
    """The plugin's rt_torque_controller YAML, written by the stack's own generator."""
    from franka_experiments.utils.config import load_launch_defaults
    from franka_experiments.utils.launch_support import generate_rt_controllers_yaml
    ld, _ = load_launch_defaults()
    content = generate_rt_controllers_yaml(
        is_real=True, arm_id=arm_id, controller_type='torque',
        torque_command_topic=str(ld.get('torque_command_topic', 'torque_cmd')),
        gazebo='true', lpf_alpha=float(ld.get('lpf_alpha', 1.0)),
        tau_max_scale=float(ld.get('tau_max_scale', 1.0)))
    path = os.path.join(run_dir, 'rt_controllers_gazebo.yaml')
    with open(path, 'w') as f:
        f.write(content)
    return path


def world_sdf(run_dir: str, *, scene: str = 'rig', depth_rate_hz: float = 90.0,
              extrinsics_path: str | None = None, extra_models=(), extra_world_plugins=(),
              paths: dict | None = None) -> str:
    gz_world, _ = franka_simulation_utils()
    P = paths or default_paths()
    S = scene_paths(scene, P)
    with open(S['template']) as f:
        template = f.read()
    sdf = gz_world.configure_world(
        template, extrinsics_path=extrinsics_path or S['extrinsics'],
        intrinsics=gz_world.load_depth_intrinsics(S['intrinsics']),
        depth_rate_hz=depth_rate_hz, scene=S['furniture'], extra_models=extra_models,
        extra_world_plugins=extra_world_plugins)
    path = os.path.join(run_dir, 'world.sdf')
    with open(path, 'w') as f:
        f.write(sdf)
    return path


def deep_merge(dst: dict, src: dict) -> dict:
    for k, v in (src or {}).items():
        if isinstance(v, dict) and isinstance(dst.get(k), dict):
            deep_merge(dst[k], v)
        else:
            dst[k] = copy.deepcopy(v)
    return dst


def perception_yaml(run_dir: str, condition_overrides: dict | None = None,
                    paths: dict | None = None) -> str:
    """fr3_complete.yaml ⊕ config/sim/perception_sim.yaml ⊕ the condition's keys."""
    P = paths or default_paths()
    with open(P['fr3_complete']) as f:
        cfg = yaml.safe_load(f)
    with open(P['perception_sim']) as f:
        deep_merge(cfg, yaml.safe_load(f) or {})
    deep_merge(cfg, condition_overrides or {})
    path = os.path.join(run_dir, 'fr3_complete_sim.yaml')
    with open(path, 'w') as f:
        yaml.safe_dump(cfg, f, sort_keys=False)
    return path


def git_provenance(start: str | None = None) -> dict:
    """sha / branch / dirty of the source tree this module was loaded from."""
    candidates = [start or os.path.dirname(os.path.abspath(__file__)), '/ros2_ws/src',
                  os.getcwd()]

    def _git(cwd, *a):
        try:
            out = subprocess.run(['git', *a], cwd=cwd, capture_output=True, text=True,
                                 timeout=5)
            return out.stdout.strip() if out.returncode == 0 else None
        except Exception:                                   # noqa: BLE001
            return None
    for cwd in candidates:
        if cwd and os.path.isdir(cwd):
            sha = _git(cwd, 'rev-parse', 'HEAD')
            if sha:
                return {'sha': sha, 'branch': _git(cwd, 'rev-parse', '--abbrev-ref', 'HEAD'),
                        'dirty': bool(_git(cwd, 'status', '--porcelain'))}
    return {'sha': None, 'branch': None, 'dirty': None}


def depth_adapter_yaml(run_dir: str, overrides: dict | None = None,
                       paths: dict | None = None) -> str:
    """config/sim/depth_adapter.yaml with this run's stage overrides (e.g. range_noise)."""
    P = paths or default_paths()
    with open(P['depth_adapter']) as f:
        cfg = yaml.safe_load(f)
    deep_merge(cfg['/**']['ros__parameters'], overrides or {})
    path = os.path.join(run_dir, 'depth_adapter.yaml')
    with open(path, 'w') as f:
        yaml.safe_dump(cfg, f, sort_keys=False)
    return path
