"""Container-side helpers for the Stage 3 scripts that drive the REAL obstacle-row builder.

Builds what `cbf_safety_filter.__init__` builds (L150-239 at the baseline), from the source tree:
  * `CBFKinematics` on the hand-less URDF (`build_urdf_no_hand`), with the control points
    configured from `config/fr3_complete.yaml` (`configure_control_points`);
  * joint limits as the node derives them (`load_franka_joint_limits`, `qddot_max_abs`,
    `qddot_accel_limits`, the `FR3_VEL_Q_REF_*` clip);
  * `ConstraintBuilder(P, kin, ..., control_point_jacobian=kin.control_point_jacobian)` WITHOUT
    the optional self-collision / singularity / SSB builders: these scripts look at obstacle rows
    (group G_OBS) only, which those builders never touch.
and turns a `MultiLinkDistance` into `ObstacleSnap` items with a transcription of
`cbf_safety_filter._on_distances` L840-885 (the `Obstacle(...)` constructor call and the
`_latency_fields` / `_range_field` helpers). The node itself cannot be instantiated without a
running ROS graph, so this mapping is copied; `check_mapping()` verifies by `ast` that the node
still passes exactly these keyword names.

P is `_common3.params_namespace(config)`: the YAML `params` block plus the launch overrides,
which is what `load_cbf_config` declares (every key comes from the YAML; a missing one is a
startup failure).

Needs the ROS 2 environment (franka_msgs, builtin_interfaces, xacro, franka_description),
Pinocchio, numpy, PyYAML: run inside the `franka_ros2` container.
"""
import ast
import os

import numpy as np

import _common3 as C3

C = C3.C
C.use_source_package()

import pinocchio as pin  # noqa: E402
from franka_experiments.utils.cbf_hard_limits import FR3_VEL_Q_REF_LOWER, FR3_VEL_Q_REF_UPPER  # noqa: E402
from franka_experiments.utils.cbf_state_rows import (  # noqa: E402
    FR3_JOINT_KEYS, G_OBS, ConstraintBuilder, JointSnap, Obstacle, ObstacleSnap)
from franka_experiments.utils.config import load_franka_joint_limits  # noqa: E402
from franka_experiments.utils.kinematics import CBFKinematics, build_urdf_no_hand  # noqa: E402
from franka_experiments.utils.perception_msgs import labelled_links  # noqa: E402

FILTER_SRC = os.path.join(C.PKG_SRC, 'franka_experiments', 'nodes', 'cbf_safety_filter.py')
OBSTACLE_KWARGS = {'link', 'd', 'pr', 'ph', 'conf', 'v_vec', 'frames_seen', 'vel_cov', 'track_id',
                   'cp_label'}


class Log:
    """Stand-in for the rclpy logger: records messages, ignores throttling."""

    def __init__(self):
        self.lines = []

    def _rec(self, lvl, msg, **_):
        self.lines.append((lvl, str(msg)))

    def info(self, m, **k):
        self._rec('info', m, **k)

    def warn(self, m, **k):
        self._rec('warn', m, **k)

    def error(self, m, **k):
        self._rec('error', m, **k)


def robot_cfg():
    return C.load_yaml(os.path.join(C.CONFIG, 'fr3_complete.yaml'))


def make_kin():
    p = build_urdf_no_hand()
    try:
        model = pin.buildModelFromUrdf(p)
    finally:
        os.unlink(p)
    kin = CBFKinematics(model)
    cfg = robot_cfg()
    kin.configure_control_points(cfg['robot'], cfg['distance'])
    return kin


def make_builder(P, kin=None, log=None):
    kin = kin or make_kin()
    log = log or Log()
    jl = load_franka_joint_limits(FR3_JOINT_KEYS)
    qdd_cap = np.minimum(jl['decel_max'], P.qddot_max_abs)
    brake = qdd_cap.copy()
    acc = getattr(P, 'qddot_accel_limits', None)
    if acc:
        acc = np.asarray(acc, dtype=np.float64).reshape(-1)
        qdd_cap = np.minimum(acc, P.qddot_max_abs)
        brake = np.minimum(brake, qdd_cap)
    b = ConstraintBuilder(P, kin, q_min=np.maximum(jl['q_min'], FR3_VEL_Q_REF_LOWER),
                          q_max=np.minimum(jl['q_max'], FR3_VEL_Q_REF_UPPER),
                          acc_lb=-qdd_cap, acc_ub=qdd_cap, logger=log, brake_acc=brake,
                          qdot_max=jl['qdot_max'], control_point_jacobian=kin.control_point_jacobian)
    return b, kin, log


def _v3(p):
    return np.array([p.x, p.y, p.z], dtype=np.float64)


def obstacles_from_msg(msg):
    """Transcription of cbf_safety_filter._on_distances L840-885 (valid entries only)."""
    out = []
    for cp_label, ld in labelled_links(msg):
        if not ld.valid:
            continue
        extra = {}
        if hasattr(ld, 'obstacle_acceleration'):
            extra.update(a_vec=_v3(ld.obstacle_acceleration),
                         pos_cov=np.asarray(ld.position_covariance, dtype=np.float64).reshape(3, 3),
                         pv_cov=np.asarray(ld.position_velocity_covariance, dtype=np.float64).reshape(3, 3))
        if hasattr(ld, 'range_m') and ld.range_m > 0.0:
            extra.update(range_m=float(ld.range_m))
        out.append(Obstacle(link=ld.robot_link_name, d=float(ld.distance), pr=_v3(ld.closest_point_robot),
                            ph=_v3(ld.closest_point_human), conf=float(ld.confidence),
                            v_vec=_v3(ld.obstacle_velocity), frames_seen=int(ld.frames_seen),
                            vel_cov=np.asarray(ld.velocity_covariance, dtype=np.float64).reshape(3, 3),
                            track_id=int(ld.track_id), cp_label=cp_label, **extra))
    return tuple(out)


def check_mapping() -> bool:
    """True if the node's Obstacle(...) call in _on_distances uses exactly OBSTACLE_KWARGS
    (plus the two ** helper expansions) — i.e. the transcription above is still faithful."""
    tree = ast.parse(open(FILTER_SRC).read())
    for fn in ast.walk(tree):
        if isinstance(fn, ast.FunctionDef) and fn.name == '_on_distances':
            for n in ast.walk(fn):
                if isinstance(n, ast.Call) and getattr(n.func, 'id', '') == 'Obstacle':
                    names = {k.arg for k in n.keywords if k.arg}
                    stars = [k for k in n.keywords if k.arg is None]
                    return names == OBSTACLE_KWARGS and len(stars) == 2
    return False


def snap(items, t_cap):
    return ObstacleSnap(items, t_cap, t_cap)


def obstacle_rows(con):
    """(label, a, h, v_obs, c) of every obstacle row of a ConstraintSnap."""
    if con is None:
        return []
    idx = np.flatnonzero(con.group == G_OBS)
    return [(con.links[i], con.A[i], float(con.h_bar[i]), float(con.v_obs[i]), float(con.jdot_qdot[i]))
            for i in idx]
