#!/usr/bin/env python3
"""From q̈_safe to τ: inverse dynamics, limits, and what the arm would actually do (chapter 09).

Production code exercised WITHOUT a ROS graph: the unbound methods
`QddotToTorqueNode._compute_tau` and `_limit_and_report` (nodes/qddot_to_torque.py) are called on a
stub `self` built with the same production helpers the constructor uses
(`generate_urdf_from_xacro` → hand:=true model, `load_pinocchio_model`, `resolve_arm_joint_ids`,
`load_franka_joint_limits`). The publishers are replaced by no-ops.

  1. (9.4) _compute_tau == M(q)q̈ + C(q,q̇)q̇ == RNEA(q,q̇,q̈) − g(q) (independent Pinocchio call),
     200 random states                                                                        (NUM)
  2. (9.3) gravity is NOT in τ: _compute_tau(q, 0, 0) = 0                                       (NUM)
  3. data.M from computeAllTerms is symmetric (F-2-3 context; Pinocchio version printed)        (NUM)
  4. _limit_and_report with iso_enabled = false returns τ UNCLIPPED even beyond effort_max;
     with iso_enabled = true it clips to effort_max and rate-limits                            (FACT)
  5. torque needed by the acceleration box: max |M q̈ + C q̇| over the 128 box vertices at random
     poses (q̇ = 0) vs effort_max — the QP has no torque rows                       (NUM, illustrative)
  6. realised vs commanded acceleration when the plant's inertia differs from the model by the
     Gazebo armature (diagonal approximation of `gz_robot_description.ARMATURE`): the feedforward
     alone realises (M + A)⁻¹M q̈_safe                                           (NUM, illustrative)

Needs: the ROS 2 container (Pinocchio, xacro, franka_description). Read-only.
"""
import os
import sys
from types import SimpleNamespace

sys.dont_write_bytecode = True
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _common4 as C4  # noqa: E402

C = C4.C
import numpy as np  # noqa: E402
import pinocchio as pin  # noqa: E402
from franka_experiments.nodes.qddot_to_torque import QddotToTorqueNode as QN  # noqa: E402
from franka_experiments.utils.config import load_franka_joint_limits  # noqa: E402
from franka_experiments.utils.kinematics import (  # noqa: E402
    generate_urdf_from_xacro, load_pinocchio_model, resolve_arm_joint_ids)

R = C.Report('check_torque_realization (chapter 09 §1-§2, §5)')
R.info(f'pinocchio {pin.__version__}')
model, data = load_pinocchio_model(generate_urdf_from_xacro())
jids = resolve_arm_joint_ids(model)
v_ids = [model.joints[j].idx_v for j in jids]
q_ids = [model.joints[j].idx_q for j in jids]


class _Pub:
    def publish(self, _m):
        pass


class _Log:
    def warn(self, *_a, **_k):
        pass


jl = load_franka_joint_limits([f'joint{i}' for i in range(1, 8)])
S = SimpleNamespace(_model=model, _data=data, _arm_v_ids=v_ids, _arm_q_ids=q_ids,
                    _arm_v_ix=np.ix_(v_ids, v_ids), _q_neutral=pin.neutral(model), _q_full=pin.neutral(model),
                    _qdot_full=np.zeros(model.nv), _M_arm=np.zeros((7, 7)), _C_qdot_arm=np.zeros(7),
                    _effort_max=jl['effort_max'], _tau_prev=None, _tau_rate_max=40.0 * 100.0 / 300.0,
                    _iso_enabled=False, _sat_pub=_Pub(), _sat_msg=SimpleNamespace(data=[0.0] * 7),
                    get_logger=lambda: _Log())
R.info(f'model nv = {model.nv} (arm 7 + fingers); effort_max = {jl["effort_max"].tolist()} N·m')

rng = np.random.default_rng(9)
lo, hi = jl['q_min'], jl['q_max']
err_id, err_g, err_sym = 0.0, 0.0, 0.0
for _ in range(200):
    q = rng.uniform(lo, hi)
    qd = rng.uniform(-1.5, 1.5, 7)
    qdd = rng.uniform(-5, 5, 7)
    tau = QN._compute_tau(S, q, qd, qdd)
    qf, vf, af = pin.neutral(model), np.zeros(model.nv), np.zeros(model.nv)
    qf[q_ids], vf[v_ids], af[v_ids] = q, qd, qdd
    ref = pin.rnea(model, model.createData(), qf, vf, af) - pin.computeGeneralizedGravity(model, model.createData(), qf)
    err_id = max(err_id, float(np.abs(tau - ref[v_ids]).max()))
    err_g = max(err_g, float(np.abs(QN._compute_tau(S, q, np.zeros(7), np.zeros(7))).max()))
    err_sym = max(err_sym, float(np.abs(S._M_arm - S._M_arm.T).max()))
R.below('1. _compute_tau == RNEA(q, q̇, q̈) − g(q) (200 random states) [N·m]', err_id, 1e-9)
R.below('2. gravity excluded: _compute_tau(q, 0, 0) = 0 [N·m]', err_g, 1e-12)
R.below('3. data.M (arm block) symmetric', err_sym, 1e-12)

# 4) limiting
big = jl['effort_max'] * 1.5
S._iso_enabled, S._tau_prev = False, None
out_off = QN._limit_and_report(S, big.copy())
S._iso_enabled, S._tau_prev = True, np.zeros(7)
out_on = QN._limit_and_report(S, big.copy())
R.fact('4. iso_enabled = false (the shipped default): τ = 1.5·effort_max goes out UNCLIPPED',
       np.array_equal(out_off, big))
R.fact(f'4. iso_enabled = true: clipped to effort_max AND rate-limited to {S._tau_rate_max:.2f} N·m per call',
       np.allclose(out_on, np.clip(big, -jl['effort_max'], jl['effort_max']).clip(-S._tau_rate_max, S._tau_rate_max)))

# 5) torque demanded by the acceleration box
from itertools import product  # noqa: E402
P = C4.C3.params_namespace('hw_default')
L = C4.static_limits(P)
verts = np.array(list(product(*[(L['lb'][i], L['ub'][i]) for i in range(7)])))
worst = np.zeros(7)
n_exceed = 0
for _ in range(100):
    q = rng.uniform(lo, hi)
    for v in verts:
        t = np.abs(QN._compute_tau(S, q, np.zeros(7), v))
        worst = np.maximum(worst, t)
        n_exceed += int(np.any(t > jl['effort_max']))
R.info(f'5. max |τ| over box vertices at 100 random poses: {np.round(worst, 1).tolist()} N·m '
       f'vs effort_max {jl["effort_max"].tolist()}')
R.fact(f'5. box-admissible q̈ that need more than effort_max (no torque row in the QP): '
       f'{n_exceed} of {100 * len(verts)} vertex×pose cases', True,
       'reported, not asserted: whether a QP solution reaches such a vertex is runtime')

# 6) armature mismatch (Gazebo plant) — diagonal approximation
ARM = np.array([0.195, 0.195, 0.195, 0.195, 0.074, 0.074, 0.074])     # gz_robot_description.ARMATURE
q = C.Q_HOME.copy()
QN._compute_tau(S, q, np.zeros(7), np.zeros(7))
M = S._M_arm.copy()
ratio = np.diag(np.linalg.solve(M + np.diag(ARM), M))
R.info('6. diag((M + A)⁻¹M) at q_home — fraction of a single-joint q̈_safe the feedforward alone realises: '
       + ', '.join(f'j{i + 1} {r:.2f}' for i, r in enumerate(ratio)))
R.fact('6. with the armature the feedforward under-delivers on every joint (ratio < 1); the 1 kHz PD on the '
       'integrated reference (09 §3) is what closes the gap', bool(np.all(ratio < 1.0)),
       'illustrative: armature modelled as M + diag(A); the plugin adds a disk inertia on the child link')
sys.exit(R.finish())
