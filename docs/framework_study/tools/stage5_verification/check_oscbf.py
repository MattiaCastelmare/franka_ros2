#!/usr/bin/env python3
"""Pipeline C: the torque-level OSCBF filter (chapter 10 §3).

Production code: `cbf_OSCBF_filter._RobotDynamics`, `_OSCBFQPBuilder.build_cost`,
`_build_joint_position_cbf`, `_build_obstacle_cbf`, `_build_urdf_no_hand`, on the real models
(`generate_urdf_from_xacro` hand:=true for the dynamics; hand-less URDF for the obstacle points).

Row convention (`_OSCBFQPBuilder.solve`): A τ + s ≥ b. With q̈ = M⁻¹(τ − C q̇), every row can be
rewritten as a condition on q̈ (or ḧ); the scripts read back the implied gains.

  1. (10.10) joint-position rows: A τ ≥ b ⇔ q̈ᵢ ≥ −(α₁+α₂)q̇ᵢ − α₁α₂hᵢ — the HOCBF chain      (NUM)
  2. (10.11) obstacle rows: A τ ≥ b ⇔ ḧ ≥ −α₂ḣ − α₁α₂h; the HOCBF chain requires −(α₁+α₂)ḣ − α₁α₂h:
     the α₁ḣ term is missing (F-5-4)                                                           (NUM)
  3. consequence: the obstacle row's closed loop is s² + α₂s + α₁α₂ (ζ = 0.5 at α = 10): impulse
     undershoot exp(−ζπ/√(1−ζ²)) = 16.3 %; 1-D approach where the code law crosses h = 0 and the
     correct law does not                                                        (NUM, illustrative)
  4. the obstacle row ignores LinkDistance.distance: h = ‖p_r − p_h‖ − 0.15 (centre distance,
     hard-coded d_safe)                                                                        (NUM)
  5. (10.8) cost: P_arm = A_jᵀA_j + A_oᵀA_o is positive definite; the unconstrained optimum is τ_nom;
     ½(τ−τ_n)ᵀP(τ−τ_n) equals the documented ½‖w_j M⁻¹Nᵀ(τ−τ_n)‖² + ½‖w_o J M⁻¹(τ−τ_n)‖²       (NUM)
  6. defaults and wiring: enable_obstacle_cbf false; RELIABLE subscription vs BEST_EFFORT
     publisher; output /NS_1/torque_safe; joint states from the 30 Hz topic                   (FACT)

Needs: the ROS 2 container (Pinocchio, xacro, franka_description, qpsolvers). Read-only.
"""
import math
import os
import re
import sys

sys.dont_write_bytecode = True
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _common5 as C5  # noqa: E402

C = C5.C
import numpy as np  # noqa: E402
import pinocchio as pin  # noqa: E402
from franka_experiments.nodes import cbf_OSCBF_filter as OS  # noqa: E402
from franka_experiments.utils.kinematics import (  # noqa: E402
    generate_urdf_from_xacro, load_pinocchio_model, resolve_arm_joint_ids, resolve_frame_id)

R = C.Report('check_oscbf (chapter 10 §3)')
dm, _ = load_pinocchio_model(generate_urdf_from_xacro())
jids = resolve_arm_joint_ids(dm)
dyn = OS._RobotDynamics(dm, resolve_frame_id(dm, 'fr3_hand_tcp'), [dm.joints[j].idx_q for j in jids],
                        [dm.joints[j].idx_v for j in jids])
p = OS._build_urdf_no_hand()
try:
    km = pin.buildModelFromUrdf(p)
finally:
    os.unlink(p)
kd = km.createData()
a1, a2 = 10.0, 10.0                                            # node defaults alpha1, alpha2
rng = np.random.default_rng(4)
q = C.Q_HOME.copy()
qd = rng.uniform(-0.4, 0.4, 7)
dyn.update(q, qd)
M_inv, Cq = dyn.M_inv, dyn.C_qdot

# 1) joint position rows
lo, hi = q - 0.5, q + 0.5
A, b = OS._build_joint_position_cbf(q, qd, M_inv, Cq, lo, hi, a1, a2)
MiC = M_inv @ Cq
imp_lo = b[0::2] - MiC                     # A τ ≥ b ⇔ q̈ ≥ b − M⁻¹Cq̇ (lower rows)
R.close('1. lower joint rows imply q̈ ≥ −(α₁+α₂)q̇ − α₁α₂(q − q_min)', imp_lo, -(a1 + a2) * qd - a1 * a2 * (q - lo),
        1e-10, kind='NUM')

# 2) obstacle rows (via the real kinematics model)
pin.forwardKinematics(km, kd, q, qd)
pin.computeJointJacobians(km, kd, q)
pin.computeJointJacobiansTimeVariation(km, kd, q, qd)
pin.updateFramePlacements(km, kd)
fid = km.getFrameId('fr3_link7')
pr = kd.oMf[fid].translation.copy()
ph = pr + np.array([0.25, 0.05, -0.02])
item = dict(valid=True, confidence=0.9, closest_point_robot=pr, closest_point_human=ph, link_name='fr3_link7',
            distance=0.123)
Ao, bo = OS._build_obstacle_cbf([item], km, kd, q, qd, M_inv, Cq, a1, a2, d_safe=0.15)
J6 = pin.getFrameJacobian(km, kd, fid, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
Jd6 = pin.getFrameJacobianTimeVariation(km, kd, fid, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
Jp, Jpd = J6[:3], Jd6[:3]                   # r = 0: the point is the frame origin
d_ctr = float(np.linalg.norm(pr - ph))
n = (pr - ph) / d_ctr
h = d_ctr - 0.15
hdot = float(n @ Jp @ qd)
ndot = (Jp @ qd - n * hdot) / d_ctr
c = float(n @ Jpd @ qd + ndot @ (Jp @ qd))
R.close('2. obstacle row A = n̂ᵀJ_p M⁻¹', Ao[0], n @ Jp @ M_inv, 1e-10, kind='NUM')
implied = bo[0] - (n @ Jp @ M_inv @ Cq - c)          # A τ ≥ b ⇔ ḧ ≥ implied
R.close('2. the row implies ḧ ≥ −α₂ḣ − α₁α₂h (as coded)', implied, -a2 * hdot - a1 * a2 * h, 1e-10, kind='NUM')
hocbf = -(a1 + a2) * hdot - a1 * a2 * h
R.fact(f'2. F-5-4: the HOCBF chain needs ḧ ≥ {hocbf:+.4f}; the row demands {implied:+.4f}; the missing term '
       f'α₁ḣ = {a1 * hdot:+.4f}', abs((implied - hocbf) - a1 * hdot) < 1e-10)

# 3) consequence
zeta = a2 / (2 * math.sqrt(a1 * a2))
und = math.exp(-zeta * math.pi / math.sqrt(1 - zeta ** 2))
R.close('3. coded obstacle loop s² + α₂s + α₁α₂: ζ', zeta, 0.5, 1e-12, kind='NUM')
R.close('3. impulse-response negative lobe exp(−ζπ/√(1−ζ²))', und, 0.163, 5e-4, kind='NUM')


def approach(k1, k0, h0=0.20, v0=-1.0, T=2.0, dt=1e-4):
    hh, v, hmin = h0, v0, h0
    for _ in range(int(T / dt)):
        acc = max(0.0, -k1 * v - k0 * hh)          # nominal ḧ = 0; the row binds when its bound > 0
        v += acc * dt
        hh += v * dt
        hmin = min(hmin, hh)
    return hmin


hm_code = approach(a2, a1 * a2)
hm_true = approach(a1 + a2, a1 * a2)
R.fact(f'3. 1-D approach at 1.0 m/s from h = 0.20 m (rows binding exactly): coded law min h = {hm_code * 1e3:+.1f} mm, '
       f'HOCBF law min h = {hm_true * 1e3:+.1f} mm', hm_code < 0.0 <= hm_true,
       'illustrative: ḧ = the row bound whenever it is positive')

# 4) distance field ignored
item2 = dict(item, distance=0.9)
Ao2, bo2 = OS._build_obstacle_cbf([item2], km, kd, q, qd, M_inv, Cq, a1, a2, d_safe=0.15)
R.fact('4. the row is identical whatever LinkDistance.distance says (h from ‖p_r − p_h‖ − 0.15)',
       np.array_equal(Ao, Ao2) and np.array_equal(bo, bo2), f'centre distance {d_ctr:.4f} m vs distance field 0.123/0.9')

# 5) cost
Pq, qv = OS._OSCBFQPBuilder(1.0, 1.0, 1000.0, 'osqp').build_cost(M_inv, dyn.J, np.zeros(7))
Parm = Pq[:7, :7]
ev = np.linalg.eigvalsh(0.5 * (Parm + Parm.T))
R.fact(f'5. P_arm positive definite (eigenvalues {ev.min():.3e} … {ev.max():.3e})', ev.min() > 0)
tn = rng.normal(size=7) * 5
_, qv2 = OS._OSCBFQPBuilder(1.0, 1.0, 1000.0, 'osqp').build_cost(M_inv, dyn.J, tn)
R.below('5. unconstrained optimum −P⁻¹q = τ_nom', float(np.abs(np.linalg.solve(Parm, -qv2[:7]) - tn).max()), 1e-6)
JM = dyn.J @ M_inv
Lam = np.linalg.inv(JM @ dyn.J.T + 1e-6 * np.eye(6))
NT = np.eye(7) - dyn.J.T @ Lam @ JM
dt_ = rng.normal(size=7)
lhs = 0.5 * dt_ @ Parm @ dt_
rhs = 0.5 * np.sum((M_inv @ NT @ dt_) ** 2) + 0.5 * np.sum((JM @ dt_) ** 2)
R.close('5. ½ΔτᵀPΔτ = ½‖M⁻¹NᵀΔτ‖² + ½‖JM⁻¹Δτ‖² (w_j = w_o = 1)', lhs, rhs, 1e-9 * abs(rhs), kind='NUM')

# 6) wiring facts
S = C5.src('franka_experiments/franka_experiments/nodes/cbf_OSCBF_filter.py', 'py')
R.fact("6. enable_obstacle_cbf defaults to False", "P('enable_obstacle_cbf', False)" in S)
R.fact('6. distance subscription with an integer depth (default RELIABLE QoS)',
       re.search(r"MultiLinkDistance, '/cbf/per_link_distances',\s*\n\s*self\._on_distances, 10\)", S) is not None)
RT = C5.src('franka_experiments/franka_experiments/nodes/real_time_distance.py', 'py')
R.fact('6. real_time_distance publishes per_link_distances BEST_EFFORT (so a RELIABLE subscriber gets nothing)',
       'BEST_EFFORT' in RT, 'F-1-6')
R.fact("6. output /NS_1/torque_safe; input /NS_1/torque_cmd; joint state from topics.joint_states_topic (30 Hz)",
       "P('tau_safe_topic',      '/NS_1/torque_safe')" in S and "P('tau_nom_topic',       '/NS_1/torque_cmd')" in S
       and "cfg['topics']['joint_states_topic']" in S)
R.fact('6. infeasible QP → τ_nom passed unfiltered; stale nominal → τ = 0; stale distances → rows skipped',
       'return tau_nom' in S and 'tau_nom   = np.zeros(_N_ARM) if nom_stale' in S
       and "'Link distances stale — obstacle CBF skipped'" in S)
sys.exit(R.finish())
