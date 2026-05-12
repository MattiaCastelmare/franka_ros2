#!/usr/bin/env python3
"""Operational Space CBF (OSCBF) safety filter for torque-controlled FR3.

Implements the Dynamic-OSCBF from:
  Morton & Pavone, "Safe, Task-Consistent Manipulation with Operational
  Space Control Barrier Functions", arXiv:2503.06736, 2025.

Decision variable: τ ∈ ℝ⁷  (published torque, gravity EXCLUDED — hardware adds g(q))

QP cost (Eq. 37 of paper):
  min  ½ ‖Wⱼ M⁻¹ Nᵀ(τ − τ_nom)‖² + ½ ‖Wₒ J M⁻¹(τ − τ_nom)‖²

where  Nᵀ = I − Jᵀ Λ J M⁻¹   (null-space projector on torques)
       Λ  = (J M⁻¹ Jᵀ)⁻¹      (operational-space mass matrix)
       J  = J_ee(q)  6×7        (LOCAL_WORLD_ALIGNED Jacobian)

CBF constraints enforced:
  - Joint position limits   (RD2 HOCBF, torque control)
  - Joint velocity limits   (RD1 CBF,   torque control)  [optional]
  - EE workspace box        (RD2 HOCBF)                   [optional]
  - Obstacle / link dist    (RD2 HOCBF via MultiLinkDistance) [optional]
  - Torque box bounds

Subscribe:
  tau_nom_topic       (std_msgs/Float64MultiArray, 7)
  joint_states_topic  (sensor_msgs/JointState)
  /cbf/per_link_distances  (franka_msgs/MultiLinkDistance)  [if enable_obstacle_cbf]

Publish:
  tau_safe_topic      (std_msgs/Float64MultiArray, 7)

Parameters: see load_params() docstring.
"""
from __future__ import annotations

import glob
import os
import subprocess
import tempfile
from typing import Optional

import numpy as np
import pinocchio as pin
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from franka_experiments.utils.constants import FR3_JOINT_NAMES, NUM_JOINTS
from franka_experiments.utils.kinematics import (
    generate_urdf_from_xacro,
    load_pinocchio_model,
    resolve_arm_joint_ids,
    resolve_frame_id,
)
from franka_experiments.utils.logging_utils import ThrottledLogger, vec_to_str
from franka_msgs.msg import MultiLinkDistance

import rclpy

# ── Solver import with graceful fallback ────────────────────────────────────
try:
    import qpsolvers
    _QP_AVAILABLE = True
except ImportError:
    _QP_AVAILABLE = False


# ── Constants ───────────────────────────────────────────────────────────────
_N_ARM = NUM_JOINTS          # 7
_LAMBDA_REG = 1e-6           # regularisation added to Λ = (J M⁻¹ Jᵀ)⁻¹


def _load_control_yaml() -> dict:
    path = os.path.join(
        get_package_share_directory('franka_experiments'),
        'config', 'fr3_control.yaml')
    with open(path) as fh:
        return yaml.safe_load(fh)


def _build_urdf_no_hand() -> str:
    """Generate FR3 URDF without hand and return path to a temp file."""
    share = get_package_share_directory('franka_description')
    candidates = glob.glob(
        os.path.join(share, '**', 'fr3.urdf.xacro'), recursive=True)
    if not candidates:
        raise RuntimeError(f'fr3.urdf.xacro not found under {share}')
    tmp = tempfile.NamedTemporaryFile(
        suffix='.urdf', delete=False, prefix='fr3_oscbf_nohand_')
    tmp.close()
    res = subprocess.run(
        ['xacro', candidates[0], 'hand:=false', '-o', tmp.name],
        capture_output=True, text=True)
    if res.returncode != 0:
        raise RuntimeError(f'xacro failed:\n{res.stderr}')
    return tmp.name


# ── Dynamics helper ──────────────────────────────────────────────────────────

class _RobotDynamics:
    """Thin wrapper around Pinocchio: computes M, C_qdot, J_ee, dJ_ee."""

    def __init__(self, model: pin.Model, ee_frame_id: int,
                 arm_q_ids: list[int], arm_v_ids: list[int]):
        self.model       = model
        self.data        = model.createData()
        self.ee_frame_id = ee_frame_id
        self.arm_q_ids   = arm_q_ids
        self.arm_v_ids   = arm_v_ids
        self.arm_v_ix    = np.ix_(arm_v_ids, arm_v_ids)

        nv = model.nv
        self._q_neutral   = pin.neutral(model)
        self._q_full      = pin.neutral(model)
        self._qdot_full   = np.zeros(nv)
        self._Cqdot_full  = np.zeros(nv)

        # Pre-allocated outputs
        self.M        = np.zeros((_N_ARM, _N_ARM))
        self.M_inv    = np.zeros((_N_ARM, _N_ARM))
        self.C_qdot   = np.zeros(_N_ARM)   # C(q,q̇)·q̇ vector (gravity NOT included)
        self.J        = np.zeros((6, _N_ARM))   # 6×7 EE Jacobian
        self.dJ_qdot  = np.zeros(6)             # dJ·q̇  (bias acceleration term)
        self.p_ee     = np.zeros(3)             # EE position in world frame

        # Warm-up to avoid first-call JIT overhead
        pin.computeAllTerms(model, self.data, self._q_full, self._qdot_full)

    def update(self, q: np.ndarray, qdot: np.ndarray) -> None:
        """Run a full Pinocchio pass. Updates M, M_inv, C_qdot, J, dJ_qdot, p_ee."""
        np.copyto(self._q_full, self._q_neutral)
        self._qdot_full[:] = 0.0
        for k, (iq, iv) in enumerate(zip(self.arm_q_ids, self.arm_v_ids)):
            self._q_full[iq]    = q[k]
            self._qdot_full[iv] = qdot[k]

        pin.computeAllTerms(self.model, self.data,
                            self._q_full, self._qdot_full)
        pin.updateFramePlacements(self.model, self.data)
        pin.computeJointJacobians(self.model, self.data, self._q_full)
        pin.computeJointJacobiansTimeVariation(self.model, self.data,
                                               self._q_full, self._qdot_full)

        # M 7×7
        M_full = np.asarray(self.data.M)
        np.copyto(self.M, M_full[self.arm_v_ix])

        # C·qdot vector 7
        np.dot(np.asarray(self.data.C), self._qdot_full, out=self._Cqdot_full)
        for k, iv in enumerate(self.arm_v_ids):
            self.C_qdot[k] = self._Cqdot_full[iv]

        # J 6×7 (LOCAL_WORLD_ALIGNED)
        J_full = pin.getFrameJacobian(
            self.model, self.data, self.ee_frame_id,
            pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
        for i, iv in enumerate(self.arm_v_ids):
            self.J[:, i] = J_full[:, iv]

        # dJ·qdot 6-vector
        dJ_full = pin.getFrameJacobianTimeVariation(
            self.model, self.data, self.ee_frame_id,
            pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
        dJ_arm = np.asarray([dJ_full[:, iv] for iv in self.arm_v_ids]).T  # 6×7
        self.dJ_qdot[:] = dJ_arm @ qdot

        # EE position
        self.p_ee[:] = self.data.oMf[self.ee_frame_id].translation

        # M_inv (small 7×7 inversion — unavoidable)
        try:
            np.copyto(self.M_inv, np.linalg.inv(self.M))
        except np.linalg.LinAlgError:
            np.copyto(self.M_inv, np.linalg.pinv(self.M))


# ── OSCBF QP builder ─────────────────────────────────────────────────────────

class _OSCBFQPBuilder:
    """Builds the OSCBF QP matrices from current robot state and CBF constraints.

    The QP has n_dec = _N_ARM + 1 decision variables: [τ (7), slack s (1)].

    Solver convention (qpsolvers):
      minimize    ½ xᵀ P x + qᵀ x
      subject to  G x ≤ h          (inequality: G x ≤ h)
                  lb ≤ x ≤ ub
    """

    def __init__(self, w_j: float, w_o: float,
                 rho_slack: float, solver: str):
        self.w_j      = float(w_j)
        self.w_o      = float(w_o)
        self.rho      = float(rho_slack)
        self.solver   = solver
        self.n_dec    = _N_ARM + 1   # [τ(7), s(1)]
        self._warm    : Optional[np.ndarray] = None

    def build_cost(
        self,
        M_inv : np.ndarray,  # 7×7
        J     : np.ndarray,  # 6×7
        tau_nom: np.ndarray, # 7
    ) -> tuple[np.ndarray, np.ndarray]:
        """Return (P, q_vec) for the task-consistent OSCBF cost.

        Paper Eq. 38:
          P_QP  = (Wⱼ M⁻¹ Nᵀ)ᵀ (Wⱼ M⁻¹ Nᵀ) + (Wₒ J M⁻¹)ᵀ (Wₒ J M⁻¹)
          q_QP  = −P_QP τ_nom    (padded with 0 for slack)
        """
        # Λ = (J M⁻¹ Jᵀ)⁻¹   — regularised to handle near-singularities
        JM_inv  = J @ M_inv                     # 6×7
        JM_invJT = JM_inv @ J.T                 # 6×6
        try:
            Lam = np.linalg.inv(
                JM_invJT + _LAMBDA_REG * np.eye(6))
        except np.linalg.LinAlgError:
            Lam = np.linalg.pinv(JM_invJT)

        # Nᵀ = I − Jᵀ Λ J M⁻¹   (7×7 null-space projector on torques)
        NT = np.eye(_N_ARM) - J.T @ Lam @ JM_inv   # 7×7

        # Weighted maps
        A_j = self.w_j * (M_inv @ NT)   # Wⱼ M⁻¹ Nᵀ   (7×7)
        A_o = self.w_o * JM_inv          # Wₒ J M⁻¹    (6×7)

        # P_arm = Aⱼᵀ Aⱼ + Aₒᵀ Aₒ   (7×7)
        P_arm = A_j.T @ A_j + A_o.T @ A_o

        # Full P (with slack penalty)
        P = np.zeros((self.n_dec, self.n_dec))
        P[:_N_ARM, :_N_ARM] = P_arm
        P[_N_ARM, _N_ARM]   = self.rho

        # Linear term
        q_arm = -(P_arm @ tau_nom)
        q_vec         = np.zeros(self.n_dec)
        q_vec[:_N_ARM] = q_arm
        # slack coefficient = 0

        return P, q_vec

    def solve(
        self,
        P       : np.ndarray,          # (n_dec, n_dec)
        q_vec   : np.ndarray,          # (n_dec,)
        A_cbf   : np.ndarray,          # (n_c, 7)  constraint: A_cbf τ + s ≥ b_cbf
        b_cbf   : np.ndarray,          # (n_c,)
        tau_min : np.ndarray,          # (7,)
        tau_max : np.ndarray,          # (7,)
    ) -> tuple[Optional[np.ndarray], float]:
        """Solve and return (tau_safe, slack) or (None, 0.0) on failure."""
        n_c = A_cbf.shape[0] if A_cbf.ndim == 2 else 0

        # Box bounds on decision variable [τ, s]
        lb = np.concatenate([tau_min, [0.0]])
        ub = np.concatenate([tau_max, [1e8]])

        G_rows, h_rows = [], []

        if n_c > 0:
            # CBF: A_cbf τ + s ≥ b_cbf  →  −A_cbf τ − s ≤ −b_cbf
            G_cbf = np.zeros((n_c, self.n_dec))
            G_cbf[:, :_N_ARM] = -A_cbf
            G_cbf[:, _N_ARM]  = -1.0
            G_rows.append(G_cbf)
            h_rows.append(-b_cbf)

        if G_rows:
            G     = np.vstack(G_rows)
            h_vec = np.concatenate(h_rows)
        else:
            G     = np.zeros((0, self.n_dec))
            h_vec = np.zeros(0)

        x = qpsolvers.solve_qp(
            P=P, q=q_vec,
            G=G, h=h_vec,
            lb=lb, ub=ub,
            solver=self.solver,
            initvals=self._warm,
            verbose=False,
        )

        if x is None or not np.all(np.isfinite(x)):
            self._warm = None
            return None, 0.0

        self._warm = x.copy()
        return x[:_N_ARM].copy(), float(x[_N_ARM])


# ── CBF constraint builders ───────────────────────────────────────────────────

def _build_joint_position_cbf(
    q      : np.ndarray,   # (7,)
    qdot   : np.ndarray,   # (7,)
    M_inv  : np.ndarray,   # (7,7)
    C_qdot : np.ndarray,   # (7,)  C(q,qdot)·qdot
    q_min  : np.ndarray,   # (7,)
    q_max  : np.ndarray,   # (7,)
    alpha1 : float,
    alpha2 : float,
) -> tuple[np.ndarray, np.ndarray]:
    """HOCBF joint position constraints (relative degree 2 under torque control).

    Derives A_rows, b_rows such that A_rows @ τ ≥ b_rows enforces
    q_min ≤ q ≤ q_max via the HOCBF condition.

    For lower bound  h_i = q_i − q_min_i:
      h₂ = q̇_i + α₁·h_i
      condition: [M⁻¹]ᵢ · τ ≥ [M⁻¹ C_qdot]ᵢ − α₁·q̇_i − α₂·h₂_i

    For upper bound  h_i = q_max_i − q_i:
      h₂ = −q̇_i + α₁·h_i
      condition: −[M⁻¹]ᵢ · τ ≥ −[M⁻¹ C_qdot]ᵢ − α₁·q̇_i − α₂·h₂_i
    """
    M_inv_C = M_inv @ C_qdot   # 7-vector

    rows_A, rows_b = [], []
    for i in range(_N_ARM):
        # Lower bound
        h_lo  = q[i] - q_min[i]
        h2_lo = qdot[i] + alpha1 * h_lo
        A_lo  = M_inv[i, :]
        b_lo  = M_inv_C[i] - alpha1 * qdot[i] - alpha2 * h2_lo
        rows_A.append(A_lo)
        rows_b.append(b_lo)

        # Upper bound  (sign flip)
        h_hi  = q_max[i] - q[i]
        h2_hi = -qdot[i] + alpha1 * h_hi
        A_hi  = -M_inv[i, :]
        b_hi  = -M_inv_C[i] - alpha1 * qdot[i] - alpha2 * h2_hi
        rows_A.append(A_hi)
        rows_b.append(b_hi)

    return np.array(rows_A), np.array(rows_b)


def _build_joint_velocity_cbf(
    qdot    : np.ndarray,   # (7,)
    M_inv   : np.ndarray,   # (7,7)
    C_qdot  : np.ndarray,   # (7,)
    qdot_min: np.ndarray,   # (7,)
    qdot_max: np.ndarray,   # (7,)
    alpha_v : float,
) -> tuple[np.ndarray, np.ndarray]:
    """RD1 CBF for joint velocity limits under torque control.

    For  h_i = q̇_i − q̇_min_i  (lower):
      ḣ = q̈_i = [M⁻¹(τ − C_qdot)]_i
      condition: [M⁻¹]_i · τ ≥ [M⁻¹ C_qdot]_i − α·h_i
    """
    M_inv_C = M_inv @ C_qdot

    rows_A, rows_b = [], []
    for i in range(_N_ARM):
        # Lower velocity bound
        h_lo = qdot[i] - qdot_min[i]
        rows_A.append(M_inv[i, :])
        rows_b.append(M_inv_C[i] - alpha_v * h_lo)

        # Upper velocity bound
        h_hi = qdot_max[i] - qdot[i]
        rows_A.append(-M_inv[i, :])
        rows_b.append(-M_inv_C[i] - alpha_v * h_hi)

    return np.array(rows_A), np.array(rows_b)


def _build_ee_workspace_cbf(
    q       : np.ndarray,    # (7,)
    qdot    : np.ndarray,    # (7,)
    p_ee    : np.ndarray,    # (3,) EE world position
    J3      : np.ndarray,    # (3,7) translational Jacobian
    dJ3_qdot: np.ndarray,    # (3,) dJ_lin·qdot
    M_inv   : np.ndarray,    # (7,7)
    C_qdot  : np.ndarray,    # (7,)
    ws_min  : np.ndarray,    # (3,)
    ws_max  : np.ndarray,    # (3,)
    alpha1  : float,
    alpha2  : float,
) -> tuple[np.ndarray, np.ndarray]:
    """HOCBF workspace box constraints for the EE (relative degree 2).

    For lower bound along axis k:  h_k = p_ee_k − ws_min_k
      h_dot = J3[k,:] q̇
      h₂    = J3[k,:] q̇ + α₁·h_k
      ḣ₂    = J3[k,:] q̈ + dJ3_qdot[k] + α₁ J3[k,:] q̇
      condition: J3[k,:] M⁻¹ τ ≥ J3[k,:] M⁻¹ C_qdot − dJ3_qdot[k]
                                   − α₁ J3[k,:] q̇ − α₂ h₂_k
    """
    JM = J3 @ M_inv            # (3,7)
    JM_C = JM @ C_qdot         # (3,)
    v_ee = J3 @ qdot           # (3,) EE translational velocity

    rows_A, rows_b = [], []
    for k in range(3):
        # Lower bound
        h_lo  = p_ee[k] - ws_min[k]
        h2_lo = v_ee[k] + alpha1 * h_lo
        A_lo  = JM[k, :]
        b_lo  = JM_C[k] - dJ3_qdot[k] - alpha1 * v_ee[k] - alpha2 * h2_lo
        rows_A.append(A_lo)
        rows_b.append(b_lo)

        # Upper bound
        h_hi  = ws_max[k] - p_ee[k]
        h2_hi = -v_ee[k] + alpha1 * h_hi
        A_hi  = -JM[k, :]
        b_hi  = -JM_C[k] + dJ3_qdot[k] - alpha1 * v_ee[k] - alpha2 * h2_hi
        rows_A.append(A_hi)
        rows_b.append(b_hi)

    return np.array(rows_A), np.array(rows_b)


def _build_obstacle_cbf(
    link_distances : list,
    kin_model      : pin.Model,
    kin_data       : pin.Data,
    q              : np.ndarray,    # (7,)
    qdot           : np.ndarray,    # (7,)
    M_inv          : np.ndarray,    # (7,7)
    C_qdot         : np.ndarray,    # (7,)
    alpha1         : float,
    alpha2         : float,
    d_safe         : float,
    min_confidence : float = 0.2,
) -> tuple[np.ndarray, np.ndarray]:
    """HOCBF obstacle-avoidance constraints from MultiLinkDistance message.

    Re-uses the same CBFKinematics-style derivation as cbf_constraints.py
    but with τ as decision variable (relative degree 2 under torque control).

    For link i vs obstacle: h = ‖p_i − p_obs‖ − d_safe
      h_dot  = n_i · (Jₚ q̇ − v_obs)
      h₂     = h_dot + α₁ h
      ḣ₂     = n_i · (Jₚ q̈ + dJₚ q̇ − a_obs) + ṅ_i·rel_vel + α₁ h_dot
      condition: n_i · Jₚ M⁻¹ τ ≥ n_i · Jₚ M⁻¹ C_qdot − c_i − α₂ h₂
    """
    from franka_experiments.utils.cbf_kinematics import CBFKinematics

    def _skew(v):
        return np.array([[ 0.0, -v[2],  v[1]],
                         [ v[2],  0.0, -v[0]],
                         [-v[1],  v[0],  0.0]])

    rows_A, rows_b = [], []

    for item in link_distances:
        if not item.get('valid', False):
            continue
        if item.get('confidence', 1.0) < min_confidence:
            continue

        p_i   = np.asarray(item['closest_point_robot'], dtype=np.float64)
        p_obs = np.asarray(item['closest_point_human'], dtype=np.float64)
        Delta = p_i - p_obs
        d_num = float(np.linalg.norm(Delta))
        if d_num < 1e-8:
            continue

        n_i = Delta / d_num

        # Resolve frame
        lname = item['link_name']
        fid = None
        if kin_model.existFrame(lname):
            fid = kin_model.getFrameId(lname)
        else:
            for frame in kin_model.frames:
                if lname in frame.name:
                    fid = kin_model.getFrameId(frame.name)
                    break
        if fid is None:
            continue

        # Jacobian at control point (world-aligned)
        oMf = kin_data.oMf[fid]
        r   = p_i - oMf.translation
        J6  = pin.getFrameJacobian(kin_model, kin_data, fid,
                                   pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
        Jdot6 = pin.getFrameJacobianTimeVariation(
            kin_model, kin_data, fid,
            pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)

        r_x = _skew(r)
        Jp   = J6[:3, :] - r_x @ J6[3:, :]    # 6×nv
        Jpd  = Jdot6[:3, :] - r_x @ Jdot6[3:, :]

        # Extract arm columns
        arm_v_ids = [kin_model.joints[j].idx_v
                     for j in range(1, kin_model.njoints)
                     if kin_model.joints[j].nv == 1][:_N_ARM]
        Jp_arm  = Jp[:, arm_v_ids]   # 3×7
        Jpd_arm = Jpd[:, arm_v_ids]  # 3×7

        p_i_dot  = Jp_arm @ qdot
        rel_vel  = p_i_dot   # obs velocity assumed zero
        hdot     = float(n_i @ rel_vel)
        n_dot    = (rel_vel - n_i * hdot) / d_num

        h   = d_num - d_safe
        h2  = hdot + alpha1 * h
        c_i = float(n_i @ (Jpd_arm @ qdot) + n_dot @ rel_vel)

        # n_i · Jp_arm M⁻¹ τ ≥ n_i · Jp_arm M⁻¹ C_qdot − c_i − α₂ h₂
        nJM  = n_i @ (Jp_arm @ M_inv)     # (7,)
        nJMC = float(n_i @ (Jp_arm @ M_inv @ C_qdot))

        A_row = nJM
        b_val = nJMC - c_i - alpha2 * h2

        if np.all(np.isfinite(A_row)) and np.isfinite(b_val):
            rows_A.append(A_row)
            rows_b.append(b_val)

    if not rows_A:
        return np.zeros((0, _N_ARM)), np.zeros(0)
    return np.array(rows_A), np.array(rows_b)


# ── Main node ─────────────────────────────────────────────────────────────────

class OSCBFFilterNode(Node):
    """OSCBF safety filter for torque-controlled FR3.

    Receives τ_nom from an upstream commander, applies the task-consistent
    OSCBF QP, and publishes τ* (safe torques) to rt_torque_controller.
    """

    def __init__(self):
        super().__init__('oscbf_filter')

        if not _QP_AVAILABLE:
            self.get_logger().fatal('qpsolvers not installed — cannot run OSCBF')
            raise RuntimeError('qpsolvers not found')

        self.load_params()
        self._build_models()
        self._init_state()
        self._create_ros_interfaces()

        self._tlog = ThrottledLogger(self.get_logger(), period_s=1.0)

        mode = 'BYPASS' if self.bypass_filter else 'OSCBF QP'
        self.get_logger().info(
            f'oscbf_filter  [{mode}]  {self._rate_hz:.0f} Hz\n'
            f'  tau_nom  ← {self._tau_nom_topic}\n'
            f'  tau_safe → {self._tau_safe_topic}\n'
            f'  solver   : {self._solver}\n'
            f'  w_j/w_o  : {self._w_j}/{self._w_o}\n'
            f'  alpha1/2 : {self._alpha1}/{self._alpha2}\n'
            f'  joint pos CBF   : enabled\n'
            f'  joint vel CBF   : {"enabled" if self._enable_vel_cbf else "disabled"}\n'
            f'  workspace CBF   : {"enabled" if self._enable_ws_cbf else "disabled"}\n'
            f'  obstacle CBF    : {"enabled" if self._enable_obstacle_cbf else "disabled"}'
        )

    # ── Parameter loading ────────────────────────────────────────────────────

    def load_params(self) -> None:
        """Declare and read all ROS parameters.

        Parameters
        ----------
        bypass_filter       : bool   — skip QP, pass τ_nom straight through
        tau_nom_topic       : str    — topic for nominal torque input
        tau_safe_topic      : str    — topic for safe torque output
        control_rate_hz     : float  — QP execution frequency [Hz]
        w_j                 : float  — null-space weighting scalar Wⱼ
        w_o                 : float  — operational-space weighting scalar Wₒ
        alpha1              : float  — α₁ class-K gain for HOCBF (position CBF)
        alpha2              : float  — α₂ class-K gain for HOCBF
        rho_slack           : float  — slack penalty weight
        qp_solver           : str    — qpsolvers backend (osqp, cvxopt, …)
        tau_max             : float  — symmetric torque limit [N·m] (scalar or list 7)
        joint_limit_margin  : float  — inward margin on joint position limits [rad]
        enable_vel_cbf      : bool   — enforce joint velocity CBF
        alpha_vel           : float  — α class-K gain for velocity CBF
        enable_ws_cbf       : bool   — enforce EE workspace box CBF
        workspace_min       : list   — [x,y,z] lower corner [m]
        workspace_max       : list   — [x,y,z] upper corner [m]
        ee_frame            : str    — EE Pinocchio frame name
        enable_obstacle_cbf : bool   — subscribe to /cbf/per_link_distances
        dist_timeout_s      : float  — stale distance timeout [s]
        nom_timeout_s       : float  — stale τ_nom timeout [s]
        """
        P = self.declare_parameter
        P('bypass_filter',       False)
        P('tau_nom_topic',       '/NS_1/torque_cmd')
        P('tau_safe_topic',      '/NS_1/torque_safe')
        P('control_rate_hz',     100.0)
        P('w_j',                 1.0)
        P('w_o',                 1.0)
        P('alpha1',              10.0)
        P('alpha2',              10.0)
        P('rho_slack',           1000.0)
        P('qp_solver',           'osqp')
        P('tau_max',             [87.0, 87.0, 87.0, 87.0, 12.0, 12.0, 12.0])
        P('joint_limit_margin',  0.05)
        P('enable_vel_cbf',      False)
        P('alpha_vel',           5.0)
        P('enable_ws_cbf',       False)
        P('workspace_min',       [-0.8, -0.8,  0.0])
        P('workspace_max',       [ 0.8,  0.8,  1.2])
        P('ee_frame',            'fr3_hand_tcp')
        P('enable_obstacle_cbf', False)
        P('dist_timeout_s',      0.5)
        P('nom_timeout_s',       0.5)

        def gp(name):
            return self.get_parameter(name).value

        self.bypass_filter          = bool(gp('bypass_filter'))
        self._tau_nom_topic         = str(gp('tau_nom_topic'))
        self._tau_safe_topic        = str(gp('tau_safe_topic'))
        self._rate_hz               = float(gp('control_rate_hz'))
        self._w_j                   = float(gp('w_j'))
        self._w_o                   = float(gp('w_o'))
        self._alpha1                = float(gp('alpha1'))
        self._alpha2                = float(gp('alpha2'))
        self._rho_slack             = float(gp('rho_slack'))
        self._solver                = str(gp('qp_solver'))
        self._enable_vel_cbf        = bool(gp('enable_vel_cbf'))
        self._alpha_vel             = float(gp('alpha_vel'))
        self._enable_ws_cbf         = bool(gp('enable_ws_cbf'))
        self._enable_obstacle_cbf   = bool(gp('enable_obstacle_cbf'))
        self._dist_timeout          = float(gp('dist_timeout_s'))
        self._nom_timeout           = float(gp('nom_timeout_s'))
        self._ee_frame_name         = str(gp('ee_frame'))

        tau_max_param = gp('tau_max')
        if isinstance(tau_max_param, (int, float)):
            self._tau_max = np.full(_N_ARM, float(tau_max_param))
        else:
            self._tau_max = np.array(list(tau_max_param), dtype=np.float64)
        self._tau_min = -self._tau_max

        ws_min = list(gp('workspace_min'))
        ws_max = list(gp('workspace_max'))
        self._ws_min = np.array(ws_min, dtype=np.float64)
        self._ws_max = np.array(ws_max, dtype=np.float64)

        # Joint limits from fr3_control.yaml (with inward margin)
        cfg   = _load_control_yaml()
        lims  = cfg['joint_limits']
        keys  = [f'joint{i}' for i in range(1, 8)]
        margin = float(gp('joint_limit_margin'))
        self._q_min    = np.array([lims[k][0] for k in keys]) + margin
        self._q_max    = np.array([lims[k][1] for k in keys]) - margin
        self._qdot_max = np.array([lims[k][2] for k in keys])
        self._qdot_min = -self._qdot_max

        self._dt = 1.0 / self._rate_hz

    # ── Model construction ───────────────────────────────────────────────────

    def _build_models(self) -> None:
        """Build two Pinocchio models: dynamics (with hand) and kinematics (no hand)."""
        self.get_logger().info('Building Pinocchio models…')

        # Dynamics model (with hand) — M, C, g
        urdf_xml = generate_urdf_from_xacro()
        self._dyn_model, self._dyn_data = load_pinocchio_model(urdf_xml)

        pin_jids     = resolve_arm_joint_ids(self._dyn_model)
        arm_q_ids    = [self._dyn_model.joints[j].idx_q for j in pin_jids]
        arm_v_ids    = [self._dyn_model.joints[j].idx_v for j in pin_jids]

        ee_frame_id  = resolve_frame_id(self._dyn_model, self._ee_frame_name)

        self._dyn = _RobotDynamics(
            self._dyn_model, ee_frame_id, arm_q_ids, arm_v_ids)

        # CBF kinematics model (no hand, nv=7) — obstacle Jacobians
        cbf_urdf_path = _build_urdf_no_hand()
        self._kin_model = pin.buildModelFromUrdf(cbf_urdf_path)
        self._kin_data  = self._kin_model.createData()

        self._qp = _OSCBFQPBuilder(
            w_j=self._w_j, w_o=self._w_o,
            rho_slack=self._rho_slack, solver=self._solver,
        )

        self.get_logger().info(
            f'Dynamics model: nv={self._dyn_model.nv}  '
            f'ee_frame="{self._ee_frame_name}"  '
            f'id={ee_frame_id}')

    # ── State initialisation ─────────────────────────────────────────────────

    def _init_state(self) -> None:
        self.q              : Optional[np.ndarray] = None
        self.qdot           : Optional[np.ndarray] = None
        self._t_js          : Optional[float]      = None
        self._tau_nom       = np.zeros(_N_ARM)
        self._t_nom         : Optional[float]      = None
        self._link_distances: list                  = []
        self._t_dist        : Optional[float]      = None
        self._js_index_map  : Optional[list]       = None

        # Pre-allocated message
        self._tau_msg       = Float64MultiArray()
        self._tau_msg.data  = [0.0] * _N_ARM

    # ── ROS interfaces ────────────────────────────────────────────────────────

    def _create_ros_interfaces(self) -> None:
        cfg    = _load_control_yaml()
        js_topic = cfg['topics']['joint_states_topic']

        self.create_subscription(
            JointState, js_topic, self._on_joint_state, 10)
        self.create_subscription(
            Float64MultiArray, self._tau_nom_topic, self._on_tau_nom, 10)

        if self._enable_obstacle_cbf:
            self.create_subscription(
                MultiLinkDistance, '/cbf/per_link_distances',
                self._on_distances, 10)

        self._pub = self.create_publisher(
            Float64MultiArray, self._tau_safe_topic, 10)

        self.create_timer(self._dt, self._tick)

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _on_joint_state(self, msg: JointState) -> None:
        if self._js_index_map is None:
            try:
                self._js_index_map = [msg.name.index(n) for n in FR3_JOINT_NAMES]
            except ValueError:
                return

        max_idx = max(self._js_index_map)
        if len(msg.position) < max_idx + 1:
            return

        self.q    = np.array([msg.position[i] for i in self._js_index_map])
        self.qdot = np.array(
            [msg.velocity[i] if len(msg.velocity) > i else 0.0
             for i in self._js_index_map])
        self._t_js = self.get_clock().now().nanoseconds * 1e-9

    def _on_tau_nom(self, msg: Float64MultiArray) -> None:
        data = np.asarray(msg.data, dtype=np.float64)
        if data.size == _N_ARM:
            self._tau_nom = data.copy()
            self._t_nom   = self.get_clock().now().nanoseconds * 1e-9

    def _on_distances(self, msg: MultiLinkDistance) -> None:
        out = []
        for ld in msg.links:
            out.append({
                'link_name':           ld.robot_link_name,
                'distance':            float(ld.distance),
                'closest_point_robot': np.array([ld.closest_point_robot.x,
                                                 ld.closest_point_robot.y,
                                                 ld.closest_point_robot.z]),
                'closest_point_human': np.array([ld.closest_point_human.x,
                                                 ld.closest_point_human.y,
                                                 ld.closest_point_human.z]),
                'valid':      bool(ld.valid),
                'confidence': float(ld.confidence),
                'zone':       ld.zone,
            })
        self._link_distances = out
        self._t_dist = self.get_clock().now().nanoseconds * 1e-9

    # ── Control tick ──────────────────────────────────────────────────────────

    def _tick(self) -> None:
        if self.q is None or self.qdot is None:
            return

        now       = self.get_clock().now().nanoseconds * 1e-9
        nom_stale = (self._t_nom is None) or (now - self._t_nom > self._nom_timeout)
        tau_nom   = np.zeros(_N_ARM) if nom_stale else self._tau_nom.copy()

        if self.bypass_filter:
            self._publish(tau_nom)
            return

        # ── Dynamics ──────────────────────────────────────────────────────────
        try:
            self._dyn.update(self.q, self.qdot)
        except Exception as exc:
            self.get_logger().error(
                f'Dynamics update failed: {exc}', throttle_duration_sec=1.0)
            self._publish(tau_nom)
            return

        tau_safe = self.compute_safe_torque(
            q=self.q, qdot=self.qdot, tau_nom=tau_nom, now=now)

        self._publish(tau_safe)

    def compute_safe_torque(
        self,
        q       : np.ndarray,
        qdot    : np.ndarray,
        tau_nom : np.ndarray,
        now     : float,
    ) -> np.ndarray:
        """Solve the OSCBF QP and return τ* (falls back to τ_nom on failure)."""
        M_inv  = self._dyn.M_inv
        J      = self._dyn.J           # 6×7
        C_qdot = self._dyn.C_qdot      # 7-vector
        p_ee   = self._dyn.p_ee        # 3-vector
        dJ_qdot = self._dyn.dJ_qdot    # 6-vector (dJ·q̇)

        # ── Build cost ────────────────────────────────────────────────────────
        P, q_vec = self._qp.build_cost(M_inv, J, tau_nom)

        # ── Build CBF constraints ─────────────────────────────────────────────
        A_rows, b_rows = self.build_constraints(
            q=q, qdot=qdot,
            M_inv=M_inv, J=J, C_qdot=C_qdot,
            p_ee=p_ee, dJ_qdot=dJ_qdot,
            now=now,
        )

        # ── Solve QP ──────────────────────────────────────────────────────────
        tau_safe, slack = self._qp.solve(
            P=P, q_vec=q_vec,
            A_cbf=A_rows, b_cbf=b_rows,
            tau_min=self._tau_min, tau_max=self._tau_max,
        )

        if tau_safe is None:
            self.get_logger().warn(
                'OSCBF QP infeasible — passing τ_nom',
                throttle_duration_sec=1.0)
            return tau_nom

        if slack > 1e-4:
            self._tlog.warn(f'OSCBF slack={slack:.3e}  n_cbf={A_rows.shape[0]}')

        return tau_safe

    def build_constraints(
        self,
        q       : np.ndarray,
        qdot    : np.ndarray,
        M_inv   : np.ndarray,
        J       : np.ndarray,
        C_qdot  : np.ndarray,
        p_ee    : np.ndarray,
        dJ_qdot : np.ndarray,
        now     : float,
    ) -> tuple[np.ndarray, np.ndarray]:
        """Assemble all active CBF constraint rows (A, b): A τ ≥ b."""
        all_A: list[np.ndarray] = []
        all_b: list[np.ndarray] = []

        # ── 1. Joint position HOCBF (always active) ───────────────────────────
        A_q, b_q = _build_joint_position_cbf(
            q, qdot, M_inv, C_qdot,
            self._q_min, self._q_max,
            self._alpha1, self._alpha2,
        )
        all_A.append(A_q)
        all_b.append(b_q)

        # ── 2. Joint velocity CBF (optional) ──────────────────────────────────
        if self._enable_vel_cbf:
            A_v, b_v = _build_joint_velocity_cbf(
                qdot, M_inv, C_qdot,
                self._qdot_min, self._qdot_max,
                self._alpha_vel,
            )
            all_A.append(A_v)
            all_b.append(b_v)

        # ── 3. EE workspace box CBF (optional) ────────────────────────────────
        if self._enable_ws_cbf:
            J3       = J[:3, :]          # 3×7 translational
            dJ3_qdot = dJ_qdot[:3]       # 3-vector
            A_w, b_w = _build_ee_workspace_cbf(
                q, qdot, p_ee,
                J3, dJ3_qdot,
                M_inv, C_qdot,
                self._ws_min, self._ws_max,
                self._alpha1, self._alpha2,
            )
            all_A.append(A_w)
            all_b.append(b_w)

        # ── 4. Obstacle avoidance CBF (optional) ──────────────────────────────
        if self._enable_obstacle_cbf and self._link_distances:
            dist_stale = (self._t_dist is None or
                          now - self._t_dist > self._dist_timeout)
            if dist_stale:
                self.get_logger().warn(
                    'Link distances stale — obstacle CBF skipped',
                    throttle_duration_sec=1.0)
            else:
                # Update CBF kinematics model for obstacle Jacobians
                self._update_kin_model(self.q, self.qdot)
                A_o, b_o = _build_obstacle_cbf(
                    self._link_distances,
                    self._kin_model, self._kin_data,
                    self.q, self.qdot, M_inv, C_qdot,
                    self._alpha1, self._alpha2,
                    d_safe=0.15,
                )
                if A_o.size > 0:
                    all_A.append(A_o)
                    all_b.append(b_o)

        if not all_A:
            return np.zeros((0, _N_ARM)), np.zeros(0)

        A_all = np.vstack([a for a in all_A if a.size > 0])
        b_all = np.concatenate([b for b in all_b if b.size > 0])
        return A_all, b_all

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _update_kin_model(self, q: np.ndarray, qdot: np.ndarray) -> None:
        """Run FK + Jacobian update on the no-hand kinematics model."""
        q_full   = pin.neutral(self._kin_model)
        qdot_full = np.zeros(self._kin_model.nv)
        for k in range(min(_N_ARM, self._kin_model.nv)):
            q_full[k]    = q[k]
            qdot_full[k] = qdot[k]
        pin.forwardKinematics(self._kin_model, self._kin_data, q_full, qdot_full)
        pin.computeJointJacobians(self._kin_model, self._kin_data, q_full)
        pin.computeJointJacobiansTimeVariation(
            self._kin_model, self._kin_data, q_full, qdot_full)
        pin.updateFramePlacements(self._kin_model, self._kin_data)

    def _publish(self, tau: np.ndarray) -> None:
        tau_clipped = np.clip(tau, self._tau_min, self._tau_max)
        for i in range(_N_ARM):
            self._tau_msg.data[i] = float(tau_clipped[i])
        self._pub.publish(self._tau_msg)


# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = OSCBFFilterNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
