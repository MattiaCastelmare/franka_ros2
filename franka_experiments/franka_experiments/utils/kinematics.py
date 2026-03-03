"""Pinocchio / URDF helpers for resolved-rate control."""

from __future__ import annotations

import os
import subprocess
import tempfile
from typing import List, Optional

import numpy as np

from .constants import FR3_JOINT_NAMES

try:
    import pinocchio as pin
except ImportError as exc:
    raise ImportError(
        'pinocchio is required but not installed. '
        'Install with: pip install pin'
    ) from exc


# ---------------------------------------------------------------------------
# URDF generation
# ---------------------------------------------------------------------------

def generate_urdf_from_xacro() -> str:
    """Generate a plain URDF string by running xacro on the FR3 xacro file."""
    from ament_index_python.packages import get_package_share_directory
    desc_share = get_package_share_directory('franka_description')
    xacro_path = os.path.join(desc_share, 'robots', 'fr3', 'fr3.urdf.xacro')
    if not os.path.isfile(xacro_path):
        raise FileNotFoundError(f'FR3 xacro not found at {xacro_path}')
    result = subprocess.run(
        ['xacro', xacro_path, 'hand:=true', 'ee_id:=franka_hand'],
        capture_output=True, text=True, check=True,
    )
    return result.stdout


# ---------------------------------------------------------------------------
# Model loading
# ---------------------------------------------------------------------------

def load_pinocchio_model(urdf_xml: str):
    """Build a Pinocchio model + data from a URDF XML string.

    Returns ``(model, data)`` tuple.
    """
    with tempfile.NamedTemporaryFile(
        mode='w', suffix='.urdf', delete=False
    ) as tmp:
        tmp.write(urdf_xml)
        tmp_path = tmp.name
    try:
        model = pin.buildModelFromUrdf(tmp_path)
    finally:
        os.unlink(tmp_path)
    data = model.createData()
    return model, data


# ---------------------------------------------------------------------------
# Frame / joint resolution
# ---------------------------------------------------------------------------

def resolve_frame_id(model, frame_name: str) -> int:
    """Return the Pinocchio frame ID for *frame_name*.

    Raises ``RuntimeError`` with a list of available frames if not found.
    """
    if not model.existFrame(frame_name):
        available = [model.frames[i].name for i in range(model.nframes)]
        raise RuntimeError(
            f'Frame "{frame_name}" not found in model.\n'
            f'Available frames: {available}')
    return model.getFrameId(frame_name)


def resolve_arm_joint_ids(model) -> List[int]:
    """Return Pinocchio joint IDs for the 7 FR3 arm joints.

    Raises ``RuntimeError`` if any joint is missing.
    """
    ids: List[int] = []
    for jname in FR3_JOINT_NAMES:
        if not model.existJointName(jname):
            raise RuntimeError(f'Joint "{jname}" not in Pinocchio model')
        ids.append(model.getJointId(jname))
    return ids


# ---------------------------------------------------------------------------
# Forward kinematics & Jacobian
# ---------------------------------------------------------------------------

def compute_ee_fk(model, data, q_full: np.ndarray, ee_frame_id: int):
    """Forward kinematics — return the SE3 placement of the EE frame.

    .. note:: Modifies *data* in place (standard Pinocchio pattern).
    """
    pin.forwardKinematics(model, data, q_full)
    pin.updateFramePlacement(model, data, ee_frame_id)
    return data.oMf[ee_frame_id]


def compute_arm_jacobian(
    model, data, q_full: np.ndarray,
    ee_frame_id: int, pin_joint_ids: List[int],
) -> np.ndarray:
    """Compute the 6×7 Jacobian for the 7 arm joints (LOCAL_WORLD_ALIGNED).

    .. note:: Modifies *data* in place.
    """
    J_full = pin.computeFrameJacobian(
        model, data, q_full, ee_frame_id,
        pin.ReferenceFrame.LOCAL_WORLD_ALIGNED,
    )  # 6 × nv
    arm_v_ids = [model.joints[pid].idx_v for pid in pin_joint_ids]
    return J_full[:, arm_v_ids]  # 6 × 7


def transform_ee_to_frame(
    model, data,
    ref_frame_id: int,
    oMee,
    J_arm_6x7: np.ndarray,
):
    """Express EE position and translational Jacobian in a reference frame.

    Parameters
    ----------
    ref_frame_id : int
        Pinocchio frame ID of the reference frame.
    oMee : SE3
        EE placement in world (returned by :func:`compute_ee_fk`).
    J_arm_6x7 : ndarray (6×7)
        Arm Jacobian in world frame.

    Returns
    -------
    p_ee : ndarray (3,)
        EE position expressed in the reference frame.
    J_pos : ndarray (3×7)
        Translational Jacobian expressed in the reference frame.
    """
    pin.updateFramePlacement(model, data, ref_frame_id)
    oMref = data.oMf[ref_frame_id]
    p_ee = np.array(oMref.actInv(oMee.translation))
    R_ref = np.array(oMref.rotation)
    J_pos = R_ref.T @ J_arm_6x7[:3, :]
    return p_ee, J_pos


# ---------------------------------------------------------------------------
# Damped-least-squares solver
# ---------------------------------------------------------------------------

def dls_solve(
    J_pos: np.ndarray,
    v_cmd: np.ndarray,
    damping: float,
    damping_boost: float = 0.0,
) -> Optional[np.ndarray]:
    """Damped-least-squares solve:  ``J_pos @ qdot ≈ v_cmd``.

    Returns ``None`` on numerical failure (singular + non-finite result).
    """
    lam = damping + damping_boost
    try:
        cond = np.linalg.cond(J_pos)
        if cond > 100.0:
            lam = max(lam, 0.1)
    except np.linalg.LinAlgError:
        lam = 0.1

    n_rows = J_pos.shape[0]
    JJt = J_pos @ J_pos.T + (lam ** 2) * np.eye(n_rows)
    try:
        J_pinv = J_pos.T @ np.linalg.inv(JJt)
    except np.linalg.LinAlgError:
        try:
            J_pinv = np.linalg.pinv(J_pos)
        except np.linalg.LinAlgError:
            return None

    qdot = J_pinv @ v_cmd
    if not np.all(np.isfinite(qdot)):
        return None
    return qdot
