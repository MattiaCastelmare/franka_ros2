"""Pinocchio / URDF helpers and joint-state management.

``JointStateManager`` (merged from ``joint_state.py``) lives here so that
all Pinocchio-dependent code is in a single module.
"""

from __future__ import annotations

import glob
import os
import subprocess
import tempfile
import time
from typing import List, Optional

import numpy as np

from rclpy.node import Node as RclpyNode
from sensor_msgs.msg import JointState

from .constants import AUTO_SENTINEL, FR3_JOINT_NAMES
from .math_utils import skew as _skew
from .node_runtime import get_namespace_from_config

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


def load_pinocchio_model_from_urdf_path(urdf_path: str):
    """Build a Pinocchio model + data directly from a URDF file path.

    Returns ``(model, data)`` tuple.
    """
    if not os.path.isfile(urdf_path):
        raise FileNotFoundError(f'URDF not found: {urdf_path}')
    model = pin.buildModelFromUrdf(urdf_path)
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
# SO(3) logarithm (axis*angle) — robust orientation error
# ---------------------------------------------------------------------------

def so3_log(R_err: np.ndarray, eps: float = 1e-6) -> np.ndarray:
    """SO(3) logarithm: return ``axis * angle`` of the rotation ``R_err``.

    This is the exact (large-angle) generalisation of the classic small-angle
    orientation error ``½·vee(R_err − R_errᵀ)``: for small angles it reduces to
    exactly that vector (the ``angle < eps`` branch), but its magnitude equals
    the TRUE geodesic angle and is therefore **monotone over the whole
    ``[0, π]`` range** — unlike the bare antisymmetric vee, whose norm is
    ``sin(angle)`` and folds back to zero at π (non-monotone above 90°, which
    inverts a PD gradient that minimises it).

    Robust at both singular endpoints:
      * ``angle → 0``:  ``½·vee`` (continuous limit of ``angle/(2 sin angle)``).
      * ``angle → π``:  ``sin(angle) → 0`` makes the vee term vanish (0/0), so
        the axis is extracted from ``R_err + I = 2·a·aᵀ`` (largest diagonal
        column, for numerical stability), with the sign disambiguated from the
        (vanishing) antisymmetric part when usable.

    Notes
    -----
    The returned vector lives in the SAME frame ``R_err`` is written in. For a
    world-frame feedback loop combined with a LOCAL_WORLD_ALIGNED Jacobian, the
    caller must rotate it into world (e.g. ``−R_des @ so3_log(R_des.T @ R_cur)``
    ``= so3_log(R_des @ R_cur.T)``).
    """
    cos_t = np.clip((np.trace(R_err) - 1.0) / 2.0, -1.0, 1.0)
    angle = np.arccos(cos_t)
    vee = np.array([R_err[2, 1] - R_err[1, 2],
                    R_err[0, 2] - R_err[2, 0],
                    R_err[1, 0] - R_err[0, 1]])
    if angle < eps:
        return 0.5 * vee
    if (np.pi - angle) < eps:
        A = (R_err + np.eye(3)) / 2.0          # = a·aᵀ  (a = unit axis)
        k = int(np.argmax(np.diag(A)))
        axis = A[:, k] / np.sqrt(max(A[k, k], 0.0))
        axis = axis / np.linalg.norm(axis)
        if np.dot(axis, vee) < 0.0:            # keep continuity with the else-branch
            axis = -axis
        return angle * axis
    axis = vee / (2.0 * np.sin(angle))
    return angle * axis


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
        # Smooth ramp: extra damping grows linearly as cond goes from 50→300.
        # Avoids the discontinuous lambda jump that causes qdot spikes near
        # singularities.  At cond=50 boost=0; at cond≥300 boost=0.1.
        cond_lo, cond_hi = 50.0, 300.0
        boost = 0.1 * float(np.clip(
            (cond - cond_lo) / (cond_hi - cond_lo), 0.0, 1.0))
        lam = max(lam, boost)
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


# ---------------------------------------------------------------------------
# Joint-state subscription manager (merged from joint_state.py)
# ---------------------------------------------------------------------------

class JointStateManager:
    """Manages joint-state subscription with optional auto-detection.

    Composes into any ``Node`` — does **not** inherit from ``Node``.

    Parameters
    ----------
    node : rclpy.node.Node
        Parent ROS 2 node (used for subscriptions, timers, logging).
    pin_model, pin_joint_ids
        Pinocchio model and list of FR3 arm joint IDs.
    topic_param : str
        If ``AUTO_SENTINEL``, auto-detection is used; otherwise subscribe
        directly to the given topic.
    """

    def __init__(
        self,
        node: RclpyNode,
        pin_model,
        pin_joint_ids: List[int],
        *,
        topic_param: str = AUTO_SENTINEL,
    ) -> None:
        self._node = node
        self._pin_model = pin_model
        self._pin_joint_ids = pin_joint_ids

        # Public read-only state
        self._q: Optional[np.ndarray] = None
        self._q_full: Optional[np.ndarray] = None
        self._stamp = node.get_clock().now()

        # Internal
        self._js_index_map: Optional[List[int]] = None
        self._sub: Optional[object] = None
        self._topic_resolved: Optional[str] = None
        self._auto_detect = (topic_param == AUTO_SENTINEL)
        self._candidates: List[str] = []
        self._on_fallback = False

        if self._auto_detect:
            ns = get_namespace_from_config()
            if ns:
                self._candidates.append(f'/{ns}/franka/joint_states')
                self._candidates.append(f'/{ns}/joint_states')
            self._candidates.append('/franka/joint_states')
            self._candidates.append('/joint_states')

            self._discovery_start = time.monotonic()
            self._discovery_timer = node.create_timer(
                1.0, self._discover_topic)
            node.get_logger().info(
                f'Joint-state auto-detect enabled  '
                f'candidates={self._candidates}')
        else:
            self._topic_resolved = topic_param
            self._sub = node.create_subscription(
                JointState, topic_param, self._joint_state_cb, 10)
            node.get_logger().info(
                f'Joint-state topic (explicit): {topic_param}')

    # -- Properties --------------------------------------------------------

    @property
    def q(self) -> Optional[np.ndarray]:
        """Latest 7 arm joint positions, or ``None``."""
        return self._q

    @property
    def q_full(self) -> Optional[np.ndarray]:
        """Latest full Pinocchio q vector, or ``None``."""
        return self._q_full

    @property
    def stamp(self):
        """ROS Time of last joint-state update."""
        return self._stamp

    @property
    def topic_resolved(self) -> Optional[str]:
        """Currently subscribed topic (or ``None`` if not yet resolved)."""
        return self._topic_resolved

    # -- Discovery ---------------------------------------------------------

    def _discover_topic(self) -> None:
        """Try to find a JointState topic on the ROS graph (called at 1 Hz)."""
        if self._q is not None:
            self._node.get_logger().info(
                f'Receiving joint states on '
                f'{self._topic_resolved} — discovery complete')
            self._discovery_timer.cancel()
            return

        available = self._node.get_topic_names_and_types()
        js_topics = {
            name
            for name, types in available
            if 'sensor_msgs/msg/JointState' in types
        }

        chosen: Optional[str] = None
        for candidate in self._candidates:
            if candidate in js_topics:
                chosen = candidate
                break

        if chosen is None:
            extras = sorted(js_topics - set(self._candidates))
            if extras:
                chosen = extras[0]

        if chosen is not None and chosen != self._topic_resolved:
            self._subscribe(chosen, source='auto-detected')
            self._on_fallback = False
            return

        elapsed = time.monotonic() - self._discovery_start
        if elapsed > 2.0 and self._sub is None:
            fallback = '/joint_states'
            self._subscribe(fallback, source='fallback (2 s timeout)')
            self._on_fallback = True
            self._node.get_logger().warn(
                f'No JointState topic discovered after {elapsed:.1f} s\n'
                f'  candidates : {self._candidates}\n'
                f'  graph      : {sorted(js_topics) or "(none)"}\n'
                f'  → falling back to {fallback}, will keep retrying …')

    def _subscribe(self, topic: str, *, source: str = '') -> None:
        """(Re-)create the JointState subscription on *topic*."""
        if self._sub is not None:
            self._node.destroy_subscription(self._sub)
        self._sub = self._node.create_subscription(
            JointState, topic, self._joint_state_cb, 10)
        self._topic_resolved = topic
        self._node.get_logger().info(
            f'Joint-state topic: {topic}  ({source})\n'
            f'  candidates tried: {self._candidates}')

    # -- Callback ----------------------------------------------------------

    def _joint_state_cb(self, msg: JointState) -> None:
        """Store latest joint positions for the 7 arm joints."""
        if self._js_index_map is None:
            try:
                self._js_index_map = [
                    msg.name.index(jn) for jn in FR3_JOINT_NAMES
                ]
            except ValueError:
                return

        if len(msg.position) < max(self._js_index_map) + 1:
            return

        q7 = np.array([msg.position[i] for i in self._js_index_map])
        self._q = q7

        q_full = pin.neutral(self._pin_model)
        for k, pid in enumerate(self._pin_joint_ids):
            idx_q = self._pin_model.joints[pid].idx_q
            q_full[idx_q] = q7[k]
        self._q_full = q_full
        self._stamp = self._node.get_clock().now()

    # -- Cleanup -----------------------------------------------------------

    def cancel_discovery(self) -> None:
        """Cancel the discovery timer (call during shutdown)."""
        if self._auto_detect and hasattr(self, '_discovery_timer'):
            try:
                self._discovery_timer.cancel()
            except Exception:
                pass


# MOVED here from utils/cbf_kinematics.py (Phase 2): point Jacobians are
# part of the FR3 robot model, not a separate concern.
class CBFKinematics:
    def __init__(self, model: pin.Model):
        self.model = model
        self.data = model.createData()

    def update(self, q: np.ndarray, qdot: np.ndarray,
               with_jdot: bool = True) -> None:
        """Single pass per tick: FK + Jacobians (+ Jdot unless with_jdot=False)."""
        pin.forwardKinematics(self.model, self.data, q, qdot)
        pin.computeJointJacobians(self.model, self.data, q)
        if with_jdot:
            pin.computeJointJacobiansTimeVariation(self.model, self.data, q, qdot)
        pin.updateFramePlacements(self.model, self.data)

    def resolve_frame_id(self, link_name: str) -> int | None:
        """Return frame id for link_name; try exact match, then substring."""
        if self.model.existFrame(link_name):
            return self.model.getFrameId(link_name)
        for frame in self.model.frames:
            if link_name in frame.name:
                return self.model.getFrameId(frame.name)
        return None

    def prepare_frame_cache(self, link_names: list) -> None:
        pass

    def point_jacobian_pos(self, frame_id: int, p_world: np.ndarray) -> np.ndarray:
        """Return J_p only, shape (3, nv) — valid after update(with_jdot=False)."""
        oMf = self.data.oMf[frame_id]
        r_cross = _skew(p_world - oMf.translation)
        J6 = pin.getFrameJacobian(
            self.model, self.data, frame_id,
            pin.ReferenceFrame.LOCAL_WORLD_ALIGNED,
        )
        return J6[:3, :] - r_cross @ J6[3:, :]

    def point_jacobian(
        self, frame_id: int, p_world: np.ndarray
    ) -> tuple[np.ndarray, np.ndarray]:
        """Return (J_p, Jdot_p) shape (3, nv) for point p_world on the link.
        update() must be called first.
        """
        oMf = self.data.oMf[frame_id]
        r = p_world - oMf.translation
        r_cross = _skew(r)

        J6 = pin.getFrameJacobian(
            self.model, self.data, frame_id,
            pin.ReferenceFrame.LOCAL_WORLD_ALIGNED,
        )
        Jdot6 = pin.getFrameJacobianTimeVariation(
            self.model, self.data, frame_id,
            pin.ReferenceFrame.LOCAL_WORLD_ALIGNED,
        )
        Jv,  Jw  = J6[:3, :],    J6[3:, :]
        Jvd, Jwd = Jdot6[:3, :], Jdot6[3:, :]

        Jp   = Jv  - r_cross @ Jw
        Jpd  = Jvd - r_cross @ Jwd   # first-order approx; exact for fixed r
        return Jp, Jpd


def build_urdf_no_hand() -> str:
    """Generate a hand-less FR3 URDF and return the path to the temp file.

    MOVED here from ``nodes/cbf_safety_filter.py`` in Phase 3: URDF generation
    is part of the robot model, not of the CBF filter. Body unchanged.

    Returns:
        Absolute path to a freshly written temporary ``.urdf`` file. The caller
        owns the file; it is not deleted automatically.

    Raises:
        RuntimeError: If ``fr3.urdf.xacro`` cannot be found under the
            ``franka_description`` share directory, or if ``xacro`` fails.
    """
    from ament_index_python.packages import get_package_share_directory
    share = get_package_share_directory('franka_description')
    xs = glob.glob(os.path.join(share, '**', 'fr3.urdf.xacro'), recursive=True)
    if not xs:
        raise RuntimeError('fr3.urdf.xacro not found under franka_description share')
    tmp = tempfile.NamedTemporaryFile(suffix='.urdf', delete=False, prefix='fr3_cbf_')
    tmp.close()
    r = subprocess.run(['xacro', xs[0], 'hand:=false', '-o', tmp.name],
                       capture_output=True, text=True)
    if r.returncode != 0:
        raise RuntimeError(f'xacro failed:\n{r.stderr}')
    return tmp.name
