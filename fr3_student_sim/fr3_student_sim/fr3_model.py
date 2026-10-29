"""Kinematic and dynamic model of the simulated FR3 (Pinocchio).

The model is built from the very URDF that Gazebo simulates (see
:mod:`fr3_student_sim.robot_description`), so M(q), C(q, dq) and g(q) here are
the plant's own, rotor inertia included.

All quantities are expressed in the world frame (= robot base, ``fr3_link0``).
Joint vectors have 7 entries, ordered fr3_joint1 ... fr3_joint7.

Gravity convention (as on the real FR3): the simulator ADDS g(q) to every torque
you command. Do not add ``g(q)`` to your command. In the dynamics

    M(q) ddq + C(q, dq) dq + g(q) = tau_motor + tau_ext,   tau_motor = tau_cmd + g(q)

so  M ddq + C dq = tau_cmd + tau_ext.

Example::

    from fr3_student_sim.fr3_model import FR3Model
    robot = FR3Model()
    p = robot.position(q)          # (3,)  tool point [m]
    J = robot.jacobian_pos(q)      # (3, 7)
    M = robot.mass_matrix(q)       # (7, 7)
"""
from __future__ import annotations

import numpy as np
import pinocchio as pin

GRAVITY = 9.8          # [m/s^2] the value Gazebo and the plugin's KDL model use
TOOL_FRAME = 'fr3_fingertip'   # tip of the fingers, 9.5 mm beyond fr3_hand_tcp

#: FR3 datasheet limits.
TAU_MAX = np.array([87.0, 87.0, 87.0, 87.0, 12.0, 12.0, 12.0])        # [N m]
DQ_MAX = np.array([2.62, 2.62, 2.62, 2.62, 5.26, 4.18, 5.26])        # [rad/s]
Q_MIN = np.array([-2.7437, -1.7837, -2.9007, -3.0421, -2.8065, 0.5445, -3.0159])
Q_MAX = np.array([2.7437, 1.7837, 2.9007, -0.1518, 2.8065, 4.5169, 3.0159])


class FR3Model:
    """Pinocchio wrapper with plain-numpy inputs and outputs."""

    def __init__(self, urdf_xml: str | None = None, tool_frame: str = TOOL_FRAME):
        if urdf_xml is None:
            from fr3_student_sim.robot_description import build_urdf, process_xacro
            urdf_xml = build_urdf(process_xacro())
        self.model = pin.buildModelFromXML(urdf_xml)
        self.model.gravity.linear = np.array([0.0, 0.0, -GRAVITY])
        self.data = self.model.createData()
        if self.model.nq != 7:
            raise ValueError(f'expected 7 joints, the URDF has {self.model.nq}')
        self.tool_frame = tool_frame
        self._fid = {}
        self.frame_id(tool_frame)

    # ── frames ────────────────────────────────────────────────────────────────
    def frame_id(self, name: str) -> int:
        if name not in self._fid:
            if not self.model.existFrame(name):
                raise KeyError(f'no frame {name!r} in the model')
            self._fid[name] = self.model.getFrameId(name)
        return self._fid[name]

    # ── kinematics ────────────────────────────────────────────────────────────
    def pose(self, q, frame: str | None = None):
        """(p (3,), R (3,3)) of ``frame`` (default: the tool point)."""
        fid = self.frame_id(frame or self.tool_frame)
        pin.forwardKinematics(self.model, self.data, np.asarray(q, float))
        pin.updateFramePlacement(self.model, self.data, fid)
        T = self.data.oMf[fid]
        return T.translation.copy(), T.rotation.copy()

    def position(self, q, frame: str | None = None) -> np.ndarray:
        return self.pose(q, frame)[0]

    def jacobian(self, q, frame: str | None = None) -> np.ndarray:
        """(6, 7) geometric Jacobian [linear; angular], world-aligned axes."""
        fid = self.frame_id(frame or self.tool_frame)
        return pin.computeFrameJacobian(self.model, self.data, np.asarray(q, float), fid,
                                        pin.LOCAL_WORLD_ALIGNED).copy()

    def jacobian_pos(self, q, frame: str | None = None) -> np.ndarray:
        """(3, 7) linear-velocity Jacobian of the frame origin."""
        return self.jacobian(q, frame)[:3]

    def velocity(self, q, dq, frame: str | None = None) -> np.ndarray:
        """(3,) linear velocity of the frame origin."""
        return self.jacobian_pos(q, frame) @ np.asarray(dq, float)

    def jdot_dq(self, q, dq, frame: str | None = None) -> np.ndarray:
        """(3,) dJ/dt * dq of the linear part (the Cartesian acceleration at ddq = 0)."""
        fid = self.frame_id(frame or self.tool_frame)
        q, dq = np.asarray(q, float), np.asarray(dq, float)
        pin.forwardKinematics(self.model, self.data, q, dq, np.zeros(7))
        a = pin.getFrameClassicalAcceleration(self.model, self.data, fid,
                                              pin.LOCAL_WORLD_ALIGNED)
        return a.linear.copy()

    # ── dynamics ──────────────────────────────────────────────────────────────
    def mass_matrix(self, q) -> np.ndarray:
        """(7, 7) joint-space inertia matrix M(q)."""
        M = pin.crba(self.model, self.data, np.asarray(q, float))
        return np.triu(M) + np.triu(M, 1).T

    def mass_matrix_inv(self, q) -> np.ndarray:
        Mi = pin.computeMinverse(self.model, self.data, np.asarray(q, float))
        return np.triu(Mi) + np.triu(Mi, 1).T

    def coriolis_matrix(self, q, dq) -> np.ndarray:
        """(7, 7) C(q, dq) with dM/dt - 2C skew-symmetric (as the momentum observer needs)."""
        return pin.computeCoriolisMatrix(self.model, self.data, np.asarray(q, float),
                                         np.asarray(dq, float)).copy()

    def coriolis(self, q, dq) -> np.ndarray:
        """(7,) C(q, dq) dq."""
        q, dq = np.asarray(q, float), np.asarray(dq, float)
        return pin.rnea(self.model, self.data, q, dq, np.zeros(7)) - self.gravity(q)

    def gravity(self, q) -> np.ndarray:
        """(7,) g(q). Only for analysis: the simulator already adds it to your command."""
        return pin.computeGeneralizedGravity(self.model, self.data, np.asarray(q, float)).copy()

    # ── effective mass ────────────────────────────────────────────────────────
    def effective_mass(self, q, n, frame: str | None = None) -> float:
        """Directional effective mass 1 / (n^T J M^-1 J^T n) at the frame origin."""
        n = np.asarray(n, float) / np.linalg.norm(n)
        J = self.jacobian_pos(q, frame)
        return 1.0 / float(n @ J @ self.mass_matrix_inv(q) @ J.T @ n)

    def kinetic_energy(self, q, dq) -> float:
        dq = np.asarray(dq, float)
        return 0.5 * float(dq @ self.mass_matrix(q) @ dq)
