"""The two nominal laws of cbf_scenarios.run_scenario, as functions. No rclpy.

They are inline in ``run_scenario`` (cbf_scenarios.py:355-374), so they cannot
be imported. They are restated here term for term, and
test_scenario_laws.py executes the EXACT source lines extracted from
cbf_scenarios on random states and requires bit-equal results:

    joint PD     q̈ = −12 (q − goal) − 2√12 q̇
    Cartesian    a  = 25 e + 10 (v_ref − v),  J⁺ = Jᵀ (J Jᵀ + 0.05² I)⁻¹
                 q̈ = J⁺ (a − J̇ q̇) − 6 (I − J⁺ J) q̇

The Cartesian law runs on the flange (fr3_link8 origin), with the point
Jacobian of the same no-hand Pinocchio model (``kinematics.build_urdf_no_hand``,
``CBFKinematics.point_jacobian``).
"""
from __future__ import annotations

import numpy as np

KQ = 12.0          # joint PD stiffness
KP_CART = 25.0
KD_CART = 10.0
LAMBDA = 0.05      # damped pseudo-inverse
KN = 6.0           # null-space damping


def joint_pd(q, qdot, goal) -> np.ndarray:
    q, qdot, goal = (np.asarray(x, float) for x in (q, qdot, goal))
    return -KQ * (q - goal) - 2.0 * np.sqrt(KQ) * qdot


def cartesian(Jp, Jpd, qdot, e, v_ref, v_ee) -> np.ndarray:
    Jp, Jpd = np.asarray(Jp, float), np.asarray(Jpd, float)
    qdot = np.asarray(qdot, float)
    a_task = KP_CART * np.asarray(e, float) + KD_CART * (np.asarray(v_ref, float)
                                                          - np.asarray(v_ee, float))
    JJt = Jp @ Jp.T + (LAMBDA ** 2) * np.eye(3)
    J_pinv = Jp.T @ np.linalg.inv(JJt)
    qddot = J_pinv @ (a_task - Jpd @ qdot)
    qddot -= KN * (np.eye(Jp.shape[1]) - J_pinv @ Jp) @ qdot
    return qddot


class FlangeKinematics:
    """fr3_link8 position and point Jacobian, exactly as cbf_scenarios.Filter builds them."""

    def __init__(self):
        import pinocchio as pin
        from franka_experiments.utils.kinematics import CBFKinematics, build_urdf_no_hand
        self.kin = CBFKinematics(pin.buildModelFromUrdf(build_urdf_no_hand()))
        self.fid = self.kin.resolve_frame_id('fr3_link8')

    def update(self, q, qdot):
        self.kin.update(np.asarray(q, float), np.asarray(qdot, float), with_jdot=True)
        flange = self.kin.data.oMf[self.fid].translation.copy()
        Jp, Jpd = self.kin.point_jacobian(self.fid, flange)
        return flange, Jp, Jpd


class NominalLaw:
    """One scenario's nominal, evaluated against the episode clock."""

    def __init__(self, spec: dict, q0, kin: FlangeKinematics | None = None):
        self.spec = dict(spec)
        self.law = self.spec.get('law', 'joint_pd')
        self.q0 = np.asarray(q0, float)
        if self.law not in ('joint_pd', 'cartesian'):
            raise ValueError(f'scenario commander law {self.law!r} (pentagon runs its own node)')
        self.goal = np.asarray(self.spec.get('goal_q', q0), float)
        self.cart_ref = np.asarray(self.spec.get('cart_ref', (0, 0, 0)), float)
        self.kin = kin if kin is not None or self.law != 'cartesian' else FlangeKinematics()
        self.p_ref0 = None

    def hold(self, q, qdot) -> np.ndarray:
        """Before the episode: regulate to Q0 with the same joint PD."""
        return joint_pd(q, qdot, self.q0)

    def __call__(self, t_ep: float, q, qdot):
        """(q̈_nom, p_ref or None, flange or None) at episode time t_ep >= 0."""
        if self.law == 'joint_pd':
            return joint_pd(q, qdot, self.goal), None, None
        flange, Jp, Jpd = self.kin.update(q, qdot)
        if self.p_ref0 is None:
            self.p_ref0 = flange.copy()
        p_ref = self.p_ref0 + self.cart_ref * t_ep
        v_ee = Jp @ np.asarray(qdot, float)
        return cartesian(Jp, Jpd, qdot, p_ref - flange, self.cart_ref, v_ee), p_ref, flange
