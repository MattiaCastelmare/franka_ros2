import numpy as np
import pinocchio as pin
import pytest

from franka_experiments.utils.cbf_kinematics import CBFKinematics
from franka_experiments.utils.cbf_constraints import build_hocbf_constraints, CBFParams

URDF_PATH = "/ros2_ws/src/franka_hardware/test/fr3.urdf"


@pytest.fixture(scope="module")
def kin():
    model = pin.buildModelFromUrdf(URDF_PATH)
    return CBFKinematics(model)


def test_build_hocbf_constraints(kin):
    model = kin.model
    q    = pin.neutral(model)
    qdot = np.ones(model.nv) * 0.1

    kin.update(q, qdot)

    frame_id = kin.resolve_frame_id("fr3_link7")
    assert frame_id is not None, "Frame 'fr3_link7' not found in model"

    p_robot = kin.data.oMf[frame_id].translation.copy()
    p_human = p_robot + np.array([0.3, 0.0, 0.0])

    link_distances = [{
        'link_name':            'fr3_link7',
        'closest_point_robot':  p_robot,
        'closest_point_human':  p_human,
        'distance':             0.3,
        'valid':                True,
        'confidence':           1.0,
        'zone':                 'warning',
    }]

    params = CBFParams(k0=25.0, k1=10.0, d_safe_default=0.15)
    A, b, meta = build_hocbf_constraints(
        kin, q, qdot, link_distances,
        p_h_dot_est=None, p_h_ddot_est=None,
        cbf_params=params,
    )

    assert A.shape == (1, 7), f"Expected A shape (1, 7), got {A.shape}"
    assert np.isfinite(b[0]), "b[0] is not finite"
    assert abs(meta[0]['h'] - 0.15) < 1e-6, (
        f"h = {meta[0]['h']:.8f}, expected 0.15")

    # n_i = (p_robot - p_human) / ||...|| = [-1, 0, 0]
    Jp, _ = kin.point_jacobian(frame_id, p_robot)
    n_i = np.array([-1.0, 0.0, 0.0])
    expected_hdot = float(n_i @ (Jp @ qdot))
    assert abs(meta[0]['hdot'] - expected_hdot) < 1e-6, (
        f"hdot = {meta[0]['hdot']:.8f}, expected {expected_hdot:.8f}")

    assert np.all(np.isfinite(A[0])), "A[0] contains non-finite values"
