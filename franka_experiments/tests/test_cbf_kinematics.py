import numpy as np
import pinocchio as pin
import pytest

from franka_experiments.utils.cbf_kinematics import CBFKinematics

URDF_PATH = "/ros2_ws/src/franka_hardware/test/fr3.urdf"


@pytest.fixture(scope="module")
def kin():
    model = pin.buildModelFromUrdf(URDF_PATH)
    return CBFKinematics(model)


def test_point_jacobian_at_origin(kin):
    model = kin.model
    q    = pin.neutral(model)
    qdot = np.ones(model.nv) * 0.05

    kin.update(q, qdot)

    frame_id = kin.resolve_frame_id("fr3_link7")
    assert frame_id is not None, "Frame 'fr3_link7' not found in model"

    p_world = kin.data.oMf[frame_id].translation.copy()
    Jp, Jpd = kin.point_jacobian(frame_id, p_world)

    J6 = pin.getFrameJacobian(
        model, kin.data, frame_id,
        pin.ReferenceFrame.LOCAL_WORLD_ALIGNED,
    )

    assert Jp.shape == (3, 7), f"Expected shape (3, 7), got {Jp.shape}"
    assert not np.any(np.isnan(Jp)), "Jp contains NaN"
    assert np.allclose(Jp, J6[:3, :], atol=1e-10), (
        f"Jp mismatch with J6[:3,:]\n"
        f"max abs diff = {np.abs(Jp - J6[:3, :]).max():.2e}"
    )
