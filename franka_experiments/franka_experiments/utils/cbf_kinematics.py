import numpy as np
import pinocchio as pin


def _skew(v: np.ndarray) -> np.ndarray:
    return np.array([[ 0.0, -v[2],  v[1]],
                     [ v[2],  0.0, -v[0]],
                     [-v[1],  v[0],  0.0]])


class CBFKinematics:
    def __init__(self, model: pin.Model):
        self.model = model
        self.data = model.createData()

    def update(self, q: np.ndarray, qdot: np.ndarray) -> None:
        """Single pass per tick: FK + Jacobians + Jdot."""
        pin.forwardKinematics(self.model, self.data, q, qdot)
        pin.computeJointJacobians(self.model, self.data, q)
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
