"""FR3 forward kinematics and Jacobian — URDF-based, numpy only.

Builds the kinematic chain directly from the joint origins in the FR3 URDF
(no DH parameters, no external libraries).

Each joint transform:
    T_i(q_i) = Trans(xyz_i) @ Rx(rpy_i) @ Rz(q_i)

where Trans, Rx, Rz are standard 4×4 homogeneous matrices and all FR3 joints
rotate about their local Z-axis (type=revolute, axis=[0,0,1]).

Joint origins (from franka_description/robots/fr3/fr3.urdf):
    joint1: xyz=[0,     0,      0.333],  rpy=[0,       0, 0]
    joint2: xyz=[0,     0,      0    ],  rpy=[-π/2,    0, 0]
    joint3: xyz=[0,    -0.316,  0    ],  rpy=[+π/2,    0, 0]
    joint4: xyz=[0.0825, 0,     0    ],  rpy=[+π/2,    0, 0]
    joint5: xyz=[-0.0825, 0.384, 0   ],  rpy=[-π/2,    0, 0]
    joint6: xyz=[0,     0,      0    ],  rpy=[+π/2,    0, 0]
    joint7: xyz=[0.088, 0,      0    ],  rpy=[+π/2,    0, 0]
    flange: xyz=[0,     0,      0.107],  rpy=[0,       0, 0]  (fixed joint fr3_joint8)

Geometric Jacobian — translational part (3×7):
    J[:, i] = z_i × (p_EE − p_i)
where z_i = z-axis of the frame BEFORE joint i rotates,
      p_i = origin of that frame.

Damped-least-squares pseudoinverse (DLS):
    qdot = Jᵀ (J Jᵀ + λ² I)⁻¹ xdot
with variable λ that increases near singularities.
"""

import math
import numpy as np

# ── Utility transforms ─────────────────────────────────────────────────────────

def _rx(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1., 0., 0., 0.],
                     [0., c, -s,  0.],
                     [0., s,  c,  0.],
                     [0., 0., 0., 1.]])

def _rz(q: float) -> np.ndarray:
    c, s = math.cos(q), math.sin(q)
    return np.array([[ c, -s, 0., 0.],
                     [ s,  c, 0., 0.],
                     [0., 0., 1., 0.],
                     [0., 0., 0., 1.]])

def _trans(x: float, y: float, z: float) -> np.ndarray:
    return np.array([[1., 0., 0., x],
                     [0., 1., 0., y],
                     [0., 0., 1., z],
                     [0., 0., 0., 1.]])

# ── Constant offset transforms (independent of joint angles) ──────────────────
# T_offset_i = Trans(xyz_i) @ Rx(rpy_i[0])   (rpy[1]=rpy[2]=0 for all FR3 joints)

_OFFSETS: tuple = (
    _trans(0.0,    0.0,    0.333) @ _rx( 0.0        ),   # joint 1
    _trans(0.0,    0.0,    0.0  ) @ _rx(-math.pi/2  ),   # joint 2
    _trans(0.0,   -0.316,  0.0  ) @ _rx( math.pi/2  ),   # joint 3
    _trans(0.0825, 0.0,    0.0  ) @ _rx( math.pi/2  ),   # joint 4
    _trans(-0.0825, 0.384, 0.0  ) @ _rx(-math.pi/2  ),   # joint 5
    _trans(0.0,    0.0,    0.0  ) @ _rx( math.pi/2  ),   # joint 6
    _trans(0.088,  0.0,    0.0  ) @ _rx( math.pi/2  ),   # joint 7
    _trans(0.0,    0.0,    0.107),                        # flange (fixed)
)

# FR3 joint velocity limits [rad/s] (Franka documentation)
QDOT_MAX = np.array([2.62, 2.62, 2.62, 2.62, 5.26, 4.18, 5.26])

# Home pose matching Gazebo xacro initial_position values
Q_HOME = np.array([0.0, -math.pi/4, 0.0, -3*math.pi/4, 0.0, math.pi/2, math.pi/4])


# ── Public API ────────────────────────────────────────────────────────────────

def fk(q7: np.ndarray):
    """Forward kinematics to the FR3 flange.

    Parameters
    ----------
    q7 : array (7,)  joint angles [rad]

    Returns
    -------
    pos : array (3,)   EE / flange position in world frame [m]
    T   : array (4, 4) homogeneous transform T_world_EE
    """
    T = np.eye(4)
    for i in range(7):
        T = T @ _OFFSETS[i] @ _rz(q7[i])
    T = T @ _OFFSETS[7]     # fixed flange offset
    return T[:3, 3].copy(), T


def jacobian(q7: np.ndarray):
    """Positional geometric Jacobian (3×7).

    For each revolute joint i, the contribution to the Jacobian is:
        J[:, i] = z_i × (p_EE − p_i)
    where z_i is the z-axis of the frame BEFORE joint i applies its rotation,
    and p_i is the origin of that frame in world coordinates.

    Parameters
    ----------
    q7 : array (7,)  joint angles [rad]

    Returns
    -------
    J   : array (3, 7)  positional Jacobian
    pos : array (3,)    EE position [m]
    """
    # T_before[i] = T_world_{frame before joint i rotates}
    #             = (product of joints 0..i-1) @ OFFSET_i
    T = np.eye(4)
    T_before = []
    frames_after = []
    for i in range(7):
        T_off = T @ _OFFSETS[i]        # apply constant offset
        T_before.append(T_off.copy())  # frame before joint i spins
        T = T_off @ _rz(q7[i])        # apply joint rotation
        frames_after.append(T.copy())

    # EE position (after flange)
    T_ee = frames_after[6] @ _OFFSETS[7]
    p_ee = T_ee[:3, 3]

    J = np.zeros((3, 7))
    for i in range(7):
        z_i = T_before[i][:3, 2]   # z-axis → rotation axis of joint i
        p_i = T_before[i][:3, 3]   # origin
        J[:, i] = np.cross(z_i, p_ee - p_i)
    return J, p_ee


def gravity_torques(q7: np.ndarray, g: float = 9.81) -> np.ndarray:
    """Analytic gravity compensation vector G(q) for the FR3 arm.

    Computes   G_i(q) = Σ_j  m_j · g⃗ · J_com_j[:, i]

    where g⃗ = [0, 0, g] (world Z up), m_j is link j's mass, and J_com_j is the
    3×7 positional Jacobian of link j's centre of mass.

    Only joints 0..j-1 contribute to link j's CoM Jacobian (distal joints have
    zero influence on proximal CoMs).

    To counteract gravity: command τ = G(q) on the effort controller.

    Link masses and CoM positions (link frame) from franka_description/fr3.urdf:
        link1: 2.927 kg  com=[ 0.000,  -0.018,  -0.039]
        link2: 2.936 kg  com=[ 0.003,  -0.074,   0.009]
        link3: 2.245 kg  com=[ 0.041,  -0.005,  -0.029]
        link4: 2.616 kg  com=[-0.046,   0.063,  -0.009]
        link5: 2.327 kg  com=[-0.002,   0.029,  -0.097]
        link6: 1.817 kg  com=[ 0.060,  -0.041,  -0.010]
        link7: 0.627 kg  com=[ 0.005,   0.009,  -0.016]

    Parameters
    ----------
    q7 : (7,)  joint angles [rad]
    g  : float  gravitational acceleration [m/s²]

    Returns
    -------
    G : (7,)  gravity torques [Nm]  — command these to hold the arm at rest
    """
    _MASSES = np.array([
        2.9274653454, 2.9355370338, 2.2449013699, 2.6155955791,
        2.3271207594, 1.8170376524, 0.6271432862,
    ])
    _COM = np.array([
        [ 0.0000004128, -0.0181251324, -0.0386035970],   # link1
        [ 0.0031828864, -0.0743221644,  0.0088146084],   # link2
        [ 0.0407015686, -0.0048200565, -0.0289730823],   # link3
        [-0.0459100965,  0.0630492960, -0.0085187868],   # link4
        [-0.0016039605,  0.0292536262, -0.0972965990],   # link5
        [ 0.0597131221, -0.0410294666, -0.0101692726],   # link6
        [ 0.0045225817,  0.0086261921, -0.0161633251],   # link7
    ])

    # Rebuild the kinematic chain (same structure as jacobian())
    T = np.eye(4)
    T_before: list = []
    frames_after: list = []
    for i in range(7):
        T_off = T @ _OFFSETS[i]
        T_before.append(T_off.copy())
        T = T_off @ _rz(q7[i])
        frames_after.append(T.copy())

    g_vec = np.array([0., 0., g])   # gravity direction (world Z up)
    G = np.zeros(7)

    for j in range(7):              # link j+1 (0-indexed)
        # CoM of link j in world frame
        T_lj = frames_after[j]
        p_com = T_lj[:3, :3] @ _COM[j] + T_lj[:3, 3]

        # Only joints 0..j contribute to this CoM's Jacobian column
        for i in range(j + 1):
            z_i = T_before[i][:3, 2]
            p_i = T_before[i][:3, 3]
            J_col = np.cross(z_i, p_com - p_i)   # positional Jacobian column
            G[i] += _MASSES[j] * np.dot(g_vec, J_col)

    return G


def dls_qdot(J: np.ndarray, xdot: np.ndarray, lam: float = 0.05) -> np.ndarray:
    """Damped-least-squares joint velocity from Cartesian velocity.

    Solves:  J @ qdot ≈ xdot
    using the regularised pseudoinverse:  Jᵀ (J Jᵀ + λ² I)⁻¹

    Damping λ grows automatically when manipulability drops (near singularity),
    causing the robot to slow down smoothly instead of producing velocity spikes.

    Parameters
    ----------
    J    : (3, 7)  positional Jacobian
    xdot : (3,)   desired Cartesian velocity [m/s]
    lam  : float   base damping coefficient

    Returns
    -------
    qdot : (7,)  joint velocities [rad/s]
    """
    JJt = J @ J.T   # (3, 3)

    # Manipulability w = σ₁·σ₂·σ₃ = √det(J Jᵀ)
    det = np.linalg.det(JJt)
    w = math.sqrt(max(0.0, det))

    # Boost damping near singularity (w_thresh tuned for metre-scale robots)
    w_thresh = 0.04
    if w < w_thresh:
        lam = lam + 0.15 * (1.0 - w / w_thresh)

    qdot = J.T @ np.linalg.solve(JJt + (lam * lam) * np.eye(3), xdot)
    return qdot
