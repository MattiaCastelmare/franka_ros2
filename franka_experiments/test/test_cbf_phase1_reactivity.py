"""Phase 1 closed loop: the commanded retreat follows the obstacle's speed.

B1/B2 of the target behaviour, checked on the REAL FR3 kinematics with the REAL
QP: an obstacle approaches the flange control point at 0.2 / 0.5 / 1.0 m/s, the
filter's rows are rebuilt at 50 Hz and solved by OSQP at 100 Hz exactly as
``cbf_safety_filter._qp_tick`` assembles them, the joint state is integrated in
between, and the separation rate the arm actually reaches at the barrier is
read back. The parameters are the SHIPPED ones from config/fr3_control.yaml
with the Phase-1 defaults (tracker source + residual floor), so a retune of the
retreat cap that breaks proportionality fails here, not on the robot.

Needs Pinocchio, OSQP and the FR3 xacro — runs in the ROS container, skips
elsewhere.
"""

import os
from types import SimpleNamespace

import numpy as np
import pytest
import yaml

pin = pytest.importorskip('pinocchio')
osqp = pytest.importorskip('osqp')
sparse = pytest.importorskip('scipy.sparse')

from franka_experiments.utils.cbf_hard_limits import (          # noqa: E402
    apply_slew_limit, position_velocity_accel_box)
from franka_experiments.utils.cbf_qp_assembly import (          # noqa: E402
    build_osqp_A, build_osqp_bounds, build_row_rhs, pad_rows_to_block,
    tangential_bias)
from franka_experiments.utils.cbf_state_rows import (           # noqa: E402
    G_CAP, G_OBS, G_QLIM, G_SC, G_SING, G_SPD, NV, NX, N_SLACK,
    ConstraintBuilder, JointSnap, Obstacle, ObstacleSnap)

try:
    from franka_experiments.utils.kinematics import (
        CBFKinematics, build_urdf_no_hand)
    _MODEL = pin.buildModelFromUrdf(build_urdf_no_hand())
except Exception as exc:                                       # pragma: no cover
    pytest.skip(f'FR3 model unavailable: {exc}', allow_module_level=True)

CFG = os.path.join(os.path.dirname(__file__), '..', 'config', 'fr3_control.yaml')

# Official FR3, franka_description/robots/fr3/joint_limits.yaml
Q_MIN = np.array([-2.9007, -1.8361, -2.9007, -3.0770, -2.8763, 0.4398, -3.0508])
Q_MAX = np.array([2.9007, 1.8361, 2.9007, -0.1169, 2.8763, 4.6216, 3.0508])
QD = np.array([2.62, 2.62, 2.62, 2.62, 5.26, 4.18, 5.26])
QDD = np.array([6.0, 2.585, 3.5, 4.0, 17.0, 5.5, 17.0])
Q0 = np.array([0.0, -0.70, 0.0, -2.35, 0.0, 1.57, 0.70])
R_CAP = 0.05
CP_LINK = 'fr3_link8'


class _Log:
    def info(self, *a, **k):    pass
    def warn(self, *a, **k):    pass
    def warning(self, *a, **k): pass
    def error(self, *a, **k):   pass
    def debug(self, *a, **k):   pass


def shipped_params(**over):
    with open(CFG) as f:
        P = SimpleNamespace(**yaml.safe_load(f)['params'])
    # Phase-1 runtime defaults (launch_defaults.yaml turns the tracker on).
    P.obstacle_velocity_source = 'tracker'
    # Families that are not under test and would only add rows here.
    P.joint_limit_rows_enabled = False
    P.enable_lateral_evasion = False
    P.enable_uncertainty_margin = False
    for k, v in over.items():
        assert hasattr(P, k), k
        setattr(P, k, v)
    return P


class Sim:
    """The filter's control loop, headless: rows at 50 Hz, OSQP at 100 Hz,
    a double integrator for the joints."""

    def __init__(self, P):
        self.P = P
        self.kin = CBFKinematics(_MODEL)
        self.fid = self.kin.resolve_frame_id(CP_LINK)
        self.builder = ConstraintBuilder(
            P, self.kin, q_min=Q_MIN, q_max=Q_MAX, acc_lb=-QDD, acc_ub=QDD,
            logger=_Log())
        P_mat = np.eye(NX)
        for g, rho in ((G_OBS, P.rho_slack), (G_SC, P.rho_slack_self_collision),
                       (G_QLIM, P.rho_slack_joint_limit),
                       (G_SING, P.rho_slack_singularity),
                       (G_CAP, P.rho_slack_retreat), (G_SPD, P.rho_slack_link_speed)):
            P_mat[NV + g, NV + g] = rho
        self.P_csc = sparse.csc_matrix(P_mat)
        self.qvec = np.zeros(NX)
        self.box_lb = np.concatenate([-QDD, np.zeros(N_SLACK)])
        self.box_ub = np.concatenate([QDD, np.full(N_SLACK, 1e6)])
        self.prob, self.prev_rows = None, -1
        self.dt = 1.0 / P.qp_rate_hz
        self.rebuild_every = int(round(P.qp_rate_hz / P.cbf_update_rate_hz))

    def cp(self, q, qdot):
        self.kin.update(q, qdot, with_jdot=True)
        return self.kin.data.oMf[self.fid].translation.copy()

    def solve(self, con, q, qdot, qddot_nom, qddot_prev, tan):
        P = self.P
        h_qp = None
        if con is not None:
            h_qp, _ = build_row_rhs(con, qdot, qdot, k0=P.k0_cbf, k1=P.k1_cbf,
                                    retreat_horizon=P.retreat_cap_horizon_s,
                                    speed_horizon=P.link_speed_horizon_s)
            raw = tangential_bias(qddot_nom, qdot, con, gain=P.cbf_tangential_gain,
                                  engage_margin=P.cbf_tangential_engage_margin,
                                  max_bias=P.cbf_tangential_max_bias)
            a_t = P.cbf_tangential_filter_alpha
            tan[:] = a_t * tan + (1.0 - a_t) * raw
            qddot_nom = qddot_nom + tan
        self.qvec[:NV] = -qddot_nom
        position_velocity_accel_box(
            q, qdot, acc_lb=-QDD, acc_ub=QDD, qdot_max=QD,
            v_margin=P.velocity_box_margin, q_min=Q_MIN, q_max=Q_MAX,
            q_margin=P.position_margin_rad, brake_eta=P.position_brake_eta,
            dt=self.dt, relax_dt=P.state_box_relax_s,
            out_lb=self.box_lb[:NV], out_ub=self.box_ub[:NV],
            clip_to_limits=P.accel_box_clip_to_limits)
        if P.slew_box_enabled:
            self.box_lb[:NV], self.box_ub[:NV] = apply_slew_limit(
                self.box_lb[:NV], self.box_ub[:NV], qddot_prev, P.max_qddot_delta)
        G = None if con is None else con.G
        G, h = pad_rows_to_block(G, h_qp, P.qp_row_block)
        n_rows = 0 if G is None else G.shape[0]
        l, u = build_osqp_bounds(G, h, self.box_lb, self.box_ub)
        if n_rows != self.prev_rows or self.prob is None:
            self.prev_rows = n_rows
            self.prob = osqp.OSQP()
            self.prob.setup(P=self.P_csc, q=self.qvec, A=build_osqp_A(G, NV, N_SLACK),
                            l=l, u=u, warm_start=True, max_iter=P.osqp_max_iter,
                            verbose=False)
        elif n_rows > 0:
            self.prob.update(q=self.qvec, l=l, u=u, Ax=build_osqp_A(G, NV, N_SLACK).data)
        else:
            self.prob.update(q=self.qvec, l=l, u=u)
        res = self.prob.solve()
        assert res.info.status_val == osqp.constant('OSQP_SOLVED'), res.info.status
        return res.x[:NV].copy(), res.x[NV:].copy(), qddot_nom

    def run(self, v_obs, *, d0=0.45, T=None, static_gap=None, nominal=None):
        """A PURSUING obstacle: it starts ``d0`` beyond the flange on the +y
        side and moves toward the control point's CURRENT position at
        ``v_obs`` — a hand following the flange — so the closing speed along
        n̂ is ``v_obs`` for as long as the run lasts, whatever the arm does.
        (A straight-line obstacle is outrun at 1 m/s: the k1 anticipation
        engages the row at 0.45 m and the flange leaves its path.) With
        ``static_gap`` the obstacle is parked instead. ``nominal(t, q, qdot)``
        is the task command; default holds Q0 with a soft PD. The run is sized
        so the arm retreats well under half a metre: joint limits are not what
        is under test here."""
        q, qdot = Q0.copy(), np.zeros(NV)
        p0 = self.cp(q, qdot)
        y_hat = np.array([0.0, 1.0, 0.0])
        if T is None:
            T = 3.0 if static_gap is not None else min(2.5, 0.35 / v_obs + 0.45)
        ph = p0 + ((static_gap if static_gap is not None else d0) + R_CAP) * y_hat
        qddot_prev, tan = np.zeros(NV), np.zeros(NV)
        con = None
        rec = []
        n = int(round(T / self.dt))
        for k in range(n):
            t = k * self.dt
            pr = self.cp(q, qdot)
            if static_gap is not None:
                v_vec = np.zeros(3)
            else:
                dirn = (pr - ph) / np.linalg.norm(pr - ph)
                if k > 0:
                    ph = ph + v_obs * self.dt * dirn
                v_vec = v_obs * dirn                       # toward the CP
            gap = float(np.linalg.norm(pr - ph)) - R_CAP
            if k % self.rebuild_every == 0:
                ob = Obstacle(link=CP_LINK, d=max(gap, 0.0), pr=pr, ph=ph, conf=1.0,
                              v_vec=v_vec, frames_seen=10,
                              vel_cov=(0.02 ** 2) * np.eye(3), track_id=1)
                con = self.builder.build(JointSnap(q=q, qdot=qdot, stamp=t),
                                         ObstacleSnap(items=(ob,), stamp=t, t_cap=t), t)
            if nominal is None:
                qddot_nom = -8.0 * (q - Q0) - 2.0 * np.sqrt(8.0) * qdot
            else:
                qddot_nom = nominal(t, q, qdot)
            qddot, slack, nom_used = self.solve(con, q, qdot, qddot_nom, qddot_prev, tan)
            qddot_prev[:] = qddot
            # Separation rate of the control point along n̂ (obstacle → CP).
            n_hat = (pr - ph) / np.linalg.norm(pr - ph)
            Jp, _ = self.kin.point_jacobian(self.fid, pr)
            sep = float((n_hat @ Jp) @ qdot)
            rec.append(dict(t=t, gap=gap, sep=sep, slack=slack,
                            dnorm=float(np.linalg.norm(qddot - nom_used)),
                            v_obs=float(con.v_obs[0]) if con is not None else 0.0))
            qdot = qdot + qddot * self.dt
            q = q + qdot * self.dt
        return rec


def retreat_speed(rec, window=10):
    """The retreat the arm actually settles into: the peak of the 100 ms
    moving average of the separation rate along n̂. The transient is excluded
    by the averaging, the cap bounds the peak from above, and the barrier —
    which asks for ḣ → 0, i.e. aᵀq̇ → v_obs — pins it from below."""
    s = np.array([r['sep'] for r in rec])
    m = np.convolve(s, np.ones(window) / window, mode='valid')
    return float(m.max())


# ── B2: retreat is proportional to the closing speed ────────────────────────

@pytest.fixture(scope='module')
def retreats():
    P = shipped_params()
    out = {}
    for v in (0.2, 0.5, 1.0):
        rec = Sim(P).run(v)
        out[v] = (retreat_speed(rec), rec)
    return out


def test_retreat_speed_is_monotone_in_the_approach_speed(retreats):
    r = [retreats[v][0] for v in (0.2, 0.5, 1.0)]
    assert r[0] < r[1] < r[2], r


def test_retreat_speed_is_roughly_linear_in_the_approach_speed(retreats):
    """The HOCBF asks for ḣ → 0, i.e. aᵀq̇ → v_obs, and the cap sits at
    base + v_obs (+ the depth term while the barrier is dented). So the
    retreat must track v_obs to within the creep and that escalation, over
    the whole 0.2-1.0 m/s range — not saturate, which is what the old
    retreat_cap_max_speed = 0.6 did from 0.55 m/s upward."""
    for v in (0.2, 0.5, 1.0):
        r = retreats[v][0]
        assert 0.7 * v <= r <= 1.3 * v + 0.15, (v, r)
    r02, r05, r10 = (retreats[v][0] for v in (0.2, 0.5, 1.0))
    assert 1.5 <= r10 / r05 <= 2.7, (r05, r10)
    assert 1.6 <= r05 / r02 <= 3.4, (r02, r05)


def test_the_estimated_closing_speed_reaches_the_qp(retreats):
    """The regression this phase fixes: v_obs must be non-zero and close to the
    true approach speed while the obstacle is inside the engagement gap."""
    for v in (0.2, 0.5, 1.0):
        rec = retreats[v][1]
        near = [r['v_obs'] for r in rec if r['gap'] < 0.40]
        assert np.median(near) > 0.8 * v, (v, np.median(near))


def test_the_barrier_is_not_crossed_by_a_walking_pace_approach(retreats):
    """0.2 and 0.5 m/s are well inside the retreat authority of the flange, so
    the gap must never drop meaningfully below d_safe; 1 m/s is allowed the
    dent the depth term exists for, but never contact."""
    P = shipped_params()
    assert min(r['gap'] for r in retreats[1.0][1]) > 0.5 * P.d_safe
    for v in (0.2, 0.5):
        gmin = min(r['gap'] for r in retreats[v][1])
        assert gmin > P.d_safe - 0.03, (v, gmin)


# ── B1: a static obstacle leaves the task alone ─────────────────────────────

def test_static_obstacle_the_cap_never_binds_and_the_nominal_is_tracked():
    """Obstacle parked at 0.35 m (outside the engagement gap) while the task
    sweeps joint 1: no cap slack, no obstacle slack, and the QP returns the
    nominal to within numerical tolerance on every tick."""
    P = shipped_params()

    def sweep(t, q, qdot):
        A, w = 0.30, 1.5
        q_ref = Q0.copy(); q_ref[0] += A * np.sin(w * t)
        qd_ref = np.zeros(NV); qd_ref[0] = A * w * np.cos(w * t)
        qdd_ff = np.zeros(NV); qdd_ff[0] = -A * w * w * np.sin(w * t)
        return qdd_ff - 20.0 * (q - q_ref) - 9.0 * (qdot - qd_ref)

    rec = Sim(P).run(0.0, T=3.0, static_gap=0.35, nominal=sweep)
    s = np.array([r['slack'] for r in rec])
    assert np.all(s[:, G_CAP] < 1e-3), s[:, G_CAP].max()   # OSQP eps, not a binding row
    assert np.all(s[:, G_OBS] < 1e-3), s[:, G_OBS].max()
    assert max(r['dnorm'] for r in rec[20:]) < 0.05, max(r['dnorm'] for r in rec)
    assert min(r['gap'] for r in rec) > P.d_safe + 0.05   # the sweep's arc brings the flange ~0.1 m closer; still outside the barrier
