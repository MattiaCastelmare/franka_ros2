#!/usr/bin/env python3
"""Phase 5 integration: the whole filter, closed loop, on the real FR3 model.

Every scenario runs the SHIPPED code — ConstraintBuilder with the parameters in
config/fr3_control.yaml plus the launch defaults, the QP assembled exactly as
cbf_safety_filter._qp_tick assembles it, OSQP with the fallback ladder, the
livelock detector — against Pinocchio kinematics of the FR3 and a double
integrator for the joints. What is EMULATED is the perception chain, with the
numbers Phase 0 measured (scripts/latency_budget.py):

    depth frames at `fps`, each stamped `t_cam` after the scene it shows;
    a track is born on first sight and reports a velocity only from its
    third frame (frames_seen >= 3, the shipped gate); the per-CP message is
    published only when some control point is inside max_thresh (0.7 m);
    the constraint rebuild runs at 50 Hz on the latest message and the QP at
    100 Hz, so the rate waits are real, not modelled.

Scenarios (B1-B4 of the target behaviour, plus the B5 answer from 0c):

    static     an obstacle beside the path; the task must complete around it
    slow       a hand pursuing the flange at 0.3 m/s
    fast       a pursuer at 3 m/s, above the flange's retreat authority
    ball       a 5 cm ball thrown at 4 m/s from 2 m, through the real gates
    wedge      two static obstacles forming a V; the task drives the flange
               into the vertex (the livelock case)

Each reports: min h, whether h ever went negative and for how long, the
retreat speed reached against v_obs, the escape direction chosen (blend
weight and the angle between the Cartesian push and v_obs), the largest slack
per family, which fallback rungs fired, and whether the task completed.

    python3 scripts/cbf_scenarios.py            # all scenarios
    python3 scripts/cbf_scenarios.py --only ball --latency-compensation
"""
from __future__ import annotations

import argparse
import os
from types import SimpleNamespace

import numpy as np
import osqp
import pinocchio as pin
import scipy.sparse as sparse
import yaml

from franka_experiments.utils.cbf_hard_limits import (
    apply_slew_limit, position_velocity_accel_box)
from franka_experiments.utils.cbf_qp_assembly import (
    LEVEL_NAMES, OSQP_LEVEL_BOX, OSQP_LEVEL_BRAKE, OSQP_LEVEL_FULL,
    accept_iterate, box_only_solve, braking_command, build_osqp_A,
    build_osqp_bounds, build_row_rhs, pad_rows_to_block, tangential_bias)
from franka_experiments.utils.cbf_state_rows import (
    FR3_JOINT_KEYS, G_CAP, G_OBS, G_QLIM, G_SC, G_SING, G_SPD, GROUP_NAMES, NV, NX,
    N_SLACK, ConstraintBuilder, JointSnap, Obstacle, ObstacleSnap,
    build_optional_row_builders, retreat_speed_available)
from franka_experiments.utils.config import load_franka_joint_limits
from franka_experiments.utils.kinematics import CBFKinematics, build_urdf_no_hand
from franka_experiments.utils.livelock import LivelockDetector, ProgressWindow

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
Q0 = np.array([0.0, -0.70, 0.0, -2.35, 0.0, 1.57, 0.70])
R_CAP = 0.05
CP_SEGS = [('fr3_link3', 'fr3_link4', 2), ('fr3_link4', 'fr3_link5', 2),
           ('fr3_link5', 'fr3_link6', 2), ('fr3_link6', 'fr3_link7', 2),
           ('fr3_link7', 'fr3_link8', 3)]


class _Log:
    def __init__(self):
        self.lines = []

    def _e(self, m):
        self.lines.append(m)

    def info(self, m, **k):    self._e(m)
    def warn(self, m, **k):    self._e(m)
    def warning(self, m, **k): self._e(m)
    def error(self, m, **k):   self._e(m)
    def debug(self, m, **k):   pass


def shipped_params(**over):
    # The SOURCE config, not the installed copy: this is a development tool
    # and must see the file being edited.
    with open(os.path.join(PKG, 'config', 'fr3_control.yaml')) as f:
        P = SimpleNamespace(**yaml.safe_load(f)['params'])
    # launch_defaults.yaml turns the prediction stack on
    ld = yaml.safe_load(open(os.path.join(PKG, 'config', 'launch_defaults.yaml')))
    P.obstacle_velocity_source = ld.get('obstacle_velocity_source', 'tracker')
    P.enable_lateral_evasion = bool(ld.get('lateral_evasion', False))
    P.enable_uncertainty_margin = bool(ld.get('uncertainty_margin', False))
    P.enable_outrun_evasion = bool(ld.get('outrun_evasion', False))
    P.enable_livelock_escape = bool(ld.get('livelock_escape', False))
    P.enable_latency_compensation = bool(ld.get('latency_compensation', False))
    for k, v in over.items():
        assert hasattr(P, k), k
        setattr(P, k, v)
    return P


# ── Obstacles ────────────────────────────────────────────────────────────────

def camera_velocity_noise(rng, n=1):
    """[m/s] a draw from the tracker's MEASURED noise floor on a static scene.

    Fitted to the real cluster-centroid sequences of rosbag/arm_complex and
    rosbag/handratacker_object replayed through the shipped Kalman filter:
    speed median 0.06 m/s, p90 0.19-0.25 m/s, excursions past 1.6 m/s. A
    lognormal reproduces the body (median 0.06, p90 0.24 gives sigma = 1.08)
    and the tail is added explicitly, because it is the tail that used to put
    3 rad/s^2 steps into the QP.
    """
    mag = rng.lognormal(np.log(0.06), 1.08, n)
    spike = rng.random(n) < 0.01
    mag[spike] *= rng.uniform(4.0, 12.0, int(spike.sum()))
    d = rng.normal(size=(n, 3))
    d /= np.linalg.norm(d, axis=1, keepdims=True)
    return d * mag[:, None]


class Sphere:
    """A point obstacle of radius r with a position law. ``pursue`` makes it
    home on the flange's CURRENT position at ``speed`` (a hand following the
    arm); otherwise ``vel`` is a constant velocity from ``p0``."""

    def __init__(self, p0, vel=(0, 0, 0), r=0.05, pursue=False, speed=0.0):
        self.p = np.asarray(p0, dtype=np.float64)
        self.v = np.asarray(vel, dtype=np.float64)
        self.r = float(r)
        self.pursue = pursue
        self.speed = float(speed)
        self.first_seen = None          # sim time of the first depth frame

    def step(self, dt, target):
        if self.pursue:
            d = target - self.p
            n = np.linalg.norm(d)
            if n > 1e-9:
                self.v = self.speed * d / n
        self.p = self.p + self.v * dt


# ── The filter, headless ─────────────────────────────────────────────────────

class Filter:
    def __init__(self, P):
        self.P = P
        self.kin = CBFKinematics(pin.buildModelFromUrdf(build_urdf_no_hand()))
        jl = load_franka_joint_limits(FR3_JOINT_KEYS)
        self.lb, self.ub = -jl['decel_max'], jl['decel_max']
        self.qd_max, self.q_min, self.q_max = jl['qdot_max'], jl['q_min'], jl['q_max']
        self.log = _Log()
        opt = build_optional_row_builders(P, self.kin, self.log)
        self.builder = ConstraintBuilder(P, self.kin, q_min=self.q_min, q_max=self.q_max,
                                         acc_lb=self.lb, acc_ub=self.ub, logger=self.log,
                                         qdot_max=self.qd_max, **opt)
        P_mat = np.eye(NX)
        for g, rho in ((G_OBS, P.rho_slack), (G_SC, P.rho_slack_self_collision),
                       (G_QLIM, P.rho_slack_joint_limit), (G_SING, P.rho_slack_singularity),
                       (G_CAP, P.rho_slack_retreat), (G_SPD, P.rho_slack_link_speed)):
            P_mat[NV + g, NV + g] = rho
        self.P_csc = sparse.csc_matrix(P_mat)
        self.qvec = np.zeros(NX)
        self.box_lb = np.concatenate([self.lb, np.zeros(N_SLACK)])
        self.box_ub = np.concatenate([self.ub, np.full(N_SLACK, 1e6)])
        self.prob, self.prev_rows = None, -1
        self.dt = 1.0 / P.qp_rate_hz
        self.tan = np.zeros(NV)
        self.esc = np.zeros(NV)
        self.outr = np.zeros(NV)
        self.lock_bias = np.zeros(NV)
        self.qddot_prev = np.zeros(NV)
        self.nom_prev = np.zeros(NV)
        self.lock = LivelockDetector(stall_s=P.livelock_stall_s, ramp_s=P.livelock_ramp_s,
                                     max_s=P.livelock_max_s, cooldown_s=P.livelock_cooldown_s)
        self.progress = ProgressWindow(P.livelock_progress_window_s)
        self.fids = {n: self.kin.resolve_frame_id(n) for n in
                     [f'fr3_link{i}' for i in range(3, 9)]}
        self.levels = [0, 0, 0]

    def control_points(self, q, qdot):
        self.kin.update(q, qdot, with_jdot=True)
        pos = {n: self.kin.data.oMf[f].translation.copy() for n, f in self.fids.items()}
        cps = []
        for s, e, n in CP_SEGS:
            ts = [(k + 1) / n for k in range(n)] if e == 'fr3_link8' else \
                 [(k + 1) / (n + 1) for k in range(n)]
            for t in ts:
                cps.append((e, pos[s] + t * (pos[e] - pos[s])))
        return cps

    def tick(self, t, con, q, qdot, qddot_nom):
        P = self.P
        h_qp = None
        lock_mag = 0.0
        nom_raw = np.array(qddot_nom, copy=True)
        if con is not None:
            h_qp, _ = build_row_rhs(con, qdot, qdot, k0=P.k0_cbf, k1=P.k1_cbf,
                                    retreat_horizon=P.retreat_cap_horizon_s,
                                    speed_horizon=P.link_speed_horizon_s)
            raw = tangential_bias(qddot_nom, qdot, con, gain=P.cbf_tangential_gain,
                                  engage_margin=P.cbf_tangential_engage_margin,
                                  max_bias=P.cbf_tangential_max_bias)
            a_t = P.cbf_tangential_filter_alpha
            self.tan = a_t * self.tan + (1 - a_t) * raw
            qddot_nom = qddot_nom + self.tan
            if con.esc_bias is not None:
                self.esc = a_t * self.esc + (1 - a_t) * con.esc_bias
                qddot_nom = qddot_nom + self.esc
            else:
                self.esc *= a_t
            a_o = P.outrun_evasion_filter_alpha
            if con.outrun_bias is not None:
                self.outr = a_o * self.outr + (1 - a_o) * con.outrun_bias
                qddot_nom = qddot_nom + self.outr
            else:
                self.outr *= a_o
            if P.enable_livelock_escape:
                blocked = (con.livelock_dir is not None
                           and np.linalg.norm(nom_raw) > P.livelock_nominal_min
                           and np.linalg.norm(self.qddot_prev - self.nom_prev) > P.livelock_dnorm_thr)
                moving = self.progress.push(t, q) > P.livelock_progress_thr
                lock_mag = self.lock.update(t, blocked=blocked, moving=moving)
                raw_lock = (P.livelock_gain * lock_mag * con.livelock_dir
                            if lock_mag > 0.0 and con.livelock_dir is not None
                            else np.zeros(NV))
                self.lock_bias = a_t * self.lock_bias + (1 - a_t) * raw_lock
                qddot_nom = qddot_nom + self.lock_bias
        self.nom_prev = qddot_nom
        self.qvec[:NV] = -qddot_nom
        position_velocity_accel_box(
            q, qdot, acc_lb=self.lb, acc_ub=self.ub, qdot_max=self.qd_max,
            v_margin=P.velocity_box_margin, q_min=self.q_min, q_max=self.q_max,
            q_margin=P.position_margin_rad, brake_eta=P.position_brake_eta,
            dt=self.dt, relax_dt=P.state_box_relax_s,
            out_lb=self.box_lb[:NV], out_ub=self.box_ub[:NV],
            clip_to_limits=P.accel_box_clip_to_limits)
        if P.slew_box_enabled:
            self.box_lb[:NV], self.box_ub[:NV] = apply_slew_limit(
                self.box_lb[:NV], self.box_ub[:NV], self.qddot_prev, P.max_qddot_delta)
        G = None if con is None else con.G
        G, h = pad_rows_to_block(G, h_qp, P.qp_row_block)
        n_rows = 0 if G is None else G.shape[0]
        l, u = build_osqp_bounds(G, h, self.box_lb, self.box_ub)
        if n_rows != self.prev_rows or self.prob is None:
            self.prev_rows = n_rows
            self.prob = osqp.OSQP()
            self.prob.setup(P=self.P_csc, q=self.qvec, A=build_osqp_A(G, NV, N_SLACK),
                            l=l, u=u, warm_start=True, max_iter=P.osqp_max_iter, verbose=False)
        elif n_rows > 0:
            self.prob.update(q=self.qvec, l=l, u=u, Ax=build_osqp_A(G, NV, N_SLACK).data)
        else:
            self.prob.update(q=self.qvec, l=l, u=u)
        res = self.prob.solve()
        x = accept_iterate(res.x, res.info.status_val, self.box_lb, self.box_ub, NV,
                           solved=osqp.constant('OSQP_SOLVED'),
                           inaccurate=osqp.constant('OSQP_SOLVED_INACCURATE'))
        level = OSQP_LEVEL_FULL
        if x is None:
            x = box_only_solve(self.P_csc, self.qvec, self.box_lb, self.box_ub,
                               max_iter=P.osqp_max_iter)
            level = OSQP_LEVEL_BOX
            if x is None:
                x = np.concatenate([braking_command(qdot, self.box_lb[:NV], self.box_ub[:NV],
                                                    k_brake=P.k_brake), np.zeros(N_SLACK)])
                level = OSQP_LEVEL_BRAKE
            self.prob, self.prev_rows = None, -1
        self.levels[level] += 1
        self.qddot_prev[:] = x[:NV]
        return x[:NV], x[NV:], qddot_nom, level, lock_mag


# ── One scenario ─────────────────────────────────────────────────────────────

def run_scenario(name, P, spheres, *, T, goal_q=None, fps=30.0, t_cam=0.013,
                 max_thresh=0.7, seed=0, verbose=False, cam_noise=False,
                 cart_ref=None):
    rng = np.random.default_rng(seed)
    F = Filter(P)
    d_safe = P.d_safe
    q, qdot = Q0.copy(), np.zeros(NV)
    goal = Q0.copy() if goal_q is None else np.asarray(goal_q, dtype=np.float64)
    dt = F.dt
    n = int(round(T / dt))
    frame_period = 1.0 / fps
    next_frame = 0.0
    last_msg = None          # (t_cap, obstacles) as the CBF would receive it
    msg_t_recv = -1.0
    con = None
    rec = dict(t=[], hmin=[], sep=[], vobs=[], level=[], slack=[], lock=[], outr_w=[],
               push_cos=[], ee_err=[], vavail=[], jerk=[], step=[], qddot=[])
    obs_state_hist = []      # (t, [sphere positions/vels]) for the camera latency
    fl_fid = F.fids['fr3_link8']
    F.control_points(q, qdot)          # place the kinematics BEFORE reading FK
    p_ref0 = F.kin.data.oMf[fl_fid].translation.copy()
    cart_ref = None if cart_ref is None else np.asarray(cart_ref, dtype=np.float64)
    for k in range(n):
        t = k * dt
        cps = F.control_points(q, qdot)
        flange = F.kin.data.oMf[fl_fid].translation.copy()
        for s in spheres:
            s.step(dt, flange)
        obs_state_hist.append((t, [(s.p.copy(), s.v.copy()) for s in spheres]))

        # ── perception: a depth frame every 1/fps, delivered t_cam later ──
        if t >= next_frame:
            next_frame += frame_period
            t_cap = t
            # the frame shows the scene at capture time; the CBF receives it at t_cap + t_cam
            snap = []
            for s in spheres:
                if s.first_seen is None:
                    s.first_seen = t_cap
                frames_seen = int((t_cap - s.first_seen) / frame_period) + 1
                snap.append((s, s.p.copy(), s.v.copy(), frames_seen))
            last_frame = (t_cap, snap, cps)
            msg_t_recv = t_cap + t_cam
            pending = last_frame
        if last_msg is None or (msg_t_recv >= 0 and t >= msg_t_recv and last_msg[0] < pending[0]):
            if msg_t_recv >= 0 and t >= msg_t_recv:
                last_msg = pending

        # ── constraint rebuild at 50 Hz on the latest received message ─────
        if k % 2 == 0 and last_msg is not None:
            t_cap, snap, cps_cap = last_msg
            items = []
            any_in_band = False
            for (link, p_cp) in cps:            # rows are built at the CURRENT q
                best = None
                for s, p_s, v_s, seen in snap:
                    gap = float(np.linalg.norm(p_cp - p_s)) - s.r - R_CAP
                    if best is None or gap < best[0]:
                        best = (gap, s, p_s, v_s, seen)
                gap, s, p_s, v_s, seen = best
                if gap <= max_thresh:
                    any_in_band = True
                ph = p_s + s.r * (p_cp - p_s) / max(np.linalg.norm(p_cp - p_s), 1e-9)
                # What perception REPORTS for this obstacle. With cam_noise the
                # tracker's measured noise floor is added, which is the whole
                # point of that scenario: the obstacle is static and the
                # camera says otherwise.
                v_meas = v_s + (camera_velocity_noise(rng)[0] if cam_noise
                                else rng.normal(0, 0.02, 3))
                items.append(Obstacle(link=link, d=max(gap, 0.0), pr=p_cp, ph=ph, conf=1.0,
                                      v_vec=v_meas, frames_seen=seen, vel_cov=(0.05 ** 2) * np.eye(3),
                                      track_id=1, a_vec=np.zeros(3),
                                      pos_cov=(0.01 ** 2) * np.eye(3), pv_cov=np.zeros((3, 3))))
            if any_in_band:
                con = F.builder.build(JointSnap(q=q, qdot=qdot, stamp=t),
                                      ObstacleSnap(items=tuple(items), stamp=t, t_cap=t_cap), t)
            else:
                con = None
        # ── task ──────────────────────────────────────────────────────────
        if cart_ref is not None:
            # Cartesian reference advancing at a constant speed, like a path
            # commander: the reference does NOT stop at the obstacle, so the
            # barrier has to hold a sustained push. That is the regime the
            # oscillation report is about; a joint PD to a fixed goal relaxes
            # its own pull as it converges and hides it.
            p_ref = p_ref0 + cart_ref * t
            Jp_ee, Jpd_ee = F.kin.point_jacobian(fl_fid, flange)
            e = p_ref - flange
            v_ee = Jp_ee @ qdot
            # Resolved acceleration, ~5 rad/s and critically damped, through a
            # damped pseudo-inverse so a stretched pose cannot make the task
            # itself the source of large commands.
            a_task = 25.0 * e + 10.0 * (cart_ref - v_ee)
            JJt = Jp_ee @ Jp_ee.T + (0.05 ** 2) * np.eye(3)
            J_pinv = Jp_ee.T @ np.linalg.inv(JJt)
            qddot_nom = J_pinv @ (a_task - Jpd_ee @ qdot)
            qddot_nom -= 6.0 * (np.eye(NV) - J_pinv @ Jp_ee) @ qdot
        else:
            qddot_nom = -12.0 * (q - goal) - 2.0 * np.sqrt(12.0) * qdot
        qddot, slack, nom_used, level, lock_mag = F.tick(t, con, q, qdot, qddot_nom)

        # ── truth for the report ──────────────────────────────────────────
        hmin, sep, vo, va, cos_push, wmax = np.inf, 0.0, 0.0, np.nan, np.nan, 0.0
        for (link, p_cp) in cps:
            for s in spheres:
                gap = float(np.linalg.norm(p_cp - s.p)) - s.r - R_CAP
                if gap - d_safe < hmin:
                    hmin = gap - d_safe
                    n_hat = (p_cp - s.p) / max(np.linalg.norm(p_cp - s.p), 1e-9)
                    Jp, _ = F.kin.point_jacobian(F.fids[link], p_cp)
                    a = n_hat @ Jp
                    sep = float(a @ qdot)
                    vo = float(n_hat @ s.v)
                    va = retreat_speed_available(a, P.velocity_box_margin * F.qd_max)
                    if np.linalg.norm(s.v) > 1e-6 and np.linalg.norm(F.outr) > 1e-9:
                        push = Jp @ F.outr
                        if np.linalg.norm(push) > 1e-9:
                            cos_push = float(push @ s.v) / (np.linalg.norm(push) * np.linalg.norm(s.v))
        if verbose and k % 25 == 0:
            print(f'   t={t:5.2f} hmin={hmin:+.3f} |qdot|={np.linalg.norm(qdot):.3f} '
                  f'dnorm={np.linalg.norm(qddot - nom_used):.2f} lock={F.lock.state:9s} mag={lock_mag:.2f} '
                  f's_obs={slack[G_OBS]:.3f} s_cap={slack[G_CAP]:.3f} lvl={level} '
                  f'ldir={"y" if (con is not None and con.livelock_dir is not None) else "-"}')
        rec['t'].append(t); rec['hmin'].append(hmin); rec['sep'].append(sep)
        rec['vobs'].append(max(vo, 0.0)); rec['level'].append(level)
        rec['slack'].append(slack.copy()); rec['lock'].append(lock_mag)
        rec['outr_w'].append(F.builder.diag_outrun_w); rec['push_cos'].append(cos_push)
        rec['ee_err'].append(float(np.linalg.norm(q - goal))); rec['vavail'].append(va)
        # SMOOTHNESS, measured: how much the commanded acceleration changes
        # from one tick to the next. This is what the arm feels and what the
        # firmware sees as a torque discontinuity.
        # Two readings of the same thing. The NORM is what the arm feels; the
        # per-JOINT step is what the slew box bounds (max_qddot_delta), so it
        # is the one that says whether the command is admissible at all.
        if rec['qddot']:
            step = np.abs(qddot - rec['qddot'][-1])
            rec['jerk'].append(float(np.linalg.norm(qddot - rec['qddot'][-1])) / F.dt)
            rec['step'].append(float(step.max()))
        else:
            rec['jerk'].append(0.0); rec['step'].append(0.0)
        rec['qddot'].append(qddot.copy())
        qdot = qdot + qddot * dt
        q = q + qdot * dt
    return summarize(name, P, rec, F, spheres)


def _oscillation(rec, dt, d_safe):
    """How much the arm hunts once the barrier has engaged.

    Measured on the settled part of the run (from the first time the barrier
    engages), because the approach transient is not what "oscillates" means:

    * ``sep_sign_hz`` — how often per second the separation rate changes sign.
      A converged standoff crosses zero once; a limit cycle crosses it at its
      own frequency, so this IS the oscillation frequency.
    * ``gap_ptp`` — peak-to-peak of the surface gap over the same window.
    """
    h = np.array(rec['hmin']); sep = np.array(rec['sep'])
    eng = np.flatnonzero(h < 0.10)
    if eng.size < 20:
        return dict(sep_sign_hz=0.0, gap_ptp=0.0, engaged_s=0.0)
    i0 = int(eng[0])
    s = sep[i0:]; g = h[i0:] + d_safe
    dead = 0.01                                   # ignore numerical zero-crossings
    sig = np.sign(np.where(np.abs(s) < dead, 0.0, s))
    sig = sig[sig != 0]
    flips = int(np.count_nonzero(np.diff(sig))) if sig.size > 1 else 0
    return dict(sep_sign_hz=flips / max(len(s) * dt, 1e-9),
                gap_ptp=float(g.max() - g.min()),
                engaged_s=float(len(s) * dt))


def summarize(name, P, rec, F, spheres):
    h = np.array(rec['hmin']); t = np.array(rec['t'])
    neg = h < 0.0
    sl = np.array(rec['slack'])
    sep = np.array(rec['sep']); vo = np.array(rec['vobs'])
    eng = vo > 0.05
    lv = np.array(rec['level'])
    out = dict(
        name=name, min_h=float(h.min()), h_negative=bool(neg.any()),
        neg_duration_s=float(neg.sum() * F.dt), contact=bool((h < -P.d_safe).any()),
        v_obs_max=float(vo.max()), retreat_peak=float(np.convolve(sep, np.ones(10) / 10, 'valid').max()),
        v_avail_at_min_h=float(np.array(rec['vavail'])[int(np.argmin(h))]),
        outrun_w_max=float(np.max(rec['outr_w'])),
        push_vs_vobs_cos=(float(np.nanmedian(rec['push_cos'])) if np.isfinite(rec['push_cos']).any() else None),
        slack_max={GROUP_NAMES[g]: float(sl[:, g].max()) for g in range(N_SLACK)},
        fallback_levels={LEVEL_NAMES[i]: int(c) for i, c in enumerate(F.levels)},
        livelock_escapes=F.lock.n_escapes, livelock_max=float(np.max(rec['lock'])),
        task_final_err=float(rec['ee_err'][-1]), task_done=bool(rec['ee_err'][-1] < 0.05),
        **_oscillation(rec, F.dt, P.d_safe),
        jerk_p50=float(np.percentile(rec['jerk'][1:], 50)),
        jerk_p99=float(np.percentile(rec['jerk'][1:], 99)),
        jerk_max=float(np.max(rec['jerk'][1:])),
        step_p99=float(np.percentile(rec['step'][1:], 99)),
        step_max=float(np.max(rec['step'][1:])),
        slew_limit=float(P.max_qddot_delta),
    )
    return out


def print_report(r):
    print(f"\n== {r['name']}")
    print(f"   min h = {r['min_h']:+.3f} m   h<0: {'YES for %.2f s' % r['neg_duration_s'] if r['h_negative'] else 'no'}"
          f"   contact(h<-d_safe): {'YES' if r['contact'] else 'no'}")
    print(f"   v_obs max = {r['v_obs_max']:.2f} m/s   retreat peak = {r['retreat_peak']:.2f} m/s   "
          f"v_avail at min h = {r['v_avail_at_min_h']:.2f} m/s")
    cosv = r['push_vs_vobs_cos']
    print(f"   escape: outrun blend w max = {r['outrun_w_max']:.2f}"
          + (f", push·v_obs cos = {cosv:+.2f} (0 = orthogonal)" if cosv is not None else ", no lateral push"))
    print(f"   slack max: " + '  '.join(f"{k}={v:.3f}" for k, v in r['slack_max'].items()))
    print(f"   fallback ticks: {r['fallback_levels']}   livelock escapes: {r['livelock_escapes']} (max mag {r['livelock_max']:.2f})")
    print(f"   task: final joint error {r['task_final_err']:.3f} rad -> {'COMPLETED' if r['task_done'] else 'NOT completed'}")
    print(f"   SMOOTHNESS |d(qddot)/dt| [rad/s^3]: p50={r['jerk_p50']:7.1f}  p99={r['jerk_p99']:8.1f}  max={r['jerk_max']:8.1f}")
    print(f"   OSCILLAZIONE (dopo l'ingaggio, {r['engaged_s']:.1f} s): "
          f"inversioni del verso di allontanamento = {r['sep_sign_hz']:.1f} Hz, "
          f"gap picco-picco = {1000 * r['gap_ptp']:.0f} mm")
    ok = 'OK' if r['step_max'] <= r['slew_limit'] + 1e-6 else 'VIOLATED'
    print(f"   step per giunto [rad/s^2/tick]:      p99={r['step_p99']:7.2f}  max={r['step_max']:8.2f}"
          f"   (limite slew {r['slew_limit']:.1f} -> {ok})")


def scenarios(P, only=None, fps=30.0, verbose=None):
    F0 = Filter(P)
    cps = F0.control_points(Q0, np.zeros(NV))
    flange = cps[-1][1]
    y = np.array([0.0, 1.0, 0.0])
    goal_sweep = Q0.copy(); goal_sweep[0] += 0.8        # the task: swing joint 1 by 0.8 rad
    S = {}
    # static: an obstacle BESIDE the swing (above and slightly inside the arc),
    # close enough to engage the rows, not on the path — B1 is "steer around,
    # nominal continues", not "stop at a wall".
    S['static'] = dict(spheres=[Sphere(flange + np.array([0.0, 0.18, 0.40]), r=0.08)],
                       T=4.0, goal_q=goal_sweep)
    # around: the obstacle sits ON the swing's arc at flange height, halfway
    # along a 1.6 rad sweep (0.52 m of arc, the goal 0.17 m clear of it), with
    # room above and below. The joint-space nominal alone stops at the
    # barrier; the tangential bias and the livelock escape have to carry the
    # flange round it and the task must still complete.
    goal_wide = Q0.copy(); goal_wide[0] += 1.6
    rad = np.hypot(flange[0], flange[1])
    S['around'] = dict(spheres=[Sphere(np.array([rad * np.cos(0.8), rad * np.sin(0.8), flange[2]]),
                                       r=0.04)],
                       T=10.0, goal_q=goal_wide)
    # THE regression scenario: a static obstacle, and a camera that says it is
    # moving. Nothing in the scene moves; every metre per second reaching the
    # QP is the tracker's measured noise floor.
    S['static_noisy'] = dict(spheres=[Sphere(flange + np.array([0.0, 0.36, 0.0]), r=0.06)],
                             T=6.0, cam_noise=True)
    # THE report: the EE is driven INTO a static obstacle at a constant
    # Cartesian speed and the barrier has to hold it off. The reference never
    # stops, so this is the sustained standoff, not a transient.
    x = np.array([1.0, 0.0, 0.0])
    S['approach_hold'] = dict(spheres=[Sphere(flange + 0.40 * x, r=0.06)],
                              T=6.0, cart_ref=0.12 * x)
    S['slow'] = dict(spheres=[Sphere(flange + 0.55 * y, r=0.05, pursue=True, speed=0.3)], T=3.0)
    # The same approach WITH the camera noise on top: the arm must still react
    # to the 0.3 m/s that is real, through a noise floor of the same order.
    S['slow_noisy'] = dict(spheres=[Sphere(flange + 0.55 * y, r=0.05, pursue=True, speed=0.3)],
                           T=3.0, cam_noise=True)
    # fast: a 3 m/s pass AIMED at the flange on a straight line (a homing
    # pursuer above the arm's own top speed cannot be escaped by anything, so
    # it would only measure the pursuer). Above the flange's v_avail: B3.
    S['fast'] = dict(spheres=[Sphere(flange + 1.5 * y, vel=-3.0 * y, r=0.08)], T=0.9)
    # the same pass, seen from 1.5 m instead of the 0.7 m publish gate: what
    # the filter can do when perception is not the wall (0c: ~3.3-3.7 m/s)
    S['fast_seen_early'] = dict(spheres=[Sphere(flange + 1.5 * y, vel=-3.0 * y, r=0.08)],
                                T=0.9, max_thresh=1.5)
    S['ball'] = dict(spheres=[Sphere(flange + 2.0 * y, vel=-4.0 * y, r=0.05)], T=0.9)
    # the 0c boundary: a new object at ~1.5 m/s from the 0.7 m publish gate
    S['ball_slow'] = dict(spheres=[Sphere(flange + 2.0 * y, vel=-1.5 * y, r=0.05)], T=1.8)
    # wedge: two spheres either side of the +x path, the task drives the flange between them
    goal_fwd = Q0.copy(); goal_fwd[1] += 0.45; goal_fwd[3] += 0.55   # reach forward
    S['wedge'] = dict(spheres=[Sphere(flange + np.array([0.22, 0.12, 0.0]), r=0.06),
                               Sphere(flange + np.array([0.22, -0.12, 0.0]), r=0.06)],
                      T=7.0, goal_q=goal_fwd)
    out = []
    for name, kw in S.items():
        if only and name not in only:
            continue
        out.append(run_scenario(name, P, fps=fps, verbose=bool(verbose and name in verbose), **kw))
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--only', nargs='*')
    ap.add_argument('--fps', type=float, default=30.0)
    ap.add_argument('--latency-compensation', action='store_true')
    ap.add_argument('--no-outrun', action='store_true')
    ap.add_argument('--no-livelock', action='store_true')
    ap.add_argument('--raw-track', action='store_true',
                    help='disable the tracked-velocity conditioning (median + deadband), '
                         'i.e. reproduce the jitter it was added for')
    ap.add_argument('--trace', nargs='*', help='scenario names to print a time trace for')
    ap.add_argument('--bag', help='replay this rosbag through the real perception chain instead')
    ap.add_argument('--no-inject', dest='inject', action='store_false')
    ap.add_argument('--max-frames', type=int, default=900)
    args = ap.parse_args()
    over = {}
    if args.latency_compensation:
        over['enable_latency_compensation'] = True
    if args.no_outrun:
        over['enable_outrun_evasion'] = False
    if args.no_livelock:
        over['enable_livelock_escape'] = False
    if args.raw_track:
        over['obstacle_velocity_median'] = 1
        over['obstacle_velocity_track_deadband'] = 0.0
    P = shipped_params(**over)
    print(f"flags: source={P.obstacle_velocity_source} floor={P.obstacle_velocity_residual_floor} "
          f"lateral={P.enable_lateral_evasion} outrun={P.enable_outrun_evasion} "
          f"livelock={P.enable_livelock_escape} uncertainty={P.enable_uncertainty_margin} "
          f"latency_comp={P.enable_latency_compensation} fps={args.fps}")
    if args.bag:
        bag_replay(P, args.bag, max_frames=args.max_frames, inject=args.inject)
        return
    for r in scenarios(P, only=args.only, fps=args.fps, verbose=args.trace):
        print_report(r)


# ═════════════════════════════════════════════════════════════════════════════
#  Bag replay: the real perception chain into the real filter, open loop
# ═════════════════════════════════════════════════════════════════════════════

def bag_replay(P, bag, *, max_frames=900, inject=True):
    """Replay a bag through DistanceEngine + tracker + ConstraintBuilder + QP.

    Open loop: the joints follow the recording, the QP's output is computed
    and scored but not integrated (the arm in the bag did not have this
    filter). Reports what the filter would have asked for.
    """
    import sys
    sys.path.insert(0, os.path.join(PKG, 'scripts'))
    import compare_vobs as cv
    import trimesh
    from ament_index_python.packages import get_package_share_directory
    from cv_bridge import CvBridge

    from franka_experiments.utils.distance_engine import DistanceEngine
    from franka_experiments.utils.distance_utils import (
        compute_roi, define_control_points, load_extrinsics, load_robot_config)
    from franka_experiments.utils.mask_builder import MaskBuilder
    from franka_experiments.utils.obstacle_sim import InjectedSphere
    from franka_experiments.utils.obstacle_track_pipeline import ObstacleTrackPipeline
    from franka_experiments.utils.tf_manager import TFManager

    log = _Log()
    cfg = load_robot_config(os.path.join(PKG, 'config', 'fr3_complete.yaml'))
    robot_cfg, mask_cfg, mesh_cfg = cfg['robot'], cfg['mask'], cfg['meshes']
    dcfg = dict(cfg['distance']); dcfg['export_obstacle_cloud'] = True
    trk = cfg.get('tracking', {}) or {}
    R_base, t_base = load_extrinsics(os.path.join(PKG, 'config', 'camera_extrinsics.yaml'))
    R32, t32 = R_base.astype(np.float32), t_base.astype(np.float32)
    tf_buf, _ = cv.build_tf_buffer(bag)
    tf_mgr = TFManager(tf_buffer=tf_buf, base_frame=robot_cfg['base_frame'],
                       critical_links=['fr3_link8'], cache_max_age_s=None, logger=log)
    mesh_dir = get_package_share_directory(mesh_cfg.get('package', 'franka_description'))
    samples = {n: trimesh.load(os.path.join(mesh_dir, rel), force='mesh').sample(300)
               for n, rel in mesh_cfg['files'].items()}
    mb = MaskBuilder(link_mesh_samples=samples, R_base=R_base, t_base=t_base,
                     ee_link='fr3_link8', mask_cfg=mask_cfg, logger=log)
    eng = DistanceEngine(dcfg, logger=log)
    pipe = ObstacleTrackPipeline(
        voxel_m=trk['cluster_voxel_m'], min_cluster_points=trk['cluster_min_points'],
        max_clusters=trk['max_clusters'], max_cluster_radius=trk.get('cluster_max_radius_m'),
        depth_jump=trk['cluster_depth_jump_m'], contains_tol=trk['cluster_contains_tol_m'],
        q_jerk=trk['q_jerk'], sigma_meas=trk['sigma_meas_m'])
    sphere = InjectedSphere([0.0, 0.0, 1.6], [0.0, 0.0, -0.5], radius=0.15,
                            period=2.0, amplitude=0.5) if inject else None
    F = Filter(P)
    joints = cv.JointTrace(F.kin.model, [f'fr3_joint{i}' for i in range(1, 8)])
    K = None
    for topic, _, msg in cv.read_bag(bag, {'/NS_1/joint_states', '/camera/camera/aligned_depth_to_color/camera_info'}):
        if topic == '/NS_1/joint_states':
            st = msg.header.stamp
            joints.add(st.sec + st.nanosec * 1e-9, msg)
        elif K is None:
            K = np.array(msg.k).reshape(3, 3)
    joints.finish()
    mb.set_intrinsics(K)
    br = CvBridge()
    rec = dict(h=[], vobs=[], nc=[], slack=[], level=[], dnorm=[], sep=[], cap=[])
    n = 0
    for _, _, msg in cv.read_bag(bag, {'/camera/camera/aligned_depth_to_color/image_raw'}):
        n += 1
        if n > max_frames:
            break
        depth = br.imgmsg_to_cv2(msg, 'passthrough')
        st = msg.header.stamp; tc = st.sec + st.nanosec * 1e-9
        if sphere is not None:
            depth = depth.copy(); sphere.render(depth, tc, K)
        qv = joints.at(tc)
        if qv is None:
            continue
        q_full, v_full = qv
        q, qdot = q_full[:NV], v_full[:NV]
        tr = tf_mgr.lookup_all(robot_cfg['segment_links'], st)
        if not tr:
            continue
        cps = define_control_points(tr, robot_cfg, dcfg)
        H, W = depth.shape
        mb.rebuild(tr, depth.shape)
        roi = compute_roi(mb.search_exclusion_mask, H, W, 10, 90) or (10, 10, W - 10, H - 10)
        res, _ = eng.compute(depth=depth, cx_f32=np.float32(K[0, 2]), cy_f32=np.float32(K[1, 2]),
                             fx_inv_f32=np.float32(1 / K[0, 0]), fy_inv_f32=np.float32(1 / K[1, 1]),
                             R_base_f32=R32, t_base_f32=t32, control_points=cps,
                             x=np.array([roi[0], roi[2]]), y=np.array([roi[1], roi[3]]), step=10,
                             search_exclusion_mask=mb.search_exclusion_mask,
                             ee_source_mask=mb.ee_source_mask,
                             dilation_margins_px=mb.dilation_margins_px, frame_stamp=tc)
        if res is None:
            continue
        pipe.update(eng.last_obstacle_cloud, R_base, t_base, stamp=tc)
        items = []
        for r in res:
            if not np.isfinite(r.distance) or r.closest_obstacle_point is None:
                continue
            info = pipe.track_info_for_point(np.asarray(r.closest_obstacle_point, float))
            items.append(Obstacle(link=r.end_link, d=float(r.distance),
                                  pr=np.asarray(r.point, float),
                                  ph=np.asarray(r.closest_obstacle_point, float), conf=1.0,
                                  v_vec=info.velocity, frames_seen=info.frames_seen,
                                  vel_cov=info.velocity_cov, track_id=info.track_id,
                                  a_vec=info.acceleration, pos_cov=info.position_cov,
                                  pv_cov=info.pos_vel_cov))
        if not items or min(o.d for o in items) > 0.7:
            continue
        con = F.builder.build(JointSnap(q=q, qdot=qdot, stamp=tc),
                              ObstacleSnap(items=tuple(items), stamp=tc, t_cap=tc), tc)
        if con is None:
            continue
        qddot, slack, nom_used, level, _ = F.tick(tc, con, q, qdot, np.zeros(NV))
        i = int(np.argmin(con.h_bar[con.group == G_OBS]))
        a = con.A[con.group == G_OBS][i]
        rec['h'].append(float(con.h_bar[con.group == G_OBS].min()))
        rec['vobs'].append(float(con.v_obs.max()))
        rec['nc'].append(int(con.A.shape[0]))
        rec['slack'].append(slack.copy()); rec['level'].append(level)
        rec['dnorm'].append(float(np.linalg.norm(qddot - nom_used)))
        rec['sep'].append(float(a @ (qdot + qddot * F.dt)))
        rec['cap'].append(float(con.cap_v[:con.n_rtr].min()) if con.n_rtr else np.nan)
    h = np.array(rec['h']); sl = np.array(rec['slack']); lv = np.array(rec['level'])
    print(f"\n== bag replay {bag} (frames with an obstacle in band: {h.size}, inject={inject})")
    print(f"   min h = {h.min():+.3f} m   frames h<0: {int((h < 0).sum())} ({100 * (h < 0).mean():.0f}%)   "
          f"n_c med={np.median(rec['nc']):.0f} max={max(rec['nc'])}")
    print(f"   v_obs max = {max(rec['vobs']):.2f} m/s   commanded one-step separation rate max = {max(rec['sep']):.2f} m/s   "
          f"retreat cap min = {np.nanmin(rec['cap']):.2f} m/s")
    print(f"   slack max: " + '  '.join(f"{GROUP_NAMES[g]}={sl[:, g].max():.3f}" for g in range(N_SLACK)))
    print(f"   fallback ticks: {dict(zip(LEVEL_NAMES, [int((lv == i).sum()) for i in range(3)]))}   "
          f"dnorm med={np.median(rec['dnorm']):.2f} max={max(rec['dnorm']):.2f}")


if __name__ == '__main__':
    main()
