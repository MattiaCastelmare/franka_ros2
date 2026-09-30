#!/usr/bin/env python3
"""CLOSED-LOOP bench: a recorded ball throw against the SHIPPED cbf_safety_filter and a simulated arm.

WHY
---
bag_replay.launch.py is open loop: q and q̇ are the recorded ones, so the arm never moves out of the
way and nothing can be said about what the command DOES to the arm (overshoot, the return pull of the
task, velocity against the firmware envelope, jerk). This bench closes that loop without a robot.

WHAT IS REAL
  * the filter: the actual ``CBFSafetyFilter`` node class, constructed in-process with the launch's own
    parameters (+ an optional override YAML) and driven tick by tick on a simulated clock. Nothing is
    re-implemented, so every gate, EMA, slew box, zone and fallback rung is the shipped one;
  * the perception: the ``/cbf/per_link_distances`` messages produced by ``bag_replay.launch.py`` from the
    recorded depth stream (obstacle points, tracks, velocities, covariances, capture stamps and the
    measured capture -> receipt latency of each message);
  * the ball: colour-tracked ground truth (rosbag/<bag>_truth.npz), used only to score clearance.
WHAT IS MODELLED
  * the arm: q̈ = 0.96 · first-order-lag(10 ms)(q̈_safe)  (identified on ball_throws_3: no dead time, rms
    residual 0.6 rad/s² against a 1.1 rad/s² signal; ``--plant-gain/--plant-tau`` change it);
  * the task: pentagon_qddot_commander's measured-channel law (Cartesian PD on the recorded ee_desired,
    DLS pseudo-inverse, posture null-space, cart_err_max clamp). It is what pulls the arm BACK after a
    dodge, which a recorded q̈_nom cannot reproduce. ``--check-nominal`` compares it to the recorded one.
  * obstacle rows for the simulated arm: the recorded control points are carried on their links
    (p_local = M_link(q_rec)⁻¹ p) to the simulated pose, the obstacle point and track stay as recorded, and
    the surface gap is re-derived with the recorded capsule offset. The obstacle does not react to the arm.

USAGE (inside the container, sourced; PYTHONPATH must include the package source)
    python3 scripts/ball_closed_loop.py --bag rosbag/ball_throws_3 --dist rosbag/replay_base_ball_throws_3 \\
        --truth rosbag/ball_throws_3_truth.npz [--cbf-overrides x.yaml] [--json out.json]
"""
from __future__ import annotations

import argparse
import importlib.util
import json
import os
import sys
import tempfile

import numpy as np
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
sys.path.insert(0, HERE)

from ball_throw_eval import read_bag, _stamp, passes, DIST  # noqa: E402

K = [f'fr3_joint{i}' for i in range(1, 8)]
FAST = '/NS_1/franka/joint_states'
SLOW = '/NS_1/joint_states'
EE_DES = '/NS_1/ee_desired'
EE_ACT = '/NS_1/ee_actual'
NOM = '/NS_1/qddot_nom'
R_CAP = 0.05        # capsule radius used for the clearance score (same for every variant)
R_BALL = 0.033


# ── data ─────────────────────────────────────────────────────────────────────

class Recording:
    def __init__(self, bag, dist_bag):
        js, slow, eed, nom = [], [], [], []
        for tp, recv, m in read_bag(bag, {FAST, SLOW, EE_DES, NOM}):
            if tp in (FAST, SLOW):
                d = dict(zip(m.name, zip(m.position, m.velocity)))
                if all(k in d for k in K):
                    (js if tp == FAST else slow).append(
                        (_stamp(m) or recv, [d[k][0] for k in K], [d[k][1] for k in K]))
            elif tp == EE_DES:
                eed.append((_stamp(m) or recv, [m.point.x, m.point.y, m.point.z]))
            else:
                nom.append((recv, list(m.data[:7])))
        if not js:                      # older bags: only the 30 Hz republisher was recorded
            js = slow
        js.sort(key=lambda x: x[0])
        self.tj = np.array([j[0] for j in js])
        self.Qj = np.array([j[1] for j in js])
        self.Vj = np.array([j[2] for j in js])
        eed.sort(key=lambda x: x[0])
        self.te = np.array([e[0] for e in eed]) if eed else np.zeros(0)
        self.Pd = np.array([e[1] for e in eed]) if eed else np.zeros((0, 3))
        self.tn = np.array([n[0] for n in nom])
        self.Qn = np.array([n[1] for n in nom])
        self.dist = []          # (t_cap, t_recv, msg)
        for tp, recv, m in read_bag(dist_bag, {DIST}):
            self.dist.append((_stamp(m), recv, m))
        self.dist.sort(key=lambda x: x[0])
        self.td = np.array([d[0] for d in self.dist])

    def q_at(self, t):
        i = int(np.clip(np.searchsorted(self.tj, t), 1, len(self.tj) - 1))
        a = (t - self.tj[i - 1]) / max(self.tj[i] - self.tj[i - 1], 1e-9)
        a = min(max(a, 0.0), 1.0)
        return (1 - a) * self.Qj[i - 1] + a * self.Qj[i], (1 - a) * self.Vj[i - 1] + a * self.Vj[i]

    def ref_at(self, t):
        """p_d, v_d, a_d from the recorded ee_desired (smoothed central differences)."""
        i = int(np.clip(np.searchsorted(self.te, t), 4, len(self.te) - 5))
        sl = slice(i - 4, i + 5)
        tt = self.te[sl] - self.te[i]
        A = np.vander(tt, 4, increasing=True)              # cubic LS fit around t
        coef = np.linalg.lstsq(A, self.Pd[sl], rcond=None)[0]
        dt = t - self.te[i]
        p = coef[0] + coef[1] * dt + coef[2] * dt ** 2 + coef[3] * dt ** 3
        v = coef[1] + 2 * coef[2] * dt + 3 * coef[3] * dt ** 2
        a = 2 * coef[2] + 6 * coef[3] * dt
        return p, v, a


class CircleRef:
    """The commander's circular path with its obstacle-driven phase governor.

    Centre / radius / cycle time are the config's (path_*); the PHASE and its direction come from the
    recorded end-effector position at the window start (forward kinematics of the recorded q), so it works on
    bags that never recorded /NS_1/ee_desired. The phase is then advanced by the governor's own law (sigma
    from the filter's published nearest gap), so the reference parks and resumes the way the real one does
    instead of replaying a recorded phase that belongs to a different arm motion.
    """
    R_FULL, R_STOP, SIG_MIN = 1.5, 0.8, 0.05
    CENTER, RADIUS, CYCLE = (0.4, 0.0, 0.45), 0.25, 6.0

    def __init__(self, rec, t_start, d_safe, ee):
        self.x, self.cy, self.cz = self.CENTER
        self.r = self.RADIUS

        def phase(t):
            p = ee(rec.q_at(t)[0])
            return np.arctan2(p[2] - self.cz, p[1] - self.cy)
        a, b = phase(t_start - 0.15), phase(t_start + 0.15)
        d = (b - a + np.pi) % (2 * np.pi) - np.pi
        self.w = float(np.sign(d) * 2 * np.pi / self.CYCLE) if abs(d) > 1e-3 else 2 * np.pi / self.CYCLE
        # the reference leads the measured EE by a few cm of arc: one tracking lag of phase
        self.phi = float(phase(t_start) + 0.03 * np.sign(self.w))
        self.d_full, self.d_stop = self.R_FULL * d_safe, self.R_STOP * d_safe

    def sigma(self, d_obs):
        if not np.isfinite(d_obs) or d_obs >= self.d_full:
            return 1.0
        if d_obs <= self.d_stop:
            return self.SIG_MIN
        f = (d_obs - self.d_stop) / max(self.d_full - self.d_stop, 1e-9)
        return self.SIG_MIN + (1 - self.SIG_MIN) * f

    def step(self, dt, d_obs):
        sg = self.sigma(d_obs)
        self.phi += self.w * sg * dt
        return sg

    def ref(self, sg=1.0):
        w = self.w * sg
        c, s_ = np.cos(self.phi), np.sin(self.phi)
        p = np.array([self.x, self.cy + self.r * c, self.cz + self.r * s_])
        v = np.array([0.0, -self.r * s_ * w, self.r * c * w])
        a = np.array([0.0, -self.r * c * w * w, -self.r * s_ * w * w])
        return p, v, a


# ── the task law (pentagon_qddot_commander, measured channel) ────────────────

class Commander:
    KP, KD, KPR, KDR = 20.0, 9.0, 5.0, 6.0
    K_NULL, D_NULL = 5.0, 2.0
    LAM_MIN, LAM_MAX, MANIP = 1e-4, 5e-2, 0.05
    CART_ERR_MAX = 0.08
    QDD_MAX = np.array([6.0, 2.585, 3.5, 4.0, 17.0, 5.5, 17.0])

    def __init__(self, q_home):
        import pinocchio as pin
        from franka_experiments.utils.kinematics import (
            generate_urdf_from_xacro, load_pinocchio_model, resolve_frame_id, resolve_arm_joint_ids, so3_log)
        self.pin, self.so3_log = pin, so3_log
        self.model, self.data = load_pinocchio_model(generate_urdf_from_xacro())
        self.fid = resolve_frame_id(self.model, 'fr3_hand_tcp')
        jids = resolve_arm_joint_ids(self.model)
        self.iq = [self.model.joints[j].idx_q for j in jids]
        self.iv = [self.model.joints[j].idx_v for j in jids]
        self.qn = pin.neutral(self.model)
        self.q_home = np.asarray(q_home, float)
        self.R_des = self._fk(self.q_home, np.zeros(7))[1]

    def _fk(self, q, qd, jac=False):
        pin = self.pin
        qf = self.qn.copy(); vf = np.zeros(self.model.nv)
        for k in range(7):
            qf[self.iq[k]] = q[k]; vf[self.iv[k]] = qd[k]
        pin.computeAllTerms(self.model, self.data, qf, vf)
        pin.updateFramePlacements(self.model, self.data)
        M = self.data.oMf[self.fid]
        if not jac:
            return np.array(M.translation), np.array(M.rotation)
        J = pin.getFrameJacobian(self.model, self.data, self.fid, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
        dJ = pin.getFrameJacobianTimeVariation(self.model, self.data, self.fid,
                                               pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
        return np.array(M.translation), np.array(M.rotation), J[:, self.iv], dJ[:, self.iv]

    def ee(self, q):
        return self._fk(q, np.zeros(7))[0]

    def qddot(self, q, qd, p_d, v_d, a_d):
        p, R, J, dJ = self._fk(q, qd, jac=True)
        e = p_d - p
        n = np.linalg.norm(e)
        if self.CART_ERR_MAX > 0 and n > self.CART_ERR_MAX:
            e = e * (self.CART_ERR_MAX / n)
        e_rot = -self.R_des @ self.so3_log(self.R_des.T @ R)
        ed = np.zeros(6)
        ed[:3] = v_d - J[:3] @ qd
        ed[3:] = -J[3:] @ qd
        xdd = np.zeros(6)
        xdd[:3] = a_d + self.KP * e + self.KD * ed[:3]
        xdd[3:] = self.KPR * e_rot + self.KDR * ed[3:]
        JJt = J @ J.T
        w = np.sqrt(max(0.0, np.linalg.det(JJt)))
        f = (1 - w / self.MANIP) ** 2 if w < self.MANIP else 0.0
        lam = self.LAM_MIN + (self.LAM_MAX - self.LAM_MIN) * f
        Jp = np.linalg.solve(JJt + lam * np.eye(6), J).T
        qdd = Jp @ (xdd - dJ @ qd)
        N = np.eye(7) - Jp @ J
        qdd = qdd + N @ (-self.K_NULL * (q - self.q_home) - self.D_NULL * qd)
        return np.clip(qdd, -self.QDD_MAX, self.QDD_MAX)


# ── the real filter, headless ────────────────────────────────────────────────

class _Sink:
    def __init__(self):
        self.last = None

    def publish(self, msg):
        self.last = msg


_RAW = {}


def make_filter(overrides):
    """Construct the shipped CBFSafetyFilter in-process with the launch's parameters + overrides."""
    import rclpy
    spec = importlib.util.spec_from_file_location(
        'tcs', os.path.join(PKG, 'launch', 'torque_control_stack.launch.py'))
    tcs = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(tcs)
    params = {}
    for d in tcs._cbf_parameters(dict(tcs._DEFAULTS)):
        params.update(d)
    params.update(overrides or {})
    # keys the filter reads straight from the YAML (lists, optionals) are not ROS parameters: patch the loader
    from franka_experiments.utils import config as _cfg
    raw = {k: params.pop(k) for k in list(params) if k in _cfg.CBF_RAW_KEYS}
    if raw and not getattr(_cfg.load_package_yaml, '_patched', False):
        _orig = _cfg.load_package_yaml

        def _patched(pkg, rel, _o=_orig):
            d = _o(pkg, rel)
            if rel.endswith('fr3_control.yaml'):
                d['params'].update(_RAW)
            return d
        _patched._patched = True
        _cfg.load_package_yaml = _patched
    _RAW.clear()
    _RAW.update(raw)
    f = tempfile.NamedTemporaryFile('w', suffix='.yaml', delete=False)
    yaml.safe_dump({'cbf_safety_filter': {'ros__parameters': params}}, f)
    f.close()
    if not rclpy.ok():
        rclpy.init(args=['--ros-args', '--params-file', f.name, '--log-level', os.environ.get('CL_LOG', 'warn')])
    from franka_experiments.nodes.cbf_safety_filter import CBFSafetyFilter
    _use_source_share()
    node = CBFSafetyFilter()
    node._pub = _Sink()
    node._status_pub = _Sink()
    node._priority_set = True
    return node


def _use_source_share():
    """Resolve the franka_experiments share dir to the SOURCE tree (the install is a stale copy)."""
    for m in list(sys.modules.values()):
        orig = getattr(m, 'get_package_share_directory', None)
        if orig is not None and not getattr(orig, '_src', False):
            def patched(pkg, _o=orig):
                return PKG if pkg == 'franka_experiments' else _o(pkg)
            patched._src = True
            m.get_package_share_directory = patched


class Clock:
    t = 0.0


def run_pass(rec, p, args, node_factory):
    """One simulated window around pass ``p``. Returns the metrics dict."""
    import pinocchio as pin
    from builtin_interfaces.msg import Time
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Float64MultiArray
    from franka_experiments.utils.cbf_hard_limits import fr3_velocity_envelope
    from franka_experiments.utils.kinematics import build_urdf_no_hand

    t_c = p['tc']
    t0, t1 = t_c - args.pre, t_c + args.post
    clk = Clock()
    node = node_factory()
    node._now = lambda: clk.t
    sink = node._pub

    model = pin.buildModelFromUrdf(build_urdf_no_hand())
    data = model.createData()

    def place(q):
        pin.forwardKinematics(model, data, q)
        pin.updateFramePlacements(model, data)

    def frame_M(q, name):
        place(q)
        return data.oMf[model.getFrameId(name)]

    # control-point table for the clearance score (same set for every variant and for the recording)
    from franka_experiments.utils.kinematics import build_urdf_no_hand as _b  # noqa: F401
    segs = [('fr3_link3', 'fr3_link4', 2), ('fr3_link4', 'fr3_link5', 2), ('fr3_link5', 'fr3_link6', 2),
            ('fr3_link6', 'fr3_link7', 2), ('fr3_link7', 'fr3_link8', 3)]

    def cps(q):
        place(q)
        pos = {n: data.oMf[model.getFrameId(n)].translation.copy() for n in
               [f'fr3_link{i}' for i in range(3, 9)]}
        out = []
        for s, e, n in segs:
            ts = [(k + 1) / n for k in range(n)] if e == 'fr3_link8' else [(k + 1) / (n + 1) for k in range(n)]
            out += [pos[s] + t * (pos[e] - pos[s]) for t in ts]
        return np.array(out)

    q, qd = rec.q_at(t0)
    q, qd = q.copy(), qd.copy()
    cmd = Commander(q_home=args.q_home)
    if args.nom_cap > 0:
        cmd.QDD_MAX = np.minimum(cmd.QDD_MAX, args.nom_cap)
    tt, ok, PB = args.truth
    plant_a = np.zeros(7)
    circ = CircleRef(rec, t0, node.P.d_safe, cmd.ee)
    sg = 1.0
    hist_t, hist_q = [t0], [q.copy()]
    # messages in the window, with their own capture -> receipt latency
    lo, hi = np.searchsorted(rec.td, t0 - 0.05), np.searchsorted(rec.td, t1)
    pend = []
    for i in range(lo, hi):
        tc_, rv, m = rec.dist[i]
        pend.append((tc_ + (rv - tc_ if 0 < rv - tc_ < 0.3 else 0.044), i))
    pend.sort()
    pk = 0
    dt = 0.01
    steps = int(round((t1 - t0) / dt))
    log = {k: [] for k in ['t', 'safe', 'nom', 'q', 'qd', 'acc', 'cl_sim', 'cl_rec', 'ee_err', 'off_sim', 'off_rec']}
    a_real = np.zeros(7)
    last_nom_t = -1.0
    for s in range(steps):
        t = t0 + s * dt
        clk.t = t
        # deliver perception that has arrived
        while pk < len(pend) and pend[pk][0] <= t:
            _, i = pend[pk]
            pk += 1
            t_cap, _, m = rec.dist[i]
            msg = _reproject(m, rec, hist_t, hist_q, t_cap, model, data, pin)
            node._on_distances(msg)
        js = JointState()
        js.name = list(K)
        js.position = [float(x) for x in q]
        js.velocity = [float(x) for x in qd]
        js.header.stamp = _ros_time(t)
        node._on_joint_state(js)
        p_d, v_d, a_d = circ.ref(sg)
        qdd_nom = cmd.qddot(q, qd, p_d, v_d, a_d)
        if args.oracle > 0:
            qdd_nom = qdd_nom + _oracle(args, t, t_c, q, tt, ok, PB, model, data, pin, cps_fn=cps)
        nm = Float64MultiArray()
        nm.data = [float(x) for x in qdd_nom]
        node._on_qddot_nom(nm)
        if s % 2 == 0:
            node._update_constraints()
        node._qp_tick()
        safe = np.array(sink.last.data[:7]) if sink.last is not None else np.zeros(7)
        st = node._status_pub.last
        d_obs = float(st.data[4]) if st is not None and len(st.data) > 4 else float('inf')
        sg = circ.step(dt, d_obs)
        if args.no_cbf:
            safe = np.clip(qdd_nom, -10.0, 10.0)
        # plant, 1 ms sub-steps
        for _ in range(10):
            a_real += (args.plant_gain * safe - a_real) * (0.001 / (args.plant_tau + 0.001))
            qd = qd + a_real * 0.001
            q = q + qd * 0.001
        hist_t.append(t + dt); hist_q.append(q.copy())
        ball = _ball_at(tt, ok, PB, t + dt)
        qr, _ = rec.q_at(t + dt)
        cl_s = cl_r = np.nan
        if ball is not None:
            cl_s = float(np.min(np.linalg.norm(cps(q) - ball, axis=1)) - R_CAP - R_BALL)
            cl_r = float(np.min(np.linalg.norm(cps(qr) - ball, axis=1)) - R_CAP - R_BALL)
        if args.probe and abs((t - t_c) - args.probe) < 0.005 and node._con is not None:
            con = node._con
            from franka_experiments.utils.cbf_qp_assembly import build_row_rhs
            h, _ = build_row_rhs(con, qd, node._qdot_cbf, k0=node.P.k0_cbf, k1=node.P.k1_cbf,
                                 retreat_horizon=node.P.retreat_cap_horizon_s, speed_horizon=node.P.link_speed_horizon_s)
            amax = np.abs(con.A) @ node._ub
            ach = con.A @ safe
            print(f'  PROBE t-tc={t - t_c:+.3f}  n_c={con.A.shape[0]} safe={np.round(safe, 1)} nom={np.round(qdd_nom, 1)}')
            print(f'    slack by group {np.round(node._diag_slack, 2)}  box ub={np.round(node._box_ub[:7], 1)}')
            obs_i = np.nonzero(con.group == 0)[0]
            order = obs_i[np.argsort(h[obs_i])[:7]]
            for i in order:
                print(f'    row {i:2d} {str(con.links[i])[:16]:16s} demand={-h[i]:7.2f} amax={amax[i]:5.2f} a.safe={ach[i]:6.2f} '
                      f'|a|={np.linalg.norm(con.A[i]):.2f} vobs={con.v_obs[i]:.2f} h_bar={con.h_bar[i]:.3f}')
        if args.trace and s % 10 == 0:
            print(f'   t-tc={t - t_c:+5.2f} d_obs={d_obs:5.3f} sg={sg:4.2f} ee_err={float(np.linalg.norm(p_d - cmd.ee(q)))*100:5.1f}cm '
                  f'|safe|={np.abs(safe).max():5.2f} |nom|={np.abs(qdd_nom).max():5.2f} |qd|={np.abs(qd).max():4.2f} '
                  f'clr={_fmt(_ball_at(tt, ok, PB, t), cps, q)}')
        def _off(qq):
            e = cmd.ee(qq)
            return float(np.hypot(np.hypot(e[1] - circ.cy, e[2] - circ.cz) - circ.r, e[0] - circ.x))
        log['off_sim'].append(_off(q)); log['off_rec'].append(_off(qr))
        log['t'].append(t); log['safe'].append(safe); log['nom'].append(qdd_nom)
        log['q'].append(q.copy()); log['qd'].append(qd.copy()); log['acc'].append(a_real.copy())
        log['cl_sim'].append(cl_s); log['cl_rec'].append(cl_r)
        log['ee_err'].append(float(np.linalg.norm(p_d - cmd.ee(q))))
    try:
        node.destroy_node()
    except Exception:
        pass
    return _metrics(log, t_c, dt, fr3_velocity_envelope)


_ORACLE = {}


def _oracle(args, t, t_c, q, tt, ok, PB, model, data, pin, cps_fn):
    """EXPERIMENT: a perfectly informed sideways dodge, to bound what the arm could physically do.

    Knows the ball's true line from ``args.oracle_lead`` s before closest approach; picks the control point
    that would pass closest, and pushes it perpendicular to the flight with a raised-cosine acceleration of
    peak ``args.oracle`` m/s² until 0.1 s after closest approach.
    """
    t_e = t_c - args.oracle_lead
    if t < t_e or t > t_c + 0.10:
        return np.zeros(7)
    if 'sel' not in _ORACLE or _ORACLE.get('tc') != t_c:
        i = np.argmin(abs(tt - t_e))
        idx = [j for j in range(i - 3, i + 4) if ok[j]]
        A = np.c_[tt[idx] - tt[i], np.ones(len(idx))]
        v = np.linalg.lstsq(A, PB[idx], rcond=None)[0][0]
        p0 = PB[i]
        ts = np.arange(0, 0.6, 0.01)
        best = None
        C = cps_fn(q)
        for k, c in enumerate(C):
            d = np.linalg.norm(p0 + np.outer(ts, v) - c, axis=1)
            if best is None or d.min() < best[0]:
                j = int(np.argmin(d))
                best = (d.min(), k, p0 + ts[j] * v, v)
        _, k, pc, v = best
        _ORACLE.update(sel=k, tc=t_c, v=v / max(np.linalg.norm(v), 1e-9), pc=pc)
    k, vh = _ORACLE['sel'], _ORACLE['v']
    C = cps_fn(q)
    off = C[k] - _ORACLE['pc']
    u = off - (off @ vh) * vh
    n = np.linalg.norm(u)
    if n < 1e-6:
        u = np.cross(vh, [0, 0, 1.0]); n = np.linalg.norm(u)
    u = u / n
    # point Jacobian of control point k on its link: use the distal link frame of the segment
    segs = [('fr3_link3', 'fr3_link4', 2), ('fr3_link4', 'fr3_link5', 2), ('fr3_link5', 'fr3_link6', 2),
            ('fr3_link6', 'fr3_link7', 2), ('fr3_link7', 'fr3_link8', 3)]
    names = []
    for sname, e, nn in segs:
        names += [e] * nn
    fid = model.getFrameId(names[k])
    pin.computeJointJacobians(model, data, q)
    pin.updateFramePlacements(model, data)
    J = pin.getFrameJacobian(model, data, fid, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
    r = C[k] - data.oMf[fid].translation
    Jp = J[:3] + np.cross(J[3:].T, r).T
    ph = (t - t_e) / (t_c + 0.10 - t_e)
    w = 0.5 * (1 - np.cos(2 * np.pi * ph))
    a = args.oracle * w * u
    return Jp.T @ np.linalg.solve(Jp @ Jp.T + 1e-3 * np.eye(3), a)


def _fmt(ball, cps, q):
    if ball is None:
        return '  -  '
    return f'{(np.min(np.linalg.norm(cps(q) - ball, axis=1)) - R_CAP - R_BALL) * 100:5.1f}'


def _ros_time(t):
    from builtin_interfaces.msg import Time
    m = Time()
    m.sec = int(t)
    m.nanosec = int((t - int(t)) * 1e9)
    return m


def _ball_at(tt, ok, P, t):
    j = int(np.clip(np.searchsorted(tt, t), 1, len(tt) - 1))
    good = [k for k in (j - 1, j) if ok[k] and abs(tt[k] - t) < 0.08]
    if not good:
        return None
    if len(good) == 2:
        a = (t - tt[j - 1]) / max(tt[j] - tt[j - 1], 1e-9)
        return (1 - a) * P[j - 1] + a * P[j]
    return P[good[0]]


def _reproject(m, rec, hist_t, hist_q, t_cap, model, data, pin):
    """The recorded message, with every control point carried to the SIMULATED pose at capture time."""
    from franka_msgs.msg import MultiLinkDistance
    out = MultiLinkDistance()
    out.header = m.header
    q_rec, _ = rec.q_at(t_cap)
    i = int(np.clip(np.searchsorted(hist_t, t_cap), 0, len(hist_t) - 1))
    q_sim = hist_q[i]

    def M(q, name):
        pin.forwardKinematics(model, data, q)
        pin.updateFramePlacements(model, data)
        return data.oMf[model.getFrameId(name)]

    offs = []
    for ld in m.links:
        if ld.valid and ld.distance > 1e-6:
            pr = np.array([ld.closest_point_robot.x, ld.closest_point_robot.y, ld.closest_point_robot.z])
            ph = np.array([ld.closest_point_human.x, ld.closest_point_human.y, ld.closest_point_human.z])
            offs.append(np.linalg.norm(pr - ph) - ld.distance)
    off_def = float(np.median(offs)) if offs else 0.06
    cache = {}
    links = []
    for ld in m.links:
        nl = type(ld)()
        for f in nl.get_fields_and_field_types():
            setattr(nl, f, getattr(ld, f))
        if ld.valid:
            name = ld.robot_link_name
            if name not in cache:
                cache[name] = (M(q_rec, name), M(q_sim, name))
            Mr, Ms = cache[name]
            pr = np.array([ld.closest_point_robot.x, ld.closest_point_robot.y, ld.closest_point_robot.z])
            ph = np.array([ld.closest_point_human.x, ld.closest_point_human.y, ld.closest_point_human.z])
            off = (np.linalg.norm(pr - ph) - ld.distance) if ld.distance > 1e-6 else off_def
            prs = Ms.act(Mr.actInv(pr))
            v = prs - ph
            n = np.linalg.norm(v)
            nl.closest_point_robot.x, nl.closest_point_robot.y, nl.closest_point_robot.z = map(float, prs)
            nl.distance = float(max(n - off, 0.0))
            if n > 1e-9:
                nl.direction.x, nl.direction.y, nl.direction.z = map(float, v / n)
        links.append(nl)
    out.links = links
    return out


def _metrics(log, t_c, dt, envelope):
    t = np.array(log['t'])
    S = np.array(log['safe']); N = np.array(log['nom']); QD = np.array(log['qd']); Q = np.array(log['q'])
    A = np.array(log['acc'])
    jerk_cmd = np.abs(np.diff(S, axis=0)) / dt
    jerk_real = np.abs(np.diff(A, axis=0)) / dt
    wt = (t > t_c - 0.6) & (t < t_c + 0.8)                   # the manoeuvre
    w = wt[1:]
    vr = []
    for q, qd in zip(Q, QD):
        up, lo = envelope(q)
        r = np.where(qd >= 0, qd / np.maximum(up, 1e-3), qd / np.minimum(lo, -1e-3))
        vr.append(np.max(r))
    cl_s = np.array(log['cl_sim']); cl_r = np.array(log['cl_rec'])
    # reversals: Schmitt-trigger sign changes of each joint's command (|q̈| > 1.5 rad/s² to count)
    rev = 0
    for j in range(S.shape[1]):
        state = 0
        for x in S[wt][:, j]:
            if abs(x) > 1.5 and np.sign(x) != state:
                rev += 1 if state != 0 else 0
                state = int(np.sign(x))
    tv = float(np.abs(np.diff(S[wt], axis=0)).sum())
    err_all = np.array(log['ee_err'])
    after = t > t_c
    rec_t = next((float(tt_ - t_c) for tt_, e_ in zip(t[after], err_all[after]) if e_ < 0.05), float('nan'))
    dev = np.linalg.norm(S - N, axis=1)
    err = np.array(log['ee_err'])
    return dict(
        clr_sim=float(np.nanmin(cl_s)) if np.isfinite(cl_s).any() else None,
        clr_rec=float(np.nanmin(cl_r)) if np.isfinite(cl_r).any() else None,
        peak_cmd=float(np.abs(S[wt]).max()),
        peak_acc=float(np.abs(A[wt]).max()),
        jerk_cmd_p99=float(np.percentile(jerk_cmd[w].max(axis=1), 99)),
        jerk_cmd_max=float(jerk_cmd[w].max()),
        jerk_real_p99=float(np.percentile(jerk_real[w].max(axis=1), 99)),
        flips=int(rev), tv=tv, recover_s=rec_t,
        dev_rms=float(np.sqrt(np.mean(dev[wt] ** 2))),
        vratio_max=float(np.max(np.array(vr)[wt])),
        qd_peak=float(np.abs(QD[wt]).max()),
        ee_err_max=float(err[wt].max()),
        ee_err_end=float(err[-1]),
        off_sim=float(np.max(np.array(log['off_sim'])[wt])), off_rec=float(np.max(np.array(log['off_rec'])[wt])),
    )


# ── CLI ──────────────────────────────────────────────────────────────────────

def find_passes(rec, truth):
    tt, ok, P = truth
    rows_t = rec.td
    cpr = []
    for _, _, m in rec.dist:
        cpr.append(np.array([[l.closest_point_robot.x, l.closest_point_robot.y, l.closest_point_robot.z]
                             for l in m.links if l.valid]) if m.links else np.zeros((0, 3)))
    return passes(tt, ok, P, rows_t, cpr, 0.6, 1.5)


def check_nominal(rec, args):
    """Compare the task-law replica against the recorded q̈_nom on stretches where the filter is quiet."""
    cmd = Commander(q_home=args.q_home)
    errs = []
    for t in rec.tn[::40]:
        if t < rec.tj[0] + 1 or t > rec.tj[-1] - 1:
            continue
        q, qd = rec.q_at(t)
        p, v, a = rec.ref_at(t)
        mine = cmd.qddot(q, qd, p, v, a)
        i = np.searchsorted(rec.tn, t)
        errs.append(np.abs(mine - rec.Qn[min(i, len(rec.tn) - 1)]).max())
    errs = np.array(errs)
    print(f'nominal replica vs recorded q̈_nom, max over joints: median {np.median(errs):.2f} '
          f'p90 {np.percentile(errs, 90):.2f} p99 {np.percentile(errs, 99):.2f} rad/s² (n={len(errs)})')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--bag', required=True)
    ap.add_argument('--dist', required=True, help='bag_replay output holding /cbf/per_link_distances')
    ap.add_argument('--truth', required=True)
    ap.add_argument('--cbf-overrides', default='')
    ap.add_argument('--set', nargs='*', default=[], help='key=value CBF parameter overrides (YAML values)')
    ap.add_argument('--pre', type=float, default=2.0)
    ap.add_argument('--post', type=float, default=1.0)
    ap.add_argument('--plant-gain', type=float, default=0.96)
    ap.add_argument('--plant-tau', type=float, default=0.010)
    ap.add_argument('--only', type=int, nargs='*', default=None, help='1-based pass numbers')
    ap.add_argument('--check-nominal', action='store_true')
    ap.add_argument('--trace', action='store_true')
    ap.add_argument('--oracle', type=float, default=0.0, help='EXPERIMENT: perfectly informed sideways dodge, peak m/s²')
    ap.add_argument('--oracle-lead', type=float, default=0.30)
    ap.add_argument('--nom-cap', type=float, default=0.0, help='cap |q̈_nom| per joint at this value [rad/s²] (experiment)')
    ap.add_argument('--probe', type=float, default=0.0, help='print the QP rows at this time relative to closest approach')
    ap.add_argument('--no-cbf', action='store_true', help='plant + task law only (validation of the model)')
    ap.add_argument('--json', default='')
    ap.add_argument('--label', default='')
    args = ap.parse_args()

    rec = Recording(args.bag, args.dist)
    T = np.load(args.truth)
    args.truth = (T['t'], T['ok'], T['p'])
    # q_home: the recorded posture when the trajectory started (first ee_desired sample)
    mov = np.nonzero(np.abs(rec.Vj).max(axis=1) > 0.05)[0]
    qh, _ = rec.q_at(rec.tj[mov[0]] - 0.3 if len(mov) else rec.tj[0])
    args.q_home = qh
    if args.check_nominal:
        check_nominal(rec, args)
        return
    ps = find_passes(rec, args.truth)
    over = yaml.safe_load(open(args.cbf_overrides)) if args.cbf_overrides else {}
    for kv in args.set:
        k, v = kv.split('=', 1)
        over[k] = yaml.safe_load(v)
    rows = []
    for n, p in enumerate(ps, 1):
        if args.only and n not in args.only:
            continue
        m = run_pass(rec, p, args, lambda: make_filter(over))
        m['pass'] = n; m['v'] = p['v']
        rows.append(m)
        print(f"pass {n} v={p['v']:.1f}  clr {m['clr_rec']*100:5.1f} -> {m['clr_sim']*100:5.1f} cm  "
              f"peak cmd {m['peak_cmd']:5.2f} acc {m['peak_acc']:5.2f}  jerk p99 cmd {m['jerk_cmd_p99']:6.0f} "
              f"real {m['jerk_real_p99']:6.0f}  flips {m['flips']:2d}  vratio {m['vratio_max']:.2f}  "
              f"ee_err max {m['ee_err_max']*100:4.1f} end {m['ee_err_end']*100:4.1f} cm  off-path sim {m['off_sim']*100:4.1f} rec {m['off_rec']*100:4.1f}", flush=True)
    if rows:
        g = lambda k: np.mean([r[k] for r in rows])
        print(f"{args.label or 'run'} MEAN  clr_gain {100*(g('clr_sim')-g('clr_rec')):+5.1f} cm  "
              f"min_clr {100*min(r['clr_sim'] for r in rows):5.1f}  peak_cmd {g('peak_cmd'):5.2f}  "
              f"jerk_cmd_p99 {g('jerk_cmd_p99'):6.0f}  jerk_real_p99 {g('jerk_real_p99'):6.0f}  "
              f"rev {g('flips'):4.1f} tv {g('tv'):4.0f} rec {np.nanmean([r['recover_s'] for r in rows]):4.2f}s  vratio {g('vratio_max'):.2f} (max {max(r['vratio_max'] for r in rows):.2f})  "
              f"ee_err_max {100*g('ee_err_max'):4.1f}")
    if args.json:
        json.dump(rows, open(args.json, 'w'), indent=1)


if __name__ == '__main__':
    main()
