#!/usr/bin/env python3
"""Recorded people against the MPC, closed loop and paired: CV forecast vs frozen arm,
with and without the CBF shield downstream.

Every configuration sees the same person, the same task and the same perception output,
so the differences between them are the configuration's alone (cbf_human_replay.py does
the same for the CBF alone, and this script reuses its recording and perception).

INPUT
-----
A replay .npz of ``human_kf_consistency.py replay``: the HumanArmStates the real tracker
published on a lab bag, the measured keypoints and the robot joints of the recording.

METHOD
------
task     The TCP path of the RECORDED arm (forward kinematics of its joints), published
         to the MPC as its task preview: the motion the person reacted to.
mpc      The real mpc_commander node (its control_loop, mpc.yaml), on an isolated ROS
         domain with every publisher replaced by a sink and its clock driven from here.
         It receives the joint state every cycle and each HumanArmState T_CAM after its
         capture stamp, as on the robot.
arm      Double integrator at the CBF's qp_rate_hz; the MPC's q̈ is held between its
         cycles (dt of mpc.yaml). --plant-delay-s D: each q̈ reaches the arm D after its
         cycle (the loop's own time on the robot), the previous one acting until then.
shield   +cbf: the CBF of cbf_human_replay (shipped ConstraintBuilder and QP,
         fr3_control.yaml, prediction rows off) filters the held q̈, and its output is
         handed back to the MPC as the q̈ in force (its qddot_safe subscription). +shield: the same
         with config/cbf_shield.yaml on top, as easy_torque runs it behind the MPC. Rows are rebuilt at cbf_update_rate_hz, robot-only rows
         included when nobody is near (perception keeps publishing empty frames).
truth    The measured keypoints (median of 3, no filter): every configuration is scored
         against them.

Approach events are the intervals where the measured person comes within EVENT_GAP of the
RECORDED arm: they do not depend on the configuration, so the closest point of each event
and the distance at which the MPC first reacted are compared event by event (paired).

With a shield, every tick where it changes the MPC's q̈ by more than SHIELD_ACTIVE is
attributed to what asked for it: a family of rows the MPC's q̈ violates (obs = the human
barrier, cap = retreat caps, spd = link speed caps, sc, qlim, sing), the slew limit
(max_qddot_delta from the previous output), the other limits of the box (q̈, q̇, q), or
only the objective biases (tangential, evasion).

The person is open loop: in the lab they reacted to the recorded arm, not to this one.

USAGE (inside the ROS container, workspace sourced, this checkout on PYTHONPATH)
--------------------------------------------------------------------------------
    python3 scripts/human_kf_consistency.py replay /bags/cbf_final --out cbf_final.npz
    python3 scripts/mpc_human_replay.py cbf_final.npz \\
        --configs static prediction static+cbf prediction+cbf [--shift-m 0.15]

Configs: <static|prediction>[+cbf|+shield][[key=value,...]], the brackets overriding mpc.yaml
keys of that configuration only, e.g. prediction+shield[input_delay_s=0.018,slack_linear=1e4].
--shift-m D moves the person D metres toward the robot (a stress test: the lab people mostly
kept clear of the arm). --cbf-set '{"key": v}' overrides fr3_control.yaml keys of the shield
(to try a looser one).
"""
from __future__ import annotations

import argparse
import json
import os
import sys

import yaml

# Isolated graph: the replayed MPC must never reach a robot's topics
os.environ['ROS_DOMAIN_ID'] = os.environ.get('REPLAY_DOMAIN_ID', '88')
os.environ['ROS_LOCALHOST_ONLY'] = '1'

import numpy as np  # noqa: E402
import rclpy  # noqa: E402
from rclpy.clock import ClockType  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from rclpy.time import Time  # noqa: E402
from scipy.spatial.transform import Rotation, Slerp  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402
from std_msgs.msg import Float64MultiArray  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import cbf_scenarios as cs  # noqa: E402
from cbf_human_replay import T_CAM, HumanDistance, Recording, events  # noqa: E402
from cbf_human_scenarios import easy_torque_params  # noqa: E402
from human_kf_consistency import _Sink  # noqa: E402
from franka_msgs.msg import HumanArmState  # noqa: E402
from franka_experiments.nodes.mpc_commander import MPCCommander  # noqa: E402
from franka_experiments.utils.cbf_qp_assembly import build_row_rhs  # noqa: E402
from franka_experiments.utils.cbf_state_rows import (  # noqa: E402
    GROUP_NAMES, NV, JointSnap, ObstacleSnap)
from franka_experiments.utils.constants import FR3_JOINT_NAMES  # noqa: E402
from franka_experiments.utils.human_utils import make_ee_trajectory_msg, fill_ee_trajectory_msg  # noqa: E402

EVENT_GAP = 0.40       # [m] an approach: the person within this of the recorded arm (MPC d_activate)
REACTION_WINDOW_S = 1.0  # a binding MPC row up to this long before an event counts as its reaction
SHIELD_ACTIVE = 0.5    # [rad/s²] ‖q̈_safe − q̈_mpc‖ above this = the shield changed the command
ROW_TOL = 1e-3         # [m/s²] the MPC's q̈ violates a CBF row by more than this
PREVIEW_STEP, PREVIEW_POINTS = 0.02, 40   # task preview handed to the MPC (0.8 s)
CAUSES = list(GROUP_NAMES) + ['limits', 'slew', 'bias']


class ReplayClock:
    """Stands in for the node's clock: the replay sets the time."""

    def __init__(self):
        self.t = 0.0

    def now(self):
        return Time(nanoseconds=int(round(self.t * 1e9)), clock_type=ClockType.ROS_TIME)


def stamp(t):
    return Time(nanoseconds=int(round(t * 1e9)), clock_type=ClockType.ROS_TIME).to_msg()


def parse_config(config):
    """'prediction+shield[input_delay_s=0.018]' -> ('prediction', 'shield', {'input_delay_s': 0.018}).

    The values take the type of the mpc.yaml key they override.
    """
    name, _, brackets = config.partition('[')
    human_mode, _, shield = name.partition('+')
    if human_mode not in ('static', 'prediction') or shield not in ('', 'cbf', 'shield'):
        raise SystemExit(f'unknown config {config}')
    with open(os.path.join(cs.PKG, 'config', 'mpc.yaml')) as f:
        defaults = {k: v for section in yaml.safe_load(f).values() for k, v in section.items()}
    mpc_set = {}
    for item in filter(None, brackets.rstrip(']').split(',')):
        key, _, value = (part.strip() for part in item.partition('='))
        if key not in defaults:
            raise SystemExit(f'{config}: {key} is not a key of mpc.yaml')
        default = defaults[key]
        mpc_set[key] = type(default)(value) if isinstance(default, (int, float)) and not isinstance(default, bool) else value
    return human_mode, shield, mpc_set


def make_mpc(human_mode, mpc_set):
    """A real MPCCommander (mpc.yaml with mpc_set on top), sinks for publishers and a replay clock."""
    node = MPCCommander(parameter_overrides=[Parameter(k, value=v) for k, v in mpc_set.items()])
    node.get_logger().set_level(rclpy.logging.LoggingSeverity.ERROR)
    node.destroy_timer(node.timer)   # control_loop is called from here
    node.pub = node.sp_pub = node.plan_pub = node.diag_pub = _Sink()
    clock = ReplayClock()
    node.get_clock = lambda: clock
    node.human_mode = human_mode
    return node, clock


def arm_state_msg(pub, frame, t_cap):
    """The HumanArmState the tracker published at this frame (cbf_human_replay's arrays)."""
    msg = HumanArmState()
    msg.header.stamp = stamp(t_cap)
    msg.header.frame_id = 'fr3_link0'
    valid = pub['valid'][frame]
    pos = np.nan_to_num(pub['pos'][frame])
    vel = np.nan_to_num(pub['vel'][frame])
    for i, name in enumerate(('shoulder', 'elbow', 'wrist', 'hand')):
        point, velocity = getattr(msg, name), getattr(msg, f'{name}_vel')
        point.x, point.y, point.z = (float(c) for c in pos[i])
        velocity.x, velocity.y, velocity.z = (float(c) for c in vel[i])
    cov = np.nan_to_num(pub['cov'][frame])   # P_pp, P_vv, P_pv
    msg.position_covariance = cov[0].ravel().tolist()
    msg.velocity_covariance = cov[1].ravel().tolist()
    msg.position_velocity_covariance = cov[2].ravel().tolist()
    msg.keypoint_valid = [bool(v) for v in valid]
    msg.measured = [bool(v) for v in valid]
    msg.frames_seen = [int(s) for s in pub['seen'][frame]]
    msg.confidence = float(pub['conf'][frame])
    msg.valid = bool(np.any(valid))
    return msg


def task_path(node, rec, t0, t1):
    """TCP path of the recorded joints on a 100 Hz grid: (times, positions, Slerp)."""
    times = np.arange(t0, t1 + 1.0, 0.01)
    q = np.stack([np.interp(times, rec.jt, rec.jq[:, j]) for j in range(NV)], axis=1)
    positions, rotations = [], []
    for q_k in q:
        positions.append(node.tcp_position(q_k))
        rotations.append(node.kin.data.oMf[node.ee_frame_id].rotation.copy())
    return times, np.array(positions), Slerp(times, Rotation.from_matrix(np.array(rotations)))


def run(rec, config, *, t_start=0.0, duration=0.0, cbf_set=None, plant_delay=0.0):
    human_mode, shield, mpc_set = parse_config(config)
    P = easy_torque_params('off')
    overrides = {}
    if shield == 'shield':
        with open(os.path.join(cs.PKG, 'config', 'cbf_shield.yaml')) as f:
            overrides = yaml.safe_load(f)['cbf_safety_filter']['ros__parameters']
    for key, value in {**overrides, **(cbf_set or {})}.items():
        if not hasattr(P, key):
            raise SystemExit(f'--cbf-set: {key} is not a key of fr3_control.yaml')
        setattr(P, key, value)
    F = cs.Filter(P)                       # kinematics for the scoring, and the shield
    hd = HumanDistance(F, rec.sides)
    node, clock = make_mpc(human_mode, mpc_set)
    dt = F.dt
    every_rows = max(1, int(round(P.qp_rate_hz / P.cbf_update_rate_hz)))   # also the scoring rate
    every_mpc = max(1, int(round(node.dt / dt)))
    t0 = rec.t0 + t_start
    t1 = rec.t1 if not duration else min(rec.t1, t0 + duration)
    ts = t0 + np.arange(int((t1 - t0) / dt)) * dt
    q_ref, dq_ref, _ = rec.reference(ts)
    tb = ts[::every_rows]
    truth = rec.truth(tb)
    path_t, path_p, path_R = task_path(node, rec, t0, t1)
    preview = make_ee_trajectory_msg(node.ee_frame, PREVIEW_POINTS, PREVIEW_STEP)
    js_msg = JointState(name=list(FR3_JOINT_NAMES))

    q, qdot = q_ref[0].copy(), dq_ref[0].copy()
    u = qddot = np.zeros(NV)
    pending = []                           # (time it reaches the arm, q̈) of the MPC's outputs
    safe_msg = Float64MultiArray()
    frame, latest, latest_cap, latest_conf = 0, {s: None for s in rec.sides}, None, {}
    con = None
    out = dict(gap=[], gap_ref=[], dev=[], mpc_t=[], binding=[], slack=[], change=[], causes=[])
    for k, t in enumerate(ts):
        clock.t = t
        # ── perception: each published state delivered T_CAM after its capture
        while frame < len(rec.t) and rec.t[frame] + T_CAM <= t:
            for s in rec.sides:
                p = rec.pub[s]
                if p['pub'][frame]:
                    latest[s] = (p['pos'][frame], p['vel'][frame], p['valid'][frame],
                                 p['cov'][frame], p['seen'][frame])
                    latest_conf[s] = p['conf'][frame]
                    latest_cap = rec.t[frame]
                    node.latest_arm_states[s] = (arm_state_msg(p, frame, rec.t[frame]), clock.now())
            frame += 1

        # ── MPC cycle: joint state and task preview, then the node's own control_loop
        if k % every_mpc == 0:
            js_msg.position, js_msg.velocity = q.tolist(), qdot.tolist()
            node.latest_joint_state = (js_msg, clock.now())
            t_prev = t + PREVIEW_STEP * np.arange(PREVIEW_POINTS)
            fill_ee_trajectory_msg(
                preview, stamp(t),
                np.column_stack([np.interp(t_prev, path_t, path_p[:, a]) for a in range(3)]),
                orientations=path_R(np.clip(t_prev, path_t[0], path_t[-1])).as_quat())
            node.latest_task_trajectory = (preview, clock.now())
            node.control_loop()
            pending.append((t + plant_delay, np.array(node.qddot_msg.data)))
            out['mpc_t'].append(len(out['gap']))   # scoring index of this cycle
            out['binding'].append(node.problem.diag.human_binding)
            out['slack'].append(node.problem.diag.max_slack)

        # ── shield rows (50 Hz) and scoring, both at the current q
        if k % every_rows == 0:
            j = k // every_rows
            F.kin.update(q_ref[k], dq_ref[k], with_jdot=True)
            true_caps = hd.truth_capsules(truth, j)
            out['gap_ref'].append(hd.gap(hd.control_points(), true_caps))
            F.kin.update(q, qdot, with_jdot=True)
            cps = hd.control_points()
            out['gap'].append(hd.gap(cps, true_caps))
            out['dev'].append(float(np.linalg.norm(
                node.tcp_position(q) - [np.interp(t, path_t, path_p[:, a]) for a in range(3)])))
            if shield:
                caps = hd.capsules(latest) if latest_cap is not None else []
                obs = ()
                if caps:
                    conf = np.mean([latest_conf[s] for s in rec.sides if latest[s] is not None])
                    obs = hd.obstacles(hd.message(cps, caps, conf))
                con = F.builder.build(JointSnap(q=q, qdot=qdot, stamp=t),
                                      ObstacleSnap(items=obs, stamp=t, t_cap=latest_cap or t), t)

        # ── the arm: the MPC's q̈ once it arrives, through the shield with +cbf
        while pending and pending[0][0] <= t + 1e-9:
            u = pending.pop(0)[1]
        qddot_prev, qddot = qddot, u
        if shield:
            qddot, _, nom, _, _ = F.tick(t, con, q, qdot, u)
            change = float(np.linalg.norm(qddot - u))
            causes = np.zeros(len(CAUSES), dtype=bool)
            if change > SHIELD_ACTIVE:
                if con is not None:
                    h_qp, _ = build_row_rhs(con, qdot, qdot, k0=P.k0_cbf, k1=P.k1_cbf,
                                            retreat_horizon=P.retreat_cap_horizon_s,
                                            speed_horizon=P.link_speed_horizon_s)
                    violated = con.A @ u + h_qp < -ROW_TOL
                    for g in np.unique(con.group[violated]):
                        causes[int(g)] = True
                # The box is the slew window intersected with the limits: outside the window
                # it is the slew, inside it but outside the box it is the limits
                slew = P.slew_box_enabled and bool(np.any(np.abs(u - qddot_prev) > P.max_qddot_delta + 1e-6))
                outside = bool(np.any(u < F.box_lb[:NV] - 1e-6) or np.any(u > F.box_ub[:NV] + 1e-6))
                causes[-3], causes[-2] = outside and not slew, slew
                causes[-1] = not causes[:-1].any() and float(np.linalg.norm(nom - u)) > 1e-3
            out['change'].append(change)
            out['causes'].append(causes)
            if node.qddot_safe_topic:      # what the node's subscription would receive
                safe_msg.data = qddot.tolist()
                node.latest_qddot_safe = (safe_msg, clock.now())
        qdot = qdot + qddot * dt
        q = q + qdot * dt
    node.destroy_node()
    res = {key: np.asarray(v) for key, v in out.items()}
    res.update(dt_score=dt * every_rows, dt=dt, d_safe=P.d_safe, delta=node.settings.delta,
               config=config)
    return res


def score(r, ev):
    dt, d_safe, delta = r['dt_score'], r['d_safe'], r['delta']
    gap, fin = r['gap'], np.isfinite(r['gap'])
    pad = int(round(0.5 / dt))
    bind_idx = r['mpc_t'][r['binding'] > 0]
    ev_min, ev_react = [], []
    for a, b in ev:
        window = np.where(fin[a:b + pad], gap[a:b + pad], np.inf)
        i_min = a + int(np.argmin(window))
        ev_min.append(float(window.min()))
        first = bind_idx[(bind_idx >= a - REACTION_WINDOW_S / dt) & (bind_idx <= i_min)]
        ev_react.append(float(gap[first[0]]) if first.size and np.isfinite(gap[first[0]]) else np.nan)
    s = dict(
        min_gap=float(gap[fin].min()) if fin.any() else np.inf,
        inside_s=float(np.sum(fin & (gap < d_safe)) * dt),
        inside_delta_s=float(np.sum(fin & (gap < d_safe + delta)) * dt),
        dev_mean=float(np.mean(r['dev'])), dev_p95=float(np.percentile(r['dev'], 95)),
        slack_cycles=int(np.sum(r['slack'] > 1e-3)),
        ev_min=np.array(ev_min), ev_react=np.array(ev_react))
    if r['change'].size:
        # Ticks where the shield changed the MPC's q̈, near the person or not
        per_score = int(round(r['dt_score'] / r['dt']))
        near = np.repeat(fin & (gap < EVENT_GAP), per_score)[:r['change'].size]
        active = r['change'] > SHIELD_ACTIVE
        s['shield_s'] = float(active.sum() * r['dt'])
        s['shield_near_s'] = float((active & near).sum() * r['dt'])
        s['near_s'] = float(near.sum() * r['dt'])
        s['shield_causes'] = {c: float(r['causes'][active, i].mean()) if active.any() else 0.0
                              for i, c in enumerate(CAUSES)}
        s['shield_causes_near'] = {c: float(r['causes'][active & near, i].mean())
                                   if (active & near).any() else 0.0 for i, c in enumerate(CAUSES)}
    return s


def paired(scores, key, base, label, unit=100.0, suffix='cm'):
    for c, s in scores.items():
        if c == base:
            continue
        diff = s[key] - scores[base][key]
        diff = diff[np.isfinite(diff)]
        if diff.size:
            print(f'   {label}, {c} − {base}: {unit * np.mean(diff):+.1f} {suffix} on average '
                  f'(median {unit * np.median(diff):+.1f}, worst {unit * np.min(diff):+.1f}, best '
                  f'{unit * np.max(diff):+.1f}); larger in {int(np.sum(diff > 0.005))}, smaller in '
                  f'{int(np.sum(diff < -0.005))} of {diff.size}')


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('replays', nargs='+', help='human_kf_consistency.py replay .npz files')
    ap.add_argument('--configs', nargs='+', default=['static', 'prediction'])
    ap.add_argument('--start-s', type=float, default=0.0)
    ap.add_argument('--duration-s', type=float, default=0.0)
    ap.add_argument('--shift-m', type=float, default=0.0,
                    help='stress test: move the person this far toward the robot base')
    ap.add_argument('--cbf-set', default='', help='JSON of fr3_control.yaml keys for the shield')
    ap.add_argument('--plant-delay-s', type=float, default=0.0,
                    help="each MPC q̈ reaches the arm this long after its cycle (the loop's own time)")
    ap.add_argument('--json', default='', help='write every score here')
    args = ap.parse_args(argv)
    cbf_set = json.loads(args.cbf_set) if args.cbf_set else None
    if cbf_set:
        print(f'shield overrides: {cbf_set}')
    rclpy.init()
    dump = {}
    for path in args.replays:
        rec = Recording(path)
        print(f'\n== {rec.name}: {rec.t1 - rec.t0:.0f} s, sides {rec.sides}')
        if args.shift_m:
            rec.shift_toward_robot(args.shift_m)
            print(f'   person moved {100 * args.shift_m:.0f} cm toward the robot base')
        results = {c: run(rec, c, t_start=args.start_s, duration=args.duration_s, cbf_set=cbf_set,
                          plant_delay=args.plant_delay_s) for c in args.configs}
        first = results[args.configs[0]]
        ev = events(first['gap_ref'], first['dt_score'], EVENT_GAP)
        print(f'   {len(ev)} approach events (person within {EVENT_GAP:.2f} m of the recorded arm), '
              f'd_safe {first["d_safe"]:.2f} m, MPC target d_safe+δ {first["d_safe"] + first["delta"]:.2f} m')
        scores = {c: score(r, ev) for c, r in results.items()}
        w = max(18, *(len(c) for c in args.configs))
        print(f"   {'config':{w}s} {'min gap':>8s} {'<d_safe':>8s} {'<+δ':>7s} {'react d':>8s} "
              f"{'dev mean':>9s} {'dev p95':>8s} {'slack cyc':>9s} {'shield':>7s} {'near':>12s}")
        for c, s in scores.items():
            react = np.nanmedian(s['ev_react']) if np.isfinite(s['ev_react']).any() else np.nan
            shield = (f"{s['shield_s']:6.1f}s {s['shield_near_s']:5.1f}/{s['near_s']:4.1f}s"
                      if 'shield_s' in s else f"{'–':>7s} {'–':>12s}")
            print(f"   {c:{w}s} {100 * s['min_gap']:+7.1f}c {s['inside_s']:7.2f}s {s['inside_delta_s']:6.2f}s "
                  f"{100 * react:7.1f}c {100 * s['dev_mean']:7.1f}cm {100 * s['dev_p95']:6.1f}cm "
                  f"{s['slack_cycles']:9d} {shield}")
        paired(scores, 'ev_min', args.configs[0], 'closest point per event')
        paired(scores, 'ev_react', args.configs[0], 'distance at the first binding row')
        for c, s in scores.items():
            if 'shield_causes_near' in s:
                print(f'   {c}: shield active near the person, asked by: '
                      + ', '.join(f'{name} {100 * v:.0f}%' for name, v in s['shield_causes_near'].items() if v > 0.005))
                print(f'   {c}: shield active anywhere, asked by: '
                      + ', '.join(f'{name} {100 * v:.0f}%' for name, v in s['shield_causes'].items() if v > 0.005))
        dump[rec.name] = {c: {k: (v.tolist() if isinstance(v, np.ndarray) else v) for k, v in s.items()}
                          for c, s in scores.items()}
        dump[rec.name]['events'] = [[int(a), int(b)] for a, b in ev]
    if args.json:
        with open(args.json, 'w') as f:
            json.dump(dump, f, indent=1)
    rclpy.shutdown()

if __name__ == '__main__':
    main()