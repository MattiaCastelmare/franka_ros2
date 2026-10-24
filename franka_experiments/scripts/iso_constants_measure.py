#!/usr/bin/env python3
"""Measure the ISO input constants the SSM layer consumes.

WHY THIS EXISTS
---------------
Every number in the ``iso_*`` block of ``config/fr3_control.yaml`` feeds the
separation-distance formula of ISO 10218-2:2025 Annex L. Guessing them makes
the whole chain decorative: ``S_p`` is linear in ``T_r`` and quadratic in
``1/a_s``, so a deceleration assumed twice as good as the realized one halves
the stopping term. Hardware diagnostics already show commanded deceleration far
above realized (``qdd_cmd_rad`` vs ``qdd_real_rad`` in the CBFDIAG line), which
is exactly the error this script exists to remove.

Nothing here publishes to the robot EXCEPT ``stop``, which moves the arm on
purpose and refuses to run without ``--i-am-supervising``.

SUB-COMMANDS
------------
``reaction``     [E]  T_r  — wraps scripts/latency_budget.py
``stop``         [S] procedure / [E] values  — a_s, T_s, S_s on the real arm
``detection``    [R] input for C — detection capability d of the depth pipeline
``uncertainty``  [E]  Z_d — depth/TF residual p95 against a static target
``yaml``              print the ready-to-paste block from a results file

NORMATIVE TAGS
--------------
``[R]`` required by the standard, ``[S]`` value/formula from the standard,
``[E]`` engineering assumption made here. Printed next to every number so a
reader cannot mistake one for another.

USAGE (inside the ROS container)
--------------------------------
    python3 scripts/iso_constants_measure.py reaction --bag rosbag/arm_complex
    python3 scripts/iso_constants_measure.py stop --i-am-supervising --payload-frac 1.0
    python3 scripts/iso_constants_measure.py detection --object-size-mm 30 --range-m 2.0
    python3 scripts/iso_constants_measure.py uncertainty --duration 20
    python3 scripts/iso_constants_measure.py yaml --results iso_constants.json

Every sub-command appends its result to ``--results`` (default
``iso_constants.json`` in the current directory) so ``yaml`` can assemble the
block from runs made on different days.
"""
from __future__ import annotations

import argparse
import datetime
import json
import os
import subprocess
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)

#: Tag of every constant this script produces, so the YAML writer cannot invent
#: one. Keep in step with the ``iso_*`` block of config/fr3_control.yaml.
TAGS = {
    'iso_t_reaction':  ('E', 'T_r, measured (this script, `reaction`)'),
    'iso_a_stop':      ('E', 'a_s, MEASURED realized Cartesian decel (`stop`)'),
    'iso_v_human':     ('S', 'ISO 13855:2024 approach speed'),
    'iso_c_intrusion': ('R', 'ISO 13855:2024 intrusion distance C from d (`detection`)'),
    'iso_z_depth':     ('E', 'Z_d, depth+calibration position uncertainty (`uncertainty`)'),
    'iso_z_robot':     ('E', 'Z_r, robot position accuracy (FR3 product manual)'),
}


# ═════════════════════════════════════════════════════════════════════════════
#  Results file
# ═════════════════════════════════════════════════════════════════════════════

def _load(path: str) -> dict:
    if not os.path.exists(path):
        return {}
    with open(path) as fh:
        return json.load(fh)


def _store(path: str, key: str, payload: dict) -> None:
    """Append one measurement, stamped, keeping any previous run under ``_history``."""
    doc = _load(path)
    payload = dict(payload)
    payload['measured'] = datetime.date.today().isoformat()
    hist = doc.setdefault('_history', {}).setdefault(key, [])
    if key in doc:
        hist.append(doc[key])
    doc[key] = payload
    with open(path, 'w') as fh:
        json.dump(doc, fh, indent=2, sort_keys=True)
    print(f'\n→ wrote {key} to {path}')


def _pct(x, q=95.0):
    x = np.asarray(x, dtype=float)
    return float(np.percentile(x, q)) if x.size else float('nan')


# ═════════════════════════════════════════════════════════════════════════════
#  reaction — T_r  [E]
# ═════════════════════════════════════════════════════════════════════════════

def cmd_reaction(args):
    """T_r = p95(total blind time) + 1/qp_rate_hz + 0.001  [E].

    The blind time comes from ``latency_budget.py``: the depth→CBF→torque chain
    it already measures, hop by hop, with the shipped code paths. The two extra
    terms are the QP period (a snapshot can be one tick old when the solver
    reads it) and 1 ms of actuation lag at the 1 kHz controller.
    """
    lb = os.path.join(HERE, 'latency_budget.py')
    blind_ms = args.blind_ms

    if blind_ms is None and args.bag:
        cmd = [sys.executable, lb, 'perception', '--bag', args.bag]
        print(f'== running: {" ".join(cmd)}')
        env = dict(os.environ, PYTHONPATH=PKG + os.pathsep + os.environ.get('PYTHONPATH', ''))
        out = subprocess.run(cmd, capture_output=True, text=True, env=env)
        sys.stdout.write(out.stdout)
        sys.stderr.write(out.stderr)
        for ln in out.stdout.splitlines():
            if 'total (depth in hand' in ln and 'p95=' in ln:
                blind_ms = float(ln.split('p95=')[1].split()[0])
                break
        if blind_ms is None:
            print('!! could not parse a p95 total from latency_budget.py — pass '
                  '--blind-ms explicitly (see its `bag`/`cam`/`dds` sub-commands '
                  'for the transport hops it does NOT fold into that total).')
            return 1

    if blind_ms is None:
        print('!! give either --bag (to run the perception replay) or --blind-ms '
              '(the p95 total blind time in ms you already measured).')
        return 1

    t_r = blind_ms * 1e-3 + 1.0 / args.qp_rate_hz + 0.001
    print('\n== reaction  [E] ==================================================')
    print(f'  p95 blind time (depth in hand → bytes out) : {blind_ms:8.2f} ms')
    print(f'  + QP period (1/{args.qp_rate_hz:g} Hz)     : '
          f'{1000.0 / args.qp_rate_hz:8.2f} ms')
    print(f'  + actuation lag at the 1 kHz controller    : {1.0:8.2f} ms')
    print(f'  ------------------------------------------------------')
    print(f'  iso_t_reaction = T_r                       : {t_r:8.4f} s   [E]')
    print('\n  NOTE: latency_budget.py `perception` measures COMPUTE only. The '
          'camera\n  exposure→stamp hop (`cam`), the DDS hops (`dds`) and the '
          'rate-induced\n  waits are separate; if your budget includes them, '
          'pass their sum as\n  --blind-ms instead of using --bag.')
    _store(args.results, 'iso_t_reaction',
           dict(value=round(t_r, 4), tag='E', blind_ms=blind_ms,
                qp_rate_hz=args.qp_rate_hz,
                source='latency_budget.py perception' if args.bag else 'operator-supplied'))
    return 0


# ═════════════════════════════════════════════════════════════════════════════
#  stop — a_s, T_s, S_s  [S] procedure, [E] values
# ═════════════════════════════════════════════════════════════════════════════

#: Grid of ISO 10218-1:2025 Annex H: 33 / 66 / 100 % of speed, of rated payload
#: and of arm extension. The annex's Category 2 stop is the profile commanded
#: here (the drives stay powered and the controller brakes), which is what the
#: acceleration-space stack can produce; Category 0/1 come from the product
#: manual and are cross-checked, not reproduced. [S]
ANNEX_H_FRACTIONS = (0.33, 0.66, 1.0)


def cmd_stop(args):
    """Drive the arm, command the stop profile, report the REALIZED decel.

    Per run: realized Cartesian TCP speed at trip, stopping time ``T_s``,
    stopping distance ``S_s``, realized Cartesian deceleration
    ``a_s = v_tcp / T_s``. The reported ``iso_a_stop`` is the MINIMUM ``a_s``
    over every run and direction — a maximum would make ``S_p`` optimistic by
    exactly the ratio between them.
    """
    if not args.i_am_supervising:
        print('REFUSED: `stop` MOVES THE ARM and then brakes it hard.\n'
              '  Clear the workspace, keep the enabling device in hand, and '
              're-run with --i-am-supervising.')
        return 2

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Float64MultiArray

    sys.path.insert(0, PKG)
    from franka_experiments.utils.cbf_state_rows import FR3_JOINT_KEYS, FR3_JOINTS, NV
    from franka_experiments.utils.config import load_franka_joint_limits
    from franka_experiments.utils.kinematics import CBFKinematics, build_urdf_no_hand
    import pinocchio as pin

    jl = load_franka_joint_limits(FR3_JOINT_KEYS)
    qdot_max, decel_max = jl['qdot_max'], jl['decel_max']

    class _Runner(Node):
        def __init__(self):
            super().__init__('iso_constants_stop')
            self.kin = CBFKinematics(pin.buildModelFromUrdf(build_urdf_no_hand()))
            self.tcp_fid = self.kin.resolve_frame_id(args.tcp_link)
            if self.tcp_fid is None:
                raise RuntimeError(f'TCP link {args.tcp_link!r} not in the model')
            self.q = self.qdot = None
            self.log: list = []
            self.recording = False
            self.create_subscription(
                JointState, args.joint_states, self._js, QoSProfile(depth=1))
            self.create_subscription(
                Float64MultiArray, args.qddot_safe, self._safe,
                QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
            self.pub = self.create_publisher(Float64MultiArray, args.qddot_nom, 10)
            self.last_safe = np.zeros(NV)

        def _js(self, msg):
            n2p = dict(zip(msg.name, msg.position))
            n2v = dict(zip(msg.name, msg.velocity))
            try:
                self.q = np.array([n2p[n] for n in FR3_JOINTS])
                self.qdot = np.array([n2v[n] for n in FR3_JOINTS])
            except KeyError:
                return
            if self.recording:
                self.log.append((self.now(), self.q.copy(), self.qdot.copy(),
                                 self.last_safe.copy()))

        def _safe(self, msg):
            d = np.asarray(msg.data, dtype=float)
            if d.shape == (NV,):
                self.last_safe = d

        def now(self):
            return self.get_clock().now().nanoseconds * 1e-9

        def send(self, qddot):
            m = Float64MultiArray()
            m.data = [float(v) for v in qddot]
            self.pub.publish(m)

        def tcp(self, q, qdot):
            self.kin.update(q, qdot, with_jdot=False)
            p = np.array(self.kin.data.oMf[self.tcp_fid].translation)
            Jp = self.kin.point_jacobian_pos(self.tcp_fid, p)
            return p, Jp @ qdot

    rclpy.init()
    node = _Runner()
    runs = []
    try:
        # Wait for state.
        t0 = node.now()
        while node.q is None and node.now() - t0 < 5.0:
            rclpy.spin_once(node, timeout_sec=0.05)
        if node.q is None:
            print(f'!! no {args.joint_states} in 5 s — is the robot up?')
            return 1

        for f_speed in ANNEX_H_FRACTIONS:
            for sign in (+1.0, -1.0):
                v_target = f_speed * args.velocity_box_margin * qdot_max
                print(f'\n== run: speed {f_speed:.0%}, direction {sign:+.0f}, '
                      f'payload {args.payload_frac:.0%}, extension '
                      f'{args.extension_frac:.0%}')
                # ── accelerate ──────────────────────────────────────────
                node.recording = False
                node.log.clear()
                t_acc = node.now()
                while node.now() - t_acc < args.accel_s:
                    err = sign * v_target - node.qdot
                    node.send(np.clip(err / 0.15, -decel_max, decel_max))
                    rclpy.spin_once(node, timeout_sec=0.005)
                # ── trip the stop profile ───────────────────────────────
                node.recording = True
                t_trip = node.now()
                q_trip, qdot_trip = node.q.copy(), node.qdot.copy()
                p_trip, v_trip = node.tcp(q_trip, qdot_trip)
                v_tcp = float(np.linalg.norm(v_trip))
                while node.now() - t_trip < args.stop_window_s:
                    node.send(np.clip(-node.qdot / args.stop_tau,
                                      -decel_max, decel_max))
                    rclpy.spin_once(node, timeout_sec=0.005)
                node.send(np.zeros(NV))
                node.recording = False

                # ── reduce ──────────────────────────────────────────────
                t_s = float('nan')
                p_end = p_trip
                for (t, q, qd, _sf) in node.log:
                    p, v = node.tcp(q, qd)
                    p_end = p
                    if float(np.linalg.norm(v)) <= args.v_rest:
                        t_s = t - t_trip
                        break
                s_s = float(np.linalg.norm(p_end - p_trip))
                a_s = v_tcp / t_s if (t_s == t_s and t_s > 1e-6) else float('nan')
                print(f'   v_tcp at trip = {v_tcp:.3f} m/s   T_s = {t_s:.3f} s   '
                      f'S_s = {s_s:.3f} m   a_s = {a_s:.3f} m/s^2')
                runs.append(dict(speed_frac=f_speed, direction=sign,
                                 payload_frac=args.payload_frac,
                                 extension_frac=args.extension_frac,
                                 v_tcp=v_tcp, t_s=t_s, s_s=s_s, a_s=a_s))
    finally:
        try:
            node.send(np.zeros(NV))
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()

    good = [r['a_s'] for r in runs if r['a_s'] == r['a_s']]
    if not good:
        print('!! no run reached rest inside --stop-window-s; nothing to report.')
        return 1
    a_min = min(good)
    print('\n== stop  [S] procedure, [E] values ================================')
    print(f'  runs                                   : {len(runs)}')
    print(f'  realized Cartesian decel, MINIMUM      : {a_min:8.3f} m/s^2')
    print(f'  realized Cartesian decel, max          : {max(good):8.3f} m/s^2')
    print(f'  longest stopping distance S_s          : '
          f'{max(r["s_s"] for r in runs):8.3f} m')
    print(f'  longest stopping time T_s              : '
          f'{max(r["t_s"] for r in runs if r["t_s"] == r["t_s"]):8.3f} s')
    print('\n  iso_a_stop = the MINIMUM above, not the mean and not the max.')
    print('  CROSS-CHECK against the Franka Research 3 product manual\'s own\n'
          '  stopping data (measured to EN ISO 10218-1:2011 Annex B, renumbered\n'
          '  Annex H in the 2025 edition) and KEEP THE WORSE VALUE. The profile\n'
          '  commanded here is a Category 2 stop: drives powered, controller\n'
          '  braking. Categories 0 and 1 are firmware/brake events and are NOT\n'
          '  reproduced by this script.')
    print('  This grid is finite: 3 speeds x 2 directions at ONE payload and\n'
          '  ONE extension per invocation. Re-run at 33/66/100 % of rated\n'
          '  payload and of arm extension and keep the global minimum.')
    _store(args.results, 'iso_a_stop',
           dict(value=round(a_min, 3), tag='E', runs=runs,
                note='minimum realized Cartesian deceleration over the runs above'))
    return 0


# ═════════════════════════════════════════════════════════════════════════════
#  detection — d, and the C it implies  [R]
# ═════════════════════════════════════════════════════════════════════════════

def c_from_detection_capability(d_mm: float) -> float:
    """[m] ISO 13855:2024 intrusion distance C from detection capability d. [S]

        d <= 40 mm  ->  C = 8*(d - 14) mm      (finger/hand detection)
        d >  40 mm  ->  C = 850 mm             (body detection only)

    ``d`` is the smallest object the protective device reliably detects. It is
    NOT a free parameter and it is NOT the depth sensor's pixel pitch: it has to
    be DEMONSTRATED at the far end of the working range, over the whole field of
    view, on the worst-case surface and reflectivity in the cell.
    """
    d_mm = float(d_mm)
    if d_mm <= 40.0:
        return max(8.0 * (d_mm - 14.0), 0.0) * 1e-3
    return 0.850


def cmd_detection(args):
    """Record the detection-capability test and derive C. [R] input for C."""
    print('== detection  [R] input for C ====================================')
    print('  ISO 13855:2024 requires the intrusion distance C to be derived')
    print('  from the DETECTION CAPABILITY d of the protective device — the')
    print('  smallest object it reliably detects, demonstrated:')
    print('    * at the far end of the working range,')
    print('    * over the WHOLE field of view (corners included),')
    print('    * on the worst-case surface and reflectivity present in the cell')
    print('      (dark cloth, skin, a black sleeve — a depth camera loses all')
    print('      three long before it loses a white test card).')
    print()

    if args.object_size_mm is None:
        print('  This sub-command RECORDS a test you run; it cannot perform it.')
        print('  Procedure:')
        print('    1. Park the arm. Start real_time_distance with the cell\'s')
        print('       real lighting and the real camera pose.')
        print('    2. Move a test object of known size through the protected')
        print('       space at --range-m, covering the field of view on a grid.')
        print('    3. A PASS is: the object produces a LinkDistance entry with')
        print('       valid=True on every sample, at every position, for the')
        print('       whole traverse. One dropout is a FAIL for that size.')
        print('    4. Repeat, shrinking the object, until it fails. d is the')
        print('       smallest size that PASSED.')
        print('    5. Re-run this command with --object-size-mm <d> --range-m R')
        print('       --detections N --samples N --surface "<what you used>".')
        print()
        print('  Until d is demonstrated, the conformant value is the body-')
        print('  detection one: C = 0.85 m. The depth pipeline is also NOT a')
        print('  rated protective device (no IEC/TS 61496-4-3 assessment), so d')
        print('  alone would not make it one.')
        return 0

    d_mm = float(args.object_size_mm)
    rate = (args.detections / args.samples) if args.samples else 0.0
    c = c_from_detection_capability(d_mm)
    reliable = args.samples > 0 and args.detections == args.samples
    print(f'  test object            : {d_mm:.1f} mm')
    print(f'  range                  : {args.range_m:.2f} m')
    print(f'  surface / reflectivity : {args.surface}')
    print(f'  detections / samples   : {args.detections} / {args.samples} '
          f'({rate:.1%})')
    print(f'  RELIABLE (100 %)       : {"yes" if reliable else "NO"}')
    print(f'  implied C              : {c:.3f} m   [R] ISO 13855:2024')
    if not reliable:
        print('\n  !! NOT a detection capability: anything below 100 % is a FAIL')
        print('     for this size. C stays at the body-detection value 0.850 m.')
        c = 0.850
    if c > 0.0:
        print(f'\n  Sanity: the protective separation distance grows by exactly')
        print(f'  this C. At C = {c:.3f} m, check S_p against the FR3\'s reach')
        print(f'  (855 mm) before assuming conformant SSM is achievable here.')
    _store(args.results, 'iso_c_intrusion',
           dict(value=round(c, 3), tag='R', detection_capability_mm=d_mm,
                range_m=args.range_m, surface=args.surface,
                detections=args.detections, samples=args.samples,
                reliable=reliable,
                note='ISO 13855:2024; C = 8*(d-14) mm for d <= 40 mm, else 850 mm'))
    return 0


# ═════════════════════════════════════════════════════════════════════════════
#  uncertainty — Z_d  [E]
# ═════════════════════════════════════════════════════════════════════════════

def cmd_uncertainty(args):
    """p95 of the depth/TF residual against the projected robot model → Z_d. [E]

    Reuses ``utils.calibration_check.calibration_residual``: the same comparison
    ``real_time_distance`` already runs once a second, between the robot mesh
    projected through the extrinsics and the measured depth. Its residual is the
    combined depth + calibration + TF position error, which is exactly ``Z_d``.
    """
    import rclpy
    import trimesh
    import yaml
    from ament_index_python.packages import get_package_share_directory
    from cv_bridge import CvBridge
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import CameraInfo, Image
    from tf2_ros import Buffer, TransformListener

    sys.path.insert(0, PKG)
    from franka_experiments.utils.calibration_check import calibration_residual
    from franka_experiments.utils.config import load_extrinsics
    from franka_experiments.utils.tf_manager import TFManager

    with open(args.robot_config) as fh:
        cfg = yaml.safe_load(fh)
    robot_cfg, dist_cfg, mesh_cfg = cfg['robot'], cfg['distance'], cfg['meshes']
    R_base, t_base = load_extrinsics(args.extrinsics)

    mesh_dir = get_package_share_directory(mesh_cfg.get('package', 'franka_description'))
    samples = {k: trimesh.load(os.path.join(mesh_dir, v), force='mesh').sample(
        int(mesh_cfg.get('sample_points_per_link', 300)))
        for k, v in mesh_cfg['files'].items()}

    class _Probe(Node):
        def __init__(self):
            super().__init__('iso_constants_uncertainty')
            self.bridge = CvBridge()
            self.K = None
            self.res: list = []
            self.buf = Buffer()
            self.lis = TransformListener(self.buf, self)
            self.tf = TFManager(tf_buffer=self.buf,
                                base_frame=robot_cfg['base_frame'],
                                critical_links=robot_cfg.get(
                                    'critical_links', [robot_cfg.get('ee_link', 'fr3_link8')]),
                                cache_max_age_s=0.5, logger=self.get_logger())
            qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
            self.create_subscription(CameraInfo, args.info_topic, self._info, qos)
            self.create_subscription(Image, args.depth_topic, self._depth, qos)

        def _info(self, msg):
            self.K = np.asarray(msg.k, dtype=float).reshape(3, 3)

        def _depth(self, msg):
            if self.K is None:
                return
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            depth_m = depth.astype(np.float32) * float(args.depth_scale)
            tfs = self.tf.lookup_all(robot_cfg['segment_links'], msg.header.stamp)
            if not tfs:
                return
            pts = {}
            for link, pts_link in samples.items():
                T = tfs.get(link)
                if T is None:
                    continue
                pts[link] = (np.asarray(pts_link) @ np.asarray(T[:3, :3]).T
                             + np.asarray(T[:3, 3]))
            if not pts:
                return
            r = calibration_residual(
                pts, R_base, t_base, self.K, depth_m,
                min_depth=float(dist_cfg['min_depth_m']),
                max_depth=float(dist_cfg['max_depth_m']))
            v = getattr(r, 'median_abs', None)
            if v is None:
                v = getattr(r, 'median', None)
            if v is not None and np.isfinite(v):
                self.res.append(abs(float(v)))

    rclpy.init()
    node = _Probe()
    t0 = node.get_clock().now().nanoseconds * 1e-9
    try:
        while (node.get_clock().now().nanoseconds * 1e-9 - t0) < args.duration:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    if not node.res:
        print('!! no residual samples — check --depth-topic / --info-topic and TF.')
        return 1
    z_d = _pct(node.res, 95.0)
    print('\n== uncertainty  [E] ==============================================')
    print(f'  frames                     : {len(node.res)}')
    print(f'  residual median            : {np.median(node.res):.4f} m')
    print(f'  residual p95  -> iso_z_depth: {z_d:.4f} m   [E]')
    print('\n  This is the STATIC residual: the arm must be parked and the')
    print('  target rigid. A moving arm folds TF timing error into it, which')
    print('  belongs in the latency budget, not in Z_d.')
    _store(args.results, 'iso_z_depth',
           dict(value=round(z_d, 4), tag='E', frames=len(node.res),
                median=round(float(np.median(node.res)), 4),
                depth_topic=args.depth_topic))
    return 0


# ═════════════════════════════════════════════════════════════════════════════
#  yaml — the ready-to-paste block
# ═════════════════════════════════════════════════════════════════════════════

#: Defaults used for any constant that has not been measured yet, so the block
#: is always complete and always says which entries are still placeholders.
PLACEHOLDERS = {
    'iso_t_reaction':  0.10,
    'iso_a_stop':      1.0,
    'iso_v_human':     2.0,
    'iso_c_intrusion': 0.85,
    'iso_z_depth':     0.06,
    'iso_z_robot':     0.01,
}


def cmd_yaml(args):
    doc = _load(args.results)
    print('  # ── ISO 10218-1/-2:2025 measured inputs '
          '────────────────────────────')
    print('  # Generated by scripts/iso_constants_measure.py — paste into the')
    print('  # iso_* block of config/fr3_control.yaml, replacing those entries.')
    missing = []
    for key, default in PLACEHOLDERS.items():
        tag, what = TAGS[key]
        entry = doc.get(key)
        if entry is None and tag == 'S':
            # A [S] value comes FROM the standard; there is nothing to measure
            # and nothing to flag. Lowering it is what would need evidence.
            print(f'  {key}: {default}'.ljust(34) + f'# [{tag}] {what}')
        elif entry is None:
            missing.append(key)
            print(f'  {key}: {default}'.ljust(34)
                  + f'# [{tag}] {what} — PLACEHOLDER, NOT MEASURED')
        else:
            print(f'  {key}: {entry["value"]}'.ljust(34)
                  + f'# [{tag}] {what} — measured {entry["measured"]}')
    if 'iso_z_robot' in missing:
        print()
        print('  # iso_z_robot [E]: take the "worst case safe Cartesian position')
        print('  #   accuracy for stopping functions" from the Franka Research 3')
        print('  #   product manual and record the manual revision here. It is a')
        print('  #   datasheet figure, not something this script can measure.')
    if missing:
        print()
        print(f'  # !! {len(missing)} value(s) still at their placeholder: '
              f'{", ".join(missing)}')
        print('  # !! scripts/iso_preflight_check.py FAILS if iso_enabled is true')
        print('  #    while any of them is unchanged. That is intended.')
    return 0


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--results', default='iso_constants.json',
                    help='JSON file every sub-command appends its result to')
    sub = ap.add_subparsers(dest='cmd', required=True)

    p = sub.add_parser('reaction', help='T_r [E]')
    p.add_argument('--bag', default='', help='rosbag for latency_budget.py perception')
    p.add_argument('--blind-ms', type=float, default=None,
                   help='p95 total blind time [ms], if already measured')
    p.add_argument('--qp-rate-hz', type=float, default=100.0)
    p.set_defaults(fn=cmd_reaction)

    p = sub.add_parser('stop', help='a_s / T_s / S_s [S] procedure, [E] values')
    p.add_argument('--i-am-supervising', action='store_true',
                   help='REQUIRED: this sub-command moves the arm')
    p.add_argument('--payload-frac', type=float, default=0.0,
                   help='fraction of rated payload mounted for this grid')
    p.add_argument('--extension-frac', type=float, default=1.0,
                   help='fraction of arm extension for this grid')
    p.add_argument('--velocity-box-margin', type=float, default=0.6)
    p.add_argument('--accel-s', type=float, default=1.0)
    p.add_argument('--stop-window-s', type=float, default=1.5)
    p.add_argument('--stop-tau', type=float, default=0.05,
                   help='braking time constant of the stop profile (iso_stop_tau)')
    p.add_argument('--v-rest', type=float, default=0.01, help='[m/s] "stopped"')
    p.add_argument('--tcp-link', default='fr3_link8')
    p.add_argument('--joint-states', default='/NS_1/joint_states')
    p.add_argument('--qddot-nom', default='/NS_1/qddot_nom')
    p.add_argument('--qddot-safe', default='/NS_1/qddot_safe')
    p.set_defaults(fn=cmd_stop)

    p = sub.add_parser('detection', help='detection capability d -> C [R]')
    p.add_argument('--object-size-mm', type=float, default=None)
    p.add_argument('--range-m', type=float, default=2.0)
    p.add_argument('--detections', type=int, default=0)
    p.add_argument('--samples', type=int, default=0)
    p.add_argument('--surface', default='(not recorded)')
    p.set_defaults(fn=cmd_detection)

    p = sub.add_parser('uncertainty', help='Z_d [E]')
    p.add_argument('--duration', type=float, default=20.0)
    p.add_argument('--robot-config', default=os.path.join(PKG, 'config', 'fr3_complete.yaml'))
    p.add_argument('--extrinsics', default=os.path.join(PKG, 'config', 'camera_extrinsics.yaml'))
    p.add_argument('--depth-topic', default='/cams/d455/aligned_depth_to_color/image_raw')
    p.add_argument('--info-topic', default='/cams/d455/aligned_depth_to_color/camera_info')
    p.add_argument('--depth-scale', type=float, default=1e-3)
    p.set_defaults(fn=cmd_uncertainty)

    p = sub.add_parser('yaml', help='print the ready-to-paste YAML block')
    p.set_defaults(fn=cmd_yaml)

    args = ap.parse_args()
    return args.fn(args) or 0


if __name__ == '__main__':
    sys.exit(main())
