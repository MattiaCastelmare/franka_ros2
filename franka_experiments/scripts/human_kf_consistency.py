#!/usr/bin/env python3
"""Measure the human-arm Kalman filter against its own model, on recorded bags.

WHY THIS EXISTS
---------------
ArmKalmanFilter observes each keypoint through a converted measurement: MediaPipe gives
the pixel (u, v), the aligned depth gives d, and deproject_arm turns (u, v, d) into a point
of fr3_link0 before the filter sees it (H = [I 0]). The filter is exact only if R is the
covariance of that converted point. This script measures, on real data, the noise of
(u, v, d), how far the isotropic R = sigma_m^2 I is from it, and what the first-order
covariance of the deprojection

    R = R_cb J Sigma J^T R_cb^T,   J = d(X, Y, Z) / d(u, v, d),
    Sigma = diag(sigma_px^2, sigma_px^2, sigma_d(Z)^2)

changes in the filter.

METHOD
------
replay   Runs the real HumanTracker on a bag, frame by frame (MediaPipe included), and
         saves every measurement, every filter step, the HumanArmState it published and
         the robot joints of the recording to an .npz (cbf_human_replay.py reads it).
         ROS runs on an isolated domain, localhost only, and every publisher is
         replaced by a sink: nothing is published.
noise    Filter-independent noise of the measurement: robust std of the second and third
         differences of u, v [px] and d [m] along each keypoint track, per depth bin.
         A difference of order n cancels any motion polynomial of degree < n; for white
         noise Var(D2) = 6 s^2 and Var(D3) = 20 s^2. Low-passed noise (MediaPipe's
         landmark smoothing) shows mostly as a lower level of both (compare a replay with
         and without it); D3 / D2 drops only a few percent below 1.
compare  Replays the filter on the recorded measurements with each measurement model and
         reports its consistency (NIS, also along and across the camera ray), the
         whiteness of the innovations and the error of the constant-velocity prediction
         against later measurements.

USAGE (inside the ROS container, workspace sourced)
----------------------------------------------------
    python3 scripts/human_kf_consistency.py replay /bags/varied --out /tmp/varied.npz
    python3 scripts/human_kf_consistency.py replay /bags/varied --out /tmp/varied_raw.npz \\
        --raw-landmarks
    python3 scripts/human_kf_consistency.py noise /tmp/varied.npz /tmp/varied_raw.npz
    python3 scripts/human_kf_consistency.py compare /tmp/varied.npz \
        "/tmp/varied.npz@pinhole:px=4.5,d0=0.03,dq=0.0015,bo=1" \
        /tmp/varied_raw.npz "/tmp/varied_raw.npz@pinhole:px=4.5,d0=0.03,dq=0.0015,bo=1" \
        --reference /tmp/varied_raw.npz

A compare variant is run.npz[@model[:key=value,...]], see parse_variant. Without a model
it is the tracker's own filter, and the replay is checked to reproduce it exactly.

``replay`` needs ROS 2, MediaPipe and the bag; ``noise`` and ``compare`` only numpy and
this package.
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time
import traceback

import numpy as np
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)

KEYPOINTS = ("shoulder", "elbow", "wrist", "index")


def _human_config():
    with open(os.path.join(PKG, "config", "human_params.yaml")) as f:
        return yaml.safe_load(f)["human_tracker"]


# ═════════════════════════════════════════════════════════════════════════════
#  replay — the real HumanTracker on a bag, nothing published
# ═════════════════════════════════════════════════════════════════════════════

class _Sink:
    """Stands in for a publisher: keeps the last message, sends nothing."""

    def __init__(self):
        self.last = None

    def publish(self, msg):
        self.last = msg


def _empty_side():
    return dict(
        lm_uv=np.full((4, 2), np.nan),     # MediaPipe pixel, float (before rounding)
        vis=np.zeros(4),
        pix=np.full((4, 2), np.nan),       # pixel used by the deprojection (rounded)
        own_depth=np.zeros(4),             # own depth after the occlusion check, 0 = none
        depth=np.zeros(4),                 # depth used (own or borrowed)
        pos=np.full((4, 3), np.nan),       # deprojected point, before validation
        step=False,                        # the filter ran this frame (engaged)
        dt=np.nan,
        z=np.full((4, 3), np.nan),         # filter input: validated points
        cov=np.full((4, 3, 3), np.nan),    # filter input: measurement covariances
        x=np.full((4, 6), np.nan),         # filter state after the step
        measured=np.zeros(4, dtype=bool),
        nis=np.full(4, np.nan),
        gated=np.zeros(4, dtype=bool),
        # the HumanArmState the tracker published this frame (what human_distance reads)
        pub=False,
        pub_pos=np.full((4, 3), np.nan),
        pub_vel=np.full((4, 3), np.nan),
        pub_valid=np.zeros(4, dtype=bool),
        pub_cov=np.full((3, 4, 3, 3), np.nan),   # P_pp, P_vv, P_pv
        pub_seen=np.zeros(4, dtype=np.int64),
    )


def _record_published(rec, msg):
    """Copy a published HumanArmState into a frame record (None: nothing published)."""
    from franka_experiments.utils.human_utils import (
        extract_human_covariances, extract_human_keypoints)
    if msg is None:
        return
    pos, vel, valid = extract_human_keypoints(msg)
    P_pp, P_vv, P_pv, seen = extract_human_covariances(msg)
    rec.update(pub=True, pub_pos=pos, pub_vel=vel, pub_valid=valid,
               pub_cov=np.stack([P_pp, P_vv, P_pv]), pub_seen=seen)


def replay(args):
    # Isolated graph: a replay must never reach the robot's /human topics
    os.environ["ROS_DOMAIN_ID"] = str(args.domain_id)
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    import rclpy
    from rclpy.serialization import deserialize_message
    from rosbag2_py import ConverterOptions, SequentialReader, StorageFilter, StorageOptions
    from rosidl_runtime_py.utilities import get_message
    from franka_experiments.nodes import human_tracker as ht

    config = _human_config()
    overrides = json.loads(args.set) if args.set else {}
    if args.raw_landmarks:
        overrides["smooth_landmarks"] = False
    config.update(overrides)
    load_config = ht.load_robot_config

    def load(path):
        cfg = load_config(path)
        if os.path.basename(path) == "human_params.yaml":
            cfg["human_tracker"].update(overrides)
        return cfg

    # Config of this checkout, with the overrides
    ht.get_package_share_directory = lambda _name: PKG
    ht.load_robot_config = load

    rclpy.init()
    node = ht.HumanTracker()
    node.stop_worker()   # frames are fed below, in bag order
    node.get_logger().set_level(rclpy.logging.LoggingSeverity.WARN)
    for pubs in (node.state_pubs, node.raw_state_pubs, node.prediction_pubs,
                 node.landmarks_2d_pubs, node.kf_diag_pubs):
        for side in pubs:
            pubs[side] = _Sink()

    # Recording hooks: the node's own calls, observed
    sides = list(node.active_sides)
    frame = {}
    cursor = {"side": -1}
    extract = ht.extract_arm_landmarks

    def extract_hook(pose_landmarks, image_shape, side, names):
        landmarks = extract(pose_landmarks, image_shape, side, names)
        if landmarks is not None:
            frame[side]["lm_uv"] = np.array([[landmarks[n]["x_px"], landmarks[n]["y_px"]]
                                             for n in names])
            frame[side]["vis"] = np.array([landmarks[n]["visibility"] for n in names])
        return landmarks

    keypoint_depths = node.measurer.keypoint_depths

    def keypoint_depths_hook(*a, **kw):
        cursor["side"] += 1   # called once per side, in active_sides order
        return keypoint_depths(*a, **kw)

    deproject_arm = node.measurer.deproject_arm

    def deproject_hook(pixels, own_depths, camera_tf):
        out = deproject_arm(pixels, own_depths, camera_tf)
        rec = frame[sides[cursor["side"]]]   # the last call of a side is the one used
        rec["own_depth"] = np.array(own_depths, dtype=float)
        rec["pix"] = np.full((4, 2), np.nan)
        for i, (u, v) in pixels.items():
            rec["pix"][i] = (u, v)
        rec["pos"], rec["depth"] = np.array(out[0]), np.array(out[1])
        return out

    ht.extract_arm_landmarks = extract_hook
    node.measurer.keypoint_depths = keypoint_depths_hook
    node.measurer.deproject_arm = deproject_hook
    for side, kf in node.kfs.items():
        def step_hook(*a, _step=kf.step, _side=side, _kf=kf, **kw):
            out = _step(*a, **kw)
            rec = frame[_side]
            rec["step"], rec["dt"] = True, _kf.dt
            rec["z"] = np.array(kw["positions"], dtype=float)
            if kw.get("measurement_covariances") is not None:
                rec["cov"] = np.array(kw["measurement_covariances"], dtype=float)
            rec["x"] = np.where(_kf.initialized[:, None], _kf.x, np.nan)
            rec["measured"] = np.array(out[2], dtype=bool)
            rec["nis"], rec["gated"] = _kf.get_consistency()
            return out
        kf.step = step_hook

    # Bag
    reader = SequentialReader()
    reader.open(StorageOptions(uri=args.bag, storage_id="sqlite3"),
                ConverterOptions(input_serialization_format="cdr",
                                 output_serialization_format="cdr"))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    color_topic, depth_topic = config["color_topic"], config["depth_topic"]
    info_topic = config["camera_info_topic"]
    # The robot's joints too (1 kHz feed if recorded): the arm the person moved around
    joint_topic = next((t for t in ("/NS_1/franka/joint_states", "/NS_1/joint_states")
                        if t in types), None)
    wanted = [t for t in (color_topic, depth_topic, info_topic, "/tf", "/tf_static",
                          joint_topic) if t in types]
    if color_topic not in types or depth_topic not in types:
        raise SystemExit(f"{args.bag}: no {color_topic} / {depth_topic}")
    reader.set_filter(StorageFilter(topics=wanted))
    msg_types = {t: get_message(types[t]) for t in wanted}
    joint_names = [f"fr3_joint{i}" for i in range(1, 8)]
    joints = []   # (stamp, q, dq)

    def stamp_ns(msg):
        return msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec

    slop_ns = int(float(config["sync_slop_s"]) * 1e9)
    min_spacing_ns = int(0.9e9 / max(1.0, float(config["inference_hz"])))
    colors, depths = [], []
    t_first = None
    last_stamp = None
    records = []
    wall0 = time.monotonic()

    failures = []

    def process(color_msg, depth_msg):
        frame.clear()
        frame.update({s: _empty_side() for s in sides})
        cursor["side"] = -1
        for side in sides:
            node.state_pubs[side].last = None
        try:
            node.process_rgbd(color_msg, depth_msg)
        except Exception as exc:
            # As processing_loop does live: the frame is lost, the node goes on
            if not failures:
                traceback.print_exc()
            failures.append((stamp_ns(color_msg) * 1e-9, repr(exc)))
            return
        for side in sides:
            _record_published(frame[side], node.state_pubs[side].last)
        records.append((stamp_ns(color_msg) * 1e-9, node.is_engaged,
                        {s: dict(frame[s]) for s in sides}))
        if len(records) % 300 == 0:
            rate = len(records) / (time.monotonic() - wall0)
            print(f"  {len(records)} frames, {rate:.1f} frames/s", flush=True)

    while reader.has_next():
        topic, data, bag_ns = reader.read_next()
        msg = deserialize_message(data, msg_types[topic])
        if topic == joint_topic:
            index = {n: i for i, n in enumerate(msg.name)}
            if all(n in index for n in joint_names) and len(msg.velocity) == len(msg.name):
                stamp = stamp_ns(msg) or bag_ns
                joints.append((stamp * 1e-9, [msg.position[index[n]] for n in joint_names],
                               [msg.velocity[index[n]] for n in joint_names]))
            continue
        if topic == "/tf_static":
            for tr in msg.transforms:
                node.tf_buffer.set_transform_static(tr, "replay")
            continue
        if topic == "/tf":
            for tr in msg.transforms:
                node.tf_buffer.set_transform(tr, "replay")
            continue
        if topic == info_topic:
            if node.measurer.intrinsics is None:
                node.camera_info_cb(msg)
            continue
        (colors if topic == color_topic else depths).append(msg)
        # Pair the two streams as an approximate-time synchronizer with the same slop
        while colors and depths:
            tc, td = stamp_ns(colors[0]), stamp_ns(depths[0])
            if abs(tc - td) > slop_ns:
                (colors if tc < td else depths).pop(0)
                continue
            color_msg, depth_msg = colors.pop(0), depths.pop(0)
            t_first = tc if t_first is None else t_first
            rel_s = (tc - t_first) * 1e-9
            if rel_s < args.start_s:
                break
            if args.duration_s and rel_s > args.start_s + args.duration_s:
                break
            # The worker's rate cap, on the image stamps
            if last_stamp is not None and 0 <= tc - last_stamp < min_spacing_ns:
                break
            last_stamp = tc
            process(color_msg, depth_msg)
        if args.duration_s and t_first is not None and \
                (stamp_ns(msg) - t_first) * 1e-9 > args.start_s + args.duration_s + 1.0:
            break

    if not records:
        raise SystemExit("no frame processed")
    out = dict(
        t=np.array([r[0] for r in records]),
        engaged=np.array([r[1] for r in records]),
        sides=np.array(sides),
        R_cb=np.asarray(node.R_camera_to_base, dtype=float),
        t_cb=np.asarray(node.t_camera_to_base, dtype=float),
        intrinsics=np.asarray(node.measurer.intrinsics, dtype=float),
        segment_max=np.asarray(node.segment_max_m if node.segment_max_m else [], dtype=float),
        config=json.dumps(config),
        raw_landmarks=not bool(config["smooth_landmarks"]),
        bag=os.path.abspath(args.bag),
        failed_t=np.array([f[0] for f in failures]),
        joints_t=np.array([j[0] for j in joints]),
        joints_q=np.array([j[1] for j in joints]).reshape(-1, 7),
        joints_dq=np.array([j[2] for j in joints]).reshape(-1, 7),
    )
    for side in sides:
        for key in records[0][2][side]:
            out[f"{side}/{key}"] = np.stack([r[2][side][key] for r in records])
    np.savez_compressed(args.out, **out)
    steps = sum(out[f"{s}/step"].sum() for s in sides)
    print(f"{len(records)} frames ({steps} filter steps over {len(sides)} sides) "
          f"in {time.monotonic() - wall0:.0f} s -> {args.out}")
    if failures:
        print(f"{len(failures)} frames failed in process_rgbd (lost, as live); first: "
              f"t={failures[0][0]:.3f} {failures[0][1]}")
    node.destroy_node()
    if rclpy.ok():   # the tracker may have shut the context down already
        rclpy.shutdown()


# ═════════════════════════════════════════════════════════════════════════════
#  A recorded replay, and the camera geometry of its points
# ═════════════════════════════════════════════════════════════════════════════

class Run:
    """A replay .npz: per side and per frame, what the tracker measured and filtered."""

    def __init__(self, path):
        data = np.load(path)
        self.path = path
        self.name = os.path.splitext(os.path.basename(path))[0]
        self.t = data["t"]
        self.engaged = data["engaged"]
        self.sides = [str(s) for s in data["sides"]]
        self.R_cb = data["R_cb"]
        self.t_cb = data["t_cb"]
        self.intrinsics = tuple(float(x) for x in data["intrinsics"])
        self.segment_max = data["segment_max"]
        self.config = json.loads(str(data["config"]))
        self.raw_landmarks = bool(data["raw_landmarks"])
        self.side = {s: {k.split("/", 1)[1]: data[k] for k in data.files if k.startswith(s + "/")}
                     for s in self.sides}

    @property
    def label(self):
        return f"{self.name} (MediaPipe smoothing {'OFF' if self.raw_landmarks else 'ON'})"

    def own(self, side):
        """(N, 4): an accepted measurement with the keypoint's own depth, filter running."""
        r = self.side[side]
        return r["step"][:, None] & np.isfinite(r["z"]).all(axis=2) & (r["own_depth"] > 0.0)


def ray_basis(run, points):
    """(..., 3, 3) unit rows in the base frame: along the camera ray, across it (image x, y)."""
    ray = np.asarray(points, dtype=float) - run.t_cb
    ray /= np.linalg.norm(ray, axis=-1, keepdims=True)
    image_x = run.R_cb[:, 0]
    across_x = image_x - (ray @ image_x)[..., None] * ray
    across_x /= np.linalg.norm(across_x, axis=-1, keepdims=True)
    across_y = np.cross(ray, across_x)
    return np.stack([ray, across_x, across_y], axis=-2)


def robust_std(x, scale=1.0):
    """1.4826 MAD / sqrt(scale): the std of a Gaussian core, blind to a few outliers."""
    x = np.asarray(x, dtype=float)
    x = x[np.isfinite(x)]
    if x.size < 10:
        return np.nan
    return 1.4826 * float(np.median(np.abs(x - np.median(x)))) / np.sqrt(scale)


# ═════════════════════════════════════════════════════════════════════════════
#  noise — the measurement alone, no filter
# ═════════════════════════════════════════════════════════════════════════════

#: Order-n differences: coefficients and Var(D_n) / sigma^2 for white noise
DIFFERENCES = {2: (np.array([1.0, -2.0, 1.0]), 6.0),
               3: (np.array([-1.0, 3.0, -3.0, 1.0]), 20.0)}
DEPTH_BINS = (1.0, 1.75, 2.5, 3.25, 4.0)


def differences(values, valid, t, order):
    """Order-n differences of values (N, ...) over n+1 consecutive valid, evenly spaced frames.

    Returns the differences and the index of the frame each one is centred on.
    """
    coeffs, _ = DIFFERENCES[order]
    spacing = np.diff(t)
    nominal = np.median(spacing)
    even = np.abs(spacing - nominal) <= 0.25 * nominal
    n = len(t) - order
    ok = np.ones(n, dtype=bool)
    for j in range(order + 1):
        ok &= valid[j:j + n]
    for j in range(order):
        ok &= even[j:j + n]
    idx = np.flatnonzero(ok)
    out = sum(c * values[idx + j] for j, c in enumerate(coeffs))
    return out, idx + order // 2


def noise_table(run):
    """Per keypoint and depth bin: n, sigma_u, sigma_v [px] and sigma_d [m] by D2 and D3."""
    fx = run.intrinsics[0]
    rows = []
    for i, name in enumerate(KEYPOINTS):
        series = {order: {"u": [], "v": [], "d": [], "z": []} for order in DIFFERENCES}
        for side in run.sides:
            r = run.side[side]
            valid = run.own(side)[:, i] & np.isfinite(r["lm_uv"][:, i]).all(axis=1)
            for order in DIFFERENCES:
                for key, values in (("u", r["lm_uv"][:, i, 0]), ("v", r["lm_uv"][:, i, 1]),
                                    ("d", r["depth"][:, i])):
                    diff, centre = differences(values, valid, run.t, order)
                    series[order][key].append(diff)
                series[order]["z"].append(r["depth"][centre, i])
        series = {o: {k: np.concatenate(v) for k, v in s.items()} for o, s in series.items()}
        for lo, hi in zip((0.0,) + DEPTH_BINS[:-1], (np.inf,) + DEPTH_BINS[1:]):
            row = {"keypoint": name, "bin": "all" if hi == np.inf else f"{lo:.2f}-{hi:.2f}"}
            for order, (_, var) in DIFFERENCES.items():
                s = series[order]
                sel = (s["z"] >= lo) & (s["z"] < hi)
                row[f"n{order}"] = int(sel.sum())
                for key in ("u", "v", "d"):
                    row[f"{key}{order}"] = robust_std(s[key][sel], var)
                if order == 2:
                    row["z"] = float(np.median(s["z"][sel])) if sel.any() else np.nan
                    big = np.abs(s["d"][sel]) > 5.0 * np.sqrt(var) * row["d2"]
                    row["d_outliers"] = float(big.mean()) if sel.any() else np.nan
            row["lateral"] = 0.5 * (row["u2"] + row["v2"]) * row["z"] / fx
            rows.append(row)
    return rows


def fit_depth_std(rows):
    """Weighted least squares of sigma_d^2 = floor^2 + (quad Z^2)^2 over the depth bins."""
    bins = [r for r in rows if r["bin"] != "all" and r["n2"] >= 30 and np.isfinite(r["d2"])]
    if len(bins) < 2:
        return np.nan, np.nan
    z = np.array([r["z"] for r in bins])
    s2 = np.array([r["d2"] for r in bins]) ** 2
    w = np.sqrt([r["n2"] for r in bins])
    A = np.stack([np.ones_like(z), z**4], axis=1) * w[:, None]
    (a, b), *_ = np.linalg.lstsq(A, s2 * w, rcond=None)
    return float(np.sqrt(max(a, 0.0))), float(np.sqrt(max(b, 0.0)))


def noise(args):
    for path in args.runs:
        run = Run(path)
        rows = noise_table(run)
        print(f"\n=== {run.label}: own-depth measurements, fx = {run.intrinsics[0]:.0f} px")
        print(f"{'keypoint':9s} {'Z bin [m]':>10s} {'n':>5s} {'Z':>5s} | "
              f"{'su D2':>6s} {'su D3':>6s} {'sv D2':>6s} {'sv D3':>6s} [px] | "
              f"{'sd D2':>6s} {'sd D3':>6s} [cm] | {'s_lat':>5s} [cm] | d >5s")
        for r in rows:
            if r["n2"] < 30:
                continue
            print(f"{r['keypoint']:9s} {r['bin']:>10s} {r['n2']:5d} {r['z']:5.2f} | "
                  f"{r['u2']:6.2f} {r['u3']:6.2f} {r['v2']:6.2f} {r['v3']:6.2f}      | "
                  f"{100 * r['d2']:6.2f} {100 * r['d3']:6.2f}      | {100 * r['lateral']:5.2f}      | "
                  f"{100 * r['d_outliers']:4.1f}%")
        pooled = []
        for i, name in enumerate(KEYPOINTS):
            floor, quad = fit_depth_std([r for r in rows if r["keypoint"] == name])
            pooled.append(next(r for r in rows if r["keypoint"] == name and r["bin"] == "all"))
            print(f"  {name:8s} sigma_d(Z) = hypot({100 * floor:.2f} cm, {quad:.4f} Z^2)")
        px = np.nanmedian([0.5 * (r["u2"] + r["v2"]) for r in pooled])
        print(f"  pixel std, median over keypoints (D2): {px:.2f} px")


# ═════════════════════════════════════════════════════════════════════════════
#  compare — the filter replayed with each measurement model
# ═════════════════════════════════════════════════════════════════════════════

def _probe_filter_class():
    from franka_experiments.utils.arm_kf import ArmKalmanFilter

    class ProbeKalmanFilter(ArmKalmanFilter):
        """ArmKalmanFilter logging every update: frame, keypoint, innovation, S, accepted."""

        def __init__(self, *a, **kw):
            super().__init__(*a, **kw)
            self.frame = -1
            self.log = []

        def _update_keypoint(self, index, position, R_custom=None):
            if not self.initialized[index]:
                return super()._update_keypoint(index, position, R_custom)
            R = self.R if R_custom is None else R_custom
            innovation = position - self.H @ self.x[index]
            S = self.H @ self.P[index] @ self.H.T + R
            accepted = super()._update_keypoint(index, position, R_custom)
            self.log.append((self.frame, index, innovation, S, accepted))
            return accepted

    return ProbeKalmanFilter


def parse_variant(spec):
    """'run.npz@model:key=value,...' -> (path, model, params). Model: isotropic | pinhole.

    Keys: px [px], d0 [m], dq [1/m] (pinhole), sm [m] (isotropic), sa [m/s^2] (one value or
    one per keypoint, sa=2/3/3.5/4) and bo = 1 (bearing update of gated measurements).
    """
    path, _, model = spec.partition("@")
    name, _, params = model.partition(":")
    values = dict(item.split("=") for item in params.split(",") if item)
    # sa may be one value per keypoint: sa=2/3/3.5/4 (shoulder/elbow/wrist/index)
    return path, name or "isotropic", {
        k: [float(x) for x in v.split("/")] if "/" in v else float(v) for k, v in values.items()}


def run_filter(run, model, params):
    """The tracker's filter loop (step, limit_segments, resets) on the recorded measurements.

    Covariances come from ArmMeasurer itself, with the requested model. Returns per side:
    the published estimate (pos, vel, valid), the state right after each step and the log
    of the updates.
    """
    from franka_experiments.utils.human_measurement import ArmMeasurer
    from franka_experiments.utils.human_utils import measurement_age

    cfg = run.config
    sm = params.get("sm", cfg["kf_measurement_std"])
    measurer = ArmMeasurer(
        KEYPOINTS, cfg["visibility_threshold"], cfg["depth_patch_radius"], cfg["min_depth_m"],
        cfg["max_depth_m"], sm, cfg["fallback_depth_std_m"], measurement_model=model,
        pixel_std_px=params.get("px", cfg.get("kf_pixel_std_px", 0.0)),
        depth_std_m=params.get("d0", cfg.get("kf_depth_std_m", 0.0)),
        depth_std_quad=params.get("dq", cfg.get("kf_depth_std_quad", 0.0)))
    measurer.set_intrinsics(*run.intrinsics)
    camera_tf = (run.R_cb, run.t_cb)
    max_age = float(cfg["max_state_age_s"])
    reset_after = max(max_age, float(cfg["reset_after_s"]))
    filter_class = _probe_filter_class()
    n = len(run.t)
    results = {}
    for side in run.sides:
        r = run.side[side]
        kf = filter_class(
            dt=float(cfg["kf_nominal_dt"]),
            process_accel_std=params.get("sa", cfg["kf_process_accel_std"]),
            measurement_std=sm,
            initial_position_std=float(cfg["kf_initial_position_std"]),
            initial_velocity_std=float(cfg["kf_initial_velocity_std"]),
            visibility_threshold=float(cfg["visibility_threshold"]),
            mahalanobis_threshold=float(cfg["kf_mahalanobis_threshold"]),
        )
        out = dict(pos=np.full((n, 4, 3), np.nan), vel=np.full((n, 4, 3), np.nan),
                   valid=np.zeros((n, 4), dtype=bool), x=np.full((n, 4, 6), np.nan),
                   bearing=np.zeros((n, 4), dtype=bool))
        last_valid = np.full(4, np.nan)
        for k in range(n):
            if not r["step"][k]:
                continue
            if k == 0 or not run.engaged[k - 1]:
                # (Re)engaged: the tracker reset every filter when it disengaged
                kf.reset()
                last_valid[:] = np.nan
            pixels = {i: (int(r["pix"][k, i, 0]), int(r["pix"][k, i, 1]))
                      for i in range(4) if np.isfinite(r["pix"][k, i, 0])}
            covs = measurer.deproject_arm(pixels, r["own_depth"][k].copy(), camera_tf)[2]
            kf.frame = k
            _, _, measured = kf.step(positions=r["z"][k], visibilities=r["vis"][k],
                                     depths=r["depth"][k], dt=float(r["dt"][k]),
                                     measurement_covariances=covs,
                                     ray_origin=run.t_cb if params.get("bo") else None)
            out["x"][k] = np.where(kf.initialized[:, None], kf.x, np.nan)
            out["bearing"][k] = kf.bearing_only
            if len(run.segment_max):
                pos, vel = kf.limit_segments(measured, run.segment_max)
            else:
                pos, vel = kf.get_estimates()
            last_valid[measured] = run.t[k]
            age = measurement_age(last_valid, run.t[k])
            for i in range(4):
                if kf.initialized[i] and age[i] > reset_after:
                    kf.reset(i)
                    last_valid[i] = np.nan
                    pos[i] = vel[i] = np.nan
                    age[i] = -1.0
            valid = (kf.initialized & np.all(np.isfinite(pos), axis=1)
                     & (age >= 0.0) & (age <= max_age))
            speed = np.linalg.norm(vel, axis=1)
            for i in range(4):
                if valid[i] and speed[i] > float(cfg["max_speed_m_s"]):
                    valid[i] = False
                    kf.reset(i)
            out["pos"][k], out["vel"][k], out["valid"][k] = pos, vel, valid
        out["log"] = kf.log
        results[side] = out
    return results


CHI2_3_MEDIAN = 2.366   # medians of chi^2_3 and chi^2_1: a consistent filter's NIS
CHI2_1_MEDIAN = 0.4549


def update_stats(run, result, own_only=True):
    """Innovations of the logged updates, normalised along and across the camera ray."""
    nis, w, y, keys, gated = [], [], [], [], []
    threshold = float(run.config["kf_mahalanobis_threshold"])
    for side in run.sides:
        r = run.side[side]
        for k, i, innovation, S, accepted in result[side]["log"]:
            if own_only and r["own_depth"][k, i] <= 0.0:
                continue
            basis = ray_basis(run, r["z"][k, i])
            nis.append(float(innovation @ np.linalg.solve(S, innovation)))
            w.append(basis @ innovation / np.sqrt(np.einsum("ij,jk,ik->i", basis, S, basis)))
            y.append(basis @ innovation)
            keys.append((side, i, k))
            gated.append(threshold > 0.0 and nis[-1] > threshold)
    return dict(nis=np.array(nis), w=np.array(w).reshape(-1, 3), y=np.array(y).reshape(-1, 3),
                keys=keys, gated=np.array(gated, dtype=bool))


def lag1(stats):
    """Lag-1 autocorrelation of the normalised innovations, along and across the ray.

    Consecutive updates of one keypoint only; values clipped at +-4 so a few outliers do
    not decide it. A white innovation sequence (the optimal filter's) gives 0.
    """
    index = {key: j for j, key in enumerate(stats["keys"])}
    pairs = [(j, index[(s, i, k + 1)]) for j, (s, i, k) in enumerate(stats["keys"])
             if (s, i, k + 1) in index]
    if len(pairs) < 10:
        return np.nan, np.nan
    a, b = np.array(pairs).T
    w = np.clip(stats["w"], -4.0, 4.0)

    def corr(cols):
        x, y = w[a][:, cols].ravel(), w[b][:, cols].ravel()
        return float(np.sum(x * y) / np.sqrt(np.sum(x * x) * np.sum(y * y)))

    return corr([0]), corr([1, 2])


def prediction_errors(variants, reference, horizons):
    """Error of the constant-velocity prediction p + v (t' - t) against a later measurement.

    The target is an own-depth measurement of the reference run at the frame h steps
    later. Its noise is the same whatever filter predicted, so differences between filters
    are differences in their true prediction error. Only predictions every variant could
    make are compared. Returns {h: ([(n, 3) errors along the ray / across x / across y per
    variant], (n,) keypoint index of each error)}.
    """
    ref_index = {round(t * 1e3): j for j, t in enumerate(reference.t)}
    out = {}
    for h in horizons:
        per_variant = []
        for run, result in variants:
            errors = {}
            nominal = np.median(np.diff(run.t))
            for side in run.sides:
                target_ok = reference.own(side)
                z_ref = reference.side[side]["z"]
                res = result[side]
                for k in np.flatnonzero(res["valid"].any(axis=1)):
                    if k + h >= len(run.t):
                        break
                    lead = run.t[k + h] - run.t[k]
                    if abs(lead - h * nominal) > 0.25 * nominal:
                        continue
                    j = ref_index.get(round(run.t[k + h] * 1e3))
                    if j is None:
                        continue
                    for i in np.flatnonzero(res["valid"][k] & target_ok[j]):
                        predicted = res["pos"][k, i] + res["vel"][k, i] * lead
                        errors[(side, i, round(run.t[k] * 1e3))] = (z_ref[j, i] - predicted,
                                                                     z_ref[j, i])
            per_variant.append(errors)
        keys = sorted(set.intersection(*(set(e) for e in per_variant)))
        rows = []
        for (run, _), errors in zip(variants, per_variant):
            e = np.array([errors[key][0] for key in keys]).reshape(-1, 3)
            z = np.array([errors[key][1] for key in keys]).reshape(-1, 3)
            rows.append(np.einsum("nij,nj->ni", ray_basis(run, z), e))
        out[h] = (rows, np.array([key[1] for key in keys], dtype=int))
    return out


def velocity_jitter(run, result, keypoint=None):
    """RMS change of the published velocity between consecutive frames [m/s], along / across."""
    steps = []
    for side in run.sides:
        res = result[side]
        both = res["valid"][1:] & res["valid"][:-1]
        if keypoint is not None:
            both[:, [i for i in range(4) if i != keypoint]] = False
        k, i = np.nonzero(both)
        dv = res["vel"][k + 1, i] - res["vel"][k, i]
        steps.append(np.einsum("nij,nj->ni", ray_basis(run, res["pos"][k + 1, i]), dv))
    dv = np.concatenate(steps)
    return float(np.sqrt(np.mean(dv[:, 0] ** 2))), float(np.sqrt(np.mean(dv[:, 1:] ** 2)))


def compare(args):
    variants, labels = [], []
    for spec in args.variants:
        path, model, params = parse_variant(spec)
        run = Run(path)
        result = run_filter(run, model, params)
        variants.append((run, result))
        tag = ",".join(f"{k}={'/'.join(f'{x:g}' for x in v) if isinstance(v, list) else f'{v:g}'}"
                       for k, v in params.items())
        labels.append(f"{run.name}@{model}" + (f":{tag}" if tag else ""))
        if model == "isotropic" and not params:
            # The replay must reproduce what the tracker did, step for step
            diffs = [np.nanmax(np.abs(result[s]["x"] - run.side[s]["x"]))
                     for s in run.sides if run.side[s]["step"].any()]
            print(f"[fidelity] {labels[-1]}: max |x_replay - x_tracker| = {max(diffs):.2e}")
    reference = Run(args.reference) if args.reference else variants[0][0]

    width = max(len(label) for label in labels)
    print(f"\nConsistency on own-depth updates (1.00 = consistent; > 1: the filter is "
          f"overconfident in that direction)")
    print(f"{'variant':{width}s} {'n':>6s} {'NIS mean':>8s} {'NIS med/2.37':>12s} "
          f"{'gated':>6s} | {'ray':>5s} {'across':>6s} | {'rms y ray':>9s} {'across':>7s} [cm] | "
          f"{'lag-1 ray':>9s} {'across':>6s} | {'gated: ray-led':>14s}")
    for label, (run, result) in zip(labels, variants):
        st = update_stats(run, result)
        ray_ratio = np.median(st["w"][:, 0] ** 2) / CHI2_1_MEDIAN
        across_ratio = np.median(st["w"][:, 1:] ** 2) / CHI2_1_MEDIAN
        rho_ray, rho_across = lag1(st)
        g = st["gated"]
        ray_led = (np.mean(st["w"][g, 0] ** 2 > np.sum(st["w"][g, 1:] ** 2, axis=1))
                   if g.any() else np.nan)
        print(f"{label:{width}s} {len(st['nis']):6d} {np.mean(st['nis']):8.2f} "
              f"{np.median(st['nis']) / CHI2_3_MEDIAN:12.2f} {100 * g.mean():5.1f}% | "
              f"{ray_ratio:5.2f} {across_ratio:6.2f} | "
              f"{100 * robust_std(st['y'][:, 0]):9.2f} {100 * robust_std(st['y'][:, 1:]):7.2f}      | "
              f"{rho_ray:9.2f} {rho_across:6.2f} | {100 * ray_led:13.0f}%")

    print(f"\nPer keypoint: median NIS along / across the ray, normalised (1.00 = consistent)")
    print(f"{'variant':{width}s} " + " ".join(f"{name:>13s}" for name in KEYPOINTS))
    for label, (run, result) in zip(labels, variants):
        st = update_stats(run, result)
        kp = np.array([key[1] for key in st["keys"]])
        cells = []
        for i in range(4):
            sel = kp == i
            cells.append(f"{np.median(st['w'][sel, 0] ** 2) / CHI2_1_MEDIAN:5.2f} / "
                         f"{np.median(st['w'][sel, 1:] ** 2) / CHI2_1_MEDIAN:5.2f}"
                         if sel.sum() > 10 else f"{'-':>13s}")
        print(f"{label:{width}s} " + " ".join(cells))

    horizons = [int(h) for h in args.horizons.split(",")]
    errors = prediction_errors(variants, reference, horizons)
    print(f"\nConstant-velocity prediction error against the measurement h frames later "
          f"(reference {reference.label}) [cm]:\n  across = rms across the ray, per axis "
          f"(depth outliers do not reach it); ray = robust std along the ray; "
          f"p95 = 95th percentile of the 3D error")
    print(f"{'variant':{width}s} " + " ".join(f"{'h=' + str(h) + ' across/ray/p95':>24s}"
                                             for h in horizons))
    for v, label in enumerate(labels):
        cells = []
        for h in horizons:
            local = errors[h][0][v]
            cells.append(f"{100 * np.sqrt(np.mean(local[:, 1:] ** 2)):6.2f} "
                         f"{100 * robust_std(local[:, 0]):6.2f} "
                         f"{100 * np.percentile(np.linalg.norm(local, axis=1), 95):6.2f}")
        print(f"{label:{width}s} " + " ".join(f"{c:>24s}" for c in cells))
    print(f"{'pairs':{width}s} " + " ".join(f"{len(errors[h][1]):>24d}" for h in horizons))
    if args.by_keypoint:
        for i, name in enumerate(KEYPOINTS):
            print(f"  {name}: across rms / ray robust std / p95 [cm], then velocity jitter along/across [m/s]")
            for v, label in enumerate(labels):
                cells = []
                for h in horizons:
                    local = errors[h][0][v][errors[h][1] == i]
                    cells.append(f"{100 * np.sqrt(np.mean(local[:, 1:] ** 2)):6.2f} "
                                 f"{100 * robust_std(local[:, 0]):6.2f} "
                                 f"{100 * np.percentile(np.linalg.norm(local, axis=1), 95):6.2f}"
                                 if len(local) else "-")
                ray_j, across_j = velocity_jitter(*variants[v], keypoint=i)
                print(f"  {label:{width}s} " + " ".join(f"{c:>24s}" for c in cells)
                      + f"   {ray_j:.3f}/{across_j:.3f}")

    print(f"\nVelocity jitter: rms change between consecutive frames [m/s], along / across")
    for label, (run, result) in zip(labels, variants):
        ray_j, across_j = velocity_jitter(run, result)
        bearing = sum(result[s]["bearing"].sum() for s in run.sides)
        print(f"{label:{width}s} {ray_j:6.3f} / {across_j:6.3f}"
              + (f"   ({bearing} bearing updates)" if bearing else ""))


# ═════════════════════════════════════════════════════════════════════════════
#  CLI
# ═════════════════════════════════════════════════════════════════════════════

def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    sub = parser.add_subparsers(dest="cmd", required=True)

    p = sub.add_parser("replay", help="run HumanTracker on a bag and record every step")
    p.add_argument("bag")
    p.add_argument("--out", required=True)
    p.add_argument("--raw-landmarks", action="store_true",
                   help="MediaPipe with smooth_landmarks=False")
    p.add_argument("--set", default="",
                   help='JSON overrides of human_tracker parameters, e.g. \'{"pose_side": "right"}\'')
    p.add_argument("--start-s", type=float, default=0.0)
    p.add_argument("--duration-s", type=float, default=0.0)
    p.add_argument("--domain-id", type=int, default=87,
                   help="ROS_DOMAIN_ID of the isolated replay graph")
    p.set_defaults(func=replay)

    p = sub.add_parser("noise", help="noise of u, v and d from the measurements alone")
    p.add_argument("runs", nargs="+", help="replay .npz files")
    p.set_defaults(func=noise)

    p = sub.add_parser("compare", help="replay the filter with each measurement model")
    p.add_argument("variants", nargs="+",
                   help="run.npz[@isotropic|pinhole[:px=..,d0=..,dq=..,sm=..,sa=..]]")
    p.add_argument("--reference", default="",
                   help="replay .npz whose measurements are the prediction targets "
                        "(default: the first variant's)")
    p.add_argument("--horizons", default="1,3,6,9", help="prediction horizons [frames]")
    p.add_argument("--by-keypoint", action="store_true",
                   help="also break the prediction error and the jitter down per keypoint")
    p.set_defaults(func=compare)

    args = parser.parse_args(argv)
    # This checkout's package, ahead of the installed copy
    sys.path[:] = [PKG] + [p for p in sys.path if os.path.abspath(p) != PKG]
    args.func(args)


if __name__ == "__main__":
    main()
