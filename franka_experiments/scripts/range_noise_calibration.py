#!/usr/bin/env python3
"""Fit the depth sensor's range-noise constant, sigma_d_px, from a static target.

WHY THIS EXISTS
---------------
``utils.cbf_state_rows.sensor_range_uncertainty`` prices the raw depth
reading's own noise as a barrier tightening:

    sigma_z(z) = z^2 / (f_px * baseline_m) * sigma_d_px
    margin     = min(k_sigma * sigma_z, margin_max)

f_px comes straight off the live CameraInfo (exact, no fitting needed) and
baseline_m is the sensor's physical stereo/structured-light baseline (a
manufacturer spec, not something this script measures). sigma_d_px — the
disparity (or equivalent) measurement error, roughly constant for a given
sensor — is the one constant that has to be FIT, and this script fits it the
same way scripts/latency_budget.py measures its own constants: with real
stamps and real pixels, not a guess.

METHOD
------
Point the camera at a static, flat, roughly-Lambertian target (a wall, a
board — nothing glossy or transparent) at several known distances. At each
distance, ``capture`` samples the raw range of a small patch around the image
centre for a few seconds and records its empirical mean and std-dev. Once
several distances are captured, ``fit`` regresses the model above (no
intercept: zero range error at zero range is the physical boundary
condition) and reports sigma_d_px together with the fit quality — it does NOT
assume the quadratic law holds, it shows the residual.

USAGE (inside the ROS container, camera streaming)
----------------------------------------------------
    python3 scripts/range_noise_calibration.py capture --distance 0.20
    python3 scripts/range_noise_calibration.py capture --distance 0.40
    python3 scripts/range_noise_calibration.py capture --distance 0.70
    python3 scripts/range_noise_calibration.py capture --distance 1.00
    python3 scripts/range_noise_calibration.py fit --baseline-m 0.095

``capture`` needs a sourced ROS 2 environment and a live depth stream.
``fit`` needs only numpy and the JSON file ``capture`` wrote.
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)

#: Depth images on this stack are uint16 millimetres — same convention as
#: distance_engine.DistanceEngine._DEPTH_TO_M. Duplicated rather than
#: imported: that name is private to distance_engine and this script has no
#: other dependency on it.
_DEPTH_TO_M = 0.001

DEFAULT_OUT = '/tmp/range_noise_calibration.json'


# ═════════════════════════════════════════════════════════════════════════════
#  fit — pure regression, no ROS
# ═════════════════════════════════════════════════════════════════════════════

def fit_sigma_d(z_m, sigma_z_m, f_px, baseline_m):
    """Least-squares fit of ``sigma_d_px`` from paired (z, measured sigma_z).

    Model: ``sigma_z = z^2 / (f_px * baseline_m) * sigma_d_px`` — linear in
    sigma_d_px with NO INTERCEPT, because zero disparity error must predict
    zero range error at every distance; fitting an intercept would let a
    single bad distance bin absorb its error as a constant offset instead of
    the quadratic term the physics actually predicts.

    ``f_px`` may be a scalar (one focal length for every sample) or an array
    the same length as ``z_m`` (a slightly different one read per capture).

    Args:
        z_m: (N,) ground-truth distances [m].
        sigma_z_m: (N,) empirical std-dev of the raw range reading at each
            distance [m].
        f_px: focal length(s) [px].
        baseline_m: [m] the sensor's physical stereo baseline.

    Returns:
        ``(sigma_d_px, r_squared)``. ``r_squared`` is 1.0 for a perfect fit,
        0.0 (not negative) when the fit explains no more variance than the
        mean would, and exactly ``(0.0, 0.0)`` when every ``x`` is zero (all
        ``z_m`` are zero) — no information to fit from.
    """
    z = np.asarray(z_m, dtype=np.float64).ravel()
    y = np.asarray(sigma_z_m, dtype=np.float64).ravel()
    f = np.broadcast_to(np.asarray(f_px, dtype=np.float64), z.shape)
    x = (z * z) / (f * float(baseline_m))
    denom = float(np.sum(x * x))
    if denom <= 0.0 or not np.isfinite(denom):
        return 0.0, 0.0
    sigma_d = float(np.sum(x * y) / denom)
    y_pred = sigma_d * x
    ss_res = float(np.sum((y - y_pred) ** 2))
    ss_tot = float(np.sum((y - np.mean(y)) ** 2))
    if ss_tot <= 0.0:
        # Every sample reports the same sigma_z (or there is only one
        # distance): R^2 is undefined by the usual formula. A perfect
        # (zero-residual) fit is still reported as 1.0; anything else as 0.0
        # rather than dividing by zero.
        r2 = 1.0 if ss_res < 1e-18 else 0.0
    else:
        r2 = max(0.0, 1.0 - ss_res / ss_tot)
    return sigma_d, r2


def cmd_fit(args):
    if not os.path.isfile(args.inp):
        raise SystemExit(f'no capture file at {args.inp} — run `capture` first')
    with open(args.inp) as fh:
        records = json.load(fh)
    if len(records) < 2:
        raise SystemExit(
            f'only {len(records)} distance(s) captured in {args.inp} — need at '
            f'least 2 to fit a quadratic law, and 4+ spread across the working '
            f'range (0.15-1.0 m) to trust the fit')

    z = np.array([r['distance_truth_m'] for r in records])
    sigma_z = np.array([r['std_z_m'] for r in records])
    f_px = np.array([r['f_px'] for r in records])
    n = np.array([r['n_samples'] for r in records])

    order = np.argsort(z)
    z, sigma_z, f_px, n, records = (z[order], sigma_z[order], f_px[order],
                                    n[order], [records[i] for i in order])

    print(f'{"distance [m]":>14s}  {"mean_z [m]":>12s}  {"std_z [m]":>12s}  '
          f'{"n":>8s}  {"f_px":>8s}')
    for r in records:
        print(f'{r["distance_truth_m"]:>14.3f}  {r["mean_z_m"]:>12.4f}  '
              f'{r["std_z_m"]:>12.5f}  {r["n_samples"]:>8d}  {r["f_px"]:>8.1f}')

    sigma_d, r2 = fit_sigma_d(z, sigma_z, f_px, args.baseline_m)

    print()
    print(f'fitted sigma_d_px = {sigma_d:.4f} px   (R^2 = {r2:.4f} over '
          f'{len(records)} distances)')
    if r2 < 0.5:
        print('WARNING: R^2 is low — the quadratic law does not explain most '
              'of the variance. Do not trust this fit; check for outliers, a '
              'non-flat/non-Lambertian target, or a distance range too narrow '
              'to separate the quadratic term from measurement noise.')

    # Predicted vs actual, per distance, so a poor fit is visible per-bin and
    # not hidden inside one R^2 number.
    predicted = sigma_d * (z * z) / (f_px * args.baseline_m)
    print()
    print(f'{"distance [m]":>14s}  {"actual sigma_z":>16s}  '
          f'{"predicted sigma_z":>18s}  {"ratio":>8s}')
    for zi, a, p in zip(z, sigma_z, predicted):
        ratio = a / p if p > 0.0 else float('nan')
        print(f'{zi:>14.3f}  {a:>16.5f}  {p:>18.5f}  {ratio:>8.2f}')

    print()
    print('Paste into config/fr3_control.yaml under sensor_range_uncertainty:')
    print(f'  sensor_range_baseline_m: {args.baseline_m:.4f}')
    print(f'  sensor_range_sigma_d_px: {sigma_d:.4f}')
    print(f'  # f_px measured at capture time: '
          f'{" / ".join(f"{f:.1f}" for f in np.unique(f_px))}')


# ═════════════════════════════════════════════════════════════════════════════
#  capture — live: sample a patch of the depth image at one known distance
# ═════════════════════════════════════════════════════════════════════════════

def cmd_capture(args):
    import rclpy
    from cv_bridge import CvBridge
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import CameraInfo, Image

    class Probe(Node):
        def __init__(self):
            super().__init__('range_noise_calibration_probe')
            self.bridge = CvBridge()
            self.samples = []
            self.f_px = None
            qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
            self.create_subscription(Image, args.depth_topic, self._on_depth, qos)
            self.create_subscription(CameraInfo, args.info_topic, self._on_info, qos)

        def _on_info(self, msg):
            if self.f_px is None:
                # fx and fy are equal to within manufacturing tolerance on the
                # RealSense cameras this rig uses — same simplification
                # distance_engine and sensor_range_uncertainty both make.
                self.f_px = float(msg.k[0])
                self.get_logger().info(f'f_px = {self.f_px:.2f} (from CameraInfo.k[0])')

        def _on_depth(self, msg):
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            h, w = depth.shape[:2]
            cy, cx = h // 2 + args.roi_offset[1], w // 2 + args.roi_offset[0]
            r = args.roi_half_px
            patch = depth[max(cy - r, 0):cy + r, max(cx - r, 0):cx + r]
            z = patch.astype(np.float64).ravel() * _DEPTH_TO_M
            z = z[np.isfinite(z) & (z > 0.0)]
            if z.size:
                self.samples.append(z)

    rclpy.init()
    node = Probe()
    print(f'Capturing a {2 * args.roi_half_px}x{2 * args.roi_half_px} px patch '
          f'for {args.duration:.1f} s at ground-truth distance '
          f'{args.distance:.3f} m ...')
    t0 = time.time()
    while time.time() - t0 < args.duration:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()

    if not node.samples or node.f_px is None:
        raise SystemExit(
            'no depth samples (or no CameraInfo) received — check '
            '--depth-topic/--info-topic and that the camera is streaming')

    z_all = np.concatenate(node.samples)
    record = dict(
        distance_truth_m=float(args.distance),
        mean_z_m=float(np.mean(z_all)),
        std_z_m=float(np.std(z_all)),
        n_samples=int(z_all.size),
        f_px=float(node.f_px),
        captured_unix_s=time.time(),
    )
    print(f'mean_z={record["mean_z_m"]:.4f} m  std_z={record["std_z_m"]:.5f} m '
          f' n={record["n_samples"]}  (truth={record["distance_truth_m"]:.3f} m, '
          f'error={record["mean_z_m"] - record["distance_truth_m"]:+.4f} m)')

    records = []
    if os.path.isfile(args.out):
        with open(args.out) as fh:
            records = json.load(fh)
    records = [r for r in records
              if abs(r['distance_truth_m'] - args.distance) > 1e-6]
    records.append(record)
    with open(args.out, 'w') as fh:
        json.dump(records, fh, indent=2)
    print(f'-> {args.out} ({len(records)} distance(s) captured so far)')


# ═════════════════════════════════════════════════════════════════════════════

def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='cmd', required=True)

    p = sub.add_parser('capture')
    p.add_argument('--distance', type=float, required=True,
                    help='ground-truth distance to the flat target [m]')
    p.add_argument('--depth-topic', default='/camera/camera/depth/image_rect_raw')
    p.add_argument('--info-topic', default='/camera/camera/depth/camera_info')
    p.add_argument('--roi-half-px', type=int, default=15,
                    help='half-width of the sampled patch, centred on the image')
    p.add_argument('--roi-offset', type=int, nargs=2, default=(0, 0), metavar=('DX', 'DY'),
                    help='shift the patch centre off the image centre, e.g. to avoid a mount')
    p.add_argument('--duration', type=float, default=5.0)
    p.add_argument('--out', default=DEFAULT_OUT)
    p.set_defaults(fn=cmd_capture)

    p = sub.add_parser('fit')
    p.add_argument('--in', dest='inp', default=DEFAULT_OUT)
    p.add_argument('--baseline-m', type=float, required=True,
                    help='sensor physical stereo/structured-light baseline [m] '
                         '(manufacturer spec, NOT tracking.calibration_check.baseline_m)')
    p.set_defaults(fn=cmd_fit)

    args = ap.parse_args()
    args.fn(args)


if __name__ == '__main__':
    main()
