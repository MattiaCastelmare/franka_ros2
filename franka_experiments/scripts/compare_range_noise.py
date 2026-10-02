#!/usr/bin/env python3
"""Phase 4: does the quadratic range-noise law explain the actual error?

WHY THIS EXISTS, AND WHAT IT DOES NOT CLAIM
--------------------------------------------
``utils.cbf_state_rows.sensor_range_uncertainty`` assumes

    sigma_z(z) = z^2 / (f_px * baseline_m) * sigma_d_px

This script does NOT assume that holds — it checks. Two independent modes,
because they check two different things and neither alone is the whole
picture:

``synthetic`` — inject a KNOWN disparity noise sigma_d_px into a synthetic
    flat target at a sweep of known distances, run it through the REAL
    ``DistanceEngine`` (the same argmin/unprojection code path a live camera
    drives), and check that the empirical std of the resulting `range_m`
    readings matches the model's prediction, and that
    ``range_noise_calibration.fit_sigma_d`` recovers the injected constant.
    This validates the CODE and the FITTING PROCEDURE against a law we
    constructed on purpose — it says nothing about what a real sensor's
    sigma_d_px actually is. Needs only numpy; no ROS.

``bag`` — replay a real rosbag with a sphere of KNOWN trajectory injected
    into the depth stream (utils.obstacle_sim.InjectedSphere, the same
    machinery scripts/compare_vobs.py uses), and compare each control
    point's `range_m` against the EXACT analytic ray-sphere intersection
    depth at the same pixel. ``InjectedSphere.render`` paints a NOISE-FREE
    depth (integer-millimetre quantisation is the only error source) unless
    ``--inject-sigma-d-px`` adds synthetic per-pixel disparity noise to it.
    Without that flag this mode measures the PLUMBING's agreement with
    ground truth on real TF / real pixel selection, not sensor noise; with
    it, it is a second, more realistic end-to-end check of the same thing
    `synthetic` checks in isolation.

NEITHER MODE MEASURES THE REAL CAMERA'S ACTUAL sigma_d_px. That number can
only come from a live camera looking at a real target — see
scripts/range_noise_calibration.py. This script exists to make sure the code
and the fit are trustworthy before that number is plugged in.

BAG MODE NEEDS A GEOMETRICALLY PLAUSIBLE INJECTION POINT, PER BAG. Being
close to the CAMERA is not enough for a control point to pick the sphere as
its nearest obstacle: DistanceEngine's argmin is a 3D distance from the
CONTROL POINT to the candidate pixel, and a CP can be physically far (in 3D)
from a point that is optically close to the camera. Tried against
rosbag/arm_complex (recorded to exercise arm motion, not obstacle avoidance)
across several `--inject-start` placements, including ones confirmed by
direct inspection to render inside the ROI and outside the robot's own
exclusion mask: zero on-sphere samples over the whole bag. The blocker is CP-
to-sphere 3D proximity, not the render/mask/ROI pipeline (each checked
independently). Finding a placement that intersects a specific bag's actual
arm trajectory needs interactive tuning against that bag — the `synthetic`
mode above does not have this problem (there is no arm, only a control point
at a fixed offset from the target) and is the mode this repo's Phase 4
validation actually relies on; treat `bag` mode as a secondary, real-data
plumbing check to enable once a working placement for a given bag is found.

USAGE
-----
    python3 scripts/compare_range_noise.py synthetic --sigma-d-px 0.20
    python3 scripts/compare_range_noise.py bag --bag rosbag/arm_complex \\
        --robot-config config/fr3_complete.yaml \\
        --extrinsics config/camera_extrinsics.yaml \\
        --inject-sigma-d-px 0.20

``synthetic`` needs only numpy. ``bag`` needs a sourced ROS 2 environment,
like scripts/compare_vobs.py.
"""
from __future__ import annotations

import argparse
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
sys.path.insert(0, HERE)

from franka_experiments.utils.cbf_state_rows import sensor_range_uncertainty  # noqa: E402
from franka_experiments.utils.distance_engine import DistanceEngine  # noqa: E402
import range_noise_calibration as rnc  # noqa: E402

DEFAULT_Z_SWEEP = tuple(np.round(np.linspace(0.15, 1.0, 9), 3))


# ═════════════════════════════════════════════════════════════════════════════
#  synthetic — no ROS: DistanceEngine driven directly, known disparity noise
# ═════════════════════════════════════════════════════════════════════════════

# Synthetic camera, identity extrinsics — same convention as
# test_sensor_range_field.py, so a reader can compare the two directly.
_W = _H = 320
_FX = _FY = 428.8
_CX = _CY = 160.0
_STEP = 10

_R_EYE = np.eye(3, dtype=np.float32)
_T_ZERO = np.zeros(3, dtype=np.float32)


def _noisy_single_pixel_depth(z_true, f_px, baseline_m, sigma_d_px, rng):
    """A depth frame with EXACTLY ONE valid pixel (image centre), at
    ``z_true`` plus disparity noise applied through the EXACT
    (non-linearised) relation ``z = f*B/d`` — see the module docstring for
    why this, not additive Gaussian noise directly on z, is the physically
    honest way to inject a KNOWN sigma_d_px.

    ONE pixel, not a wall filling the frame, is deliberate: DistanceEngine
    reports the ARGMIN over every candidate pixel, so painting independent
    noise across an N-pixel target would make the reported range the
    MINIMUM of N noisy draws — a tighter, left-skewed order statistic, not a
    fair sample of one pixel's own noise. A single candidate pixel removes
    that selection effect and isolates exactly the quantity
    sensor_range_uncertainty models: one reading's own std-dev. (A REAL,
    spatially-broad obstacle — a wall, a torso — DOES suffer the
    argmin-order-statistic effect in the live pipeline; that is a separate,
    genuine finding, not something this synthetic check is measuring.)

    ``baseline_m`` MUST be the same value the caller later fits/predicts
    against — it does NOT cancel out of anything here; an earlier version of
    this function fixed it at 1.0 "for cancellation" and that was simply
    wrong, off by exactly a factor of ``baseline_m`` (caught by the
    recover-the-injected-constant self-test failing at a suspiciously
    baseline_m-shaped ratio).
    """
    d_true = (f_px * baseline_m) / z_true
    d_noisy = max(float(d_true + rng.normal(0.0, sigma_d_px)), 1e-6)
    z_noisy = (f_px * baseline_m) / d_noisy
    depth = np.zeros((_H, _W), dtype=np.uint16)
    depth[int(_CY), int(_CX)] = int(np.clip(z_noisy * 1000.0, 0, 65535))
    return depth


def synthetic_noise_sweep(z_values, *, f_px=_FX, baseline_m=0.095,
                          sigma_d_px_true=0.20, n_frames=200, seed=0,
                          radius=0.02):
    """Sweep ``z_values``, injecting a KNOWN ``sigma_d_px_true`` at each, and
    return one record per distance: ``dict(distance_truth_m, mean_z_m,
    std_z_m, n_samples, f_px)`` — the exact shape
    ``range_noise_calibration.fit_sigma_d``/``cmd_fit`` consume.

    ``baseline_m`` only rescales the disparity axis (see ``_noisy_wall_depth``)
    and cancels exactly in ``z = f*B/d``, so it does not change the injected
    z-noise at all; it is threaded through purely so the reported records use
    the SAME baseline_m the caller will fit sigma_d_px against.
    """
    rng = np.random.default_rng(seed)
    engine = DistanceEngine({'min_depth_m': 0.05, 'max_depth_m': 4.0, 'lpf_alpha': 0.0})
    cp = {'point': np.array([0.0, 0.0, 0.0], dtype=np.float64), 'seg_idx': 0,
          'cp_idx': 0, 'radius': radius, 'start_link': 'a', 'end_link': 'b'}
    records = []
    for z in z_values:
        z = float(z)
        # The CP sits AT the camera-frame origin's Z=0 plane, radius near
        # zero, so `distance` (surface gap) tracks `range_m` (raw Z) to
        # within `radius`; what this sweep cares about is range_m, not the
        # gap, but placing the CP a hair off the wall keeps min_dist finite.
        samples = np.empty(n_frames)
        for k in range(n_frames):
            depth = _noisy_single_pixel_depth(z, f_px, baseline_m, sigma_d_px_true, rng)
            results, _ = engine.compute(
                depth=depth, cx_f32=np.float32(_CX), cy_f32=np.float32(_CY),
                fx_inv_f32=np.float32(1.0 / f_px), fy_inv_f32=np.float32(1.0 / f_px),
                R_base_f32=_R_EYE, t_base_f32=_T_ZERO, control_points=[cp],
                x=np.array([0, _W]), y=np.array([0, _H]), step=_STEP,
                search_exclusion_mask=None)
            r = results[0]
            assert r.range_m is not None, f'no obstacle pixel at z={z}'
            samples[k] = r.range_m
        records.append(dict(distance_truth_m=z, mean_z_m=float(np.mean(samples)),
                            std_z_m=float(np.std(samples)), n_samples=n_frames,
                            f_px=float(f_px)))
    return records


def cmd_synthetic(args):
    print(f'injecting sigma_d_px={args.sigma_d_px:.4f} across z = '
          f'{args.z_min:.2f}..{args.z_max:.2f} m ({args.n_distances} points, '
          f'{args.n_frames} frames/point) ...', flush=True)
    z_values = np.linspace(args.z_min, args.z_max, args.n_distances)
    records = synthetic_noise_sweep(
        z_values, f_px=args.f_px, baseline_m=args.baseline_m,
        sigma_d_px_true=args.sigma_d_px, n_frames=args.n_frames, seed=args.seed)

    z = np.array([r['distance_truth_m'] for r in records])
    sigma_z_actual = np.array([r['std_z_m'] for r in records])
    fitted_sigma_d, r2 = rnc.fit_sigma_d(z, sigma_z_actual, args.f_px, args.baseline_m)

    print()
    header = (f'{"z_true [m]":>12s}  {"std_z actual":>14s}  '
             f'{"std_z model(true sigma_d)":>24s}  {"ratio":>8s}')
    print(header)
    for zi, sa in zip(z, sigma_z_actual):
        sp = (zi ** 2) / (args.f_px * args.baseline_m) * args.sigma_d_px
        print(f'{zi:>12.3f}  {sa:>14.6f}  {sp:>24.6f}  {sa / sp if sp > 0 else float("nan"):>8.3f}')

    print()
    print(f'injected sigma_d_px  = {args.sigma_d_px:.4f} px')
    print(f'fitted   sigma_d_px  = {fitted_sigma_d:.4f} px   (R^2 = {r2:.4f})')
    rel_err = abs(fitted_sigma_d - args.sigma_d_px) / args.sigma_d_px
    print(f'relative error       = {rel_err * 100:.2f} %')
    if r2 < 0.9 or rel_err > 0.15:
        print('WARNING: the fit does not recover the injected constant well. '
              'This means the CODE PATH (DistanceEngine -> range_m -> '
              'fit_sigma_d) has a bug, or n_frames is too small for the '
              'noise level tested -- investigate before trusting a live '
              'calibration run.')
    else:
        print('The quadratic law, the DistanceEngine code path and '
              'fit_sigma_d agree with each other on known-noise synthetic '
              'data. This does NOT measure the real sensor.')

    # sensor_range_uncertainty itself, sanity-checked against the same sweep:
    # the margin it would produce at k_sigma=1 should track the ACTUAL std,
    # not the (also fitted) model -- the two only coincide if the law holds.
    print()
    print('sensor_range_uncertainty(z, k_sigma=1) vs actual std_z (k_sigma=1 '
          'is a 1-sigma margin, directly comparable to a std-dev):')
    for zi, sa in zip(z, sigma_z_actual):
        m = sensor_range_uncertainty(zi, f_px=args.f_px, baseline_m=args.baseline_m,
                                     sigma_d_px=fitted_sigma_d, k_sigma=1.0,
                                     margin_max=10.0)
        print(f'  z={zi:.3f}  margin(fitted)={m:.6f}  actual_std={sa:.6f}')


# ═════════════════════════════════════════════════════════════════════════════
#  bag — real replay, InjectedSphere ground truth (reuses compare_vobs.py)
# ═════════════════════════════════════════════════════════════════════════════

def _sphere_ray_z(u, v, c, radius, K):
    """Exact analytic ray-sphere intersection depth at pixel (u, v) — the
    SAME formula InjectedSphere.render uses per-pixel, evaluated at one
    pixel instead of a whole patch, for the GROUND TRUTH comparison. `c` is
    the sphere centre in camera frame at this instant; K the 3x3 intrinsic.
    """
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    d = np.array([(u - cx) / fx, (v - cy) / fy, 1.0])
    A = float(d @ d)
    B = -2.0 * float(d @ c)
    C = float(c @ c) - radius ** 2
    disc = B * B - 4.0 * A * C
    if disc < 0.0:
        return None
    s = (-B - np.sqrt(disc)) / (2.0 * A)
    return float(s) if s > 0.0 else None


def cmd_bag(args):
    import compare_vobs as cv                      # bag reading, TF, InjectedSphere

    log = cv._Log(args.verbose)
    from franka_experiments.utils.distance_utils import (
        compute_roi, define_control_points, load_extrinsics, load_robot_config)
    from franka_experiments.utils.kinematics import build_urdf_no_hand
    from franka_experiments.utils.mask_builder import MaskBuilder
    from franka_experiments.utils.tf_manager import TFManager
    import pinocchio as pin
    import trimesh
    from ament_index_python.packages import get_package_share_directory
    from cv_bridge import CvBridge

    cfg = load_robot_config(args.robot_config)
    robot_cfg, mask_cfg, mesh_cfg = cfg['robot'], cfg['mask'], cfg['meshes']
    distance_cfg = dict(cfg['distance'])

    R_base, t_base = load_extrinsics(args.extrinsics)
    R32, t32 = R_base.astype(np.float32), t_base.astype(np.float32)

    tf_buf, n_tf = cv.build_tf_buffer(args.bag)
    ee_link = robot_cfg.get('ee_link', 'fr3_link8')
    tf_mgr = TFManager(tf_buffer=tf_buf, base_frame=robot_cfg['base_frame'],
                       critical_links=robot_cfg.get('critical_links', [ee_link]),
                       cache_max_age_s=None, logger=log)

    model = pin.buildModelFromUrdf(build_urdf_no_hand())
    joints = cv.JointTrace(model, [f'fr3_joint{i}' for i in range(1, 8)])

    mesh_dir = get_package_share_directory(mesh_cfg.get('package', 'franka_description'))
    samples = {n: trimesh.load(os.path.join(mesh_dir, rel), force='mesh')
               .sample(int(mesh_cfg.get('sample_points_per_link', 300)))
               for n, rel in mesh_cfg['files'].items()}
    mask_builder = MaskBuilder(link_mesh_samples=samples, R_base=R_base,
                               t_base=t_base, ee_link=ee_link,
                               mask_cfg=mask_cfg, logger=log)
    engine = DistanceEngine(distance_cfg=distance_cfg, logger=log)
    bridge = CvBridge()

    sphere = cv.InjectedSphere(args.inject_start, args.inject_vel,
                               radius=args.inject_radius, period=args.inject_period,
                               amplitude=args.inject_amplitude)
    rng = np.random.default_rng(args.seed)
    print(f'reading TF from {args.bag} ({n_tf} transforms) ...', flush=True)

    K = None
    for topic, _, msg in cv.read_bag(args.bag, {args.joint_topic, args.info_topic}):
        if topic == args.joint_topic:
            s = msg.header.stamp
            joints.add(s.sec + s.nanosec * 1e-9, msg)
        elif K is None:
            K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
    joints.finish()
    if K is None:
        raise SystemExit(f'no CameraInfo on {args.info_topic}')
    mask_builder.set_intrinsics(K)
    cx32, cy32 = np.float32(K[0, 2]), np.float32(K[1, 2])
    fxi32, fyi32 = np.float32(1.0 / K[0, 0]), np.float32(1.0 / K[1, 1])
    step = int(distance_cfg['pixel_step'])
    margin = int(distance_cfg['image_margin_px'])

    z_true_all, err_all, n_frames = [], [], 0
    prev_shape = None
    for topic, _, msg in cv.read_bag(args.bag, {args.depth_topic}):
        n_frames += 1
        if args.max_frames and n_frames > args.max_frames:
            break
        depth = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        stamp = msg.header.stamp
        t_cap = stamp.sec + stamp.nanosec * 1e-9
        depth = depth.copy()
        sphere.render(depth, t_cap, K)
        if args.inject_sigma_d_px > 0.0:
            _add_disparity_noise(depth, K[0, 0], args.baseline_m,
                                 args.inject_sigma_d_px, rng)

        if prev_shape != depth.shape:
            prev_shape = depth.shape
            mask_builder.invalidate(); engine.invalidate_grid_cache(); engine.reset_lpf()
        qv = joints.at(t_cap)
        if qv is None:
            continue
        q, qdot = qv
        transforms = tf_mgr.lookup_all(robot_cfg['segment_links'], stamp)
        if not transforms:
            continue
        cps = define_control_points(transforms, robot_cfg, distance_cfg)
        if not cps:
            continue
        H, W = depth.shape
        mask_builder.rebuild(transforms, depth.shape)
        roi = compute_roi(mask_builder.search_exclusion_mask, H, W, margin,
                          int(distance_cfg['roi_pad_px'])) or \
            (margin, margin, W - margin, H - margin)
        x0, y0, x1, y1 = roi
        cp_results, _ = engine.compute(
            depth=depth, cx_f32=cx32, cy_f32=cy32, fx_inv_f32=fxi32, fy_inv_f32=fyi32,
            R_base_f32=R32, t_base_f32=t32, control_points=cps,
            x=np.array([x0, x1]), y=np.array([y0, y1]), step=step,
            search_exclusion_mask=mask_builder.search_exclusion_mask,
            frame_stamp=t_cap)
        if cp_results is None:
            continue

        c_now = sphere.centre(t_cap)
        for r in cp_results:
            if r.range_m is None or r.closest_pixel is None:
                continue
            u, v = r.closest_pixel
            obs_cam = R_base.T @ (np.asarray(r.closest_obstacle_point,
                                             dtype=np.float64) - t_base)
            on_sphere = np.linalg.norm(obs_cam - c_now) < sphere.radius + 0.05
            if not on_sphere:
                continue
            z_true = _sphere_ray_z(u, v, c_now, sphere.radius, K)
            if z_true is None:
                continue
            z_true_all.append(z_true)
            err_all.append(r.range_m - z_true)

    print(f'{n_frames} depth frames, {len(z_true_all)} on-sphere samples')
    if len(z_true_all) < 20:
        raise SystemExit('too few on-sphere samples to report anything — '
                         'check --inject-start / the bag actually has a '
                         'control point looking that way')

    z_true_all = np.array(z_true_all)
    err_all = np.array(err_all)
    bins = np.linspace(z_true_all.min(), z_true_all.max(), args.n_bins + 1)
    idx = np.digitize(z_true_all, bins) - 1
    print()
    print(f'{"z bin [m]":>16s}  {"n":>6s}  {"mean err":>10s}  {"std err":>10s}')
    zc, stds = [], []
    for b in range(args.n_bins):
        m = idx == b
        if m.sum() < 5:
            continue
        zc.append(0.5 * (bins[b] + bins[b + 1]))
        stds.append(float(np.std(err_all[m])))
        print(f'{bins[b]:.2f}-{bins[b + 1]:.2f}  {m.sum():>6d}  '
              f'{np.mean(err_all[m]):>10.5f}  {stds[-1]:>10.5f}')

    if len(zc) >= 2:
        fitted, r2 = rnc.fit_sigma_d(np.array(zc), np.array(stds),
                                     K[0, 0], args.baseline_m)
        print(f'\nfitted sigma_d_px from bag replay = {fitted:.4f} px  (R^2={r2:.4f})')
        if args.inject_sigma_d_px > 0.0:
            print(f'injected sigma_d_px               = {args.inject_sigma_d_px:.4f} px')
    else:
        print('\nnot enough distinct bins for a fit — widen the sphere sweep '
              'or lower --n-bins')


def _add_disparity_noise(depth, f_px, baseline_m, sigma_d_px, rng):
    """In place: perturb every nonzero pixel of `depth` (uint16 mm) through
    the disparity-space model, same physics as _noisy_single_pixel_depth.
    Uses the REAL baseline_m — see that function's docstring for why a
    placeholder value here would silently rescale the injected noise."""
    z = depth.astype(np.float64) * 0.001
    hit = z > 0.0
    if not hit.any():
        return
    d = (f_px * baseline_m) / z[hit]
    d_noisy = np.maximum(d + rng.normal(0.0, sigma_d_px, size=d.shape), 1e-6)
    z_noisy = (f_px * baseline_m) / d_noisy
    depth[hit] = np.clip(z_noisy * 1000.0, 0, 65535).astype(depth.dtype)


# ═════════════════════════════════════════════════════════════════════════════

def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='cmd', required=True)

    p = sub.add_parser('synthetic')
    p.add_argument('--sigma-d-px', type=float, default=0.20)
    p.add_argument('--f-px', type=float, default=_FX)
    p.add_argument('--baseline-m', type=float, default=0.095)
    p.add_argument('--z-min', type=float, default=0.15)
    p.add_argument('--z-max', type=float, default=1.0)
    p.add_argument('--n-distances', type=int, default=9)
    p.add_argument('--n-frames', type=int, default=200)
    p.add_argument('--seed', type=int, default=0)
    p.set_defaults(fn=cmd_synthetic)

    p = sub.add_parser('bag')
    p.add_argument('--bag', required=True)
    p.add_argument('--robot-config', default=os.path.join(PKG, 'config', 'fr3_complete.yaml'))
    p.add_argument('--extrinsics', default=os.path.join(PKG, 'config', 'camera_extrinsics.yaml'))
    p.add_argument('--depth-topic', default='/camera/camera/aligned_depth_to_color/image_raw')
    p.add_argument('--info-topic', default='/camera/camera/aligned_depth_to_color/camera_info')
    p.add_argument('--joint-topic', default='/NS_1/joint_states')
    p.add_argument('--baseline-m', type=float, default=0.095)
    p.add_argument('--max-frames', type=int, default=0)
    p.add_argument('--n-bins', type=int, default=8)
    p.add_argument('--seed', type=int, default=0)
    p.add_argument('--inject-start', type=float, nargs=3, default=(0.0, 0.0, 0.9))
    p.add_argument('--inject-vel', type=float, nargs=3, default=(0.0, 0.0, -0.3))
    p.add_argument('--inject-radius', type=float, default=0.15)
    p.add_argument('--inject-period', type=float, default=4.0)
    p.add_argument('--inject-amplitude', type=float, default=0.4)
    p.add_argument('--inject-sigma-d-px', type=float, default=0.0,
                    help='0 = no synthetic noise, i.e. plumbing-only validation '
                         'against a quantisation-limited noise floor')
    p.add_argument('--verbose', action='store_true')
    p.set_defaults(fn=cmd_bag)

    args = ap.parse_args()
    args.fn(args)


if __name__ == '__main__':
    main()
