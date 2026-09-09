"""Is the hand-eye calibration still the one this camera was calibrated with?

WHY THIS EXISTS
---------------
The whole distance pipeline is built on one assumption: that projecting the
robot's link meshes through ``camera_extrinsics.yaml`` lands them on the robot's
actual pixels. Every downstream number inherits it. If the extrinsic is stale,
the mask is subtracted from the WRONG PLACE, so the arm is not removed — and
since "obstacle" is defined as "not robot", the arm becomes its own obstacle.
The filter then brakes, retreats and (with the tracker) assigns a velocity to
the robot's own body, all with complete confidence and no error anywhere.

Nothing in the pipeline notices, because nothing in it ever compares the model
against the measurement. That is what this module does, and it is one
subtraction:

    residual = depth_measured(u, v) − depth_model(u, v)

over the pixels the robot model covers. Where the calibration is right the model
surface IS the measured surface and the residual is a couple of centimetres of
mesh-sampling and depth noise. Where it is wrong the model sits in front of or
behind the real arm and the residual is the calibration error, in metres, with a
sign.

WHY A MEDIAN, AND WHY OVER THE MODEL'S FRONT SURFACE
-----------------------------------------------------
The link meshes are sampled as point clouds (``sample_points_per_link``), so a
link contributes points on the side facing the camera AND on the side facing
away. A depth camera only ever sees the front, so back-facing samples have a
model depth LARGER than anything measurable and would drag any mean toward a
large negative residual whatever the calibration.

So the model depth at a pixel is the MINIMUM over the samples that land on it —
the model's own front surface — and the statistic over pixels is the MEDIAN,
which survives the pixels where the arm is genuinely occluded by something in
front of it (a real obstacle, a hand) without needing to know which those are.

IT IS A DRIFT DETECTOR, NOT AN ABSOLUTE VERDICT — AND THAT IS MEASURED
----------------------------------------------------------------------
The absolute value of the residual is NOT trustworthy, and pretending otherwise
would be worse than not shipping the check at all. The reason is the model side
of the subtraction: the links are represented by ``sample_points_per_link``
points scattered over each mesh, so the "model front surface" at a pixel is the
nearest of however many samples happened to land there — which is biased FARTHER
than the true surface, by an amount that depends on the sampling density and not
on the calibration.

Measured on ``rosbag/arm_complex``, one scene, one extrinsic, varying only the
sample count:

    samples/link      median residual      spread (MAD)
        300              −0.9 cm              10.2 cm
       3000              +1.4 cm               9.1 cm
      20000              +6.0 cm               7.2 cm

Seven centimetres of swing from a number that has nothing to do with the
calibration. And the three successive ``camera_extrinsics.yaml`` in this repo's
history all score within 1 cm of each other on the same bag, so the statistic
does not separate them either.

What it DOES do reliably is detect CHANGE. Run it right after a calibration you
trust, record the number, and compare against it thereafter: the sampling bias
is identical between the two runs and cancels, so a shift means the geometry
moved. That is the failure this is for — a camera knocked, a mount loosened, an
extrinsic file that no longer matches the rig — and it is the one an operator
cannot see. ``baseline_m`` is where that recorded number goes; with no baseline
the check reports and judges nothing.

It also cannot separate a bad extrinsic from a bad URDF, a wrong depth scale, or
a camera that has been moved. It says the model and the world disagree and by
how much they have MOVED APART since you last agreed they did not.

Pure numpy, no ROS.
"""

from __future__ import annotations

from typing import Dict, NamedTuple, Optional

import numpy as np


class CalibrationResidual(NamedTuple):
    """Outcome of one check.

    Attributes:
        median_m: [m] median of ``measured − model`` over covered pixels.
            POSITIVE means the measured surface is FARTHER than the model, i.e.
            the model sits in front of the real arm. NaN when nothing was
            covered.
        mad_m: [m] median absolute deviation of the same residual — the spread.
            A large median with a small spread is a rigid offset (a stale
            extrinsic); a large spread is a shape/scale problem instead.
        n_pixels: how many pixels carried both a model depth and a valid
            measurement. Small counts make the median meaningless, so the
            caller has to look at this before believing the rest.
        coverage: fraction of the model's projected pixels that had a valid
            depth measurement. A low value is itself a symptom: the model is
            projecting somewhere the sensor returns nothing.
    """

    median_m: float
    mad_m: float
    n_pixels: int
    coverage: float

    def is_suspicious(self, baseline_m: Optional[float] = None,
                      tol_m: float = 0.03) -> bool:
        """Has the residual DRIFTED from a baseline you recorded when it was good?

        Returns ``False`` when no baseline is given — the absolute value carries
        a sampling bias of several centimetres (see the module docstring), so
        judging it against zero would fire on a perfectly good calibration and
        the warning would rightly be ignored.
        """
        if baseline_m is None or self.n_pixels < 50 or not np.isfinite(self.median_m):
            return False
        return abs(self.median_m - float(baseline_m)) > float(tol_m)

    def describe(self, baseline_m: Optional[float] = None,
                 tol_m: float = 0.03) -> str:
        if self.n_pixels < 50:
            return (f'calibration check inconclusive: only {self.n_pixels} '
                    f'covered pixels (coverage {self.coverage:.0%})')
        head = (f'calibration residual measured − model = '
                f'{self.median_m * 100:+.1f} cm (spread {self.mad_m * 100:.1f} '
                f'cm) over {self.n_pixels} px, coverage {self.coverage:.0%}')
        if baseline_m is None:
            return (head + ' — no baseline set, so this is a reading and not a '
                    'verdict. Record it as tracking.calibration_check.'
                    'baseline_m right after a calibration you trust; a later '
                    'SHIFT from it is what means the geometry moved.')
        d = self.median_m - float(baseline_m)
        if self.is_suspicious(baseline_m, tol_m):
            return (head + f' — DRIFTED {d * 100:+.1f} cm from the baseline. '
                    'The robot mask is being subtracted from the wrong place, '
                    'so the arm is partly reading as its own obstacle. '
                    'Re-run the hand-eye calibration.')
        return head + f' — {d * 100:+.1f} cm from baseline, within tolerance.'


def calibration_residual(
    link_samples_base: Dict[str, np.ndarray],
    R_base: np.ndarray,
    t_base: np.ndarray,
    K: np.ndarray,
    depth_m: np.ndarray,
    *,
    min_depth: float = 0.15,
    max_depth: float = 4.0,
    step: int = 4,
) -> CalibrationResidual:
    """Compare the projected robot model against the measured depth.

    Args:
        link_samples_base: link name → (M, 3) mesh sample points already in BASE
            frame (i.e. the same points ``MaskBuilder`` projects, after their
            link transform). Values are concatenated; the split by link is
            irrelevant here and only kept so the caller can pass its own dict.
        R_base / t_base: camera→base extrinsic, the same pair the rest of the
            pipeline uses. ``p_cam = R_baseᵀ (p_base − t_base)``.
        K: (3, 3) depth intrinsics.
        depth_m: (H, W) depth in METRES. Note metres, not the raw millimetre
            uint16 — the caller converts, because the scale factor is the
            caller's business and getting it wrong here would look exactly like
            a calibration error.
        min_depth / max_depth: validity band for a measurement, matching the
            distance engine's own.
        step: subsample the model points by this factor. The check runs at most
            once a second, and a few thousand samples already pin a median.

    Returns:
        A :class:`CalibrationResidual`.
    """
    pts = [np.asarray(v, dtype=np.float64) for v in link_samples_base.values()
           if v is not None and len(v)]
    if not pts or depth_m is None:
        return CalibrationResidual(float('nan'), float('nan'), 0, 0.0)
    P = np.concatenate(pts)[::max(1, int(step))]

    R = np.asarray(R_base, dtype=np.float64)
    t = np.asarray(t_base, dtype=np.float64).ravel()
    p_cam = (R.T @ (P - t).T).T
    z = p_cam[:, 2]
    ok = z > 1e-3
    if not np.any(ok):
        return CalibrationResidual(float('nan'), float('nan'), 0, 0.0)
    p_cam, z = p_cam[ok], z[ok]

    K = np.asarray(K, dtype=np.float64)
    u = np.rint(K[0, 0] * p_cam[:, 0] / z + K[0, 2]).astype(np.int64)
    v = np.rint(K[1, 1] * p_cam[:, 1] / z + K[1, 2]).astype(np.int64)
    H, W = depth_m.shape
    inside = (u >= 0) & (u < W) & (v >= 0) & (v < H)
    u, v, z = u[inside], v[inside], z[inside]
    if u.size == 0:
        return CalibrationResidual(float('nan'), float('nan'), 0, 0.0)

    # Model FRONT surface per pixel: the minimum model depth among the samples
    # landing on it. np.minimum.at rather than a sort — the arrays are small and
    # this keeps the intent visible.
    flat = v * W + u
    model = np.full(H * W, np.inf)
    np.minimum.at(model, flat, z)

    covered = np.flatnonzero(np.isfinite(model))
    meas = depth_m.reshape(-1)[covered]
    valid = (meas >= min_depth) & (meas <= max_depth)
    n_cov = int(covered.size)
    if not np.any(valid):
        return CalibrationResidual(float('nan'), float('nan'), 0,
                                   0.0 if n_cov == 0 else 0.0)

    res = meas[valid] - model[covered][valid]
    med = float(np.median(res))
    mad = float(np.median(np.abs(res - med)))
    return CalibrationResidual(med, mad, int(valid.sum()),
                               float(valid.sum()) / max(1, n_cov))
