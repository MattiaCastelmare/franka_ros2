"""Draw the two EE traces on the camera image: project, fade, composite.

Used by ``nodes/trajectory_overlay_node``. Same split as
``utils/trajectory_trace``: everything here is a function of numbers and pixels,
so the parts that can be wrong — a point behind the camera drawn as if it were
in front, a trail that fades on the wrong end, a segment whose visible piece
moves when it is clipped — are testable without a camera or a running graph.

WHAT THE OVERLAY IS FOR
-----------------------
Red is ``P(s)``, the point the commander is asking for. Blue is where the end
effector actually is. Both come from the same commander tick and the same
forward kinematics, in ``fr3_link0``; this module only moves them into the
camera and onto the pixels, using the SAME extrinsic calibration
(``camera_extrinsics.yaml``) that ``real_time_distance`` projects the robot mask
with. That is deliberate: if the red dot does not sit on the gripper in the
image, the calibration the safety pipeline depends on is the thing that is
wrong, and this overlay is where you would see it first.

WHY A TRAIL AND NOT A PATH
--------------------------
Each vertex carries a timestamp and dies of old age (``TraceBuffer.expire``),
and what survives is drawn with an alpha that decays toward the tail. Two short
comets chasing each other say "the blue one is behind the red one, by this
much, right now" — which is the tracking question — where two curves that grow
for the whole run say only "the arm went roughly there", and after a minute say
nothing at all because they cover the robot.

THE PIXEL SKEW NOBODY CAN FIX HERE
----------------------------------
The image is 30-60 ms older than the EE points drawn on it (camera exposure,
driver, transport), so both dots lead the arm you see by that much: about 5 mm
at 0.1 m/s, in the SAME direction for both. It offsets the pair against the
picture, never red against blue, so the gap between them — the number this
overlay exists to show — stays honest.
"""

from __future__ import annotations

from typing import List, Optional, Sequence, Tuple

import numpy as np

#: Coordinates beyond this are clamped before they reach OpenCV. A point near
#: the vanishing line projects to millions of pixels, and int32 fixed-point
#: drawing overflows long before that; clamping bends only the part that is
#: offscreen anyway, where the visible crossing is unchanged.
_COORD_LIMIT = 1.0e6


def project_base_to_pixels(
    P_base: Sequence[Sequence[float]],
    R_base: np.ndarray,
    t_base: np.ndarray,
    K: np.ndarray,
    D: Optional[Sequence[float]] = None,
    *,
    min_z: float = 1.0e-3,
) -> Tuple[np.ndarray, np.ndarray]:
    """Project base-frame points into the colour image.

    Args:
        P_base: (N, 3) points in the robot base frame.
        R_base / t_base: the camera→base extrinsic as the rest of the pipeline
            stores it (``p_base = R_base · p_cam + t_base``), so the transform
            applied here is its inverse: ``p_cam = R_baseᵀ (p_base − t_base)``.
        K: (3, 3) colour intrinsics.
        D: distortion coefficients from the same ``CameraInfo``
            (``k1, k2, p1, p2, k3``, extras ignored). ``None`` or all-zero skips
            the model entirely — the RealSense colour stream usually reports
            zeros, and paying for a no-op on every frame is silly.
        min_z: a point must be at least this far IN FRONT of the camera. Behind
            it the pinhole equations still return a pixel — the mirrored one —
            which is how a trace ends up drawn across a frame the arm was never
            in.

    Returns:
        ``(uv, valid)`` with ``uv`` of shape (N, 2), float. Entries where
        ``valid`` is False hold NaN and must not be drawn.
    """
    P = np.asarray(P_base, dtype=np.float64).reshape(-1, 3)
    R = np.asarray(R_base, dtype=np.float64)
    t = np.asarray(t_base, dtype=np.float64).ravel()
    K = np.asarray(K, dtype=np.float64)

    uv = np.full((P.shape[0], 2), np.nan)
    if P.shape[0] == 0:
        return uv, np.zeros(0, dtype=bool)

    with np.errstate(invalid='ignore'):
        # A NaN in P is a refusal, not an event: the caller gets `valid=False`
        # for that row. Letting numpy warn about it would put a traceback-shaped
        # line in the node's log for a point that was correctly ignored.
        p_cam = (R.T @ (P - t).T).T
    valid = np.isfinite(p_cam).all(axis=1) & (p_cam[:, 2] > float(min_z))
    if not np.any(valid):
        return uv, valid

    z = p_cam[valid, 2]
    xn = p_cam[valid, 0] / z
    yn = p_cam[valid, 1] / z
    if D is not None:
        xn, yn = _distort(xn, yn, D)
    uv[valid, 0] = K[0, 0] * xn + K[0, 1] * yn + K[0, 2]
    uv[valid, 1] = K[1, 1] * yn + K[1, 2]
    # A distortion model evaluated far outside its calibrated field can return
    # inf; treat that as "not projectable" rather than letting NaN reach a draw.
    finite = np.isfinite(uv).all(axis=1)
    valid &= finite
    uv[~valid] = np.nan
    return uv, valid


def _distort(xn: np.ndarray, yn: np.ndarray,
             D: Sequence[float]) -> Tuple[np.ndarray, np.ndarray]:
    """Brown-Conrady (``plumb_bob``) on normalised coordinates."""
    d = np.asarray(D, dtype=np.float64).ravel()
    if d.size == 0 or not np.any(d):
        return xn, yn
    k = np.zeros(5)
    k[:min(5, d.size)] = d[:5]
    k1, k2, p1, p2, k3 = k
    r2 = xn * xn + yn * yn
    radial = 1.0 + r2 * (k1 + r2 * (k2 + r2 * k3))
    x = xn * radial + 2.0 * p1 * xn * yn + p2 * (r2 + 2.0 * xn * xn)
    y = yn * radial + p1 * (r2 + 2.0 * yn * yn) + 2.0 * p2 * xn * yn
    return x, y


def trail_alphas(times: Sequence[float], now: float, ttl: float, *,
                 head_hold: float = 0.25) -> np.ndarray:
    """Opacity per vertex: 1 at the tip, 0 at ``ttl`` seconds old.

    Args:
        times: vertex timestamps, oldest first.
        now: the instant the frame is being drawn for.
        ttl: trail length in seconds. Non-positive means no fade at all — the
            caller has disabled expiry too, and a trail that never dies must not
            be drawn invisible.
        head_hold: fraction of ``ttl`` the newest vertices keep full opacity.
            Without it the tip itself is already a shade below solid and the
            whole comet reads as washed out; with it there is a bright head and
            a fading body, which is what makes the direction of travel obvious.

    Returns:
        (N,) float array in [0, 1].
    """
    t = np.asarray(times, dtype=np.float64).ravel()
    if t.size == 0:
        return np.zeros(0)
    if ttl <= 0.0:
        return np.ones(t.size)
    age = float(now) - t
    hold = float(np.clip(head_hold, 0.0, 0.99)) * ttl
    a = np.where(age <= hold, 1.0, (ttl - age) / max(ttl - hold, 1e-9))
    return np.clip(a, 0.0, 1.0)


class PixelBounds:
    """The rectangle actually drawn into, so compositing stays local.

    A full-frame alpha blend is a few milliseconds of pure memory traffic at
    720p, thirty times a second, to composite a curve that covers a few hundred
    pixels. Tracking the bounding box while drawing costs nothing and makes the
    blend proportional to the trail instead of to the camera.
    """

    __slots__ = ('x0', 'y0', 'x1', 'y1')

    def __init__(self) -> None:
        self.x0 = self.y0 = self.x1 = self.y1 = 0
        self.x1 = -1                       # empty sentinel: x1 < x0

    @property
    def empty(self) -> bool:
        return self.x1 < self.x0 or self.y1 < self.y0

    def add(self, x: int, y: int) -> None:
        if self.empty:
            self.x0 = self.x1 = int(x)
            self.y0 = self.y1 = int(y)
            return
        self.x0 = min(self.x0, int(x))
        self.x1 = max(self.x1, int(x))
        self.y0 = min(self.y0, int(y))
        self.y1 = max(self.y1, int(y))

    def roi(self, margin: int, width: int,
            height: int) -> Optional[Tuple[int, int, int, int]]:
        """``(x0, y0, x1, y1)`` grown by *margin* and clipped, or None."""
        if self.empty:
            return None
        x0 = max(0, self.x0 - margin)
        y0 = max(0, self.y0 - margin)
        x1 = min(width - 1, self.x1 + margin)
        y1 = min(height - 1, self.y1 + margin)
        if x1 < x0 or y1 < y0:
            return None
        return x0, y0, x1, y1


def fading_runs(uv: np.ndarray, valid: np.ndarray, alphas: np.ndarray,
                levels: int = 8) -> List[Tuple[np.ndarray, float]]:
    """Split a polyline into constant-opacity runs, ready for ``polylines``.

    One OpenCV call per segment would be ~1600 calls a frame for two full
    trails; quantising the fade into *levels* steps collapses that to a handful,
    and eight steps are already finer than the eye resolves on a 2 px line.

    Runs overlap by one vertex on purpose: shared endpoints keep the curve
    continuous across a change of opacity instead of leaving a gap the width of
    the line at every step.

    Invalid vertices (behind the camera, unprojectable) END a run rather than
    being skipped over — joining across one is how a trace gets drawn along a
    path the arm never took.
    """
    n = int(np.asarray(valid).size)
    if n < 2:
        return []
    lv = max(1, int(levels))
    # Quantise DOWN so that only a true 1.0 reaches the top bucket: with
    # rounding, everything above the last half-step would be indistinguishable
    # from the tip, and the bright head would be twice as long as asked.
    bucket = np.minimum((np.asarray(alphas, dtype=np.float64) * lv).astype(int),
                        lv - 1)
    out: List[Tuple[np.ndarray, float]] = []
    run: List[Tuple[float, float]] = []
    run_bucket = -1
    for i in range(n):
        if not valid[i]:
            _flush(out, run, run_bucket, lv)
            run, run_bucket = [], -1
            continue
        p = (float(uv[i, 0]), float(uv[i, 1]))
        b = int(bucket[i])
        if not run:
            run, run_bucket = [p], b
        elif b != run_bucket:
            run.append(p)                    # close the old run ON this vertex
            _flush(out, run, run_bucket, lv)
            run, run_bucket = [p], b         # and open the next one from it
        else:
            run.append(p)
    _flush(out, run, run_bucket, lv)
    return out


def _flush(out, run, bucket, levels) -> None:
    if len(run) < 2 or bucket < 0:
        return
    pts = np.clip(np.asarray(run, dtype=np.float64),
                  -_COORD_LIMIT, _COORD_LIMIT)
    # Mid-bucket opacity: the run covers [b/levels, (b+1)/levels), and its
    # centre is the value that is wrong by the least on both sides.
    out.append((np.rint(pts).astype(np.int32),
                (bucket + 0.5) / float(levels)))


def draw_fading_polyline(color_layer: np.ndarray, alpha_layer: np.ndarray,
                         uv: np.ndarray, valid: np.ndarray,
                         alphas: np.ndarray, color: Tuple[int, int, int],
                         thickness: int, bounds: PixelBounds,
                         *, levels: int = 8) -> int:
    """Draw one trace into the scratch layers. Returns segments drawn.

    ``color_layer`` carries the colour, ``alpha_layer`` (single channel, uint8)
    how much of it survives the blend. Two layers rather than one RGBA image
    because OpenCV's drawing functions ignore an alpha channel: the opacity has
    to be painted as a greyscale value and applied by hand in :func:`composite`.

    Later runs overwrite earlier ones, and runs arrive oldest-first, so where
    the trail crosses itself the NEWER, brighter pass is the one that shows.
    """
    h, w = alpha_layer.shape[:2]
    import cv2  # local: keeps the module importable in a test that has no cv2

    drawn = 0
    for pts, a in fading_runs(uv, valid, alphas, levels=levels):
        ok, clipped = _clip_run(pts, w, h)
        if not ok:
            continue
        cv2.polylines(color_layer, [pts], False, color, thickness, cv2.LINE_AA)
        cv2.polylines(alpha_layer, [pts], False, int(round(a * 255)),
                      thickness, cv2.LINE_AA)
        drawn += len(pts) - 1
        for x, y in clipped:
            bounds.add(x, y)
    return drawn


def _clip_run(pts: np.ndarray, width: int,
              height: int) -> Tuple[bool, List[Tuple[int, int]]]:
    """Is any of this run on screen, and which of its vertices are?

    Only the bounding box needs the answer — the drawing itself is left to
    OpenCV, which clips exactly. A run fully off one side contributes nothing
    and is skipped; a run that straddles the edge contributes its on-screen
    vertices plus the frame corner-clamped rest, which overstates the box by at
    most the frame itself.
    """
    inside = [(int(np.clip(x, 0, width - 1)), int(np.clip(y, 0, height - 1)))
              for x, y in pts]
    if (np.all(pts[:, 0] < 0) or np.all(pts[:, 0] >= width)
            or np.all(pts[:, 1] < 0) or np.all(pts[:, 1] >= height)):
        return False, []
    return True, inside


def composite(img: np.ndarray, color_layer: np.ndarray,
              alpha_layer: np.ndarray,
              roi: Optional[Tuple[int, int, int, int]]) -> None:
    """Blend the scratch layers into *img*, in place, over *roi* only."""
    if roi is None:
        return
    x0, y0, x1, y1 = roi
    dst = img[y0:y1 + 1, x0:x1 + 1]
    src = color_layer[y0:y1 + 1, x0:x1 + 1]
    a = alpha_layer[y0:y1 + 1, x0:x1 + 1].astype(np.float32) / 255.0
    a = a[:, :, None]
    np.copyto(dst, (dst.astype(np.float32) * (1.0 - a)
                    + src.astype(np.float32) * a).astype(np.uint8))
