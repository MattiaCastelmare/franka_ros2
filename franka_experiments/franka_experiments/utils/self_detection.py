"""Is this "obstacle" actually the robot's own body?

WHY THIS EXISTS
---------------
The distance pipeline finds obstacles by removing the robot from the depth
image: ``MaskBuilder`` projects the link meshes, dilates them, and everything
left over is an obstacle by definition. So the moment that removal is wrong,
the arm becomes its own obstacle — and the pipeline cannot tell, because "not
robot" is the only definition of obstacle it has.

It goes wrong in two ways, and both are ordinary rather than exotic:

* **mask leakage.** The dilation is finite and the depth is noisy, so a rim of
  real robot pixels survives at every silhouette edge. ``fr3_complete.yaml``
  already documents this — ``min_thresh`` exists specifically to "reject robot
  pixels that leak through the dilated exclusion mask, which read as a gap of
  ~0". That is a threshold, i.e. a guess, and it is one that trades a real
  contact-regime measurement for the rejection.
* **a stale or wrong hand-eye calibration.** Then the mask is projected in the
  WRONG PLACE, the arm is not removed at all, and a whole limb reads as an
  obstacle a few centimetres away. Observed on this repo's own bags, which
  predate the current ``camera_extrinsics.yaml``.

WHY IT MATTERS MORE NOW THAN IT DID
-----------------------------------
With the scalar residual, self-detection was mostly a nuisance: a spurious
barrier that the arm pushed against. The estimate ``aᵀq̇ − ḋ`` even partially
cancelled it, because a rigidly attached "obstacle" has ``ḋ ≈ 0`` and
``v_obs ≈ aᵀq̇``.

With a TRACKER it is a feedback loop. The robot's own moving arm becomes a
cluster with a genuine, large velocity; that velocity is fed back as ``v_obs``;
the barrier tightens because the arm is moving; the arm brakes; ``v_obs``
drops; it accelerates again. The filter ends up chasing itself, and with
lateral evasion on it would also step sideways away from its own elbow.

THE TEST, AND WHY IT IS CALIBRATION-INDEPENDENT
------------------------------------------------
Not geometry — geometry is exactly what is broken in the calibration case.
KINEMATICS:

    a point that is part of the robot moves WITH the robot.

Each control point reports its own position ``p_robot`` and its nearest
obstacle point ``p_human``, both in base frame, every frame. If the "obstacle"
is really the link itself, the OFFSET ``p_human − p_robot`` is rigid: it stays
put while ``p_robot`` sweeps through the workspace. If it is a real obstacle,
the offset changes as soon as either of them moves independently.

So the discriminator is: **``p_robot`` moved a lot AND the offset did not.**
A wrong extrinsic displaces both points by the same transform, so the offset is
displaced too — but it stays CONSTANT, which is the property being tested. That
is why this catches the miscalibrated case that a "is the point inside my
capsule model?" test cannot.

WHAT IT DOES WITH THE ANSWER — AND WHAT IT DOES NOT
----------------------------------------------------
It suppresses the TRACK (the velocity, the covariance, the evasion direction)
for that control point. It does NOT suppress the DISTANCE, and it must not: if
the judgement is wrong, the consequence is a barrier that falls back to
``v_obs = 0``, which is exactly the behaviour the filter has today. If it were
allowed to drop the distance, a wrong judgement would delete a barrier.

Same conservative asymmetry as everything else here: this can only ever remove
an estimate, never a constraint.

Pure numpy, no ROS.
"""

from __future__ import annotations

from typing import Dict, List, Optional, Tuple

import numpy as np


class SelfDetectionMonitor:
    """Per-control-point detector for "this obstacle is my own arm".

    Args:
        window: frames of history kept per control point. It has to span enough
            arm motion for the test to mean anything — at 30 Hz, 20 frames is
            0.67 s, in which a moving arm covers far more than ``motion_min_m``.
        motion_min_m: [m] how far ``p_robot`` must have travelled across the
            window before any judgement is made. Below it the test is vacuous:
            a stationary arm next to a stationary obstacle also has a constant
            offset, and calling that self-detection would flag every genuinely
            static scene.
        offset_tol_m: [m] how much the offset ``p_human − p_robot`` may wander
            and still count as rigid. Sized above the depth noise on a single
            argmin pixel (which hops between surface patches frame to frame)
            and well below the offset change a real obstacle produces while the
            arm sweeps ``motion_min_m``.
        confirm: consecutive flagged frames before the verdict is published.
        release: consecutive clean frames before it is withdrawn. Larger than
            ``confirm`` on purpose — the failure is persistent by nature (a
            calibration does not come and go), so the verdict should be sticky,
            and a real obstacle that briefly happens to move with the arm should
            not be able to clear a standing verdict in one frame.
    """

    def __init__(self, *, window: int = 20, motion_min_m: float = 0.08,
                 offset_tol_m: float = 0.02, confirm: int = 5,
                 release: int = 15) -> None:
        self.window = int(window)
        self.motion_min_m = float(motion_min_m)
        self.offset_tol_m = float(offset_tol_m)
        self.confirm = int(confirm)
        self.release = int(release)

        self._hist: Dict[str, List[Tuple[np.ndarray, np.ndarray]]] = {}
        self._hits: Dict[str, int] = {}
        self._clean: Dict[str, int] = {}
        self._flagged: set = set()
        #: Diagnostic, per flagged key: (arm travel [m], offset spread [m]).
        self.evidence: Dict[str, Tuple[float, float]] = {}

    # ── Per-frame ───────────────────────────────────────────────────────────

    def update(self, key: str, p_robot, p_human) -> bool:
        """Feed one control point's frame; return whether it is flagged as self.

        Args:
            key: stable identifier for the control point (``'fr3_link5#0'``).
            p_robot: (3,) the control point, base frame.
            p_human: (3,) its nearest obstacle point, base frame.
        """
        pr = np.asarray(p_robot, dtype=np.float64).ravel()
        ph = np.asarray(p_human, dtype=np.float64).ravel()
        if pr.size != 3 or ph.size != 3 or not (np.all(np.isfinite(pr))
                                                and np.all(np.isfinite(ph))):
            return key in self._flagged

        h = self._hist.setdefault(key, [])
        h.append((pr, ph))
        if len(h) > self.window:
            del h[:-self.window]
        if len(h) < self.window:
            return key in self._flagged

        robot = np.array([a for a, _ in h])
        offset = np.array([b - a for a, b in h])
        # Travel is the SPREAD of p_robot over the window, not the endpoint
        # displacement: an arm that goes out and comes back has travelled, and
        # an endpoint difference would call that stationary.
        travel = float(np.linalg.norm(robot.max(axis=0) - robot.min(axis=0)))
        spread = float(np.linalg.norm(offset.max(axis=0) - offset.min(axis=0)))

        is_rigid = travel >= self.motion_min_m and spread <= self.offset_tol_m
        if is_rigid:
            self._hits[key] = self._hits.get(key, 0) + 1
            self._clean[key] = 0
            if self._hits[key] >= self.confirm:
                self._flagged.add(key)
                self.evidence[key] = (travel, spread)
        else:
            self._clean[key] = self._clean.get(key, 0) + 1
            self._hits[key] = 0
            if self._clean[key] >= self.release:
                self._flagged.discard(key)
                self.evidence.pop(key, None)
        return key in self._flagged

    # ── Query ───────────────────────────────────────────────────────────────

    def is_self(self, key: str) -> bool:
        return key in self._flagged

    @property
    def flagged(self) -> set:
        return set(self._flagged)

    def report(self) -> Optional[str]:
        """A one-line diagnostic naming the offenders, or ``None`` when clean.

        Written to be actionable rather than merely alarming: it says which
        control points, how far the arm moved while their "obstacle" did not
        move relative to it, and the two things that actually cause it.
        """
        if not self._flagged:
            return None
        parts = []
        for k in sorted(self._flagged):
            trav, spr = self.evidence.get(k, (float('nan'), float('nan')))
            parts.append(f'{k}(arm moved {trav * 100:.0f}cm, offset held '
                         f'{spr * 1000:.0f}mm)')
        return ('SELF-DETECTION: the nearest "obstacle" is moving rigidly with '
                'the arm for ' + ', '.join(parts) + '. Their obstacle VELOCITY '
                'is suppressed (the distance is not). Cause is almost always a '
                'stale camera_extrinsics.yaml, or a robot_mask_dilate_px too '
                'small for the depth noise at this range.')

    def reset(self) -> None:
        self._hist.clear()
        self._hits.clear()
        self._clean.clear()
        self._flagged.clear()
        self.evidence.clear()
