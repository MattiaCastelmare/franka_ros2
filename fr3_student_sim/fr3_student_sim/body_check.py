"""How deep the hand body is inside the wall (the wall only pushes the fingertip).

The virtual wall applies its force at one point, ``fr3_fingertip``. If the hand
turns, its body can cross the wall plane with no force at all. This module
measures that with boxes around the hand and the fingers (from the collision
geometry of franka_description, in the ``fr3_hand`` frame), so the runner can
warn about it.
"""
from __future__ import annotations

import itertools

import numpy as np

#: (min, max) corners of the boxes, ``fr3_hand`` frame [m].
#: hand.stl collision mesh, and the two fingers (open) down to their tips.
BOXES = (
    ((-0.0316, -0.1040, -0.0259), (0.0316, 0.1004, 0.0660)),
    ((-0.0110, -0.0660, 0.0584), (0.0110, 0.0660, 0.1129)),
)
CORNERS = np.array([c for lo, hi in BOXES
                    for c in itertools.product(*zip(lo, hi))])


def body_depth(robot, q, wall) -> float:
    """[m] deepest point of the hand/finger boxes behind the wall plane
    (> 0: inside the wall)."""
    p, R = robot.pose(q, 'fr3_hand')
    pts = p + CORNERS @ R.T
    return float(np.max((wall.point - pts) @ wall.normal))
