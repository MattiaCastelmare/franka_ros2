"""Virtual compliant wall: a plane that pushes the tool point back.

Spring-damper contact as in the planar 3R study, with damping only while
compressing:

    delta = n . (p_wall - p)                 penetration (> 0 inside the wall)
    v_n   = -n . v                           penetration rate
    F_n   = k_e delta + b_e max(v_n, 0)      if delta > 0, else 0   (and F_n >= 0)
    F     = F_n n                            force on the robot (world frame)

``n`` is the outward unit normal (pointing from the wall into free space).
The runner applies ``tau_ext = J_c^T F`` to the joints after clipping the
student's torque, so the force is external: the student's controller never
sees it, and only an observer can detect it.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np


@dataclass
class Wall:
    point: np.ndarray = field(default_factory=lambda: np.array([0.6, 0.0, 0.0]))
    normal: np.ndarray = field(default_factory=lambda: np.array([-1.0, 0.0, 0.0]))
    stiffness: float = 1500.0     # k_e [N/m]
    damping: float = 15.0         # b_e [N s/m]
    enabled: bool = True

    def __post_init__(self):
        self.point = np.asarray(self.point, float)
        self.normal = np.asarray(self.normal, float) / np.linalg.norm(self.normal)

    @classmethod
    def from_dict(cls, d: dict) -> 'Wall':
        return cls(point=d.get('point', [0.6, 0.0, 0.0]), normal=d.get('normal', [-1.0, 0, 0]),
                   stiffness=float(d.get('stiffness', 1500.0)),
                   damping=float(d.get('damping', 15.0)))

    def contact(self, p, v):
        """(F (3,), F_n, delta, v_n) for tool position ``p`` and velocity ``v``."""
        delta = float(self.normal @ (self.point - p))
        v_n = float(-self.normal @ v)
        if not self.enabled or delta <= 0.0:
            return np.zeros(3), 0.0, delta, v_n
        f_n = max(self.stiffness * delta + self.damping * max(v_n, 0.0), 0.0)
        return f_n * self.normal, f_n, delta, v_n
