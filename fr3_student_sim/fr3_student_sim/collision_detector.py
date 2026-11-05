"""Per-joint residual thresholds, separate from observer and control.

Robotics 2, lecture 18, slide 28; Haddadin, De Luca and Albu-Schaeffer
(2017), equation 54.

The collision flag is true when any |r_i| exceeds its configured threshold.
An event marks each false-to-true transition; detection does not change torque.
"""

import numpy as np

from fr3_student_sim.momentum_observer import joint_vector


class CollisionDetector:

    def __init__(self, threshold=0.1):
        self.threshold = joint_vector(threshold, 'residual threshold', positive=True)
        self.detected = False
        self.joints = np.zeros(7, dtype=bool)

    def update(self, residual):
        residual = joint_vector(residual, 'residual')
        self.joints = np.abs(residual) > self.threshold
        detected = bool(np.any(self.joints))
        event = detected and not self.detected
        self.detected = detected
        return detected, event
