"""Kalman filter bank for four 3D arm keypoints.

Each keypoint is estimated independently with a 6D constant-velocity state:
    [x, y, z, vx, vy, vz]

Constant-velocity transition model:
    p(k+1) = p(k) + v(k) * dt
    v(k+1) = v(k)

White acceleration noise model:
    Q = G G^T sigma_a^2

The filter receives only 3D position measurements. When a measurement is
invalid (low visibility, zero depth, NaN or Inf), the corresponding filter
performs prediction only.
"""

from __future__ import annotations
from typing import Sequence
import numpy as np


class ArmKalmanFilter:
    """Four independent 3D constant-velocity Kalman filters.

    Parameters are expressed in SI units when input positions are in metres:
    - ``process_accel_std``: expected unmodelled acceleration in m/s².
    - ``measurement_std``: standard deviation of the 3D measurement in m.
    """

    DEFAULT_KEYPOINT_NAMES = ("shoulder", "elbow", "wrist", "index")
    STATE_SIZE = 6
    MEASUREMENT_SIZE = 3

    def __init__(
        self,
        dt: float = 1.0 / 30.0,
        process_accel_std: float = 2.0,
        measurement_std: float = 0.02,
        initial_position_std: float = 0.05,
        initial_velocity_std: float = 1.0,
        visibility_threshold: float = 0.5,
        keypoint_names: Sequence[str] = DEFAULT_KEYPOINT_NAMES,
        mahalanobis_threshold: float = 11.34,
    ) -> None:
        
        if len(keypoint_names) != 4:
            raise ValueError("Exactly four keypoint names are required.")
        if dt <= 0.0:
            raise ValueError("dt must be greater than zero.")
        if process_accel_std < 0.0 or measurement_std <= 0.0:
            raise ValueError("Noise standard deviations must be positive.")

        self.keypoint_names = tuple(keypoint_names)
        self.num_keypoints = len(self.keypoint_names)
        self.visibility_threshold = float(visibility_threshold)
        self.process_accel_std = float(process_accel_std)
        self.measurement_std = float(measurement_std)
        self.initial_position_std = float(initial_position_std)
        self.initial_velocity_std = float(initial_velocity_std)
        self.mahalanobis_threshold = float(mahalanobis_threshold)

        # One state vector and one covariance matrix for each keypoint
        self.x = np.zeros((self.num_keypoints, self.STATE_SIZE), dtype=float) # shape (4, 6)
        self.P = np.zeros(
            (self.num_keypoints, self.STATE_SIZE, self.STATE_SIZE), dtype=float
        ) # shape (4, 6, 6)
        self.initialized = np.zeros(self.num_keypoints, dtype=bool)

        # We directly observe x, y and z, but not the velocities
        self.H = np.hstack(
            (np.eye(self.MEASUREMENT_SIZE), np.zeros((3, 3), dtype=float))
        ) # shape (3, 6)
        self.R = (self.measurement_std**2) * np.eye(self.MEASUREMENT_SIZE) # shape (3, 3)
        self.I = np.eye(self.STATE_SIZE) # shape (6, 6)

        self.dt = 0.0
        self.F = np.eye(self.STATE_SIZE) # shape (6, 6)
        self.Q = np.zeros((self.STATE_SIZE, self.STATE_SIZE), dtype=float) # shape (6, 6)
        self._set_dt(float(dt))

    def _set_dt(self, dt: float) -> None:
        """Update transition and process-noise matrices for a new time step."""
        if not np.isfinite(dt) or dt <= 0.0:
            raise ValueError("dt must be a finite value greater than zero.")

        self.dt = float(dt)
        dt2 = self.dt**2

        # Constant-velocity transition model
        self.F = np.eye(self.STATE_SIZE)
        self.F[0:3, 3:6] = self.dt * np.eye(3)

        # White acceleration noise model
        G = np.vstack((0.5 * dt2 * np.eye(3), self.dt * np.eye(3)))
        self.Q = (self.process_accel_std**2) * (G @ G.T)

    def _initial_covariance(self) -> np.ndarray:
        return np.diag(
            [
                self.initial_position_std**2,
                self.initial_position_std**2,
                self.initial_position_std**2,
                self.initial_velocity_std**2,
                self.initial_velocity_std**2,
                self.initial_velocity_std**2,
            ]
        )

    def _initialize_keypoint(self, index: int, position: np.ndarray) -> None:
        self.x[index, 0:3] = position
        self.x[index, 3:6] = 0.0
        self.P[index] = self._initial_covariance()
        self.initialized[index] = True

    def _predict_keypoint(self, index: int) -> None:
        if not self.initialized[index]:
            return

        self.x[index] = self.F @ self.x[index]
        self.P[index] = self.F @ self.P[index] @ self.F.T + self.Q

    def _update_keypoint(self, index: int, position: np.ndarray) -> None:
        """Execute the update and return True if the measurement is accepted, False if rejected by the gate."""
        if not self.initialized[index]:
            self._initialize_keypoint(index, position)
            return True

        innovation = position - self.H @ self.x[index]
        innovation_covariance = self.H @ self.P[index] @ self.H.T + self.R

        # --- INNOVATION GATE (Mahalanobis Distance) ---
        # D^2 = innovation^T * S^-1 * innovation
        inv_S_y = np.linalg.solve(innovation_covariance, innovation)
        mahalanobis_dist_sq = np.dot(innovation, inv_S_y)

        # If the Mahalanobis distance exceeds the threshold, discard the measurement
        if self.mahalanobis_threshold > 0.0 and mahalanobis_dist_sq > self.mahalanobis_threshold:
            return False

        # K = P H^T S^-1, computed without explicitly inverting S
        PHt = self.P[index] @ self.H.T
        kalman_gain = np.linalg.solve(innovation_covariance.T, PHt.T).T

        self.x[index] = self.x[index] + kalman_gain @ innovation

        # Joseph form: slightly longer than P=(I-KH)P, but numerically safer
        correction = self.I - kalman_gain @ self.H
        self.P[index] = (
            correction @ self.P[index] @ correction.T
            + kalman_gain @ self.R @ kalman_gain.T
        )

        return True

    def step(
        self,
        positions: np.ndarray,
        visibilities: Sequence[float],
        depths: Sequence[float],
        dt: float | None = None,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Run one filter cycle for all four keypoints.

        Args:
            positions: Array with shape ``(4, 3)`` containing measured XYZ.
            visibilities: Four MediaPipe visibility values.
            depths: Four depth values. A value ``<= 0`` is invalid.
            dt: Optional elapsed time since the previous call.

        Returns:
            ``filtered_positions`` with shape ``(4, 3)``;
            ``filtered_velocities`` with shape ``(4, 3)``;
            ``valid_mask`` with shape ``(4,)``.

        Notes:
            Until a keypoint receives its first valid measurement, no meaningful
            prediction is possible and its returned state is NaN.
        """
        
        positions_array = np.asarray(positions, dtype=float)
        visibility_array = np.asarray(visibilities, dtype=float)
        depth_array = np.asarray(depths, dtype=float)

        if positions_array.shape != (self.num_keypoints, 3):
            raise ValueError(
                f"positions must have shape ({self.num_keypoints}, 3), "
                f"received {positions_array.shape}."
            )
        if visibility_array.shape != (self.num_keypoints,):
            raise ValueError(
                f"visibilities must have shape ({self.num_keypoints},)."
            )
        if depth_array.shape != (self.num_keypoints,):
            raise ValueError(f"depths must have shape ({self.num_keypoints},).")

        if dt is not None:
            self._set_dt(float(dt))

        finite_measurement = np.all(np.isfinite(positions_array), axis=1)
        valid_mask = (
            finite_measurement
            & np.isfinite(visibility_array)
            & (visibility_array >= self.visibility_threshold)
            & np.isfinite(depth_array)
            & (depth_array > 0.0)
        )

        for index in range(self.num_keypoints):
            # Every initialized filter advances in time
            self._predict_keypoint(index)

            # A valid measurement corrects the prediction, otherwise predict only
            if valid_mask[index]:
                accepted = self._update_keypoint(index, positions_array[index])
                if not accepted:
                    valid_mask[index] = False

        filtered_positions, filtered_velocities = self.get_estimates()

        return filtered_positions, filtered_velocities, valid_mask

    def get_estimates(self) -> tuple[np.ndarray, np.ndarray]:
        """Return current positions and velocities; uninitialized states are NaN."""
        positions = np.full((self.num_keypoints, 3), np.nan, dtype=float)
        velocities = np.full((self.num_keypoints, 3), np.nan, dtype=float)

        positions[self.initialized] = self.x[self.initialized, 0:3]
        velocities[self.initialized] = self.x[self.initialized, 3:6]
        return positions, velocities

    def get_states(self) -> np.ndarray:
        """Return a copy of the four complete 6D states."""
        states = np.full_like(self.x, np.nan)
        states[self.initialized] = self.x[self.initialized]
        return states

    def reset(self, keypoint: int | str | None = None) -> None:
        """Reset one keypoint or the complete filter bank."""
        if keypoint is None:
            indices = range(self.num_keypoints)
        elif isinstance(keypoint, str):
            try:
                indices = [self.keypoint_names.index(keypoint)]
            except ValueError as exc:
                raise ValueError(f"Unknown keypoint name: {keypoint}") from exc
        else:
            index = int(keypoint)
            if index < 0 or index >= self.num_keypoints:
                raise IndexError("Keypoint index out of range.")
            indices = [index]

        for index in indices:
            self.x[index] = 0.0
            self.P[index] = 0.0
            self.initialized[index] = False