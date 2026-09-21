#!/usr/bin/env python3
"""
Palm geometry used by HumanHandTracker.

Contains only the RGB-D palm-plane candidate computation.
Selection, ROS orchestration and RGB-D reconstruction live in
their dedicated modules.
"""

import numpy as np
import math
from collections import deque

class PalmGeometryMixin:
    """Palm geometry operations used by HumanHandTracker."""

    @staticmethod
    def _normalise_geometry_vector(
        vector,
    ):
        vector = np.asarray(
            vector,
            dtype=float,
        )

        if not np.isfinite(
            vector
        ).all():
            return None

        norm = float(
            np.linalg.norm(
                vector
            )
        )

        if norm <= 1e-12:
            return None

        return vector / norm

    def compute_palm_geometry_candidate(
        self,
        hand,
        image_shape,
        depth_image,
        depth_encoding,
    ):
        """
        RGBD5_PCA geometry candidate.

        Uses DIRECT RGB-D observations only:
            0  wrist
            5  index MCP
            9  middle MCP
            13 ring MCP
            17 pinky MCP

        At least four valid 3-D points are needed for PCA.

        wrist/index/pinky must be available to compute the
        anatomical anchor cross product.

        Returns:
            (plane_normal, anchor_cross)

        plane_normal intentionally keeps the arbitrary PCA
        +/- sign. The anatomical sign is resolved later,
        AFTER handedness stabilization.
        """

        points = {}

        for landmark_id in (
            self.GEOMETRY_LANDMARK_IDS
        ):

            point_camera = (
                self.landmark_to_3d(
                    hand[
                        landmark_id
                    ],
                    image_shape,
                    depth_image,
                    depth_encoding,
                )
            )

            if point_camera is None:
                continue

            point_base = (
                self.apply_transform(
                    point_camera
                )
            )

            if not np.isfinite(
                point_base
            ).all():
                continue

            points[
                landmark_id
            ] = np.asarray(
                point_base,
                dtype=float,
            )

        if len(points) < 4:

            return (
                None,
                None,
            )

        if not all(
            landmark_id in points
            for landmark_id
            in (
                0,
                5,
                17,
            )
        ):

            return (
                None,
                None,
            )

        point_matrix = np.stack(
            list(
                points.values()
            )
        )

        centred = (
            point_matrix
            -
            np.mean(
                point_matrix,
                axis=0,
            )
        )

        covariance = (
            centred.T
            @ centred
            /
            len(
                point_matrix
            )
        )

        try:

            eigenvalues, eigenvectors = (
                np.linalg.eigh(
                    covariance
                )
            )

        except np.linalg.LinAlgError:

            return (
                None,
                None,
            )

        plane_normal = (
            eigenvectors[
                :,
                np.argmin(
                    eigenvalues
                ),
            ]
        )

        plane_normal = (
            self._normalise_geometry_vector(
                plane_normal
            )
        )

        wrist = points[0]

        index_vector = (
            points[5]
            - wrist
        )

        pinky_vector = (
            points[17]
            - wrist
        )

        anchor_cross = (
            np.cross(
                index_vector,
                pinky_vector,
            )
        )

        anchor_cross = (
            self._normalise_geometry_vector(
                anchor_cross
            )
        )

        if (
            plane_normal is None
            or anchor_cross is None
        ):

            return (
                None,
                None,
            )

        return (
            plane_normal,
            anchor_cross,
        )

# ============================================================
# Palm velocity estimation
# ============================================================

SOURCE_NONE = 0
SOURCE_UPDATED = 1
SOURCE_HOLD = 2

BANK = (3, 4, 5, 7, 9, 12)

def weighted_linear_velocity(times, positions, tau_s):
    times = np.asarray(times, dtype=float)
    positions = np.asarray(positions, dtype=float)
    if len(times) < 3:
        return None
    x = times - times[-1]
    weights = np.exp(x / float(tau_s))
    sw = float(np.sum(weights))
    if sw <= 0.0:
        return None
    x_mean = float(np.sum(weights * x) / sw)
    p_mean = (
        np.sum(
            weights[:, None] * positions,
            axis=0,
        )
        / sw
    )
    xc = x - x_mean
    denominator = float(
        np.sum(
            weights * xc * xc
        )
    )
    if denominator <= 1e-12:
        return None
    return np.asarray(
        np.sum(
            weights[:, None]
            * xc[:, None]
            * (positions - p_mean),
            axis=0,
        )
        / denominator,
        dtype=float,
    )

def slope_filter(times):
    times = np.asarray(times, dtype=float)
    if len(times) < 2:
        return None
    x = times - times[-1]
    # Exact closed-form slope row for a linear regression
    # with intercept.
    #
    # This is mathematically equivalent to:
    #     np.linalg.pinv([1, x])[1]
    # but avoids computing a pseudoinverse every update.
    xc = x - np.mean(x)
    denominator = float(
        np.dot(
            xc,
            xc,
        )
    )
    if (
        not np.isfinite(denominator)
        or
        denominator <= 1e-12
    ):
        return None
    return np.asarray(
        xc / denominator,
        dtype=float,
    )

def direct_derivative_sum_filter(times, n0):
    times = np.asarray(times, dtype=float)
    N = len(times)
    if n0 < 1 or N < n0 + 1:
        return None
    s = np.zeros(
        N,
        dtype=float,
    )
    for i in range(
        N - n0,
        N,
    ):
        dt = float(
            times[i]
            - times[i - 1]
        )
        if dt <= 1e-9:
            return None
        s[i] += 1.0 / dt
        s[i - 1] -= 1.0 / dt
    return s

def surde_candidates(
    times,
    positions,
    sigma_mm,
):
    times = np.asarray(
        times,
        dtype=float,
    )
    positions = np.asarray(
        positions,
        dtype=float,
    )
    if len(times) < BANK[0]:
        return None
    sigma2 = (
        float(sigma_mm)
        / 1000.0
    ) ** 2
    available_bank = [
        N
        for N in BANK
        if N <= len(times)
    ]
    if not available_bank:
        return None
    n0 = BANK[0] - 1
    rows = []
    for N in available_bank:
        tt = times[-N:]
        yy = positions[-N:]
        v = slope_filter(tt)
        s = direct_derivative_sum_filter(
            tt,
            n0,
        )
        if v is None or s is None:
            continue
        velocity = v @ yy
        direct_sum = s @ yy
        # Q = sigma^2 I, therefore:
        #
        # v.T Q s = sigma^2 dot(v, s)
        # v.T Q v = sigma^2 dot(v, v)
        # s.T Q s = sigma^2 dot(s, s)
        #
        # Avoid explicitly allocating Q.
        Cvs = (
            sigma2
            * float(
                np.dot(
                    v,
                    s,
                )
            )
        )
        Vw = (
            sigma2
            * float(
                np.dot(
                    v,
                    v,
                )
            )
        )
        Sw = (
            sigma2
            * float(
                np.dot(
                    s,
                    s,
                )
            )
        )
        costs_xyz = (
            n0 * velocity ** 2
            + 2.0 * Cvs
            - 2.0
            * velocity
            * direct_sum
        )
        rows.append(
            {
                'N': int(N),
                'velocity':
                    np.asarray(
                        velocity,
                        dtype=float,
                    ),
                'cost':
                    float(
                        np.sum(
                            costs_xyz
                        )
                    ),
                'Vw':
                    Vw,
                'Sw':
                    Sw,
                'Cvs':
                    Cvs,
            }
        )
    if not rows:
        return None
    return {
        'rows': rows,
        'n0': n0,
    }

def surde_temperature(candidates):
    rows = candidates['rows']
    K = len(rows)
    if K <= 1:
        return np.inf
    shortest = rows[0]
    Vw = float(
        shortest['Vw']
    )
    Sw = float(
        shortest['Sw']
    )
    Cvs = float(
        shortest['Cvs']
    )
    n0 = int(
        candidates['n0']
    )
    nu_axis = (
        2.0
        * n0 ** 2
        * Vw ** 2
        - 8.0
        * n0
        * Vw
        * Cvs
        + 4.0
        * Vw
        * Sw
        + 4.0
        * Cvs ** 2
    )
    nu_xyz = (
        3.0
        * max(
            nu_axis,
            0.0,
        )
    )
    if nu_xyz <= 1e-30:
        return 1e-15
    return max(
        math.sqrt(
            nu_xyz
            /
            (
                2.0
                * math.log(K)
            )
        ),
        1e-15,
    )

def surde_soft(
    times,
    positions,
    sigma_mm,
):
    candidates = surde_candidates(
        times,
        positions,
        sigma_mm,
    )
    if candidates is None:
        return None
    rows = candidates['rows']
    costs = np.asarray(
        [
            row['cost']
            for row in rows
        ],
        dtype=float,
    )
    Ns = np.asarray(
        [
            row['N']
            for row in rows
        ],
        dtype=float,
    )
    velocities = np.stack(
        [
            row['velocity']
            for row in rows
        ],
        axis=0,
    )
    if len(rows) == 1:
        weights = np.ones(
            1,
            dtype=float,
        )
    else:
        T = surde_temperature(
            candidates
        )
        logw = (
            np.log(Ns)
            - costs / T
        )
        logw -= np.max(logw)
        weights = np.exp(logw)
        total = float(
            np.sum(weights)
        )
        if (
            not np.isfinite(total)
            or total <= 0.0
        ):
            best = int(
                np.argmin(costs)
            )
            weights = np.zeros(
                len(rows),
                dtype=float,
            )
            weights[best] = 1.0
        else:
            weights /= total
    return np.asarray(
        np.sum(
            weights[:, None]
            * velocities,
            axis=0,
        ),
        dtype=float,
    )

class W75VelocityEstimator:
    """
    Frozen configuration validated in Stage 5C and ROS streaming.

    EWL:
      tau = 0.20 s
      latest 5 fresh samples
      retention = 0.40 s

    SURDE:
      SOFT N3
      sigma = 2 mm
      bank = (3,4,5,7,9,12)
      white covariance
      retention = 0.80 s

    Fusion:
      75% SURDE
      25% EWL

    Hold:
      <= 0.10 s

    Prediction:
      OFF
    """

    def __init__(self):
        self.ewl_history = deque(
            maxlen=5
        )
        self.surde_history = deque(
            maxlen=12
        )
        self.ewl_last_velocity = None
        self.ewl_last_update = None
        self.surde_last_velocity = None
        self.surde_last_update = None

    def reset(self):
        self.ewl_history.clear()
        self.surde_history.clear()
        self.ewl_last_velocity = None
        self.ewl_last_update = None
        self.surde_last_velocity = None
        self.surde_last_update = None

    @staticmethod
    def prune(
        history,
        now,
        maximum_age,
    ):
        while (
            history
            and
            now - history[0][0]
            > maximum_age
        ):
            history.popleft()

    def update_ewl(
        self,
        now,
        palm,
        fresh,
        position_available,
    ):
        self.prune(
            self.ewl_history,
            now,
            0.40,
        )
        updated = False
        if fresh:
            self.ewl_history.append(
                (
                    now,
                    palm.copy(),
                )
            )
            if len(
                self.ewl_history
            ) >= 3:
                tt = np.asarray(
                    [
                        item[0]
                        for item
                        in self.ewl_history
                    ],
                    dtype=float,
                )
                pp = np.asarray(
                    [
                        item[1]
                        for item
                        in self.ewl_history
                    ], dtype=float,)
                velocity = (weighted_linear_velocity(tt, pp, 0.20,))
                if velocity is not None:
                    self.ewl_last_velocity = (velocity.copy())
                    self.ewl_last_update = now
                    updated = True
        if (position_available
            and self.ewl_last_velocity is not None and self.ewl_last_update is not None):
            age = (now - self.ewl_last_update)
            if (-1e-9 <= age <= 0.10 + 1e-9):
                return (True, self.ewl_last_velocity.copy(),
                    (SOURCE_UPDATED if updated else SOURCE_HOLD), float(age), updated,)
        return (False, None, SOURCE_NONE, np.nan, updated,)

    def update_surde(self, now, palm, fresh, position_available,):
        self.prune(self.surde_history, now, 0.80,)
        updated = False
        if fresh:
            self.surde_history.append((now, palm.copy(),))
            if len(self.surde_history) >= BANK[0]:
                tt = np.asarray([item[0] for item in self.surde_history], dtype=float,)
                pp = np.asarray([item[1] for item in self.surde_history], dtype=float,)
                velocity = surde_soft(tt, pp, 2.0,)
                if velocity is not None:
                    self.surde_last_velocity = (velocity.copy())
                    self.surde_last_update = now
                    updated = True
        if (position_available
            and self.surde_last_velocity is not None and self.surde_last_update is not None):
            age = (now - self.surde_last_update)
            if (-1e-9 <= age <= 0.10 + 1e-9):
                return (True, self.surde_last_velocity.copy(),
                    (SOURCE_UPDATED if updated else SOURCE_HOLD), float(age), updated,)
        return (False, None, SOURCE_NONE, np.nan, updated,)

    def update(self, now, palm, fresh, position_available,):
        (ewl_available, ewl_velocity, ewl_source,
            ewl_age, ewl_updated,) = self.update_ewl(now, palm, fresh, position_available,)
        (surde_available, surde_velocity, surde_source,
            surde_age, surde_updated,) = self.update_surde(now, palm, fresh, position_available,)
        if (ewl_available and surde_available):
            return {'available':
                    True, 'velocity':
                    (0.25 * ewl_velocity + 0.75 * surde_velocity), 'source':
                    (SOURCE_UPDATED if (ewl_updated or surde_updated) else SOURCE_HOLD), 'age_s':
                    max(float(ewl_age), float(surde_age),),}
        if surde_available:
            return {'available':
                    True, 'velocity':
                    surde_velocity.copy(), 'source':
                    int(surde_source), 'age_s':
                    float(surde_age),}
        if ewl_available:
            return {'available':
                    True, 'velocity':
                    ewl_velocity.copy(), 'source':
                    int(ewl_source), 'age_s':
                    float(ewl_age),}
        return {'available':
                False, 'velocity':
                np.zeros(3, dtype=float,), 'source':
                SOURCE_NONE, 'age_s':
                np.nan,}


# Empirical W75 constant-velocity prediction uncertainty.
# q97.5 3-D palm error [mm], calibrated offline by speed class.
PREDICTION_HORIZONS_MS = np.asarray(
    (33, 67, 100, 133, 167, 200, 250, 300),
    dtype=float,
)

PREDICTION_Q975_MM = {
    'slow': np.asarray((15.143, 20.815, 30.395, 37.848, 49.529, 66.295, 84.938, 108.248), dtype=float),
    'low': np.asarray((25.957, 40.220, 61.148, 84.904, 91.217, 104.230, 139.686, 179.249), dtype=float),
    'medium': np.asarray((48.072, 69.596, 90.289, 106.856, 106.349, 134.713, 182.454, 233.515), dtype=float),
    'fast': np.asarray((48.820, 86.009, 99.605, 128.562, 162.389, 205.623, 246.683, 296.686), dtype=float),
}

PREDICTION_CHI3_Q975_RADIUS = 3.057515920563


def prediction_speed_class(speed):
    if speed < 0.10:
        return 'slow'
    if speed < 0.30:
        return 'low'
    if speed < 0.60:
        return 'medium'
    return 'fast'


def prediction_q975_error_m(age_s, speed):
    age_ms = float(age_s) * 1000.0

    if (
        not np.isfinite(age_ms)
        or not np.isfinite(speed)
        or age_ms < 0.0
        or age_ms > 300.0 + 1e-9
    ):
        return np.nan

    values = np.maximum.accumulate(
        PREDICTION_Q975_MM[
            prediction_speed_class(float(speed))
        ]
    )

    horizons = np.concatenate((
        np.asarray([0.0]),
        PREDICTION_HORIZONS_MS,
    ))

    errors = np.concatenate((
        np.asarray([0.0]),
        values,
    ))

    return float(
        np.interp(
            age_ms,
            horizons,
            errors,
        )
    ) / 1000.0
