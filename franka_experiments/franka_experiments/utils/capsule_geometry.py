#!/usr/bin/env python3
"""Simple capsule models for a human arm and a robot.

Every point must already be expressed in the same frame, normally fr3_link0.
"""

from __future__ import annotations

import numpy as np

Capsule = dict[str, object]
ControlPoint = dict[str, object]


def _vec3(value, name: str) -> np.ndarray:
    """Return a finite NumPy vector with shape (3,)."""
    point = np.asarray(value, dtype=float)
    if point.shape != (3,) or not np.all(np.isfinite(point)):
        raise ValueError(f"{name} must be a finite vector with shape (3,)")
    return point


def point_to_segment_distance(
    point: np.ndarray,
    start: np.ndarray,
    end: np.ndarray,
) -> tuple[float, np.ndarray]:
    """Distance and closest point from a point to a finite segment."""
    point = _vec3(point, "point")
    start = _vec3(start, "start")
    end = _vec3(end, "end")

    axis = end - start
    axis_length_squared = float(axis @ axis)

    if axis_length_squared < 1.0e-12:
        closest = start
    else:
        alpha = float(((point - start) @ axis) / axis_length_squared)
        alpha = float(np.clip(alpha, 0.0, 1.0))
        closest = start + alpha * axis

    return float(np.linalg.norm(point - closest)), closest


def point_to_capsule_distance(
    point: np.ndarray,
    capsule: Capsule,
    point_radius: float = 0.0,
) -> tuple[float, np.ndarray]:
    """Signed distance from a point/sphere to a capsule.

    Positive: separated. Zero: contact. Negative: overlap.
    """
    axis_distance, closest_axis_point = point_to_segment_distance(
        point,
        capsule["p0"],
        capsule["p1"],
    )
    distance = (
        axis_distance
        - float(point_radius)
        - float(capsule["radius"])
    )
    return float(distance), closest_axis_point


class HumanArmGeometry:
    """Build three capsules from shoulder, elbow, wrist and hand."""

    def __init__(
        self,
        upper_arm_radius: float = 0.075,
        forearm_radius: float = 0.065,
        hand_radius: float = 0.075,
        safety_margin: float = 0.02,
    ) -> None:
        self.radii = (
            float(upper_arm_radius),
            float(forearm_radius),
            float(hand_radius),
        )
        self.safety_margin = float(safety_margin)

    def build_capsules(
        self,
        keypoints: np.ndarray,
        valid: np.ndarray | None = None,
        extra_margin: float = 0.0,
    ) -> list[Capsule]:
        """Create upper-arm, forearm and hand capsules.

        ``keypoints`` has shape (4, 3) and fixed order:
        shoulder, elbow, wrist, hand/index.
        """
        points = np.asarray(keypoints, dtype=float)
        if points.shape != (4, 3):
            raise ValueError("keypoints must have shape (4, 3)")

        finite = np.all(np.isfinite(points), axis=1)
        valid_mask = finite if valid is None else finite & np.asarray(valid, bool)
        if valid_mask.shape != (4,):
            raise ValueError("valid must have shape (4,)")

        names = ("human_upper_arm", "human_forearm", "human_hand")
        endpoint_pairs = ((0, 1), (1, 2), (2, 3))
        margin = self.safety_margin + float(extra_margin)

        capsules: list[Capsule] = []
        for name, (start_index, end_index), radius in zip(
            names, endpoint_pairs, self.radii
        ):
            if not (valid_mask[start_index] and valid_mask[end_index]):
                continue

            capsules.append(
                {
                    "name": name,
                    "p0": points[start_index].copy(),
                    "p1": points[end_index].copy(),
                    "radius": radius + margin,
                }
            )

        return capsules


class RobotGeometry:
    """Build robot capsules and optional spherical control points."""

    def __init__(
        self,
        definitions: list[dict[str, object]],
        safety_margin: float = 0.01,
    ) -> None:
        """Store capsule definitions.

        Each definition contains: name, start, end and radius.
        ``start`` and ``end`` are keys in the ``frame_points`` dictionary.
        """
        self.definitions = definitions
        self.safety_margin = float(safety_margin)

    def build_capsules(
        self,
        frame_points: dict[str, np.ndarray],
        extra_margin: float = 0.0,
    ) -> list[Capsule]:
        """Create robot capsules from Cartesian link/frame points."""
        margin = self.safety_margin + float(extra_margin)
        capsules: list[Capsule] = []

        for definition in self.definitions:
            start_name = str(definition["start"])
            end_name = str(definition["end"])

            if start_name not in frame_points or end_name not in frame_points:
                continue

            capsules.append(
                {
                    "name": str(definition["name"]),
                    "p0": _vec3(frame_points[start_name], start_name).copy(),
                    "p1": _vec3(frame_points[end_name], end_name).copy(),
                    "radius": float(definition["radius"]) + margin,
                }
            )

        return capsules

    def build_control_points(
        self,
        capsules: list[Capsule],
        points_per_capsule: int = 3,
    ) -> list[ControlPoint]:
        """Sample a few spherical points along each robot capsule."""
        if points_per_capsule < 2:
            raise ValueError("points_per_capsule must be at least 2")

        control_points: list[ControlPoint] = []
        for capsule in capsules:
            p0 = np.asarray(capsule["p0"], dtype=float)
            p1 = np.asarray(capsule["p1"], dtype=float)

            for index, alpha in enumerate(
                np.linspace(0.0, 1.0, points_per_capsule)
            ):
                control_points.append(
                    {
                        "name": f"{capsule['name']}_cp_{index}",
                        "position": p0 + float(alpha) * (p1 - p0),
                        "radius": float(capsule["radius"]),
                        "source_capsule": str(capsule["name"]),
                        "alpha": float(alpha),
                    }
                )

        return control_points

    def minimum_distance_to_human(
        self,
        control_points: list[ControlPoint],
        human_capsules: list[Capsule],
    ) -> dict[str, object] | None:
        """Find the closest robot control-point/human-capsule pair."""
        best = None

        for robot_point in control_points:
            for human_capsule in human_capsules:
                distance, closest_human_point = point_to_capsule_distance(
                    robot_point["position"],
                    human_capsule,
                    point_radius=float(robot_point["radius"]),
                )

                if best is None or distance < best["distance"]:
                    best = {
                        "distance": distance,
                        "robot_point": robot_point["name"],
                        "human_capsule": human_capsule["name"],
                        "robot_position": robot_point["position"].copy(),
                        "closest_human_point": closest_human_point.copy(),
                    }

        return best