#!/usr/bin/env python3

import cv2


def draw_selected_landmarks(image, landmarks, landmark_ids):
    """Compatibility helper used by human_hand_tracker."""
    height, width = image.shape[:2]

    for landmark_id in landmark_ids:
        landmark = landmarks[landmark_id]

        u = int(round(landmark.x * (width - 1)))
        v = int(round(landmark.y * (height - 1)))

        u = max(0, min(width - 1, u))
        v = max(0, min(height - 1, v))

        cv2.circle(
            image,
            (u, v),
            4,
            (0, 0, 255),
            1,
        )


def draw_status(image, status, wrist=None):
    """Compatibility helper used by human_hand_tracker."""
    cv2.putText(
        image,
        status,
        (20, 35),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (0, 255, 0),
        1,
    )


def draw_hand_state_overlay(image, state):
    """Minimal HandState overlay: scalar C5 palm speed only."""

    if state is None:
        return

    valid = bool(state.valid)

    color = (
        (0, 255, 0)
        if valid
        else (0, 0, 255)
    )

    text = (
        f'v = {state.palm_speed:.2f} m/s'
        if valid
        else 'v = --'
    )

    # Tiny box + one text draw.
    # No image.copy(), alpha blending, bars or extra data.
    cv2.rectangle(
        image,
        (8, 8),
        (185, 38),
        (0, 0, 0),
        -1,
    )

    cv2.putText(
        image,
        text,
        (14, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        color,
        1,
    )


def draw_palm_outline(image, hand_landmarks):
    """
    Optional ultra-light palm outline.

    Zero runtime cost unless human_hand_tracker explicitly
    calls this function.

    Uses only landmarks:
        0  = wrist
        5  = index MCP
        9  = middle MCP
        17 = pinky MCP

    Computational work:
        4 coordinate conversions
        4 cv2.line()
    """

    landmarks = getattr(
        hand_landmarks,
        'landmark',
        hand_landmarks,
    )

    height, width = image.shape[:2]

    points = []

    for landmark_id in (0, 5, 9, 17):
        landmark = landmarks[landmark_id]

        u = int(round(
            landmark.x * (width - 1)
        ))

        v = int(round(
            landmark.y * (height - 1)
        ))

        u = max(
            0,
            min(width - 1, u),
        )

        v = max(
            0,
            min(height - 1, v),
        )

        points.append(
            (u, v)
        )

    for start, end in zip(
        points,
        points[1:] + points[:1],
    ):
        cv2.line(
            image,
            start,
            end,
            (0, 255, 0),
            2,
        )