#!/usr/bin/env python3

import cv2
import numpy as np


def draw_selected_landmarks(image, landmarks, landmark_ids):
    """Draw and number the selected landmarks in red."""
    height, width = image.shape[:2]

    for landmark_id in landmark_ids:
        landmark = landmarks[landmark_id]

        u = int(np.clip(
            round(landmark.x * (width - 1)),
            0,
            width - 1,
        ))
        v = int(np.clip(
            round(landmark.y * (height - 1)),
            0,
            height - 1,
        ))

        cv2.circle(image, (u, v), 7, (0, 0, 255), 2)
        cv2.putText(
            image,
            str(landmark_id),
            (u + 8, v - 8),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 0, 255),
            2,
            cv2.LINE_AA,
        )


def draw_status(image, status, wrist=None):
    """Draw the tracking status and, if available, the wrist position in cm."""
    cv2.putText(
        image,
        status,
        (20, 35),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.9,
        (0, 255, 0),
        2,
        cv2.LINE_AA,
    )

    if wrist is None:
        return

    text = (
        f'x={100.0 * wrist[0]:.1f} cm  '
        f'y={100.0 * wrist[1]:.1f} cm  '
        f'z={100.0 * wrist[2]:.1f} cm'
    )

    cv2.putText(
        image,
        text,
        (20, 70),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.65,
        (0, 255, 0),
        2,
        cv2.LINE_AA,
    )
