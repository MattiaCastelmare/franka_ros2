"""RViz markers of the tracked arm and of the robot-human distances, for human_visualizer.

Per arm: the arm as a blue line strip (valid keypoints only), the fading red ghosts of the
constant-velocity prediction and a green velocity arrow per keypoint. Distances: the
robot control points and an arrow per link, the shortest one coloured by its zone.
"""

from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker

VELOCITY_ARROW_SCALE = 0.3   # [s] arrow length = speed x this
ZONE_COLORS = {
    'critical': ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0),
    'danger': ColorRGBA(r=1.0, g=0.5, b=0.0, a=1.0),
}
SAFE_COLOR = ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)


def _marker(frame_id, stamp, ns, marker_id=0, marker_type=Marker.LINE_STRIP,
            action=Marker.ADD):
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = stamp
    marker.ns = ns
    marker.id = marker_id
    marker.type = marker_type
    marker.action = action
    return marker


def _scale(marker, x, y=None, z=None):
    marker.scale.x = x
    marker.scale.y = x if y is None else y
    marker.scale.z = marker.scale.y if z is None else z


def delete_all(namespaces, frame_id, stamp):
    """One DELETEALL marker per namespace."""
    return [_marker(frame_id, stamp, ns, action=Marker.DELETEALL) for ns in namespaces]


def arm_markers(state, prediction, side, frame_id, stamp):
    """Markers of one arm; all of its markers deleted when no keypoint is valid."""
    valid = state.keypoint_valid
    if not any(valid):
        return delete_all([f"human_arm_{side}", f"human_prediction_lines_{side}",
                           f"human_prediction_joints_{side}", f"velocities_{side}"],
                          frame_id, stamp)

    keypoints = [state.shoulder, state.elbow, state.wrist, state.hand]
    velocities = [state.shoulder_vel, state.elbow_vel, state.wrist_vel, state.hand_vel]

    arm = _marker(frame_id, stamp, f"human_arm_{side}")
    arm.scale.x = 0.12
    arm.color = ColorRGBA(r=0.0, g=0.5, b=1.0, a=0.5)
    arm.points = [pt for i, pt in enumerate(keypoints) if valid[i]]
    markers = [arm]

    if prediction is not None:
        markers += prediction_markers(prediction, side, frame_id, stamp)

    for i, (pt, v) in enumerate(zip(keypoints, velocities)):
        if not valid[i]:
            # The keypoint is lost: remove its arrow
            markers.append(_marker(frame_id, stamp, f"velocities_{side}", i + 200,
                                   action=Marker.DELETE))
            continue
        arrow = _marker(frame_id, stamp, f"velocities_{side}", i + 200, Marker.ARROW)
        arrow.points = [pt, Point(x=pt.x + v.x * VELOCITY_ARROW_SCALE,
                                  y=pt.y + v.y * VELOCITY_ARROW_SCALE,
                                  z=pt.z + v.z * VELOCITY_ARROW_SCALE)]
        _scale(arrow, 0.015, 0.030)
        arrow.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.8)
        markers.append(arrow)
    return markers


def prediction_markers(prediction, side, frame_id, stamp, max_steps=10):
    """Fading red ghost arms (line strip + joints) of the first steps of the prediction."""
    markers = []
    for step in range(min(max_steps, prediction.num_steps)):
        color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=max(0.05, 0.4 - 0.15 * step))
        future = [prediction.shoulder[step], prediction.elbow[step],
                  prediction.wrist[step], prediction.hand[step]]
        points = [pt for i, pt in enumerate(future) if prediction.keypoint_valid[i]]
        if not points:
            continue
        lines = _marker(frame_id, stamp, f"human_prediction_lines_{side}", step + 10)
        lines.scale.x = 0.04
        joints = _marker(frame_id, stamp, f"human_prediction_joints_{side}", step + 50,
                         Marker.SPHERE_LIST)
        _scale(joints, 0.06)
        for marker in (lines, joints):
            marker.color = color
            marker.points = points
            markers.append(marker)
    return markers


def distance_markers(links, frame_id, stamp):
    """Robot control points and distance arrows; both deleted when there are no links."""
    if not links:
        return delete_all(["robot_points", "distances"], frame_id, stamp)

    markers = []
    for i, link in enumerate(links):
        sphere = _marker(frame_id, stamp, "robot_points", i, Marker.SPHERE)
        sphere.pose.position = link.closest_point_robot
        _scale(sphere, 0.06)
        sphere.color = ColorRGBA(r=1.0, g=0.8, b=0.0, a=0.8)
        markers.append(sphere)

    min_link = min(links, key=lambda l: l.distance)
    for i, link in enumerate(links):
        arrow = _marker(frame_id, stamp, "distances", i, Marker.ARROW)
        arrow.points = [link.closest_point_human, link.closest_point_robot]
        if link == min_link:
            _scale(arrow, 0.02, 0.04)
            arrow.color = ZONE_COLORS.get(link.zone, SAFE_COLOR)
        else:
            _scale(arrow, 0.005, 0.010)
            arrow.color = ColorRGBA(r=0.6, g=0.6, b=0.6, a=0.4)
        markers.append(arrow)
    return markers