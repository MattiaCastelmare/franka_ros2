# Grasp on the object in the hand (handover_grasp.py pose), for the grasp executor.
# Header: stamp of the frame the grasp is for, base frame. Between two backend grasps the node republishes
# the last one moved with the palm at every HandState (camera rate).
std_msgs/Header header
# Stamp of the depth image the backend computed the grasp on (= header stamp when not moved).
builtin_interfaces/Time source_stamp
# Franka TCP pose of the grasp: z approach, y closing direction.
geometry_msgs/Pose pose
# Gripper opening of the grasp [m].
float32 width
# Backend score of this grasp (GSNet / AnyGrasp), not a calibrated probability.
float32 score
# 0..1: score clipped to [0, 1] times (1 + recent grasps within 3 cm and 30 deg of this one) /
# (1 + recent grasps), over the last 10 frames: a grasp that keeps coming back is more trustworthy.
float32 confidence
# Do not use the grasp after this time (header stamp + validity of the node).
builtin_interfaces/Time valid_until
