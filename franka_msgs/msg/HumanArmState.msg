std_msgs/Header header

geometry_msgs/Point shoulder
geometry_msgs/Point elbow
geometry_msgs/Point wrist
geometry_msgs/Point hand

geometry_msgs/Vector3 shoulder_vel
geometry_msgs/Vector3 elbow_vel
geometry_msgs/Vector3 wrist_vel
geometry_msgs/Vector3 hand_vel

float32[4] visibility
bool[4] measured
bool[4] keypoint_valid
float32[4] measurement_age

float32 confidence
bool valid
bool occluded

# Kalman covariance blocks per keypoint, base frame, row-major 3x3 each
# (keypoint order shoulder, elbow, wrist, hand -> 4 x 9 = 36 values).
# All zeros for a keypoint that is not valid.
float64[36] position_covariance
float64[36] velocity_covariance
float64[36] position_velocity_covariance

# Accepted measurement updates since the keypoint filter was (re)initialised.
# Counts measurements only: a keypoint coasting through an occlusion does not grow it.
uint32[4] frames_seen
