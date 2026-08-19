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