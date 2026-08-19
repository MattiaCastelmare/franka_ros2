std_msgs/Header header

float32 step_dt
uint32 num_steps

geometry_msgs/Point[] shoulder
geometry_msgs/Point[] elbow
geometry_msgs/Point[] wrist
geometry_msgs/Point[] hand

bool[4] keypoint_valid
float32[4] measurement_age