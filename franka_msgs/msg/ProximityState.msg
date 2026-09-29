# Point-to-point proximity in header.frame_id, stamped at the hand measurement.
# EE–active-palm only for now. Not surface clearance or a safety constraint.
std_msgs/Header header
string robot_point_id
string target_point_id
bool valid
geometry_msgs/Point robot_position
geometry_msgs/Point target_position
float32 distance
float32 distance_sigma

bool relative_velocity_valid
geometry_msgs/Vector3 relative_velocity
uint8 RATE_SOURCE_NONE=0
uint8 RATE_SOURCE_RELATIVE_UPDATED=1
uint8 RATE_SOURCE_RELATIVE_HOLD=2
uint8 RATE_SOURCE_DISTANCE_C5=3
bool rate_valid
uint8 rate_source
bool rate_degraded
float32 rate_age_s
# Positive rate means separation; positive closing means approach.
float32 distance_rate
float32 closing_velocity
bool ttc_valid
float32 ttc
