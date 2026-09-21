std_msgs/Header header

bool valid

bool geometry_ok

uint8 filter_state

bool position_valid
bool position_fresh
float32 position_age_s

geometry_msgs/Point palm_position
geometry_msgs/Vector3 palm_position_variance

uint8 VELOCITY_SOURCE_NONE=0
uint8 VELOCITY_SOURCE_UPDATED=1
uint8 VELOCITY_SOURCE_HOLD=2

uint8 VELOCITY_ESTIMATOR_C5=0
uint8 VELOCITY_ESTIMATOR_W75=1

bool velocity_valid
uint8 velocity_source
uint8 velocity_estimator
float32 velocity_age_s

geometry_msgs/Vector3 palm_velocity
float32 palm_speed

geometry_msgs/Vector3 palm_normal
geometry_msgs/Vector3 palm_longitudinal
float32 tracking_confidence
float32 motion_stability
float32 processing_latency_ms
