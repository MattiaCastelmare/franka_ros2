std_msgs/Header header

bool valid

geometry_msgs/Point palm_position
geometry_msgs/Point ee_control_point
geometry_msgs/Vector3 ee_to_palm

float32 distance

float32 tracking_confidence
float32 motion_stability
float32 distance_sigma

uint8 RATE_SOURCE_NONE=0
uint8 RATE_SOURCE_RELATIVE_UPDATED=1
uint8 RATE_SOURCE_RELATIVE_HOLD=2
uint8 RATE_SOURCE_DISTANCE_C5=3

bool rate_valid
uint8 rate_source
bool rate_degraded
float32 rate_age_s

float32 distance_rate
float32 closing_velocity

uint8 HAND_VELOCITY_SOURCE_NONE=0
uint8 HAND_VELOCITY_SOURCE_UPDATED=1
uint8 HAND_VELOCITY_SOURCE_HOLD=2

uint8 HAND_VELOCITY_ESTIMATOR_C5=0
uint8 HAND_VELOCITY_ESTIMATOR_W75=1

bool hand_velocity_valid
uint8 hand_velocity_source
uint8 hand_velocity_estimator
float32 hand_velocity_age_s
geometry_msgs/Vector3 hand_velocity

bool ee_velocity_valid
float32 ee_velocity_age_s
geometry_msgs/Vector3 ee_velocity

geometry_msgs/Vector3 relative_velocity

bool legacy_rate_valid
float32 legacy_distance_rate
float32 rate_consistency_error

bool ttc_valid
float32 ttc

bool legacy_ttc_valid
float32 legacy_ttc
