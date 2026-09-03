std_msgs/Header header

uint8 UNINITIALIZED=0
uint8 TRACKING=1
uint8 PREDICT_ONLY=2
uint8 LOST=3
uint8 filter_state

uint8[4] landmark_ids

geometry_msgs/Point[4] positions
geometry_msgs/Vector3[4] velocities

uint8[4] landmark_state

bool[4] measurement_used

uint8 DIRECT=0
uint8 ESTIMATED=1
uint8 INVALID=2
uint8[4] measurement_type

float32[4] mahalanobis_sq

geometry_msgs/Vector3[4] position_variance
geometry_msgs/Vector3[4] velocity_variance

float32[4] age_s

uint32[4] missed_updates

float32 processing_latency_ms