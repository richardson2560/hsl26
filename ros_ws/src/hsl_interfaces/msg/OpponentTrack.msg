std_msgs/Header header
string id

# State estimator lifecycle.
uint8 STATE_SEARCHING = 0
uint8 STATE_TRACKED = 1
uint8 STATE_COASTING = 2
uint8 STATE_OCCLUDED_BELIEF = 3
uint8 STATE_LOST = 4
uint8 state

geometry_msgs/Pose2D pose
geometry_msgs/Twist twist
float64[16] covariance
bool yaw_valid

# Temporal and spatial validity.
builtin_interfaces/Time last_measurement
builtin_interfaces/Time valid_until
string localization_epoch
uint32 map_version

string source_id
