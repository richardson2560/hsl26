hsl_interfaces/ContractHeader meta
string option_instance_id
uint8 kind
uint8 role
bool has_target_node
uint64 target_node_id
geometry_msgs/Pose2D target_pose
bool target_yaw_required
float64 position_tolerance_m
float64 yaw_tolerance_rad
builtin_interfaces/Time deadline
string goal_zone_id
string parameters_id

uint8 HOLD_SAFE = 0
uint8 SEARCH_PORTAL = 1
uint8 INTERCEPT_PORTAL = 2
uint8 PRESSURE_ROUTE = 3
uint8 APPROACH_CAPTURE = 4
uint8 RECOVER_VIEW = 5
uint8 FALLBACK_DEFEND_BASE = 6
uint8 ADVANCE_BASE = 7
uint8 BREAK_LOS = 8
uint8 TAKE_ALTERNATE_PORTAL = 9
uint8 KEEP_ESCAPE_ROUTE = 10
uint8 OBSERVE_SAFE = 11
