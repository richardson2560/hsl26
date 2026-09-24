hsl_interfaces/ContractHeader meta
uint8 stage_number
uint8 role
uint8 phase
builtin_interfaces/Time stage_started_at
builtin_interfaces/Time freeze_ends_at
builtin_interfaces/Time stage_ends_at
bool motion_authorized
bool event_hold
uint8 terminal_kind
uint8 terminal_evidence
string score_profile_id
string own_start_zone_id
string target_zone_id
string config_hash

uint8 EXPLORER = 0
uint8 GUARDIAN = 1
uint8 INIT = 0
uint8 FREEZE = 1
uint8 ACTIVE = 2
uint8 TERMINAL = 3
uint8 NONE = 0
uint8 CAPTURE = 1
uint8 ARRIVAL = 2
uint8 TIMEOUT = 3
uint8 OFFICIAL_ABORT = 4
uint8 AMBIGUOUS = 5
uint8 ESTIMATE = 0
uint8 SIM_TRUTH = 1
uint8 OFFICIAL = 2
