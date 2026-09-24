hsl_interfaces/ContractHeader meta
string option_instance_id
uint8 phase
bool has_current_node
uint64 current_node_id
float64 remaining_path_m
builtin_interfaces/Duration elapsed
bool eta_valid
builtin_interfaces/Duration eta
string reason
uint64 last_candidate_seq
uint64 last_safety_seq

uint8 IDLE = 0
uint8 PLANNING = 1
uint8 EXECUTING = 2
uint8 REPLANNING = 3
uint8 CANCELING = 4
uint8 FINISHED = 5
