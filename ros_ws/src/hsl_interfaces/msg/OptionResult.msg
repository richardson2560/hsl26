hsl_interfaces/ContractHeader meta
string option_instance_id
uint8 outcome
builtin_interfaces/Duration elapsed
string reason
string terminal_event_id

uint8 SUCCESS = 0
uint8 CANCELED = 1
uint8 TIMEOUT = 2
uint8 PRECONDITION_FAILED = 3
uint8 FEASIBILITY_LOST = 4
uint8 SAFETY_STOP = 5
uint8 STAGE_ENDED = 6
uint8 INTERNAL_ERROR = 7
