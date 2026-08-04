# FSM state transition event with full context
# Published on every state transition (success or failure)

uint8 from_state
uint8 to_state
string from_name
string to_name
bool success
string message
builtin_interfaces/Time stamp
