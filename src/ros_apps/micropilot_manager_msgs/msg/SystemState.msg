# Top-level FSM state with numeric ID and human-readable name
# Maps to micropilot::system_manager::core::SystemState enum

uint8 state
string name
builtin_interfaces/Time stamp

# SystemState constants matching the C++ enum
uint8 OFF=0
uint8 INIT=1
uint8 STANDBY=2
uint8 AUTONOMOUS=3
uint8 FREE_DRIVING=4
uint8 FAULT=5
uint8 CHARGING=6
