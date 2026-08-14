# Operator session state — published by HMI bridge node

builtin_interfaces/Time stamp

string  operator_id
uint8   state
float64 connected_since     # epoch seconds
float64 last_command_time   # epoch seconds
string  control_mode        # "keyboard" / "joystick" / "gamepad"

# OperatorSessionState constants
uint8 DISCONNECTED=0
uint8 CONNECTED=1
uint8 ACTIVE=2
uint8 IDLE=3
