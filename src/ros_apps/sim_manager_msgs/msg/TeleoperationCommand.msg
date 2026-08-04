# Validated teleop command — published by HMI bridge node
# Maps to micropilot::system_manager::core::TeleoperationCommand

float64 velocity           # m/s
float64 acceleration       # m/s²
float64 steering_angle     # degrees
float64 steering_velocity  # deg/s
bool    brake

int8    steering_mode

# SteeringMode constants
int8 DISABLE = 0
int8 FRONT_ACKERMAN = 1
int8 DOUBLE_ACKERMAN = 2
int8 CRAB_STEER = 3
int8 FRONT_PARALLEL = 4
int8 DOUBLE_PARALLEL = 5
int8 GO_TO_HOME = 6
int8 CALIBRATION = 7
