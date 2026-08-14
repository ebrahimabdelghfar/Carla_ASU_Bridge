# Unified vehicle telemetry snapshot — published by HMI bridge node
builtin_interfaces/Time stamp

# Identity
string vehicle_id

# GPS
float64 gps_latitude
float64 gps_longitude
float64 gps_altitude

# Odometry (local frame, metres)
float64 odom_x
float64 odom_y
float64 odom_yaw

# Orientation (radians)
float64 roll
float64 pitch
float64 yaw

# Motion
float64 speed
string  driving_mode
string  steering_mode

# Power
float64 battery_percentage
float64 temperature

# Connectivity
int32   connection_quality

# Distance / Duration
float64 traveled_distance
float64 odometer
string  elapsed_duration

# Surroundings (metres)
float64 surr_front_center
float64 surr_back_center
float64 surr_front_right
float64 surr_back_right
float64 surr_front_left
float64 surr_back_left

# Lights
bool headlights
bool strobe_light
bool left_indicator
bool right_indicator

# Doors
string door_top
string door_front_right
string door_front_left
string door_back_right
string door_back_left
