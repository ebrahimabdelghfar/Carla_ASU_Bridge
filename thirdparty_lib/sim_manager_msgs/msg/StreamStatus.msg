# Combined stream status — published by HMI bridge node
builtin_interfaces/Time stamp

# Overall state
uint8 cameras_state
uint8 audio_state
bool  server_healthy

# StreamState constants
uint8 STOPPED=0
uint8 STARTING=1
uint8 STREAMING=2
uint8 STATE_ERROR=3

# Per-camera status arrays (parallel arrays, same length)
string[]  camera_serials
uint8[]   camera_healths
bool[]    camera_process_alive
string[]  camera_errors

# Audio status
bool    audio_process_alive
uint8   audio_health
string  audio_error

# StreamHealthStatus constants
uint8 HEALTH_UNKNOWN=0
uint8 HEALTH_HEALTHY=1
uint8 HEALTH_NO_RTP=2
uint8 HEALTH_NOT_PUBLISHED=3
uint8 HEALTH_PROCESS_DOWN=4
uint8 HEALTH_SERVER_DOWN=5
