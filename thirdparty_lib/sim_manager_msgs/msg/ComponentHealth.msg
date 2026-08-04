# IDD §10.1 — uniform per-node health (T10: /system_manager/component_health)
string component_id        # Node name (e.g. "camera_driver")
uint8 status
uint64 heartbeat_counter
string message
builtin_interfaces/Time stamp

uint8 UNKNOWN=0
uint8 OK=1
uint8 WARN=2
uint8 ERROR=3
