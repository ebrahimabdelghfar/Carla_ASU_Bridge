# IDD §10.2 — SAR checkpoint report (T30: /safety/sar_checkpoints)
string component_id        # Node name publishing this checkpoint
string sub_component_id    # Component within node (optional, "" if N/A)
uint8 checkpoint_type
uint8 status
builtin_interfaces/Time stamp
string diagnostic_data     # JSON with detection-specific details

uint8 DATA_QUALITY=0
uint8 TIMING=1
uint8 RANGE_CHECK=2
uint8 CONSISTENCY=3
uint8 CONNECTIVITY=4
uint8 FUNCTIONAL=5

uint8 PASS=0
uint8 WARN=1
uint8 FAIL=2
