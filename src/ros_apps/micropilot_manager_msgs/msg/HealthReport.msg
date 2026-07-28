# Aggregated health report for all registered components

string[] component_names
uint8[] statuses
builtin_interfaces/Time stamp

# HealthStatus constants matching the C++ enum
uint8 UNKNOWN=0
uint8 OK=1
uint8 WARN=2
uint8 ERROR=3
