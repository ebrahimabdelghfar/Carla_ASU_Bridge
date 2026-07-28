# IDD §10.3 — safety risk assessment (T31: /safety/risk_level)
uint8 level
uint8 sar_risk
uint8 health_risk
uint32 failing_components
string[] triggered_scenarios
uint8 selected_fallback
builtin_interfaces/Time stamp

uint8 NOMINAL=0
uint8 DEGRADED=1
uint8 CRITICAL=2
uint8 EMERGENCY=3

uint8 FALLBACK_NONE=0
uint8 FALLBACK_SLOWDOWN=1
uint8 FALLBACK_STOP=2
uint8 FALLBACK_SAFE_PARK=3
uint8 FALLBACK_EMERGENCY_STOP=4
