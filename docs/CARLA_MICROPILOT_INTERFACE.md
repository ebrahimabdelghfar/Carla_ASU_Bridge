# carla_micropilot_interface

Bridge node between `micropilot_vehicle_interface` SimulationTransport (`/sim/control/*` and `/sim/feedback/*`) and the CARLA ROS 2 bridge (`/carla/<ego_role>/…`).

The node implements the simulation transport contract defined in `micropilot_vehicle_interface/docs/simulation_interface.md` so that the autonomy stack can drive a CARLA ego vehicle without any CARLA-specific code in the vehicle interface repository.

---

## Architecture

```text
micropilot_vehicle_interface_node  (interface_type=3)
          │
          ▼
SimulationTransport
          │  /sim/control/*
          ▼
carla_micropilot_interface_node          ◄── /carla/hero/speedometer
          │
          │  /carla/hero/vehicle_control_cmd
          ▼
CARLA ROS 2 Bridge  ──►  CARLA Simulator (ego vehicle)
          │
          ▼
/sim/feedback/speed            ──►  SimulationTransport
/sim/feedback/steering_angle   ──►  SimulationTransport
```

---

## Topic contract

### Subscriptions (inputs)

| Topic | Type | Source | Description |
|-------|------|--------|-------------|
| `/sim/control/velocity_rpm` | `std_msgs/Float32` | SimulationTransport | Wheel velocity command in RPM. Negative = reverse. |
| `/sim/control/steering_angle_deg` | `std_msgs/Float32` | SimulationTransport | Steering angle in degrees. `+` = left, `−` = right (ICD convention). |
| `/sim/control/brake` | `std_msgs/Bool` | SimulationTransport | `true` = full emergency brake. Zeroes throttle and steer. |
| `/carla/hero/speedometer` | `std_msgs/Float32` | CARLA ROS bridge | Current ego vehicle speed in m/s. |

### Publications (outputs)

| Topic | Type | Destination | Description |
|-------|------|-------------|-------------|
| `/carla/hero/vehicle_control_cmd` | `carla_msgs/CarlaEgoVehicleControl` | CARLA ROS bridge | Throttle, steer, brake, reverse command. |
| `/sim/feedback/speed` | `std_msgs/Float32` | SimulationTransport | Speed in m/s, forwarded directly from CARLA speedometer. |
| `/sim/feedback/steering_angle` | `std_msgs/Float32` | SimulationTransport | Steering angle in degrees, echoed from command. |

> **QoS:** All publishers and subscribers use `KeepLast(10)` to match `SimulationTransport` defaults.

---

## Signal conversions

### RPM → CARLA throttle

Linear mapping over `[0, max_rpm]` → `[0.0, 1.0]`:

```
throttle = clamp(|rpm| / max_rpm, 0.0, 1.0)
```

- Negative RPM sets `reverse = true` in the control message.
- `max_rpm` is a configurable parameter (default: `150.0`).

### Steering degrees → CARLA steer

Linear mapping over `[−max_steering_angle_deg, +max_steering_angle_deg]` → `[+1.0, −1.0]`:

```
steer = clamp(−deg / max_steering_angle_deg, −1.0, +1.0)
```

Sign is **inverted** because the ICD convention (`+` = left) is opposite to CARLA's convention (`+` = right).

- `max_steering_angle_deg` is a configurable parameter (default: `16.0`).

### Brake

When `/sim/control/brake` is `true`: `throttle = 0.0`, `steer = 0.0`, `brake = 1.0`. Velocity and steering state are preserved internally and resume on release.

---

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `max_rpm` | `double` | `150.0` | Wheel RPM that maps to CARLA throttle `1.0`. Set to platform max. |
| `max_steering_angle_deg` | `double` | `16.0` | Absolute steering limit in degrees (`±`). Maps to CARLA steer `±1.0`. |
| `carla_role_name` | `string` | `"hero"` | CARLA ego vehicle role name. Must match the value in `carla_ros_bridge`. |

All parameters are declared in `config/carla_micropilot_interface.yaml` and overridable from the launch file.

---

## Build

From the `micropilot_sim` repo root:

```bash
source /opt/ros/humble/setup.bash
cd scripts/ros_apps_build
bash colcon_build.sh
```

`carla_msgs` (vendored in `thirdparty_lib/ros-carla-msgs`) is built first automatically. Source the install overlay before running:

```bash
source install/ros_apps/setup.bash
```

---

## Running

Start the following in order, one terminal each.

**Terminal 1 — CARLA simulator:**

```bash
cd ~/carla
./CarlaUE4.sh -quality-level=Low -windowed -nosound
```

**Terminal 2 — CARLA ROS 2 bridge:**

```bash
source /opt/ros/humble/setup.bash
ros2 launch carla_ros_bridge carla_ros_bridge_with_example_ego_vehicle.launch.py town:=Town01
```

**Terminal 3 — this bridge node:**

```bash
source /opt/ros/humble/setup.bash
source install/ros_apps/setup.bash
ros2 launch carla_micropilot_interface carla_micropilot_interface.launch.py
```

**Terminal 4 — micropilot vehicle interface (simulation mode):**

```bash
ros2 launch micropilot_vehicle_interface_node simulation_interface.launch.py
```

### Launch arguments

```bash
ros2 launch carla_micropilot_interface carla_micropilot_interface.launch.py \
  max_rpm:=150.0 \
  max_steering_angle_deg:=16.0 \
  carla_role_name:=hero
```

---

## Verification

```bash
# Confirm speed feedback is flowing
ros2 topic echo /sim/feedback/speed

# Confirm control commands are being sent to CARLA
ros2 topic echo /carla/hero/vehicle_control_cmd

# Check publish rate
ros2 topic hz /sim/feedback/speed
```

### Manual test commands

```bash
# Drive forward at half throttle (75 RPM of 150 max → throttle 0.5)
ros2 topic pub /sim/control/velocity_rpm std_msgs/Float32 "data: 75.0"

# Steer left 8 degrees (half of ±16 limit → CARLA steer −0.5)
ros2 topic pub /sim/control/steering_angle_deg std_msgs/Float32 "data: 8.0"

# Apply emergency brake
ros2 topic pub /sim/control/brake std_msgs/Bool "data: true"
```

### Expected CARLA values

| Input | Expected CARLA field |
|-------|----------------------|
| `velocity_rpm = 75.0` | `throttle = 0.5` |
| `steering_angle_deg = 8.0` | `steer = −0.5` (sign inverted) |
| `brake = true` | `brake = 1.0`, `throttle = 0.0` |

---

## Known limitations

- **Steering feedback is echoed from command** — CARLA does not publish a measured steering angle topic. `SimulationTransport::getMotorsData()` therefore reflects command steering, not true wheel angle.
- **IMU, battery, limit switches, pod status** are not populated — consistent with `SimulationTransport` placeholders documented in `simulation_interface.md`.
- **`/sim/control/acceleration_rpm_s`** and **`/sim/control/steering_angular_velocity_deg_s`** are not consumed — CARLA has no direct acceleration or steering rate input; throttle ramp is handled by CARLA's internal physics.

---

## Related documents

| Document | Location |
|----------|----------|
| Simulation transport contract (ICD) | `micropilot_vehicle_interface/docs/simulation_interface.md` |
| micropilot_vehicle_interface README | `micropilot_vehicle_interface/README.md` |
| CARLA ROS 2 bridge documentation | https://carla.readthedocs.io/projects/ros-bridge/en/latest/ |
