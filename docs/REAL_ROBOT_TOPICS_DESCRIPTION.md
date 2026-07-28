# Micropilot Vehicle Interface — ROS 2 Topics (ICD Data Sheet)

This document lists every ROS 2 topic used by **`micropilot_vehicle_interface_node`** (`Vehicle_Interface_Node`), in the same field layout as the project Interface Control Document (ICD): topic name, endpoints, message type, behavior, rates, QoS, frames (where applicable), latency notes, and indicative message size.

**Implementation reference:** `src/ros_apps/src/micropilot_vehicle_interface_node/src/vehicle_interface_topics.cpp`

**Topic naming:** Subscribers and publishers are declared with names like `robot/...` (no leading slash). When the node runs in the **root** namespace, tools show them as **`/robot/...`**. Tables below use the fully qualified form for ICD clarity; apply your launch **namespace** when mapping to actual graph names.

---

## Node identity

| Field | Value |
| :--- | :--- |
| **Node name** | `micropilot_vehicle_interface_node` |
| **Node type** | ROS 2 **lifecycle** node (`rclcpp_lifecycle::LifecycleNode`) |
| **Role** | Hardware abstraction: subscribes to motion/pod **commands**, publishes motion/pod **feedback** and health. |

---

## Global QoS profiles (as implemented)

| Role | Reliability | Durability | History |
| :--- | :--- | :--- | :--- |
| **All publishers** (feedback) | Best effort | Transient local | Keep last (depth **1**) |
| **All subscribers** (commands) | Reliable (default) | Volatile (default) | Keep last (depth **1**) |

Rates marked “default” come from `config/vehicle_interface_config.yaml` and can be overridden at launch.

---

## 1. Motion control — command topics (subscribers)

These topics are **inputs to** `micropilot_vehicle_interface_node`. The node is the **destination**. Sources are typically autonomy, teleop, or test nodes (exact names are system-dependent).

Command application rate is **adaptive**: up to `command_send_rate_active_hz` (default **100 Hz**) when moving, and `command_send_rate_idle_hz` (default **10 Hz**) when idle (velocity near zero and brake engaged). Individual scalar topics update internal state as messages arrive; aggregated JSON on `robot/control/drive` replaces the full drive struct when used.

### 1.1 `robot/control/velocity`

| Field | Value |
| :--- | :--- |
| **Topic** | `/robot/control/velocity` |
| **Source Node/s** | Autonomy / teleop / test (e.g. velocity controller) |
| **Destination Node** | `micropilot_vehicle_interface_node` |
| **Message Type** | `std_msgs/msg/Float32` |
| **Description** | Wheel velocity command in **RPM** (`data` → internal drive command `velocity_rpm`). |
| **Expected publish rate** | Up to **100 Hz** (typical when active); aligns with command timer when combined with other motion inputs. |
| **QoS Reliability** | Reliable |
| **QoS Durability** | Volatile |
| **QoS History** | Keep last (depth 1) |
| **Coordinate Frame** | — |
| **Latency requirement** | **&lt; 20 ms** end-to-end recommended for tight control loops (10 ms nominal at 100 Hz). |
| **Estimated message size** | **~16 bytes** (Float32 payload + overhead). |

### 1.2 `robot/control/acceleration`

| Field | Value |
| :--- | :--- |
| **Topic** | `/robot/control/acceleration` |
| **Source Node/s** | Autonomy / teleop / test |
| **Destination Node** | `micropilot_vehicle_interface_node` |
| **Message Type** | `std_msgs/msg/Float32` |
| **Description** | Acceleration command in **RPM/s** (`data` → `acceleration_rpm_s`). If zero, node may substitute configured max acceleration clamp. |
| **Expected publish rate** | Up to **100 Hz** when active. |
| **QoS Reliability** | Reliable |
| **QoS Durability** | Volatile |
| **QoS History** | Keep last (depth 1) |
| **Coordinate Frame** | — |
| **Latency requirement** | **&lt; 20 ms** recommended. |
| **Estimated message size** | **~16 bytes**. |

### 1.3 `robot/control/steering_angle`

| Field | Value |
| :--- | :--- |
| **Topic** | `/robot/control/steering_angle` |
| **Source Node/s** | Autonomy / teleop / test |
| **Destination Node** | `micropilot_vehicle_interface_node` |
| **Message Type** | `std_msgs/msg/Float32` |
| **Description** | Steering angle command in **degrees** (`data` → `steering_angle_deg`). Clamped to `max_steering_angle_deg` (parameter). |
| **Expected publish rate** | Up to **100 Hz** when active. |
| **QoS Reliability** | Reliable |
| **QoS Durability** | Volatile |
| **QoS History** | Keep last (depth 1) |
| **Coordinate Frame** | — |
| **Latency requirement** | **&lt; 20 ms** recommended. |
| **Estimated message size** | **~16 bytes**. |

### 1.4 `robot/control/steering_angular_velocity`

| Field | Value |
| :--- | :--- |
| **Topic** | `/robot/control/steering_angular_velocity` |
| **Source Node/s** | Autonomy / teleop / test |
| **Destination Node** | `micropilot_vehicle_interface_node` |
| **Message Type** | `std_msgs/msg/Float32` |
| **Description** | Steering angular velocity in **deg/s** (`data` → `steering_angular_velocity_deg_s`). If zero, node may substitute configured max rate clamp. |
| **Expected publish rate** | Up to **100 Hz** when active. |
| **QoS Reliability** | Reliable |
| **QoS Durability** | Volatile |
| **QoS History** | Keep last (depth 1) |
| **Coordinate Frame** | — |
| **Latency requirement** | **&lt; 20 ms** recommended. |
| **Estimated message size** | **~16 bytes**. |

### 1.5 `robot/control/drive`

| Field | Value |
| :--- | :--- |
| **Topic** | `/robot/control/drive` |
| **Source Node/s** | Autonomy / teleop / test |
| **Destination Node** | `micropilot_vehicle_interface_node` |
| **Message Type** | `std_msgs/msg/String` (**JSON** payload) |
| **Description** | Single JSON object setting the full drive command: `velocity_rpm`, `acceleration_rpm_s`, `steering_angle_deg`, `steering_angular_velocity_deg_s` (see `Vehicle::JSON::from_json_to_drive_message` in `Vehicle_types.hpp`). |
| **Expected publish rate** | Up to **100 Hz** when active. |
| **QoS Reliability** | Reliable |
| **QoS Durability** | Volatile |
| **QoS History** | Keep last (depth 1) |
| **Coordinate Frame** | — |
| **Latency requirement** | **&lt; 20 ms** recommended. |
| **Estimated message size** | **~150–250 bytes** (typical JSON string). |

### 1.6 `robot/control/brake`

| Field | Value |
| :--- | :--- |
| **Topic** | `/robot/control/brake` |
| **Source Node/s** | Autonomy / teleop / test |
| **Destination Node** | `micropilot_vehicle_interface_node` |
| **Message Type** | `std_msgs/msg/Bool` |
| **Description** | Emergency brake request (`true` = brake engaged). Used with velocity threshold to determine idle vs active command rate. |
| **Expected publish rate** | Up to **100 Hz** when active. |
| **QoS Reliability** | Reliable |
| **QoS Durability** | Volatile |
| **QoS History** | Keep last (depth 1) |
| **Coordinate Frame** | — |
| **Latency requirement** | **&lt; 20 ms** recommended. |
| **Estimated message size** | **~8 bytes** (Bool payload + overhead). |

### 1.7 `robot/control/steering_mode`

| Field | Value |
| :--- | :--- |
| **Topic** | `/robot/control/steering_mode` |
| **Source Node/s** | Autonomy / teleop / test |
| **Destination Node** | `micropilot_vehicle_interface_node` |
| **Message Type** | `std_msgs/msg/Int8` |
| **Description** | Steering mode enum as integer (`Vehicle::Control::SteeringMode_t`). `STEERING_DISABLE` may be mapped to `STEERING_DOUBLE_ACKERMAN` before send. |
| **Expected publish rate** | On change or low rate (implementation sends only when mode changes). |
| **QoS Reliability** | Reliable |
| **QoS Durability** | Volatile |
| **QoS History** | Keep last (depth 1) |
| **Coordinate Frame** | — |
| **Latency requirement** | **&lt; 100 ms** typical (mode changes are infrequent). |
| **Estimated message size** | **~8 bytes**. |

---

## 2. Motion feedback — publisher topics

**Source node:** `micropilot_vehicle_interface_node`. **Destination node/s:** logging, state estimation, HMI, safety monitors (system-specific).

Publish cadence is driven by the feedback timer: **`feedback_publish_rate_hz`** (default **50 Hz**).

### 2.1 `robot/feedback/motors_data`

| Field | Value |
| :--- | :--- |
| **Topic** | `/robot/feedback/motors_data` |
| **Source Node** | `micropilot_vehicle_interface_node` |
| **Destination Node/s** | Diagnostics / control / logging |
| **Message Type** | `std_msgs/msg/String` (**JSON**) |
| **Description** | Per-wheel feedback: `timestamp`, `frame_id` (`"Motors_Data"`), and `front_left` / `front_right` / `back_left` / `back_right` wheel objects (speed, steering, brake, power, torque, errors). |
| **Publish rate** | **50 Hz** (default; configurable) |
| **QoS Reliability** | Best effort |
| **QoS Durability** | Transient local |
| **QoS History** | Keep last (depth 1) |
| **Coordinate Frame** | JSON `frame_id`: **`Motors_Data`** |
| **Latency requirement** | Freshness within one feedback period: **&lt; 20 ms** at 50 Hz (plus transport). |
| **Estimated message size** | **~1.5–3 KB** (typical JSON). |

**JSON layout:** Root object has `timestamp` (ms, integer), `frame_id` (`"Motors_Data"`), and four wheel objects: `front_left`, `front_right`, `back_left`, `back_right`. Each wheel object contains numeric fields such as `speed_rpm`, `speed_mps`, `steering_angle_deg`, `steering_angle_rad`, `steering_angular_velocity`, `steering_angular_velocity_radps`, `brake_percentage`, `power`, `torque`, `drive_motor_error`, `steering_motor_error`, `brake_motor_error` (see `Vehicle::JSON::from_wheel_state` / `from_motor_data_to_json` in `Vehicle_types.hpp`).

**Example — parse the string payload:** Subscribe to `robot/feedback/motors_data` (`std_msgs/msg/String`); the JSON is in `msg.data`. Below, generic scalars illustrate reading root and one wheel; repeat the wheel pattern for `front_right`, `back_left`, `back_right` as needed.

C++ (nlohmann/json):

```cpp
#include <nlohmann/json.hpp>
#include <std_msgs/msg/string.hpp>

void on_motors_data(const std_msgs::msg::String::SharedPtr msg)
{
  try {
    const auto j = nlohmann::json::parse(msg->data);
    int64_t timestamp_ms = j.value("timestamp", int64_t{0});
    std::string frame_id = j.value("frame_id", std::string{});

    const auto &wheel = j.at("front_left");  // or "front_right", "back_left", "back_right"
    float speed_rpm = wheel.value("speed_rpm", 0.0f);
    float brake_pct = wheel.value("brake_percentage", 0.0f);
    int drive_err = wheel.value("drive_motor_error", 0);
    // use timestamp_ms, frame_id, speed_rpm, brake_pct, drive_err …
  } catch (const std::exception &) {
    // malformed JSON or missing keys — handle per application
  }
}
```
### 2.2 `robot/feedback/battery`

| Field | Value |
| :--- | :--- |
| **Topic** | `/robot/feedback/battery` |
| **Source Node** | `micropilot_vehicle_interface_node` |
| **Destination Node/s** | Power management / HMI |
| **Message Type** | `sensor_msgs/msg/BatteryState` |
| **Description** | Voltage (V), current (A), temperature (°C), charge **percentage**. |
| **Publish rate** | **50 Hz** (default) |
| **QoS Reliability** | Best effort |
| **QoS Durability** | Transient local |
| **QoS History** | Keep last (depth 1) |
| **Coordinate Frame** | `header.frame_id`: **`Battery_State`** |
| **Latency requirement** | **&lt; 50 ms** typical. |
| **Estimated message size** | **~200–400 bytes**. |

### 2.3 `robot/feedback/imu`

| Field | Value |
| :--- | :--- |
| **Topic** | `/robot/feedback/imu` |
| **Source Node** | `micropilot_vehicle_interface_node` |
| **Destination Node/s** | Localization / filtering / logging |
| **Message Type** | `sensor_msgs/msg/Imu` |
| **Description** | Orientation fields carry roll/pitch/yaw **radians** in `orientation.x/y/z` with `orientation.w = 0` (non-standard; consumers should treat per project convention). Angular velocity and linear acceleration filled from platform feedback. |
| **Publish rate** | **50 Hz** (default) |
| **QoS Reliability** | Best effort |
| **QoS Durability** | Transient local |
| **QoS History** | Keep last (depth 1) |
| **Coordinate Frame** | `header.frame_id`: **`IMU`** |
| **Latency requirement** | **&lt; 50 ms** typical. |
| **Estimated message size** | **~300–600 bytes**. |

### 2.4 `robot/feedback/robot_speed_rpm`

| Field | Value |
| :--- | :--- |
| **Topic** | `/robot/feedback/robot_speed_rpm` |
| **Source Node** | `micropilot_vehicle_interface_node` |
| **Destination Node/s** | Control / HMI |
| **Message Type** | `std_msgs/msg/Float32` |
| **Description** | Robot speed in **RPM**. |
| **Publish rate** | **50 Hz** (default) |
| **QoS Reliability** | Best effort |
| **QoS Durability** | Transient local |
| **QoS History** | Keep last (depth 1) |
| **Coordinate Frame** | — |
| **Latency requirement** | **&lt; 50 ms** typical. |
| **Estimated message size** | **~16 bytes**. |

### 2.5 `robot/feedback/robot_speed_mps`

| Field | Value |
| :--- | :--- |
| **Topic** | `/robot/feedback/robot_speed_mps` |
| **Source Node** | `micropilot_vehicle_interface_node` |
| **Destination Node/s** | Control / HMI |
| **Message Type** | `std_msgs/msg/Float32` |
| **Description** | Robot speed in **m/s**. |
| **Publish rate** | **50 Hz** (default) |
| **QoS Reliability** | Best effort |
| **QoS Durability** | Transient local |
| **QoS History** | Keep last (depth 1) |
| **Coordinate Frame** | — |
| **Latency requirement** | **&lt; 50 ms** typical. |
| **Estimated message size** | **~16 bytes**. |

### 2.6 `robot/feedback/robot_avg_steering_angle_deg`

| Field | Value |
| :--- | :--- |
| **Topic** | `/robot/feedback/robot_avg_steering_angle_deg` |
| **Source Node** | `micropilot_vehicle_interface_node` |
| **Destination Node/s** | Control / logging |
| **Message Type** | `std_msgs/msg/Float32` |
| **Description** | Average of front-left and front-right measured steering angles (**degrees**). |
| **Publish rate** | **50 Hz** (default; same timer as motors JSON) |
| **QoS Reliability** | Best effort |
| **QoS Durability** | Transient local |
| **QoS History** | Keep last (depth 1) |
| **Coordinate Frame** | — |
| **Latency requirement** | **&lt; 50 ms** typical. |
| **Estimated message size** | **~16 bytes**. |

### 2.7 `robot/feedback/steering_health_check`

| Field | Value |
| :--- | :--- |
| **Topic** | `/robot/feedback/steering_health_check` |
| **Source Node** | `micropilot_vehicle_interface_node` |
| **Destination Node/s** | Safety / supervisor |
| **Message Type** | `std_msgs/msg/Bool` |
| **Description** | Steering subsystem health flag from platform (`true` = healthy when available). |
| **Publish rate** | **50 Hz** (default) when feedback is retrieved |
| **QoS Reliability** | Best effort |
| **QoS Durability** | Transient local |
| **QoS History** | Keep last (depth 1) |
| **Coordinate Frame** | — |
| **Latency requirement** | **&lt; 100 ms** typical. |
| **Estimated message size** | **~8 bytes**. |

### 2.8 `robot/feedback/braking_health_check`

| Field | Value |
| :--- | :--- |
| **Topic** | `/robot/feedback/braking_health_check` |
| **Source Node** | `micropilot_vehicle_interface_node` |
| **Destination Node/s** | Safety / supervisor |
| **Message Type** | `std_msgs/msg/Bool` |
| **Description** | Braking subsystem health flag from platform. |
| **Publish rate** | **50 Hz** (default) when feedback is retrieved |
| **QoS Reliability** | Best effort |
| **QoS Durability** | Transient local |
| **QoS History** | Keep last (depth 1) |
| **Coordinate Frame** | — |
| **Latency requirement** | **&lt; 100 ms** typical. |
| **Estimated message size** | **~8 bytes**. |

### 2.9 `robot/feedback/steering_mode`

| Field | Value |
| :--- | :--- |
| **Topic** | `/robot/feedback/steering_mode` |
| **Source Node** | `micropilot_vehicle_interface_node` |
| **Destination Node/s** | Control / logging |
| **Message Type** | `std_msgs/msg/Int32` |
| **Description** | Current steering mode as reported by platform (`SteeringMode` enum as integer). **Note:** command uses `Int8`; feedback uses `Int32`. |
| **Publish rate** | **50 Hz** (default) when available |
| **QoS Reliability** | Best effort |
| **QoS Durability** | Transient local |
| **QoS History** | Keep last (depth 1) |
| **Coordinate Frame** | — |
| **Latency requirement** | **&lt; 100 ms** typical. |
| **Estimated message size** | **~16 bytes**. |

### 2.10 `robot/feedback/autonomous_status`

| Field | Value |
| :--- | :--- |
| **Topic** | `/robot/feedback/autonomous_status` |
| **Source Node** | `micropilot_vehicle_interface_node` |
| **Destination Node/s** | Supervisor / HMI |
| **Message Type** | `std_msgs/msg/Bool` |
| **Description** | Autonomous mode status from platform when available. |
| **Publish rate** | **50 Hz** (default) when available |
| **QoS Reliability** | Best effort |
| **QoS Durability** | Transient local |
| **QoS History** | Keep last (depth 1) |
| **Coordinate Frame** | — |
| **Latency requirement** | **&lt; 100 ms** typical. |
| **Estimated message size** | **~8 bytes**. |

### 2.11 `robot/feedback/can_health_bool`

| Field | Value |
| :--- | :--- |
| **Topic** | `/robot/feedback/can_health_bool` |
| **Source Node** | `micropilot_vehicle_interface_node` |
| **Destination Node/s** | Watchdog / health aggregator |
| **Message Type** | `std_msgs/msg/Bool` |
| **Description** | **`true`** if feedback heartbeat is within `feedback_timeout_ms` (default **300 ms**); **`false`** if stale. |
| **Publish rate** | **50 Hz** (default) |
| **QoS Reliability** | Best effort |
| **QoS Durability** | Transient local |
| **QoS History** | Keep last (depth 1) |
| **Coordinate Frame** | — |
| **Latency requirement** | Should reflect heartbeat: **&lt; `feedback_timeout_ms`** for valid **true** semantics. |
| **Estimated message size** | **~8 bytes**. |

### 2.12 `robot/feedback/ros_node_status`

| Field | Value |
| :--- | :--- |
| **Topic** | `/robot/feedback/ros_node_status` |
| **Source Node** | `micropilot_vehicle_interface_node` |
| **Destination Node/s** | Supervisor / diagnostics |
| **Message Type** | `std_msgs/msg/Bool` |
| **Description** | Published **`true`** while the feedback callback runs (node active path). |
| **Publish rate** | **50 Hz** (default) |
| **QoS Reliability** | Best effort |
| **QoS Durability** | Transient local |
| **QoS History** | Keep last (depth 1) |
| **Coordinate Frame** | — |
| **Latency requirement** | **&lt; 100 ms** typical. |
| **Estimated message size** | **~8 bytes**. |

---

## 3. Pod command topics (subscribers, conditional)

Created only if **`pod_model_type`** supports the capability (see `PodFactory` / `vehicle_interface_config.yaml`).

| `pod_model_type` | Siren | Lights | Doors | Drone |
| :---: | :---: | :---: | :---: | :---: |
| NONE | — | — | — | — |
| PATROL | ✓ | ✓ | — | — |
| EDGE | ✓ | ✓ | ✓ | — |
| DRONE | ✓ | ✓ | ✓ | ✓ |
| DELIVERY | — | ✓ | ✓ | — |

### 3.1 `robot/pod/siren`

| Field | Value |
| :--- | :--- |
| **Topic** | `/robot/pod/siren` |
| **Source Node/s** | Pod / mission commander (when capability enabled) |
| **Destination Node** | `micropilot_vehicle_interface_node` |
| **Message Type** | `std_msgs/msg/String` (**JSON**: `horn`, `siren`) |
| **Description** | Horn and siren requests. Siren edge triggers an internal pulse state machine (`siren_pulse_duration_ms`). |
| **Subscriber exists when** | PATROL, EDGE, DRONE |
| **Expected publish rate** | Event-driven / low rate |
| **QoS Reliability** | Reliable |
| **QoS Durability** | Volatile |
| **QoS History** | Keep last (depth 1) |
| **Coordinate Frame** | — |
| **Latency requirement** | **&lt; 100 ms** typical. |
| **Estimated message size** | **&lt; 100 bytes**. |

### 3.2 `robot/pod/lights`

| Field | Value |
| :--- | :--- |
| **Topic** | `/robot/pod/lights` |
| **Source Node/s** | Pod / mission commander |
| **Destination Node** | `micropilot_vehicle_interface_node` |
| **Message Type** | `std_msgs/msg/String` (**JSON**: head/night/brake/reverse/strobe/left/right indicators) |
| **Description** | Pod lighting bitmask; sent on change after callback. |
| **Subscriber exists when** | PATROL, EDGE, DRONE, DELIVERY |
| **Expected publish rate** | On change |
| **QoS Reliability** | Reliable |
| **QoS Durability** | Volatile |
| **QoS History** | Keep last (depth 1) |
| **Coordinate Frame** | — |
| **Latency requirement** | **&lt; 100 ms** typical. |
| **Estimated message size** | **~200–400 bytes**. |

### 3.3 `robot/pod/door`

| Field | Value |
| :--- | :--- |
| **Topic** | `/robot/pod/door` |
| **Source Node/s** | Pod / mission commander |
| **Destination Node** | `micropilot_vehicle_interface_node` |
| **Message Type** | `std_msgs/msg/String` (**JSON**: `front_open`, `rear_open`, `left_open`, `right_open`) |
| **Description** | Door commands; sent on change after callback. |
| **Subscriber exists when** | EDGE, DRONE, DELIVERY |
| **Expected publish rate** | On change |
| **QoS Reliability** | Reliable |
| **QoS Durability** | Volatile |
| **QoS History** | Keep last (depth 1) |
| **Coordinate Frame** | — |
| **Latency requirement** | **&lt; 100 ms** typical. |
| **Estimated message size** | **~150–250 bytes**. |

### 3.4 `robot/pod/drone`

| Field | Value |
| :--- | :--- |
| **Topic** | `/robot/pod/drone` |
| **Source Node/s** | Pod / mission commander |
| **Destination Node** | `micropilot_vehicle_interface_node` |
| **Message Type** | `std_msgs/msg/String` (**JSON**: `launch`, `drone_id`) |
| **Description** | Drone launch request; sent on change after callback. |
| **Subscriber exists when** | DRONE only |
| **Expected publish rate** | On change |
| **QoS Reliability** | Reliable |
| **QoS Durability** | Volatile |
| **QoS History** | Keep last (depth 1) |
| **Coordinate Frame** | — |
| **Latency requirement** | **&lt; 100 ms** typical. |
| **Estimated message size** | **~80–150 bytes**. |

---

## 4. Pod feedback — publisher topic

### 4.1 `robot/feedback/pod_status`

| Field | Value |
| :--- | :--- |
| **Topic** | `/robot/feedback/pod_status` |
| **Source Node** | `micropilot_vehicle_interface_node` |
| **Destination Node/s** | HMI / mission logic |
| **Message Type** | `std_msgs/msg/String` (**JSON**) |
| **Description** | `siren_active`, `doors_locked`, `drone_ready`, `drone_airborne`. **Not published** when `pod_model_type` is **NONE** (callback returns early). |
| **Publish rate** | **50 Hz** timer path when pod is non-NONE and transport returns status |
| **QoS Reliability** | Best effort |
| **QoS Durability** | Transient local |
| **QoS History** | Keep last (depth 1) |
| **Coordinate Frame** | — |
| **Latency requirement** | **&lt; 50 ms** typical (aligned with feedback rate). |
| **Estimated message size** | **~120–200 bytes**. |

---

## 5. Topic index (quick reference)

**Subscriptions (commands):**

- `/robot/control/velocity`
- `/robot/control/acceleration`
- `/robot/control/steering_angle`
- `/robot/control/steering_angular_velocity`
- `/robot/control/drive`
- `/robot/control/brake`
- `/robot/control/steering_mode`
- `/robot/pod/siren` (conditional)
- `/robot/pod/lights` (conditional)
- `/robot/pod/door` (conditional)
- `/robot/pod/drone` (conditional)

**Publications (feedback / health):**

- `/robot/feedback/motors_data`
- `/robot/feedback/battery`
- `/robot/feedback/imu`
- `/robot/feedback/robot_speed_rpm`
- `/robot/feedback/robot_speed_mps`
- `/robot/feedback/robot_avg_steering_angle_deg`
- `/robot/feedback/steering_health_check`
- `/robot/feedback/braking_health_check`
- `/robot/feedback/steering_mode`
- `/robot/feedback/autonomous_status`
- `/robot/feedback/can_health_bool`
- `/robot/feedback/ros_node_status`
- `/robot/feedback/pod_status`

---

*Generated to match `vehicle_interface_topics.cpp` QoS and topic names. Destination/source node names for external stacks should be filled in per deployment ICD.*
