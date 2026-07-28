# micropilot_sim

Simulation workspace for **micropilot** — ROS 2 bridge packages and supporting assets that connect the micropilot autonomy stack to external driving simulators.

Each simulator is isolated behind its own bridge package implementing the `/sim/control/*` → `/sim/feedback/*` contract defined in `micropilot_vehicle_interface`. The autonomy stack remains unchanged regardless of which simulator is active.

---

## Simulators

| Simulator | Bridge package | Status | Notes |
|-----------|---------------|--------|-------|
| **CARLA (Standard)** | `carla_micropilot_interface` | ✅ Available | Connects using standard `carla_ros_bridge` |
| **CARLA (Custom/Telemetry)** | `carla_telemetry_cpp` / `carla_telemetry` | ✅ Available | High-performance C++ lifecycle telemetry node supporting Scenario Runner, active sensors, and custom PhysX overrides |
| **Isaac Sim** | `isaac_sim_micropilot_interface` | ✅ Available | Available as OmniGraph embedded inside the USD |

---

## Repository layout

```text
micropilot_sim/
├── docs/
│   ├── CARLA_CUSTOM_BRIDGE_.md         # CARLA Custom C++ lifecycle telemetry bridge reference
│   ├── CARLA_MICROPILOT_INTERFACE.md   # Standard CARLA bridge reference (topics, parameters, launch)
│   ├── ISAAC_SIM_INTERFACE_MICROPILOT_INTERFACE.md # Isaac Sim bridge reference
│   ├── REAL_ROBOT_TOPICS_DESCRIPTION.md # Real robot topic contract reference
│   └── SCENARIO_RUNNER.md              # CARLA Scenario Runner & Telemetry Bridge integration
├── scripts/
│   ├── download_assests/
│   │   └── download_carla_micropolis.sh # Helper script to download CARLA simulator assets
│   ├── operate_sim/
│   │   └── run_isaac_sim.sh            # Run Isaac Sim environment helper
│   └── ros_apps_build/
│       ├── colcon_build.sh             # Build entry point
│       └── config_colcon.yaml          # Colcon base-paths, build and install locations
├── src/
│   └── ros_apps/
│       ├── carla_micropilot_interface/ # Standard CARLA ↔ micropilot bridge package
│       ├── carla_telemetry/            # Custom Python telemetry bridge implementation
│       ├── carla_telemetry_cpp/        # High-performance C++ lifecycle telemetry bridge
│       └── micropilot_manager_msgs/    # Manager messages for simulation components health
├── data/
│   └── environments/                   # Simulation environment assets (USDZ, FBX, textures)
└── thirdparty_lib/                     # Git submodules
    ├── ros-carla-msgs/                 # Vendored carla_msgs — no external install required
    ├── kit-extension-template-cpp/     # Omniverse Kit C++ baseline for Isaac Sim extensions
    ├── OpenUSD/
    └── PhysX/
```

---

## Prerequisites

- ROS 2 Humble on Ubuntu 22.04
- Submodules initialised:

  ```bash
  git submodule update --init --recursive
  ```

---

## Build

To compile the ROS 2 workspace and build all bridge packages (including `carla_telemetry_cpp` and standard interfaces):

```bash
source /opt/ros/humble/setup.bash
make setup_ros2_workspace
```

Or compile using the underlying script directly:

```bash
source /opt/ros/humble/setup.bash
cd scripts/ros_apps_build
bash colcon_build.sh
```

Source the install overlay before running nodes:

```bash
source install/ros_apps/setup.bash
```

---

## Quick Start

### 1. Standard CARLA Bridge

```bash
# 1. Start CARLA
./CarlaUE4.sh -quality-level=Low -windowed -nosound

# 2. Start CARLA ROS 2 bridge
ros2 launch carla_ros_bridge carla_ros_bridge_with_example_ego_vehicle.launch.py town:=Town01

# 3. Start micropilot bridge
ros2 launch carla_micropilot_interface carla_micropilot_interface.launch.py

# 4. Start micropilot vehicle interface in simulation mode
ros2 launch micropilot_vehicle_interface_node simulation_interface.launch.py
```

### 2. High-Performance C++ Telemetry Bridge (`carla_telemetry_cpp`)

The custom lifecycle telemetry bridge interacts directly with the CARLA C++ API to stream high-frequency sensor telemetry (cameras, LiDAR, IMU, GPS, battery) without standard bridge overhead.

```bash
# 1. Download CARLA simulator assets (if not already present)
make download_Carla_Assets

# 2. Launch CARLA and the telemetry bridge node (starts in Unconfigured state)
make launch_carla_sim

# 3. Transition the lifecycle node to active in a separate terminal
ros2 lifecycle set /micropilot_carla_bridge_node configure
ros2 lifecycle set /micropilot_carla_bridge_node activate
```

Alternatively, launch the node to configure and activate automatically:
```bash
make launch_carla_sim AUTO_START=true
```

### 3. Coordinated Scenario Runner Integration

To avoid simulator crashes during map/world reloads while streaming sensor data, the Python **Scenario Runner** coordinates map loading with the **C++ Telemetry Bridge** using ROS 2 Lifecycle transitions.

```bash
# 1. Start the C++ Telemetry Node (starts in Unconfigured state)
make launch_carla_sim

# 2. Execute Scenario Runner with coordination arguments in a separate terminal
python3 scenario_runner.py \
  --scenario FollowLeadingVehicle_1 \
  --reloadWorld \
  --waitForEgo
```

---

## Documentation

| Document | Description |
|----------|-------------|
| [`docs/CARLA_CUSTOM_BRIDGE_.md`](docs/CARLA_CUSTOM_BRIDGE_.md) | Custom CARLA C++ Lifecycle Telemetry Bridge reference — architecture, configuration, topics, lifecycle states, and build |
| [`docs/SCENARIO_RUNNER.md`](docs/SCENARIO_RUNNER.md) | CARLA Scenario Runner & Telemetry Bridge integration — coordinated reload sequence and implementation |
| [`docs/CARLA_MICROPILOT_INTERFACE.md`](docs/CARLA_MICROPILOT_INTERFACE.md) | Standard CARLA bridge reference — topic contract, signal conversions, parameters, and manual verification |
| [`docs/ISAAC_SIM_INTERFACE_MICROPILOT_INTERFACE.md`](docs/ISAAC_SIM_INTERFACE_MICROPILOT_INTERFACE.md) | Isaac Sim bridge reference — topics, parameters, and verification |
| [`docs/REAL_ROBOT_TOPICS_DESCRIPTION.md`](docs/REAL_ROBOT_TOPICS_DESCRIPTION.md) | Real robot topic contract reference and physical interface details |
| `micropilot_vehicle_interface/docs/simulation_interface.md` | Simulation transport ICD — the contract this workspace implements |