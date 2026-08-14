# Updating CARLA Dependencies in asurt

This document explains the steps to follow whenever you modify the core CARLA source code (e.g., in `/home/ebrahim/carla`) and need those changes to take effect in the `asurt` ROS 2 nodes.

## Understanding the Dependency
The C++ nodes in the `asurt` workspace (such as `carla_telemetry_node`) do not compile the CARLA source code from scratch. Instead, they link statically against the pre-compiled CARLA client library (`libcarla_client.a`) located in the `thirdparty_lib` directory.

If you make *any* changes to the `LibCarla` C++ source code, you must rebuild this static library and update it in `asurt`. Rebuilding the Python API (`make PythonAPI`) alone is **not** enough for the C++ ROS nodes.

## How to Apply CARLA Source Changes to `asurt`

Follow these 3 steps whenever you modify the CARLA source code:

### Step 1: Rebuild `libcarla_client.a` in the main CARLA repository
Navigate to the main CARLA repository where you modified the source files, and run the Makefile target to build the client release library:
```bash
cd /home/ebrahim/carla
make LibCarla.client.release
```
*Note: This will compile the C++ source and produce a new static library in the `PythonAPI/carla/dependencies/lib/` folder.*

### Step 2: Copy the compiled library to the `asurt` workspace
Copy the newly built library to replace the existing dependency in your ROS 2 workspace's `thirdparty_lib` folder:
```bash
cp /home/ebrahim/carla/PythonAPI/carla/dependencies/lib/libcarla_client.a \
   /home/ebrahim/asurt/thirdparty_lib/carla/PythonAPI/carla/dependencies/lib/libcarla_client.a
```
*(Optional: If you also modified headers, be sure to copy the updated headers to `thirdparty_lib/carla/LibCarla/source/` as well).*

### Step 3: Rebuild the dependent ROS 2 packages
Navigate to your `asurt` workspace and rebuild any packages that depend on the CARLA client library (e.g., `carla_telemetry_cpp`) so they link against the new `libcarla_client.a`:
```bash
cd /home/ebrahim/Carla_ASU_Bridge/
colcon build --packages-select carla_telemetry_cpp
```
