cmake_minimum_required(VERSION 3.14)
project(carla_telemetry_cpp)

# ── C++17 ─────────────────────────────────────────────────────────────
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── ROS 2 dependencies ───────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(carla_msgs REQUIRED)
find_package(micropilot_manager_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)

# ── yaml-cpp ─────────────────────────────────────────────────────────
find_package(yaml-cpp REQUIRED)

# ── libcarla ─────────────────────────────────────────────────────────
# Use embedded libraries from thirdparty_lib
set(CARLA_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../../thirdparty_lib/carla")

# CARLA include paths (LibCarla headers + third-party deps)
set(CARLA_INCLUDE_DIRS
  ${CARLA_ROOT}/LibCarla/source
  ${CARLA_ROOT}/Build/libcarla-build/LibCarla/cmake/client
  # Third-party headers bundled with CARLA (rpc/msgpack, recast, moodycamel, pugixml, etc.)
  ${CARLA_ROOT}/PythonAPI/carla/dependencies/include
  ${CARLA_ROOT}/PythonAPI/carla/dependencies/include/system
)

# CARLA library path
find_library(LIBCARLA_CLIENT
  NAMES carla_client libcarla_client
  PATHS
    ${CARLA_ROOT}/Build/libcarla-client-build.release/LibCarla/cmake/client
    ${CARLA_ROOT}/PythonAPI/carla/dependencies/lib
    ${CARLA_ROOT}/lib
  NO_DEFAULT_PATH
)

# rpc library (libcarla_client.a depends on it)
find_library(LIBRPC
  NAMES rpc librpc
  PATHS
    ${CARLA_ROOT}/PythonAPI/carla/dependencies/lib
    ${CARLA_ROOT}/Build/rpclib-carla-callbacks-c10-libstdcxx-install/lib
  NO_DEFAULT_PATH
)

# Recast/Detour (libcarla_client.a depends on them)
find_library(LIBRECAST NAMES Recast PATHS ${CARLA_ROOT}/PythonAPI/carla/dependencies/lib NO_DEFAULT_PATH)
find_library(LIBDETOUR  NAMES Detour  PATHS ${CARLA_ROOT}/PythonAPI/carla/dependencies/lib NO_DEFAULT_PATH)
find_library(LIBDETOUR_CROWD NAMES DetourCrowd PATHS ${CARLA_ROOT}/PythonAPI/carla/dependencies/lib NO_DEFAULT_PATH)

# Boost (use CARLA's bundled Boost 1.90 — static to avoid runtime .so issues)
set(CARLA_BOOST_ROOT ${CARLA_ROOT}/Build/boost-1.90.0-c10-install)
if(NOT EXISTS "${CARLA_BOOST_ROOT}/lib/libboost_filesystem.a")
  set(CARLA_BOOST_ROOT ${CARLA_ROOT}/PythonAPI/carla/dependencies)
endif()
# Force static linking — find_library prefers .so, so use direct paths
set(CARLA_BOOST_FILESYSTEM ${CARLA_BOOST_ROOT}/lib/libboost_filesystem.a)
set(CARLA_BOOST_ATOMIC     ${CARLA_BOOST_ROOT}/lib/libboost_atomic.a)

# ── Executable ────────────────────────────────────────────────────────
set(SOURCES
  src/main.cpp
  src/node.cpp
  src/vehicle.cpp
  src/sensor_manager.cpp
  src/ros2_backend.cpp
  src/perf_monitor.cpp
  src/sensors/battery.cpp
  src/sensors/gps.cpp
  src/sensors/imu.cpp
  src/sensors/odometry.cpp
  src/sensors/ground_truth_boxes.cpp
  src/sensors/camera.cpp
  src/sensors/lidar.cpp
  src/sensors/depth_lidar.cpp
  src/walkers.cpp
  src/npc_vehicles.cpp
  src/dynamic_props.cpp
)

add_executable(carla_telemetry_node ${SOURCES})

target_include_directories(carla_telemetry_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CARLA_INCLUDE_DIRS}
  ${CARLA_BOOST_ROOT}/include
)

target_link_libraries(carla_telemetry_node
  ${LIBCARLA_CLIENT}
  ${LIBRPC}
  ${LIBRECAST}
  ${LIBDETOUR}
  ${LIBDETOUR_CROWD}
  yaml-cpp
  ${CARLA_BOOST_FILESYSTEM}
  ${CARLA_BOOST_ATOMIC}
  pthread
)

ament_target_dependencies(carla_telemetry_node
  rclcpp
  rclcpp_lifecycle
  std_msgs
  std_srvs
  sensor_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  ackermann_msgs
  tf2_ros
  builtin_interfaces
  carla_msgs
  micropilot_manager_msgs
  rosgraph_msgs
)

# ── DDS profiler node (consumer-side rate/latency; no libcarla) ────────
add_executable(dds_profiler_node
  src/dds_profiler_node.cpp
  src/perf_monitor.cpp
)
target_include_directories(dds_profiler_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(dds_profiler_node
  yaml-cpp
  pthread
)
ament_target_dependencies(dds_profiler_node
  rclcpp
  sensor_msgs
  nav_msgs
)

# ── Install ───────────────────────────────────────────────────────────
install(TARGETS carla_telemetry_node dds_profiler_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)
ament_package()
