cmake_minimum_required(VERSION 3.8)
project(carla_micropilot_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(carla_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Bridge node executable
add_executable(carla_micropilot_interface_node
  src/main.cpp
  src/carla_micropilot_interface_node.cpp
)

target_include_directories(carla_micropilot_interface_node
  PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

ament_target_dependencies(carla_micropilot_interface_node
  rclcpp
  std_msgs
  carla_msgs
  sensor_msgs
  geometry_msgs
)

# Install
install(TARGETS carla_micropilot_interface_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
