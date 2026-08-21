cmake_minimum_required(VERSION 3.8)
project(rs_motor_ros2)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_default_runtime REQUIRED)
find_package(robot_msgs REQUIRED)


# Create a ament index resource to make the package findable
ament_export_dependencies(rosidl_default_runtime)

# Include directories
include_directories(
  include
)

# Declare a cpp library with a different name to avoid conflict
add_library(${PROJECT_NAME}_lib
  src/motor_cfg.cpp
  src/can_thread_manager.cpp
)

# Rename the output to avoid confusion
set_target_properties(${PROJECT_NAME}_lib PROPERTIES
  OUTPUT_NAME ${PROJECT_NAME}_lib
)

# Use the generated interfaces
ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  std_msgs
  sensor_msgs
  robot_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_lib)


# Motor control test executable (use unique target name to avoid conflicts with rosidl custom targets)
add_executable(rs_motor_ros2_node
  src/main.cpp
)
# add_executable(enable_motor
#   src/enable_motor.cpp
# )

ament_target_dependencies(rs_motor_ros2_node
  rclcpp
  std_msgs
  sensor_msgs
  robot_msgs
  rosidl_default_runtime
)
# ament_target_dependencies(enable_motor
#   rclcpp
#   std_msgs
#   can_msgs
# )

target_link_libraries(rs_motor_ros2_node
  ${PROJECT_NAME}_lib
)
# target_link_libraries(enable_motor
#   ${PROJECT_NAME}_lib
# )

# Install include directories
install(
  DIRECTORY include/
  DESTINATION include
)

# Install targets
install(
  TARGETS ${PROJECT_NAME}_lib rs_motor_ros2_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install config and launch files
install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# robot_msgs interfaces are available through ament_target_dependencies, no need for explicit rosidl_target_interfaces call

# Install launch files
# install(
#   DIRECTORY launch
#   DESTINATION share/${PROJECT_NAME}
# )

ament_package()