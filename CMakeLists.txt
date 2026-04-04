cmake_minimum_required(VERSION 3.8)
project(velocity_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Fast DDS dependencies
find_package(fastcdr REQUIRED)
find_package(fastrtps REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(serial_comm_lib STATIC
  library/serial_comm/process_data_packet.c
)
target_include_directories(serial_comm_lib PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/library/serial_comm
)

add_library(serial_driver_lib STATIC
  library/serial_driver/serial_linux.cpp
)
target_include_directories(serial_driver_lib PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/library/serial_driver
)

add_library(fastdds_comm_lib STATIC
  fastdds_comm/raw_packet_fastdds.cpp
)
target_include_directories(fastdds_comm_lib PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/fastdds_comm
)
target_link_libraries(fastdds_comm_lib PUBLIC
  fastcdr
  fastrtps
)

add_executable(pi_bridge
  pi_bridge/velocity_control.cpp
)
target_include_directories(pi_bridge PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/library/serial_comm
  ${CMAKE_CURRENT_SOURCE_DIR}/library/serial_driver
  ${CMAKE_CURRENT_SOURCE_DIR}/fastdds_comm
)
target_link_libraries(pi_bridge
  serial_comm_lib
  serial_driver_lib
  fastdds_comm_lib
)

add_executable(ros2_bridge
  ros2_bridge/ros2_bridge_node.cpp
)
target_include_directories(ros2_bridge PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/library/serial_comm
  ${CMAKE_CURRENT_SOURCE_DIR}/fastdds_comm
)
ament_target_dependencies(ros2_bridge
  rclcpp
  std_msgs
  geometry_msgs
)
target_link_libraries(ros2_bridge
  serial_comm_lib
  fastdds_comm_lib
)

add_executable(ps3_utils_control
  joy_stick/ps3_utils_control.cpp
)
ament_target_dependencies(ps3_utils_control
  rclcpp
  sensor_msgs
  std_msgs
)

install(TARGETS
  pi_bridge
  ros2_bridge
  ps3_utils_control
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY joy_stick/
  DESTINATION share/${PROJECT_NAME}/joy_stick
)

install(FILES
  ros2_bridge/ros2_bridge_params.yaml
  ros2_bridge/ros2_bridge.launch.py
  DESTINATION share/${PROJECT_NAME}/ros2_bridge
)

ament_package()
