cmake_minimum_required(VERSION 3.8)
project(velocity_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# Fast DDS dependencies
find_package(fastcdr REQUIRED)
find_package(fastrtps REQUIRED)

include_directories(
  include
  fastdds_comm/wheel_enc
  fastdds_comm/cmd_vel
)

#  Create library for WheelEncoderMsgSubscriber
file(GLOB DDS_WHEELENC_SRC "fastdds_comm/wheel_enc/*.cxx")
add_library(WheelEncoderMsg_lib ${DDS_WHEELENC_SRC})
target_link_libraries(WheelEncoderMsg_lib fastcdr fastrtps)

#  Create library for VelocityCmdPublisher
file(GLOB DDS_VELCMD_SRC "fastdds_comm/cmd_vel/*.cxx")
add_library(VelocityCmdMsg_lib ${DDS_VELCMD_SRC})
target_link_libraries(VelocityCmdMsg_lib fastcdr fastrtps)

# Generate the ROS2 node
add_executable(encoder_odom_node src/encoder_odometry.cpp)
ament_target_dependencies(encoder_odom_node
rclcpp
std_msgs
nav_msgs
geometry_msgs
tf2_ros
)

# Link the Fast DDS libraries
target_link_libraries(encoder_odom_node fastrtps fastcdr WheelEncoderMsg_lib)

# Install the executable
install(TARGETS
  encoder_odom_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
