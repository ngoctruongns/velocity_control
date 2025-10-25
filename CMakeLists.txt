cmake_minimum_required(VERSION 3.8)
project(arduino_velocity_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Fast DDS dependencies
find_package(fastcdr REQUIRED)
find_package(fastrtps REQUIRED)

include_directories(
  include
  fastdds_comm/wheel_enc
  fastdds_comm/cmd_vel
  linux_comm_driver
  serial_comm
)

#  Create library for WheelEncoderMsgSubscriber
file(GLOB DDS_WHEELENC_SRC "fastdds_comm/wheel_enc/*.cxx")
add_library(WheelEncoderMsg_lib ${DDS_WHEELENC_SRC})
target_link_libraries(WheelEncoderMsg_lib fastcdr fastrtps)

#  Create library for VelocityCmdPublisher
file(GLOB DDS_VELCMD_SRC "fastdds_comm/cmd_vel/*.cxx")
add_library(VelocityCmdMsg_lib ${DDS_VELCMD_SRC})
target_link_libraries(VelocityCmdMsg_lib fastcdr fastrtps)

# Create library for Serial Linux driver
file(GLOB LINUX_COMM_SRC "linux_comm_driver/*.cpp")
add_library(SerialLinux_lib ${LINUX_COMM_SRC})
target_link_libraries(SerialLinux_lib pthread)

# Create library for Serial communication protocol
file(GLOB SERIAL_COMM_SRC "serial_comm/*.c")
add_library(SerialComm_lib ${SERIAL_COMM_SRC})
target_link_libraries(SerialComm_lib pthread)

# Generate the ROS2 node
add_executable(arduino_velocity_control src/velocity_control.cpp)

# Link the Fast DDS libraries
target_link_libraries(arduino_velocity_control
  fastrtps
  fastcdr
  WheelEncoderMsg_lib
  VelocityCmdMsg_lib
  SerialLinux_lib
  SerialComm_lib
)
