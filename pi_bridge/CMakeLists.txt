cmake_minimum_required(VERSION 3.8)
project(pi_bridge_standalone)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(fastcdr REQUIRED)
find_package(fastrtps REQUIRED)
find_package(Threads REQUIRED)

add_library(serial_comm_lib STATIC
  ../library/serial_comm/process_data_packet.c
)
target_include_directories(serial_comm_lib PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/../library/serial_comm
)

add_library(serial_driver_lib STATIC
  ../library/serial_driver/serial_linux.cpp
)
target_include_directories(serial_driver_lib PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/../library/serial_driver
)

add_library(fastdds_comm_lib STATIC
  ../fastdds_comm/raw_packet_fastdds.cpp
)
target_include_directories(fastdds_comm_lib PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/../fastdds_comm
)
target_link_libraries(fastdds_comm_lib PUBLIC
  fastcdr
  fastrtps
)

add_executable(pi_bridge
  velocity_control.cpp
)
target_include_directories(pi_bridge PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/../library/serial_comm
  ${CMAKE_CURRENT_SOURCE_DIR}/../library/serial_driver
  ${CMAKE_CURRENT_SOURCE_DIR}/../fastdds_comm
)
target_link_libraries(pi_bridge PRIVATE
  serial_comm_lib
  serial_driver_lib
  fastdds_comm_lib
  Threads::Threads
)
