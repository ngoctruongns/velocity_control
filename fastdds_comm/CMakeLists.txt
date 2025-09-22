
cmake_minimum_required(VERSION 2.8.12)

project("generated_code")

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_EXTENSIONS OFF)

# Find requirements
find_package(fastcdr REQUIRED)
find_package(fastrtps REQUIRED)


message(STATUS "Configuring VelocityMsg...")
add_library(VelocityMsg_lib VelocityMsg.cxx)
target_link_libraries(VelocityMsg_lib fastcdr fastrtps)

add_executable(VelocityMsg VelocityMsgPubSubTypes.cxx VelocityMsgPublisher.cxx VelocityMsgSubscriber.cxx VelocityMsgPubSubMain.cxx)
target_link_libraries(VelocityMsg fastcdr fastrtps
        VelocityMsg_lib )



