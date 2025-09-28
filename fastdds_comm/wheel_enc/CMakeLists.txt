
cmake_minimum_required(VERSION 2.8.12)

project("generated_code")

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_EXTENSIONS OFF)

# Find requirements
find_package(fastcdr REQUIRED)
find_package(fastrtps REQUIRED)


message(STATUS "Configuring WheelEncoderMsg...")
add_library(WheelEncoderMsg_lib WheelEncoderMsg.cxx)
target_link_libraries(WheelEncoderMsg_lib fastcdr fastrtps)

add_executable(WheelEncoderMsg WheelEncoderMsgPubSubTypes.cxx WheelEncoderMsgPublisher.cxx WheelEncoderMsgSubscriber.cxx WheelEncoderMsgPubSubMain.cxx)
target_link_libraries(WheelEncoderMsg fastcdr fastrtps
        WheelEncoderMsg_lib )



