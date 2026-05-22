cmake_minimum_required(VERSION 3.10)
project(tof_demo)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs roscpp std_msgs
)

set(HM_LD1_SDK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../hm_ld1_sdk")
if(NOT EXISTS "${HM_LD1_SDK_DIR}/CMakeLists.txt")
  message(FATAL_ERROR "hm_ld1_sdk not found at ${HM_LD1_SDK_DIR}. Keep tof_demo and hm_ld1_sdk as sibling directories in the catkin src workspace.")
endif()

add_library(hm_ld1_sdk STATIC
  ${HM_LD1_SDK_DIR}/src/camera.cpp
  ${HM_LD1_SDK_DIR}/src/internal/frame_event.cpp
  ${HM_LD1_SDK_DIR}/src/protocol/frame_parser.cpp
  ${HM_LD1_SDK_DIR}/src/protocol/serial_protocol.cpp
  ${HM_LD1_SDK_DIR}/src/protocol/udp_protocol.cpp
  ${HM_LD1_SDK_DIR}/src/transport/packet_source.cpp
  ${HM_LD1_SDK_DIR}/src/transport/packet_source_serial.cpp
  ${HM_LD1_SDK_DIR}/src/transport/packet_source_udp.cpp
  ${HM_LD1_SDK_DIR}/src/transport/packet_source_uvc.cpp
  ${HM_LD1_SDK_DIR}/src/transport/serial_port.cpp
  ${HM_LD1_SDK_DIR}/src/transport/udp_network_configure.cpp
  ${HM_LD1_SDK_DIR}/src/transport/udp_network_format.cpp
  ${HM_LD1_SDK_DIR}/src/transport/udp_network_probe.cpp
  ${HM_LD1_SDK_DIR}/src/transport/udp_socket.cpp
  ${HM_LD1_SDK_DIR}/src/transport/uvc_device.cpp
  ${HM_LD1_SDK_DIR}/src/protocol/uvc_protocol.cpp
)

target_include_directories(hm_ld1_sdk
  PUBLIC
    ${HM_LD1_SDK_DIR}/include
  PRIVATE
    ${HM_LD1_SDK_DIR}/src
)

target_compile_features(hm_ld1_sdk PUBLIC cxx_std_17)

add_executable(tof_reverse_brake_node
  src/tof_reverse_brake_node.cpp
)

target_include_directories(tof_reverse_brake_node
  PRIVATE
    ${catkin_INCLUDE_DIRS}
)

target_link_libraries(tof_reverse_brake_node
  PRIVATE
    hm_ld1_sdk
    ${catkin_LIBRARIES}
)

add_dependencies(tof_reverse_brake_node
  ${catkin_EXPORTED_TARGETS}
)

install(TARGETS tof_reverse_brake_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
