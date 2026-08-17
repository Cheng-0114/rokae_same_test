cmake_minimum_required(VERSION 3.8)
project(arm_teleop_usb)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBUSB REQUIRED libusb-1.0)

add_executable(usb_motor_bridge_node
  src/usb_motor_bridge_node.cpp
  src/usb_cdc.cpp
  src/protocol.c
)

target_include_directories(usb_motor_bridge_node PRIVATE
  include
  ${LIBUSB_INCLUDE_DIRS}
)

target_link_libraries(usb_motor_bridge_node
  ${LIBUSB_LIBRARIES}
  pthread
)

ament_target_dependencies(usb_motor_bridge_node
  rclcpp
  sensor_msgs
  std_srvs
)

install(TARGETS usb_motor_bridge_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
