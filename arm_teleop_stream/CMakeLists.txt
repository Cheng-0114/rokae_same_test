cmake_minimum_required(VERSION 3.8)
project(arm_teleop_stream)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(orocos_kdl REQUIRED)

add_definitions(-DXMATEMODEL_LIB_SUPPORTED)

# -----------------------------------------------------------------------------
# Rokae SDK 路径：复用 rokae_ros2/rokae_hardware 里的 SDK，不重复拷贝。
# 不能用 ${CMAKE_CURRENT_SOURCE_DIR}/../rokae_ros2/... 这种相对路径——本包是
# 从仓库外软链接进 $WS/src/ 的(参考 rokae_example 那种"同一份git clone里的
# 兄弟目录"用相对路径没问题，但本包和 rokae_ros2 不在同一处 git 仓库，符号
# 链接的 ".." 会按链接目标的真实位置解析，解出来的路径根本不是 $WS/src/)。
# 改用 colcon 安装前缀反推工作空间根目录 $WS，这个和源码是不是符号链接无关。
# -----------------------------------------------------------------------------
get_filename_component(_ws_install_dir "${CMAKE_INSTALL_PREFIX}" DIRECTORY)
get_filename_component(_ws_root "${_ws_install_dir}" DIRECTORY)
unset(_ws_install_dir)

set(SDK_INCLUDE_DIR ${_ws_root}/src/rokae_ros2/rokae_hardware/sdk/include)
set(SDK_LIB_DIR     ${_ws_root}/src/rokae_ros2/rokae_hardware/sdk/lib)
unset(_ws_root)

set(_XCORE_LIB_CHECK "${SDK_LIB_DIR}/libxCoreSDK.a")
if(NOT EXISTS "${_XCORE_LIB_CHECK}")
  message(FATAL_ERROR
    "找不到 rokae xCore SDK: ${_XCORE_LIB_CHECK}\n"
    "本包依赖 rokae_ros2/rokae_hardware 已经编译过、SDK 库文件已放到位"
    "(参考仓库 README 三、拉取 rokae_ros2 官方源码 一节)。")
endif()
unset(_XCORE_LIB_CHECK)

link_directories(${SDK_LIB_DIR})

add_library(xCoreSDK STATIC IMPORTED)
set_target_properties(xCoreSDK PROPERTIES
  IMPORTED_LOCATION "${SDK_LIB_DIR}/libxCoreSDK.a"
)

add_library(xMateModel STATIC IMPORTED)
set_target_properties(xMateModel PROPERTIES
  IMPORTED_LOCATION "${SDK_LIB_DIR}/libxMateModel.a"
  POSITION_INDEPENDENT_CODE ON
)

set(COMMON_SDK_LIBS
  xCoreSDK
  xMateModel
  orocos-kdl
  ${Eigen3_LIBRARIES}
)

add_executable(streaming_sdk_bridge_node src/streaming_sdk_bridge_node.cpp)

target_include_directories(streaming_sdk_bridge_node PRIVATE
  ${SDK_INCLUDE_DIR}
  ${Eigen3_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
)

ament_target_dependencies(streaming_sdk_bridge_node
  rclcpp
  std_msgs
  sensor_msgs
)

target_link_libraries(streaming_sdk_bridge_node ${COMMON_SDK_LIBS})

install(TARGETS streaming_sdk_bridge_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
