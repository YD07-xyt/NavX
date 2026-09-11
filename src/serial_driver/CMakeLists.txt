cmake_minimum_required(VERSION 3.10)

project(ma)

set(CMAKE_CXX_STANDARD 20)                # 尝试直接声明
set(CMAKE_CXX_STANDARD_REQUIRED ON)       # 强制要求
set(CMAKE_CXX_EXTENSIONS OFF)             # 禁用 GNU 扩展


set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake_auto REQUIRED)
find_package(ament_cmake REQUIRED)

set(ROS2_LIB_NAMES geometry_msgs rclcpp std_msgs nav2_msgs)
foreach(LIB_NAME ${ROS2_LIB_NAMES})
    find_package(${LIB_NAME} REQUIRED)
endforeach()


add_subdirectory(src/tools)
add_subdirectory(src/rm_decision)
add_subdirectory(src/serial)
add_subdirectory(src/ros2)

target_include_directories(serial PUBLIC src)

ament_auto_add_executable(ma_node
  main_node.cc
)
ament_target_dependencies(ma_node PUBLIC ${ROS2_LIB_NAMES})
target_link_libraries(ma_node PUBLIC serial rm_decision ros2)

##  TODO:
ament_auto_add_executable(test_node
  src/test/rm_decision_test.cpp
  src/test/test_node/test_node.cpp
)

target_link_libraries(test_node PUBLIC tools rm_decision)

ament_target_dependencies(test_node PUBLIC ${ROS2_LIB_NAMES})

ament_auto_package(
  USE_SCOPED_HEADER_INSTALL_DIR
  INSTALL_TO_SHARE
  config
  launch
)