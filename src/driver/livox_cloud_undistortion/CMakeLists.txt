cmake_minimum_required(VERSION 3.5)
project(livox_dedistortion_pkg)

# Set C++ standard (ROS 2 recommends at least C++14)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Ament CMake
find_package(ament_cmake REQUIRED)

# ROS 2 dependencies
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
# 如果增加了 tf2_eigen 依赖，则添加：
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(nav_msgs REQUIRED)
# System libraries
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(spdlog REQUIRED)

# Include directories (non‑ROS packages handled manually)
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

# ---------- deskew_node ----------
add_executable(deskew_node
  src/livox_dedistortion.cpp
  src/data_process.cpp
  src/gyr_int.cpp
)

# Link ROS 2 dependencies (automatically propagates includes and libraries)
ament_target_dependencies(deskew_node
  rclcpp
  std_msgs
  sensor_msgs
  livox_ros_driver2
  pcl_conversions
  pcl_ros
  nav_msgs
  # 如果有 tf2_eigen 依赖，也加上
  tf2_eigen
  tf2_geometry_msgs
)

# PCL libraries need explicit linking
target_link_libraries(deskew_node ${PCL_LIBRARIES} spdlog::spdlog)

# ---------- livox_repub_node ----------
add_executable(livox_repub_node
  src/livox_repub.cpp
)

ament_target_dependencies(livox_repub_node
  rclcpp
  std_msgs
  sensor_msgs
  livox_ros_driver2
  pcl_conversions
  pcl_ros
  nav_msgs
  tf2_eigen
  tf2_geometry_msgs
)

target_link_libraries(livox_repub_node ${PCL_LIBRARIES} spdlog::spdlog)

# Install executables
install(TARGETS deskew_node livox_repub_node
  DESTINATION lib/${PROJECT_NAME}
)

# Standard ament installation of include directory (if headers exist)
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

ament_package()