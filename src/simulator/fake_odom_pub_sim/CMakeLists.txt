cmake_minimum_required(VERSION 3.10)
project(fake_odom_pub_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(fake_odom_pub_sim src/fake_odom_pub_sim.cpp)

ament_target_dependencies(fake_odom_pub_sim
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  visualization_msgs
  nav_msgs
  tf2
  tf2_ros
)


install(TARGETS fake_odom_pub_sim
  DESTINATION lib/${PROJECT_NAME})


ament_auto_package(
  USE_SCOPED_HEADER_INSTALL_DIR
  INSTALL_TO_SHARE
  launch
)
