cmake_minimum_required(VERSION 3.10)
project(terrain_analysis)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#######################
## 核心修复：添加Boost查找 ##
#######################
# 指定Boost最小版本（适配ROS 2 Humble的1.71+）
set(Boost_MINIMUM_VERSION 1.71.0)
find_package(Boost ${Boost_MINIMUM_VERSION} REQUIRED COMPONENTS date_time)

# 原有依赖查找
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(message_filters REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)

# 构建可执行文件
add_executable(terrainAnalysis src/terrainAnalysis.cpp)

# 链接Boost::date_time目标（核心修复）
target_link_libraries(terrainAnalysis Boost::date_time)

# 原有ROS 2依赖链接
ament_target_dependencies(
  terrainAnalysis
  rclcpp
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  message_filters
  pcl_ros
  pcl_conversions)

# 确保Boost头文件被包含（增强鲁棒性）
target_include_directories(terrainAnalysis
  PUBLIC
  ${Boost_INCLUDE_DIRS}
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS terrainAnalysis DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()