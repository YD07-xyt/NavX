cmake_minimum_required(VERSION 3.8)
project(gcopter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE "Release")

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

add_definitions(-DORIGIN_AT_CORNER)
# 查找依赖
find_package(nlohmann_json REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem thread)
find_package(yaml-cpp REQUIRED)
find_package(ompl REQUIRED)
find_package(spdlog REQUIRED)
find_package(osqp REQUIRED)
find_package(OsqpEigen REQUIRED)
find_package(PCL REQUIRED)
find_package(behaviortree_cpp REQUIRED)
# ROS2 核心依赖
find_package(ament_cmake_auto REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
# CasADi 通常安装在 /usr/local
include_directories(/usr/local/include)
link_directories(/usr/local/lib)

# 包含头文件目录
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIRS}
)
file(GLOB_RECURSE map_sources CONFIGURE_DEPENDS "src/map/*.cpp")
add_library(3d_occ_map SHARED
${map_sources}
)

target_link_libraries(3d_occ_map
spdlog::spdlog
${PCL_LIBRARIES}
yaml-cpp::yaml-cpp
)

add_library(planner
src/ros2_2d.cpp
src/replan/fsm.cpp)

target_link_libraries(planner
  ${OMPL_LIBRARIES}
  spdlog::spdlog
  casadi
  osqp::osqp
  Boost::boost
  OsqpEigen::OsqpEigen
  yaml-cpp::yaml-cpp
)

ament_target_dependencies(planner
  rclcpp
  std_msgs
  std_srvs
  geometry_msgs
  sensor_msgs
  visualization_msgs
  nav_msgs
  tf2
  tf2_ros
  pcl_conversions
)

# 添加可执行文件
add_executable(global_planning src/global_planning.cpp)
set_target_properties(global_planning PROPERTIES
    INSTALL_RPATH "$ORIGIN"
    BUILD_WITH_INSTALL_RPATH TRUE
)
# 链接依赖库
target_link_libraries(global_planning
  ${OMPL_LIBRARIES}
  spdlog::spdlog
  casadi
  OsqpEigen::OsqpEigen
  Boost::boost
  libqpOASES.a
  yaml-cpp::yaml-cpp
  planner
  3d_occ_map
)

# 使用 ament_target_dependencies 添加 ROS2 依赖
target_link_libraries(global_planning
  ${EIGEN3_LIBRARIES}
)

ament_target_dependencies(global_planning
  rclcpp
  std_msgs
  std_srvs
  geometry_msgs
  sensor_msgs
  visualization_msgs
  nav_msgs
  pcl_conversions
)


# 安装规则
install(TARGETS 3d_occ_map planner
        DESTINATION lib/${PROJECT_NAME})
        
install(TARGETS global_planning
  DESTINATION lib/${PROJECT_NAME})

# 导出包
ament_auto_package(
  USE_SCOPED_HEADER_INSTALL_DIR
  INSTALL_TO_SHARE
  launch
  config
)