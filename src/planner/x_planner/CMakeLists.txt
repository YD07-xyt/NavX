cmake_minimum_required(VERSION 3.10)

project(x_planner)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
#  libqpOASES.a
link_directories("/usr/local/lib")
include_directories("/usr/local/include")

file(GLOB_RECURSE x_planner_src CONFIGURE_DEPENDS src/path_search/*.cc 
src/trajectory_optimization/*.cc
src/control/*.cc
src/plan_env/*.cc)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS} 
  ${PCL_INCLUDE_DIRS}
)

add_definitions(-DHAVE_CSTDDEF)
add_library(x_planner ${x_planner_src})

target_link_directories(x_planner PUBLIC include)

target_link_libraries(x_planner PRIVATE Eigen3::Eigen ${OpenCV_LIBS} ${PCL_LIBRARIES} libqpOASES.a ipopt)

# add_executable(test src/test/test.cpp)
# target_link_libraries(test x_planner)
# target_link_directories(test PRIVATE /include/visualization.hpp)

# add_executable(main main.cpp)
# target_link_libraries(main x_planner)

add_executable(test1 src/test/planner_manager.cpp)
target_link_libraries(test1 x_planner)

ament_auto_add_library(ros2_node src/ros2_node.cc)

# 不需要手动添加 include 目录，ament_auto 会处理
target_include_directories(ros2_node PRIVATE include)

ament_auto_package(
  USE_SCOPED_HEADER_INSTALL_DIR
  INSTALL_TO_SHARE
)