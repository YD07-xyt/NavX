cmake_minimum_required(VERSION 3.10)
project(st_opt)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find Eigen3
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)  

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()


# Fallback for systems without proper Eigen3 CMake config
if(NOT TARGET Eigen3::Eigen)
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(EIGEN3 REQUIRED eigen3)
    include_directories(${EIGEN3_INCLUDE_DIRS})
endif()

# Include directories
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/perception_tool)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/frontend_tool)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/backend_tool)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

file(GLOB_RECURSE ros2_interface CONFIGURE_DEPENDS ./ros2/*.cc)

ament_auto_add_library(st_opt_ros2 SHARED
    ${ros2_interface}
)

target_include_directories(st_opt_ros2 PRIVATE
    ${PROJECT_SOURCE_DIR}/ros2
)
pluginlib_export_plugin_description_file(nav2_core st_opt_plugin.xml)
# Set build type if not specified
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

ament_auto_package(
  USE_SCOPED_HEADER_INSTALL_DIR
  INSTALL_TO_SHARE
)