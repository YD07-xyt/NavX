cmake_minimum_required(VERSION 3.8)
project(cloud_preprocess)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(spdlog REQUIRED)
find_package(PCL REQUIRED)
find_package(TBB REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)

set(includes
        include
        ${EIGEN3_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
)

set(third_party_libs
        ${PCL_LIBRARIES}
        spdlog::spdlog
        TBB::tbb
)

include_directories(${includes})

# ---- 核心算法库（与 ROS2 解耦） ----
file(GLOB SRC_FILES "src/cloud_preprocess.cpp")
add_library(cloud_preprocess ${SRC_FILES})
target_include_directories(cloud_preprocess PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

# ---- ROS2 节点 ----
add_executable(cloud_preprocess_node src/cloud_preprocess_node.cpp)
target_link_libraries(cloud_preprocess_node cloud_preprocess ${third_party_libs})
ament_target_dependencies(cloud_preprocess_node rclcpp sensor_msgs pcl_conversions)

# 安装可执行文件与核心库（否则 lib/<pkg> 缺失，launch 报 libexec 错误）
install(TARGETS cloud_preprocess cloud_preprocess_node
        DESTINATION lib/${PROJECT_NAME})

ament_auto_package(
  USE_SCOPED_HEADER_INSTALL_DIR
  INSTALL_TO_SHARE
  config
  launch
)
