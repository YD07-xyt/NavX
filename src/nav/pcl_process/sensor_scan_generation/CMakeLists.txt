cmake_minimum_required(VERSION 3.10)
project(sensor_scan_generation)

## Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## 更优雅的编译选项（替代add_definitions，避免全局污染）
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Werror)
endif()

## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#######################
## Find dependencies ##
#######################

# 核心修复：显式查找Boost并指定date_time组件
set(Boost_MINIMUM_VERSION 1.71.0) # ROS 2 Humble默认Boost版本
find_package(Boost ${Boost_MINIMUM_VERSION} REQUIRED COMPONENTS date_time)

# ROS 2自动依赖查找
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# 显式查找rclcpp_components（避免隐式依赖缺失）
find_package(rclcpp_components REQUIRED)

###########
## Build ##
###########

ament_auto_add_library(${PROJECT_NAME} SHARED
  DIRECTORY src
)

# 显式链接Boost::date_time目标
target_link_libraries(${PROJECT_NAME} Boost::date_time)

# 确保Boost头文件路径被包含（增强鲁棒性）
target_include_directories(${PROJECT_NAME}
  PUBLIC
  ${Boost_INCLUDE_DIRS}
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 注册组件节点（插件名称加引号，避免符号解析错误）
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "sensor_scan_generation::SensorScanGenerationNode"
  EXECUTABLE ${PROJECT_NAME}_node
)

#############
## Testing ##
#############

if(BUILD_TESTING)
  set(ament_cmake_clang_format_CONFIG_FILE "${CMAKE_SOURCE_DIR}/.clang-format")
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_uncrustify
    ament_cmake_flake8
  )
  ament_lint_auto_find_test_dependencies()
endif()

#############
## Install ##
#############

ament_auto_package(
  INSTALL_TO_SHARE
  launch
)