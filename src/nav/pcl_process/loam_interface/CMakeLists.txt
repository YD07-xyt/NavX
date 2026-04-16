cmake_minimum_required(VERSION 3.10)
project(loam_interface)

## Use C++14 (ROS 2 Humble推荐C++17，可按需调整)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## 更优雅的编译选项（替代add_definitions）
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Werror)
endif()

## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#######################
## Find dependencies ##
#######################

# 1. 显式指定Boost版本（可选，避免版本兼容问题）
set(Boost_MINIMUM_VERSION 1.71.0) # Humble默认Boost版本
find_package(Boost ${Boost_MINIMUM_VERSION} REQUIRED COMPONENTS date_time)

# 2. ROS 2依赖查找
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# 3. 确保rclcpp_components可用（显式查找，避免隐式依赖缺失）
find_package(rclcpp_components REQUIRED)

###########
## Build ##
###########

# 构建库（使用PROJECT_NAME变量更通用）
ament_auto_add_library(${PROJECT_NAME} SHARED
  DIRECTORY src
)

# 1. 链接Boost（使用目标变量，避免硬编码）
target_link_libraries(${PROJECT_NAME} 
  Boost::date_time
)

# 2. 确保包含Boost头文件（显式声明，提升鲁棒性）
target_include_directories(${PROJECT_NAME}
  PUBLIC
  ${Boost_INCLUDE_DIRS}
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 注册组件节点（添加命名空间保护，避免符号冲突）
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "loam_interface::LoamInterfaceNode"  # 加引号，避免特殊字符问题
  EXECUTABLE ${PROJECT_NAME}_node
)

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