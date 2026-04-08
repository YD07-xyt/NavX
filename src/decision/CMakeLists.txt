cmake_minimum_required(VERSION 3.10)

project(decision)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# 改用 ament_auto_add_library
ament_auto_add_library(decision src/fsm.cc)

# 不需要手动添加 include 目录，ament_auto 会处理
target_include_directories(decision PRIVATE include)

ament_auto_add_executable(decision_node src/decision_node.cc)

# 不需要手动 target_link_libraries，ament_auto 会处理
# target_link_libraries(decision_node decision)

ament_auto_package(
  USE_SCOPED_HEADER_INSTALL_DIR
  INSTALL_TO_SHARE
)