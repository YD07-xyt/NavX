cmake_minimum_required(VERSION 3.10)

project(serial)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(Eigen3 REQUIRED)
find_package(spdlog REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(nlohmann_json REQUIRED)
find_package(behaviortree_cpp REQUIRED)


add_subdirectory(src/tools)
add_subdirectory(src/rm_decision)


ament_auto_add_library(serial src/serial/serial.cc
src/serial/crc.cpp
src/decision/decision.cc
src/decision/patrol.cpp
src/ros2/serial_node.cc)

target_include_directories(serial PUBLIC src)

target_link_libraries(serial  ${Boost_LIBRARIES} Eigen3::Eigen)


ament_auto_add_executable(serial_node
  main_node.cc
)

target_link_libraries(serial_node  serial tools rm_decision)

ament_auto_add_executable(test_node
src/test/rm_decision_test.cpp)

target_link_libraries(test_node tools rm_decision)

ament_auto_package(
  USE_SCOPED_HEADER_INSTALL_DIR
  INSTALL_TO_SHARE
  config
  launch
)