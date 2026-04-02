cmake_minimum_required(VERSION 3.10)

project(serial)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(Boost REQUIRED COMPONENTS system)

add_library(serial src/serial.cc
src/crc.cpp)

target_include_directories(serial PUBLIC src/serial.h src/crc.hpp)

target_link_libraries(serial PRIVATE  ${Boost_LIBRARIES})


ament_auto_add_executable(serial_node
  serial_node.cc
)

target_link_libraries(serial_node  serial)

ament_auto_package(
  USE_SCOPED_HEADER_INSTALL_DIR
  INSTALL_TO_SHARE
)