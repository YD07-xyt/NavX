cmake_minimum_required(VERSION 3.10)

project(serial)

find_package(Boost REQUIRED COMPONENTS system)



add_library(serial serial.cc
crc.cpp)

target_include_directories(serial PUBLIC serial.h crc.hpp)

target_link_libraries(serial PRIVATE  ${Boost_LIBRARIES})

add_executable(serial_main main.cc)

target_link_libraries(serial_main PRIVATE serial)