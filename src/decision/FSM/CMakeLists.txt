cmake_minimum_required(VERSION 3.10)

project(FSM)
find_package(glog REQUIRED)

file(GLOB_RECURSE FSM_SRC
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cc"
)

include_directories(../../)

add_library(FSM ${FSM_SRC})

target_link_directories(FSM PUBLIC include)

target_link_libraries(FSM PRIVATE glog::glog)