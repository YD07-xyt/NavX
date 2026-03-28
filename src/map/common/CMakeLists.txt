cmake_minimum_required(VERSION 3.10)

project(map)

file(GLOB_RECURSE MAP_COM_SOURCES 
    "${CMAKE_CURRENT_SOURCE_DIR}/esdf/src/*.cc"
    "${CMAKE_CURRENT_SOURCE_DIR}/occupancy/src/*.cc"
)


add_library(map_com ${MAP_COM_SOURCES})

target_include_directories(map_com PUBLIC         
        ${CMAKE_CURRENT_SOURCE_DIR}/occupancy/include
        ${CMAKE_CURRENT_SOURCE_DIR}/esdf/include)

add_executable(map map.cc)

target_link_libraries(map map_com)
