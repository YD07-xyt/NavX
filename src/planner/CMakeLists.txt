cmake_minimum_required(VERSION 3.10)

project(planner)

# 设置 C++ 20
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(rog_map REQUIRED) 
find_package(Eigen3 REQUIRED)

file(GLOB_RECURSE path_search_src CONFIGURE_DEPENDS path_search/src/*.cc )

add_library(path_search ${path_search_src})

target_link_directories(path_search PUBLIC path_search/include
${EIGEN3_INCLUDE_DIRS} )

target_link_libraries(path_search PUBLIC rog_map)

add_executable(main main.cpp)

target_link_libraries(main path_search)
