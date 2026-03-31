cmake_minimum_required(VERSION 3.10)

project(planner)

file(GLOB_RECURSE path_search_src CONFIGURE_DEPENDS path_search/src/*.cc )

add_library(path_search ${path_search_src})

target_link_directories(path_search PUBLIC path_search/include)

target_link_libraries(path_search PUBLIC rog_map)

add_executable(main main.cpp)

target_link_libraries(main path_search)
