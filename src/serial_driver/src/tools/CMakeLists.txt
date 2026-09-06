cmake_minimum_required(VERSION 3.16)

add_library(tools OBJECT 
    src/plotter.cpp
)
target_include_directories(tools PUBLIC include)
