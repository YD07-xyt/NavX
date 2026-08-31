cmake_minimum_required(VERSION 3.16)

ament_auto_add_library(rm_decision OBJECT 
    src/rm_decision.cpp
)

target_link_directories(rm_decision PUBLIC ../include)

target_link_libraries(rm_decision behaviortree_cpp::behaviortree_cpp spdlog::spdlog)