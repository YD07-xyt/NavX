cmake_minimum_required(VERSION 3.16)
project(ros2 VERSION 0.10)

find_package(ament_cmake REQUIRED)
set(ROS2_LIB_NAMES geometry_msgs rclcpp std_msgs)
foreach(LIB_NAME ${ROS2_LIB_NAMES})
    find_package(${LIB_NAME} REQUIRED)
endforeach()

find_package(yaml-cpp REQUIRED)

file(GLOB_RECURSE SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")

add_library(ros2
${SOURCES})

ament_target_dependencies(ros2 PUBLIC ${ROS2_LIB_NAMES})

target_include_directories(${PROJECT_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
target_include_directories(${PROJECT_NAME}
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)
target_link_libraries(ros2 PUBLIC serial rm_decision yaml-cpp)
