cmake_minimum_required(VERSION 3.16)
project(serial VERSION 0.10)
#include(cmake/common.cmake)
set(LIB_NAMES  spdlog)
set(LIB_3RD "")

foreach(LIB_NAME ${LIB_NAMES})
    find_package(${LIB_NAME} REQUIRED)
    list(APPEND LIB_3RD "${LIB_NAME}::${LIB_NAME}")
endforeach()

file(GLOB_RECURSE SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")

add_library(${PROJECT_NAME}  SHARED ${SOURCES})

# 公开头文件路径（外部可见）
target_include_directories(${PROJECT_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

# 私有头文件路径（仅编译本库时可见）
target_include_directories(${PROJECT_NAME}
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(${PROJECT_NAME} PRIVATE ${LIB_3RD})

find_package(Boost REQUIRED COMPONENTS system)
target_link_libraries(${PROJECT_NAME} PRIVATE Boost::system)

target_link_libraries(serial PUBLIC tools)