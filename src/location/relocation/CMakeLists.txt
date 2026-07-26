cmake_minimum_required(VERSION 3.8)
project(relocation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# add_compile_options(-fsanitize=address -g)
# add_link_options(-fsanitize=address)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(Eigen3 REQUIRED)
find_package(spdlog REQUIRED)
find_package(PCL REQUIRED)
find_package(small_gicp REQUIRED)
find_package(TBB REQUIRED)
# 在 find_package 部分添加
# find_package(PkgConfig REQUIRED)
# pkg_check_modules(LZ4 REQUIRED liblz4)
# add_subdirectory(./3rd/FastGlobalRegistration)

set(includes
        include
        ${EIGEN3_INCLUDE_DIRS}
        #            ${OpenCV_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
)


set(third_party_libs
        #            ${OpenCV_LIBRARIES}
        ${PCL_LIBRARIES}
        spdlog::spdlog
        small_gicp::small_gicp
        TBB::tbb
        #${LZ4_LIBRARIES}
        #FastGlobalRegistrationLib
)
include_directories(
        ${includes}
)

file(GLOB SRC_FILES "src/relocation.cpp")
add_library(relocation ${SRC_FILES})

ament_auto_add_executable(relocation_node src/relocation_node.cpp)

target_link_libraries(relocation_node relocation ${third_party_libs})

target_include_directories(relocation_node
  PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)


ament_auto_package(
  USE_SCOPED_HEADER_INSTALL_DIR
  INSTALL_TO_SHARE
  config
  launch
)