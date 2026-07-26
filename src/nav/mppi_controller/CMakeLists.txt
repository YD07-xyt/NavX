cmake_minimum_required(VERSION 3.10)
project(mppi_controller)

# 使用 ament_cmake_auto，自动管理依赖
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# 内嵌 xtensor 第三方库路径（不属于 ament 生态，需手动添加）
set(XTENSOR_INCLUDE_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}/3rd/xtensor/include
    ${CMAKE_CURRENT_SOURCE_DIR}/3rd/xsimd/include
    ${CMAKE_CURRENT_SOURCE_DIR}/3rd/xtl/include
)

# 添加全局包含目录
include_directories(
    include
    ${XTENSOR_INCLUDE_DIRS}
)

# 添加可执行文件，ament_auto 会自动链接 package.xml 中声明的依赖库
ament_auto_add_executable(${PROJECT_NAME}_node
    src/mppi_ros2_node.cpp
)

# 安装 C++ 可执行文件
install(TARGETS ${PROJECT_NAME}_node
    DESTINATION lib/${PROJECT_NAME}
)

# 安装 Python 脚本
install(PROGRAMS
    scripts/mppi_path_publisher.py
    scripts/mppi_path_publisher_fz.py
    scripts/JZJ_path.py
    DESTINATION lib/${PROJECT_NAME}
)

# 安装配置文件
install(DIRECTORY config/
    DESTINATION share/${PROJECT_NAME}/config
)

# 安装其他脚本（如 RViz 配置）
install(DIRECTORY scripts/
    DESTINATION share/${PROJECT_NAME}/scripts
)

# 生成 ament 包（自动处理依赖导出等）
ament_auto_package()