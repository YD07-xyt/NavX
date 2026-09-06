# common_setup.cmake
function(setup_project PROJ_NAME LIB_NAMES_LIST)
    # 查找所有依赖库，并构建目标列表
    set(LIB_3RD "")
    foreach(LIB_NAME ${LIB_NAMES_LIST})
        find_package(${LIB_NAME} REQUIRED)
        # 这里假设每个库导出的目标名都是 ${LIB_NAME}::${LIB_NAME}
        # 如果你的库目标名不同，可在此处调整
        list(APPEND LIB_3RD "${LIB_NAME}::${LIB_NAME}")
    endforeach()

    # 递归收集 src 下的所有 .cpp 文件
    file(GLOB_RECURSE SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")

    # 将结果传递给父作用域（调用者）
    set(SOURCES ${SOURCES} PARENT_SCOPE)
    set(LIB_3RD ${LIB_3RD} PARENT_SCOPE)
    # 同时把项目名也传出去（可选，因为 PROJECT_NAME 在调用者作用域不可见）
    set(PROJECT_NAME ${PROJ_NAME} PARENT_SCOPE)
endfunction()