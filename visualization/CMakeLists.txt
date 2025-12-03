set(SUBSYS_NAME visualization)
set(SUBSYS_DESC "Point cloud dock visualization library")


## ----- add visualization library ----- ##

set(LIB_NAME "pcdl_${SUBSYS_NAME}")
include_directories(include)
add_library(${LIB_NAME} SHARED
        src/pcdl_visualization.cpp
)

target_link_libraries(${LIB_NAME}
        ${PCL_LIBRARIES}
        spdlog::spdlog
        ${OpenCV_LIBS}
)

# 为target 设置 宏
# 设置导出
if (WIN32)
    target_compile_definitions(${LIB_NAME} PRIVATE PCDL_VISUALIZATION_EXPORTS)
    target_compile_definitions(${LIB_NAME} PRIVATE PCDL_VISUALIZATION_EXPORTS)


endif ()


## ----- install library ----- ##
# 设置安装目标路径
set(INSTALL_DIR "${CMAKE_SOURCE_DIR}/install")


# 复制依赖的 DLL 到 install/lib 目录
add_custom_command(TARGET ${LIB_NAME} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E make_directory ${INSTALL_DIR}/lib/3rd
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
        $<TARGET_RUNTIME_DLLS:${LIB_NAMgiE}>
        ${INSTALL_DIR}/lib/3rd
        COMMAND_EXPAND_LISTS
        COMMENT "Copying dependent DLLs to install directory"
)

# 安装库文件
install(TARGETS ${LIB_NAME}
        DESTINATION "${INSTALL_DIR}/lib/pcdl/"
        COMPONENT ${LIB_NAME}
)
# 安装头文件
install(DIRECTORY include/
        DESTINATION "${INSTALL_DIR}/include/pcdl/${SUBSYS_NAME}"
        COMPONENT ${LIB_NAME}
)


## ----- 自动安装 (编译后安装) ----- ##
# 设置 CMAKE_INSTALL_PREFIX，指示安装目录
set(CMAKE_INSTALL_PREFIX "${CMAKE_BINARY_DIR}/install")

# 自定义命令：在库构建成功后，自动执行安装操作
add_custom_command(TARGET ${LIB_NAME} POST_BUILD
        COMMAND ${CMAKE_COMMAND} --install ${CMAKE_BINARY_DIR} --component ${LIB_NAME} --config $<CONFIG> --prefix ${INSTALL_DIR}

        COMMENT "Automatically installing after build"
)