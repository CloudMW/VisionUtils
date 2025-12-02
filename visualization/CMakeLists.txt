set(SUBSYS_NAME visualization)
set(SUBSYS_DESC "Point cloud dock visualization library")


## ----- add visualization library ----- ##

set(LIB_NAME "pcdl_${SUBSYS_NAME}")
include_directories(include)
add_library(${LIB_NAME}  SHARED
    src/pcdl_visualization.cpp
)

target_link_libraries(${LIB_NAME} PRIVATE
    ${PCL_LIBRARIES}
    spdlog::spdlog
    ${OpenCV_LIBS}
)













## ----- install library ----- ##
# 设置安装目标路径
set(INSTALL_DIR "${CMAKE_SOURCE_DIR}/install")

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
        COMMAND ${CMAKE_COMMAND} --install ${CMAKE_BINARY_DIR} --component ${LIB_NAME} --config $<CONFIG>  --prefix ${INSTALL_DIR}

        COMMENT "Automatically installing after build"
)