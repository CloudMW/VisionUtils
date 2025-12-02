

set(SUBSYS_NAME vis_demo)

add_executable(${SUBSYS_NAME})



target_sources(${SUBSYS_NAME} PRIVATE main.cpp)
target_include_directories(${SUBSYS_NAME} PRIVATE
        ${CMAKE_SOURCE_DIR}/install/include)
target_link_libraries(${SUBSYS_NAME} PRIVATE
        ${PCL_LIBRARIES}
        pcdl_visualization
        pcdl_io
        spdlog::spdlog )

# 为target 设置 宏
target_compile_definitions(${SUBSYS_NAME} PRIVATE
        # PRIVATE：仅当前目标可见；PUBLIC：目标和依赖它的目标可见；INTERFACE：仅依赖它的目标可见
        VIS_DEMO_PATH="${CMAKE_CURRENT_SOURCE_DIR}"
)


# 依赖项目统一注册为一个自定义目标
add_custom_target(pcdl_deps DEPENDS pcdl_visualization pcdl_io)

# 确保 vis_demo 在构建前先完成依赖
add_dependencies(${SUBSYS_NAME} pcdl_deps)