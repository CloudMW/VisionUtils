

set(SUBSYS_NAME demo_pcl_utils_vis)

add_executable(${SUBSYS_NAME})



target_sources(${SUBSYS_NAME} PRIVATE main.cpp)
target_include_directories(${SUBSYS_NAME} PRIVATE
        ${CMAKE_SOURCE_DIR}/)
target_link_libraries(${SUBSYS_NAME} PRIVATE
        ${PCL_LIBRARIES}
        spdlog::spdlog )

# 为target 设置 宏
target_compile_definitions(${SUBSYS_NAME} PRIVATE
        # PRIVATE：仅当前目标可见；PUBLIC：目标和依赖它的目标可见；INTERFACE：仅依赖它的目标可见
        DEMO_PATH="${CMAKE_CURRENT_SOURCE_DIR}/../"
)

set(SUBSYS_NAME_SUB demo_pcl_utils_vis_shape)
add_executable(${SUBSYS_NAME_SUB})
target_sources(${SUBSYS_NAME_SUB} PRIVATE shapes_persistence_demo.cpp)
target_include_directories(${SUBSYS_NAME_SUB} PRIVATE
        ${CMAKE_SOURCE_DIR}/)
target_link_libraries(${SUBSYS_NAME_SUB} PRIVATE
        ${PCL_LIBRARIES}
        spdlog::spdlog )

# 为target 设置 宏
target_compile_definitions(${SUBSYS_NAME_SUB} PRIVATE
        # PRIVATE：仅当前目标可见；PUBLIC：目标和依赖它的目标可见；INTERFACE：仅依赖它的目标可见
        DEMO_PATH="${CMAKE_CURRENT_SOURCE_DIR}/../"
)
