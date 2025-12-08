

set(SUBSYS_NAME demo_XX)

add_executable(${SUBSYS_NAME})



target_sources(${SUBSYS_NAME} PRIVATE main.cpp)
target_include_directories(${SUBSYS_NAME} PRIVATE
        ${CMAKE_SOURCE_DIR}/)
target_link_libraries(${SUBSYS_NAME} PRIVATE
        ${PCL_LIBRARIES}
        spdlog::spdlog )

# 为target 设置 宏
target_compile_definitions(${SUBSYS_NAME} PRIVATE)
