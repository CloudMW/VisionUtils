cmake_minimum_required(VERSION 3.10)

# 设置项目名称
project(error_code_demos)

# 设置C++标准
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 添加头文件路径
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../pcdl/error_code)

# 添加示例程序
add_executable(error_code_example example_usage.cpp)

# 添加测试程序
add_executable(error_code_test test_error_code.cpp)

# 如果需要，可以添加编译选项
if(MSVC)
    target_compile_options(error_code_example PRIVATE /W4)
    target_compile_options(error_code_test PRIVATE /W4)
else()
    target_compile_options(error_code_example PRIVATE -Wall -Wextra)
    target_compile_options(error_code_test PRIVATE -Wall -Wextra)
endif()

# 启用测试
enable_testing()
add_test(NAME ErrorCodeTests COMMAND error_code_test)

# 安装规则（可选）
#install(TARGETS error_code_example error_code_test DESTINATION bin)
#install(FILES example_usage.cpp test_error_code.cpp DESTINATION examples)
#install(FILES PORTING_GUIDE.md QUICK_REFERENCE.md DESTINATION docs)
#
