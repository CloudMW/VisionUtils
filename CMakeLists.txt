cmake_minimum_required(VERSION 4.0)
project(PointCloudDockLib)
set(CMAKE_CXX_STANDARD 17)




##---------# find dependencies #---------##
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
find_package(spdlog CONFIG REQUIRED)

##----------# add subdirectories #----------##
add_subdirectory(visualization)
add_subdirectory(io)
add_subdirectory(demo)


##----------# Debug: Print all CMake variables #----------##
message(STATUS "---- All CMake Variables ----")
get_cmake_property(_vars VARIABLES)
foreach(_var ${_vars})
    message(STATUS "${_var} = ${${_var}}")
endforeach()
message(STATUS "-----------------------------")