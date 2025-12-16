cmake_minimum_required(VERSION 4.0)
project(VisionUtils)
set(CMAKE_CXX_STANDARD 14)




##---------# find dependencies #---------##
find_package(PCL REQUIRED)
find_package(OpenCV CONFIG OPTIONAL_COMPONENTS)
include_directories(${OpenCV_INCLUDE_DIRS})
find_package(spdlog CONFIG REQUIRED)

##--------- # include directories #---------##
include_directories(pcl_utils)
include_directories(common_utils)
include_directories(cv_utils)
##----------# add subdirectories #----------##

add_subdirectory(demo)


##----------# Debug: Print all CMake variables #----------##
message(STATUS "---- All CMake Variables ----")
get_cmake_property(_vars VARIABLES)
foreach(_var ${_vars})
    message(STATUS "${_var} = ${${_var}}")
endforeach()
message(STATUS "-----------------------------")