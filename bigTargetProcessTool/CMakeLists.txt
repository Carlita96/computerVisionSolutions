cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(bigTargetProcessTool)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (bigTargetProcessTool bigTargetProcessTool.cpp)
target_link_libraries (bigTargetProcessTool ${PCL_LIBRARIES})