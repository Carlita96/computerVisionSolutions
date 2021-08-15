cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(NARFTool)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (NARFTool NARFTool.cpp)
target_link_libraries (NARFTool ${PCL_LIBRARIES})
