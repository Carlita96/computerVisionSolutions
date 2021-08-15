cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(cropboxTool)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (cropboxTool cropboxTool.cpp)
target_link_libraries (cropboxTool ${PCL_LIBRARIES})
