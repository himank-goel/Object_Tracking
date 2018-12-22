cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(object_tracking)

find_package(PCL 1.5 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (tracking_bg tracking_bg.cpp)
target_link_libraries (tracking_bg ${PCL_LIBRARIES})