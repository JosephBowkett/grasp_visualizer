cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(grasp_visualizer)

find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (grasp_visualizer src/grasp_visualizer.cpp)
target_link_libraries (grasp_visualizer ${PCL_LIBRARIES})
