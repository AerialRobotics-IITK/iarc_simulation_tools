cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(gazebo REQUIRED)
include_directories(include ${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(example SHARED src/example_plugin.cpp)
add_library(example_orientation SHARED src/orientation_controller_example.cpp)
target_link_libraries(example ${GAZEBO_LIBRARIES})
target_link_libraries(example_orientation ${GAZEBO_LIBRARIES})