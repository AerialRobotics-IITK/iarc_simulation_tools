project(iarc_simulation_tools)
cmake_minimum_required(VERSION 2.8.3)


find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_library(wind_plugin SHARED src/wind_plugin.cc)
target_link_libraries(wind_plugin ${GAZEBO_LIBRARIES})

cs_install()
cs_export()
