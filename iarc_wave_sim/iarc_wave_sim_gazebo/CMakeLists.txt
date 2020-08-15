project(iarc_wave_sim_gazebo)
cmake_minimum_required(VERSION 2.8.3)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

find_package(gazebo REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)

include_directories(${GAZEBO_INCLUDE_DIRS})
include_directories(${GAZEBO_MSG_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

cs_add_executable(${PROJECT_NAME}_sea_state_node src/ros_sea_state_subscriber_node.cpp)
cs_add_library(${PROJECT_NAME}_sea_state src/ros_sea_state_subscriber)

target_link_libraries(${PROJECT_NAME}_sea_state_node ${PROJECT_NAME}_sea_state ${GAZEBO_LIBRARIES})

cs_install()
cs_export()
