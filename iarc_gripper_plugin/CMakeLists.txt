cmake_minimum_required(VERSION 3.0.2)
project(iarc_gripper_plugin)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

find_package(gazebo REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)

include_directories(${GAZEBO_INCLUDE_DIRS})
include_directories(${GAZEBO_MSG_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(${PROJECT_NAME} SHARED src/gripper_plugin.cpp)
target_link_libraries(${PROJECT_NAME} ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

cs_install()
cs_export()