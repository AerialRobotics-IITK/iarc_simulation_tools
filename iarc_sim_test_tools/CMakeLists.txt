project(iarc_sim_test_tools)
cmake_minimum_required(VERSION 2.8.3)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

find_package(gazebo REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)

include_directories(${GAZEBO_INCLUDE_DIRS})
include_directories(${GAZEBO_MSG_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(${PROJECT_NAME}_variable_load_plugin SHARED src/variable_load_plugin.cpp)
target_link_libraries(${PROJECT_NAME}_variable_load_plugin ${GAZEBO_LIBRARIES} ${roscpp_LIBRARIES})

cs_add_executable(check src/check.cpp)
target_link_libraries(check ${catkin_LIBRARIES})

cs_install()

cs_install_scripts(scripts/plot.py)

cs_export()
