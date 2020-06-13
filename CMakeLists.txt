project(iarc_simulation_tools)
cmake_minimum_required(VERSION 2.8.3)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

find_package(gazebo REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(${PROJECT_NAME}_light_plugin SHARED src/light_plugin.cpp)
target_link_libraries(${PROJECT_NAME}_light_plugin ${GAZEBO_LIBRARIES})

add_library(${PROJECT_NAME}_link_plugin SHARED src/link_state_publisher.cpp)
target_link_libraries(${PROJECT_NAME}_link_plugin ${GAZEBO_LIBRARIES})

add_library(${PROJECT_NAME}_dipole_moment SHARED src/dipole_magnet.cc)
target_link_libraries(${PROJECT_NAME}_dipole_moment ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

cs_install()
cs_export()
