project(iarc_simulation_tools)
cmake_minimum_required(VERSION 2.8.3)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

find_package(gazebo REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)

include_directories(${GAZEBO_INCLUDE_DIRS})
include_directories(${GAZEBO_MSG_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(${PROJECT_NAME}_light_plugin SHARED src/light_plugin.cpp)
target_link_libraries(${PROJECT_NAME}_light_plugin ${GAZEBO_LIBRARIES})

add_library(${PROJECT_NAME}_link_plugin SHARED src/link_state_publisher.cpp)
target_link_libraries(${PROJECT_NAME}_link_plugin ${GAZEBO_LIBRARIES})

add_library(${PROJECT_NAME}_joint_plugin SHARED src/revolute_joint_rotator_plugin.cpp)
target_link_libraries(${PROJECT_NAME}_joint_plugin ${GAZEBO_LIBRARIES})

add_library(${PROJECT_NAME}_variable_load_plugin SHARED src/variable_load_plugin.cpp)
target_link_libraries(${PROJECT_NAME}_variable_load_plugin ${GAZEBO_LIBRARIES} ${roscpp_LIBRARIES})

add_library(${PROJECT_NAME}_magnet_plugin SHARED src/dipole_magnet.cpp)
target_link_libraries(${PROJECT_NAME}_magnet_plugin ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

cs_add_executable(check src/check.cpp)
target_link_libraries(check ${catkin_LIBRARIES})

cs_add_executable(${PROJECT_NAME}_sea_state_node src/ros_sea_state_subscriber_node.cpp)
cs_add_library(${PROJECT_NAME}_sea_state src/ros_sea_state_subscriber)
target_link_libraries(${PROJECT_NAME}_sea_state_node ${PROJECT_NAME}_sea_state ${GAZEBO_LIBRARIES})
add_dependencies(${PROJECT_NAME}_sea_state_node ${catkin_EXPORTED_TARGETS})

catkin_install_python(PROGRAMS
   src/plot.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

cs_install()
cs_export()
