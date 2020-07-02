project(iarc_simulation_tools)
cmake_minimum_required(VERSION 2.8.3)

find_package(Eigen3 REQUIRED)
include_directories(${Eigen3_INCLUDE_DIRS})
find_package(Boost REQUIRED COMPONENTS thread)
find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")
find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

set(PROTOBUF_IMPORT_DIRS "")
foreach(ITR ${GAZEBO_INCLUDE_DIRS})
  if(ITR MATCHES ".*gazebo-[0-9.]+$")
    set(PROTOBUF_IMPORT_DIRS "${ITR}/gazebo/msgs/proto")
  endif()
endforeach()
message(STATUS "PROTOBUF_IMPORT_DIRS = " ${PROTOBUF_IMPORT_DIRS})

find_package(Protobuf REQUIRED)
pkg_check_modules(PROTOBUF protobuf)

if ("${PROTOBUF_VERSION}" VERSION_LESS "2.5.0")
  message(FATAL_ERROR "protobuf version: ${PROTOBUF_VERSION} not compatible, must be >= 2.5.0")
endif()

set(GAZEBO_MSG_INCLUDE_DIRS)
foreach(ITR ${GAZEBO_INCLUDE_DIRS})
  if(ITR MATCHES ".*gazebo-[0-9.]+$")
    set(GAZEBO_MSG_INCLUDE_DIRS "${ITR}/gazebo/msgs")
  endif()
endforeach()

include_directories(${GAZEBO_MSG_INCLUDE_DIRS})

file(GLOB msgs msgs/*.proto)

PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ${msgs})

link_libraries(mav_msgs)

include_directories(${CMAKE_CURRENT_BINARY_DIR})
include_directories(include ${catkin_INCLUDE_DIRS} ${GAZEBO_MSG_INCLUDE_DIRS})

cs_add_library(mav_msgs SHARED ${PROTO_SRCS})
target_link_libraries(mav_msgs ${PROTOBUF_LIBRARY} gazebo_msgs)

cs_add_library(${PROJECT_NAME}_wind_plugin SHARED src/wind_plugin.cpp)
target_link_libraries(${PROJECT_NAME}_wind_plugin ${GAZEBO_LIBRARIES})

add_library(${PROJECT_NAME}_light_plugin SHARED src/light_plugin.cpp)
target_link_libraries(${PROJECT_NAME}_light_plugin ${GAZEBO_LIBRARIES})

add_library(${PROJECT_NAME}_link_plugin SHARED src/link_state_publisher.cpp)
target_link_libraries(${PROJECT_NAME}_link_plugin ${GAZEBO_LIBRARIES})

add_library(${PROJECT_NAME}_joint_plugin SHARED src/revolute_joint_rotator_plugin.cpp)
target_link_libraries(${PROJECT_NAME}_joint_plugin ${GAZEBO_LIBRARIES})

add_library(${PROJECT_NAME}_magnet_plugin SHARED src/dipole_magnet.cpp)
target_link_libraries(${PROJECT_NAME}_magnet_plugin ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

cs_add_executable(check src/check.cpp)
target_link_libraries(check ${catkin_LIBRARIES})

cs_add_executable(ros_bridge src/ros_bridge.cpp)

catkin_install_python(PROGRAMS
   src/plot.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


cs_install()
cs_export()
