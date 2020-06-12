project(iarc_simulation_tools)
cmake_minimum_required(VERSION 2.8.3)

find_package(Eigen3 REQUIRED)
include_directories(${Eigen3_INCLUDE_DIRS})

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

# ============================================== #
# =================== PROTOBUF ================= #
# ============================================== #

set(PROTOBUF_IMPORT_DIRS "")
foreach(ITR ${GAZEBO_INCLUDE_DIRS})
  if(ITR MATCHES ".*gazebo-[0-9.]+$")
    set(PROTOBUF_IMPORT_DIRS "${ITR}/gazebo/msgs/proto")
  endif()
endforeach()
message(STATUS "PROTOBUF_IMPORT_DIRS = " ${PROTOBUF_IMPORT_DIRS})

# protobuf required for publishing/subscribing to Gazebo messages
# WARNING: THIS MUST BE RUN AFTER PROTOBUF_IMPORT_DIRS IS SETUP
find_package(Protobuf REQUIRED)
pkg_check_modules(PROTOBUF protobuf)

# Protobuf version check
if ("${PROTOBUF_VERSION}" VERSION_LESS "2.5.0")
  message(FATAL_ERROR "protobuf version: ${PROTOBUF_VERSION} not compatible, must be >= 2.5.0")
endif()

# After running this block of code,
# we should have something similar to
# GAZEBO_MSG_INCLUDE_DIRS = /usr/include/gazebo-7/gazebo/msgs
set(GAZEBO_MSG_INCLUDE_DIRS)
foreach(ITR ${GAZEBO_INCLUDE_DIRS})
  if(ITR MATCHES ".*gazebo-[0-9.]+$")
    set(GAZEBO_MSG_INCLUDE_DIRS "${ITR}/gazebo/msgs")
  endif()
endforeach()

include_directories(${GAZEBO_MSG_INCLUDE_DIRS})

# Get lists of all .proto files in the msgs directory
file(GLOB msgs msgs/*.proto)

# Finally, generate the .cpp files from the .proto files
PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ${msgs})

# Create a shared library of protobuf messages (.so extension on Linux platforms)
cs_add_library(mav_msgs SHARED ${PROTO_SRCS})
target_link_libraries(mav_msgs ${PROTOBUF_LIBRARY} gazebo_msgs)

# This causes mav_msgs to be linked with every created library in this file from this
# point forward.
# NOTE: This is deprecated, should be using target_link_libraries instead
link_libraries(mav_msgs)

include_directories(${CMAKE_CURRENT_BINARY_DIR})

# ============================================== #
# =================== PROTOBUF ================= #
# ============================================== #

cs_add_library(wind_plugin SHARED src/wind_plugin.cpp)
target_link_libraries(wind_plugin ${GAZEBO_LIBRARIES})

cs_install()
cs_export()
