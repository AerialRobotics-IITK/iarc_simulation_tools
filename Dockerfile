FROM ros:melodic-ros-core-bionic

RUN cd /home && mkdir -p ros_ws/src/iarc_simulation_tools
RUN apt-get update && apt-get install -y python-catkin-tools
RUN apt-get update && apt-get install -y build-essential git
RUN apt-get update && apt-get install -y ros-melodic-gazebo-ros
RUN apt-get update && apt-get install -y ros-melodic-eigen-conversions
RUN apt-get update && apt-get install -y python-wstool