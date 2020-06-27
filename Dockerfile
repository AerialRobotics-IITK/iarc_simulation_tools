FROM ros:melodic-ros-core-bionic

RUN cd /home && \
    mkdir -p ros_ws/src/iarc_simulation_tools && \
    apt-get update && apt-get install -y \
    python-catkin-tools \
    build-essential \
    git \
    ros-melodic-gazebo-ros \
    ros-melodic-eigen-conversions \
    python-wstool