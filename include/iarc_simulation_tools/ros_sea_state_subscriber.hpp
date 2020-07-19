#pragma once

#include <cstdlib>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <iarc_simulation_tools/seastate.hpp>
#include <ros/ros.h>
#include <rotors_comm/WindSpeed.h>
#include <std_msgs/Int32.h>
#include <string.h>

namespace iarc_simulation_tools {

class RosBridge {
  public:
    RosBridge() {
    }
    ~RosBridge() {
    }
    void getSeaState(const std_msgs::Int32& msg);
    void getSeaDirection(const geometry_msgs::Point& msg);
    void init(ros::NodeHandle& nh);
    void publishWaveState();
    void getWindDirection(const rotors_comm::WindSpeed& msg);

  private:
    class SeaState states_;
    std_msgs::Int32 current_sea_state_;
    geometry_msgs::Point current_sea_direction_;
    geometry_msgs::Vector3 direction_;
    ros::Subscriber sea_state_sub_;
    ros::Subscriber sea_direction_sub_;
    ros::Subscriber wind_sub_;
    gazebo::transport::NodePtr node_;
    gazebo::transport::PublisherPtr wave_pub_;
    gazebo::msgs::Param_V wave_msg_;
};

}  // namespace iarc_simulation_tools
