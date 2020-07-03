#pragma once

#include <ros/ros.h>
#include <string.h>
#include <cstdlib>
#include <iarc_simulation_tools/seastate.hpp>
#include <std_msgs/Int32.h>
#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/msgs/msgs.hh>
#include <geometry_msgs/Vector3.h>
#include <rotors_comm/WindSpeed.h>

namespace iarc_simulation_tools {

class RosBridge {

 public:
    RosBridge() {
    }
    ~RosBridge() {
    }
    void getSeaState(const std_msgs::Int32& msg);
    void init(ros::NodeHandle& nh);
    void publishWaveState();
    void getWindDirection(const rotors_comm::WindSpeed& msg);

    private:
    class SeaState states_;
    std_msgs::Int32 current_sea_state_ ;
    geometry_msgs::Vector3 direction_;
    ros::Subscriber sea_state_sub_;
    ros::Subscriber wind_sub_;
    gazebo::transport::NodePtr node_;
    gazebo::transport::PublisherPtr wave_pub_;
    gazebo::msgs::Param_V wave_msg_;

};

} //namespace iarc_simulation_tools


