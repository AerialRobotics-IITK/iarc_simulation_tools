#pragma once

#include <ros/ros.h>
#include<string.h>
#include <cstdlib>
#include <iarc_simulation_tools/seastate.hpp>
#include <std_msgs/Int32.h>
#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/msgs/msgs.hh>

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

    private:
    std_msgs::Int32 current_sea_state_ ;
    class SeaState states_;
    ros::Subscriber sea_state_sub_;
    gazebo::transport::NodePtr node_;
    gazebo::transport::PublisherPtr wave_pub_;
    gazebo::msgs::Param_V wave_msg_;

};

} //namespace iarc_simulation_tools


