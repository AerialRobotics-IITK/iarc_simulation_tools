#pragma once

#include <boost/thread/mutex.hpp>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/msgs/msgs.hh>
#include <mav_msgs/Actuators.h>
#include <std_msgs/Float32.h>
#include <ros/ros.h>
#include <string>
#include <thread>



namespace gazebo {

class BatteryPlugin: public ModelPlugin {

public:
    void Load(physics::ModelPtr parent, sdf::ElementPtr sdf);
    void onUpdate(const common::UpdateInfo &_info);
    void setTimeCallback(const std_msgs::Float32 & msg);

private:
    physics::ModelPtr model_;
    physics::WorldPtr world_;
    sdf::ElementPtr sdf_;
    event::ConnectionPtr updateConnection_;
    ros::NodeHandle *rosnode_;
    gazebo::transport::NodePtr node_handle_;
    // gazebo::transport::PublisherPtr motor_speed_pub_;
    ros::Subscriber set_time_limit_;
    ros::Publisher battery_status_;
    ros::Publisher motor_speed_pub_;
    common::Time now;
    mav_msgs::Actuators motor_speeds_;
    std::string namespace_;
    std::string node_name_;
    std::string topic_name_;
    float time_limit_;
    int total_motors_;
    int count_ = 0;
};

} // namespace gazebo