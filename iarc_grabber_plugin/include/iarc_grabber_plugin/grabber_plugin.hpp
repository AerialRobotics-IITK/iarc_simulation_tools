#pragma once

#include <boost/thread/mutex.hpp>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <iarc_grabber_plugin/GrabberCmd.h>
#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <string>
#include <thread>

namespace gazebo {

class GrabberPlugin : public ModelPlugin {
  public:
    void Load(physics::ModelPtr parent, sdf::ElementPtr sdf);
    void onUpdate(const common::UpdateInfo& _info);
    bool serverCallback(iarc_grabber_plugin::GrabberCmd::Request &req,
                      iarc_grabber_plugin::GrabberCmd::Response &res);

  private:
    physics::ModelPtr model_;
    sdf::ElementPtr sdf_;
    event::ConnectionPtr updateConnection_;
    physics::LinkPtr link1_;
    physics::LinkPtr link2_;
    
    std::string link_name1_;
    std::string link_name2_;
    std::string namespace_;
    ignition::math::Vector3d force_direction_;
    ignition::math::Vector3d dummytwo_;
    ros::NodeHandle* rosnode_;

    double force_magnitude_;
  ros::ServiceServer cmd_server_;
  boost::mutex lock_;
};
}  // namespace gazebo