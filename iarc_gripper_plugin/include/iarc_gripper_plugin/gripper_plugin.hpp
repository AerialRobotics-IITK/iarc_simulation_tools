#pragma once

#include <boost/thread/mutex.hpp>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <iarc_gripper_plugin/GripperCmd.h>
#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <string>
#include <thread>

namespace gazebo {

class GripperPlugin : public ModelPlugin {
public:
  void Load(physics::ModelPtr parent, sdf::ElementPtr sdf);
  void onUpdate(const common::UpdateInfo & /*_info*/);
  bool serverCallback(iarc_gripper_plugin::GripperCmd::Request &req,
                      iarc_gripper_plugin::GripperCmd::Response &res);

private:
  physics::ModelPtr model_;
  sdf::ElementPtr sdf_;
  event::ConnectionPtr updateConnection_;
  physics::LinkPtr link_;

  std::string link_name_;
  std::string namespace_;
  ignition::math::Vector3d torque_direction_;
  ignition::math::Vector3d xyz_offset_;
  ros::NodeHandle *rosnode_;

  double torque_magnitude_;
  bool flag_ = true;

  ros::ServiceServer cmd_server_;
  boost::mutex lock_;
};

} // namespace gazebo
