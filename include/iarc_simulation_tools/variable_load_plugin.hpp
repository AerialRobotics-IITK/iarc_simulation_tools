#pragma once

#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32MultiArray.h"
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>
#include <iarc_simulation_tools/common.h>
#include <iarc_simulation_tools/var_load.h>
#include <ignition/math/Vector3.hh>
#include <ros/ros.h>
#include <string>
#include <thread>
#include <vector>

#include <gazebo/common/Plugin.hh>

namespace gazebo {

static const std::string kDefaultFrameId = "world";
static const std::string kDefaultLinkName = "base_link";

class GazeboVarForcePlugin : public ModelPlugin {

public:
  GazeboVarForcePlugin()
      : ModelPlugin(), namespace_(kDefaultNamespace),
        link_name_(kDefaultLinkName) {}

  virtual ~GazeboVarForcePlugin();

protected:
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  void OnUpdate(const common::UpdateInfo & /*_info*/);
  void OnRosMsg(const std_msgs::Float32MultiArrayConstPtr &_msg);
  void QueueThread();

private:
  event::ConnectionPtr update_connection_;
  physics::WorldPtr world_;
  physics::ModelPtr model_;
  physics::LinkPtr link_;
  std::string namespace_;
  std::string link_name_;
  ignition::math::Vector3d xyz_offset_;
  ignition::math::Vector3d force_direction_;
  std::unique_ptr<ros::NodeHandle> rosNode;
  ros::Subscriber rosSub;
  ros::CallbackQueue rosQueue;
  std::thread rosQueueThread;

  double mass;
};
} // namespace gazebo
