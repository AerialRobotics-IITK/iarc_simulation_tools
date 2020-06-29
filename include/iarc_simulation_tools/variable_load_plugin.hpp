#pragma once

#include <vector>
#include <ros/ros.h>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <functional>
#include <random>
#include <string>
#include <iarc_simulation_tools/var_load.h>
#include <glog/logging.h>
#include <thread>
// #include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float32.h"

#include <gazebo/common/Plugin.hh>

namespace gazebo {

static const std::string kDefaultFrameId = "world";
static const std::string kDefaultLinkName = "base_link";
static const std::string kDefaultNamespace = "namespace";

class GazeboVarForcePlugin : public ModelPlugin {

public:
  GazeboVarForcePlugin()
      : ModelPlugin(), namespace_(kDefaultNamespace),
      frame_id_(kDefaultFrameId), 
      link_name_(kDefaultLinkName) {}

  virtual ~GazeboVarForcePlugin();
protected:
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  void OnUpdate(const common::UpdateInfo & /*_info*/);
  // void cb(const boost::shared_ptr<const iarc_simulation_tools::var_load>& msg);
  void OnRosMsg(const std_msgs::Float32MultiArrayConstPtr &_msg);
  void QueueThread();


private:
  // iarc_simulation_tools::var_load& var_load_;
  event::ConnectionPtr update_connection_;
  physics::WorldPtr world_;
  physics::ModelPtr model_;
  physics::LinkPtr link_;
  std::string namespace_;
  std::string frame_id_;
  std::string link_name_;
  ignition::math::Vector3d xyz_offset_;
  ignition::math::Vector3d force_direction_;
  // gazebo::transport::SubscriberPtr params_sub_;
  // gazebo::transport::NodePtr node_handle_;
  std::unique_ptr<ros::NodeHandle> rosNode;
  ros::Subscriber rosSub;
  ros::CallbackQueue rosQueue;
  std::thread rosQueueThread;
  

};
}
