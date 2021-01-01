#include <iarc_gripper_plugin/gripper_plugin.hpp>
#include <ros/ros.h>
#include <std_srvs/Trigger.h>

namespace gazebo {

void GripperPlugin::Load(physics::ModelPtr parent, sdf::ElementPtr sdf) {
  model_ = parent;
  sdf_ = sdf;

  gzmsg << "Gripper plugin loaded for " << model_->GetName() << std::endl;

  if (sdf->HasElement("namespace")) {
    namespace_ = sdf->GetElement("namespace")->Get<std::string>();
    gzmsg << namespace_ << std::endl;
  } else {
    gzerr << "Element <namespace>> is not specified. Aborting.";
    return;
  }

  if (sdf->HasElement("link_name")) {
    link_name_ = sdf->GetElement("link_name")->Get<std::string>();
  } else {
    gzerr << "Element <link_name> is not specified. Aborting.";
    return;
  }

  if (sdf->HasElement("torque_magnitude")) {
    torque_magnitude_ = sdf->GetElement("torque_magnitude")->Get<double>();
  } else {
    torque_magnitude_ = 1.0;
  }

  link_ = model_->GetLink(parent->GetName() + "::" + link_name_);

  torque_direction_.Set(0, 0, 0);
  xyz_offset_.Set(0, 0, 0);

  if (!ros::isInitialized()) {
    ROS_FATAL_STREAM(
        "A ROS node for Gazebo has not been initialized, unable to load "
        "plugin. "
        << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the "
           "gazebo_ros package.");
    return;
  }

  rosnode_ = new ros::NodeHandle(namespace_);
  cmd_server_ = rosnode_->advertiseService(
      "gripper_cmd", &GripperPlugin::serverCallback, this);

  updateConnection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&GripperPlugin::onUpdate, this, _1));
}

bool GripperPlugin::serverCallback(
    iarc_simulation_msgs::GripperCmd::Request &req,
    iarc_simulation_msgs::GripperCmd::Response &res) {
  boost::mutex::scoped_lock scoped_lock(lock_);

  if (req.flag == 1) {
    torque_direction_.Set(0, 0, -1);
    res.message = "Opening gripper";
  } else if (req.flag == 2) {
    torque_direction_.Set(0, 0, 0);
    res.message = "Pausing gripper";
  } else if (req.flag == 0) {
    torque_direction_.Set(0, 0, 1);
    res.message = "Closing gripper";
  } else {
    res.message = "Choose from 0 to retract, 1 to expand or 2 to pause";
  }
  return true;
}

void GripperPlugin::onUpdate(const common::UpdateInfo &_info) {

  link_->AddRelativeTorque(torque_direction_ * torque_magnitude_);
}

GZ_REGISTER_MODEL_PLUGIN(GripperPlugin);
} // namespace gazebo
