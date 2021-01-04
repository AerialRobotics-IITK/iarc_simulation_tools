// #include <iarc_gripper_plugin/GripperCmd.h>
#include <iarc_grabber_plugin/GrabberCmd.h>
#include <iarc_grabber_plugin/grabber_plugin.hpp>
#include <ros/ros.h>
// #include <std_srvs/Trigger.h>

namespace gazebo {

void GrabberPlugin::Load(physics::ModelPtr parent, sdf::ElementPtr sdf) {
    model_ = parent;
    sdf_ = sdf;

    gzmsg << "Grabber plugin loaded for " << model_->GetName() << std::endl;

    if (sdf->HasElement("namespace")) {
        namespace_ = sdf->GetElement("namespace")->Get<std::string>();
        gzmsg << namespace_ << std::endl;
    } else {
        gzerr << "Element <namespace>> is not specified. Aborting.";
        return;
    }

    if (sdf->HasElement("link_name1")) {
        link_name1_ = sdf->GetElement("link_name1")->Get<std::string>();
    } else {
        gzerr << "Element <link_name1> is not specified. Aborting.";
        return;
    }

    if (sdf->HasElement("link_name2")) {
        link_name2_ = sdf->GetElement("link_name2")->Get<std::string>();
    } else {
        gzerr << "Element <link_name2> is not specified. Aborting.";
        return;
    }

    if (sdf->HasElement("force_magnitude")) {
        force_magnitude_ = sdf->GetElement("force_magnitude")->Get<double>();
    } else {
        force_magnitude_ = 1.0;
    }

    if (!ros::isInitialized()) {
        ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load "
                         "plugin. "
                         << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the "
                            "gazebo_ros package.");
        return;
    }

    rosnode_ = new ros::NodeHandle(namespace_);
    cmd_server_ = rosnode_->advertiseService("grabber_cmd", &GrabberPlugin::serverCallback, this);

    link1_ = model_->GetLink(parent->GetName() + "::" + link_name1_);
    link2_ = model_->GetLink(parent->GetName() + "::" + link_name2_);

    updateConnection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&GrabberPlugin::onUpdate, this, _1));
}

bool GrabberPlugin::serverCallback(iarc_grabber_plugin::GrabberCmd::Request& req, iarc_grabber_plugin::GrabberCmd::Response& res) {
    boost::mutex::scoped_lock scoped_lock(lock_);

    if (req.flag == 1) {
        force_direction_.Set(-1, 0, 0);
        res.message = "Opening grabber";
    } else if (req.flag == 2) {
        force_direction_.Set(0, 0, 0);
        res.message = "Pausing grabber";
    } else if (req.flag == 0) {
        force_direction_.Set(1, 0, 0);
        res.message = "Closing grabber";
    } else {
        res.message = "Choose from 0 to close, 1 to expand or 2 to pause";
    }
    return true;
}

void GrabberPlugin::onUpdate(const common::UpdateInfo& _info) {
    // force_direction_.Set(1, 0, 0);
    // this->model_->GetJoint("left_hand")->SetVelocity(0,0.001);
    link1_->AddRelativeForce(force_magnitude_ * force_direction_);
    link2_->AddRelativeForce(force_magnitude_ * force_direction_ * (-1));
}

GZ_REGISTER_MODEL_PLUGIN(GrabberPlugin);

}  // namespace gazebo