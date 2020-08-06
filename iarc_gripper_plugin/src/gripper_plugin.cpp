#include <iarc_gripper_plugin/GripperCmd.h>
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

    link1_ = model_->GetLink(parent->GetName() + "::" + link_name1_);
    link2_ = model_->GetLink(parent->GetName() + "::" + link_name2_);

    force_direction_.Set(0, -1, 0);
    xyz_offset_.Set(0, 0, 0);

    if (!ros::isInitialized()) {
        ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
                         << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package.");
        return;
    }

    rosnode_ = new ros::NodeHandle(namespace_);

    cmd_server_ = rosnode_->advertiseService("gripper_cmd", &GripperPlugin::serverCallback, this);
    // ros::ServiceClient gripper_cmd_client = rosnode_.serviceClient("gripper_cmd");

    updateConnection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&GripperPlugin::onUpdate, this, _1));
}

// bool GripperPlugin::serverCallback( std_srvs::Trigger::Request & req, std_srvs::Trigger::Response &resp){
//     flag_ = !flag_;
//     if (flag_==true){
//         force_direction_ = force_direction_*(-1);
//         std::cout << "Expanding gripper";
//     }
//     else{
//         std::cout << "Contracting gripper";
//     }
//     resp.success = true;
//     resp.message = "Triggered";
//     return true;
// }

bool GripperPlugin::serverCallback(iarc_gripper_plugin::GripperCmd::Request& req, iarc_gripper_plugin::GripperCmd::Response& res) {
    boost::mutex::scoped_lock scoped_lock(lock_);

    if (req.flag == 1) {
        force_direction_ = -force_direction_;
        ROS_INFO_STREAM("Expanding gripper " << force_direction_);
    } else {
        ROS_INFO_STREAM("Retracting gripper" << force_direction_);
    }
    res.message = "Triggered";
    return true;
}

void GripperPlugin::onUpdate(const common::UpdateInfo& _info) {
    // std_srvs::Trigger srv;
    // srv.request.flag = 1;
    // if (client.call(srv)){
    //     if (){
    //         force_direction_ = force_direction_*(-1);
    //         std::cout << "Expanding gripper");
    //     }
    //     else{
    //         std::cout <<"Contracting gripper");
    //     }
    // }

    link1_->AddForceAtRelativePosition(force_magnitude_ * force_direction_, xyz_offset_);
    link1_->AddForceAtRelativePosition(force_magnitude_ * force_direction_, xyz_offset_);
}

GZ_REGISTER_MODEL_PLUGIN(GripperPlugin);
}  // namespace gazebo
