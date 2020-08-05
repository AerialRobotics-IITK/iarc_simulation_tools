#include <iarc_gripper_plugin/gripper_plugin.hpp>
#include <ros/ros.h>
#include "iarc_gripper_plugin/GripperCmd.h>"

namespace gazebo{

void GripperPlugin::Load(physics::ModelPtr parent, sdf::ElementPtr sdf){
    model_ = parent;
    sdf_= sdf;

    gzmsg << "Gripper plugin loaded for " << model_->GetName() << std::endl;

    if (sdf->HasElement("link_name1")){
        link_name1_ = sdf->GetElement("link_name1")->Get<std::string>();
    }
    else{
        gzerr << "Element <link_name1> is not specified. Aborting.";
        return;
    }

    if (sdf->HasElement("link_name2")){
        link_name2_ = sdf->GetElement("link_name2")->Get<std::string>();
    }
    else{
        gzerr << "Element <link_name2> is not specified. Aborting.";
        return;
    }

    if(sdf->HasElement("force_magnitude")){
        force_magnitude_ = sdf->GetElement("force_magnitude")->Get<double>();
    }
    else{
        force_magnitude_ = 10.0;
    }

    link1_ = model_->GetLink(parent->GetName() + "::" + link_name1_);
    link2_ = model_->GetLink(parent->GetName() + "::" + link_name2_);
    
    force_direction_.Set(0,-1,0);
    xyz_offset_.Set(0,0,0);

    ros::NodeHandle rosnode_;
    ros::ServiceClient gripper_cmd_client = rosnode_->serviceClient<iarc_gripper_plugin::G

    updateConnection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&GripperPlugin::onUpdate, this, _1));
}

void GripperPlugin::onUpdate(const common::UpdateInfo& _info) {


    link1_->AddForceAtRelativePosition(force_magnitude_ * force_direction_, xyz_offset_);
    link1_->AddForceAtRelativePosition(force_magnitude_ * force_direction_, xyz_offset_);    
}

GZ_REGISTER_MODEL_PLUGIN(GripperPlugin);
}