#include <iarc_service_plugin/service_plugin.hpp>

namespace gazebo {

void ServicePlugin::Load(physics::ModelPtr parent, sdf::ElementPtr sdf) {

    if (sdf->HasElement("desired_relative_pose")) {
        desired_relative_pose_ = sdf->GetElement("desired_relative_pose")->Get<ignition::math::Pose3d>();
    } else {
        gzerr << "<desired_relative_pose> tag is missing. Aborting.";
        return;
    }

    mast_position_ = Eigen::Vector3d(0,0,0);
    mast_orientation_ = Eigen::Quaterniond(1,0,0,0);

    desired_relative_position_ = Eigen::Vector3d(desired_relative_pose_.Pos().X(), desired_relative_pose_.Pos().Y(), desired_relative_pose_.Pos().Z());
    desired_relative_orientation_ = Eigen::Quaterniond(
        desired_relative_pose_.Rot().W(), desired_relative_pose_.Rot().X(), desired_relative_pose_.Rot().Y(), desired_relative_pose_.Rot().Z());

    comm_block_position_ = desired_relative_position_;
    comm_block_orientation_ = desired_relative_orientation_;

    // gzmsg << "Light plugin loaded for " << led_color_ << " light" << std::endl;

    model_ = parent;
    flag_ = false;
    node_ = transport::NodePtr(new transport::Node);
    node_->Init();
    comm_pose_sub_ = node_->Subscribe("~/communications_module/link/pose/info", &ServicePlugin::commPoseCallback, this);
    mast_pose_sub_ = node_->Subscribe("~/table/mast::link/pose/info", &ServicePlugin::mastPoseCallback, this);
    updateConnection = event::Events::ConnectPreRender(std::bind(&ServicePlugin::OnUpdate, this));
}

void ServicePlugin::OnUpdate() {
    comm_wrt_mast_position_ = mast_orientation_.inverse().toRotationMatrix() * (comm_block_position_ - mast_position_);
    comm_wrt_mast_orientation_ = mast_orientation_.inverse() * comm_block_orientation_;

    bool orientation_match = fabs(comm_wrt_mast_orientation_.angularDistance(desired_relative_orientation_)) < 0.02;
    bool position_match = (comm_wrt_mast_position_ - desired_relative_position_).norm() < 0.01;

    // Debug message. Uncomment for mast and comm block relative pose and match
    // gzmsg << "correct_position: " << position_match << " Distance: " << (comm_wrt_mast_position_ - desired_relative_position_).norm()
    //           << std::endl;
    // gzmsg << "correct_orientation: " << orientation_match
    //           << " Angular Distance:" << comm_wrt_mast_orientation_.angularDistance(desired_relative_orientation_) << std::endl;

    if (orientation_match && position_match){
        flag_ = true;
        // std::cout << flag_ << "TESTINGGG";
    }
    
        // model_visual_->SetMaterial("Gazebo/" + led_color_ + "Transparent");
}

void ServicePlugin::commPoseCallback(ConstPoseStampedPtr& msg) {
    comm_block_position_ = Eigen::Vector3d(msg->pose().position().x(), msg->pose().position().y(), msg->pose().position().z());
    comm_block_orientation_ =
        Eigen::Quaterniond(msg->pose().orientation().w(), msg->pose().orientation().x(), msg->pose().orientation().y(), msg->pose().orientation().z());
}

void ServicePlugin::mastPoseCallback(ConstPoseStampedPtr& msg) {
    mast_position_ = Eigen::Vector3d(msg->pose().position().x(), msg->pose().position().y(), msg->pose().position().z());
    mast_orientation_ =
        Eigen::Quaterniond(msg->pose().orientation().w(), msg->pose().orientation().x(), msg->pose().orientation().y(), msg->pose().orientation().z());
}

GZ_REGISTER_MODEL_PLUGIN(ServicePlugin)

}  // namespace gazebo