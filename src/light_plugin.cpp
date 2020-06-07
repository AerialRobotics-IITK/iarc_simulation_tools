#include <iarc_simulation_tools/light_plugin.hpp>

namespace gazebo {

void ModelLight::Load(rendering::VisualPtr visual, sdf::ElementPtr sdf) {    
    if (sdf->HasElement("led_color")) {
        led_color_ = sdf->GetElement("led_color")->Get<std::string>();
    } else {
        gzerr << "<led_color> tag missing. Aborting.";
        return;
    }

    std::cout << "Light plugin loaded for " << led_color_ << " light" << std::endl;

    model_visual_ = visual;

    node_ = transport::NodePtr(new transport::Node);
    node_->Init();
    comm_pose_sub_ = node_->Subscribe("~/communications_module/link/pose/info", &ModelLight::commPoseCallback, this);
    mast_pose_sub_ = node_->Subscribe("~/table/mast/pose/info", &ModelLight::mastPoseCallback, this);

    updateConnection = event::Events::ConnectPreRender(std::bind(&ModelLight::OnUpdate, this));
}

void ModelLight::OnUpdate() {
    comm_wrt_mast_position_ = mast_orientation_.inverse().toRotationMatrix() * (comm_block_position_ - mast_position_);
    comm_wrt_mast_orientation_ = mast_orientation_.inverse() * comm_block_orientation_;

    bool orientation_match = fabs(comm_wrt_mast_orientation_.angularDistance(Eigen::Quaterniond(0, 0, 0, 1))) < 0.02;
    bool position_match = (comm_wrt_mast_position_ - Eigen::Vector3d(-0.069, 0.21, 1.032)).norm() < 0.001;

    // Debug message. Uncomment for mast and comm block relative pose and match
    // std::cout << "correct_position: " << position_match << " Distance: " << (comm_wrt_mast_position_ - Eigen::Vector3d(-0.069, 0.21, 1.032)).norm()
    //           << std::endl;
    // std::cout << "correct_orientation: " << orientation_match
    //           << " Angular Distance:" << comm_wrt_mast_orientation_.angularDistance(Eigen::Quaterniond(0, 0, 0, 1)) << std::endl;

    if (orientation_match && position_match)
        model_visual_->SetMaterial("Gazebo/" + led_color_ + "Glow");
    else
        model_visual_->SetMaterial("Gazebo/" + led_color_ + "Transparent");
}

void ModelLight::commPoseCallback(ConstPoseStampedPtr& msg) {
    comm_block_position_ = Eigen::Vector3d(msg->pose().position().x(), msg->pose().position().y(), msg->pose().position().z());
    comm_block_orientation_ =
        Eigen::Quaterniond(msg->pose().orientation().w(), msg->pose().orientation().x(), msg->pose().orientation().y(), msg->pose().orientation().z());
}

void ModelLight::mastPoseCallback(ConstPoseStampedPtr& msg) {
    mast_position_ = Eigen::Vector3d(msg->pose().position().x(), msg->pose().position().y(), msg->pose().position().z());
    mast_orientation_ =
        Eigen::Quaterniond(msg->pose().orientation().w(), msg->pose().orientation().x(), msg->pose().orientation().y(), msg->pose().orientation().z());
}

GZ_REGISTER_VISUAL_PLUGIN(ModelLight)

} // namespace gazebo
