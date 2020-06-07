#include <iarc_simulation_tools/light_plugin.hpp>

namespace gazebo {

void ModelLight::Load(rendering::VisualPtr visual, sdf::ElementPtr sdf) {
    if (sdf->HasElement("led_color")) {
        led_color_ = sdf->GetElement("led_color")->Get<std::string>();
    } else {
        gzerr << "<led_color> tag missing. Aborting.";
        return;
    }
    model_visual_ = visual;
    std::cout << "Plugin Loaded" << std::endl;
    node_ = transport::NodePtr(new transport::Node);
    node_->Init();
    comm_pose_sub_ = node_->Subscribe("~/communications_module/link/pose/info", &ModelLight::commPoseCallback, this);
    mast_pose_sub_ = node_->Subscribe("~/table/mast/pose/info", &ModelLight::mastPoseCallback, this);
    
    updateConnection = event::Events::ConnectPreRender(std::bind(&ModelLight::OnUpdate, this));  
}

void ModelLight::OnUpdate() {
    comm_wrt_mast_position_ = mast_orientation_.inverse().toRotationMatrix() * (comm_block_position_  - mast_position_);

    comm_wrt_mast_orientation_ = mast_orientation_.inverse() * comm_block_orientation_;

    // std::cout << "x:" << comm_wrt_mast_position_[0] << " y:" << comm_wrt_mast_position_[1] << " z:" << comm_wrt_mast_position_[2] << std::endl;
    // std::cout << "x:" << comm_wrt_mast_orientation_.x() << " y:" << comm_wrt_mast_orientation_.y() << " z:" << comm_wrt_mast_orientation_.z() << " w:" << comm_wrt_mast_orientation_.w() << std::endl << std::endl;

    // bool position_match = fabs(comm_wrt_mast_position_[0] - (-0.069)) < 0.001 && fabs(comm_wrt_mast_position_[1] - (0.21)) < 0.001 && fabs(comm_wrt_mast_position_[2] - (1.032)) < 0.001;
    // bool orientation_match = fabs(comm_wrt_mast_orientation_.x() - (0.001)) < 0.001 && fabs(comm_wrt_mast_orientation_.y() - (0.001)) < 0.001 && fabs(comm_wrt_mast_orientation_.z() - (0.007)) < 0.001 && fabs(comm_wrt_mast_orientation_.z() - (0.999)) < 0.001;
    bool orientation_match = fabs(comm_wrt_mast_orientation_.angularDistance(Eigen::Quaterniond(0,0,0,1))) < 0.02;
    bool position_match = (comm_wrt_mast_position_ - Eigen::Vector3d(-0.069, 0.21, 1.032)).norm() < 0.001;

    std::cout << "correct_position: " << position_match << " Distance: " << (comm_wrt_mast_position_ - Eigen::Vector3d(-0.069, 0.21, 1.032)).norm() <<  std::endl;
    std::cout << "correct_orientation: " << orientation_match << " Angular Distance:" << comm_wrt_mast_orientation_.angularDistance(Eigen::Quaterniond(0,0,0,1)) <<std::endl;

    // comm_wrt_mast_orientatio
    // std::cout << comm_wrt_mast_position_[0] << " " << comm_wrt_mast_position_[1] << " " << comm_wrt_mast_position_[2] << std::endl;

    if (orientation_match && position_match) model_visual_->SetMaterial("Gazebo/" + led_color_ + "Glow");
    else model_visual_->SetMaterial("Gazebo/" + led_color_ + "Transparent");
}

void ModelLight::commPoseCallback(ConstPoseStampedPtr &msg) {
    // msg->PrintDebugString();
    // std::cout << std::endl << std::endl;
    // size_ = msg.get()->pose_size();
    // if (pose_names_ == NULL) pose_names_ = new std::string [size_];
    // msg->Ser
    // gazebo::msgs::Pose;  
    // std::cout << msg.get()->pose().size() << std::endl;
    // for (int i=0; i<size_;i++) {
    //     pose_names_[i] = std::string(msg->pose(i).name()); 
    // }
    // std::cout << msg->pose().position().x() << " " << msg->pose().position().y() << " " << msg->pose().position().z() << std::endl;
    comm_block_position_ = Eigen::Vector3d(msg->pose().position().x(), msg->pose().position().y(), msg->pose().position().z());
    comm_block_orientation_ = Eigen::Quaterniond(msg->pose().orientation().w(), msg->pose().orientation().x(), msg->pose().orientation().y(), msg->pose().orientation().z());  
}

void ModelLight::mastPoseCallback(ConstPoseStampedPtr &msg) {
    // msg->PrintDebugString();
    // std::cout << std::endl << std::endl;
    // size_ = msg.get()->pose_size();
    // if (pose_names_ == NULL) pose_names_ = new std::string [size_];
    // msg->Ser
    // gazebo::msgs::Pose;  
    // std::cout << msg.get()->pose().size() << std::endl;
    // for (int i=0; i<size_;i++) {
    //     pose_names_[i] = std::string(msg->pose(i).name()); 
    // }
    // std::cout << msg->pose().position().x() << " " << msg->pose().position().y() << " " << msg->pose().position().z() << std::endl;
    mast_position_ = Eigen::Vector3d(msg->pose().position().x(), msg->pose().position().y(), msg->pose().position().z());
    mast_orientation_ = Eigen::Quaterniond(msg->pose().orientation().w(), msg->pose().orientation().x(), msg->pose().orientation().y(), msg->pose().orientation().z());  
}

ModelLight::~ModelLight() {
    // FILE *fp = fopen("/home/kshitij/names.txt","a");
    // for (int i=0; i < size_; i++) fprintf(fp,"%s\n",pose_names_[i].c_str());
}

GZ_REGISTER_VISUAL_PLUGIN(ModelLight)

}