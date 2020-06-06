#include <iarc_simulation_tools/light_plugin.hpp>

namespace gazebo {

void ModelLight::Load(rendering::VisualPtr visual_, sdf::ElementPtr sdf_) {
    if (sdf_->HasElement("led_color")) led_color_ = sdf_->Get<ignition::math::Color>("led_color");
    else led_color_ = ignition::math::Color(1,0,0,0);
    model_visual_ = visual_;
    std::cout << "Plugin Loaded" << std::endl;
    node_ = transport::NodePtr(new transport::Node);
    node_->Init();
    comm_pose_sub_ = node_->Subscribe("~/communications_module/link/pose/info", &ModelLight::commPoseCallback, this);
    mast_pose_sub_ = node_->Subscribe("~/table/mast/pose/info", &ModelLight::mastPoseCallback, this);
    
    updateConnection = event::Events::ConnectPreRender(std::bind(&ModelLight::OnUpdate, this));  
}

void ModelLight::OnUpdate() {
    model_visual_->SetEmissive(led_color_);
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