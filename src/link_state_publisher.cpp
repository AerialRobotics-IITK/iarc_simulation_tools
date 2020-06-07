#include <iarc_simulation_tools/link_state_publisher.hpp>

namespace gazebo {

void LinkStatePublisher::Load(physics::ModelPtr parent, sdf::ElementPtr sdf) {
    model_ = parent;
    sdf_ = sdf;

    if (sdf->HasElement("link_name")) {
        link_name_ = sdf->GetElement("link_name")->Get<std::string>();
    } else {
        gzerr << "Link name doesn't exist in the model, or is not specified. Aborting.";
        return;
    }

    std::cout << "Link plugin loaded for " << model_->GetName() << std::endl;

    node_ = transport::NodePtr(new transport::Node);
    node_->Init();
    std::string topic_name = "~/" + model_->GetName() + "/" + link_name_ + "/pose/info";
    pose_pub_ = node_->Advertise<gazebo::msgs::PoseStamped>(topic_name, 5, 30);

    updateConnection_ = event::Events::ConnectWorldUpdateBegin(std::bind(&LinkStatePublisher::onUpdate, this));
}

void LinkStatePublisher::onUpdate() {
    link_ = model_->GetLink(model_->GetName() + "::" + link_name_);

    ignition::math::v6::Pose3d link_pose;
    link_pose = link_->WorldPose();

    gazebo::common::Time current_time = common::Time::GetWallTime();
    gazebo::msgs::PoseStamped link_pose_msg;

    link_pose_msg.mutable_pose()->mutable_position()->set_x(link_pose.Pos()[0]);
    link_pose_msg.mutable_pose()->mutable_position()->set_y(link_pose.Pos()[1]);
    link_pose_msg.mutable_pose()->mutable_position()->set_z(link_pose.Pos()[2]);

    link_pose_msg.mutable_pose()->mutable_orientation()->set_x(link_pose.Rot().X());
    link_pose_msg.mutable_pose()->mutable_orientation()->set_y(link_pose.Rot().Y());
    ;
    link_pose_msg.mutable_pose()->mutable_orientation()->set_z(link_pose.Rot().Z());
    link_pose_msg.mutable_pose()->mutable_orientation()->set_w(link_pose.Rot().W());

    link_pose_msg.mutable_time()->set_sec(current_time.sec);
    link_pose_msg.mutable_time()->set_nsec(current_time.nsec);

    pose_pub_->Publish(link_pose_msg);
}

} // namespace gazebo
