#include <iarc_state_plugins/revolute_joint_rotator_plugin.hpp>

namespace gazebo {

void JointRotator::Load(physics::ModelPtr parent, sdf::ElementPtr sdf) {
    model_ = parent;
    sdf_ = sdf;

    gzmsg << "Joint Plugin loaded for " << model_->GetName() << std::endl;

    if (sdf->HasElement("joint_name")) {
        joint_name_ = sdf->GetElement("joint_name")->Get<std::string>();
    } else {
        gzerr << "Element <joint_name> is not specified. Aborting.";
        return;
    }

    if (sdf->HasElement("angular_speed")) {
        angular_speed_ = sdf->GetElement("angular_speed")->Get<double>();
    } else {
        gzwarn << "Element <angular_speed> is not specified. Using default value of 1 rad/s.";
        angular_speed_ = 1.0;
    }

    joint_ = model_->GetJoint(parent->GetName() + "::" + joint_name_);

    joint_type_ = joint_->GetMsgType();

    if (joint_type_ != msgs::Joint::Type::Joint_Type_REVOLUTE) {
        gzerr << "The specified joint is not a revolute joint. Aborting.";
        return;
    }

    updateConnection_ = event::Events::ConnectWorldUpdateBegin(std::bind(&JointRotator::onUpdate, this));
}

void JointRotator::onUpdate() {
    joint_->SetVelocity(0, angular_speed_);
}
}  // namespace gazebo
