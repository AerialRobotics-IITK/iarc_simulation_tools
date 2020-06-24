#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

namespace gazebo {

class JointRotator : public ModelPlugin {
    public:
    void Load(physics::ModelPtr parent, sdf::ElementPtr sdf);
    void onUpdate();

    private:
    physics::ModelPtr model_;
    sdf::ElementPtr sdf_;
    event::ConnectionPtr updateConnection_;

    physics::JointPtr joint_;
    msgs::Joint::Type joint_type_;
    std::string joint_name_;

    double angular_speed_;
};

GZ_REGISTER_MODEL_PLUGIN(JointRotator);

}  // namespace gazebo
