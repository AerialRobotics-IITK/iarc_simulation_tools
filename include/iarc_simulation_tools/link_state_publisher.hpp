#include <functional>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>

namespace gazebo {

class LinkStatePublisher : public ModelPlugin {
    public:
        void Load(physics::ModelPtr parent, sdf::ElementPtr sdf);
        void onUpdate();

    private:
        physics::ModelPtr model_;
        sdf::ElementPtr sdf_;
        event::ConnectionPtr updateConnection_;
        physics::LinkPtr link_;

        std::string link_name_;

        transport::NodePtr node_;
        transport::PublisherPtr pose_pub_;
};

GZ_REGISTER_MODEL_PLUGIN(LinkStatePublisher);

} // namespace gazebo
