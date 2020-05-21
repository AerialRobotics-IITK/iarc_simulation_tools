#include <chrono>
#include <functional>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>
#include <thread>

namespace gazebo {

class ModelOrientation : public ModelPlugin {
	public:
        void Load(physics::ModelPtr parent, sdf::ElementPtr sdf);
        void OnUpdate();

	private:
        physics::ModelPtr model_;
        event::ConnectionPtr updateConnection;
};

GZ_REGISTER_MODEL_PLUGIN(ModelOrientation);
} // namespace gazebo