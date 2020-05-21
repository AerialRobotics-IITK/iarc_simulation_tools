#include <plugins/orientation_controller_example.hpp>

namespace gazebo {

void ModelOrientation::Load(physics::ModelPtr parent, sdf::ElementPtr sdf_) {
    model_ = parent;

    updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&ModelOrientation::OnUpdate, this));  
}

void ModelOrientation::OnUpdate() {
    static int count = 0;
    model_->SetWorldPose(ignition::math::v4::Pose3d(0,0,0,0,0,0.3925*count));
    count++;
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    if (count == 16) count = 0;
}
}