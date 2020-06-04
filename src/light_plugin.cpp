#include <iarc_simulation_tools/light_plugin.hpp>

namespace gazebo {

void ModelLight::Load(rendering::VisualPtr visual_, sdf::ElementPtr sdf_) {
    // count = 1;
    if (sdf_->HasElement("led_color")) led_color_ = sdf_->Get<ignition::math::Color>("led_color");
    else led_color_ = ignition::math::Color(1,0,0,0);
    model_visual_ = visual_;
    std::cout << "Plugin Loaded" << std::endl;
    updateConnection = event::Events::ConnectPreRender(std::bind(&ModelLight::OnUpdate, this));  
}

void ModelLight::OnUpdate() {
    model_visual_->SetEmissive(led_color_);
}

GZ_REGISTER_VISUAL_PLUGIN(ModelLight)

}