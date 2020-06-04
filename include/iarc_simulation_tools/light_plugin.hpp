#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/rendering/Visual.hh>
#include <gazebo/common/common.hh>
#include <thread>
#include <chrono>

namespace gazebo {

class GZ_PLUGIN_VISIBLE ModelLight : public VisualPlugin{
    public:
        void Load(rendering::VisualPtr parent, sdf::ElementPtr sdf) override;
        void OnUpdate();

	private:
        rendering::VisualPtr model_visual_;
        event::ConnectionPtr updateConnection;
        // int count;
        ignition::math::Color led_color_;
};
}