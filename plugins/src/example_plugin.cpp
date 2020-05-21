#include <gazebo/gazebo.hh>

namespace gazebo {

class WorldPluginTutorial : public WorldPlugin {
    public:
        WorldPluginTutorial() : WorldPlugin() {
            std::cout << "HELLLLLLO" << std::flush;
        }

        void Load(physics::WorldPtr, sdf::ElementPtr _sdf){}
};

GZ_REGISTER_WORLD_PLUGIN(WorldPluginTutorial);
}