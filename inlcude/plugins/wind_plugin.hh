#ifndef GAZEBO_PLUGINS_WINDPLUGIN_HH_
#define GAZEBO_PLUGINS_WINDPLUGIN_HH_

#include "gazebo/common/Plugin.hh"
#include "gazebo/physics/physics.hh"
#include <ignition/math/Vector3.hh>
#include <memory>

namespace gazebo
{
    class WindPluginPrivate;

    class GZ_PLUGIN_VISIBLE WindPlugin: public WindPlugin
    {
        public: WindPlugin();
        public: virtual void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf);

        public: ignition::math::Vector3d LinearVelocity(
            const physics::Wind *_wind,
            const physics::Entity *_entity
        );

        private: void OnUpdate();
        private: std::unique_ptr<WindPluginPrivate> dataPtr;
    };
}

//using namespace gazebo
#endif
