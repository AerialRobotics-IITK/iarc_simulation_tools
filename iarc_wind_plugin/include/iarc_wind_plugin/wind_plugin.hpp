#pragma once

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ros/ros.h>
#include <string>
#include <iarc_sim_test_tools/common.h>
#include <thread>

namespace gazebo{

static const std::string kDefualFrameId = "world";
static const std::string kDefaultLinkName = "base_link";

class CustomWindPlugin : public ModelPlugin {
    public:
      CustomWindPlugin()
        : ModelPlugin()
        , namespace_(kDefaultNamespace)
        , link_name_(kDefaultLinkName) {
        
      }

      virtual ~CustomWindPlugin(){};

    protected:
        void Load(physics::ModelPtr model, sdf::ElementPtr sdf);
        void onUpdate(const common::UpdateInfo& _info);

    private:
    event::ConnectionPtr update_connection_;
    
    physics::ModelPtr model_;
    sdf::ElementPtr sdf_;
    physics::LinkPtr link_;
    
    std::string namespace_;
    std::string link_name_;
    
    ignition::math::Vector3d xyz_offset_;
    ignition::math::Vector3d force_direction_;

    double windspeed_;
    double wind_angle_;

};
}