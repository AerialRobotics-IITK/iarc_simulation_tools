#pragma once

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ros/ros.h>
#include <string>
#include <iarc_sim_test_tools/common.h>
#include <thread>
#include <bits/stdc++.h>
#include <unordered_map>

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
      struct wind_params_{
        double angle;
        double speed;

      };
      struct dynamics_{
        ignition::math::Vector3d force;
        // ignition::math::Vector3d torque;
      };

      virtual ~CustomWindPlugin(){};

    protected:
        void Load(physics::ModelPtr model, sdf::ElementPtr sdf);
        void onUpdate(const common::UpdateInfo& _info);
        void bilinear_interpolation(wind_params_ params);

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

    ignition::math::Vector3d interp_force_;

    // std::unordered_map<wind_params_, dynamics_> field_table;

};
}