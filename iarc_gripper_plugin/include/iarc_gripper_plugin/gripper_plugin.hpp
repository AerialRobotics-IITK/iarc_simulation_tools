#pragma once

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <string>
#include <thread>

namespace gazebo{

class GripperPlugin:public ModelPlugin{
    public:
        void Load(physics::ModelPtr parent, sdf::ElementPtr sdf);
        void onUpdate(const common::UpdateInfo& /*_info*/);

    private:

    physics::ModelPtr model_;
    sdf::ElementPtr sdf_;
    event::ConnectionPtr updateConnection_;
    physics::LinkPtr link1_;
    physics::LinkPtr link2_;

    std::string link_name1_;
    std::string link_name2_;
    std::string namespace_;
    ignition::math::Vector3d force_direction_;
    ignition::math::Vector3d xyz_offset_;

    double force_magnitude_;
    int flag;
    
};

}