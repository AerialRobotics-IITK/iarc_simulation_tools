#include <bits/stdc++.h>
#include <cmath>
#include <cstdlib>
#include <iarc_wind_plugin/wind_plugin.hpp>
#include <iostream>
#include <math.h>

namespace gazebo {

void CustomWindPlugin::Load(physics::ModelPtr parent, sdf::ElementPtr sdf) {
    model_ = parent;
    sdf_ = sdf;

    gzmsg << "Wind plugin loaded for " << model_->GetName() << std::endl;

    if (sdf->HasElement("namespace")) {
        namespace_ = sdf->GetElement("namespace")->Get<std::string>();
        gzmsg << namespace_ << std::endl;
    } else {
        gzerr << "Element <namespace> is not specified. Aborting.";
        return;
    }

    if (sdf->HasElement("link_name")) {
        link_name_ = sdf->GetElement("link_name")->Get<std::string>();
    } else {
        gzerr << "Element <link_name> is not specified. Aborting.";
        return;
    }

    if (sdf->HasElement("windspeed")) {
        windspeed_ = sdf->GetElement("windspeed")->Get<double>();
    } else {
        windspeed_ = 1.0;
    }

    if (sdf->HasElement("windangle")) {
        wind_angle_ = sdf->GetElement("windangle")->Get<double>();
    } else {
        wind_angle_ = 0;
    }

    link_ = model_->GetLink(link_name_);

    // Input values from cfd analysis
    initializeFieldTable();

    WindParams test_params(1, 2);  // Input angle and speed
    gzmsg << interpolateWindDynamics(test_params) << "\n";

    // force_direction_.Set(cos(wind_angle_), sin(wind_angle_), 0);
    update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&CustomWindPlugin::onUpdate, this, _1));
}

ignition::math::Vector3d CustomWindPlugin::interpolateWindDynamics(WindParams params) {
    ignition::math::Vector3d interp_force;

    WindParams ceil = params;  // impl getUpperBounds
    ceil.angle++;
    ceil.speed++;
    WindParams floor = params;  // impl getLowerBounds
    floor.angle--;
    floor.speed--;

    CustomWindPlugin::DynParams bot_left = field_table_[CustomWindPlugin::WindParams(floor.angle, floor.speed)];
    CustomWindPlugin::DynParams bot_right = field_table_[CustomWindPlugin::WindParams(ceil.angle, floor.speed)];
    CustomWindPlugin::DynParams top_left = field_table_[CustomWindPlugin::WindParams(floor.angle, ceil.speed)];
    CustomWindPlugin::DynParams top_right = field_table_[CustomWindPlugin::WindParams(ceil.angle, ceil.speed)];

    /*
        interp_force =
            (bot_left * (ceil.angle - params.angle) * (ceil.speed - params.speed) + bot_right * (params.angle - floor.angle) * (ceil.speed - params.speed) +
                top_left * (ceil.angle - params.angle) * (params.speed - floor.speed) + top_right * (params.angle - floor.angle) * (params.speed -
       floor.speed)); interp_force /= (ceil.angle - floor.angle) * (ceil.speed - floor.speed);
    */
    return interp_force;
};

void CustomWindPlugin::onUpdate(const common::UpdateInfo& _info) {
    // link_->AddForce(windspeed_ * force_direction_);
    // link->AddForce(interp_force);
}

void CustomWindPlugin::initializeFieldTable() {
    // dummy initialization for testing
    for (int i = 0; i < 10; i++) {
        field_table_[WindParams(i, i + 1)] =
            CustomWindPlugin::DynParams(ignition::math::Vector3d(i + 1, i + 1, i + 1), ignition::math::Vector3d(i + 2, i + 2, i + 2));
    }
}

GZ_REGISTER_MODEL_PLUGIN(CustomWindPlugin);

}  // namespace gazebo
