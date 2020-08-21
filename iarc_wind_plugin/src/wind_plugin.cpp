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
    gzerr << "Element <namespace>> is not specified. Aborting.";
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
  // field_calc[] = __

  struct wind_params_ params; // Input angle and speed
  params.angle = 0;
  params.speed = 5;

  CustomWindPlugin::bilinear_interpolation(params);

  // force_direction_.Set(cos(wind_angle_), sin(wind_angle_), 0);
  update_connection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&CustomWindPlugin::onUpdate, this, _1));
}

void CustomWindPlugin::bilinear_interpolation(wind_params_ params) {
  // wind_params_ ceil = params.getUpperBounds();
  // wind_params_ floor = params.getLowerBounds();

  // struct dynamics_ bot_left = field_table[wind_params_(floor.angle,
  // floor.speed)]; struct dynamics_ bot_right =
  // field_table[wind_params_(ceil.angle, floor.speed)]; struct dynamics_
  // top_left = field_table[wind_params_(floor.angle, ceil.speed)]; struct
  // dynamics_ top_right = field_table[wind_params_(ceil.angle, ceil.speed)];

  interp_force =
      (bot_left * (ceil.angle - params.angle) * (ceil.speed - params.speed) +
       bot_right * (params.angle - floor.angle) * (ceil.speed - params.speed) +
       top_left * (ceil.angle - params.angle) * (params.speed - floor.speed) +
       top_right * (params.angle - floor.angle) * (params.speed - floor.speed));
  interp_force /= (ceil.angle - floor.angle) * (ceil.speed - floor.speed);
};

void CustomWindPlugin::onUpdate(const common::UpdateInfo &_info) {
  // link_->AddForce(windspeed_ * force_direction_);
  // link->AddForce(interp_force);
}

GZ_REGISTER_MODEL_PLUGIN(CustomWindPlugin);

} // namespace gazebo
