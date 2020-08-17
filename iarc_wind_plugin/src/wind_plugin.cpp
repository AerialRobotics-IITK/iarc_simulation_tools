#include <iarc_wind_plugin/wind_plugin.hpp>
#include <iostream>
#include <math.h>
#include <cmath>

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
    gzerr << "Element <link_name> is not specified. Aborting.";
    return;
  }

  link_ = model_->GetLink( link_name_ );

  force_direction_.Set(cos(wind_angle_), sin(wind_angle_), 0);

  update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&CustomWindPlugin::onUpdate, this, _1));

}

void CustomWindPlugin::onUpdate(const common::UpdateInfo& _info){
    link_->AddForce(windspeed_ * force_direction_);
}

GZ_REGISTER_MODEL_PLUGIN(CustomWindPlugin);

} //namespace gazbeo
