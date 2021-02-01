#include <iarc_battery_plugin/battery_plugin.hpp>

namespace gazebo {
void BatteryPlugin::Load(physics::ModelPtr parent, sdf::ElementPtr sdf) {
  model_ = parent;
  sdf_ = sdf;

  std::cout <<"fuck you"<<std::endl;

  gzmsg << "Battery plugin loaded for " << model_->GetName() << std::endl;
  if (sdf->HasElement("namespace")) {
    namespace_ = sdf->GetElement("namespace")->Get<std::string>();
    gzmsg << namespace_ << std::endl;
  } else {
    gzerr << "Element <namespace> is not specified. Aborting.";
    return;
  }
  if (sdf->HasElement("topic_name")) {
    topic_name_ = sdf->GetElement("topic_name")->Get<std::string>();
    gzmsg << topic_name_ << std::endl;
  } else {
    gzerr << "Element <topic_name> is not specified. Aborting.";
    return;
  }
  if (sdf->HasElement("time_limit")) {
    time_limit_ = sdf->GetElement("time_limit")->Get<double>();
    gzmsg << time_limit_ << std::endl;
  } else {
    gzerr << "Element <time_limit> is not specified. Aborting.";
    return;
  }
  rosnode_ = new ros::NodeHandle(namespace_);
  battery_status_ = rosnode_->advertise<std_msgs::Float32>("/time_left", 10);
  set_time_limit_ = rosnode_->subscribe("/set_time_limit", 1, &BatteryPlugin::setTimeCallback, this);

    updateConnection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&BatteryPlugin::onUpdate, this, _1));
}

void BatteryPlugin::setTimeCallback(const std_msgs::Float32 &msg) {
    time_limit_ = msg.data;
}
void BatteryPlugin::onUpdate(const common::UpdateInfo &_info) {
    float time_left = time_limit_ - (ros::Time::now().toSec())/60;
    battery_status_.publish(time_left);
    if(time_left < 0) {
        //kill node
    }
}

GZ_REGISTER_MODEL_PLUGIN(BatteryPlugin);

} //namespace gazebo