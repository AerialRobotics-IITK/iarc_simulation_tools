#include <iarc_battery_plugin/battery_plugin.hpp>

namespace gazebo {

void BatteryPlugin::Load(physics::ModelPtr parent, sdf::ElementPtr sdf) {
  model_ = parent;
  sdf_ = sdf;
  world_ = model_->GetWorld();

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
  if (sdf->HasElement("node_name")) {
    node_name_ = sdf->GetElement("node_name")->Get<std::string>();
    gzmsg << node_name_ << std::endl;
  } else {
    gzerr << "Element <node_name> is not specified. Aborting.";
    return;
  }
  if (sdf->HasElement("time_limit")) {
    time_limit_ = sdf->GetElement("time_limit")->Get<double>();
    gzmsg << time_limit_ << std::endl;
  } else {
    gzerr << "Element <time_limit> is not specified. Aborting.";
    return;
  }
  if (sdf->HasElement("total_motors")) {
    total_motors_ = sdf->GetElement("total_motors")->Get<int>();
    gzmsg << total_motors_ << std::endl;
  } else {
    gzerr << "Element <total_motors> is not specified. Aborting.";
    return;
  }

  // node_handle_ = gazebo::transport::NodePtr(new transport::Node());

  // Initialise with default namespace (typically /gazebo/default/)
  // node_handle_->Init();
  rosnode_ = new ros::NodeHandle();
  motor_speed_pub_ = rosnode_->advertise<mav_msgs::Actuators>(topic_name_, 10);
  battery_status_ = rosnode_->advertise<std_msgs::Float32>("/time_left", 10);
  set_time_limit_ = rosnode_->subscribe("/set_time_limit", 1, &BatteryPlugin::setTimeCallback, this);

  now = world_->SimTime();

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
      for (int i = 0; i < total_motors_; i++) {
        motor_speeds_.angular_velocities.push_back(0);
      }
      // motor_speeds_.header.stamp.nsec = 0;
      motor_speeds_.header.stamp.sec = ros::Time::now().toSec();
      motor_speeds_.header.stamp.nsec = ros::Time::now().toNSec();
      
      const char* str = ("rosnode kill "+ node_name_).c_str();
      if(!count_) std::system(str);
      motor_speed_pub_.publish(motor_speeds_);
      count_++;
        
    }
}

GZ_REGISTER_MODEL_PLUGIN(BatteryPlugin);

} //namespace gazebo