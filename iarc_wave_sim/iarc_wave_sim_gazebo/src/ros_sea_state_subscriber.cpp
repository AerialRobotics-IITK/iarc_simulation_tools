#include <iarc_wave_sim_gazebo/ros_sea_state_subscriber.hpp>

namespace iarc_simulation_tools {

void ROSBridge::init(ros::NodeHandle &nh) {
  gazebo::transport::init();
  gazebo::transport::run();
  node_ = gazebo::transport::NodePtr(new gazebo::transport::Node());
  node_->Init();
  current_sea_state_.data = 3;
  direction_.x = 1;
  direction_.y = 1;
  direction_.z = 0;
  wave_type_ = 1;
  state_changed_ = true;
  direction_changed_ = true;

  sea_state_sub_ = nh.subscribe("sea_state", 10, &ROSBridge::getSeaState, this);
  sea_direction_sub_ =
      nh.subscribe("sea_direction", 10, &ROSBridge::getSeaDirection, this);
  wave_pub_ = node_->Advertise<gazebo::msgs::Param_V>("~/wave");
  fft_wave_pub_ = node_->Advertise<gazebo::msgs::Param_V>("~/wave/wind");
}

gazebo::msgs::Param_V addParamInt(std::string param_name, int value,
                                  gazebo::msgs::Param_V &msg) {

  gazebo::msgs::Param *nextParam = msg.add_param();
  nextParam->set_name(param_name);
  nextParam->mutable_value()->set_type(gazebo::msgs::Any::INT32);
  nextParam->mutable_value()->set_int_value(value);

  return msg;
}

gazebo::msgs::Param_V addParamDouble(std::string param_name, double value,
                                     gazebo::msgs::Param_V &msg) {

  gazebo::msgs::Param *nextParam = msg.add_param();
  nextParam->set_name(param_name);
  nextParam->mutable_value()->set_type(gazebo::msgs::Any::DOUBLE);
  nextParam->mutable_value()->set_double_value(value);

  return msg;
}

gazebo::msgs::Param_V addParamVector3D(std::string param_name,
                                       geometry_msgs::Point value,
                                       gazebo::msgs::Param_V &msg) {

  gazebo::msgs::Param *nextParam = msg.add_param();
  nextParam->set_name(param_name);
  nextParam->mutable_value()->set_type(gazebo::msgs::Any::VECTOR3D);
  nextParam->mutable_value()->mutable_vector3d_value()->set_x(value.x);
  nextParam->mutable_value()->mutable_vector3d_value()->set_y(value.y);

  return msg;
}

void ROSBridge::getSeaState(const std_msgs::Int32 &msg) {

  if (current_sea_state_.data != msg.data) {
    current_sea_state_.data = msg.data;
    state_changed_ = true;
  }
}

void ROSBridge::getSeaDirection(const geometry_msgs::Point &msg) {

  if (direction_ != msg) {

    direction_changed_ = true;
    direction_.x = msg.x;
    direction_.y = msg.y;
  }
}

void ROSBridge::publish() {

  if (state_changed_ == true || direction_changed_ == true) {

    direction_changed_ = false;
    state_changed_ = false;

    // 0 for trochoidal waves
    // 1 for fft waves

    if (wave_type_ == 0) {
      ROSBridge::publishWaveState();
    } else {
      ROSBridge::publishWaveStateFFT();
    }
  }
}

void ROSBridge::getWindDirection(const rotors_comm::WindSpeed &msg) {
  double x = msg.velocity.x;
  double y = msg.velocity.y;

  double norm = sqrt(pow(x, 2) + pow(y, 2));

  // direction_.x = x / norm;
  // direction_.y = y / norm;

  // states_.v_param_[current_sea_state_.data].wind.angle = std::atan(y / x);
}

void ROSBridge::publishWaveStateFFT() {

  auto wind_param = states_.v_param_[current_sea_state_.data].wind;
  gazebo::msgs::Param_V fft_msg;

  fft_msg = addParamDouble("wind_angle", std::atan2(direction_.y, direction_.x),
                           fft_msg);
  fft_msg = addParamDouble("wind_speed", wind_param.speed, fft_msg);

  fft_wave_pub_->WaitForConnection(gazebo::common::Time(1, 0));

  fft_wave_pub_->Publish(fft_msg, true);

  ROS_INFO_STREAM("Publishing on topic [" << fft_wave_pub_->GetTopic() << "]");
  ROS_INFO_STREAM(fft_msg.DebugString());
}

void ROSBridge::publishWaveState() {

  auto v_param = states_.v_param_[current_sea_state_.data];
  gazebo::msgs::Param_V waveMsg;

  waveMsg = addParamInt("number", v_param.number, waveMsg);
  waveMsg = addParamDouble("scale", v_param.scale, waveMsg);
  waveMsg = addParamDouble("angle", v_param.angle, waveMsg);
  waveMsg = addParamDouble("amplitude", v_param.amplitude, waveMsg);
  waveMsg = addParamDouble("period", v_param.period, waveMsg);
  waveMsg = addParamVector3D("direction", direction_, waveMsg);

  wave_pub_->WaitForConnection(gazebo::common::Time(1, 0));

  wave_pub_->Publish(waveMsg, true);

  ROS_INFO_STREAM("Publishing on topic [" << wave_pub_->GetTopic() << "]");
  ROS_INFO_STREAM(waveMsg.DebugString());
}

} // namespace iarc_simulation_tools