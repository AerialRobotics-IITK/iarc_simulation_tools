#include <iarc_wave_sim_gazebo/ros_sea_state_subscriber.hpp>

namespace iarc_simulation_tools {

void RosBridge::init(ros::NodeHandle& nh) {
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

    sea_state_sub_ = nh.subscribe("/sea_state", 10, &RosBridge::getSeaState, this);
    sea_direction_sub_ = nh.subscribe("/sea_direction", 10, &RosBridge::getSeaDirection, this);
    wave_pub_ = node_->Advertise<gazebo::msgs::Param_V>("~/wave");
    fft_wave_pub_ = node_->Advertise<gazebo::msgs::Param_V>("~/wave/wind");
}

void RosBridge::getSeaState(const std_msgs::Int32& msg) {

    if ( current_sea_state_.data != msg.data ) {
        current_sea_state_.data = msg.data;
        state_changed_ = true;
    }
}

void RosBridge::getSeaDirection(const geometry_msgs::Point& msg) {
    
    if(direction_ != msg) {

        direction_changed_ = true;
        direction_.x = msg.x;
        direction_.y = msg.y;
        // RosBridge::publishWaveState();
    }
}

void RosBridge::publish() {

    if (state_changed_ == true || direction_changed_ == true) {

        direction_changed_ = false; 
        state_changed_ = false;

        // 0 for trochoidal waves   
        // 1 for fft waves

        if (wave_type_ == 0) {
            RosBridge::publishWaveState();
        }

        else {
            RosBridge::publishWaveStateFFT();
        }

    }

}

void RosBridge::getWindDirection(const rotors_comm::WindSpeed& msg) {
    double x = msg.velocity.x;
    double y = msg.velocity.y;

    // double norm = sqrt(pow(x, 2) + pow(y, 2));

    // direction_.x = x / norm;
    // direction_.y = y / norm;

    states_.v_param_[current_sea_state_.data].wind.angle = std::atan(y/x);
}

void RosBridge::publishWaveStateFFT() {

    auto wind_param = states_.v_param_[current_sea_state_.data].wind;
    gazebo::msgs::Param_V fft_msg_;
    gazebo::msgs::Param* nextParam = fft_msg_.add_param();

    nextParam->set_name("wind_angle");
    nextParam->mutable_value()->set_type(gazebo::msgs::Any::DOUBLE);
    nextParam->mutable_value()->set_double_value(std::atan(direction_.y / direction_.x));

    nextParam = fft_msg_.add_param();
    nextParam->set_name("wind_speed");
    nextParam->mutable_value()->set_type(gazebo::msgs::Any::DOUBLE);
    nextParam->mutable_value()->set_double_value(wind_param.speed);
    
    fft_wave_pub_->WaitForConnection(gazebo::common::Time(1, 0));

    fft_wave_pub_->Publish(fft_msg_, true);

    ROS_INFO_STREAM("Publishing on topic [" << fft_wave_pub_->GetTopic() << "]");
    ROS_INFO_STREAM(fft_msg_.DebugString());
}

void RosBridge::publishWaveState() {

    auto v_param = states_.v_param_[current_sea_state_.data];
    gazebo::msgs::Param_V waveMsg;
    gazebo::msgs::Param* nextParam = waveMsg.add_param();

    nextParam->set_name("number");
    nextParam->mutable_value()->set_type(gazebo::msgs::Any::INT32);
    nextParam->mutable_value()->set_int_value(v_param.number);

    nextParam = waveMsg.add_param();
    nextParam->set_name("scale");
    nextParam->mutable_value()->set_type(gazebo::msgs::Any::DOUBLE);
    nextParam->mutable_value()->set_double_value(v_param.number);

    nextParam = waveMsg.add_param();
    nextParam->set_name("angle");
    nextParam->mutable_value()->set_type(gazebo::msgs::Any::DOUBLE);
    nextParam->mutable_value()->set_double_value(v_param.angle);

    nextParam = waveMsg.add_param();
    nextParam->set_name("amplitude");
    nextParam->mutable_value()->set_type(gazebo::msgs::Any::DOUBLE);
    nextParam->mutable_value()->set_double_value(v_param.amplitude);

    nextParam = waveMsg.add_param();
    nextParam->set_name("period");
    nextParam->mutable_value()->set_type(gazebo::msgs::Any::DOUBLE);
    nextParam->mutable_value()->set_double_value(v_param.period);

    nextParam = waveMsg.add_param();
    nextParam->set_name("direction");
    nextParam->mutable_value()->set_type(gazebo::msgs::Any::VECTOR3D);
    nextParam->mutable_value()->mutable_vector3d_value()->set_x(direction_.x);
    nextParam->mutable_value()->mutable_vector3d_value()->set_y(direction_.y);

    wave_pub_->WaitForConnection(gazebo::common::Time(1, 0));

    wave_pub_->Publish(waveMsg, true);

    std::cout << "Publishing on topic [" << wave_pub_->GetTopic() << "]" << std::endl;
    std::cout << waveMsg.DebugString() << std::endl;
}

}  // namespace iarc_simulation_tools