#include <iarc_simulation_tools/variable_load_plugin.hpp>

// #include "ConnectGazeboToRosTopic.pb.h"
// #include "ConnectRosToGazeboTopic.pb.h"
#include <iostream>
#include <math.h>

namespace gazebo {

GazeboVarForcePlugin::~GazeboVarForcePlugin() {}

void GazeboVarForcePlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
    
    this->model_ = _model;
    world_ = model_->GetWorld();

    if (_sdf->HasElement("robotNamespace"))
        namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
    else
        gzerr << "[gazebo_wind_plugin] Please specify a robotNamespace.\n";

    // node_handle_ = gazebo::transport::NodePtr(new transport::Node);
    // node_handle_->Init();

    // getSdfParam<std::string>(_sdf, "frameId", frame_id_, frame_id_);
    // getSdfParam<std::string>(_sdf, "linkName", link_name_, link_name_);

    // link_ = model_->GetLink(link_name_);
    // if (link_ == NULL)
    //     gzthrow("[gazebo_wind_plugin] Couldn't find specified link \"" << link_name_
    //                                                                << "\".");
    
    // params_sub_ = node_handle_->Subscribe("var_force_topic", &GazeboVarForcePlugin::cb, this);
    
    // Initialize ros, if it has not already bee initialized.
    if (!ros::isInitialized())
    {
    int argc = 0;
    char **argv = NULL;
    ros::init(argc, argv, "gazebo_client",
        ros::init_options::NoSigintHandler);
    }

    this->rosNode.reset(new ros::NodeHandle("gazebo_client"));

    ros::SubscribeOptions so =
        ros::SubscribeOptions::create<std_msgs::Float32MultiArray>(
            "/" + this->model_->GetName() + "/vel_cmd",
            // "/testing_ros_topic",
            1,
            boost::bind(&GazeboVarForcePlugin::OnRosMsg, this, _1),
            ros::VoidPtr(), &this->rosQueue);
    this->rosSub = this->rosNode->subscribe(so);

    this->rosQueueThread =
        std::thread(std::bind(&GazeboVarForcePlugin::QueueThread, this));

    update_connection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&GazeboVarForcePlugin::OnUpdate, this, _1));

}

// void GazeboVarForcePlugin::cb(const boost::shared_ptr<const iarc_simulation_tools::var_load>& msg){
//     // var_load_ = msg;
//     std::cout << "x : " << msg.x << std::endl;
//     std::cout << "m : " << msg.m << std::endl;
// }

void GazeboVarForcePlugin::OnRosMsg(const std_msgs::Float32MultiArrayConstPtr &_msg)
{
  std::cout << "ros message : " << _msg->data[0] << std::endl;
}

void  GazeboVarForcePlugin::QueueThread()
{
  static const double timeout = 0.01;
  while (this->rosNode->ok())
  {
    this->rosQueue.callAvailable(ros::WallDuration(timeout));
  }
}
void GazeboVarForcePlugin::OnUpdate(const common::UpdateInfo &_info){

    common::Time now = world_->SimTime();
    // std::cout << "THE VARLOAD PLUGIN IS INCLUDED PROPERLY" << std::endl;

    // while(true){
    //     gazebo::common::Time::MSleep(10);
    // }


}


GZ_REGISTER_MODEL_PLUGIN(GazeboVarForcePlugin);

}
