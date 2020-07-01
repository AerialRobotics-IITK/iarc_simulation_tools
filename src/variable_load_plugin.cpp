// #include <fstream>
#include <iarc_simulation_tools/variable_load_plugin.hpp>
#include <iostream>
// #include <math.h>s

namespace gazebo {

GazeboVarForcePlugin::~GazeboVarForcePlugin() {}

void GazeboVarForcePlugin::Load(physics::ModelPtr _model,
                                sdf::ElementPtr _sdf) {

  this->model_ = _model;

  if (_sdf->HasElement("robotNamespace"))
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  else
    gzerr << "[gazebo_var_load_plugin] Please specify a robotNamespace.\n";

  getSdfParam<std::string>(_sdf, "linkName", link_name_, link_name_);

  link_ = model_->GetLink(link_name_);
  if (link_ == NULL)
    gzthrow("[gazebo_var_load_plugin] Couldn't find specified link \""
            << link_name_ << "\".");

  force_direction_.Set(0, 0, -1);

  if (!ros::isInitialized()) {
    int argc = 0;
    char **argv = NULL;
    ros::init(argc, argv, "gazebo_client", ros::init_options::NoSigintHandler);
  }

  this->rosNode.reset(new ros::NodeHandle("gazebo_client"));

  ros::SubscribeOptions so =
      ros::SubscribeOptions::create<iarc_simulation_tools::LoadParams>(
          "/" + this->model_->GetName() + "/variable_force", 1,
          boost::bind(&GazeboVarForcePlugin::OnRosMsg, this, _1),
          ros::VoidPtr(), &this->rosQueue);
  this->rosSub = this->rosNode->subscribe(so);

  this->rosQueueThread =
      std::thread(std::bind(&GazeboVarForcePlugin::QueueThread, this));

  update_connection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&GazeboVarForcePlugin::OnUpdate, this, _1));
}

void GazeboVarForcePlugin::OnRosMsg(
    const iarc_simulation_tools::LoadParamsConstPtr &_msg) {
  xyz_offset_.Set(_msg->x, _msg->y, _msg->z);
  mass = _msg->mass;
}

void GazeboVarForcePlugin::QueueThread() {
  static const double timeout = 0.01;
  while (this->rosNode->ok()) {
    this->rosQueue.callAvailable(ros::WallDuration(timeout));
  }
}
void GazeboVarForcePlugin::OnUpdate(const common::UpdateInfo &_info) {

  link_->AddForceAtRelativePosition(mass * force_direction_, xyz_offset_);
}
GZ_REGISTER_MODEL_PLUGIN(GazeboVarForcePlugin);
} // namespace gazebo
