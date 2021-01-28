#include "gazebo_ros_link_attacher.h"
#include "gazebo_ros_link_attacher/Attach.h"
#include "gazebo_ros_link_attacher/AttachRequest.h"
#include "gazebo_ros_link_attacher/AttachResponse.h"
#include <gazebo/common/Plugin.hh>
#include <ignition/math/Pose3.hh>
#include <ros/ros.h>
// #include <gazebo/plugins/events/JointEventSource.hh>

namespace gazebo {
// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(GazeboRosLinkAttacher)

// Constructor
GazeboRosLinkAttacher::GazeboRosLinkAttacher()
    : nh_("link_attacher_node") {
}

// Destructor
GazeboRosLinkAttacher::~GazeboRosLinkAttacher() {
}

void GazeboRosLinkAttacher::Load(physics::WorldPtr _world, sdf::ElementPtr sdf) {
    // Make sure the ROS node for Gazebo has already been initialized
    if (!ros::isInitialized()) {
        ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
                         << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
        return;
    }

    if (sdf->HasElement("model1")) {
        model1_ = sdf->GetElement("model1")->Get<std::string>();
        gzmsg << model1_ << std::endl;
    } else {
        gzerr << "Element <model1>> is not specified. Aborting.";
        return;
    }

    if (sdf->HasElement("link1")) {
        link1_ = sdf->GetElement("link1")->Get<std::string>();
        gzmsg << link1_ << std::endl;
    } else {
        gzerr << "Element <link1>> is not specified. Aborting.";
        return;
    }

    if (sdf->HasElement("model2")) {
        model2_ = sdf->GetElement("model2")->Get<std::string>();
        gzmsg << model2_ << std::endl;
    } else {
        gzerr << "Element <model2>> is not specified. Aborting.";
        return;
    }

    if (sdf->HasElement("link2")) {
        link2_ = sdf->GetElement("link2")->Get<std::string>();
        gzmsg << link2_ << std::endl;
    } else {
        gzerr << "Element <link2>> is not specified. Aborting.";
        return;
    }

    if (sdf->HasElement("model_e")) {
        model_e_ = sdf->GetElement("model_e")->Get<std::string>();
        gzmsg << model_e_ << std::endl;
    } else {
        gzerr << "Element <model_e>> is not specified. Aborting.";
        return;
    }

    if (sdf->HasElement("link_e")) {
        link_e_ = sdf->GetElement("link_e")->Get<std::string>();
        gzmsg << link_e_ << std::endl;
    } else {
        gzerr << "Element <link_e>> is not specified. Aborting.";
        return;
    }

    this->world = _world;
    this->physics = this->world->Physics();
    this->attach_service_ = this->nh_.advertiseService("attach", &GazeboRosLinkAttacher::attach_callback, this);
    ROS_INFO_STREAM("Attach service at: " << this->nh_.resolveName("attach"));
    this->detach_service_ = this->nh_.advertiseService("detach", &GazeboRosLinkAttacher::detach_callback, this);
    ROS_INFO_STREAM("Detach service at: " << this->nh_.resolveName("detach"));
    ROS_INFO("Link attacher node initialized.");

    this->detach_existing_service_ = this->nh_.advertiseService("detach_existing", &GazeboRosLinkAttacher::detach_existing_callback, this);
    ROS_INFO_STREAM("Detach_existing service at: " << this->nh_.resolveName("detach_existing"));

    me_ = _world->ModelByName(model_e_);
    joint_e = me_->GetJoint(link_e_);
    // joint_e = me_->GetJoint("j1");

    updateConnection_ = event::Events::ConnectWorldUpdateBegin(
    boost::bind(&GazeboRosLinkAttacher::onUpdate, this, _1));
}

bool GazeboRosLinkAttacher::attach() {
    // look for any previous instance of the joint first.
    // if we try to create a joint in between two links
    // more than once (even deleting any reference to the first one)
    // gazebo hangs/crashes
    Prismatic j;
    if (this->getJoint(model1_, link1_, model2_, link2_, j)) {
        ROS_INFO_STREAM("Joint already existed, reusing it.");
        j.joint->Attach(j.l1, j.l2);
        return true;
    } else {
        ROS_INFO_STREAM("Creating new joint.");
    }
    j.model1 = model1_;
    j.link1 = link1_;
    j.model2 = model2_;
    j.link2 = link2_;
    ROS_DEBUG_STREAM("Getting BasePtr of " << model1_);
    physics::BasePtr b1 = this->world->ModelByName(model1_);

    if (b1 == NULL) {
        ROS_ERROR_STREAM(model1_ << " model was not found");
        return false;
    }
    ROS_DEBUG_STREAM("Getting BasePtr of " << model2_);
    physics::BasePtr b2 = this->world->ModelByName(model2_);
    if (b2 == NULL) {
        ROS_ERROR_STREAM(model2_ << " model was not found");
        return false;
    }

    ROS_DEBUG_STREAM("Casting into ModelPtr");
    physics::ModelPtr m1(dynamic_cast<physics::Model*>(b1.get()));
    j.m1 = m1;
    physics::ModelPtr m2(dynamic_cast<physics::Model*>(b2.get()));
    j.m2 = m2;

    ROS_DEBUG_STREAM("Getting link: '" << link1_ << "' from model: '" << model1_ << "'");
    physics::LinkPtr l1 = m1->GetLink(link1_);
    if (l1 == NULL) {
        ROS_ERROR_STREAM(link1_ << " link was not found");
        return false;
    }
    if (l1->GetInertial() == NULL) {
        ROS_ERROR_STREAM("link1 inertia is NULL!");
    } else
        ROS_DEBUG_STREAM("link1 inertia is not NULL, for example, mass is: " << l1->GetInertial()->Mass());
    j.l1 = l1;
    ROS_DEBUG_STREAM("Getting link: '" << link2_ << "' from model: '" << model2_ << "'");
    physics::LinkPtr l2 = m2->GetLink(link2_);
    if (l2 == NULL) {
        ROS_ERROR_STREAM(link2_ << " link was not found");
        return false;
    }
    if (l2->GetInertial() == NULL) {
        ROS_ERROR_STREAM("link2 inertia is NULL!");
    } else
        ROS_DEBUG_STREAM("link2 inertia is not NULL, for example, mass is: " << l2->GetInertial()->Mass());
    j.l2 = l2;

    ROS_DEBUG_STREAM("Links are: " << l1->GetName() << " and " << l2->GetName());

    ROS_DEBUG_STREAM("Creating revolute joint on model: '" << model1_ << "'");
    j.joint = this->physics->CreateJoint("prismatic", m1);
    this->joints.push_back(j);

    ROS_DEBUG_STREAM("Attach");
    j.joint->Attach(l1, l2);
    ROS_DEBUG_STREAM("Loading links");
    j.joint->Load(l1, l2, ignition::math::Pose3d());
    ROS_DEBUG_STREAM("SetModel");
    j.joint->SetModel(m2);
    /*
     * If SetModel is not done we get:
     * ***** Internal Program Error - assertion (this->GetParentModel() != __null)
     failed in void gazebo::physics::Entity::PublishPose():
     /tmp/buildd/gazebo2-2.2.3/gazebo/physics/Entity.cc(225):
     An entity without a parent model should not happen

     * If SetModel is given the same model than CreateJoint given
     * Gazebo crashes with
     * ***** Internal Program Error - assertion (self->inertial != __null)
     failed in static void gazebo::physics::ODELink::MoveCallback(dBodyID):
     /tmp/buildd/gazebo2-2.2.3/gazebo/physics/ode/ODELink.cc(183): Inertial pointer is NULL
     */

    ROS_DEBUG_STREAM("SetHightstop");
    j.joint->SetUpperLimit(0, 1);
    ROS_DEBUG_STREAM("SetLowStop");
    j.joint->SetLowerLimit(0, -1);
    ROS_DEBUG_STREAM("Init");
    j.joint->Init();
    ROS_INFO_STREAM("Attach finished.");

    return true;
}

bool GazeboRosLinkAttacher::detach() {
    // search for the instance of joint and do detach

    Prismatic j;
    if (this->getJoint(model1_, link1_, model2_, link2_, j)) {
        j.joint->Detach();
        return true;
    }

    return false;
}

bool GazeboRosLinkAttacher::detach_existing() {
    // search for the instance of joint and do detach

    // ROS_INFO_STREAM(joint_e);
    // if (joint_e){
    joint_e->Detach();
    return true;
    // }

    // return false;

}

bool GazeboRosLinkAttacher::getJoint(std::string model1, std::string link1, std::string model2, std::string link2, Prismatic& joint) {
    Prismatic j;
    for (std::vector<Prismatic>::iterator it = this->joints.begin(); it != this->joints.end(); ++it) {
        j = *it;
        if ((j.model1.compare(model1) == 0) && (j.model2.compare(model2) == 0) && (j.link1.compare(link1) == 0) && (j.link2.compare(link2) == 0)) {
            joint = j;
            return true;
        }
    }
    return false;
}

bool GazeboRosLinkAttacher::attach_callback(gazebo_ros_link_attacher::Attach::Request& req, gazebo_ros_link_attacher::Attach::Response& res) {
    // ROS_INFO_STREAM("Received request to attach model: '" << req.model_name_1 << "' using link: '" << req.link_name_1 << "' with model: '" <<
    // req.model_name_2
    //                                                       << "' using link: '" << req.link_name_2 << "'");
    if (!this->attach()) {
        ROS_ERROR_STREAM("Could not make the attach.");
        res.ok = false;
    } else {
        ROS_INFO_STREAM("Attach was succesful");
        res.ok = true;
    }
    return true;
}

bool GazeboRosLinkAttacher::detach_callback(gazebo_ros_link_attacher::Attach::Request& req, gazebo_ros_link_attacher::Attach::Response& res) {
    // ROS_INFO_STREAM("Received request to detach model: '" << req.model_name_1 << "' using link: '" << req.link_name_1 << "' with model: '" <<
    // req.model_name_2
    //                                                       << "' using link: '" << req.link_name_2 << "'");
    if (!this->detach()) {
        ROS_ERROR_STREAM("Could not make the detach.");
        res.ok = false;
    } else {
        ROS_INFO_STREAM("Detach was succesful");
        res.ok = true;
    }
    return true;
}

bool GazeboRosLinkAttacher::detach_existing_callback(gazebo_ros_link_attacher::Attach::Request& req, gazebo_ros_link_attacher::Attach::Response& res) {
    // ROS_INFO_STREAM("Received request to detach model: '" << req.model_name_1 << "' using link: '" << req.link_name_1 << "' with model: '" <<
    // req.model_name_2
    //                                                       << "' using link: '" << req.link_name_2 << "'");
    if (!this->detach_existing()) {
        ROS_ERROR_STREAM("Could not make the detach.");
        res.ok = false;
    } else {
        ROS_INFO_STREAM("Detach was succesful");
        res.ok = true;
    }
    return true;
}


void GazeboRosLinkAttacher::onUpdate(const common::UpdateInfo &_info) {

//   link_->AddRelativeTorque(torque_direction_ * torque_magnitude_);


}


}  // namespace gazebo
