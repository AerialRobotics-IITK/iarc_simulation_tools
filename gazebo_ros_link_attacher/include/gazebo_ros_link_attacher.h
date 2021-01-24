/*
 * Desc: Gazebo link attacher plugin.
 * Author: Sammy Pfeiffer (sam.pfeiffer@pal-robotics.com)
 * Date: 05/04/2016
 */

#ifndef GAZEBO_ROS_LINK_ATTACHER_HH
#define GAZEBO_ROS_LINK_ATTACHER_HH

#include <ros/ros.h>

#include <boost/thread/mutex.hpp>
#include "gazebo/gazebo.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/physics/PhysicsTypes.hh"
#include "gazebo/transport/transport.hh"
#include "gazebo_ros_link_attacher/Attach.h"
#include "gazebo_ros_link_attacher/AttachRequest.h"
#include "gazebo_ros_link_attacher/AttachResponse.h"
#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <sdf/sdf.hh>

namespace gazebo {

class GazeboRosLinkAttacher : public WorldPlugin {
  public:
    /// \brief Constructor
    GazeboRosLinkAttacher();

    /// \brief Destructor
    virtual ~GazeboRosLinkAttacher();

    /// \brief Load the controller
    void Load(physics::WorldPtr _world, sdf::ElementPtr sdf);
    void onUpdate(const common::UpdateInfo &_info);
    /// \brief Attach with a revolute joint
    bool attach();

    /// \brief Detach
    bool detach();

    /// \brief Detach a existing joint
    bool detach_existing();

    /// \brief Internal representation of a fixed joint
    struct Prismatic {
        std::string model1;
        physics::ModelPtr m1;
        std::string link1;
        physics::LinkPtr l1;
        std::string model2;
        physics::ModelPtr m2;
        std::string link2;
        physics::LinkPtr l2;
        physics::JointPtr joint;
    };

    std::string model1_;
    std::string link1_;
    std::string model2_;
    std::string link2_;

    physics::ModelPtr me_;
    std::string model_e_;
    std::string link_e_;
    physics::JointPtr joint_e;
  event::ConnectionPtr updateConnection_;
    bool getJoint(std::string model1, std::string link1, std::string model2, std::string link2, Prismatic& joint);

  private:
    ros::NodeHandle nh_;
    ros::ServiceServer attach_service_;
    ros::ServiceServer detach_service_;
    ros::ServiceServer detach_existing_service_;

    bool attach_callback(gazebo_ros_link_attacher::Attach::Request& req, gazebo_ros_link_attacher::Attach::Response& res);
    bool detach_callback(gazebo_ros_link_attacher::Attach::Request& req, gazebo_ros_link_attacher::Attach::Response& res);
    bool detach_existing_callback(gazebo_ros_link_attacher::Attach::Request& req, gazebo_ros_link_attacher::Attach::Response& res);
    
    std::vector<Prismatic> joints;

    /// \brief The physics engine.
    physics::PhysicsEnginePtr physics;

    /// \brief Pointer to the world.
    physics::WorldPtr world;

    boost::mutex lock_;
};

}  // namespace gazebo

#endif
