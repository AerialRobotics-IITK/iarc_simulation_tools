#pragma once

#include <bits/stdc++.h>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <iarc_sim_test_tools/common.h>
#include <ros/ros.h>
#include <string>
#include <thread>
#include <unordered_map>

namespace gazebo {

static const std::string kDefualFrameId = "world";
static const std::string kDefaultLinkName = "base_link";

class CustomWindPlugin : public ModelPlugin {
  public:
    CustomWindPlugin()
        : ModelPlugin()
        , namespace_(kDefaultNamespace)
        , link_name_(kDefaultLinkName) {
    }

    typedef struct WindParams {
        double angle;
        double speed;

        WindParams(const double& angle, const double& speed)
            : angle(angle)
            , speed(speed){};

        // comparison operator to check hash collisions
        bool operator==(const WindParams& wind_params) const {
            return (angle == wind_params.angle) && (speed == wind_params.speed);
        }
    } WindParams;

    typedef struct DynParams {
        ignition::math::Vector3d force;
        ignition::math::Vector3d torque;

        DynParams(const ignition::math::Vector3d& force, const ignition::math::Vector3d& torque)
            : force(force)
            , torque(torque){};

        DynParams() {
            force = ignition::math::Vector3d::Zero;
            torque = ignition::math::Vector3d::Zero;
        };
    } DynParams;

    virtual ~CustomWindPlugin(){};

  protected:
    void Load(physics::ModelPtr model, sdf::ElementPtr sdf);
    void onUpdate(const common::UpdateInfo& _info);
    ignition::math::Vector3d interpolateWindDynamics(WindParams params);

  private:
    // hashing function for wind parameters
    typedef struct WindHasher {
        std::size_t operator()(const WindParams& params) const {
            // Compute individual hash values for angle & speed then combine
            // http://stackoverflow.com/a/1646913/126995
            std::size_t hash = 1009;
            hash = hash * 9176 + std::hash<double>()(params.angle);
            hash = hash * 9176 + std::hash<double>()(params.speed);
            return hash;
        }
    } WindHasher;

    void initializeFieldTable();

    event::ConnectionPtr update_connection_;

    physics::ModelPtr model_;
    sdf::ElementPtr sdf_;
    physics::LinkPtr link_;

    std::string namespace_;
    std::string link_name_;

    ignition::math::Vector3d xyz_offset_;
    ignition::math::Vector3d force_direction_;

    double windspeed_;
    double wind_angle_;

    std::unordered_map<WindParams, DynParams, WindHasher> field_table_;
    // std::unordered_map<WindParams, DynParams, WindHasher>::iterator low;
};
}  // namespace gazebo
