#pragma once
#include "WindSpeed.pb.h"     // Wind speed message
#include "WrenchStamped.pb.h" // Wind force message
#include "iarc_simulation_tools/common.h"
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <mav_msgs/default_topics.h>
#include <string>

namespace gazebo {

static const std::string kDefaultFrameId = "world";
static const std::string kDefaultLinkName = "base_link";
static const std::string kDefaultWindSpeedPubTopic = "wind_speed";

static constexpr double kDefaultWindForceMean = 0.0;
static constexpr double kDefaultWindForceVariance = 0.0;
static constexpr double kDefaultWindGustForceMean = 0.0;
static constexpr double kDefaultWindGustForceVariance = 0.0;

static constexpr double kDefaultWindGustStart = 10.0;
static constexpr double kDefaultWindGustDuration = 0.0;

static constexpr double kDefaultWindSpeedMean = 0.0;
static constexpr double kDefaultWindSpeedVariance = 0.0;

static const ignition::math::Vector3d kDefaultWindDirection =
    ignition::math::Vector3d(1, 0, 0);
static const ignition::math::Vector3d kDefaultWindGustDirection =
    ignition::math::Vector3d(0, 1, 0);

static constexpr bool kDefaultUseCustomStaticWindField = false;

class GazeboWindPlugin : public ModelPlugin {
public:
  GazeboWindPlugin()
      : ModelPlugin(), namespace_(kDefaultNamespace),
        wind_force_pub_topic_(mav_msgs::default_topics::EXTERNAL_FORCE),
        wind_speed_pub_topic_(mav_msgs::default_topics::WIND_SPEED),
        wind_force_mean_(kDefaultWindForceMean),
        wind_force_variance_(kDefaultWindForceVariance),
        wind_gust_force_mean_(kDefaultWindGustForceMean),
        wind_gust_force_variance_(kDefaultWindGustForceVariance),
        wind_speed_mean_(kDefaultWindSpeedMean),
        wind_speed_variance_(kDefaultWindSpeedVariance),
        wind_direction_(kDefaultWindDirection),
        wind_gust_direction_(kDefaultWindGustDirection),
        use_custom_static_wind_field_(kDefaultUseCustomStaticWindField),
        frame_id_(kDefaultFrameId), link_name_(kDefaultLinkName),
        node_handle_(nullptr), pubs_and_subs_created_(false) {}

  virtual ~GazeboWindPlugin();

protected:
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  void OnUpdate(const common::UpdateInfo & /*_info*/);

private:
  bool pubs_and_subs_created_;
  void CreatePubsAndSubs();
  event::ConnectionPtr update_connection_;
  physics::WorldPtr world_;
  physics::ModelPtr model_;
  physics::LinkPtr link_;
  std::string namespace_;
  std::string frame_id_;
  std::string link_name_;
  std::string wind_force_pub_topic_;
  std::string wind_speed_pub_topic_;

  double wind_force_mean_;
  double wind_force_variance_;
  double wind_gust_force_mean_;
  double wind_gust_force_variance_;
  double wind_speed_mean_;
  double wind_speed_variance_;

  ignition::math::Vector3d xyz_offset_;
  ignition::math::Vector3d wind_direction_;
  ignition::math::Vector3d wind_gust_direction_;

  common::Time wind_gust_end_;
  common::Time wind_gust_start_;

  bool use_custom_static_wind_field_;
  float min_x_;
  float min_y_;
  int n_x_;
  int n_y_;
  float res_x_;
  float res_y_;
  std::vector<float> vertical_spacing_factors_;
  std::vector<float> bottom_z_;
  std::vector<float> top_z_;
  std::vector<float> u_;
  std::vector<float> v_;
  std::vector<float> w_;

  void ReadCustomWindField(std::string &custom_wind_field_path);
  ignition::math::Vector3d LinearInterpolation(double position,
                                               ignition::math::Vector3d *values,
                                               double *points) const;
  ignition::math::Vector3d
  BilinearInterpolation(double *position, ignition::math::Vector3d *values,
                        double *points) const;
  ignition::math::Vector3d
  TrilinearInterpolation(ignition::math::Vector3d link_position,
                         ignition::math::Vector3d *values,
                         double *points) const;

  gazebo::transport::PublisherPtr wind_force_pub_;
  gazebo::transport::PublisherPtr wind_speed_pub_;

  gazebo::transport::NodePtr node_handle_;
  gz_geometry_msgs::WrenchStamped wrench_stamped_msg_;
  gz_mav_msgs::WindSpeed wind_speed_msg_;
};
} // namespace gazebo
