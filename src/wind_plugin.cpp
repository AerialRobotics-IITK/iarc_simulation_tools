#include "iarc_simulation_tools/wind_plugin.hh"
#include <fstream>
#include <math.h>
#include "ConnectGazeboToRosTopic.pb.h"

namespace gazebo {

GazeboWindPlugin::~GazeboWindPlugin() {}

void GazeboWindPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {

  if (kPrintOnPluginLoad) {
    gzdbg << __FUNCTION__ << "() called." << std::endl;
  }

  model_ = _model;
  world_ = model_->GetWorld();

  double wind_gust_start = kDefaultWindGustStart;
  double wind_gust_duration = kDefaultWindGustDuration;

  if (_sdf->HasElement("robotNamespace"))
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  else
    gzerr << "[gazebo_wind_plugin] Please specify a robotNamespace.\n";

  node_handle_ = gazebo::transport::NodePtr(new transport::Node());
  node_handle_->Init();

  if (_sdf->HasElement("xyzOffset"))
    xyz_offset_ = _sdf->GetElement("xyzOffset")->Get<ignition::math::Vector3d >();
  else
    gzerr << "[gazebo_wind_plugin] Please specify a xyzOffset.\n";

  getSdfParam<std::string>(_sdf, "windForcePubTopic", wind_force_pub_topic_,wind_force_pub_topic_);
  getSdfParam<std::string>(_sdf, "windSpeedPubTopic", wind_speed_pub_topic_,wind_speed_pub_topic_);
  getSdfParam<std::string>(_sdf, "frameId", frame_id_, frame_id_);
  getSdfParam<std::string>(_sdf, "linkName", link_name_, link_name_);

  getSdfParam<double>(_sdf, "windSpeedMean", wind_speed_mean_,wind_speed_mean_);
  getSdfParam<double>(_sdf, "windSpeedVariance", wind_speed_variance_,wind_speed_variance_);
  getSdfParam<ignition::math::Vector3d >(_sdf, "windDirection", wind_direction_,wind_direction_);
  getSdfParam<bool>(_sdf, "useCustomStaticWindField", use_custom_static_wind_field_,use_custom_static_wind_field_);

  if (!use_custom_static_wind_field_) {
    gzdbg << "[gazebo_wind_plugin] Using user-defined constant wind field and gusts.\n";

    getSdfParam<double>(_sdf, "windForceMean", wind_force_mean_,wind_force_mean_);
    getSdfParam<double>(_sdf, "windForceVariance", wind_force_variance_,wind_force_variance_);
    getSdfParam<double>(_sdf, "windGustStart", wind_gust_start, wind_gust_start);
    getSdfParam<double>(_sdf, "windGustDuration", wind_gust_duration,wind_gust_duration);
    getSdfParam<double>(_sdf, "windGustForceMean", wind_gust_force_mean_,wind_gust_force_mean_);
    getSdfParam<double>(_sdf, "windGustForceVariance", wind_gust_force_variance_,wind_gust_force_variance_);
    getSdfParam<ignition::math::Vector3d >(_sdf, "windGustDirection", wind_gust_direction_,wind_gust_direction_);

    wind_direction_.Normalize();
    wind_gust_direction_.Normalize();
    wind_gust_start_ = common::Time(wind_gust_start);
    wind_gust_end_ = common::Time(wind_gust_start + wind_gust_duration);
  } else {
    gzdbg << "[gazebo_wind_plugin] Using custom wind field from text file.\n";
    std::string custom_wind_field_path;
    getSdfParam<std::string>(_sdf, "customWindFieldPath", custom_wind_field_path,custom_wind_field_path);
    ReadCustomWindField(custom_wind_field_path);
  }

  link_ = model_->GetLink(link_name_);
  if (link_ == NULL)
    gzthrow("[gazebo_wind_plugin] Couldn't find specified link \"" << link_name_ << "\".");

  update_connection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&GazeboWindPlugin::OnUpdate, this, _1));
}

void GazeboWindPlugin::OnUpdate(const common::UpdateInfo& _info) {

  if (kPrintOnUpdates) {
    gzdbg << __FUNCTION__ << "() called." << std::endl;
  }

  if (!pubs_and_subs_created_) {
    CreatePubsAndSubs();
    pubs_and_subs_created_ = true;
  }

  common::Time now = world_->SimTime();
  ignition::math::Vector3d wind_velocity(5.0, 0.0, 0.0);

  if (!use_custom_static_wind_field_) {

    double wind_strength = wind_force_mean_;
    ignition::math::Vector3d wind = wind_strength * wind_direction_;

    link_->AddForceAtRelativePosition(wind, xyz_offset_);

    ignition::math::Vector3d wind_gust(0.0, 0.0, 0.0);
    if (now >= wind_gust_start_ && now < wind_gust_end_) {
      double wind_gust_strength = wind_gust_force_mean_;
      wind_gust = wind_gust_strength * wind_gust_direction_;

      link_->AddForceAtRelativePosition(wind_gust, xyz_offset_);
    }

    wrench_stamped_msg_.mutable_header()->set_frame_id(frame_id_);
    wrench_stamped_msg_.mutable_header()->mutable_stamp()->set_sec(now.sec);
    wrench_stamped_msg_.mutable_header()->mutable_stamp()->set_nsec(now.nsec);

    wrench_stamped_msg_.mutable_wrench()->mutable_force()->set_x(wind.X() + wind_gust.X());
    wrench_stamped_msg_.mutable_wrench()->mutable_force()->set_y(wind.Y() + wind_gust.Y());
    wrench_stamped_msg_.mutable_wrench()->mutable_force()->set_z(wind.Z() + wind_gust.Z());

    wrench_stamped_msg_.mutable_wrench()->mutable_torque()->set_x(3);
    wrench_stamped_msg_.mutable_wrench()->mutable_torque()->set_y(3);
    wrench_stamped_msg_.mutable_wrench()->mutable_torque()->set_z(0);

    wind_force_pub_->Publish(wrench_stamped_msg_);

    wind_velocity = wind_speed_mean_ * wind_direction_;
  } 
  else {
    ignition::math::Vector3d link_position = link_->WorldPose().Pos();

    std::size_t x_inf = floor((link_position.X() - min_x_) / res_x_);
    std::size_t y_inf = floor((link_position.Y() - min_y_) / res_y_);

    if (x_inf == n_x_ - 1u) {
      x_inf = n_x_ - 2u;
    }
    if (y_inf == n_y_ - 1u) {
      y_inf = n_y_ - 2u;
    }

    std::size_t x_sup = x_inf + 1u;
    std::size_t y_sup = y_inf + 1u;

    constexpr unsigned int n_vertices = 8;
    std::size_t idx_x[n_vertices] = {x_inf, x_inf, x_sup, x_sup, x_inf, x_inf, x_sup, x_sup};
    std::size_t idx_y[n_vertices] = {y_inf, y_inf, y_inf, y_inf, y_sup, y_sup, y_sup, y_sup};

    constexpr unsigned int n_columns = 4;
    float vertical_factors_columns[n_columns];
    for (std::size_t i = 0u; i < n_columns; ++i) {
      vertical_factors_columns[i] = (
        link_position.Z() - bottom_z_[idx_x[2u * i] + idx_y[2u * i] * n_x_]) /
        (top_z_[idx_x[2u * i] + idx_y[2u * i] * n_x_] - bottom_z_[idx_x[2u * i] + idx_y[2u * i] * n_x_]);
    }
    
    float vertical_factors_min = std::min(std::min(std::min(
      vertical_factors_columns[0], vertical_factors_columns[1]),
      vertical_factors_columns[2]), vertical_factors_columns[3]);
    float vertical_factors_max = std::max(std::max(std::max(
      vertical_factors_columns[0], vertical_factors_columns[1]),
      vertical_factors_columns[2]), vertical_factors_columns[3]);

    if (x_inf >= 0u && y_inf >= 0u && vertical_factors_max >= 0u && 
        x_sup <= (n_x_ - 1u) && y_sup <= (n_y_ - 1u) && vertical_factors_min <= 1u) {

      std::size_t idx_z[n_vertices] = {0u, static_cast<int>(vertical_spacing_factors_.size()) - 1u,
                              0u, static_cast<int>(vertical_spacing_factors_.size()) - 1u,
                              0u, static_cast<int>(vertical_spacing_factors_.size()) - 1u,
                              0u, static_cast<int>(vertical_spacing_factors_.size()) - 1u};
      for (std::size_t i = 0u; i < n_columns; ++i) {
        if (vertical_factors_columns[i] < 0u) {
          idx_z[2u * i + 1u] = 1u;
        } 
        else if (vertical_factors_columns[i] >= 1u) {
          idx_z[2u * i] = vertical_spacing_factors_.size() - 2u;
        } 
        else {
          for (std::size_t j = 0u; j < vertical_spacing_factors_.size() - 1u; ++j) {
            if (vertical_spacing_factors_[j] <= vertical_factors_columns[i] && 
                vertical_spacing_factors_[j + 1u] > vertical_factors_columns[i]) {
              idx_z[2u * i] = j;
              idx_z[2u * i + 1u] = j + 1u;
              break;
            }
          }
        }
      }

      ignition::math::Vector3d wind_at_vertices[n_vertices];
      for (std::size_t i = 0u; i < n_vertices; ++i) {
        wind_at_vertices[i].X() = u_[idx_x[i] + idx_y[i] * n_x_ + idx_z[i] * n_x_ * n_y_];
        wind_at_vertices[i].Y() = v_[idx_x[i] + idx_y[i] * n_x_ + idx_z[i] * n_x_ * n_y_];
        wind_at_vertices[i].Z() = w_[idx_x[i] + idx_y[i] * n_x_ + idx_z[i] * n_x_ * n_y_];
      }

      constexpr unsigned int n_points_interp_z = 8;
      constexpr unsigned int n_points_interp_x = 4;
      constexpr unsigned int n_points_interp_y = 2;
      double interpolation_points[n_points_interp_x + n_points_interp_y + n_points_interp_z];
      for (std::size_t i = 0u; i < n_points_interp_x + n_points_interp_y + n_points_interp_z; ++i) {
        if (i < n_points_interp_z) {
          interpolation_points[i] = (
            top_z_[idx_x[i] + idx_y[i] * n_x_] - bottom_z_[idx_x[i] + idx_y[i] * n_x_])
            * vertical_spacing_factors_[idx_z[i]] + bottom_z_[idx_x[i] + idx_y[i] * n_x_];
        }
        else if (i >= n_points_interp_z && i < n_points_interp_x + n_points_interp_z) {
          interpolation_points[i] = min_x_ + res_x_ * idx_x[2u * (i - n_points_interp_z)];
        } 
        else {
          interpolation_points[i] = min_y_ + res_y_ * idx_y[4u * (i - n_points_interp_z - n_points_interp_x)];
        }
      }
      wind_velocity = TrilinearInterpolation(link_position, wind_at_vertices, interpolation_points);
    } 
    else {
      wind_velocity = wind_speed_mean_ * wind_direction_;
    }
  }
  
  wind_speed_msg_.mutable_header()->set_frame_id(frame_id_);
  wind_speed_msg_.mutable_header()->mutable_stamp()->set_sec(now.sec);
  wind_speed_msg_.mutable_header()->mutable_stamp()->set_nsec(now.nsec);

  wind_speed_msg_.mutable_velocity()->set_x(wind_velocity.X());
  wind_speed_msg_.mutable_velocity()->set_y(wind_velocity.Y());
  wind_speed_msg_.mutable_velocity()->set_z(wind_velocity.Z());

  wind_speed_pub_->Publish(wind_speed_msg_);
}

void GazeboWindPlugin::CreatePubsAndSubs() {
  gazebo::transport::PublisherPtr connect_gazebo_to_ros_topic_pub =
      node_handle_->Advertise<gz_std_msgs::ConnectGazeboToRosTopic>(
          "~/" + kConnectGazeboToRosSubtopic, 1);

  gz_std_msgs::ConnectGazeboToRosTopic connect_gazebo_to_ros_topic_msg;

  wind_force_pub_ = node_handle_->Advertise<gz_geometry_msgs::WrenchStamped>(
      "~/" + namespace_ + "/" + wind_force_pub_topic_, 1);

  connect_gazebo_to_ros_topic_msg.set_gazebo_topic("~/" + namespace_ + "/" + wind_force_pub_topic_);
  connect_gazebo_to_ros_topic_msg.set_ros_topic(namespace_ + "/" + wind_force_pub_topic_);
  connect_gazebo_to_ros_topic_msg.set_msgtype(gz_std_msgs::ConnectGazeboToRosTopic::WRENCH_STAMPED);
  connect_gazebo_to_ros_topic_pub->Publish(connect_gazebo_to_ros_topic_msg, true);

  wind_speed_pub_ = node_handle_->Advertise<gz_mav_msgs::WindSpeed>("~/" + namespace_ + "/" + wind_speed_pub_topic_, 1);

  connect_gazebo_to_ros_topic_msg.set_gazebo_topic("~/" + namespace_ + "/" + wind_speed_pub_topic_);
  connect_gazebo_to_ros_topic_msg.set_ros_topic(namespace_ + "/" + wind_speed_pub_topic_);
  connect_gazebo_to_ros_topic_msg.set_msgtype(gz_std_msgs::ConnectGazeboToRosTopic::WIND_SPEED);
  connect_gazebo_to_ros_topic_pub->Publish(connect_gazebo_to_ros_topic_msg, true);
}

void GazeboWindPlugin::ReadCustomWindField(std::string& custom_wind_field_path) {
  std::ifstream fin;
  fin.open(custom_wind_field_path);
  if (fin.is_open()) {
    std::string data_name;
    float data;
    while (fin >> data_name) {
      if (data_name == "min_x:") {
        fin >> min_x_;
      } else if (data_name == "min_y:") {
        fin >> min_y_;
      } else if (data_name == "n_x:") {
        fin >> n_x_;
      } else if (data_name == "n_y:") {
        fin >> n_y_;
      } else if (data_name == "res_x:") {
        fin >> res_x_;
      } else if (data_name == "res_y:") {
        fin >> res_y_;
      } else if (data_name == "vertical_spacing_factors:") {
        while (fin >> data) {
          vertical_spacing_factors_.push_back(data);
          if (fin.peek() == '\n') break;
        }
      } else if (data_name == "bottom_z:") {
        while (fin >> data) {
          bottom_z_.push_back(data);
          if (fin.peek() == '\n') break;
        }
      } else if (data_name == "top_z:") {
        while (fin >> data) {
          top_z_.push_back(data);
          if (fin.peek() == '\n') break;
        }
      } else if (data_name == "u:") {
        while (fin >> data) {
          u_.push_back(data);
          if (fin.peek() == '\n') break;
        }
      } else if (data_name == "v:") {
        while (fin >> data) {
          v_.push_back(data);
          if (fin.peek() == '\n') break;
        }
      } else if (data_name == "w:") {
        while (fin >> data) {
          w_.push_back(data);
          if (fin.peek() == '\n') break;
        }
      } else {
        std::string restOfLine;
        getline(fin, restOfLine);
        gzerr << " [gazebo_wind_plugin] Invalid data name '" << data_name << restOfLine <<
              "' in custom wind field text file. Ignoring data on next line.\n";
        fin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
      }
    }
    fin.close();
    gzdbg << "[gazebo_wind_plugin] Successfully read custom wind field from text file.\n";
  } else {
    gzerr << "[gazebo_wind_plugin] Could not open custom wind field text file.\n";
  }
}

ignition::math::Vector3d GazeboWindPlugin::LinearInterpolation(
  double position, ignition::math::Vector3d * values, double* points) const {
  ignition::math::Vector3d value = values[0] + (values[1] - values[0]) /
                        (points[1] - points[0]) * (position - points[0]);
  return value;
}

ignition::math::Vector3d GazeboWindPlugin::BilinearInterpolation(
  double* position, ignition::math::Vector3d * values, double* points) const {
  ignition::math::Vector3d intermediate_values[2] = { LinearInterpolation(
                                             position[0], &(values[0]), &(points[0])),
                                           LinearInterpolation(
                                             position[0], &(values[2]), &(points[2])) };
  ignition::math::Vector3d value = LinearInterpolation(
                          position[1], intermediate_values, &(points[4]));
  return value;
}

ignition::math::Vector3d GazeboWindPlugin::TrilinearInterpolation(
  ignition::math::Vector3d link_position, ignition::math::Vector3d * values, double* points) const {
  double position[3] = {link_position.X(),link_position.Y(),link_position.Z()};
  ignition::math::Vector3d intermediate_values[4] = { LinearInterpolation(
                                             position[2], &(values[0]), &(points[0])),
                                           LinearInterpolation(
                                             position[2], &(values[2]), &(points[2])),
                                           LinearInterpolation(
                                             position[2], &(values[4]), &(points[4])),
                                           LinearInterpolation(
                                             position[2], &(values[6]), &(points[6])) };
  ignition::math::Vector3d value = BilinearInterpolation(
    &(position[0]), intermediate_values, &(points[8]));
  return value;
}

GZ_REGISTER_MODEL_PLUGIN(GazeboWindPlugin);

}  // namespace gazebo
