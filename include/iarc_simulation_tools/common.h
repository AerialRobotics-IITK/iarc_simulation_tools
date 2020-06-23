#pragma once

#include <eigen3/Eigen/Dense>
#include <gazebo/gazebo.hh>
#include <tinyxml.h>

namespace gazebo {
static const bool kPrintOnPluginLoad    = false;
static const bool kPrintOnUpdates       = false;
static const bool kPrintOnMsgCallback   = false;

static const std::string kDefaultNamespace = "";
static constexpr double kDefaultRotorVelocitySlowdownSim = 10.0;
static const std::string kConnectGazeboToRosSubtopic = "connect_gazebo_to_ros_subtopic";
static const std::string kConnectRosToGazeboSubtopic = "connect_ros_to_gazebo_subtopic";

static const std::string kBroadcastTransformSubtopic = "broadcast_transform";

template<class T>
bool getSdfParam(sdf::ElementPtr sdf, const std::string& name, T& param, const T& default_value, const bool& verbose =
                     false) {
  if (sdf->HasElement(name)) {
    param = sdf->GetElement(name)->Get<T>();
    return true;
  }
  else {
    param = default_value;
    if (verbose)
      gzerr << "[rotors_gazebo_plugins] Please specify a value for parameter \"" << name << "\".\n";
  }
  return false;
}

template <typename T>
void model_param(const std::string& world_name, const std::string& model_name, const std::string& param, T& param_value)
{
  TiXmlElement* e_param = nullptr;
  TiXmlElement* e_param_tmp = nullptr;
  std::string dbg_param;

  TiXmlDocument doc(world_name + ".xml");
  if (doc.LoadFile())
  {
    TiXmlHandle h_root(doc.RootElement());

    TiXmlElement* e_model = h_root.FirstChild("model").Element();

    for( e_model; e_model; e_model=e_model->NextSiblingElement("model") )
    {
      const char* attr_name = e_model->Attribute("name");
      if (attr_name)
      {
        //specific
        if (model_name.compare(attr_name) == 0)
        {
          e_param_tmp = e_model->FirstChildElement(param);
          if (e_param_tmp)
          {
            e_param = e_param_tmp;
            dbg_param = "";
          }
          break;
        }
      }
      else
      {
        //common
        e_param = e_model->FirstChildElement(param);
        dbg_param = "common ";
      }
    }

    if (e_param)
    {
      std::istringstream iss(e_param->GetText());
      iss >> param_value;

      gzdbg << model_name << " model: " << dbg_param << "parameter " << param << " = " << param_value << " from " << doc.Value() << "\n";
    }
  }
}
}

template <typename T>
class FirstOrderFilter {

 public:
  FirstOrderFilter(double timeConstantUp, double timeConstantDown, T initialState):
      timeConstantUp_(timeConstantUp),
      timeConstantDown_(timeConstantDown),
      previousState_(initialState) {}

  T updateFilter(T inputState, double samplingTime) {

    T outputState;
    if (inputState > previousState_) {
      double alphaUp = exp(-samplingTime / timeConstantUp_);
      outputState = alphaUp * previousState_ + (1 - alphaUp) * inputState;

    }
    else {
      double alphaDown = exp(-samplingTime / timeConstantDown_);
      outputState = alphaDown * previousState_ + (1 - alphaDown) * inputState;
    }
    previousState_ = outputState;
    return outputState;

  }

  ~FirstOrderFilter() {}

 protected:
  double timeConstantUp_;
  double timeConstantDown_;
  T previousState_;
};

template<class Derived>
Eigen::Quaternion<typename Derived::Scalar> QuaternionFromSmallAngle(const Eigen::MatrixBase<Derived> & theta) {
  typedef typename Derived::Scalar Scalar;
  EIGEN_STATIC_ASSERT_FIXED_SIZE(Derived);
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 3);
  const Scalar q_squared = theta.squaredNorm() / 4.0;

  if (q_squared < 1) {
    return Eigen::Quaternion<Scalar>(sqrt(1 - q_squared), theta[0] * 0.5, theta[1] * 0.5, theta[2] * 0.5);
  }
  else {
    const Scalar w = 1.0 / sqrt(1 + q_squared);
    const Scalar f = w * 0.5;
    return Eigen::Quaternion<Scalar>(w, theta[0] * f, theta[1] * f, theta[2] * f);
  }
}

template<class In, class Out>
void copyPosition(const In& in, Out* out) {
  out->x = in.x;
  out->y = in.y;
  out->z = in.z;
}
