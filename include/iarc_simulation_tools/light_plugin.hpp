#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/rendering/Visual.hh>
#include <gazebo/common/common.hh>
#include <thread>
#include <chrono>
#include <eigen3/Eigen/Eigen>

namespace gazebo {

class GZ_PLUGIN_VISIBLE ModelLight : public VisualPlugin{
    public:
        void Load(rendering::VisualPtr parent, sdf::ElementPtr sdf) override;
        void OnUpdate();
        ~ModelLight();
        transport::NodePtr node_;
        transport::SubscriberPtr mast_pose_sub_;
        transport::SubscriberPtr comm_pose_sub_;

    private:
        void commPoseCallback(ConstPoseStampedPtr &msg);
        void mastPoseCallback(ConstPoseStampedPtr &msg);

        rendering::VisualPtr model_visual_;
        event::ConnectionPtr updateConnection;
        int size_;
        std::string *pose_names_ = NULL;
        ignition::math::Color led_color_;
        Eigen::Vector3d mast_position_;
        Eigen::Vector3d comm_block_position_;
        Eigen::Vector3d comm_wrt_mast_position_; // in mast frame

        Eigen::Quaterniond mast_orientation_;
        Eigen::Quaterniond comm_block_orientation_;
        Eigen::Quaterniond comm_wrt_mast_orientation_;
};
}