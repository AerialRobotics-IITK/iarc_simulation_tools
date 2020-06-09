#include <chrono>
#include <eigen3/Eigen/Eigen>
#include <functional>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/rendering/Visual.hh>
#include <thread>

namespace gazebo {

class GZ_PLUGIN_VISIBLE ModelLight : public VisualPlugin {
    public:
        void Load(rendering::VisualPtr parent, sdf::ElementPtr sdf) override;
        void OnUpdate();

    private:
        void commPoseCallback(ConstPoseStampedPtr& msg);
        void mastPoseCallback(ConstPoseStampedPtr& msg);

        rendering::VisualPtr model_visual_;
        event::ConnectionPtr updateConnection;

        transport::NodePtr node_;
        transport::SubscriberPtr mast_pose_sub_;
        transport::SubscriberPtr comm_pose_sub_;

        std::string led_color_;

        Eigen::Vector3d mast_position_;
        Eigen::Vector3d comm_block_position_;
        Eigen::Vector3d comm_wrt_mast_position_;

        Eigen::Quaterniond mast_orientation_;
        Eigen::Quaterniond comm_block_orientation_;
        Eigen::Quaterniond comm_wrt_mast_orientation_;
};

} // namespace gazebo
