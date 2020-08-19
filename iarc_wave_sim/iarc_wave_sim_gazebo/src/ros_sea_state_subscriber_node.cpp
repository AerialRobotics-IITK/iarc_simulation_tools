#include <iarc_wave_sim_gazebo/ros_sea_state_subscriber.hpp>

using namespace iarc_simulation_tools;

int main(int argc, char** argv) {
    ros::init(argc, argv, "ros_bridge");
    ros::NodeHandle nh_;

    class RosBridge bridge_;

    ros::Rate rate(10);

    bridge_.init(nh_);

    while (ros::ok()) {
        ros::spinOnce();
        bridge_.publish();
        rate.sleep();
    }
    return 0;
}
