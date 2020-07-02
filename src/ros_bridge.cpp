#include <iarc_simulation_tools/ros_bridge.hpp>

using namespace iarc_simulation_tools;

int main(int argc, char** argv) {
    ros::init(argc, argv, "ros_bridge");
    ros::NodeHandle nh_;

    class RosBridge bridge_;

    ros::Rate rate(10);

    bridge_.init(nh_);

    while(ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
