#include<ros/ros.h>
#include<gazebo_msgs/LinkStates.h>
#include<geometry_msgs/PoseStamped.h>
#include<math.h>
#include<tf/tf.h>

using namespace std;
gazebo_msgs::LinkStates state;
geometry_msgs::PoseStamped rpy;

void clbk_fn(const gazebo_msgs::LinkStates& msg) {
    state = msg;
    int idx = 0;

        while(msg.name[idx] != "box::base_link") {
            idx++;
            if(idx ==10) {
                cout << "NOT FOUND";
                break;
            }
        }

        cout << msg.name[idx] << endl;

        double x  = msg.pose[idx].orientation.x;
        double y =  msg.pose[idx].orientation.y;
        double z =  msg.pose[idx].orientation.z;
        double w =  msg.pose[idx].orientation.w;

        tf::Quaternion q(x,y,z,w);
        tf::Matrix3x3 m(q);

        double roll ,pitch, yaw;

        m.getRPY(roll, pitch , yaw);

        cout << roll << pitch << yaw << endl;

        rpy.header.stamp.sec =  ros::Time::now().toSec();

        rpy.pose.position.x = roll;
        rpy.pose.position.y = pitch;
        rpy.pose.position.z = yaw;

        // cout<< msg << endl;
}

int main(int argc , char** argv) {
    ros::init(argc, argv , "check");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/gazebo/link_states" , 10 , clbk_fn);
    ros::Publisher pub = nh.advertise<geometry_msgs::PoseStamped>("/rpy_values" ,10);
    ros::Rate rate(10);

    while (ros::ok()) {
        ros::spinOnce();       
        pub.publish(rpy);
        rate.sleep();
    }
    return 0;
}