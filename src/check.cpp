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
    // cout << msg.name[2];

    int idx = 0;

        while(msg.name[idx] != "box::base_link") {
            idx++;
            // cout <<"Found";
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
        // cout << x <<"  "<<y<<z<<"  "<<w;
        // double x =0,y=0,z=0,w=1;
        // x=0;y=0;z=0;w=1;
        // double roll  = atan2(2*y*w - 2*x*z, 1 - 2*y*y - 2*z*z);
        // double pitch = atan2(2*x*w - 2*y*z, 1 - 2*x*x - 2*z*z);
        // double yaw   = asin(2*x*y + 2*z*w);

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
        // int idx = 0;

        // while(state.name[idx] != "box::base_link") {
        //     idx++;
        //     cout <<"Found";
        //     if(idx ==10) {
        //         cout << "NOT FOUND";
        //         break;
        //     }
        // }

            // cout << state.name[4] << endl;

        // double x  = state.pose[idx].orientation.x;
        // double y =  state.pose[idx].orientation.y;
        // double z =  state.pose[idx].orientation.z;
        // double w =  state.pose[idx].orientation.w;
        // cout << x <<y<<z<<w;
        // double x =0,y=0,z=0,w=1;
        // x=0;y=0;z=0;w=1;
        // double roll  = atan2(2*y*w - 2*x*z, 1 - 2*y*y - 2*z*z);
        // double pitch = atan2(2*x*w - 2*y*z, 1 - 2*x*x - 2*z*z);
        // double yaw   = asin(2*x*y + 2*z*w);

        // tf::Quaternion q(x,y,z,w);
        // tf::Matrix3x3 m(q);

        // double roll ,pitch, yaw;

        // m.getRPY(roll, pitch , yaw);

        // rpy.pose.position.x = roll;
        // rpy.pose.position.y = pitch;
        // rpy.pose.position.z = yaw;

        // // cout<< state << endl;

        pub.publish(rpy);

        rate.sleep();


    }

    return 0;
}