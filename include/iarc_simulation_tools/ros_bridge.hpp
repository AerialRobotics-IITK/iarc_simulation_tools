#pragma once

#include <ros/ros.h>
#include<string.h>
#include <cstdlib>
#include <iarc_simulation_tools/seastate.hpp>
#include <std_msgs/Int32.h>

namespace iarc_simulation_tools {

class RosBridge {
    public:
    RosBridge() {
    }
    ~RosBridge() {
    }
    void get_sea_state(const std_msgs::Int32& msg) {
        if(current_sea_state_.data != msg.data) {

            current_sea_state_.data = msg.data;
            
            std::system("roscd iarc_simulation_tools");
            std::system("cd ../..");
            std::string command;
            auto v_param = states_.v_param_;
            command = commands_[0] + v_param[current_sea_state_.data].number + commands_[1] + v_param[current_sea_state_.data].steepness + commands_[2] + v_param[current_sea_state_.data].scale 
                      + commands_[3] + v_param[current_sea_state_.data].angle  + commands_[4] + v_param[current_sea_state_.data].direction + commands_[5] 
                      + std::to_string(v_param[current_sea_state_.data].period) + commands_[6] + std::to_string(v_param[current_sea_state_.data].amplitude);
            const char * cmd = command.c_str();
            std::system(cmd);
        }
    }

    void init(ros::NodeHandle& nh) {
        sea_state_sub_ = nh.subscribe("/sea_state", 10, &RosBridge::get_sea_state, this );
        current_sea_state_.data = 12;
    }

    private:
    std_msgs::Int32 current_sea_state_ ;
    std::string commands_[7] = {"./devel/lib/asv_wave_sim_gazebo_plugins/WaveMsgPublisher \ --number " ,
                                " \ --steepness ",
                                " \ --scale ",
                                " \ --angle ",
                                " \ --direction ",
                                " \ --period ",
                                " \ --amplitude "
                               };

    class SeaState states_;
    ros::Subscriber sea_state_sub_;
};

} //namespace iarc_simulation_tools
