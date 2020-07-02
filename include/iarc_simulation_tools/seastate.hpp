#pragma once

#include <iostream>
#include <vector>

namespace iarc_simulation_tools {

class SeaState {

    public :
    SeaState() {
    }
    ~SeaState() {   
    }
    
    struct Vector {
        int x;
        int y;
        int z;
    };

    typedef struct Vector vector;

    struct Params {
        int number = 3;
        double steepness = 0.0;
        double scale = 1.5;
        double angle = 0.4;
        vector direction  = {1, 1, 0};
        double period = 8.0;
        double amplitude = 0.4;
    };

    double random_number (double lower_limit , double upper_limit) {

        double randNum = ((upper_limit - lower_limit) * ((float)rand() / RAND_MAX)) + lower_limit;

        return randNum;
    }

    int total_sea_states_ = 10;
    int current_sea_state_;
    bool state_ = false;

    // amplitude is in metres
    // values are taken from WOCE Upper Ocean Thermal Data
    double amplitude_[10] = { 0 , random_number(0 ,0.1) , random_number(0.1 ,0.5), random_number(0.5 ,1.25) ,random_number(1.25 ,2.5) ,
                            random_number(2.5 ,4) ,random_number(4 ,6) ,random_number(6 ,9), random_number(9 ,14) ,random_number(14 ,rand())
                            };

    // period is in seconds
    // values are taken from WOCE Upper Ocean Thermal Data
    int period_[10] = {10, 11, 12, 13 , rand() % 14 + 14, rand() % 5, 6, 7, 8, 9};

    std::vector<Params> make_sea_state(double* amplitue_, int* period_ , int total_sea_states_) {
        
        std::vector<Params> v_param_;
        for (int i = 0 ; i < total_sea_states_; i++) {
            Params param;
            param.amplitude = amplitude_[i];
            param.period = period_[i];
            v_param_.push_back(param);   
        }

        return v_param_;
    } 

    std::vector<Params> v_param_ = make_sea_state(amplitude_, period_ , total_sea_states_);

};

} //namespace iarc_simulation_tools