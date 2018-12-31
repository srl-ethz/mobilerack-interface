//
// Created by yasu on 31/10/18.
//

#include "Manager.h"
#include <Eigen/Dense>

/**
 * @file experiment.cpp
 * @brief Topmost code to run experiments on the Soft Trunk system.
 */



/**
 * @brief function that updates the q depending on the time. To be passed into Manager.
 * @param seconds how many seconds passed since beginning
 * @param q this will be updated
 */
void updateQ(double seconds, Vector2Nd * q){
    int experiment_type = 1;
    double maxTheta = 0.01;
    double T = 5.0;
    for (int j = 0; j < 2 * NUM_ELEMENTS; ++j) {
        (*q)(j) = 0;
    }
    if (experiment_type==1){
        // draws circle
        (*q)(0) = maxTheta * cos(seconds*2.0*PI/T);
        (*q)(2) = maxTheta * cos(seconds*2.0*PI/T + PI / 3);
//        (*q)(4) = maxTheta * cos(seconds*2.0*PI/T + PI * 2 / 3);
        (*q)(1) = maxTheta * sin(seconds*2.0*PI/T);
        (*q)(3) = maxTheta * sin(seconds*2.0*PI/T + PI / 3);
//        (*q)(5) = maxTheta * sin(seconds*2.0*PI/T + PI * 2 / 3);
        //todo: ensure smooth transition from 0
    }
    else if(experiment_type==2){
        (*q)(1) = maxTheta * cos(seconds*2.0*PI/T);
        (*q)(2) = maxTheta * cos(seconds*2.0*PI/T);
//        (*q)(5) = maxTheta * cos(seconds*2.0*PI/T);
    }
    else if(experiment_type==3){
        (*q)(0) = 0.5*maxTheta * cos(seconds*2.0*PI/T);
        (*q)(2) = 0.5*maxTheta * cos(seconds*2.0*PI/T + PI / 3);
//        (*q)(4) = 0.5*maxTheta * cos(seconds*2.0*PI/T + PI * 2 / 3);
        (*q)(1) = 0.5*maxTheta * (sin(seconds*2.0*PI/T)+1.0);
        (*q)(3) = 0.5*maxTheta * (sin(seconds*2.0*PI/T + PI / 3)+1.0);
//        (*q)(5) = 0.5*maxTheta * (sin(seconds*2.0*PI/T + PI * 2 / 3)+1.0);
    }
}

int main() {
    Manager manager{true}; // initialize Manager object
    manager.sendJointSpaceProfile((vFunctionCall)updateQ, 3); // move the arm according to the updateQ function
}