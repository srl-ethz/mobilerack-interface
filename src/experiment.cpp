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
    double maxValue = 0.01;
    double T = 2.0;
    q->setZero();
    if (experiment_type==1){
        // draws circle
        (*q)(0) = maxValue * cos(seconds*2.0*PI/T);
        (*q)(2) = (*q)(0);
        if (seconds < T/2){
            (*q)(0)=0.5*(*q)(0)-0.5*maxValue; (*q)(2) =(*q)(0);
        }
//        (*q)(4) = maxValue * cos(seconds*2.0*PI/T + PI * 2 / 3);
        (*q)(1) = maxValue * sin(seconds*2.0*PI/T);
        (*q)(3) = maxValue * sin(seconds*2.0*PI/T);
//        (*q)(5) = maxValue * sin(seconds*2.0*PI/T + PI * 2 / 3);

    }
    else if(experiment_type==2){
        (*q)(0) = maxValue * cos(seconds*2.0*PI/T);
        (*q)(2) = maxValue * cos(seconds*2.0*PI/T);
//        (*q)(5) = maxValue * cos(seconds*2.0*PI/T);
    }
    else if(experiment_type==3){
        (*q)(0) = 0.5*maxValue * cos(seconds*2.0*PI/T);
        (*q)(2) = 0.5*maxValue * cos(seconds*2.0*PI/T + PI / 3);
//        (*q)(4) = 0.5*maxValue * cos(seconds*2.0*PI/T + PI * 2 / 3);
        (*q)(1) = 0.5*maxValue * (sin(seconds*2.0*PI/T)+1.0);
        (*q)(3) = 0.5*maxValue * (sin(seconds*2.0*PI/T + PI / 3)+1.0);
//        (*q)(5) = 0.5*maxValue * (sin(seconds*2.0*PI/T + PI * 2 / 3)+1.0);
    }
}

int main() {
    bool log=true;
    bool use_pid = false;
    bool use_feedforward = true;
    Manager manager{log, use_pid, use_feedforward}; // initialize Manager object
    manager.sendJointSpaceProfile((vFunctionCall)updateQ, 10); // move the arm according to the updateQ function
}