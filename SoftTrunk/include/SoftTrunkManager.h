//
// Created by yasu on 26/10/18.
//

#ifndef SOFTTRUNK_MANAGER_H
#define SOFTTRUNK_MANAGER_H

#include "AugmentedRigidArm.h"
#include "ControllerPCC.h"
#include "SoftArm.h"
#include <Eigen/Dense>
#include "SoftTrunk_common_defs.h"
#include <thread>
#include <stdio.h>
#include <chrono>
#include <fstream>

class SoftTrunkManager{
    /*
     * topmost class for the Soft Trunk.
     */
public:
    SoftTrunkManager(bool logMode=false);
    void curvatureControl(Vector2Nd, Vector2Nd, Vector2Nd); // does single step curvature control.
    void characterize();
    SoftArm* softArm;
    ControllerPCC* controllerPCC;
    AugmentedRigidArm* augmentedRigidArm;
    ~SoftTrunkManager();
private:
    // variables and functions used to save the log of q
    bool logMode;
    std::vector<Vector2Nd> log_q_meas;
    std::vector<Vector2Nd> log_q_ref;
    std::vector<std::chrono::high_resolution_clock::duration> log_time;
    std::chrono::high_resolution_clock::time_point logBeginTime;
    int logNum=0;
    void log(Vector2Nd&, Vector2Nd&);
    void outputLog();
};

#endif //SOFTTRUNK_MANAGER_H
