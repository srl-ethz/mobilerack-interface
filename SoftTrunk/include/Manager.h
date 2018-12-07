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

/**
 * @brief The topmost class for the SoftTrunk robot system. Has instances of AugmentedRigidArm, ControllerPCC, and SoftArm and orchestrates them to control the robot.
 */
class Manager{
public:
    Manager(bool logMode=false);
    /**
     * @brief do dynamic curvature control on the robot.
     */
    void curvatureControl(Vector2Nd, Vector2Nd, Vector2Nd);
    /**
     * @brief run experiments to characterize the parameters alpha, k, and d of the soft arm.
     */
    void characterize();
    SoftArm* softArm;
    ControllerPCC* controllerPCC;
    AugmentedRigidArm* augmentedRigidArm;
    ~Manager();
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
