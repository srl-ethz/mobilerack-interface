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

// define typedef for readability's sake, so that a function can be passed as parameter to sendJointSpaceProfile().
//https://stackoverflow.com/questions/6339970/c-using-function-as-parameter
typedef void (* vFunctionCall)(double, Vector2Nd*);

/**
 * @brief The topmost class for the SoftTrunk robot system. Has instances of AugmentedRigidArm, ControllerPCC, and SoftArm classes and orchestrates them to control the robot.
 *
 * ## When setting up a new arm configuration
 * * update SoftTrunk_common_defs.h with information of new configuration. This file contains constants that are used throughout the code.
 * * run create_xacro.cpp to generate new robot.urdf.xacro
 * * run create_urdf.sh in urdf/ to generate new robot.urdf
 * * run the characterization sequence, and update values as needed (todo: more documentation about this)
 * * run experiment.cpp
 */
class Manager {
public:
    /**
     * @brief constructor for Manager class.
     * @param logMode if true, outputs log.csv which logs the reference and measured values of the arm.
     * @param use_pid whether to use PID control or not
     */
    explicit Manager(bool logMode = false, bool use_pid=false);

    /**
     * @brief moves the arm according to a profile of q.
     * @param updateQ function that updates q as a function of the current time
     * @param duration how long the motion should last
     */
    void sendJointSpaceProfile(vFunctionCall updateQ, double duration);

    void characterize_part1();
    /**
     * @brief run experiments to characterize the parameters k, and d of the soft arm(alpha should already be measured with characteize_part1(). Then, edit the code manually to change these values.
     */
    void characterize_part2();

    ~Manager();

private:
    /**
     * @brief do one step dynamic curvature control on the robot.
     */
    void curvatureControl(Vector2Nd, Vector2Nd, Vector2Nd);
    SoftArm *softArm;
    ControllerPCC *controllerPCC;
    AugmentedRigidArm *augmentedRigidArm;
    /**
     * @brief variables and functions used to save the log of q
     */
    bool logMode;
    bool use_pid;
    std::vector<Vector2Nd> log_q_meas;
    std::vector<Vector2Nd> log_q_ref;
    std::vector<std::chrono::high_resolution_clock::duration> log_time;
    std::chrono::high_resolution_clock::time_point logBeginTime;
    int logNum = 0;

    void log(Vector2Nd &, Vector2Nd &);

    void outputLog();
};

#endif //SOFTTRUNK_MANAGER_H
