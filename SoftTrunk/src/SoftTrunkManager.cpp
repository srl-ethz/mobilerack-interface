//
// Created by yasu on 26/10/18.
//

#include "SoftTrunkManager.h"

SoftTrunkManager::SoftTrunkManager() {
    // set up CurvatureCalculator, AugmentedRigidArm, and ControllerPCC objects.
    //todo: where should the force controller be implemented??
}

void SoftTrunkManager::curvatureControl(Eigen::Matrix<double, NUM_ELEMENTS * 2, 1> q,
                                        Eigen::Matrix<double, NUM_ELEMENTS * 2, 1> dq) {
    // get current measured state from CurvatureCalculator
    // send that to ControllerPCC
    // actuate the arm with the tau value.
}