//
// Created by yasu on 26/10/18.
//

#ifndef SOFTTRUNK_MANAGER_H
#define SOFTTRUNK_MANAGER_H

#include "CurvatureCalculator.h"
#include "AugmentedRigidArm.h"
#include "ControllerPCC.h"
#include <Eigen/Dense>
#define NUM_ELEMENTS 3

class SoftTrunkManager{
    /*
     * topmost class for the Soft Trunk.
     */
public:
    SoftTrunkManager();
    void curvatureControl(Eigen::Matrix<double, NUM_ELEMENTS*2, 1>, Eigen::Matrix<double, NUM_ELEMENTS*2, 1>); // does single step curvature control.
};

#endif //SOFTTRUNK_MANAGER_H
