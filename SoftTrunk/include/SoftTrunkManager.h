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

class SoftTrunkManager{
    /*
     * topmost class for the Soft Trunk.
     */
public:
    SoftTrunkManager();
    void curvatureControl(Vector2Nd, Vector2Nd); // does single step curvature control.
    void characterize();
    SoftArm* softArm;
    ControllerPCC* controllerPCC;
    AugmentedRigidArm* augmentedRigidArm;
    Vector2Nd k;
    Vector2Nd d;
};

#endif //SOFTTRUNK_MANAGER_H
