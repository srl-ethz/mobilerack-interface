//
// Created by rkk on 29.10.18.
//

#include "AugmentedRigidArm.h"
#include "ControllerPCC.h"
#include "SoftTrunkInterface.h"
#include <stdio.h>

/**
 * @file example_ControllerPCC.cpp
 * @brief demo of ControllerPCC class.
 * creates an AugmentedRigidArm and SoftArm object, and computes the torque required for curvature dynamic control.
 */

int main() {
    AugmentedRigidArm augmentedRigidArm{false};
    SoftTrunkInterface softTrunkInterface{true};
    bool simulate = true;
    ControllerPCC controllerPCC(&augmentedRigidArm, &softTrunkInterface, simulate=simulate);

    Vector2Nd q_ref = Vector2Nd::Zero();
    Vector2Nd dq_ref = Vector2Nd::Zero();
    Vector2Nd ddq_ref = Vector2Nd::Zero();

    Vector2Nd q_meas = Vector2Nd::Zero();
    Vector2Nd dq_meas = Vector2Nd::Zero();

//    q_ref(0) = 0.001;
//    q_ref(2) = 0.001;

    Vector2Nd f;
    controllerPCC.curvatureDynamicControl(q_ref, dq_ref, ddq_ref, &f);
    std::cout<< "\tB\n" << controllerPCC.B <<"\n";
    std::cout<< "\tC\n" << controllerPCC.C <<"\n";
    std::cout<< "\tG\n" << controllerPCC.G <<"\n";
    std::cout<< "\tJ\n" << controllerPCC.J <<"\n";
    std::cout<< "\tf:\n" << f << "\n";
    softTrunkInterface.actuate(f);
    return 1;
}
