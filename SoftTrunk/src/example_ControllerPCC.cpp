//
// Created by rkk on 29.10.18.
//

#include "AugmentedRigidArm.h"
#include "ControllerPCC.h"
#include "SoftArm.h"
#include <stdio.h>

/**
 * @file example_ControllerPCC.cpp
 * @brief demo of ControllerPCC class.
 * creates an AugmentedRigidArm and SoftArm object, and computes the torque required for curvature dynamic control.
 */

int main() {
    AugmentedRigidArm augmentedRigidArm{false};
    SoftArm softArm{true};
    ControllerPCC controllerPCC(&augmentedRigidArm, &softArm);

    Vector2Nd q_ref = Vector2Nd::Zero();
    Vector2Nd dq_ref = Vector2Nd::Zero();
    Vector2Nd ddq_ref = Vector2Nd::Zero();

    Vector2Nd q_meas = Vector2Nd::Zero();
    Vector2Nd dq_meas = Vector2Nd::Zero();

    q_ref(0) = 0.001;
    q_ref(2) = 0.001;

    Vector2Nd tau_pt;
    controllerPCC.curvatureDynamicControl(q_ref, dq_ref, ddq_ref, &tau_pt, true);
    std::cout << "\ttau_q:\n" << tau_pt << "\n";
    softArm.actuate(tau_pt);
    return 1;
}
