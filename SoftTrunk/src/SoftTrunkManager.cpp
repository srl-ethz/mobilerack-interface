//
// Created by yasu on 26/10/18.
//

#include "SoftTrunkManager.h"

SoftTrunkManager::SoftTrunkManager() {
    // set up CurvatureCalculator, AugmentedRigidArm, and ControllerPCC objects.
    //todo: where should the force controller be implemented?? -> in SoftArm class
    softArm = new SoftArm();
    softArm->start();
    if (USE_PID_CURVATURE_CONTROL){
        controllerPCC = new ControllerPCC{softArm};
    }
    else {
        augmentedRigidArm = new AugmentedRigidArm(false);
        controllerPCC = new ControllerPCC{augmentedRigidArm, softArm};
    }

}

void SoftTrunkManager::curvatureControl(Vector2Nd q,
                                        Vector2Nd dq,
                                        Vector2Nd ddq) {
    // get current measured state from CurvatureCalculator inside SoftArm, send that to ControllerPCC
    // actuate the arm with the tau value.
    if (USE_PID_CURVATURE_CONTROL){
        Vector2Nd output;
        controllerPCC->curvaturePIDControl(q,&output);
        softArm->actuate(output);
        return;
    }
    Vector2Nd tau;
    controllerPCC->curvatureDynamicControl(q, dq, ddq, &tau);
    softArm->actuate(tau);
}

void SoftTrunkManager::characterize() {
    std::cout << "SoftTrunkManager.characterize called. Computing characteristics of the SoftTrunk...\n";

    // first, specify the pressures to send to arm.
    Eigen::Matrix<double, NUM_ELEMENTS*2, CHARACTERIZE_STEPS*NUM_ELEMENTS*4>  pressures = Eigen::Matrix<double, NUM_ELEMENTS*2, CHARACTERIZE_STEPS*NUM_ELEMENTS*4>::Zero();

    for (int i = 0; i < NUM_ELEMENTS*4; ++i) {
        for (int j = 0; j < CHARACTERIZE_STEPS; ++j) {
            if (i%2 == 0){
                pressures(i/2, i*CHARACTERIZE_STEPS + j) = 300;
            }
            else{
                pressures(i/2, i*CHARACTERIZE_STEPS + j) = -300;
            }
        }
    }

    //std::cout<<"created pressure profile \n"<<pressures <<"\n";

    // send that to arm and log the results.

    for (int l = 0; l < CHARACTERIZE_STEPS*NUM_ELEMENTS*4; ++l) {
        softArm->actuatePressure(pressures.col(l));
        std::this_thread::sleep_for(std::chrono::milliseconds(TIME_STEP));
    }

    softArm->stop();
}
void SoftTrunkManager::stop() {
    softArm->stop();
}