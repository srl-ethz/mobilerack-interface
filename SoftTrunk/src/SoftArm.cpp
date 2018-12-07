//
// Created by yasu on 29/10/18.
//

#include "SoftArm.h"

SoftArm::SoftArm(bool simulate) : simulate(simulate) {

    // set up the impedance parameters (k&d), and actuation coefficient(alpha).
    k = 0;
    d = 0;
    alpha = 0.0001;
//    k = Vector2Nd::Zero();
//    d = Vector2Nd::Zero();
//    alpha = Vector2Nd::Zero();
//    for (int j = 0; j < NUM_ELEMENTS; ++j) {
//        alpha(2*j) = 0.00013;
//        alpha(2*j+1) = 0.00013;
//        k(2*j) = 0;
//        k(2*j+1) = 0;
//        d(2*j) = 0;
//        d(2*j+1) = 0;
//    }

    std::cout << "Starting SoftArm...\n";

    if (simulate)
        return;

    // set up the forceController and curvatureCalculator, which does the messaging with the physical arm
    forceController = new ForceController(16, MAX_PRESSURE);
    curvatureCalculator = new CurvatureCalculator(USE_OPTITRACK);
    curvatureCalculator->setupOptiTrack(LOCAL_ADDRESS, MOTIVE_ADDRESS);
    curvatureCalculator->start();
}

void SoftArm::stop() {
    if (simulate)
        return;
    curvatureCalculator->stop();
    forceController->disconnect();
}

void SoftArm::actuate(Vector2Nd tau) {
    Eigen::Matrix<double, NUM_ELEMENTS*CHAMBERS,1> pressures;
    for (int j = 0; j < NUM_ELEMENTS; ++j) {
//        pressures(j) = tau(j) / alpha(j);
        if (CHAMBERS == 3){
            pressures(3*j) = PRESSURE_OFFSET + tau(2*j)/alpha;
            pressures(3*j+1) = PRESSURE_OFFSET + tau(2*j+1)/alpha;
            pressures(3*j+2) = 3*PRESSURE_OFFSET - pressures(3*j) - pressures(3*j+1);
        }
        else if (CHAMBERS == 4){
            pressures(4*j) = PRESSURE_OFFSET + tau(2*j)/alpha;
            pressures(4*j+1) = PRESSURE_OFFSET + tau(2*j+1)/alpha;
            pressures(4*j+2) = 2*PRESSURE_OFFSET - pressures(4*j);
            pressures(4*j+3) = 2*PRESSURE_OFFSET - pressures(4*j+1);
        }
    }
    if (simulate) {
        std::cout << "In simulation mode; outputting pressure\n";
        std::cout << "\n" << pressures << "\n";
        return;
    }
    actuatePressure(pressures);
}


void SoftArm::actuatePressure(Eigen::Matrix<double, NUM_ELEMENTS*CHAMBERS,1> pressures) {
    for (int l = 0; l < NUM_ELEMENTS * CHAMBERS; ++l) {
        forceController->setSinglePressure(valve_map[l], PRESSURE_OFFSET+pressures(l));
    }
}
