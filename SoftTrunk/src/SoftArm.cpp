//
// Created by yasu on 29/10/18.
//

#include "SoftArm.h"

SoftArm::SoftArm(bool simulate) : simulate(simulate) {
    std::cout << "Setting up SoftArm...\n";
    // set up the impedance parameters (k&d), and actuation coefficient(alpha).
    k(0) = -44;
    k(1) = k(0);
    k(2) = -10;
    k(3) = k(2);
    alpha(0) = 0.0001;
    alpha(1) = alpha(0);
    alpha(2) = 0.0001;
    alpha(3) = alpha(2);
    d = Vector2Nd::Zero();

    if (CHAMBERS==3) {
        Eigen::Matrix<double, 3, 3> A;
        A << 1, 1, 1, 0, sqrt(3) / 2, -sqrt(3) / 2, 1, -0.5, -0.5;
        force_map_matrix << 0, 0, 0, 1, 1, 0;
        force_map_matrix = A.inverse() * force_map_matrix;
        std::cout << "force_map_matrix is\n" << force_map_matrix << "\n";
    }

    if (simulate)
        return;


    // set up the forceController and curvatureCalculator, which does the messaging with the physical arm
    forceController = new ForceController(16, MAX_PRESSURE);
    actuatePressure(Vector2Nd::Zero());

    curvatureCalculator = new CurvatureCalculator(USE_OPTITRACK);
    curvatureCalculator->setupOptiTrack(LOCAL_ADDRESS, MOTIVE_ADDRESS);
    curvatureCalculator->start();
    std::cout << "Setup of SoftArm done.\n";
}

void SoftArm::stop() {
    if (simulate)
        return;
    curvatureCalculator->stop();
    forceController->disconnect();
}

void SoftArm::actuate(Vector2Nd tau) {
    Vector2Nd pressures;
    for (int j = 0; j < NUM_ELEMENTS * 2; ++j) {
        pressures(j) = tau(j) / alpha(j);
    }
    actuatePressure(pressures);
}


void SoftArm::actuatePressure(Vector2Nd pressures) {
    Eigen::Matrix<double, NUM_ELEMENTS * CHAMBERS, 1> mappedPressure;
    if (CHAMBERS == 3) {
        for (int j = 0; j < NUM_ELEMENTS; ++j) {
            mappedPressure.block(3 * j, 0, 3, 1) = (force_map_matrix * pressures.block(2 * j, 0, 2, 1));
        }
    } else if (CHAMBERS == 4) {
        for (int j = 0; j < NUM_ELEMENTS; ++j) {
            mappedPressure(4 * j + 0) = pressures(2 * j + 0);
            mappedPressure(4 * j + 1) = pressures(2 * j + 1);
            mappedPressure(4 * j + 2) = mappedPressure(4 * j + 0);
            mappedPressure(4 * j + 3) = mappedPressure(4 * j + 1);
        }
    }
    for (int m = 0; m < NUM_ELEMENTS * CHAMBERS; ++m) {
        mappedPressure(m) += PRESSURE_OFFSET;
    }
    if (simulate) {
        std::cout << "In simulation mode; outputting pressure\n";
        std::cout << "\n" << mappedPressure << "\n";
        return;
    }
    for (int l = 0; l < NUM_ELEMENTS * CHAMBERS; ++l) {
        forceController->setSinglePressure(valve_map[l], mappedPressure(l));
    }
}
