//
// Created by yasu on 29/10/18.
//

#include "SoftArm.h"

SoftArm::SoftArm(bool simulate) : simulate(simulate) {
    std::cout << "Setting up SoftArm...\n";
    // set up the impedance parameters (k&d), and actuation coefficient(alpha).
    k(0) = 1000;
    k(1) = k(0);
    k(2) = 1300;
    k(3) = k(2);
    alpha(0) = 0.0158;
    alpha(1) = alpha(0);
    alpha(2) = 0.0175;
    alpha(3) = alpha(2);
    for (int l = 0; l < 2*NUM_ELEMENTS; ++l)
        d(l) = 9;

    // set up the matrix that maps from f to p
    if (CHAMBERS==3) {
        A_f2p << 2.0 / 3.0, 0, -1.0 / 3.0, 1.0 / sqrt(3), -1.0 / 3.0, -1.0 / sqrt(3);
        A_p2f << 1., -0.5, -0.5, 0., sqrt(3)/2., -sqrt(3)/2.;
    }
    else if (CHAMBERS==4) {
        A_f2p << 0.5, 0., 0., 0.5, -0.5, 0., 0., -0.5;
        A_p2f << 1., 0., -1., 0., 0., 1., 0., -1.;
    }
    A_p2f_all.setZero();
    for (int j = 0; j < NUM_ELEMENTS; ++j) {
        A_p2f_all.block(2*j, CHAMBERS*j, 2, CHAMBERS) = A_p2f;
    }
    std::cout<<"A_p2f_all=\n"<<A_p2f_all<<"\n";

    if (simulate)
        return;

    // set up the forceController and curvatureCalculator, which does the messaging with the physical arm
    forceController = new ForceController(16, MAX_PRESSURE);
    actuate(Vector2Nd::Zero());

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

void SoftArm::actuate(Vector2Nd f, bool setAlphaToOne) {
    Eigen::Matrix<double, NUM_ELEMENTS * CHAMBERS, 1> mappedPressure;
    double tmp_min0;
    double tmp_min1;
    // procedure to convert f to p, as described in report
    if (CHAMBERS == 3) {
        for (int j = 0; j < NUM_ELEMENTS; ++j) {
            mappedPressure.block(3 * j, 0, 3, 1) = (A_f2p * f.block(2 * j, 0, 2, 1))/alpha(2*j); //todo: fix sloppy alpha implementation
            if (setAlphaToOne)
                mappedPressure.block(3 * j, 0, 3, 1) *= alpha(2*j);
            for (int l = 0; l < 3; ++l) {
                mappedPressure(3*j+l) += PRESSURE_OFFSET; // add pressure offset to each
            }
            tmp_min0 = std::min(0.0, std::min(mappedPressure(3*j+0), std::min(mappedPressure(3*j+1), mappedPressure(3*j+2))));
            for (int l = 0; l < 3; ++l) {
                mappedPressure(3*j+l) -= tmp_min0;
            }
        }
    } else if (CHAMBERS == 4) {
        for (int j = 0; j < NUM_ELEMENTS; ++j) {
            mappedPressure.block(4 * j, 0, 4, 1) = (A_f2p * f.block(2 * j, 0, 2, 1))/alpha(2*j);
            if (setAlphaToOne)
                mappedPressure.block(4 * j, 0, 4, 1) *= alpha(2*j);
            for (int l = 0; l < 4; ++l) {
                mappedPressure(4*j+l) += PRESSURE_OFFSET; // add pressure offset to each
            }
            tmp_min0 = std::min(0.0, std::min(mappedPressure(4*j+0), mappedPressure(4*j+2)));
            tmp_min1 = std::min(0.0, std::min(mappedPressure(4*j+1), mappedPressure(4*j+3)));
            for (int l = 0; l < 2; ++l) {
                mappedPressure(4*j+l) -= tmp_min0;
                mappedPressure(4*j+1+l) -= tmp_min1;
            }
        }
    }
    actuatePressure(mappedPressure);
}


void SoftArm::actuatePressure(Eigen::Matrix<double, NUM_ELEMENTS*CHAMBERS, 1> pressures){
    p = pressures;
    if (simulate) {
        std::cout << "In simulation mode; outputting pressure\n";
        std::cout << "\n" << pressures << "\n";
        return;
    }
    for (int l = 0; l < NUM_ELEMENTS * CHAMBERS; ++l) {
        forceController->setSinglePressure(valve_map[l], pressures(l));
    }
}
