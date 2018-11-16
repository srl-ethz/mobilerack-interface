//
// Created by yasu on 29/10/18.
//

#include "SoftArm.h"

SoftArm::SoftArm(bool simulate) : simulate(simulate) {
    for (int i = 0; i < NUM_ELEMENTS * 4; ++i) {
        outputPressures.push_back(0.0);
    }

    k = Vector2Nd::Zero();
    d = Vector2Nd::Zero();
    alpha = Vector2Nd::Zero();
    for (int j = 0; j < 2; ++j) {
        alpha(j) = 0.00013;
        alpha(j+2) = 0.0000862;
        alpha(j+4) = 0.000045;
    }
    k(1) = -0.74;
    k(3) = -0.26;
    k(5) = -0.034;
    for (int l = 0; l < NUM_ELEMENTS; ++l) {
        d(l*2+1) = 0.00268;
    }

    std::cout << "Starting SoftArm...\n";

    if (simulate)
        return;

    // set up the forceController and curvatureCalculator, which does the messaging with the physical arm
    forceController = new ForceController(16, MAX_PRESSURE);
    curvatureCalculator = new CurvatureCalculator(NUM_ELEMENTS, USE_OPTITRACK);
    curvatureCalculator->setupOptiTrack(LOCAL_ADDRESS, MOTIVE_ADDRESS);
    curvatureCalculator->start();
}

void SoftArm::stop() {
    if (simulate)
        return;
    curvatureCalculator->stop();
    forceController->disconnect();
}

void SoftArm::actuate(Vector2Nd tau_pt, Vector2Nd ref_q) {
    Eigen::Matrix2d mat;
    Vector2Nd tau_xy;
    Vector2Nd pressures;
    double phi;
    double theta;
    for (int i = 0; i < NUM_ELEMENTS; ++i) {
        //todo: why is this matrix operation the way it is?
        if (simulate or USE_FEEDFORWARD_CONTROL)
            theta = ref_q(2 * i + 1);
        else
            theta = curvatureCalculator->q(2 * i + 1);
        d(2*i) = d(2*i+1)*theta*theta;
        if (theta < PI / 36 or simulate or USE_FEEDFORWARD_CONTROL) {
            // sensor reading for phi is unstable when theta is small. In those cases, use the reference value for phi.
            phi = ref_q(2 * i);
        } else {
            phi = curvatureCalculator->q(2 * i);
        }

        if (theta < PI / 36 or USE_FEEDFORWARD_CONTROL) {
            mat << 0, -sin(phi), 0, cos(phi);
        } else {
            mat << -cos(phi) * sin(theta), -sin(phi), -sin(phi) * sin(theta), cos(phi);
        }
        tau_xy.block(i * 2, 0, 2, 1) = mat * tau_pt.block(i * 2, 0, 2, 1);
    }

    for (int j = 0; j < NUM_ELEMENTS * 2; ++j) {
        pressures(j) = tau_xy(j) / alpha(j);
    }
    if (simulate) {
        std::cout << "In simulation mode, outputting pressure\n";
        std::cout << "\ttau_xy\n" << pressures << "\n";
        return;
    }
    actuatePressure(pressures);
}


void SoftArm::actuatePressure(Vector2Nd pressures) {
    for (int i = 0; i < NUM_ELEMENTS * 2; ++i) {
        forceController->setSinglePressure(valve_map[2 * i], PRESSURE_OFFSET + pressures(i));
        forceController->setSinglePressure(valve_map[2 * i + 1], PRESSURE_OFFSET - pressures(i));
    }
}
