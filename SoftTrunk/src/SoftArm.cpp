//
// Created by yasu on 29/10/18.
//

#include "SoftArm.h"

SoftArm::SoftArm() {
    forceController= new ForceController(16, 800);
    for (int i = 0; i < NUM_ELEMENTS*4; ++i) {
        outputPressures.push_back(0.0);
    }

    k = Vector2Nd::Zero();
    d= Vector2Nd::Zero();
    alpha = Vector2Nd::Zero();
    for (int j = 0; j < 2 * NUM_ELEMENTS; ++j) {
        alpha(j) = 0.5;
    }
}
void SoftArm::start(){
    curvatureCalculator = new CurvatureCalculator(NUM_ELEMENTS, USE_OPTITRACK);
    curvatureCalculator->setupOptiTrack(LOCAL_ADDRESS, MOTIVE_ADDRESS);
    curvatureCalculator->start();
}


void SoftArm::stop(){
    curvatureCalculator->stop();
    forceController->disconnect();
}

void SoftArm::actuate(Vector2Nd tau_pt) {
    Eigen::Matrix2d mat;
    Vector2Nd tau_xy;
    Vector2Nd pressures;
    double phi;
    double theta;
    for (int i = 0; i < NUM_ELEMENTS; ++i) {
        //todo: why is this the way it is?
        phi = tau_pt(2*i);
        theta = tau_pt(2*i+1);
        mat << -cos(phi)*sin(theta), -sin(phi),
        -sin(phi)*sin(theta), cos(phi);
        tau_xy.block(i*2,0,2,1) = mat.inverse() * tau_pt.block(i*2,0,2,1);
    }

    for (int j = 0; j < NUM_ELEMENTS*2; ++j) {
        pressures(j) = tau_xy(j)/alpha(j);
    }
    actuatePressure(pressures);
}


void SoftArm::actuatePressure(Vector2Nd pressures) {
    for (int i = 0; i < NUM_ELEMENTS * 2; ++i) {
        forceController->setSinglePressure(valve_map[2*i], PRESSURE_OFFSET + pressures(i));
        forceController->setSinglePressure(valve_map[2*i+1], PRESSURE_OFFSET - pressures(i));
    }
}