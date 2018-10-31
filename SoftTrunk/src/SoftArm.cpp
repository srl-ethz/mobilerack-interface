//
// Created by yasu on 29/10/18.
//

#include "SoftArm.h"

SoftArm::SoftArm() {
    curvatureCalculator = new CurvatureCalculator(NUM_ELEMENTS, USE_OPTITRACK);
    curvatureCalculator->setupOptiTrack(LOCAL_ADDRESS, MOTIVE_ADDRESS);
    curvatureCalculator->start();
    forceController= new ForceController(16, 1000);
    for (int i = 0; i < NUM_ELEMENTS*4; ++i) {
        outputPressures.push_back(0.0);
    }

}


void SoftArm::stop(){
    curvatureCalculator->stop();
    forceController->disconnect();
}

void SoftArm::actuate(Vector2Nd tau_pt) {

}


void SoftArm::actuatePressure(Vector2Nd pressures) {
    for (int i = 0; i < NUM_ELEMENTS * 2; ++i) {
        forceController->setSinglePressure(valve_map[2*i], PRESSURE_OFFSET + pressures(i));
        forceController->setSinglePressure(valve_map[2*i+1], PRESSURE_OFFSET - pressures(i));
    }
}