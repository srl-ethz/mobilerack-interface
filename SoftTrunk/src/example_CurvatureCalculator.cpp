//
// Created by yasu on 22/10/18.
//

#include "CurvatureCalculator.h"
#include <chrono>
#include <thread>
#include <iostream>

/*
 * An example demonstrating the use of the CurvatureCalculator.
 * Must first set up Motive to track each frame accordingly.
 */

int main(){
    int numOfBodies = 3;
    CurvatureCalculator curvatureCalculator = CurvatureCalculator(3, USE_OPTITRACK);
    curvatureCalculator.setupOptiTrack("192.168.1.1", "192.168.1.0");
    for (int i = 0; i < 10; ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        curvatureCalculator.calculateCurvature();
        for (int j = 0; j < numOfBodies; ++j) {
            std::cout << "body #" << j << "\ttheta:" << curvatureCalculator.theta[j] << "\tphi:"
                      << curvatureCalculator.phi[j] << "\n\n";
        }
    }
    curvatureCalculator.stop();
    return 1;
}