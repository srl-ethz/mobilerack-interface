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
    CurvatureCalculator curvatureCalculator = CurvatureCalculator(3, USE_OPTITRACK);
    curvatureCalculator.setupOptiTrack(LOCAL_ADDRESS, MOTIVE_ADDRESS);
    curvatureCalculator.start(); // start the thread that continuously calculates the configuration and its time derivative.

    for (int i = 0; i < 500; ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        std::cout << "q (phi0, theta0, phi1, theta1, ...)\n" << curvatureCalculator.q <<"\n";
        std::cout<< "dq\n" << curvatureCalculator.dq <<"\n";

    }
    curvatureCalculator.stop();
    return 1;
}