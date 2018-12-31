//
// Created by yasu on 22/10/18.
//

#include "CurvatureCalculator.h"
#include <chrono>
//#include <thread>
//#include <iostream>

/**
 * @file example_CurvatureCalculator.cpp
 * @brief An example demonstrating the use of the CurvatureCalculator.
 * @details This demo prints out the current q continuously.
 */

int main(){
    CurvatureCalculator curvatureCalculator = CurvatureCalculator(USE_OPTITRACK);
    curvatureCalculator.setupOptiTrack(LOCAL_ADDRESS, MOTIVE_ADDRESS);
    curvatureCalculator.start(); // start the thread that continuously calculates the configuration and its time derivative.

    for (int i = 0; i < 500; ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        std::cout << "q (deltaLa[0], deltaLb[0], deltaLa[1], deltaLb[1], ...)\n" << curvatureCalculator.q <<"\n";
        std::cout<< "dq\n" << curvatureCalculator.dq <<"\n";

    }
    curvatureCalculator.stop();
    return 1;
}