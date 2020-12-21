//
// Created by yasu on 22/10/18.
//

#include "CurvatureCalculator.h"

/**
 * @file example_CurvatureCalculator.cpp
 * @brief An example demonstrating the use of the CurvatureCalculator.
 * @details This demo prints out the current q continuously.
 */

int main() {
    CurvatureCalculator cc{};

    while (true) {
        fmt::print("==========\nq:\t{}\ndq:\t{}\nddq:\t{}\n", cc.q, cc.dq, cc.ddq);
        sleep(0.1);
    }
    return 1;
}