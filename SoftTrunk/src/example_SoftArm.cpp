//
// Created by yasu on 05/11/18.
//

#include "SoftArm.h"
#include "SoftTrunk_common_defs.h"
#include <string>

/**
 * @file example_SoftArm.cpp
 * @brief example demonstrating the SoftArm class. actuates each chamber of the soft arm.
 */
int main() {
    SoftArm softArm = SoftArm{};
    Eigen::Matrix<double, N_SEGMENTS*N_CHAMBERS,1> pressures=Eigen::Matrix<double, N_SEGMENTS*N_CHAMBERS,1>::Zero();

    for (int i = 0; i < N_SEGMENTS * N_CHAMBERS; ++i) {
        pressures(i) = 200;
        std::cout << "actuating element " << i / N_CHAMBERS << " in the +" << i % N_CHAMBERS << " direction...\n";
        softArm.actuatePressure(pressures);
        std::this_thread::sleep_for(std::chrono::milliseconds(int(2000)));
        pressures(i) = 0;
    }

    softArm.stop();
}
