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
    Vector2Nd pressures;
    std::vector<std::string> names = {"X", "Y"};

    for (int i = 0; i < NUM_ELEMENTS * 2; ++i) {
        pressures(i) = PRESSURE_OFFSET;
        std::cout << "actuating element " << i / 2 << " in the +" << i % 2 << " direction...\n";
        softArm.actuatePressure(pressures);
        std::this_thread::sleep_for(std::chrono::milliseconds(int(2000)));
        pressures(i) = 0;
    }

    softArm.stop();
}
