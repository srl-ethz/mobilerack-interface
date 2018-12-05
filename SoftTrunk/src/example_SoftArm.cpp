//
// Created by yasu on 05/11/18.
//

#include "SoftArm.h"
#include "SoftTrunk_common_defs.h"
#include <string>

int main(){
    SoftArm softArm = SoftArm{};
    Eigen::Matrix<double, NUM_ELEMENTS*CHAMBERS,1> pressures;
    std::vector<std::string> names = {"X", "Y"};

    for (int i = 0; i < NUM_ELEMENTS*CHAMBERS; ++i) {
        pressures(i) = PRESSURE_OFFSET;
        std::cout << "actuating element "<< i/CHAMBERS <<" in the +"<< i%CHAMBERS << " direction...\n";
        softArm.actuatePressure(pressures);
        std::this_thread::sleep_for(std::chrono::milliseconds(int(2000)));
        pressures(i) = 0;
    }




    softArm.stop();
}
