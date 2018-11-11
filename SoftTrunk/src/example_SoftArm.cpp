//
// Created by yasu on 05/11/18.
//

#include "SoftArm.h"
#include "SoftTrunk_common_defs.h"
#include <string>

int main(){
    SoftArm softArm = SoftArm{};
    Vector2Nd tau_xy = Vector2Nd::Zero();
    std::vector<std::string> names = {"X", "Y"};

    for (int i = 0; i < NUM_ELEMENTS*2; ++i) {
        tau_xy(i) = 200;
        std::cout << "actuating element "<< i/2 <<" in the +"<< names[i%2] << " direction...\n";
        softArm.actuatePressure(tau_xy);
        std::this_thread::sleep_for(std::chrono::milliseconds(int(2000)));
        tau_xy(i) = 0;
    }




    softArm.stop();
}
