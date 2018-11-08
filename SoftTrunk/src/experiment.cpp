//
// Created by yasu on 31/10/18.
//

#include "SoftTrunkManager.h"
#include <Eigen/Dense>
#define STEPS 10000
#define STEP_TIME 0.002

#include "SoftTrunk_common_defs.h"

int main(){
    SoftTrunkManager stm{};
    Vector2Nd q = Vector2Nd::Zero();
    Vector2Nd dq = Vector2Nd::Zero();
    Vector2Nd ddq = Vector2Nd::Zero();
    std::cout << q <<"\n";
    double seconds;

    for (int j = 0; j < STEPS; ++j) {
        seconds += STEP_TIME;

        q(0) = PI + PI*0.9 * sin(seconds);
        q(1) = 0.5;

        stm.curvatureControl(q, dq, ddq);
        std::this_thread::sleep_for(std::chrono::milliseconds(int(STEP_TIME*1000)));
    }

    stm.stop();
}