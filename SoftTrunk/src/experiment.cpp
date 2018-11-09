//
// Created by yasu on 31/10/18.
//

#include "SoftTrunkManager.h"
#include <Eigen/Dense>
#define STEPS 10000

#include "SoftTrunk_common_defs.h"

int main(){
    SoftTrunkManager stm{true};
    Vector2Nd q = Vector2Nd::Zero();
    Vector2Nd dq = Vector2Nd::Zero();
    Vector2Nd ddq = Vector2Nd::Zero();
    std::cout << q <<"\n";
    double seconds;

    for (int j = 0; j < STEPS; ++j) {
        seconds += CONTROL_PERIOD;

        q(0) = PI + 2*sin(seconds/2);
        q(1) = 0.5 * fmin(1, seconds/3);

        stm.curvatureControl(q, dq, ddq);
        std::this_thread::sleep_for(std::chrono::milliseconds(int(CONTROL_PERIOD*1000)));
    }
}