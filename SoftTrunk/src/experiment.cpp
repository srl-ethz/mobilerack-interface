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

        int a=4;
        q(0) = 0;
        q(1) = 0.25 + 0.25 * sin(a*seconds);
        dq(1) = 0.25 * a* cos(a*seconds);
        ddq(1) = -0.25 * a*a* sin(a*seconds);

        stm.curvatureControl(q, dq, ddq);
        std::this_thread::sleep_for(std::chrono::milliseconds(int(CONTROL_PERIOD*1000)));
    }
}