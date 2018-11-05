//
// Created by yasu on 31/10/18.
//

#include "SoftTrunkManager.h"
#include <Eigen/Dense>
#define STEPS 5000
#define STEP_TIME 0.005

#include "SoftTrunk_common_defs.h"

int main(){
    SoftTrunkManager stm{};
    Vector2Nd q = Vector2Nd::Zero();
    Vector2Nd dq = Vector2Nd::Zero();
    Vector2Nd ddq = Vector2Nd::Zero();
    double seconds;

    for (int j = 0; j < STEPS; ++j) {
        seconds += STEP_TIME;
        if (j % 200 == 0) std::cout<<"200 steps\n";
        //q(0) = seconds;
        //q(1) = 0.5*sin(seconds);
        q(4) = seconds/2;
        q(5) = 0.5;//*sin(seconds);
        stm.curvatureControl(q, dq, ddq);
        std::this_thread::sleep_for(std::chrono::milliseconds(int(STEP_TIME*1000)));
    }

    stm.stop();
}