//
// Created by yasu on 31/10/18.
//

#include "SoftTrunkManager.h"
#include <Eigen/Dense>
#define STEPS 1000
#define STEP_TIME 0.01

#include "SoftTrunk_common_defs.h"

int main(){
    SoftTrunkManager stm{};
    Eigen::Matrix<double, NUM_ELEMENTS*2, STEPS> q = Eigen::Matrix<double, NUM_ELEMENTS*2, STEPS>::Zero();
    Eigen::Matrix<double, NUM_ELEMENTS*2, STEPS> dq = Eigen::Matrix<double, NUM_ELEMENTS*2, STEPS>::Zero();
    Eigen::Matrix<double, NUM_ELEMENTS*2, STEPS> ddq = Eigen::Matrix<double, NUM_ELEMENTS*2, STEPS>::Zero();
    double seconds;
    for (int i = 0; i < STEPS; ++i) {
        seconds = i*STEP_TIME;
        //q(0, i) = seconds;
        //dq(0,i) = 1;
        //ddq(0,i) = 0;
        q(1, i) = 0.1*sin(seconds);
        dq(1,i) = 0.1*cos(seconds);
        dq(1,i) = -0.1*cos(seconds);
    }
    for (int j = 0; j < STEPS; ++j) {
        stm.curvatureControl(q.col(j), dq.col(j), ddq.col(j));
        std::this_thread::sleep_for(std::chrono::milliseconds(int(STEP_TIME*1000)));
    }

    stm.stop();
}