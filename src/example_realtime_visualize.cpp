//
// Created by yasu on 21/12/2020.
//

#include "CurvatureCalculator.h"
#include "AugmentedRigidArm.h"

int main() {
    // obtain data from motion tracking system and update the rigid model visualization in real time.
    // run drake-visualizer as well
    CurvatureCalculator cc{};
    AugmentedRigidArm ara{};
    Rate r{30};
    while (true) {
        ara.update(cc.q, cc.dq);
        fmt::print("----------\n{}\n", cc.q.transpose());
        r.sleep();
    }
    return 1;
}