//
// Created by yasu on 26/10/18.
//

#include "AugmentedRigidArm.h"
#include <stdio.h>
#include <chrono>
#include <thread>

/**
 * @file example_AugmentedRigidArm.cpp
 * @brief demo of AugmentedRigidArm class.
 *
 * creates an augmented rigid arm model, then gives it some values (q and dq, the soft robot's configurations) so it can update its internal variables, then prints them out.
 * for ROS_enabled systems, also animates the arm and published /joint_states ROS topics that can be viewed with RViz.
 */

Vector2Nd q_update(double seconds) {
    Vector2Nd q = Vector2Nd::Zero();
    for (int i = 0; i < NUM_ELEMENTS; i++) {
        q(2 * i + 0) = 0.03 * sin(seconds * (1.5 + (double) i));
        q(2 * i + 1) = 0.03 * cos(seconds * (2.0 + (double) i));
    }
    return q;
}

int main() {
    AugmentedRigidArm augmentedRigidArm{};

    // calculate the state of arm at a particular value of q and print out the various parameters
    Vector2Nd q = Vector2Nd::Zero();
    Vector2Nd dq = Vector2Nd::Zero();
    q(0) = 0.02;
    dq(0) = 0.01;
    std::cout << "\tq:\n" << q << "\n\tdq:\n" << dq << "\n";
    augmentedRigidArm.update(q, dq);
    std::cout << "\txi:\n" << augmentedRigidArm.xi << "\n";
    std::cout << "\tJxi:\n" << augmentedRigidArm.Jxi << "\n";
    std::cout << "\tdJxi:\n" << augmentedRigidArm.dJxi << "\n";
    std::cout << "\tB:\n" << augmentedRigidArm.B_xi << "\n";
    std::cout << "\tG:\n" << augmentedRigidArm.G_xi << "\n";

    if (!USE_ROS)
        return 1;

    // animate the arm step by step and publish to /joint_states ros topic
    double step = 0.1;
    for (double t = 0; t < 10; t += step) {
        augmentedRigidArm.update(q_update(t),
                                 dq); // don't care about updating dq, since this is just to check if xi values are appropriate
        std::this_thread::sleep_for(std::chrono::milliseconds((int) (step * 1000)));
    }
    return 1;
}
