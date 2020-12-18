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

void q_update(double seconds, VectorXd& q) {
    for (int i = 0; i < st_params::num_segments; i++) {
        if (st_params::parametrization == ParametrizationType::phi_theta){
            q(2*i + 0) = seconds + 1.1 * i;
            q(2*i + 1) = 0.35 + 0.3 * sin(seconds);
        }
        else if (st_params::parametrization == ParametrizationType::longitudinal){
            q(2 * i + 0) = 0.03 * sin(seconds * (1.5 + (double) i));
            q(2 * i + 1) = 0.03 * cos(seconds * (2.0 + (double) i));
        }
    }
}

int main() {
    AugmentedRigidArm augmentedRigidArm{};

    // calculate the state of arm at a particular value of q and print out the various parameters
    VectorXd q = VectorXd::Zero(2 * st_params::num_segments);
    VectorXd dq = VectorXd::Zero(2 * st_params::num_segments);

    std::cout << "\tq:\n" << q << "\n\tdq:\n" << dq << "\n";
    augmentedRigidArm.update(q, dq);
    std::cout << "\tm:\n" << augmentedRigidArm.m << "\n";
    std::cout << "\tJm:\n" << augmentedRigidArm.Jm << "\n";
    std::cout << "\tdJm:\n" << augmentedRigidArm.dJm << "\n";
    std::cout << "\tJxi:\n" << augmentedRigidArm.Jxi << "\n";
    std::cout << "\tB:\n" << augmentedRigidArm.B_xi << "\n";
    std::cout << "\tG:\n" << augmentedRigidArm.G_xi << "\n";

    double delta_t = 0.03;
    for (double t = 0; t<10; t+=delta_t) {
        q_update(t, q);
        augmentedRigidArm.update(q, dq);
        fmt::print("q:{}\nm:{}\n", q.transpose(), augmentedRigidArm.m.transpose());
        sleep(delta_t);
    }

    fmt::print("switching to simulation mode...\n");
    augmentedRigidArm.simulate();
    return 1;
}
