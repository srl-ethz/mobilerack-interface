//
// Created by yasu on 26/10/18.
//

#include "AugmentedRigidArm.h"
#include <stdio.h>

/*
 * demo of AugmentedRigidArm class.
 * creates an augmented rigid arm model, then gives it some values (q and dq, the soft robot's configurations) so it can update its internal variables.
 */

int main(){
    AugmentedRigidArm augmentedRigidArm{};
    Eigen::Matrix<double,NUM_ELEMENTS*2,1> q;
    Eigen::Matrix<double,NUM_ELEMENTS*2,1> dq;
    for (int i = 0; i < NUM_ELEMENTS*2; ++i) {
        q(i,0) = 0.1*i+0.1;
        dq(i,0) = 0.1*i+0.1;
    }
    std::cout << "\tq:\n" << q << "\n\tdq:\n" << dq << "\n";
    augmentedRigidArm.update(q, dq);
    std::cout << "\txi:\n" << augmentedRigidArm.xi << "\n";
    std::cout << "\tJm:\n" << augmentedRigidArm.Jm << "\n";
    std::cout << "\tdJm:\n" << augmentedRigidArm.dJm << "\n";
    std::cout << "\tB:\n" << augmentedRigidArm.B_xi << "\n";
    std::cout << "\tG:\n" << augmentedRigidArm.G_xi << "\n";
    return 1;
}