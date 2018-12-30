//
// Created by yasu on 26/10/18.
//

#include "AugmentedRigidArm.h"
#include <stdio.h>
#include <chrono>
#include <thread>
/*
 * demo of AugmentedRigidArm class.
 * creates an augmented rigid arm model, then gives it some values (q and dq, the soft robot's configurations) so it can update its internal variables.
 */

int main(){
    AugmentedRigidArm augmentedRigidArm{};
    Vector2Nd q = Vector2Nd::Zero();
    Vector2Nd dq = Vector2Nd::Zero();
    q(1) = 0.02;
    dq(1) = 0.01;

    std::cout << "\tq:\n" << q << "\n\tdq:\n" << dq << "\n";
    augmentedRigidArm.update(q, dq);
    std::cout << "\txi:\n" << augmentedRigidArm.xi << "\n";
    std::cout << "\tJxi:\n" << augmentedRigidArm.Jxi << "\n";
    std::cout << "\tdJxi:\n" << augmentedRigidArm.dJxi << "\n";
    std::cout << "\tB:\n" << augmentedRigidArm.B_xi << "\n";
    std::cout << "\tG:\n" << augmentedRigidArm.G_xi << "\n";
    for(int i=0; i<100; i++){
      augmentedRigidArm.joint_publish();
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    return 1;
}
