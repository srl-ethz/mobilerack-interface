//
// Created by rkk on 29.10.18.
//

#include "AugmentedRigidArm.h"
#include "ControllerPCC.h"
#include <stdio.h>

/*
 * demo of AugmentedRigidArm class.
 * creates an augmented rigid arm model, then gives it some values (q and dq, the soft robot's configurations) so it can update its internal variables.
 */

int main(){
  AugmentedRigidArm augmentedRigidArm(false);
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


  Vector2Nd k;
  Vector2Nd d;
  for (int i = 0; i < NUM_ELEMENTS*2; ++i) {
        k(i,0) = 0.1*i+1;
        d(i,0) = 0.01*i+0.1;
    }
    std::cout << "\tq:\n" << k << "\n\tD:\n" << d << "\n";
  // ToDo: Make k and d be part of augmentedRigidArm
  ControllerPCC controllerPCC(&augmentedRigidArm);

  Vector2Nd q_meas(q);
  Vector2Nd dq_meas(dq);

  Vector2Nd q_ref;
  Vector2Nd dq_ref;
  Vector2Nd ddq_ref;
  for (int i = 0; i < NUM_ELEMENTS*2; ++i) {
    q_ref(i) = 0.1*i+0.1;
    dq_ref(i) = 0.1*i+0.1;
    ddq_ref(i) = 0.1*i+0.1;
  }
  Vector2Nd tau;
  controllerPCC.curvatureDynamicControl(q_meas, dq_meas, q_ref,dq_ref, ddq_ref, &tau);
  std::cout << "\ttau:\n" << tau << "\n";
  return 1;
}