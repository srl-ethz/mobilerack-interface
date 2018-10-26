//
// Created by yasu on 26/10/18.
//

#include "ControllerPCC.h"

ControllerPCC::ControllerPCC(Eigen::Matrix<double, NUM_ELEMENTS * 2, NUM_ELEMENTS * 2> K,
                             Eigen::Matrix<double, NUM_ELEMENTS * 2, NUM_ELEMENTS * 2> D, AugmentedRigidArm* augmentedRigidArm) :K(K), D(D), ara(augmentedRigidArm){

}

void ControllerPCC::curvatureDynamicControl(Eigen::Matrix<double, NUM_ELEMENTS * 2, 1> q_meas,
                                            Eigen::Matrix<double, NUM_ELEMENTS * 2, 1> dq_meas,
                                            Eigen::Matrix<double, NUM_ELEMENTS * 2, 1> q_ref,
                                            Eigen::Matrix<double, NUM_ELEMENTS * 2, 1> dq_ref,
                                            Eigen::Matrix<double, NUM_ELEMENTS * 2, 1> ddq_ref,
                                            Eigen::Matrix<double, NUM_ELEMENTS * 2, 1> &tau) {
    ara->update(q_meas, dq_meas);
    B = ara->Jm.transpose() * ara->B_xi * ara->Jm;
    C = ara->Jm.transpose() * ara->B_xi * ara->dJm;
    G = ara->Jm.transpose() * ara->G_xi;
    tau = K*q_ref + D*dq_ref + G + C*dq_ref + B*ddq_ref;
}