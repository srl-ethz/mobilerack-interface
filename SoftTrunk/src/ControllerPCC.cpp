//
// Created by yasu and rkk on 26/10/18.
//

#include "ControllerPCC.h"

ControllerPCC::ControllerPCC(AugmentedRigidArm* augmentedRigidArm) : ara(augmentedRigidArm){
    k=Vector2Nd::Zero();
    d=Vector2Nd::Zero();
}

void ControllerPCC::curvatureDynamicControl(const Vector2Nd &q_meas,
                                            const Vector2Nd &dq_meas,
                                            const Vector2Nd &q_ref,
                                            const Vector2Nd &dq_ref,
                                            const Vector2Nd &ddq_ref,
                                            Vector2Nd *tau) {
    ara->update(q_meas, dq_meas);
    B = ara->Jm.transpose() * ara->B_xi * ara->Jm;
    C = ara->Jm.transpose() * ara->B_xi * ara->dJm;
    G = ara->Jm.transpose() * ara->G_xi;
    *tau = k.asDiagonal()*q_ref + d.asDiagonal()*dq_ref + G + C*dq_ref + B*ddq_ref;
}