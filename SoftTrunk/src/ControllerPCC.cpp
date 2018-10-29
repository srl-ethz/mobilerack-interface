//
// Created by yasu and rkk on 26/10/18.
//

#include "ControllerPCC.h"

ControllerPCC::ControllerPCC(Vector2Nd k,
                             Vector2Nd d,
                             AugmentedRigidArm* augmentedRigidArm) : k(k), d(d), ara(augmentedRigidArm){

}

void ControllerPCC::curvatureDynamicControl(const Vector2Nd &q_meas,
                                            const Vector2Nd &dq_meas,
                                            const Vector2Nd &q_ref,
                                            const Vector2Nd &dq_ref,
                                            const Vector2Nd &ddq_ref,
                                            Vector2Nd *tau) {
    //ToDo: check if tau is properly allocated
    ara->update(q_meas, dq_meas);
    B = ara->Jm.transpose() * ara->B_xi * ara->Jm;
    C = ara->Jm.transpose() * ara->B_xi * ara->dJm;
    G = ara->Jm.transpose() * ara->G_xi;
    *tau = k.asDiagonal()*q_ref + d.asDiagonal()*dq_ref + G + C*dq_ref + B*ddq_ref;
}